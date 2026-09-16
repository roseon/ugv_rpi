"""detector.py — the robot's object detector: one owner of which model runs.

The model name used to live in three places (``cv_ctrl`` loaded ``yolov8n.pt``,
``eyes_gaze`` defaulted to the same string again, and ``deploy.sh`` carried a
third copy to decide what to ship), so the three could drift and the eyes could
end up following a different detector from the one the app draws.  Now they all
ask this module, and ``deploy.sh`` asks it what to put on the robot.

Why this model, measured on the robot (Pi 5, 640x480 frame, four threads,
2026-09-16, ``yolov8n`` being what shipped before):

    model        ultralytics/torch   opencv dnn (this module)   found on the frame
    yolov8n           1140 ms                370 ms             person 0.53
    yolov8s           2095 ms               1039 ms             person 0.50, person 0.29
    yolov8m           3878 ms              ~2000 ms             person 0.63
    yolov8l           8737 ms              ~4400 ms             person 0.55
    yolov8x          10253 ms              ~5100 ms             person 0.78, bottle 0.34

Torch costs the same whether it has one thread or four on this CPU; OpenCV's DNN
runs the same weights about three times faster, and OpenCV is a dependency the
app already has.  That is what makes the accuracy step affordable: ``yolov8s``
here is *faster* (1039 ms) than the ``yolov8n`` it replaces was in torch
(1140 ms), so the eyes see a better model at no cost to how often they see.
Anything above ``yolov8s`` pushes the camera pass past two seconds, and the gaze
is bounded by that rate — the eyes would move once every few seconds, which is
the complaint this whole feature exists to answer.  ``yolov8n`` remains the
honest fallback if responsiveness ever matters more than recall: it leaves 370 ms
against the gaze's declared 2 Hz budget (500 ms/frame), which nothing in torch
could do.

The model file is a deployed artifact, not a committed one - this repo keeps its
weights on the robot (see ``.gitignore``) - so a robot that has none is given one
by ``deploy.sh``, which runs exactly this from the weights:

    YOLO_AUTOINSTALL=false ./ugv-env/bin/python -c \
      "from ultralytics import YOLO; YOLO('yolov8s.pt').export(format='onnx', imgsz=640, dynamic=False, opset=12)"

The ``YOLO_AUTOINSTALL=false`` is not optional: ultralytics' exporter installs its
own ``onnxruntime``/``onnxslim``, and the numpy wheel it pulls is 2.x, which this
OpenCV build cannot import - measured, it left the robot unable to ``import cv2``
in any new process until the stray ``numpy/`` was removed from the venv.  Skipping
the simplifier changes the file's bytes, not its arithmetic: a model built this way
returned output bit-identical to the shipped file (max absolute difference
0.000e+00) and the same boxes and confidences, so regenerating converges on the
same detector rather than the same bytes.

One ``Detector`` per thread that calls it: ``cv2.dnn`` forwards on a shared net
are not safe to overlap, and the app has two callers (the frame loop and Lance's
open-vocabulary/voice path), so ``detect()`` takes a lock rather than leaving
that race to whoever reads the code next.
"""

import os
import threading

import cv2
import numpy as np

# The one place a model file is named.  `deploy.sh` reads these, so a change here
# lands on the robot without a second edit anywhere - and so that a robot with no
# model file can be given one built from the weights below, rather than left with
# no detector.  Neither file is committed: this repo keeps its weights on the
# robot (they are in .gitignore), and MODEL_FILE is derived from SOURCE_WEIGHTS by
# the export in this module's header.
MODEL_FILE = "yolov8s.onnx"          # what runs, and what deploy.sh ships
SOURCE_WEIGHTS = "yolov8s.pt"        # what deploy.sh builds MODEL_FILE from

IMGSZ = 640                          # what the ONNX was exported at
NMS_IOU = 0.45                       # ultralytics' own default
MAX_DET = 300                        # ultralytics' own default
PAD_VALUE = 114                      # ultralytics' letterbox fill

# COCO, in the order the trained heads emit: index == class id.  A tuple, not a
# loaded model's `names`, so this module needs no weights to know its vocabulary.
NAMES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
)


def resolve(name, root=None):
    """Find a model file in cwd, or next to this file (the repo root)."""
    if not name:
        return None
    if os.path.isabs(name) or os.path.exists(name):
        return name
    here = root or os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, name)
    return candidate if os.path.exists(candidate) else name


def letterbox(frame, imgsz=IMGSZ):
    """The square, RGB, aspect-preserving input ultralytics trained against.

    Returns ``(canvas, scale, pad_x, pad_y)`` — the geometry ``_decode`` needs to
    put boxes back on the original frame.  Resizing straight to a square instead
    (what a plain ``blobFromImage`` does) would stretch a 640x480 frame by a third
    and bend every box with it.
    """
    height, width = frame.shape[:2]
    scale = min(imgsz / float(width), imgsz / float(height))
    new_w = max(1, int(round(width * scale)))
    new_h = max(1, int(round(height * scale)))
    canvas = np.full((imgsz, imgsz, 3), PAD_VALUE, dtype=np.uint8)
    pad_x = (imgsz - new_w) // 2
    pad_y = (imgsz - new_h) // 2
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(
        frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB), scale, pad_x, pad_y


class Detector:
    """YOLOv8 through OpenCV's DNN, with the decode ultralytics would have done.

    ``names`` is a dict exactly like a loaded ultralytics model's, because
    ``object_speech`` builds the robot's spoken vocabulary out of it.
    """

    def __init__(self, model_file=None, root=None, imgsz=IMGSZ, conf=0.30,
                 names=None):
        self.model_file = model_file or MODEL_FILE
        self.model_path = resolve(self.model_file, root)
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.names = dict(enumerate(names)) if names else dict(enumerate(NAMES))
        self._net = None
        self._lock = threading.Lock()

    # ── loading ──────────────────────────────────────────────────────────────
    def load(self):
        """Open the model once.  Raises with the path in the message on failure,
        so a missing or unexported model is a logged line, not silent blindness —
        the callers both already turn an exception here into something visible."""
        if self._net is None:
            try:
                self._net = cv2.dnn.readNetFromONNX(self.model_path)
            except Exception as exc:                                # noqa: BLE001
                raise RuntimeError(f"cannot open {self.model_path}: {exc}") from exc
        return self._net

    # ── inference ────────────────────────────────────────────────────────────
    def detect(self, frame, conf=None):
        """Hits as ``[{'name', 'conf', 'box': (x1, y1, x2, y2)}]`` in frame pixels.

        A model that cannot be opened yields no hits rather than raising: every
        caller already has a path for "the camera found nothing", and an
        exception here would take the frame loop down with it.  Opening the model
        is the caller's job (``load()``), so a missing file is loud at boot.
        """
        if frame is None or getattr(frame, "size", 0) == 0:
            return []
        try:
            net = self.load()
        except Exception:                                           # noqa: BLE001
            return []
        canvas, scale, pad_x, pad_y = letterbox(frame, self.imgsz)
        blob = canvas.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
        with self._lock:
            net.setInput(blob)
            out = net.forward()
        height, width = frame.shape[:2]
        return self._decode(out, width, height, scale, pad_x, pad_y,
                            self.conf if conf is None else float(conf))

    def _decode(self, out, width, height, scale, pad_x, pad_y, conf):
        """Raw ONNX output to frame-pixel hits.

        The graph emits ``(1, 4 + classes, anchors)``: no objectness, no NMS, and
        the centres are in letterboxed pixels.  Undoing the letterbox here is what
        keeps the boxes on the picture the panel and the eyes are drawn against.
        """
        a = np.squeeze(np.asarray(out), axis=0)
        if a.ndim != 2:
            return []
        a = a.T                                     # (anchors, 4 + classes)
        if a.shape[1] < 5:
            return []
        scores = a[:, 4:]
        classes = scores.argmax(axis=1)
        best = scores[np.arange(scores.shape[0]), classes]
        keep = best >= conf
        if not keep.any():
            return []
        xywh, classes, best = a[keep, :4], classes[keep], best[keep]
        boxes = np.stack([
            (xywh[:, 0] - pad_x) / scale - xywh[:, 2] / (2 * scale),
            (xywh[:, 1] - pad_y) / scale - xywh[:, 3] / (2 * scale),
            xywh[:, 2] / scale,
            xywh[:, 3] / scale,
        ], axis=1)

        # Per class, as ultralytics does: a person and the chair behind them
        # overlap on purpose and must not suppress each other.
        hits = []
        for cls in np.unique(classes):
            group = np.flatnonzero(classes == cls)
            idx = cv2.dnn.NMSBoxes(boxes[group].tolist(), best[group].tolist(),
                                   float(conf), NMS_IOU)
            for i in np.array(idx).flatten() if len(idx) else ():
                x, y, w, h = boxes[group][i]
                x1 = int(np.clip(round(x), 0, max(0, width - 1)))
                y1 = int(np.clip(round(y), 0, max(0, height - 1)))
                x2 = int(np.clip(round(x + w), 0, width))
                y2 = int(np.clip(round(y + h), 0, height))
                hits.append({
                    "name": self.names.get(int(cls), f"class_{int(cls)}"),
                    "conf": float(best[group][i]),
                    "box": (x1, y1, x2, y2),
                })
        hits.sort(key=lambda h: h["conf"], reverse=True)
        return hits[:MAX_DET]
