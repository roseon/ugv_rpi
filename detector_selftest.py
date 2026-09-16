#!/usr/bin/env python3
"""Headless tests for detector.py — no model file, no camera.

What is worth pinning here is the arithmetic the ONNX graph leaves to us: the
letterbox geometry that puts boxes back on the real picture, the confidence
filter, per-class NMS, and the vocabulary (`object_speech` builds the robot's
spoken table out of `Detector.names`, so its length and order are load-bearing).

`Detector.detect` is driven with a crafted output array, so the checks hold
without the weights — which is also why they cannot drift with them.

Run: python detector_selftest.py
"""

import numpy as np

import detector

CHECKS = 0
FAILURES = []


def check(label, cond, detail=""):
    global CHECKS
    CHECKS += 1
    if not cond:
        FAILURES.append(f"{label}  {detail}")
        print(f"FAIL  {label}  {detail}")
    else:
        print(f"PASS  {label}")


def fake_out(anchors, total=84):
    """A `(1, 4 + classes, anchors)` array with every score at zero."""
    return np.zeros((1, total, len(anchors)), dtype=np.float32)


def put(out, i, cx, cy, w, h, cls, score):
    out[0, 0, i], out[0, 1, i], out[0, 2, i], out[0, 3, i] = cx, cy, w, h
    out[0, 4 + cls, i] = score
    return out


# ── the letterbox geometry ────────────────────────────────────────────────────
frame = np.full((480, 640, 3), 200, dtype=np.uint8)          # a 4:3 camera frame
canvas, scale, pad_x, pad_y = detector.letterbox(frame, 640)
check("a 4:3 frame is scaled 1:1 into 640x640",
      (scale, pad_x, pad_y) == (1.0, 0, 80), (scale, pad_x, pad_y))
check("the canvas is square and the pad is the fill value",
      canvas.shape == (640, 640, 3) and int(canvas[0, 0, 0]) == detector.PAD_VALUE,
      (canvas.shape, int(canvas[0, 0, 0])))
check("the frame lands in the middle, not stretched",
      int(canvas[80, 0, 0]) == 200 and int(canvas[559, 0, 0]) == 200
      and int(canvas[79, 0, 0]) == detector.PAD_VALUE
      and int(canvas[560, 0, 0]) == detector.PAD_VALUE,
      (int(canvas[80, 0, 0]), int(canvas[559, 0, 0]), int(canvas[560, 0, 0])))

wide = np.zeros((480, 1280, 3), dtype=np.uint8)
_, scale_w, pad_xw, pad_yw = detector.letterbox(wide, 640)
check("a 1280-wide frame is halved and letterboxed vertically",
      (scale_w, pad_xw, pad_yw) == (0.5, 0, 200), (scale_w, pad_xw, pad_yw))

# ── decode: the letterbox has to be undone, to the pixel ──────────────────────
d = detector.Detector()
out = put(fake_out(range(1)), 0, 320, 320, 100, 200, 0, 0.9)
hits = d._decode(out, 640, 480, 1.0, 0, 80, 0.30)
check("a person at the canvas centre lands at the frame centre",
      len(hits) == 1 and hits[0]["name"] == "person"
      and hits[0]["box"] == (270, 140, 370, 340), hits)

d2 = detector.Detector()
out = put(fake_out(range(1)), 0, 320, 320, 200, 100, 56, 0.8)
hits = d2._decode(out, 1280, 480, 0.5, 0, 200, 0.30)
check("a halved, letterboxed frame maps back to its own pixels",
      len(hits) == 1 and hits[0]["name"] == "chair"
      and hits[0]["box"] == (440, 140, 840, 340), hits)

# ── the filter and the suppression ────────────────────────────────────────────
out = put(fake_out(range(1)), 0, 320, 320, 50, 50, 0, 0.20)
check("a hit under the confidence cut is not reported",
      d._decode(out, 640, 480, 1.0, 0, 80, 0.30) == [])

out = fake_out(range(2))
put(out, 0, 320, 320, 100, 100, 0, 0.9)
put(out, 1, 322, 322, 100, 100, 0, 0.7)                       # same class, overlapping
hits = d._decode(out, 640, 480, 1.0, 0, 80, 0.30)
check("two boxes of one class collapse to the confident one",
      len(hits) == 1 and abs(hits[0]["conf"] - 0.9) < 1e-6, hits)

out = fake_out(range(2))
put(out, 0, 320, 320, 100, 100, 0, 0.9)
put(out, 1, 322, 322, 100, 100, 56, 0.7)                      # a person and a chair
hits = d._decode(out, 640, 480, 1.0, 0, 80, 0.30)
check("a person and the chair behind them both survive",
      sorted(h["name"] for h in hits) == ["chair", "person"], hits)
check("hits come back most confident first",
      [h["conf"] for h in hits] == sorted((h["conf"] for h in hits), reverse=True),
      hits)

out = fake_out(range(1))                                       # all scores zero
check("an empty frame reports nothing", d._decode(out, 640, 480, 1.0, 0, 80, 0.30) == [])
check("a frame with no pixels is refused rather than crashing",
      d.detect(np.zeros((0, 0, 3), dtype=np.uint8)) == [])

# ── the vocabulary object_speech builds its table from ────────────────────────
check("the vocabulary is the 80 COCO names", len(detector.NAMES) == 80,
      len(detector.NAMES))
check("names line up with the class ids the graph emits",
      detector.NAMES[0] == "person" and detector.NAMES[56] == "chair",
      (detector.NAMES[0], detector.NAMES[56]))
check("Detector.names is a dict, as ultralytics' was",
      isinstance(d.names, dict) and d.names[0] == "person", type(d.names))
check("one place names the model",
      detector.MODEL_FILE.endswith(".onnx"), detector.MODEL_FILE)

# The drift this module exists to stop: `cv_ctrl` had one copy of the detector's
# filename, `eyes_gaze` had a second as a default, and `deploy.sh` a third, so a
# change to one left the others behind.  Nothing outside this module may name a
# detector-family model again.  The open-vocabulary world model in `cv_ctrl` is a
# different model and stays where it is.
import os
import re

_detector_name = re.compile(r"['\"][\w./-]*yolov8[nsmlx]\.(?:onnx|pt)['\"]")
_root = os.path.dirname(os.path.abspath(__file__))
_offenders = []
for _name in sorted(os.listdir(_root)):
    if not _name.endswith(".py") or _name in ("detector.py", "detector_selftest.py"):
        continue
    with open(os.path.join(_root, _name), encoding="utf-8", errors="replace") as _fh:
        for _i, _line in enumerate(_fh, 1):
            if _detector_name.search(_line):
                _offenders.append(f"{_name}:{_i}")
check("no other module names a detector model", not _offenders, _offenders)

# ── summary ───────────────────────────────────────────────────────────────────
print(f"\n{CHECKS - len(FAILURES)}/{CHECKS} checks passed")
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S):")
    for f in FAILURES:
        print("  -", f)
    raise SystemExit(1)
print("ALL DETECTOR PROBES PASSED")
