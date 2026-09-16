"""eyes_gaze.py — what the two eye displays look at.

This module owns exactly two things: the serial link to the eye Arduino, and the
policy that decides where the pupils point.  Two inputs feed one gaze.

  * **LIDAR proximity** — 360 degrees, always available, no camera needed.  The
    nearest obstacle inside ``prox_mm`` attracts the gaze; inside ``close_mm`` —
    the "something is getting too close" case — it overrides the camera
    entirely.  Bearings come from ``robot_state.LidarScan``, i.e. the corrected
    robot frame (0 = forward, + = left).  Nothing here re-derives that offset,
    because getting it wrong once already made "front" mean the robot's back.

  * **The camera**, through a throttled YOLOv8n pass over the frame cv_ctrl has
    *already* captured (``_latest_raw_frame``).  It is deliberately not a second
    ``cv2.VideoCapture``: opening the camera again fights the frame loop for the
    device.  The eyes follow the most prominent detection (largest box), which
    gives the gaze a name the LIDAR cannot supply.

Both inputs are turned into angles in the robot frame, and those are mapped onto
the panel by one rule with one cone (``bearing_to_px`` for x, ``elevation_to_py``
for y), so the mapping lives in a single place rather than once per source and
once per axis.

The Uno is opened through ``serial_ports.uno_port()`` — by USB identity, never by
position — and re-opened when it goes away, because this board has dropped off
the bus before.
"""

import math
import os
import threading
import time

from perception import box_bearing_deg, box_elevation_deg
from robot_state import LidarScan

import serial_ports

# Where an obstacle is worth looking at, and where it takes over the gaze.
DEFAULT_PROX_MM = 1200.0
DEFAULT_CLOSE_MM = 450.0

# The eyes express this much bearing across the screen: +-screen_fov/2 reaches
# the edges.  A camera bearing comes from the camera's own FOV; this is only the
# mapping onto the panel, so the two numbers are not the same thing.
DEFAULT_SCREEN_FOV_DEG = 120.0

# YOLOv8n's class 0 is 'person'.  The eyes look at a person in preference to
# anything else in the frame: following the person in view is the point of the
# displays, and the largest box in a room is regularly a chair, a monitor or a
# door rather than the human being looked at.
PERSON_LABELS = frozenset({"person", "people", "human"})

# A person's box runs from head to feet, and aiming at its centre parks the
# pupils on a chest.  Without a keypoint model the head is estimated as the
# top-centre of that box: the top sixth of its height (chin level) and a third of
# its width, centred - so the eyes look at the face as the person moves and as
# they come closer.
HEAD_HEIGHT_FRAC = 0.16
HEAD_WIDTH_FRAC = 0.34


# Nearest reading closer than this is treated as noise/self-return, not an object.
MIN_OBSTACLE_MM = 60.0

# A captured frame older than this means the capture loop stopped feeding us.
# Gazing on a frozen frame points the eyes at a person who has already walked
# away, and it is otherwise indistinguishable from "nobody in view".
FRAME_STALE_S = 2.0

# Send the same command again this often, so a dropped line or a reset Uno
# catches up without waiting for the gaze to change.
SEND_REPEAT_S = 1.0

# This class is the only smoothing in the gaze path: it walks the aim toward the
# newest detection, and the firmware draws what it is sent.  Two filters in
# series used to sit here - this walk plus the Uno's own easing - and measured on
# the robot, with both in place, the drawn pupil trailed the aim it had been sent
# by 5.9 px while tracking at a head's speed; with the Uno's easing removed the
# same test leaves 4.3 px, which is one frame of travel - the most a ~9 frame/s
# panel can do.  (A head-sized step costs the same either way: 25 aim units and
# ~2.2 s, because the frame-affordability cap, not the easing, governs it.)
#
# How long the walk may take is bounded by the sampling rate, not chosen: the
# detector reports at cam_hz, so closing the gap in a fixed time unrelated to it
# either lags a slow detector or leaves a fast one stepping once per sample.  A
# third of one interval is under the sampling floor and still smooth.
AIM_EASE_S = 0.15               # never slower than this, whatever the detector
AIM_EASE_OF_INTERVAL = 1.0 / 3.0


def head_box(box):
    """The estimated head region of a person box, in frame pixels."""
    x1, y1, x2, y2 = box
    centre = (x1 + x2) / 2.0
    half = max(0.0, x2 - x1) * HEAD_WIDTH_FRAC / 2.0
    return (centre - half, y1, centre + half, y1 + max(0.0, y2 - y1) * HEAD_HEIGHT_FRAC)


def bearing_to_px(bearing_deg, screen_fov_deg=DEFAULT_SCREEN_FOV_DEG):
    """Robot-frame bearing -> screen x in 0..100 (0 = left edge, 50 = ahead).

    Image x grows to the right while a bearing to the LEFT is positive in the
    robot frame, hence the minus: +30 deg must land left of centre, not right.
    """
    px = 50.0 - bearing_deg / screen_fov_deg * 100.0
    return int(round(min(100.0, max(0.0, px))))


def elevation_to_py(elevation_deg, screen_fov_deg=DEFAULT_SCREEN_FOV_DEG):
    """Robot-frame elevation -> screen y in 0..100 (0 = top edge, 50 = level).

    The same rule as ``bearing_to_px``, on the other axis: both are angles and
    both are mapped with the panel's one cone, so a target 30 deg up and a target
    30 deg to the left move the pupils equally far.  Image y grows downward and
    so does an elevation, hence the plus.

    This axis used to skip the angle entirely and use the raw pixel fraction
    (``cy * 100 / height``), which is a *different* rule: it worked out at 2.14
    aim units per degree against the horizontal's 0.83, so the vertical was 2.6x
    as sensitive, and a head slightly above the axis swung the pupils far higher
    up the screen than the same offset to the side moved them.  Measured on the
    robot over 50 samples: the aim matched the angle rule to 0.22 units on x and
    the pixel rule to 0.28 on y, which is how the mismatch was found.
    """
    py = 50.0 + elevation_deg / screen_fov_deg * 100.0
    return int(round(min(100.0, max(0.0, py))))


def gaze_command(px, py):
    """The one place a gaze becomes bytes on the wire."""
    if px is None or py is None:
        return b"T -1 -1\n"          # nothing worth looking at -> pupils centre
    return ("T %d %d\n" % (int(px), int(py))).encode()


TRUTHY = frozenset({"true", "1", "yes", "on"})
FALSEY = frozenset({"false", "0", "no", "off"})


def parse_enable(value):
    """The /eyes ``enable`` argument -> True/False, or ValueError.

    A missing argument means enable (the documented default).  The test used to
    be ``value.lower() == 'true'``, so ``enable=1`` — the most natural thing to
    type, and the form a slider or a checkbox sends — silently turned the eyes
    *off* at HTTP 200, and a bare ``enable=`` did the same.  Saying what you
    mean now means that, and anything unrecognised is rejected out loud instead
    of becoming the opposite of what was asked.
    """
    if value is None:
        return True
    want = str(value).strip().lower()
    if want in TRUTHY:
        return True
    if want in FALSEY:
        return False
    raise ValueError("enable must be true/false (1/0, yes/no and on/off also work)")


def nearest_obstacle(scan, max_mm, min_mm=MIN_OBSTACLE_MM):
    """Closest valid reading inside ``max_mm`` -> (distance_mm, bearing_deg).

    ``scan.angles`` are radians in the robot frame.  Returns None for an empty
    scan or one with nothing in range.
    """
    best = None
    for angle_rad, dist in zip(scan.angles, scan.distances):
        # A malformed packet can leave a None or a NaN in the stream; both would
        # sail through the range comparisons and then reach the JSON status as
        # invalid `NaN`, so drop anything that is not a finite number.
        if dist is None or not math.isfinite(dist):
            continue
        if dist < min_mm or dist > max_mm:
            continue
        if best is None or dist < best[0]:
            best = (float(dist), math.degrees(angle_rad))
    return best


def choose_gaze(lidar, camera, prox_mm, close_mm, screen_fov_deg=DEFAULT_SCREEN_FOV_DEG):
    """One gaze decision from the two inputs.

    ``lidar``  : (distance_mm, bearing_deg) | None
    ``camera`` : {'bearing_deg': float, 'elevation_deg': float, 'name': str} | None
                 Both are angles in the robot frame; turning them into screen
                 coordinates is this function's job, so there is one rule and
                 one cone for both axes and no caller can carry a second one.
    Returns    : (px | None, py | None, reason)

    Only a return the panels can actually point at takes the gaze.  A bearing
    past +-screen_fov/2 has nowhere to go but a clamped edge, which tells the
    user something is hard to the side when it is really behind them.  Live on
    the robot a wall 343 mm *behind* it won that comparison and pinned both
    pupils at the far left while the camera had a person at conf 0.88.
    """
    reachable = lidar is not None and abs(lidar[1]) <= screen_fov_deg / 2.0 + 1e-6
    if reachable and lidar[0] <= close_mm:
        return bearing_to_px(lidar[1], screen_fov_deg), 50, "lidar_close"
    if camera is not None:
        return (bearing_to_px(camera["bearing_deg"], screen_fov_deg),
                elevation_to_py(camera.get("elevation_deg", 0.0), screen_fov_deg),
                "camera")
    if reachable and lidar[0] <= prox_mm:
        return bearing_to_px(lidar[1], screen_fov_deg), 50, "lidar_near"
    return None, None, "idle"


class EyeGazer:
    """Drives both eye panels from LIDAR proximity and camera detections."""

    def __init__(self, base, cvf, root=None, *,
                 prox_mm=DEFAULT_PROX_MM, close_mm=DEFAULT_CLOSE_MM,
                 hz=10.0, cam_hz=2.0, hold_s=0.5, conf=0.30,
                 screen_fov_deg=DEFAULT_SCREEN_FOV_DEG,
                 model="yolov8n.pt",
                 link=None, detector=None, on_hits=None, clock=time.time):
        self.base = base
        self.cvf = cvf
        self.hz = max(0.5, float(hz))
        self.hold_s = max(0.0, float(hold_s))
        # One owner for these three: the same validation /eyes retunes through,
        # so a NaN cannot get in through either door.
        self.retune(prox_mm=prox_mm, close_mm=close_mm, cam_hz=cam_hz)
        self.conf = float(conf)
        self.screen_fov_deg = float(screen_fov_deg)
        self.model_path = self._resolve_model(model, root)

        self._link = link                 # injectable for tests / dry runs
        self._detector = detector         # callable(frame) -> [{'name','conf','box'}]
        # What the last camera pass saw, handed to whoever speaks for the robot.
        # The gaze reports, it does not decide what is worth saying.
        self._on_hits = on_hits
        self._clock = clock
        self._model = None
        self._model_failed = False
        self._model_error = None

        self._thread = None
        self._enabled = False
        self._port = None

        self._last_cam_t = 0.0
        self._cam_hit = None
        self._cam_label = None
        self._cam_conf = None
        self._cam_raw_hits = 0
        self._cam_persons = 0
        self._detections = []             # every hit this pass, boxes normalised
        self._frame_sig = None
        self._frame_change_t = 0.0
        self._frame_age_s = None

        self._hold_px = None
        self._hold_py = None
        self._hold_t = 0.0
        self._aim = None                  # the eased aim, walked toward each target
        self._aim_sent = None             # the last point the Uno was told to draw
        self._aim_t = 0.0
        self._last_cmd = None
        self._last_send_t = 0.0

        self._last_open_try = 0.0
        self._reopen_s = 2.0
        self._log_missing_link = False

        self._lidar = None                # (dist_mm, bearing_deg) | None
        self._reason = "idle"
        self._target = None               # (px, py) | None
        self._sent = 0
        self._errors = 0
        self._last_step_t = 0.0

    # ── live tuning ──────────────────────────────────────────────────────────
    def retune(self, prox_mm=None, close_mm=None, cam_hz=None):
        """Change the gaze distances / camera rate.  ValueError if unusable.

        Validates everything before assigning any of it, so a rejected retune
        leaves the running policy exactly as it was — the endpoint used to
        assign as it parsed, so a 400 still left half the change applied.

        The finiteness check is the one that matters: ``float('nan')`` does not
        raise, and a NaN ``prox_mm`` turns ``nearest_obstacle``'s upper bound
        into a no-op (nothing compares greater than NaN), so a wall at any
        distance is reported as the nearest obstacle, the "came within prox_mm"
        band can never fire again, and the NaN is then emitted through
        /eyes_status as JSON no strict parser accepts.  A negative distance is
        the silent kill: every reading is farther than it, so the LIDAR gaze
        simply stops reacting.
        """
        updates = {}
        for name, value in (("prox_mm", prox_mm), ("close_mm", close_mm)):
            if value is None:
                continue
            v = float(value)
            if not math.isfinite(v) or v <= 0:
                raise ValueError(f"{name} must be a positive, finite distance in mm")
            updates[name] = v
        if cam_hz is not None:
            v = float(cam_hz)
            if not math.isfinite(v):
                raise ValueError("cam_hz must be a finite number of frames per second")
            updates["cam_hz"] = max(0.0, v)          # 0 = LIDAR only
        for name, v in updates.items():
            setattr(self, name, v)
        return updates

    # ── setup helpers ────────────────────────────────────────────────────────
    @staticmethod
    def _resolve_model(name, root):
        """Find the weights in cwd, or next to this file (the repo root)."""
        if not name:
            return None
        if os.path.isabs(name) or os.path.exists(name):
            return name
        here = root or os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.join(here, name)
        return candidate if os.path.exists(candidate) else name

    def _model_instance(self):
        """Load YOLOv8n once, lazily — never at import or in the app's boot path.

        If this load fails the eyes cannot see a person at all, so it must not be
        the end of the feature: ``cv_ctrl`` loads *the same weights* at boot and
        keeps the model (``yolo_model``).  A second load can fail where that one
        succeeded — no internet for ultralytics' first-run download, a second
        instance, a version skew — and every previous failure here ended as a
        single printed line and a gaze that silently followed LIDAR forever.
        """
        if self._model is not None or self._model_failed:
            return self._model
        try:
            from ultralytics import YOLO
            self._model = YOLO(self.model_path)
        except Exception as e:                                  # noqa: BLE001
            self._model_failed = True
            self._model_error = str(e)
            shared = getattr(self.cvf, "yolo_model", None)
            if shared is not None:
                self._model = shared
                self._model_failed = False
                print(f"[eyes] own model load failed ({e}); "
                      "using the detector the app already loaded")
                return self._model
            print(f"[eyes] object model unavailable ({e}); gaze will follow LIDAR only")
        return self._model

    # ── camera input ─────────────────────────────────────────────────────────
    def _detect(self, frame):
        if self._detector is not None:
            return self._detector(frame)
        model = self._model_instance()
        if model is None:
            return []
        hits = []
        for r in model(frame, verbose=False, conf=self.conf):
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                name = getattr(model, "names", {}).get(int(box.cls[0]), "object")
                hits.append({"name": name, "conf": float(box.conf[0]),
                             "box": (x1, y1, x2, y2)})
        return hits

    @staticmethod
    def _frame_signature(frame):
        """A cheap fingerprint that changes when the picture does.

        Content, not object identity, in both directions: a camera backend that
        reuses one capture buffer would otherwise look like a dead camera, and a
        genuinely frozen frame would look fresh.  Sampling every 64th pixel each
        camera pass (a few hundred bytes at 2 Hz) catches both.
        """
        return (frame.shape, hash(frame[::64, ::64].tobytes()))

    def _forget_frame(self):
        """Forget the sampled frame, so no age is reported for one."""
        self._frame_sig = None
        self._frame_change_t = 0.0
        self._frame_age_s = None

    @property
    def _frame_stale(self):
        """This module's verdict on the frame, so a viewer never re-derives it.

        It is published as ``frame_stale`` precisely so the panel does not need
        its own copy of the threshold; a client that guesses gets it wrong in
        both directions, calling a stopped camera "nobody in view" and a
        stopped gaze "no camera frame".
        """
        return self._frame_age_s is not None and self._frame_age_s > FRAME_STALE_S

    def _sample_frame(self, now):
        """The freshest captured frame, and how long since it last changed.

        The age is the only way to tell "nobody in view" from "the capture loop
        stopped handing over frames": both look like an empty camera, and they
        need opposite fixes.
        """
        frame = getattr(self.cvf, "_latest_raw_frame", None)
        if frame is None:
            self._forget_frame()
            return None
        sig = self._frame_signature(frame)
        if sig != self._frame_sig:
            self._frame_sig = sig
            self._frame_change_t = now
        self._frame_age_s = max(0.0, now - self._frame_change_t)
        return frame

    def _clear_camera(self):
        """One owner for "the camera contributed nothing this pass"."""
        self._cam_hit = None
        self._cam_label = None
        self._cam_conf = None
        self._cam_raw_hits = 0
        self._cam_persons = 0
        self._detections = []

    @staticmethod
    def _normalise(hits, frame):
        """Every detection this pass, as fractions of the frame: the viewer scales
        the picture, and a second YOLO pass just for it would double the Pi's load.
        """
        height, width = frame.shape[:2]
        if not width or not height:
            return []

        def frac(v, span):
            return round(min(1.0, max(0.0, float(v) / span)), 4)

        out = []
        for h in hits:
            x1, y1, x2, y2 = h.get("box", (0, 0, 0, 0))
            name = str(h.get("name", "object"))
            person = name.lower() in PERSON_LABELS
            entry = {
                "name": name,
                "conf": round(float(h.get("conf", 0.0)), 3),
                "person": person,
                "box": [frac(x1, width), frac(y1, height),
                        frac(x2, width), frac(y2, height)],
            }
            if person:
                # What the eyes are actually aimed at, so a viewer can show it
                # instead of guessing from the whole-body box.
                hx1, hy1, hx2, hy2 = head_box((x1, y1, x2, y2))
                entry["head"] = [frac(hx1, width), frac(hy1, height),
                                 frac(hx2, width), frac(hy2, height)]
            out.append(entry)
        return out

    def camera_hit(self, now):
        """Most prominent camera detection, throttled to ``cam_hz``.

        Reuses the frame the camera thread already captured; it must not open the
        camera, or it would contend with cv_ctrl for the device.
        """
        frame = self._sample_frame(now)      # measured even while throttled
        if self.cam_hz <= 0:
            self._clear_camera()
            return None
        if now - self._last_cam_t < 1.0 / self.cam_hz:
            return self._cam_hit
        self._last_cam_t = now

        if frame is None:
            self._clear_camera()
            return None
        if self._frame_stale:
            self._clear_camera()
            return None
        try:
            hits = self._detect(frame)
        except Exception as e:                                   # noqa: BLE001
            self._errors += 1
            print(f"[eyes] camera detection failed: {e}")
            self._clear_camera()
            return None

        if not hits:
            # Clear the lot, confidence included.  Nulling only the hit and the
            # label here left the previous pass's confidence behind, so the
            # status published `camera: null` next to `camera_conf: 0.91`.
            self._clear_camera()
            return None

        self._cam_raw_hits = len(hits)
        self._cam_persons = sum(1 for h in hits
                                if str(h.get("name", "")).lower() in PERSON_LABELS)
        self._detections = self._normalise(hits, frame)
        if self._on_hits is not None:
            # The eyes are the robot's only look at the room at this rate, so a
            # speaking side-effect belongs here -- but a fault in it must never
            # break the gaze, which is what the user is actually watching.
            try:
                self._on_hits(hits)
            except Exception as e:                               # noqa: BLE001
                print(f"[eyes] object speech failed: {e}")

        height, width = frame.shape[:2]
        # People first.  The largest box in a room is often furniture, and the
        # eyes pointed at a chair instead of the person standing next to it is
        # precisely the report this answers.  With no person in frame, the most
        # prominent thing is still better than staring at nothing.
        pool = [h for h in hits
                if str(h.get("name", "")).lower() in PERSON_LABELS] or hits
        best = None
        for h in pool:
            x1, y1, x2, y2 = h["box"]
            area = max(0, x2 - x1) * max(0, y2 - y1)
            if best is None or area > best[0]:
                best = (area, h, (x1, y1, x2, y2))
        if best is None or width <= 0 or height <= 0:
            self._cam_hit = None
            self._cam_label = None
            self._cam_persons = 0
            return None

        _area, hit, box = best
        # A person is aimed at by the head, anything else by its own box.
        aim = head_box(box) if str(hit.get("name", "")).lower() in PERSON_LABELS else box
        self._cam_hit = {
            "name": hit["name"],
            "conf": hit["conf"],
            # Both angles in the robot frame.  This dict used to carry a
            # ready-made `py` taken from the raw pixel fraction, which is where
            # the second mapping rule lived; the screen position is derived from
            # these by choose_gaze, with the same cone the bearing uses.
            "bearing_deg": box_bearing_deg(aim, frame_width=float(width)),
            "elevation_deg": box_elevation_deg(aim, frame_height=float(height),
                                              frame_width=float(width)),
        }
        self._cam_label = hit["name"]
        # The confidence of the box the eyes actually took.  A viewer cannot
        # recover this from `detections` + the label: with two people in frame
        # the strongest box is regularly not the one being followed, and
        # reporting the other one's confidence contradicts the gaze.
        self._cam_conf = hit["conf"]
        return self._cam_hit

    # ── serial link ──────────────────────────────────────────────────────────
    def _ensure_link(self, now):
        """Open the Uno when it is present; keep trying quietly when it is not."""
        if self._link is not None:
            return True
        if now - self._last_open_try < self._reopen_s:
            return False
        self._last_open_try = now

        port = self._port or serial_ports.uno_port()
        if port is None:
            if not self._log_missing_link:
                self._log_missing_link = True
                print(f"[eyes] no eye Arduino on USB (expected {serial_ports.UNO_NODE}); "
                      "gaze will be computed but not sent")
            return False
        try:
            import serial
            self._link = serial.Serial(port, 115200, timeout=0.2)
            self._link.reset_input_buffer()
            self._port = port
            self._log_missing_link = False
            print(f"[eyes] gaze link open on {port}")
            return True
        except Exception as e:                                   # noqa: BLE001
            self._errors += 1
            if not self._log_missing_link:
                self._log_missing_link = True
                print(f"[eyes] cannot open {port}: {e}")
            self._link = None
            return False

    def _drop_link(self):
        try:
            if self._link is not None:
                self._link.close()
        except Exception:                                        # noqa: BLE001
            pass
        self._link = None
        self._port = None

    def _emit(self, cmd, now):
        if not self._ensure_link(now):
            return False
        try:
            self._link.write(cmd)
            self._sent += 1
            return True
        except Exception as e:                                   # noqa: BLE001
            self._errors += 1
            print(f"[eyes] write failed ({e}); dropping the link and retrying")
            self._drop_link()
            return False

    # ── the loop ─────────────────────────────────────────────────────────────
    def step(self, now=None):
        """One gaze decision and one write.  Returns the status dict."""
        now = self._clock() if now is None else now

        scan = LidarScan.read(self.base)
        self._lidar = None if scan.empty else nearest_obstacle(scan, self.prox_mm)
        camera = self.camera_hit(now)

        px, py, reason = choose_gaze(self._lidar, camera, self.prox_mm,
                                     self.close_mm, self.screen_fov_deg)

        if px is not None:
            self._hold_px, self._hold_py, self._hold_t = px, py, now
        elif self._hold_px is not None and now - self._hold_t < self.hold_s:
            # A detection that blinks for a frame should not snap the eyes back
            # to centre and then out again; hold the last gaze briefly.
            px, py, reason = self._hold_px, self._hold_py, "hold"
        else:
            self._hold_px = None

        self._reason = reason
        self._target = None if px is None else (px, py)
        self._last_step_t = now

        aim = self._ease_aim(now, px, py)
        cmd = gaze_command(None, None) if aim is None else gaze_command(*aim)
        if cmd != self._last_cmd or (now - self._last_send_t) >= SEND_REPEAT_S:
            if self._emit(cmd, now):
                self._last_cmd = cmd
                self._last_send_t = now
                # What the Uno has actually been told.  Published so the gap
                # between the head the camera found and the point the eyes were
                # sent is readable from the surface rather than inferred.
                self._aim_sent = aim
        return self.status(now)

    def aim_tau(self):
        """How long the aim may take to answer a new sample.

        A third of a detector interval, capped: with cam_hz 0 the LIDAR is the
        only target and it is read every loop tick, so the interval is the loop's.
        """
        interval = 1.0 / self.cam_hz if self.cam_hz > 0 else 1.0 / self.hz
        return min(AIM_EASE_S, interval * AIM_EASE_OF_INTERVAL)

    def _ease_aim(self, now, px, py):
        """Walk the aim toward the target; return the integer point to send.

        The first aim after nothing-to-look-at adopts the target outright, so the
        eyes do not crawl across the screen from wherever they were.
        """
        if px is None:
            self._aim, self._aim_t = None, now
            return None
        if self._aim is None:
            self._aim, self._aim_t = (float(px), float(py)), now
            return int(px), int(py)
        dt = max(0.0, now - self._aim_t)
        k = 1.0 - math.exp(-dt / self.aim_tau())
        ax = self._aim[0] + (px - self._aim[0]) * k
        ay = self._aim[1] + (py - self._aim[1]) * k
        # Land exactly once close, so a settled gaze stops changing what it sends.
        if abs(px - ax) < 0.6:
            ax = float(px)
        if abs(py - ay) < 0.6:
            ay = float(py)
        self._aim, self._aim_t = (ax, ay), now
        return int(round(ax)), int(round(ay))

    def _loop(self):
        while self._enabled:
            t0 = self._clock()
            try:
                self.step(t0)
            except Exception as e:                               # noqa: BLE001
                self._errors += 1
                print(f"[eyes] gaze step failed: {e}")
            time.sleep(max(0.0, 1.0 / self.hz - (self._clock() - t0)))
        self._park()

    def start(self):
        # Set the flag FIRST.  A loop that was mid-step when stop() ran is still
        # alive with _enabled already False; returning early on that liveness
        # check would leave the gaze silently off for good.
        self._enabled = True
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="eyes-gaze")
        self._thread.start()
        print(f"[eyes] gaze running at {self.hz:.0f} Hz "
              f"(camera {self.cam_hz:.0f} Hz, close <= {self.close_mm:.0f} mm)")

    def stop(self):
        self._enabled = False
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            # A step can be inside the first model load, which takes seconds.
            # Keep the handle in that case: start() must not launch a second
            # loop writing to the same serial port.
            if thread.is_alive():
                print("[eyes] gaze thread is still finishing its last step")
            else:
                self._thread = None
        # Park the pupils in the middle so a stopped robot is not left staring.
        if self._link is not None:
            try:
                self._link.write(b"T -1 -1\n")
            except Exception:                                    # noqa: BLE001
                pass
        self._drop_link()
        self._park()

    def _park(self):
        """Publish nothing: no target, no shape, no camera, no frame age.

        One owner for "the gaze is not running", called by stop() so the state
        goes at once and by the loop as it exits - a step that was inside the
        first model load when stop() stopped waiting would otherwise publish
        boxes, a confidence and a frame age that nothing is refreshing.  Parking
        is refused while the gaze is running, because a loop that is still
        exiting may find a restart already publishing a live decision.
        """
        if self._enabled:
            return
        self._hold_px = None
        self._hold_py = None
        self._aim = None
        self._aim_sent = None
        self._reason = "stopped"
        self._target = None
        self._clear_camera()
        self._forget_frame()

    def status(self, now=None):
        now = self._clock() if now is None else now
        # Snapshot once: the gaze thread can null these between two reads, which
        # would turn a status poll into a TypeError inside Flask.
        target = self._target
        lidar = self._lidar
        return {
            "enabled": self._enabled,
            "port": self._port,
            "connected": self._link is not None,
            "reason": self._reason,
            "target": None if target is None
                      else {"px": target[0], "py": target[1]},
            "aim": None if self._aim_sent is None
                   else {"px": self._aim_sent[0], "py": self._aim_sent[1]},
            "lidar_mm": None if lidar is None else round(lidar[0], 1),
            "lidar_bearing_deg": None if lidar is None else round(lidar[1], 1),
            "camera": self._cam_label,
            "camera_conf": self._cam_conf,
            "camera_hits": self._cam_raw_hits,
            "camera_persons": self._cam_persons,
            # What the camera sees, for a viewer that wants to draw it.  The
            # chosen gaze only ever names one box; this is all of them.
            "detections": list(self._detections),
            "frame_age_s": None if self._frame_age_s is None
                           else round(self._frame_age_s, 2),
            "frame_stale": self._frame_stale,
            # False with an error set means no person can ever be seen: the
            # model did not load.  That single line is the difference between
            # "it sees nobody" and "it cannot see".
            "model_ready": self._model is not None,
            "model_error": self._model_error,
            "prox_mm": self.prox_mm,
            "close_mm": self.close_mm,
            "hz": self.hz,
            "cam_hz": self.cam_hz,
            "sent": self._sent,
            "errors": self._errors,
            "age_s": None if not self._last_step_t else round(now - self._last_step_t, 2),
        }
