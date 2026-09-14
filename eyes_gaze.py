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

Both inputs are turned into one bearing and one screen position, so the mapping
lives in a single place (``bearing_to_px``) rather than once per source.

The Uno is opened through ``serial_ports.uno_port()`` — by USB identity, never by
position — and re-opened when it goes away, because this board has dropped off
the bus before.
"""

import math
import os
import threading
import time

from perception import box_bearing_deg
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

# Nearest reading closer than this is treated as noise/self-return, not an object.
MIN_OBSTACLE_MM = 60.0

# Send the same command again this often, so a dropped line or a reset Uno
# catches up without waiting for the gaze to change.
SEND_REPEAT_S = 1.0


def bearing_to_px(bearing_deg, screen_fov_deg=DEFAULT_SCREEN_FOV_DEG):
    """Robot-frame bearing -> screen x in 0..100 (0 = left edge, 50 = ahead).

    Image x grows to the right while a bearing to the LEFT is positive in the
    robot frame, hence the minus: +30 deg must land left of centre, not right.
    """
    px = 50.0 - bearing_deg / screen_fov_deg * 100.0
    return int(round(min(100.0, max(0.0, px))))


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
    ``camera`` : {'bearing_deg': float, 'py': int, 'name': str} | None
    Returns    : (px | None, py | None, reason)
    """
    if lidar is not None and lidar[0] <= close_mm:
        return bearing_to_px(lidar[1], screen_fov_deg), 50, "lidar_close"
    if camera is not None:
        return (bearing_to_px(camera["bearing_deg"], screen_fov_deg),
                camera.get("py", 50), "camera")
    if lidar is not None and lidar[0] <= prox_mm:
        return bearing_to_px(lidar[1], screen_fov_deg), 50, "lidar_near"
    return None, None, "idle"


class EyeGazer:
    """Drives both eye panels from LIDAR proximity and camera detections."""

    def __init__(self, base, cvf, root=None, *,
                 prox_mm=DEFAULT_PROX_MM, close_mm=DEFAULT_CLOSE_MM,
                 hz=10.0, cam_hz=2.0, hold_s=0.5, conf=0.30,
                 screen_fov_deg=DEFAULT_SCREEN_FOV_DEG,
                 model="yolov8n.pt",
                 link=None, detector=None, clock=time.time):
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
        self._clock = clock
        self._model = None
        self._model_failed = False

        self._thread = None
        self._enabled = False
        self._port = None

        self._last_cam_t = 0.0
        self._cam_hit = None
        self._cam_label = None
        self._cam_raw_hits = 0
        self._cam_persons = 0

        self._hold_px = None
        self._hold_py = None
        self._hold_t = 0.0
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
        """Load YOLOv8n once, lazily — never at import or in the app's boot path."""
        if self._model is not None or self._model_failed:
            return self._model
        try:
            from ultralytics import YOLO
            self._model = YOLO(self.model_path)
        except Exception as e:                                  # noqa: BLE001
            self._model_failed = True
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

    def camera_hit(self, now):
        """Most prominent camera detection, throttled to ``cam_hz``.

        Reuses the frame the camera thread already captured; it must not open the
        camera, or it would contend with cv_ctrl for the device.
        """
        if self.cam_hz <= 0:
            self._cam_hit = None
            self._cam_label = None
            self._cam_raw_hits = 0
            self._cam_persons = 0
            return None
        if now - self._last_cam_t < 1.0 / self.cam_hz:
            return self._cam_hit
        self._last_cam_t = now

        frame = getattr(self.cvf, "_latest_raw_frame", None)
        if frame is None:
            self._cam_hit = None
            self._cam_label = None
            self._cam_raw_hits = 0
            self._cam_persons = 0
            return None
        try:
            hits = self._detect(frame)
        except Exception as e:                                   # noqa: BLE001
            self._errors += 1
            print(f"[eyes] camera detection failed: {e}")
            self._cam_hit = None
            self._cam_label = None
            self._cam_persons = 0
            return None

        self._cam_raw_hits = len(hits)
        self._cam_persons = sum(1 for h in hits
                                if str(h.get("name", "")).lower() in PERSON_LABELS)
        if not hits:
            self._cam_hit = None
            self._cam_label = None
            return None

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

        _area, hit, (x1, y1, x2, y2) = best
        cy = (y1 + y2) / 2.0
        self._cam_hit = {
            "name": hit["name"],
            "conf": hit["conf"],
            "bearing_deg": box_bearing_deg(best[2], frame_width=float(width)),
            "py": int(round(min(100.0, max(0.0, cy * 100.0 / height)))),
        }
        self._cam_label = hit["name"]
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

        cmd = gaze_command(px, py)
        if cmd != self._last_cmd or (now - self._last_send_t) >= SEND_REPEAT_S:
            if self._emit(cmd, now):
                self._last_cmd = cmd
                self._last_send_t = now
        return self.status(now)

    def _loop(self):
        while self._enabled:
            t0 = self._clock()
            try:
                self.step(t0)
            except Exception as e:                               # noqa: BLE001
                self._errors += 1
                print(f"[eyes] gaze step failed: {e}")
            time.sleep(max(0.0, 1.0 / self.hz - (self._clock() - t0)))

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
        self._hold_px = None
        self._hold_py = None
        self._reason = "stopped"
        self._target = None

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
            "lidar_mm": None if lidar is None else round(lidar[0], 1),
            "lidar_bearing_deg": None if lidar is None else round(lidar[1], 1),
            "camera": self._cam_label,
            "camera_hits": self._cam_raw_hits,
            "camera_persons": self._cam_persons,
            "prox_mm": self.prox_mm,
            "close_mm": self.close_mm,
            "hz": self.hz,
            "cam_hz": self.cam_hz,
            "sent": self._sent,
            "errors": self._errors,
            "age_s": None if not self._last_step_t else round(now - self._last_step_t, 2),
        }
