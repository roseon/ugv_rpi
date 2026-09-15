#!/usr/bin/env python3
"""Headless tests for eyes_gaze.py — no camera, no LIDAR, no Uno.

Drives EyeGazer.step() over synthetic scans and camera boxes with a fake serial
link, and asserts the bytes that would reach the Uno.  The point is the policy
and the mapping, not the hardware.

Run: python eyes_gaze_selftest.py
"""

import math
import types

from eyes_gaze import (AIM_EASE_S, DEFAULT_SCREEN_FOV_DEG, EyeGazer, bearing_to_px,
                       choose_gaze, elevation_to_py, gaze_command, head_box,
                       nearest_obstacle, parse_enable)
from perception import box_elevation_deg, v_fov_deg

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


# ── fakes ──────────────────────────────────────────────────────────────────────

class FakeRL:
    """Stands in for base.rl, which carries the raw LIDAR streams."""

    def __init__(self, raw_angles_rad=(), distances_mm=()):
        self.lidar_angles_show = list(raw_angles_rad)
        self.lidar_distances_show = list(distances_mm)


class FakeBase:
    def __init__(self, raw_angles_rad=(), distances_mm=()):
        self.rl = FakeRL(raw_angles_rad, distances_mm)


class FakeCvf:
    def __init__(self, frame=None):
        self._latest_raw_frame = frame


class _Pixels:
    """What one sampled frame location looks like to the freshness check."""

    def __init__(self, v):
        self.v = v

    def tobytes(self):
        return bytes([self.v & 0xFF])


class FakeFrame:
    """Just enough frame: a shape, and one constant sample.  The injected detector
    ignores pixels, so only the gaze's own freshness check reads them."""

    def __init__(self, w=640, h=480):
        self.shape = (h, w, 3)

    def __getitem__(self, _key):
        return _Pixels(0)


def _aim_px(link):
    """The px the last written command aimed at (the bytes the Uno receives)."""
    return int(link.writes[-1].split()[1])


class FakeLink:
    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


def scan_for(bearing_deg, dist_mm):
    """One LIDAR return at a robot-frame bearing.  base_ctrl bakes in +180."""
    return (math.radians(bearing_deg + 180.0),), (dist_mm,)


def make_gazer(link, scan=None, *, frame=None, detector=None, cam_hz=0.0, **kw):
    angles, dists = scan if scan is not None else ((), ())
    return EyeGazer(FakeBase(angles, dists), FakeCvf(frame), link=link,
                    detector=detector, cam_hz=cam_hz, clock=lambda: 0.0, **kw)


# ── the screen mapping ─────────────────────────────────────────────────────────
check("bearing 0 deg -> screen centre", bearing_to_px(0.0) == 50)
check("bearing +30 (LEFT) -> LEFT of centre", bearing_to_px(30.0) == 25)
check("bearing -30 (RIGHT) -> RIGHT of centre", bearing_to_px(-30.0) == 75)
check("bearing +60 -> left edge", bearing_to_px(60.0) == 0)
check("bearing -60 -> right edge", bearing_to_px(-60.0) == 100)
# The same rule on the vertical axis.  Both axes are angles in the robot frame
# and both are mapped with the panel's one cone, so equal angles move the pupils
# equally far.  The two axes used to disagree: x went through an angle while y
# used the raw pixel fraction, which came out 2.6x as sensitive per degree and
# was measured on the robot as |dx| 7.4 units against |dy| 0.2.
check("elevation 0 deg -> screen centre", elevation_to_py(0.0) == 50)
check("elevation +30 (BELOW the axis) -> below centre", elevation_to_py(30.0) == 75)
check("elevation -30 (ABOVE the axis) -> above centre", elevation_to_py(-30.0) == 25)
check("elevation +60 -> bottom edge", elevation_to_py(60.0) == 100)
check("elevation -60 -> top edge", elevation_to_py(-60.0) == 0)
check("elevation beyond the cone clamps",
      elevation_to_py(200.0) == 100 and elevation_to_py(-200.0) == 0,
      f"got {elevation_to_py(200.0)}, {elevation_to_py(-200.0)}")
for _ang in (5.0, 15.0, 30.0, 45.0, 60.0):
    check(f"one cone: {_ang:.0f} deg moves x and y the same distance from centre",
          abs(bearing_to_px(_ang) - 50) == abs(elevation_to_py(-_ang) - 50),
          f"x {abs(bearing_to_px(_ang) - 50)} vs y {abs(elevation_to_py(-_ang) - 50)}")

# The vertical FOV is the pinhole's, not a copy of the horizontal one: a camera
# that sees 60 deg across a 640x480 frame sees 46.8 deg down it.
_v = v_fov_deg(640.0, 480.0, 60.0)
check("the vertical FOV comes from the aspect, not from the horizontal number",
      abs(_v - 46.826) < 0.01, f"got {_v}")
check("a box on the axis has no elevation", box_elevation_deg((0.0, 240.0, 64.0, 240.0),
                                                              480.0, 640.0, 60.0) == 0.0)
check("a box below the axis has positive elevation",
      box_elevation_deg((0.0, 400.0, 64.0, 400.0), 480.0, 640.0, 60.0) > 0)
check("the frame edge is half the vertical FOV, not 30 deg",
      abs(box_elevation_deg((0.0, 480.0, 64.0, 480.0), 480.0, 640.0, 60.0) - _v / 2.0) < 1e-9)
check("bearing +180 clamps to the edge", bearing_to_px(180.0) == 0)
check("bearing -180 clamps to the edge", bearing_to_px(-180.0) == 100)
check("full screen span is the screen FOV", DEFAULT_SCREEN_FOV_DEG == 120.0)

# ── the policy ────────────────────────────────────────────────────────────────
px, py, why = choose_gaze((300.0, 0.0), None, 1200.0, 450.0)
check("obstacle inside close_mm -> gaze ahead", (px, py, why) == (50, 50, "lidar_close"))

px, py, why = choose_gaze((300.0, 45.0), None, 1200.0, 450.0)
check("close obstacle to the LEFT -> left", px == 12 and why == "lidar_close",
      f"got px={px} reason={why}")

px, py, why = choose_gaze((900.0, 0.0), None, 1200.0, 450.0)
check("near-but-not-close obstacle -> lidar_near", (px, py, why) == (50, 50, "lidar_near"))

px, py, why = choose_gaze((2000.0, 0.0), None, 1200.0, 450.0)
check("obstacle beyond prox_mm is ignored", (px, py, why) == (None, None, "idle"))

cam = {"bearing_deg": 0.0, "name": "person"}          # no elevation field = level
px, py, why = choose_gaze(None, cam, 1200.0, 450.0)
check("camera-only -> camera reason", (px, py, why) == (50, 50, "camera"))
check("a camera hit with no elevation is treated as level, not as missing",
      choose_gaze(None, cam, 1200.0, 450.0)[1] == 50)

# 30 deg to the left and 24 deg up: -24 deg is what maps to py 30 under the same
# cone the bearing uses (50 - 24/120*100).
cam_left = {"bearing_deg": 30.0, "elevation_deg": -24.0, "name": "person"}
px, py, why = choose_gaze(None, cam_left, 1200.0, 450.0)
check("camera box on the LEFT -> left, with its own y",
      px == 25 and py == 30 and why == "camera", f"got px={px} py={py}")

px, py, why = choose_gaze((900.0, 0.0), cam_left, 1200.0, 450.0)
check("a named object outranks a merely-near wall", why == "camera")

px, py, why = choose_gaze((300.0, 0.0), cam_left, 1200.0, 450.0)
check("something genuinely too close overrides the camera",
      why == "lidar_close", f"got {why}")

# The state the live robot was in: nearest LIDAR return 343 mm at 177.8 deg —
# behind it — while the camera had a person.  It pinned both pupils at px 0.
BEHIND_CLOSE = (343.0, 177.8)
px, py, why = choose_gaze(BEHIND_CLOSE, cam_left, 1200.0, 450.0)
check("a close obstacle BEHIND does not outrank the camera",
      (px, py, why) == (25, 30, "camera"), f"got px={px} py={py} why={why}")
for mm, bearing in ((343.0, 177.8), (900.0, 177.8), (300.0, -179.0)):
    px, py, why = choose_gaze((mm, bearing), None, 1200.0, 450.0)
    check(f"an obstacle {mm:.0f} mm at {bearing} deg shows nothing, not a clamped edge",
          (px, py, why) == (None, None, "idle"), f"got px={px} why={why}")
px, py, why = choose_gaze((300.0, 60.0), cam_left, 1200.0, 450.0)
check("a close obstacle exactly at the screen edge still shows",
      (px, py, why) == (0, 50, "lidar_close"), f"got px={px} why={why}")

# ── nearest_obstacle ──────────────────────────────────────────────────────────
empty = types.SimpleNamespace(angles=[], distances=[])
check("empty scan -> no obstacle", nearest_obstacle(empty, 1200.0) is None)

scan = types.SimpleNamespace(
    angles=[math.radians(0), math.radians(10), math.radians(20)],
    distances=[3000.0, 800.0, 500.0])
near = nearest_obstacle(scan, 1200.0)
check("nearest in-range reading wins", near == (500.0, 20.0), f"got {near}")
check("out-of-range readings are excluded", nearest_obstacle(scan, 600.0) == (500.0, 20.0))

noisy = types.SimpleNamespace(angles=[math.radians(0)], distances=[10.0])
check("sub-min_mm noise is not an obstacle", nearest_obstacle(noisy, 1200.0) is None)

missing = types.SimpleNamespace(angles=[math.radians(0)], distances=[None])
check("a None distance is skipped, not raised on", nearest_obstacle(missing, 1200.0) is None)

# ── the wire format ───────────────────────────────────────────────────────────
check("no target -> centre command", gaze_command(None, None) == b"T -1 -1\n")
check("no target with a stray py -> still centre", gaze_command(None, 50) == b"T -1 -1\n")
check("target -> T px py", gaze_command(33, 50) == b"T 33 50\n")

# ── the full step, over synthetic sensors ─────────────────────────────────────
link = FakeLink()
g = make_gazer(link, scan_for(0.0, 300.0))
g.step(now=1.0)
check("step() writes the close-obstacle gaze", link.writes[-1] == b"T 50 50\n",
      f"got {link.writes}")

link = FakeLink()
g = make_gazer(link, scan_for(60.0, 250.0))
g.step(now=1.0)
check("close obstacle on the LEFT drives the left edge", link.writes[-1] == b"T 0 50\n",
      f"got {link.writes}")

link = FakeLink()
g = make_gazer(link, scan_for(60.0, 3000.0))
g.step(now=1.0)
check("nothing in range -> pupils centre", link.writes[-1] == b"T -1 -1\n")

# camera path, end to end through the real box->bearing->py math
def left_box_detector(_frame):
    return [{"name": "chair", "conf": 0.9, "box": (40, 100, 160, 340)}]

# box centre x = 100/640 -> bearing +20.6 deg -> px 33
# box centre y = 220/480 -> elevation -1.95 deg -> py 48  (the pixel fraction
# would round to 46, which is what this used to assert)
link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=left_box_detector, cam_hz=2.0)
g.step(now=1.0)
check("camera box drives the gaze through the real math",
      link.writes[-1] == b"T 33 48\n", f"got {link.writes}")

check("camera hit is reported in status", g.status(1.0)["camera"] == "chair")
check("camera reason reported", g.status(1.0)["reason"] == "camera")


def two_box_detector(_frame):
    return [{"name": "chair", "conf": 0.9, "box": (40, 100, 160, 340)},
            {"name": "person", "conf": 0.9, "box": (300, 100, 620, 460)}]


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=two_box_detector, cam_hz=2.0)
g.step(now=1.0)
check("the largest detection wins", g.status(1.0)["camera"] == "person",
      f"got {g.status(1.0)['camera']}")


# The real detector path - no injected detector - had never been run: it parses
# ultralytics' result objects.  A stand-in with the same shape pins that parsing
# without needing the weights (yolov8n.pt is not in the repo).
class _Tensor1D:
    def __init__(self, values):
        self.values = values

    def __getitem__(self, i):
        return self.values[i]

    def tolist(self):
        return list(self.values)


class _Box:
    def __init__(self, xyxy, cls, conf):
        self.xyxy = [_Tensor1D(xyxy)]
        self.cls = _Tensor1D([cls])
        self.conf = _Tensor1D([conf])


class _Result:
    def __init__(self, boxes):
        self.boxes = boxes


class _FakeYolo:
    """Stands in for an ultralytics YOLO: model(frame) -> [result]."""

    names = {0: "person", 56: "chair"}

    def __init__(self, boxes):
        self._boxes = boxes

    def __call__(self, *_a, **_kw):
        return [_Result(self._boxes)]


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), cam_hz=2.0)
g._model = _FakeYolo([_Box((40, 60, 300, 460), 56, 0.77),      # big chair, left
                      _Box((400, 90, 560, 470), 0, 0.91)])     # person, right
st = g.step(now=1.0)
check("the real detector path parses ultralytics boxes", st["camera"] == "person",
      f"got {st['camera']}")
check("the parsed confidence reaches status without error", st["camera_persons"] == 1,
      f"got {st['camera_persons']}")

# The bug this answers: the largest box in a room is often furniture, so the eyes
# looked at a chair while the person stood beside it.
def big_chair_beside_person(_frame):
    return [{"name": "chair", "conf": 0.9, "box": (0, 0, 300, 470)},        # larger
            {"name": "person", "conf": 0.9, "box": (400, 100, 560, 460)}]   # to the right


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=big_chair_beside_person, cam_hz=2.0)
st = g.step(now=1.0)
check("a person is followed even when a bigger object shares the frame",
      st["camera"] == "person", f"got {st['camera']}")
check("the gaze follows the person's side of the frame, not the chair's",
      (st["target"] or {}).get("px", 50) > 55, f"got {st['target']}")
check("the person count is reported", st["camera_persons"] == 1,
      f"got {st['camera_persons']}")


def two_people(_frame):
    return [{"name": "person", "conf": 0.9, "box": (400, 100, 560, 460)},
            {"name": "person", "conf": 0.9, "box": (40, 100, 300, 460)}]


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=two_people, cam_hz=2.0)
st = g.step(now=1.0)
check("with two people the nearer/larger one is followed",
      (st["target"] or {}).get("px", 50) < 45, f"got {st['target']}")
check("both people are counted", st["camera_persons"] == 2,
      f"got {st['camera_persons']}")


# The seam this answers: with two people in frame the stronger box is regularly
# not the one being followed, so a viewer that infers the confidence from the
# label alone reports a number that contradicts the gaze it is describing.
def two_people_uneven(_frame):
    return [{"name": "person", "conf": 0.44, "box": (40, 60, 440, 420)},     # larger
            {"name": "person", "conf": 0.93, "box": (480, 120, 560, 200)}]   # stronger


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=two_people_uneven, cam_hz=2.0)
st = g.step(now=1.0)
check("the larger box is followed even though the other is more confident",
      st["camera_conf"] == 0.44, f"got {st['camera_conf']}")
check("...and that confidence belongs to the box the gaze points from",
      (st["target"] or {}).get("px") == 44, f"got {st['target']}")
check("the whole frame is still published, strongest box included",
      [d["conf"] for d in st["detections"]] == [0.44, 0.93], f"got {st['detections']}")

seen = {"hits": [{"name": "person", "conf": 0.91, "box": (160, 120, 320, 240)}]}
link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480),
               detector=lambda _f: seen["hits"], cam_hz=2.0)
check("a detection publishes its chosen confidence",
      g.step(now=1.0)["camera_conf"] == 0.91)
seen["hits"] = []                    # the person walks out of frame
st = g.step(now=1.6)
check("with nothing detected the chosen confidence is cleared",
      st["camera_conf"] is None, f"got {st['camera_conf']}")
check("...so the payload never claims a label it no longer has",
      st["camera"] is None and st["detections"] == [], f"got {st['camera']}")


def only_objects(_frame):
    return [{"name": "chair", "conf": 0.9, "box": (400, 100, 560, 460)},
            {"name": "monitor", "conf": 0.9, "box": (40, 100, 300, 460)}]


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=only_objects, cam_hz=2.0)
st = g.step(now=1.0)
check("with no person, the largest object still drives the gaze",
      st["camera"] == "monitor", f"got {st['camera']}")
check("no people counted when none are seen", st["camera_persons"] == 0,
      f"got {st['camera_persons']}")


def exploding_detector(_frame):
    raise AssertionError("detector must not run when cam_hz=0")


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(), detector=exploding_detector, cam_hz=0.0)
g.step(now=1.0)
check("cam_hz=0 keeps the camera path unused", link.writes[-1] == b"T -1 -1\n")

# a frame that is present but yields no hits must not crash or hold a stale label
link = FakeLink()
g = make_gazer(link, frame=FakeFrame(), detector=lambda _f: [], cam_hz=2.0)
g.step(now=1.0)
check("no detections -> idle, no label", g.status(1.0)["reason"] == "idle"
      and g.status(1.0)["camera"] is None)

# ── hold, and the change-only write policy ────────────────────────────────────
link = FakeLink()
g = make_gazer(link, scan_for(0.0, 300.0))
g.step(now=1.0)
check("first step sends", len(link.writes) == 1)

g.step(now=1.1)
check("an unchanged decision is not re-sent", len(link.writes) == 1,
      f"got {len(link.writes)} writes")

g.step(now=1.1 + 1.0)
check("a 1 s heartbeat re-sends even when unchanged", len(link.writes) == 2,
      f"got {len(link.writes)} writes")

# the obstacle leaves: inside hold_s the gaze must not snap back
link = FakeLink()
g = make_gazer(link, scan_for(0.0, 300.0), hold_s=0.5)
g.step(now=1.0)                                  # sees it -> lidar_close
g.base.rl.lidar_angles_show = []                 # and now it is gone
g.base.rl.lidar_distances_show = []
writes = len(link.writes)

st = g.step(now=1.2)
check("a blink does not snap the eyes back", st["reason"] == "hold", f"got {st['reason']}")
check("the held gaze writes nothing new", len(link.writes) == writes,
      f"got {link.writes}")

st = g.step(now=1.6)
check("after hold_s the pupils centre", st["reason"] == "idle", f"got {st['reason']}")
check("centring is written", link.writes[-1] == b"T -1 -1\n", f"got {link.writes}")

# ── link handling ─────────────────────────────────────────────────────────────
class DeadLink(FakeLink):
    def write(self, data):
        raise OSError("device disappeared")


link = DeadLink()
g = make_gazer(link, scan_for(0.0, 300.0))
st = g.step(now=1.0)
check("a write failure is counted, not raised", st["errors"] >= 1)
check("a failed write drops the link", st["connected"] is False)
check("the failed write is not counted as sent", st["sent"] == 0)

# ── malformed sensor input ────────────────────────────────────────────────────
bad = types.SimpleNamespace(
    angles=[math.radians(0), math.radians(5), math.radians(10)],
    distances=[float("nan"), float("inf"), 700.0])
check("NaN and inf readings are not obstacles", nearest_obstacle(bad, 1200.0) == (700.0, 10.0),
      f"got {nearest_obstacle(bad, 1200.0)}")

allbad = types.SimpleNamespace(angles=[math.radians(0)], distances=[float("nan")])
check("an all-NaN scan yields no obstacle", nearest_obstacle(allbad, 1200.0) is None)

px, py, why = choose_gaze(None, {"bearing_deg": 0.0, "name": "x"}, 1200.0, 450.0)
check("a camera hit with no elevation still yields a valid command",
      (px, py, why) == (50, 50, "camera"), f"got px={px} py={py}")

link = FakeLink()
g = make_gazer(link, scan_for(0.0, 3000.0))
st = g.status(1.0)
check("idle status reports no target", st["target"] is None and st["lidar_mm"] is None)
check("status is JSON-safe with no readings", st["reason"] == "idle")

link = FakeLink()
g = make_gazer(link, frame=FakeFrame(), detector=lambda _f: [], cam_hz=2.0)
g.step(now=1.0)
check("camera_hits resets when nothing is found", g.status(1.0)["camera_hits"] == 0)

# ── start/stop lifecycle ──────────────────────────────────────────────────────
import threading as _threading

link = FakeLink()
g = make_gazer(link, scan_for(0.0, 300.0), hz=50.0)
before = _threading.active_count()
g.start()
first = g._thread
g.start()
check("start() twice does not launch a second loop",
      g._thread is first and _threading.active_count() <= before + 1,
      f"threads {before} -> {_threading.active_count()}")

g.stop()
check("stop() joins and clears the handle", g._thread is None and g._enabled is False)
check("stop() parks the pupils in the centre", link.writes[-1] == b"T -1 -1\n",
      f"got {link.writes}")
check("stop() leaves no target to report", g.status()["target"] is None)

g.start()
check("stop() then start() runs again (not silently dead)",
      g._enabled is True and g._thread is not None and g._thread.is_alive())
g.stop()
check("a second stop() is harmless", g._thread is None)

# ── the /eyes live-tuning contract ───────────────────────────────────────────
# float('nan') does not raise, so "it parsed as a float" is not the same as "it
# is a usable distance": a NaN prox_mm makes the range filter a no-op and then
# reaches /eyes_status as JSON no strict parser accepts.
link = FakeLink()
g = make_gazer(link, scan_for(0.0, 300.0))
check("a valid retune applies both distances",
      g.retune(prox_mm="800", close_mm="300") == {"prox_mm": 800.0, "close_mm": 300.0},
      f"got prox={g.prox_mm} close={g.close_mm}")

before = (g.prox_mm, g.close_mm)
for bad in ({"prox_mm": "nan"}, {"prox_mm": "inf"}, {"prox_mm": "-5"},
            {"prox_mm": "0"}, {"close_mm": "nan"}, {"close_mm": "-1"},
            {"cam_hz": "nan"}, {"cam_hz": "inf"}):
    try:
        g.retune(**bad)
        check(f"retune rejects {bad}", False, "no exception raised")
    except ValueError:
        check(f"retune rejects {bad}", True)
check("a rejected retune changes nothing", (g.prox_mm, g.close_mm) == before,
      f"got {(g.prox_mm, g.close_mm)} expected {before}")

# The endpoint used to assign as it parsed, so a 400 still applied the first one.
before = (g.prox_mm, g.close_mm)
try:
    g.retune(prox_mm="900", close_mm="oops")
    check("a partly-invalid retune raises", False, "no exception raised")
except ValueError:
    check("a partly-invalid retune raises", True)
check("a partly-invalid retune applies none of it", (g.prox_mm, g.close_mm) == before,
      f"got {(g.prox_mm, g.close_mm)} expected {before}")
check("a negative cam_hz still clamps to LIDAR-only",
      g.retune(cam_hz="-3")["cam_hz"] == 0.0)

try:
    make_gazer(FakeLink(), prox_mm=float("nan"))
    check("the constructor rejects a NaN prox_mm", False, "no exception raised")
except ValueError:
    check("the constructor rejects a NaN prox_mm", True)

link = FakeLink()
g = make_gazer(link, scan_for(0.0, 300.0))
g.retune(prox_mm="800", close_mm="300")
check("the gaze still fires after a retune", g.step(now=5.0)["reason"] == "lidar_close",
      f"got {g.step(now=5.0)['reason']}")

# ── enable parsing ────────────────────────────────────────────────────────────
check("a missing enable means enable", parse_enable(None) is True)
for v in ("true", "TRUE", "True", "1", "yes", "on", " true "):
    check(f"enable={v!r} enables", parse_enable(v) is True)
for v in ("false", "False", "0", "no", "off"):
    check(f"enable={v!r} disables", parse_enable(v) is False)
for v in ("", "maybe", "2"):
    try:
        parse_enable(v)
        check(f"enable={v!r} is rejected rather than inverted", False, "no exception raised")
    except ValueError:
        check(f"enable={v!r} is rejected rather than inverted", True)

# ── what the camera publishes, and what happens when it cannot see ───────────
# /eyes_status is the only window into the gaze from outside the robot, so the
# three things that decide whether a person can be followed at all have to be in
# it: frames are arriving, a model loaded, and here are the boxes.

class _Clock:
    """A clock the test moves by hand."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def tick(self, dt):
        self.t += dt


def person_detector(_frame):
    return [{"name": "person", "conf": 0.91, "box": (320, 96, 480, 384)}]


clock = _Clock(1.0)
link = FakeLink()
g = EyeGazer(FakeBase((), ()), FakeCvf(FakeFrame(640, 480)), link=link,
             detector=person_detector, cam_hz=2.0, clock=clock)
st = g.step(now=1.0)
check("the frame age is reported", st["frame_age_s"] == 0.0, f"got {st['frame_age_s']}")
check("every detection is published, not only the chosen one",
      len(st["detections"]) == 1, f"got {st['detections']}")
d = st["detections"][0]
check("a published box is a fraction of the frame, so a viewer can draw it",
      d["box"] == [0.5, 0.2, 0.75, 0.8], f"got {d['box']}")
check("a published detection says whether it is a person",
      d["person"] is True and d["name"] == "person", f"got {d}")
check("the chosen box is the one the gaze points at",
      st["camera"] == "person" and (st["target"] or {}).get("px", 50) > 50,
      f"got {st['camera']} {st['target']}")

check("a box outside the frame is clamped to 0..1",
      EyeGazer._normalise([{"name": "person", "conf": 0.5,
                            "box": (-40, -40, 900, 700)}], FakeFrame(640, 480))[0]["box"]
      == [0.0, 0.0, 1.0, 1.0])

# ── the eyes look at the head, not at the middle of the body ─────────────────
# A person box runs head to feet, so aiming at its centre parks the pupils on a
# chest.  The head is estimated as the top-centre of the box: the eyes then rise
# with the face as the person moves and as they come closer.
PERSON_BOX = (200.0, 100.0, 440.0, 460.0)         # 240 x 360 in a 640x480 frame
hx1, hy1, hx2, hy2 = head_box(PERSON_BOX)
check("the estimated head is the top of the person box",
      hy1 == 100.0 and hy2 < (100.0 + 460.0) / 2.0,
      f"got {head_box(PERSON_BOX)}")
check("the head box stays inside the person box",
      hx1 >= 200.0 and hx2 <= 440.0 and hy2 <= 460.0, f"got {head_box(PERSON_BOX)}")
check("the head is narrower than the body", hx2 - hx1 < 440.0 - 200.0,
      f"got {head_box(PERSON_BOX)}")


def person_box_detector(box):
    return lambda _f: [{"name": "person", "conf": 0.9, "box": box}]


link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=person_box_detector(PERSON_BOX),
               cam_hz=2.0)
st = g.step(now=1.0)
check("a person is aimed at by the head, not by the middle of the body",
      (st["target"] or {}).get("py") == 41, f"got {st['target']}")
check("...and that is the head's angle, not its fraction of the frame",
      round(128.8 / 480.0 * 100.0) == 27, "the old pixel rule read 27 here")
check("...while the horizontal aim still follows the person",
      (st["target"] or {}).get("px") == 50, f"got {st['target']}")
check("the bytes carry the head-level gaze", link.writes[-1] == b"T 50 41\n",
      f"got {link.writes}")
check("the published person carries the head the eyes used",
      [round(v, 3) for v in st["detections"][0]["head"]] == [0.436, 0.208, 0.564, 0.328],
      f"got {st['detections'][0].get('head')}")

link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480),
               detector=person_box_detector((200.0, -100.0, 440.0, 300.0)), cam_hz=2.0)
st = g.step(now=1.0)
check("a head above the frame is aimed by its angle, not pinned to the top",
      (st["target"] or {}).get("py") == 25, f"got {st['target']}")

link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480),
               detector=person_box_detector((200.0, -2000.0, 440.0, -1600.0)), cam_hz=2.0)
st = g.step(now=1.0)
check("a head far past the cone clamps to the top of the screen",
      (st["target"] or {}).get("py") == 0, f"got {st['target']}")

link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480),
               detector=lambda _f: [{"name": "chair", "conf": 0.9,
                                     "box": (200.0, 100.0, 440.0, 460.0)}], cam_hz=2.0)
st = g.step(now=1.0)
check("a non-person is still aimed at by its whole box",
      (st["target"] or {}).get("py") == 53, f"got {st['target']}")
check("...and publishes no head", "head" not in st["detections"][0],
      f"got {st['detections'][0]}")

# A frame that stops changing is a dead capture loop.  Read as "empty camera"
# it is wrong in both directions: the eyes point at a person who has walked away,
# and nothing anywhere says the camera itself died.
clock.tick(3.0)
st = g.step(now=4.0)
check("a frozen frame is reported as stale",
      st["frame_age_s"] is not None and st["frame_age_s"] >= 3.0,
      f"got {st['frame_age_s']}")
check("...and the gaze says so itself, so a viewer never re-derives it",
      st["frame_stale"] is True, f"got {st['frame_stale']}")
check("a frozen frame does not drive a gaze", st["reason"] != "camera",
      f"got {st['reason']}")
check("a frozen frame publishes no detections", st["detections"] == [],
      f"got {st['detections']}")


class PixelFrame(FakeFrame):
    """A frame whose sampled content the test can change underneath the gazer."""

    def __init__(self, v=0):
        super().__init__()
        self.v = v

    def __getitem__(self, _key):
        return _Pixels(self.v)


cvf = FakeCvf(PixelFrame(0))
clock = _Clock(10.0)
g = EyeGazer(FakeBase((), ()), cvf, link=FakeLink(), detector=person_detector,
             cam_hz=2.0, clock=clock)
g.step(now=10.0)
clock.tick(3.0)
cvf._latest_raw_frame.v = 200        # the picture changed, the object did not
st = g.step(now=13.0)
check("a camera backend that reuses one buffer is not read as a dead camera",
      st["reason"] == "camera", f"got {st['reason']}")
check("...and its frame age resets when the picture changes",
      st["frame_age_s"] == 0.0, f"got {st['frame_age_s']}")
check("...and a picture that is moving is not called stale",
      st["frame_stale"] is False, f"got {st['frame_stale']}")

clock.tick(3.0)
st = g.step(now=16.0)                # now the pixels stop changing too
check("a picture that genuinely stops changing is read as stale",
      st["reason"] != "camera", f"got {st['reason']}")

# The failure this answers: the eyes load their own YOLO, and when that load
# fails the gaze silently follows LIDAR forever.  cv_ctrl loads the same weights
# at boot, so the eyes borrow that model instead of losing the feature.
import sys as _sys


class _BrokenUltralytics(types.ModuleType):
    """An ultralytics that cannot load weights (no internet, no file)."""

    def __getattr__(self, name):
        raise RuntimeError("weights are not on disk and there is no internet")


_saved_ultra = _sys.modules.get("ultralytics")
_sys.modules["ultralytics"] = _BrokenUltralytics("ultralytics")
try:
    cvf = FakeCvf(FakeFrame(640, 480))
    cvf.yolo_model = _FakeYolo([_Box((400, 90, 560, 470), 0, 0.91)])   # the app's own
    link = FakeLink()
    g = EyeGazer(FakeBase((), ()), cvf, link=link, cam_hz=2.0, clock=lambda: 1.0)
    st = g.step(now=1.0)
    check("a failed model load is reported instead of silently disabling the gaze",
          bool(st["model_error"]), f"got {st['model_error']!r}")
    check("a person is still followed after the eyes' own load failed",
          st["camera"] == "person" and st["target"] is not None,
          f"got camera={st['camera']} target={st['target']}")

    # Neither model available: the feature degrades, it does not break the eyes.
    link = FakeLink()
    g = EyeGazer(FakeBase(*scan_for(0.0, 300.0)), FakeCvf(FakeFrame(640, 480)),
                 link=link, cam_hz=2.0, clock=lambda: 1.0)
    st = g.step(now=1.0)
    check("with no detector the status says so and the LIDAR gaze still works",
          st["model_ready"] is False and bool(st["model_error"])
          and st["reason"] == "lidar_close" and link.writes[-1] == b"T 50 50\n",
          f"got ready={st['model_ready']} reason={st['reason']} {link.writes}")
finally:
    if _saved_ultra is not None:
        _sys.modules["ultralytics"] = _saved_ultra
    else:
        _sys.modules.pop("ultralytics", None)

# Stopping the gaze has to stop publishing what the camera last saw: a viewer
# left drawing those boxes over a live stream, with `frame 0.0 s old` beside
# them, is showing a picture of the past as if it were current.  The engine
# decides that, not the viewer - so the fields go empty and `enabled` carries
# the state.
clock = _Clock(20.0)
g = EyeGazer(FakeBase(*scan_for(0.0, 900.0)), FakeCvf(PixelFrame(0)), link=FakeLink(),
             detector=person_detector, cam_hz=2.0, clock=clock)
st = g.step(now=20.0)
check("a running gaze publishes the boxes it just detected",
      len(st["detections"]) == 1 and st["camera"] == "person", f"got {st}")
g.stop()
st = g.status(20.0)
check("stop() publishes no detections to draw", st["detections"] == [],
      f"got {st['detections']}")
check("stop() publishes nothing the camera last saw",
      st["camera"] is None and st["camera_conf"] is None
      and st["camera_hits"] == 0 and st["camera_persons"] == 0,
      f"got camera={st['camera']} conf={st['camera_conf']} hits={st['camera_hits']}")
check("stop() publishes no frame age to read as live",
      st["frame_age_s"] is None and st["frame_stale"] is False,
      f"got age={st['frame_age_s']} stale={st['frame_stale']}")
check("stop() says why those fields are empty",
      st["enabled"] is False and st["reason"] == "stopped" and st["target"] is None,
      f"got {st['enabled']} {st['reason']} {st['target']}")

# The loop repeats that teardown as it exits, which is what covers a step still
# inside a slow model load when stop() gave up waiting for it.
clock = _Clock(30.0)
g2 = EyeGazer(FakeBase((), ()), FakeCvf(PixelFrame(0)), link=FakeLink(),
              detector=person_detector, cam_hz=2.0, clock=clock)
g2.step(now=30.0)
g2.stop()
g2.step(now=31.0)                    # a restarted gaze publishes a live decision
g2._enabled = True
g2._park()                           # an older loop reaches its exit
st = g2.status(31.0)
check("an exiting loop does not clobber a restarted gaze",
      st["reason"] == "camera" and len(st["detections"]) == 1,
      f"got {st['reason']} {st['detections']}")
g2._enabled = False
g2._park()
st = g2.status(30.0)
check("the loop takes down what it published as it exits",
      st["detections"] == [] and st["reason"] == "stopped" and st["target"] is None,
      f"got {st['detections']} {st['reason']} {st['target']}")

# ── the status payload must survive a strict JSON parser ──────────────────────
import json as _json

link = FakeLink()
g = make_gazer(link, scan_for(0.0, 300.0))
g.step(now=1.0)
try:
    _json.loads(_json.dumps(g.status(1.0)),
                parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
    check("status is strict-JSON clean", True)
except ValueError as e:
    check("status is strict-JSON clean", False, f"invalid token {e!r}")

# Detections and the frame age travel through the same payload, so they must be
# strict-JSON clean too.
link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=person_detector, cam_hz=2.0)
g.step(now=1.0)
try:
    _json.loads(_json.dumps(g.status(1.0)),
                parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
    check("status with detections is strict-JSON clean", True)
except ValueError as e:
    check("status with detections is strict-JSON clean", False, f"invalid token {e!r}")

# ── the aim the Uno is actually sent ──────────────────────────────────────────
# The detector runs at 2 Hz while the loop runs at 10 Hz.  Forwarding a detection
# unchanged makes the pupils jump once per camera frame and sit still in between,
# which is the stepping the eyes were reported as doing; the aim is walked there
# in the same number of steps a person would see as motion.
link = FakeLink()
g = make_gazer(link, scan=scan_for(0.0, 300.0), cam_hz=0.0)      # close, dead ahead
st = g.step(now=1.0)
check("the first aim adopts the target instead of crawling to it",
      link.writes[-1] == b"T 50 50\n", f"got {link.writes}")
check("...and the payload still reports the policy target",
      st["target"] == {"px": 50, "py": 50} and st["reason"] == "lidar_close",
      f"got {st['target']}")

# the same obstacle swings to the left edge of the screen (px 0)
g.base.rl.lidar_angles_show[:] = [math.radians(60.0 + 180.0)]
g.step(now=1.05)                       # a third of the 0.15 s ease later
mid = link.writes[-1]
mid_px = int(mid.split()[1])
check("a moving target is walked, not jumped",
      0 < mid_px < 50, f"got {mid!r} for a 50 -> 0 move")
check("...and the payload target is already the policy target, ahead of the aim",
      g.status(1.05)["target"] == {"px": 0, "py": 50} and mid_px > 0,
      f"got {g.status(1.05)['target']} with the aim at {mid_px}")

for i in range(30):
    g.step(now=1.15 + 0.1 * i)
check("the aim lands exactly on a target that stops moving",
      link.writes[-1] == b"T 0 50\n", f"got {link.writes[-1]}")
g.step(now=60.0)
check("a settled aim keeps sending the same point",
      link.writes[-1] == b"T 0 50\n", f"got {link.writes[-1]}")

# This walk is the whole of the smoothing - the firmware draws what it is sent -
# so its time constant is what decides how far the pupils can be behind a head.
# It is bounded by the sampling rate rather than picked: a fixed one unrelated to
# cam_hz either lags a slow detector or leaves a fast one stepping per sample.
for hz, want in ((2.0, AIM_EASE_S), (30.0, 1.0 / 90.0), (0.0, 1.0 / 30.0)):
    g = make_gazer(FakeLink(), cam_hz=hz)
    check(f"the aim's time constant at cam_hz {hz} is bounded by the sampling rate",
          abs(g.aim_tau() - want) < 1e-9, f"got {g.aim_tau()} want {want}")

# A head-sized step of the target, through the real camera path: walked there
# rather than jumped to, within one detector interval, then landed exactly.
seen_box = {"box": (40, 100, 160, 340)}          # px 33, the left of the frame
link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), cam_hz=2.0,
               detector=lambda _f: [{"name": "person", "conf": 0.9,
                                     "box": seen_box["box"]}])
g.step(now=1.0)
check("a person on the left is aimed at", _aim_px(link) == 33, f"got {_aim_px(link)}")
seen_box["box"] = (480, 100, 620, 460)            # the head steps to px 68
st = g.step(now=1.5)
stepped = _aim_px(link)
check("a step is walked toward, not jumped to", 33 < stepped < 68, f"got {stepped}")
# The status names both ends of the gap, so what the eyes were sent can be read
# off the live robot instead of inferred: the aim trails the head it is chasing.
check("the head is published where the camera found it",
      st["target"] == {"px": 68, "py": 41}, f"got {st['target']}")
check("the aim published is the point the eyes were actually sent",
      st["aim"] == {"px": stepped, "py": 41}, f"got {st['aim']} vs sent {stepped}")
check("...which is still behind the head on the step itself",
      st["aim"]["px"] < st["target"]["px"], f"aim {st['aim']} target {st['target']}")
for i in range(1, 6):
    g.step(now=1.5 + 0.1 * i)
within = _aim_px(link)
check("...and is within 2 units of the new target one detector interval later",
      abs(within - 68) <= 2, f"got {within} at +0.5 s")
for i in range(6, 10):
    st = g.step(now=1.5 + 0.1 * i)
check("...and lands on it exactly", _aim_px(link) == 68, f"got {_aim_px(link)}")
check("...at which point the aim and the head agree",
      st["aim"] == st["target"], f"got {st['aim']} {st['target']}")
for i in range(10, 20):                       # keep the same target for a while
    g.step(now=1.5 + 0.1 * i)
check("a settled gaze keeps aiming at the same point",
      _aim_px(link) == 68, f"got {_aim_px(link)}")

# nothing to look at: the eyes are parked, and the next target is adopted again
g.base.rl.lidar_angles_show[:] = []
g.base.rl.lidar_distances_show[:] = []
st = g.step(now=200.0)
check("nothing in range parks the pupils", link.writes[-1] == b"T -1 -1\n",
      f"got {link.writes[-1]}")
check("and the parked gaze publishes no aim, so nothing stale can be read as current",
      st["aim"] is None and st["target"] is None, f"got {st['aim']} {st['target']}")
g.base.rl.lidar_angles_show[:] = [math.radians(60.0 + 180.0)]
g.base.rl.lidar_distances_show[:] = [300.0]
g.step(now=200.2)
check("the first aim after a gap adopts the new target",
      link.writes[-1] == b"T 0 50\n", f"got {link.writes[-1]}")


# ── summary ───────────────────────────────────────────────────────────────────
print(f"\n{CHECKS - len(FAILURES)}/{CHECKS} checks passed")
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S):")
    for f in FAILURES:
        print("  -", f)
    raise SystemExit(1)
print("ALL EYES-GAZE PROBES PASSED")
