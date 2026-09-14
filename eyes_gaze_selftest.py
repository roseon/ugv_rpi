#!/usr/bin/env python3
"""Headless tests for eyes_gaze.py — no camera, no LIDAR, no Uno.

Drives EyeGazer.step() over synthetic scans and camera boxes with a fake serial
link, and asserts the bytes that would reach the Uno.  The point is the policy
and the mapping, not the hardware.

Run: python eyes_gaze_selftest.py
"""

import math
import types

from eyes_gaze import (DEFAULT_SCREEN_FOV_DEG, EyeGazer, bearing_to_px,
                       choose_gaze, gaze_command, nearest_obstacle,
                       parse_enable)

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


class FakeFrame:
    """Just enough frame for shape[:2] — the injected detector ignores pixels."""

    def __init__(self, w=640, h=480):
        self.shape = (h, w, 3)


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

cam = {"bearing_deg": 0.0, "py": 50, "name": "person"}
px, py, why = choose_gaze(None, cam, 1200.0, 450.0)
check("camera-only -> camera reason", (px, py, why) == (50, 50, "camera"))

cam_left = {"bearing_deg": 30.0, "py": 30, "name": "person"}
px, py, why = choose_gaze(None, cam_left, 1200.0, 450.0)
check("camera box on the LEFT -> left, with its own y",
      px == 25 and py == 30 and why == "camera", f"got px={px} py={py}")

px, py, why = choose_gaze((900.0, 0.0), cam_left, 1200.0, 450.0)
check("a named object outranks a merely-near wall", why == "camera")

px, py, why = choose_gaze((300.0, 0.0), cam_left, 1200.0, 450.0)
check("something genuinely too close overrides the camera",
      why == "lidar_close", f"got {why}")

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

# box centre x = 100/640 -> bearing +20.6 deg -> px 33 ; centre y = 220/480 -> py 46
link = FakeLink()
g = make_gazer(link, frame=FakeFrame(640, 480), detector=left_box_detector, cam_hz=2.0)
g.step(now=1.0)
check("camera box drives the gaze through the real math",
      link.writes[-1] == b"T 33 46\n", f"got {link.writes}")

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
check("a camera hit with no py still yields a valid command", (px, py, why) == (50, 50, "camera"),
      f"got px={px} py={py}")

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

# ── summary ───────────────────────────────────────────────────────────────────
print(f"\n{CHECKS - len(FAILURES)}/{CHECKS} checks passed")
if FAILURES:
    print(f"{len(FAILURES)} FAILURE(S):")
    for f in FAILURES:
        print("  -", f)
    raise SystemExit(1)
print("ALL EYES-GAZE PROBES PASSED")
