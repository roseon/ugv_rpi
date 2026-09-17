"""Regression harness for the self-drive restructure. Stdlib only.

    python selfdrive_selftest.py

It covers the parts a live run cannot pin down reproducibly:
  0. every *call site* in the refactored modules still matches the signature of
     the function it calls — the restructure moved helpers into perception.py
     and dropped their default arguments, which left app.py's SLOW branch
     calling turn_bias() with two arguments. That raised on every tick, so the
     avoider sent no wheel command in the 450-700 mm band and the exception was
     swallowed by a dead log handler. Nothing caught it until a live run. This
     check does;
  1. the refactored angle math is *numerically identical* to the code it
     replaced (old implementations inlined here verbatim from the previous
     working tree), so app.py's avoider and the planner see the same numbers;
  2. every other behavior the live robot proved, through the public API.
"""
import ast
import importlib
import inspect
import json
import math
import os
import random
import tempfile
import textwrap
import time
import types

import lance
import perception
import robot_state
import spatial_memory
import target_pursuit
from detection_source import DetectionSource
from self_drive import (HEADING_SMOOTHING, TURN_DEADBAND,  # noqa: F401
                        TURN_SLEW_PER_TICK, SelfDriver)

FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + ("  <- " + str(detail) if detail else ""))
    if not cond:
        FAILS.append(name)


class RL:
    def __init__(self, angles, dists, stamp=None, bin_age_s=0.0):
        self.lidar_angles_show = angles
        self.lidar_distances_show = dists
        self.lidar_scan_time = stamp   # moves once per revolution, like the real one
        # base_ctrl's 1-degree occupancy bins, indexed exactly as the parser
        # builds them: with the sensor mount's +180 baked in, stamped as filled.
        self.lidar_bins = [(0.0, 0.0) for _ in range(360)]
        for a, d in zip(angles, dists):
            if d > 0:
                self.lidar_bins[int(round(math.degrees(a))) % 360] = (
                    float(d), time.time() - bin_age_s)


class Base:
    def __init__(self, rl, base_data=None):
        self.rl = rl
        self.base_data = base_data or {}


class CV:
    def __init__(self, dets=None, world=None):
        self.last_detections = dets or []
        self._world = world or []
        self.world_calls = 0

    def detect_world(self):
        self.world_calls += 1
        return list(self._world)


def raw(deg_robot):
    """Robot-frame degrees (+ = left) -> the raw value the wire carries."""
    return math.radians(deg_robot) + math.pi


def scan(front_mm=3000, sectors=None):
    sectors = sectors or {}
    angles, dists = [], []
    for d in range(-180, 180):
        angles.append(raw(d))
        dists.append(sectors.get(d, front_mm))
    return angles, dists


def ticking(drv):
    """Arm a driver for direct _tick() calls, without starting its thread.

    Production has exactly one ticker per planner (its own loop).  The harness
    is the ticker here, so it must not also let the driver's thread run: two
    tickers on one planner interleave ticks, and then which heading won last is
    a matter of timing and every heading assertion goes flaky.
    """
    drv._active = True
    return drv


def planner(scan_pair=None, dets=None, world=None, base_data=None, mem=None):
    angles, dists = scan_pair or scan()
    cv = CV(dets, world)
    return SelfDriver(Base(RL(angles, dists), base_data),
                      cv, memory=mem or spatial_memory.SpatialMemory()), cv


# ── 0. call sites of every module-level function still match its signature ───
# This is the guard for the bug class the restructure introduced: moving a
# helper and dropping its defaults silently breaks callers that relied on them.
print("--- 0. call-site arity across the refactored modules ---")

MODULES = ["perception", "robot_state", "spatial_memory", "target_pursuit",
           "detection_source", "self_drive", "base_ctrl"]
SOURCES = ["app.py", "lance.py", "cv_ctrl.py", "self_drive.py",
           "spatial_memory.py", "detection_source.py", "robot_state.py",
           "target_pursuit.py", "perception.py"]
mods = {}
for _name in MODULES:
    try:
        mods[_name] = importlib.import_module(_name)
    except Exception as _e:      # e.g. base_ctrl needs pyserial, absent off-Pi
        print("  (not importable here, call sites in it are unchecked: %s: %s)"
              % (_name, _e))

arity_problems, checked = [], 0
for src in SOURCES:
    with open(src, encoding="utf-8") as fh:
        tree = ast.parse(fh.read(), filename=src)
    binds = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module in mods:
            for alias in node.names:
                binds[alias.asname or alias.name] = (node.module, alias.name)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or node.keywords:
            continue
        fn = node.func
        if isinstance(fn, ast.Name):
            key = binds.get(fn.id)
        elif isinstance(fn, ast.Attribute) and isinstance(fn.value, ast.Name) \
                and fn.value.id in mods:
            key = (fn.value.id, fn.attr)
        else:
            continue
        if not key:
            continue
        obj = getattr(mods[key[0]], key[1], None)
        if not callable(obj):
            continue
        try:
            params = [p for p in inspect.signature(obj).parameters.values()
                      if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
        except (TypeError, ValueError):
            continue
        required = len([p for p in params if p.default is p.empty])
        checked += 1
        if not required <= len(node.args) <= len(params):
            arity_problems.append("%s:%d %s.%s(%d args, signature wants %d..%d)"
                                  % (src, node.lineno, key[0], key[1],
                                     len(node.args), required, len(params)))
check("every call into a moved module matches its signature (%d call sites)"
      % checked, not arity_problems, "; ".join(arity_problems[:4]))

# ── 0b. the executor consults the planner's halt before it moves forward ─────
# app.py cannot be imported off the Pi (it boots Flask and opens hardware), so
# this is asserted from its syntax tree: in LidarAvoider._decide the planner's
# halt must be checked before any wheel command, because that is exactly what
# went wrong — halt was honored in CRUISE alone, so the 450-700 mm SLOW band
# kept driving the robot on after the planner had said to stop.
print("--- 0b. executor: halt precedes every forward command ---")
app_tree = ast.parse(open("app.py", encoding="utf-8").read(), filename="app.py")
avoider = next(n for n in ast.walk(app_tree)
               if isinstance(n, ast.ClassDef) and n.name == "LidarAvoider")
decide = next(n for n in avoider.body
              if isinstance(n, ast.FunctionDef) and n.name == "_decide")
halt_lines = [n.lineno for n in ast.walk(decide)
              if isinstance(n, ast.Attribute) and n.attr == "_planner_halted"]
send_lines = [n.lineno for n in ast.walk(decide)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr == "_send"]
check("_decide() checks _planner_halted() before every wheel command",
      bool(halt_lines) and bool(send_lines)
      and all(line > min(halt_lines) for line in send_lines),
      "halt@%s sends@%s" % (halt_lines, sorted(send_lines)))
check("the avoider has a HOLD state to report while halted",
      any(isinstance(n, ast.Assign)
          and getattr(n.targets[0], "id", "") == "HOLD" for n in avoider.body))

# ...and its commands keep arriving.  The chassis expires motion it has not
# heard again (CMD_HEART_BEAT_SET, tutorial_en/08: 3 s by default), measured
# here as ~3.5 s of travel from a single frame.  _send used to drop every repeat,
# so once a heading settled the avoider stopped talking, the heartbeat stopped
# the wheels and the robot sat still while the state machine said CRUISE.
CHASSIS_HEARTBEAT_SEC = 3.0
send = next(n for n in avoider.body
            if isinstance(n, ast.FunctionDef) and n.name == "_send")
keepalive = [n for n in ast.walk(app_tree) if isinstance(n, ast.Assign)
             and getattr(n.targets[0], "id", "") == "SEND_KEEPALIVE_SEC"]
seconds = ast.literal_eval(keepalive[0].value) if keepalive else None
check("a held drive command is re-sent on a timer, not only when it changes",
      bool(keepalive) and any(isinstance(n, ast.Attribute) and n.attr == "_sent_at"
                              for n in ast.walk(send)), seconds)
check("...and that timer is well inside the chassis heartbeat",
      isinstance(seconds, (int, float)) and 0 < seconds < CHASSIS_HEARTBEAT_SEC,
      seconds)

# ── 0c. a pause is not overtaken by the tick that was already running ────────
# pause(halt=True) clears _active and *then* sends its zeros, so a decision in
# flight could put a motion frame on the wire after them, and the chassis drives
# on that frame for the whole heartbeat.  Measured live, a halt was followed by
# 0.570 m of wheel travel, and by 0.000 m when the zeros were the last thing the
# chassis heard.  The method is run for real here against a base that records
# what arrives: this is a race in an order the syntax tree cannot show.
_src = open("app.py", encoding="utf-8").read()
_ns = {"time": time, "SEND_KEEPALIVE_SEC": seconds}
exec("import time\n" + textwrap.dedent(ast.get_source_segment(_src, send)), _ns)


class _Recorder:
    def __init__(self):
        self.frames = []

    def base_json_ctrl(self, data):
        self.frames.append(data)


_halt = type("Avoider", (), {})()
_halt._base, _halt._sent_at = _Recorder(), 0.0
_halt._last_L = _halt._last_R = 0.0
_halt._active = True
_send_fn = types.MethodType(_ns["_send"], _halt)
_send_fn(0.3, 0.3)                      # driving: the frame goes out
_after = len(_halt._base.frames)
_halt._active = False                   # as pause(halt=True) leaves it
_send_fn(0.0, 0.0)                      # ...and its own halt frame still goes
_send_fn(0.3, -0.3)                     # the in-flight tick must not
check("a paused avoider leaves no motion frame on the wire",
      [f for f in _halt._base.frames[_after:] if f["L"] or f["R"]] == [],
      _halt._base.frames[_after:])
check("...while its own halt still reaches the chassis",
      [f for f in _halt._base.frames[_after:] if not f["L"] and not f["R"]] != [])

# ── 1. the refactored angle math is numerically identical ────────────────────
print("--- 1. perception == the math it replaced (randomised) ---")


def old_sector_min(angles_rad, distances_mm, center_deg, half_deg, min_mm=50):
    lo, hi = math.radians(center_deg - half_deg), math.radians(center_deg + half_deg)
    best = float('inf')
    for a, d in zip(angles_rad, distances_mm):
        a = math.atan2(math.sin(a - math.pi), math.cos(a - math.pi))
        if lo <= a <= hi and d >= min_mm and d < best:
            best = d
    return best


def old_turn_bias(angles_rad, distances_mm, half_deg, danger_mm, min_mm=50):
    left_w = right_w = 0.0
    lo, hi = math.radians(-half_deg), math.radians(half_deg)
    for a, d in zip(angles_rad, distances_mm):
        a = math.atan2(math.sin(a - math.pi), math.cos(a - math.pi))
        if not (lo <= a <= hi) or d < min_mm or d > danger_mm:
            continue
        w = (danger_mm - d) / danger_mm
        if a < 0:
            right_w += w
        else:
            left_w += w
    total = left_w + right_w
    return 0.0 if total < 1e-6 else max(-1.0, min(1.0, (right_w - left_w) / total))


random.seed(7)
same_sector = same_bias = True
for trial in range(200):
    angles, dists = scan(random.choice([500, 1500, 3000]),
                         {random.randint(-60, 60): random.randint(50, 2000)})
    if old_sector_min(angles, dists, 0, 45) != perception.sector_min(
            [perception.robot_angle(a) for a in angles], dists, 0, 45):
        same_sector = False
    robot_angles = [perception.robot_angle(a) for a in angles]
    if old_turn_bias(angles, dists, 45, 450) != perception.turn_bias(
            robot_angles, dists, 45, 450):
        same_bias = False
check("sector_min identical to the old implementation (200 random scans)", same_sector)
check("turn_bias identical to the old implementation (200 random scans)", same_bias)
check("box on the left of frame -> + bearing",
      perception.box_bearing_deg([60, 0, 140, 100]) > 0,
      perception.box_bearing_deg([60, 0, 140, 100]))
check("box on the right of frame -> - bearing",
      perception.box_bearing_deg([440, 0, 520, 100]) < 0)
check("label matching singular/plural", perception.names_match("chairs", "chair")
      and perception.names_match("chair", "chair"))

# ── 2. lifecycle: warm / enable / disable ────────────────────────────────────
print("--- 2. thread lifecycle owns its own transitions ---")
drv, _ = planner()
drv.warm()
check("warm() = thread up, planner idle, paused",
      drv._thread is not None and drv._thread.is_alive()
      and not drv.active and drv.last_decision == "paused")
drv.enable()
check("enable() -> active", drv.active)
drv.pursue("chair")
check("pursue() sets the target and enables",
      drv.active and drv.target.name == "chair")
drv.disable()
check("disable() = inactive + thread gone + target cleared",
      not drv.active and drv._thread is None and drv.target.name is None)
drv.enable()
check("enable() after disable() restarts the thread",
      drv.active and drv._thread is not None and drv._thread.is_alive())
drv.stop()

# ── 3. same status payload shape as before the restructure ───────────────────
print("--- 3. /selfdrive_status payload unchanged ---")
drv, _ = planner(base_data={'odl': -10398.7, 'odr': -9916.4, 'v': 11.48})
drv.warm()
st = drv.status()
check("keys identical",
      set(st) == {'active', 'suggested_turn', 'front_mm', 'halt', 'wheels',
                  'decision', 'busy_cells', 'free_cells', 'objects', 'scores',
                  'target', 'map', 'course_deg', 'covered_cells'},
      sorted(st))
check("wheels read from the ESP32 frame, forward positive", st['wheels'] ==
      {'odl': 10398.7, 'odr': 9916.4, 'voltage': 11.48})
check("no target -> target is null", st['target'] is None)
check("the map reports its own frame", st['map']['pose'] == [0.0, 0.0, 0.0]
      and st['map']['motion'] in ('unknown', 'none', 'waiting', 'unchecked',
                                  'accepted', 'rejected'), st['map'])
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
t = drv.status()['target']
check("target block keys identical",
      set(t) == {'name', 'state', 'bearing_deg', 'range_m', 'seen', 'halt'}, sorted(t))
drv.stop()

# ── 4. safety comes first ────────────────────────────────────────────────────
print("--- 4. safety: no lidar, too close, surrounded ---")
drv, _ = planner(([], []))
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
check("no lidar -> refuse a heading, no bogus stop",
      drv.suggested_turn == 0.0 and drv.last_decision == "no lidar data"
      and drv.halt is False, drv.last_decision)
drv.stop()

drv, _ = planner(scan(sectors={0: 200}))
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
check("<250mm ahead -> halt + zero turn",
      drv.halt and drv.suggested_turn == 0.0 and "too close" in drv.last_decision,
      drv.last_decision)
drv.stop()

drv, _ = planner(scan(front_mm=500))
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")   # the veto only exists while pursuing
drv._tick()
check("everything blocked while pursuing -> hold position",
      drv.halt and drv.suggested_turn == 0.0
      and drv.last_decision == "surrounded — holding position", drv.last_decision)
drv.stop()

# ── 5. cruise behavior unchanged ─────────────────────────────────────────────
print("--- 5. plain cruise ---")
drv, _ = planner()
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
check("clear room, no target -> straight ahead",
      drv.suggested_turn == 0.0 and drv.last_decision.startswith("heading +0°"),
      drv.last_decision)
check("scores still published for the UI", len(drv.last_scores) == 13)
drv.stop()

# ── 5b. steering smoothing: no wobble on noise, no snap on a real turn ─────
print("--- 5b. steering smoothing ---")


def set_scan(drv, pair):
    """Swap the stub's live scan, exactly as the LIDAR thread does."""
    angles, dists = pair
    drv._base.rl.lidar_angles_show = angles
    drv._base.rl.lidar_distances_show = dists


# The candidate arcs overlap (0 deg samples -22..+22, +15 deg samples
# -8..+38), so a pinch inside -18..-16 deg costs the 0 deg heading samples
# without touching +15 deg: it makes the +15 heading win on that scan alone.
# Alternating the pinch side is exactly the scan noise that used to flip the
# chosen heading, and with it the wheels, every single tick.
PINCH_LEFT = scan(sectors={d: 900 for d in range(-18, -15)})
PINCH_RIGHT = scan(sectors={d: 900 for d in range(15, 18)})

drv, _ = planner()
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
turns, chosen = [], []
for pair in (PINCH_LEFT, PINCH_RIGHT) * 6:
    set_scan(drv, pair)
    drv._tick()
    turns.append(drv.suggested_turn)
    chosen.append(drv.last_decision.split("\u00b0")[0])
check("alternating noise -> no wheel differential at all, still aiming straight",
      turns and set(turns) == {0.0}, "turns=%s" % sorted(set(turns)))

# A wall dead ahead is a real change, so the hold must release at once — but
# the wheels ramp to the new heading instead of snapping to full lock.
set_scan(drv, scan(sectors={d: 900 for d in range(-6, 7)}))
drv._tick()
one = abs(drv.suggested_turn)
check("a real obstacle releases the hold and slews (one tick <= slew step)",
      0.0 < one <= TURN_SLEW_PER_TICK + 1e-9, one)
for _ in range(12):
    drv._tick()
check("...and converges on the new heading",
      abs(drv.suggested_turn) >= 0.3, drv.suggested_turn)

# Back to a clear room: the residual turn must collapse to EXACTLY zero so the
# two wheels get identical commands, not a permanent 1% differential.
set_scan(drv, scan())
for _ in range(8):
    drv._tick()
check("clear room again -> turn decays to exactly 0.0, wheels equal",
      drv.suggested_turn == 0.0, drv.suggested_turn)
drv.stop()

# The learned map must be able to forget. It used to count hits to 65535 and
# decay one per DECAY_SEC, so after a drive every direction read blocked
# (clearance 0.0 for all 13 headings) and the memory term stopped mattering.
print("--- 5c. learned map can decay ---")
mem = spatial_memory.SpatialMemory()
for _ in range(40):
    mem.observe_lidar([math.radians(0.0)] * 40, [1000] * 40)
cxf, cyf = mem._cell(0.0, 1.0)
check("hit count saturates at HIT_MAX, not 65535",
      mem._hits[cxf][cyf] == spatial_memory.HIT_MAX,
      "%d hits at %d,%d" % (mem._hits[cxf][cyf], cxf, cyf))
check("a saturated cell blocks the heading", mem.blocked(0.0, 1.0) is True)
for _ in range(spatial_memory.HIT_MAX + 2):
    mem._decay_locked(time.time() + 3600)
check("an unobserved cell decays away so clearance recovers",
      mem.blocked(0.0, 1.0) is False, mem._hits[cxf][cyf])

# Camera-fused object disks share the same ceiling. They used to climb past it
# (cap 65535) and then decay at 1/s, so a moved-on object stayed blocked.
ocs = spatial_memory.SpatialMemory()
for _ in range(30):
    ocs.observe_object("chair", 0.0, 1.0, 0.9)
peak_before = max(h for row in ocs._hits for h in row)
check("an object disk also saturates at HIT_MAX, not 65535",
      peak_before == spatial_memory.HIT_MAX, "%d hits" % peak_before)
for _ in range(spatial_memory.HIT_MAX + 2):
    ocs._decay_locked(time.time() + 3600)
peak_after = max(h for row in ocs._hits for h in row)
# An object is a claim about where something is now, not a place: nothing the
# LIDAR ever confirmed, so once the camera stops reporting it there is nothing
# to keep and it decays away entirely rather than lingering at the floor.
check("an unobserved object disk decays away instead of lingering",
      peak_after == 0 and not ocs.blocked(0.0, 1.0), peak_after)

# ── 6. pursuit: aim, smoothness, arrival, search, veto ──────
print("--- 6. pursuit behaviors ---")
left_det = [{"name": "chair", "confidence": 0.9, "box": [60, 100, 140, 300]}]
drv, _ = planner(dets=left_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
t = drv.status()['target']
check("left-of-frame object -> + bearing, approaching",
      t['bearing_deg'] > 0 and t['state'] == 'approaching' and t['seen'], t)
aim = t['bearing_deg'] / 45.0
discrete_snap = 15.0 / 90.0
check("aims proportionally at the object, not at the discrete +15 snap",
      abs(drv.suggested_turn - aim) < 0.01
      and abs(drv.suggested_turn - discrete_snap) > 0.01,
      "turn=%s aim=%s snap=%s" % (drv.suggested_turn, round(aim, 3), discrete_snap))
check("decision names target + range",
      drv.last_decision.startswith("approaching chair bearing"), drv.last_decision)
drv.stop()

right_det = [{"name": "chair", "confidence": 0.9, "box": [440, 100, 520, 300]}]
drv, _ = planner(dets=right_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
check("right-of-frame object -> aims right", drv.suggested_turn < 0, drv.suggested_turn)
drv.stop()

near_det = [{"name": "refrigerator", "confidence": 0.66, "box": [280, 100, 315, 300]}]
drv, _ = planner(dets=near_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("refrigerator")
turns = []
for _ in range(4):
    drv._tick()
    turns.append(round(drv.suggested_turn, 3))
check("near-centre target: identical turn every tick (no +/-15 jitter)",
      len(set(turns)) == 1, turns)
drv.stop()

# left_det sits at bearing +20.6°, so the near returns have to cover that
# bearing for it to be the *object* the lidar sees, not something beside it.
AT_BEARING = {d: 500 for d in range(11, 31)}
drv, _ = planner(scan(sectors=AT_BEARING), dets=left_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
check("0.5m object down its own bearing -> arrived + halt",
      drv.halt and drv.status()['target']['state'] == 'arrived', drv.last_decision)
# The executor's guard reads active+halt, so turning self-drive off has to clear
# both — otherwise a manual user could not drive away from the object.
drv.disable()
check("after arrival, disable() clears halt and active (manual driving unaffected)",
      not drv.halt and not drv.active)
drv.stop()

# Arrival has to be sticky. The object leaving the camera view is what happens
# the instant the robot is on top of it; re-deciding arrival each tick used to
# release halt right there and send the robot off again having reached it.
drv, cv = planner(scan(sectors=AT_BEARING), dets=left_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
cv.last_detections = []                      # object out of view now
for _ in range(3):
    drv._tick()
check("object lost from view after arrival -> halt still held, no turn",
      drv.halt and drv.suggested_turn == 0.0
      and drv.status()['target']['state'] == 'arrived',
      "halt=%s turn=%s %s" % (drv.halt, drv.suggested_turn, drv.last_decision))
# ...but seeing it again beyond the arrival band means it (or we) moved, so the
# chase must resume rather than stay frozen for good.
FAR_OUT = scan(sectors={d: 3000 for d in range(11, 31)})
drv, _ = planner(scan_pair=FAR_OUT, dets=left_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
check("object seen again beyond arrive_m -> resumes approaching, halt released",
      not drv.halt and drv.status()['target']['state'] == 'approaching',
      drv.last_decision)
drv.stop()

# The defect this replaced: the forward cone measured a wall beside the object,
# so the robot declared arrival at the wall's distance and stopped short.
BESIDE = {d: 500 for d in range(-40, -5)}
drv, _ = planner(scan(sectors=BESIDE), dets=left_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
t = drv.status()['target']
check("0.5m wall beside the object -> still approaching, no false arrival",
      not drv.halt and t['state'] == 'approaching' and t['range_m'] > 1.0,
      "range=%s halt=%s %s" % (t['range_m'], drv.halt, drv.last_decision))
drv.stop()

mem = spatial_memory.SpatialMemory()
mem.observe_object("chair", 40.0, 1.5, 0.9)          # remembered on the left
drv, _ = planner(mem=mem)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
first = drv.suggested_turn
check("not in view -> sweep toward the remembered bearing",
      first > 0 and "looking for chair" in drv.last_decision, drv.last_decision)
check("sweep flips direction after SCAN_FLIP_S",
      drv.target.next_scan_turn(now=time.time() + target_pursuit.SCAN_FLIP_S + 1) < 0)
drv.stop()

mem2 = spatial_memory.SpatialMemory()
mem2.observe_object("chair", -40.0, 1.5, 0.9)        # remembered on the right
drv, _ = planner(mem=mem2)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("chair")
drv._tick()
check("sweep direction follows the memory sign", drv.suggested_turn < 0,
      drv.suggested_turn)
drv.stop()

wall = {d: 900 for d in range(-12, 13)}
drv, _ = planner(scan(sectors=wall), dets=near_det)
ticking(drv)     # no planner thread: this harness ticks it
drv.pursue("refrigerator")
drv._tick()
check("wall across the bearing -> veto, route around, keep driving",
      dict(drv.last_scores).get(0) is None and drv.suggested_turn != 0.0
      and not drv.halt and "(going around)" in drv.last_decision,
      drv.last_decision)
drv.stop()

# ── 7. open-vocabulary top-up (now DetectionSource) ─────────────────────────
print("--- 7. detections: closed-set live stream + open-vocabulary top-up ---")
coco = [{"name": "person", "confidence": 0.9, "box": [280, 100, 360, 300]}]
world = [{"name": "refrigerator", "confidence": 0.66, "box": [60, 100, 140, 300]}]
drv, cv = planner(dets=coco, world=world)
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
check("no target -> no open-vocabulary scan", cv.world_calls == 0, cv.world_calls)
drv.pursue("refrigerator")
drv._tick()
t = drv.status()['target']
check("pursues an object only the open-vocabulary model can name",
      t['seen'] and t['state'] == 'approaching' and t['bearing_deg'] > 0, t)
check("scan throttled to one call", cv.world_calls == 1, cv.world_calls)
drv._tick()
check("throttled: next tick reuses the cache", cv.world_calls == 1, cv.world_calls)
src = DetectionSource(cv, world_scan_s=5.0)
src.for_target("refrigerator", now=100.0)
src.for_target("refrigerator", now=101.0)
check("DetectionSource throttle independent of the planner", cv.world_calls == 2,
      cv.world_calls)
drv.stop()

# object fusion still writes the corrected bearing into memory
mem3 = spatial_memory.SpatialMemory()
drv, _ = planner(dets=left_det, mem=mem3)
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
check("detections fused into memory with + bearing (left object)",
      mem3.objects and mem3.objects[0]['bearing_deg'] > 0 and
      mem3.objects[0]['name'] == 'chair', mem3.objects[:1])
drv.stop()

# ── 7b. the planner's object view does not wait for the CV overlay ──────────
# Live, the camera saw a surfboard while the map held 0 objects: the only stream
# the planner read was cvf.last_detections, which the CV overlay refreshes while
# it happens to be in one of its object modes.  The gaze already runs the shared
# detector while the eyes are on, so it is read first.
print("--- 7b. detections: the gaze's own camera pass reaches the planner ---")


class Gaze:
    """A gaze status, shaped the way eyes_gaze.status() publishes it."""

    def __init__(self, dets=None, enabled=True, stale=False, age=0.2):
        self._status = {'enabled': enabled, 'frame_stale': stale, 'age_s': age,
                        'detections': dets or []}

    def status(self):
        return dict(self._status)


# Boxes are fractions of the frame: 0.75-0.95 is the right third of it.
person_box = [{'name': 'person', 'conf': 0.81, 'person': True,
               'box': [0.75, 0.30, 0.95, 0.85]}]
cv_off = CV()                                # overlay off: it has no hits
src = DetectionSource(cv_off, gaze=Gaze(person_box))
dets = src.live()
check("a person reaches the planner with the CV overlay off (the overlay gate)",
      len(dets) == 1 and dets[0]['name'] == 'person', dets)
check("...with the gaze's fraction-of-frame box scaled into the map's terms",
      dets[0]['box'] == [480, 144, 608, 408]
      and -25.0 < perception.box_bearing_deg(dets[0]['box']) < -17.0,
      dets[0]['box'])

mem_g = spatial_memory.SpatialMemory()
mem_g.observe_detections(dets, 1.5)
check("one sighting remembers the object",
      len(mem_g.objects) == 1 and mem_g.objects[0]['name'] == 'person'
      and mem_g.objects[0]['bearing_deg'] < 0, mem_g.objects[:1])
mem_g.observe_detections(src.live(), 1.5)    # seen again, as the app keeps doing
mem_g.observe_detections(src.live(), 1.5)
check("...and once seen twice, that heading is refused",
      mem_g.clearance(math.radians(-21.0), 1.5) < 1.0,
      mem_g.clearance(math.radians(-21.0), 1.5))

# Staleness: only a gaze that is running and whose camera is answering speaks
# for the planner.  A slower, intermittent source must not be read as current.
check("a gaze whose loop has stopped stepping feeds the planner nothing",
      DetectionSource(cv_off, gaze=Gaze(person_box, age=30.0)).live() == [])
check("a switched-off gaze feeds the planner nothing",
      DetectionSource(cv_off, gaze=Gaze(person_box, enabled=False)).live() == [])
check("a gaze with a frozen camera frame feeds the planner nothing",
      DetectionSource(cv_off, gaze=Gaze(person_box, stale=True)).live() == [])
check("no gaze at all leaves the overlay stream as the only source",
      DetectionSource(cv_off).live() == [])

# Two paths reporting one chair must stay one chair.  Nothing is merged in
# DetectionSource: the memory's own name/bearing/range dedupe is the only one.
overlay_chair = [{'name': 'chair', 'confidence': 0.7, 'box': [280, 100, 360, 300]}]
gaze_chair = [{'name': 'chair', 'conf': 0.9, 'person': False,
               'box': [0.44, 0.21, 0.56, 0.63]}]
mem_b = spatial_memory.SpatialMemory()
mem_b.observe_detections(
    DetectionSource(CV(overlay_chair), gaze=Gaze(gaze_chair)).live(), 1.5)
check("the same chair from both streams is one object",
      len(mem_b.objects) == 1, mem_b.objects)

# Where things are *now*: a person crossing the frame must stop refusing the
# heading they were last seen at, and must not leave a fence of disks behind.
moving = DetectionSource(cv_off, gaze=Gaze(person_box))
mem_m = spatial_memory.SpatialMemory()
for _ in range(3):
    mem_m.observe_detections(moving.live(), 1.5)
before = mem_m.clearance(math.radians(-21.0), 1.5)
moving.gaze = Gaze([{'name': 'person', 'conf': 0.81, 'person': True,
                     'box': [0.05, 0.30, 0.25, 0.85]}])
for _ in range(3):
    mem_m.observe_detections(moving.live(), 1.5)
check("a person who walks across the frame stops refusing the old heading",
      before < 1.0 and mem_m.clearance(math.radians(-21.0), 1.5) == 1.0,
      "%s -> %s" % (before, mem_m.clearance(math.radians(-21.0), 1.5)))
check("...and the heading they are at now is the refused one",
      mem_m.clearance(math.radians(21.0), 1.5) < 1.0,
      mem_m.clearance(math.radians(21.0), 1.5))
# An empty batch (nothing in view) releases the disk the same way.
for _ in range(3):
    mem_m.observe_detections([], 1.5)
check("nothing in view releases every object cell",
      mem_m.clearance(math.radians(21.0), 1.5) == 1.0,
      mem_m.clearance(math.radians(21.0), 1.5))

# ...but a cell the LIDAR has confirmed is a *place*: a person standing in front
# of a wall must not erase the wall by walking away, or a room with people in it
# would churn the map every camera pass.
mem_s = spatial_memory.SpatialMemory()
angles, dists = scan(front_mm=1500)
for _ in range(3):
    mem_s.observe_lidar([perception.robot_angle(a) for a in angles], dists)
check("three scans of a wall 1.5 m ahead block the cell",
      mem_s.blocked(0.0, 1.5), mem_s.pose_status())
spot = mem_s._disk_cells(0.0, 1.5)
check("a person dead ahead covers LIDAR-confirmed cells (the case this pins)",
      bool(spot) and any(mem_s._sure[c][r] for c, r in spot), spot[:3])
mem_s.observe_detections([{'name': 'person', 'confidence': 0.9,
                           'box': [280, 100, 360, 300]}], 1.5)
mem_s.observe_detections([], 1.5)            # they walked away
kept = [mem_s._sure[c][r] for c, r in spot if mem_s._sure[c][r]]
check("...walking away leaves the wall's place intact, not erased",
      bool(kept) and max(mem_s._hits[c][r] for c, r in spot)
      >= spatial_memory.MEM_FLOOR, kept[:3])
for _ in range(3):                           # a few more revolutions
    mem_s.observe_lidar([perception.robot_angle(a) for a in angles], dists)
check("...and the LIDAR has its blocking back within three revolutions",
      mem_s.blocked(0.0, 1.5), [(c, r, mem_s._hits[c][r]) for c, r in spot[:3]])

# The wiring, not just the class: app.py builds the gaze *after* the planner, so
# a dropped line here would put the object path back behind the overlay gate.
with open("app.py", encoding="utf-8") as fh:
    app_tree = ast.parse(fh.read(), filename="app.py")


def _dotted(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    return ".".join(reversed(parts + [getattr(node, "id", "")]))


check("app.py attaches the gaze to the planner's detection source",
      any(isinstance(n, ast.Assign)
          and any(_dotted(t) == "self_driver.detections.gaze" for t in n.targets)
          for n in ast.walk(app_tree)))

# ── 7c. a detection is ranged where the LIDAR actually sees it ────────────
# Every detection used to be given the front cone's range, so an object 30 deg
# off to the side was stored at whatever was nearest straight ahead: live, one
# `oven` was held twice at the same bearing 0.46 m and 1.7 m apart, and a view
# of a handful of things produced 20 entries.
print("--- 7c. detections are ranged at their own bearing ---")

# The room: 1.0 m straight ahead, a wall 2.5 m away across +20..+40 deg.  The
# box is the left edge of the frame, which is that bearing.
side_det = [{"name": "chair", "confidence": 0.9, "box": [0, 100, 60, 300]}]
side_deg = perception.box_bearing_deg(side_det[0]["box"])
check("the side detection's own bearing is where the wall is (+27 deg)",
      25.0 < side_deg < 29.0, side_deg)

side_room = scan(front_mm=1000, sectors={d: 2500 for d in range(20, 41)})
drv, _ = planner(side_room, dets=side_det)
ticking(drv)
drv._tick()
obj = drv.memory.objects[0]
check("a detection 27 deg to the side is stored 2.5 m away, not the front's 1.0",
      len(drv.memory.objects) == 1 and 2.4 <= obj["range_m"] <= 2.6
      and obj["bearing_deg"] > 25.0, obj)
drv.stop()

# The look is as wide as the box, so a box only a few pixels wide still finds
# beams (the sensor samples about every 2 deg of turn).
thin_det = [{"name": "chair", "confidence": 0.9, "box": [10, 100, 26, 300]}]
drv, _ = planner(side_room, dets=thin_det)
ticking(drv)
drv._tick()
check("...and a box a few pixels wide finds them too",
      2.4 <= drv.memory.objects[0]["range_m"] <= 2.6, drv.memory.objects[0])
drv.stop()

# No return in the sector the camera can see: the front cone stays the honest
# fallback rather than a made-up distance.  (The bins are what a detection is
# ranged from, because the single revolution arrives in patches where the near
# return never came: live, that placed objects up to 2.5 m too far.)
gap_room = scan(front_mm=1000, sectors={d: 0 for d in range(15, 40)})
check("a sector with no usable return answers None, not a distance",
      robot_state.dense_sector_min_mm(Base(RL(*gap_room)), side_deg, 5.0) is None,
      robot_state.dense_sector_min_mm(Base(RL(*gap_room)), side_deg, 5.0))
drv, _ = planner(gap_room, dets=side_det)
ticking(drv)
drv._tick()
check("...so that detection keeps the front range",
      0.9 <= drv.memory.objects[0]["range_m"] <= 1.1, drv.memory.objects[0])
drv.stop()

# The LIDAR's own frame, not a mirrored one: a wall on the other side of the
# robot must not range this detection.
wrong_side = scan(front_mm=1000, sectors={d: 2500 for d in range(-33, -22)})
drv, _ = planner(wrong_side, dets=side_det)
ticking(drv)
drv._tick()
check("a wall on the other side is not used for it (the frame sign holds)",
      0.9 <= drv.memory.objects[0]["range_m"] <= 1.1, drv.memory.objects[0])
drv.stop()

# A bin nobody filled recently says nothing about where anything is now.
stale_room = scan(front_mm=1000, sectors={d: 2500 for d in range(20, 41)})
drv, _ = planner(stale_room, dets=side_det)
ticking(drv)
drv._base.rl.lidar_bins = [(d, t - 30.0) for d, t in drv._base.rl.lidar_bins]
drv._tick()
check("a stale bin is not believed (the front range is used instead)",
      0.9 <= drv.memory.objects[0]["range_m"] <= 1.1, drv.memory.objects[0])
drv.stop()

# Why the object list used to hold the same thing twice: the range moved with
# the robot's view, and two sightings more than OBJ_DEDUPE_M apart are two
# objects.  Seen twice with the front range changing, one object still.
near_det = [{"name": "oven", "confidence": 0.6, "box": [0, 100, 60, 300]}]
room_a = scan(front_mm=1000, sectors={d: 2500 for d in range(20, 41)})
room_b = scan(front_mm=460, sectors={d: 2500 for d in range(20, 41)})
drv, _ = planner(room_a, dets=near_det)
ticking(drv)
drv._tick()
drv._base.rl = RL(*room_b)                    # it drove on: the front range moved
drv._tick()
check("one object seen twice, with the front range moving, stays one object",
      len(drv.memory.objects) == 1, drv.memory.objects)
check("...and it is still stored at the wall's range both times",
      all(2.4 <= o["range_m"] <= 2.6 for o in drv.memory.objects),
      drv.memory.objects)
# ...which is what the front-only range could not do, and the failure this pins.
mem_near = spatial_memory.SpatialMemory()
mem_near.observe_detections(near_det, 1.7)
mem_near.observe_detections(near_det, 0.46)
check("the live failure itself: the front-only ranges split one object in two",
      len(mem_near.objects) == 2, mem_near.objects)
drv.stop()

# One chair, seen from two viewpoints, is one object.  Matching on the bearing
# alone is what turned a room holding a handful of things into 133 entries: as
# the robot turned, the same chair kept reappearing at a new bearing.
mem_place = spatial_memory.SpatialMemory()
mem_place.observe_detections([{"name": "chair", "confidence": 0.9,
                               "box": [0, 100, 60, 300]}], 2.0)      # +27 deg, 2.0 m
mem_place.pose = (1.0, 1.0, 0.0)                # it drove 1 m left and 1 m on
# ...so that same place is now 0.78 m away at -6.3 deg, seen through the box
# that points there.
mem_place.observe_detections([{"name": "chair", "confidence": 0.9,
                               "box": [327, 100, 447, 300]}], 0.78)
check("one chair from two viewpoints is one object, at its newest bearing",
      len(mem_place.objects) == 1 and -8.0 < mem_place.objects[0]["bearing_deg"] < -4.0,
      mem_place.objects)
# ...while two chairs in different places stay two objects.
mem_two = spatial_memory.SpatialMemory()
chair_box = [280, 100, 360, 300]                # dead ahead
mem_two.observe_detections([{"name": "chair", "confidence": 0.9,
                             "box": chair_box}], 1.0)
mem_two.observe_detections([{"name": "chair", "confidence": 0.9,
                             "box": chair_box}], 2.0)
check("two chairs a metre apart stay two objects",
      len(mem_two.objects) == 2, mem_two.objects)

# An entry is a claim about *now*, same as its cells: one no batch refreshes
# within OBJ_TTL_S is dropped.  Without this the list grew without bound --
# 268 entries live for a room holding a handful of things -- and the map file
# reloaded the ghosts into every later session.
mem_ttl = spatial_memory.SpatialMemory()
mem_ttl.observe_detections([{"name": "chair", "confidence": 0.9,
                             "box": [280, 100, 360, 300]}], 1.0)
check("a fresh entry is kept",
      len(mem_ttl.objects) == 1, mem_ttl.objects)
old = time.time() - (spatial_memory.OBJ_TTL_S + 5.0)
for o in mem_ttl.objects:
    o['t'] = old
mem_ttl.observe_detections([{"name": "tv", "confidence": 0.9,
                             "box": [0, 100, 40, 300]}], 1.5)     # a different bearing
check("an entry no batch has refreshed for OBJ_TTL_S is dropped",
      len(mem_ttl.objects) == 1 and mem_ttl.objects[0]['name'] == 'tv',
      mem_ttl.objects)
mem_keep = spatial_memory.SpatialMemory()
mem_keep.observe_detections([{"name": "chair", "confidence": 0.9,
                              "box": [280, 100, 360, 300]}], 1.0)
for o in mem_keep.objects:
    o['t'] = old
mem_keep.observe_detections([{"name": "chair", "confidence": 0.9,
                              "box": [280, 100, 360, 300]}], 1.0)   # same place, re-seen
check("an entry the camera still sees is refreshed, not dropped",
      len(mem_keep.objects) == 1
      and mem_keep.objects[0]['t'] > time.time() - spatial_memory.OBJ_TTL_S,
      mem_keep.objects)

# ...and a map file written before objects were placed must not multiply either:
# its entries re-place themselves the first time the camera sees them again.
probe = os.path.join(tempfile.gettempdir(), "_sd_selftest_place.json")
if os.path.exists(probe):
    os.remove(probe)
mem_p = spatial_memory.SpatialMemory(path=probe)
mem_p.observe_detections([{"name": "oven", "confidence": 0.6,
                           "box": [280, 100, 360, 300]}], 1.0)
mem_p.save()
with open(probe, encoding="utf-8") as fh:
    payload = json.load(fh)
for obj in payload['objects']:
    obj.pop('wx', None); obj.pop('wy', None)
with open(probe, "w", encoding="utf-8") as fh:
    json.dump(payload, fh)
mem_q = spatial_memory.SpatialMemory(path=probe)
os.remove(probe)
mem_q.observe_detections([{"name": "oven", "confidence": 0.6,
                           "box": [280, 100, 360, 300]}], 1.0)
check("a pre-place map entry re-places itself instead of multiplying",
      len(mem_q.objects) == 1 and mem_q.objects[0].get('wx') is not None,
      mem_q.objects)

# ── 8. memory: ownership, versioning, planner shortcuts ────────────────────
print("--- 8. memory ---")
tmp = os.path.join(tempfile.gettempdir(), "_sd_restructure_probe.json")
with open(tmp, "w") as fh:
    json.dump({"grid": [[7] * 121] * 121, "objects": [{"name": "x"}]}, fh)
old = spatial_memory.SpatialMemory(path=tmp)
check("v1 (pre-frame-fix) map is refused", old.busy_cells == 0 and old.objects == [])
old.observe_lidar(*[list(x) for x in ((),)]) if False else None
angles, dists = scan(front_mm=1000)
mem4 = spatial_memory.SpatialMemory(path=tmp)
mem4.observe_lidar([perception.robot_angle(a) for a in angles], dists)
drv, _ = planner(mem=mem4)
check("planner.save_memory() routes to the memory owner", drv.save_memory())
check("reloaded v2 map round-trips",
      spatial_memory.SpatialMemory(path=tmp).busy_cells > 0)
drv.clear_memory()
check("planner.clear_memory() empties the map", drv.memory.busy_cells == 0)
drv.load_memory()
check("planner.load_memory() reloads it", drv.memory.busy_cells > 0)
os.remove(tmp)

# ── 9. "drive to the <object>" reaches pursuit without the language model ────
print("--- 9. pursuit intent is read, not guessed ---")
for phrase, want in (("drive towards the chair", "chair"),
                     ("go to the door", "door"),
                     ("head for the person", "person"),
                     ("approach the table", "table"),
                     ("move over to the monitor", "monitor"),
                     ("navigate towards the tv", "tv"),
                     ("Drive To The Coffee Maker Please", "coffee maker"),
                     ("drive to the bin and then forward", "bin"),
                     ("drive towards a chair for 2 seconds", "chair"),
                     ("go to the 3rd chair", "3rd chair"),
                     ("drive to the mug on the table", "mug")):
    check("%r -> %r" % (phrase, want), lance.parse_pursuit_intent(phrase) == want,
          lance.parse_pursuit_intent(phrase))
for phrase in ("drive forward", "go back", "drive to the left",
               "drive for 2 seconds", "drive to 5 metres", "spin right",
               "turn left 90 degrees", "stop", "what do you see",
               "take a picture", "learn that this is a mug",
               "drive to the", "go to a", "approach the", "drive to"):
    check("%r is not a pursuit request" % phrase,
          lance.parse_pursuit_intent(phrase) is None,
          lance.parse_pursuit_intent(phrase))
check("approach/goto aliases all reach target pursuit",
      all(lance.ACTIONS[a] is lance._approach
          for a in ("approach", "goto", "go_to", "drive_to")))
check("pursuing is reachable the way the router calls it",
      lance.ACTIONS["approach"] is lance._approach)

# ── 10. the learned area is a place, and driving follows it ─────────────────
# The map used to be robot-centric: cells were "1.2 m ahead of me right now",
# so the same wall landed in different cells on every pass and the memory could
# say nothing about where the robot had been.  These probes drive the real
# planner and the real memory through a synthetic room whose walls never move,
# and ask whether the map agrees.
print("--- 10. place-referenced map + the learned area steering ---")

ROOM = (-2.5, 2.5, -3.0, 2.0)      # xmin, xmax, ymin, ymax — metres, map frame


def cast(pose, angles_rad, room=ROOM):
    """Ranges (mm) from a map-frame pose to the room's walls — a synthetic LIDAR."""
    x, y, theta = pose
    xmin, xmax, ymin, ymax = room
    out = []
    for a in angles_rad:
        psi = theta + a
        dx, dy = math.sin(psi), math.cos(psi)
        best = None
        for t in ([(xmax - x) / dx] if abs(dx) > 1e-9 else []) + \
                 ([(xmin - x) / dx] if abs(dx) > 1e-9 else []) + \
                 ([(ymax - y) / dy] if abs(dy) > 1e-9 else []) + \
                 ([(ymin - y) / dy] if abs(dy) > 1e-9 else []):
            if t <= 1e-9:
                continue
            hx, hy = x + t * dx, y + t * dy
            if xmin - 1e-6 <= hx <= xmax + 1e-6 and ymin - 1e-6 <= hy <= ymax + 1e-6:
                best = t if best is None else min(best, t)
        out.append(6000.0 if best is None else best * 1000.0)
    return out


_angles_raw, _ = scan()
ANG = [perception.robot_angle(a) for a in _angles_raw]
mem = spatial_memory.SpatialMemory()
for sid in (1, 2, 3):                       # three revolutions, standing still
    mem.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=sid)
check("a wall 2 m ahead is remembered at the place it is", mem.blocked(0.0, 2.0))
check("...with the way there known to be open", not mem.blocked(0.0, 1.0))

mem.observe_lidar(ANG, cast((0.0, 1.0, 0.0), ANG), odom=(1.0, 1.0), scan_id=4)
check("one metre of wheel travel is accepted as motion", mem.motion == 'accepted',
      mem.motion)
check("...and the pose followed the wheels", abs(mem.pose[1] - 1.0) < 0.05, mem.pose)
# blocked() answers in the robot frame, so a wall 2 m away that the robot has now
# closed to 1 m must read 1 m ahead -- and nothing at all 2 m ahead, which is
# where a map that never moved its frame would still be carrying the old sighting.
check("the wall is 1 m ahead of the robot, not 2", mem.blocked(0.0, 1.0))
check("the drive did not leave a second wall behind the first",
      not mem.blocked(0.0, 2.0))

# Turn left 90 degrees on the spot: the wheels report equal and opposite travel,
# so the frame rotates without the robot going anywhere.
turn = math.pi / 2 * perception.TRACK_M / 2.0
mem.observe_lidar(ANG, cast((0.0, 1.0, math.pi / 2), ANG),
                  odom=(-turn, turn), scan_id=5)
check("an in-place 90 deg turn is accepted", mem.motion == 'accepted', mem.motion)
check("the turn moved the pose, not the room", abs(mem.pose[2] - math.pi / 2) < 0.12
      and abs(mem.pose[1] - 1.0) < 0.05, mem.pose)
mem.observe_lidar(ANG, cast((0.0, 1.0, math.pi / 2), ANG), odom=(0.0, 0.0),
                  scan_id=6)
# Facing +x now: the far side wall is straight ahead (1.5 m), and the wall the
# robot started facing is off to its right, at -1.0 in a frame where +x is left.
check("the side wall is now straight ahead, at its own place",
      mem.blocked(0.0, 2.5), mem.pose)
check("the wall the robot first faced is remembered off to the right, not ahead",
      mem.blocked(-1.0, 0.0) and not mem.blocked(0.0, 2.0))

before = mem.pose
mem.observe_lidar(ANG, cast((0.0, 1.0, math.pi / 2), ANG), odom=(0.5, 0.5), scan_id=7)
check("wheels claiming motion the scan denies are rejected", mem.motion == 'rejected',
      mem.motion)
check("...and the pose stayed where the room puts it", mem.pose == before)

# The move above is the bench case at full size.  What decides whether the map
# is place-referenced in everyday driving is the *small* step: this LIDAR
# publishes a revolution every 0.10 s (measured on the robot), so a revolution
# carries 20 mm of travel at the slow speed and 35 mm at cruise — inside the
# sensor's own repeatability.  The guard used to judge every revolution with an
# 80 mm allowance, so every step this chassis can take passed it, and phantom
# travel walked the frame 0.9 m over 70 ticks before the map noticed anything.
# These memories are driven a revolution at a time at that scale: one where the
# wheels spin and the room does not move, one where the two move together.
per_rev = spatial_memory.FIT_EVERY_M / 5.0
memP = spatial_memory.SpatialMemory()
memP.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=101)
seen = {memP.motion}
for i in range(20):
    memP.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=(per_rev, per_rev),
                       scan_id=102 + i)
    seen.add(memP.motion)
check("phantom travel a revolution at a time never moves the frame",
      memP.pose == (0.0, 0.0, 0.0), memP.pose_status())
check("...because the scan refuses those steps (%.0f mm each)" % (per_rev * 1000),
      'rejected' in seen and 'accepted' not in seen, seen)

memR = spatial_memory.SpatialMemory()
memR.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=201)
travelled = 0.0
for i in range(20):
    travelled += per_rev
    memR.observe_lidar(ANG, cast((0.0, travelled, 0.0), ANG),
                       odom=(per_rev, per_rev), scan_id=202 + i)
check("...while the same travel with the room moving with it is followed",
      travelled - spatial_memory.FIT_EVERY_M <= memR.pose[1] <= travelled,
      memR.pose_status())

# The grid is a bounded window on the place frame, and the robot drives out of
# it: measured live, the pose had reached cell (152, 143) of a 0..120 grid, with
# 0 of 9,624 remembered cells within 3 m of the robot and 0 of 267 live returns
# landing inside the grid at all — nothing the sensors saw could be stored or
# steer.  Driven past the keep radius here, the window has to slide under the
# robot without moving any place in it.  The room is bigger than ROOM because
# ROOM is 5 m deep, and walking out of it is a different test.
WIDE = (-9.0, 9.0, -9.0, 5.0)
memW = spatial_memory.SpatialMemory()
memW.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG, room=WIDE), odom=None,
                   scan_id=601)
step = spatial_memory.FIT_EVERY_M
for i in range(1, 41):                       # 4 m of travel, past CENTRE_KEEP_M
    memW.observe_lidar(ANG, cast((0.0, i * step, 0.0), ANG, room=WIDE),
                       odom=(step, step), scan_id=601 + i)
check("4 m of travel is followed, not refused, in a room it can drive in",
      memW.motion == 'accepted', memW.pose_status())
check("the robot is kept inside its own grid however far it drives",
      abs(memW.pose[1]) <= spatial_memory.CENTRE_KEEP_M,
      "%s of 4.0 m claimed" % memW.pose[1])
check("...and a wall it has driven toward is still ahead of it, at its own place",
      memW.blocked(0.0, 1.0) and not memW.blocked(0.0, 2.0), memW.pose_status())

# A slip while turning is the same failure on the other axis: equal and opposite
# wheel travel with the room standing still.
memS = spatial_memory.SpatialMemory()
memS.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=301)
for i in range(10):
    memS.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG),
                       odom=(-per_rev, per_rev), scan_id=302 + i)
check("wheels claiming a turn the room does not show are refused too",
      memS.pose == (0.0, 0.0, 0.0), memS.pose_status())

# one revolution is counted once, however fast the planner ticks
mem9 = spatial_memory.SpatialMemory()
mem9.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=11)
once = mem9.busy_cells
mem9.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=11)
check("a scan already folded in is not counted again", mem9.busy_cells == once,
      "%d -> %d" % (once, mem9.busy_cells))

# What is kept, and what stops mattering.  Evidence fades to MEM_FLOOR and stays
# there, so the place survives in memory and in surroundings.json; blocking is
# decided by MIN_HITS of recent evidence, so a cell the robot has stopped seeing
# is remembered without being able to refuse a heading.  Decay is driven past
# the horizon here rather than slept through.
# Three revolutions, because the floor is for cells that reached MIN_HITS: one
# sighting is not evidence of a surface (see the stray-return check below).
memF = spatial_memory.SpatialMemory()
for sid in (401, 402, 403):
    memF.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=sid)
seen = memF.busy_cells
check("a wall 2 m ahead blocks while it is being seen",
      seen > 0 and memF.blocked(0.0, 2.0), memF.pose_status())
future = time.time() + 2 * spatial_memory.DECAY_SEC
for i in range(20):
    memF._decay_locked(future + i * (spatial_memory.DECAY_SEC + 1))
check("the place is still remembered long after the robot looked away",
      memF.busy_cells == seen, "%d -> %d" % (seen, memF.busy_cells))
check("...but stale evidence cannot refuse a heading any more",
      not memF.blocked(0.0, 2.0))
check("...and the cells are held at the floor, not decaying away",
      all(h == spatial_memory.MEM_FLOOR for _x, _y, h in memF.cells(min_hits=1)))

# A stray return is not a surface.  Measured on the robot: 7,380 of 12,449
# remembered cells sat at one hit with a median distance of 5.88 m — noise at the
# edge of the sensor's range — and 86% of everything remembered beyond 5.5 m was
# that noise, while 97% of the cells within 2 m had been confirmed.  One
# sighting must therefore decay away, and a confirmed cell must not.
memN = spatial_memory.SpatialMemory()
memN.observe_lidar(ANG, cast((0.0, 0.0, 0.0), ANG), odom=None, scan_id=501)
near = memN._cell(0.0, 2.0)
far = memN._cell(0.0, 5.5)
# A surface inside about two metres puts several rays in one cell per revolution
# (3 deg at 1.9 m is 0.1 m), so the wall the robot is looking at is confirmed by
# that single sweep; at 5.5 m the rays are 0.29 m apart, one per cell, which is
# why the far half of a real map is single-hit noise.
check("a wall close enough to return several rays is confirmed at once",
      memN._hits[near[0]][near[1]] >= spatial_memory.MIN_HITS,
      memN._hits[near[0]][near[1]])
memN._hits[far[0]][far[1]] = 1
memN._last[far[0]][far[1]] = time.time()
for i in range(20):
    memN._decay_locked(future + i * (spatial_memory.DECAY_SEC + 1))
check("a single stray return does not become a remembered cell",
      memN._hits[far[0]][far[1]] == 0, memN._hits[far[0]][far[1]])
check("...while the surface it did confirm is kept, remembered not blocking",
      memN._hits[near[0]][near[1]] == spatial_memory.MEM_FLOOR
      and not memN.blocked(0.0, 2.0))

# the map steers: a remembered wall refuses the heading into it, and a map that
# refuses everything falls back to what the live scan says
mem5 = spatial_memory.SpatialMemory()
for i in range(-7, 8):
    for j in range(6, 17):
        c = mem5._cell(i * 0.05, j * 0.05)        # a solid block 0.3-0.8 m ahead
        mem5._hits[c[0]][c[1]] = spatial_memory.HIT_MAX
drv, _ = planner(mem=mem5)
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
# The memory veto is no longer a hard refusal on its own: on the robot it fired
# while the live scan showed the corridor clear, and the robot lurched toward
# far-side gaps (+80..110 deg with the front cone clean) — the wall-adjacent
# wobble.  The map still keeps the blocked heading last on score, and still
# refuses a heading the live scan confirms blocked.
check("a remembered obstacle turns the robot away by score, not by refusal",
      dict(drv.last_scores)[0] is not None and not drv.halt
      and abs(drv.suggested_turn) > 0.0, (drv.last_decision, drv.last_scores))
check("...and the heading into it scores below the clear ones",
      dict(drv.last_scores)[0] < max(s for s in dict(drv.last_scores).values()
                                     if s is not None), drv.last_scores)
drv.stop()

mem6 = spatial_memory.SpatialMemory()
for cx in range(spatial_memory.GRID_SIZE):
    for cy in range(spatial_memory.GRID_SIZE):
        mem6._hits[cx][cy] = spatial_memory.HIT_MAX
drv, _ = planner(mem=mem6)
ticking(drv)     # no planner thread: this harness ticks it
drv._tick()
check("a map that refuses every heading does not park the robot",
      not drv.halt and "surrounded" not in drv.last_decision, drv.last_decision)
drv.stop()

# the planner is what feeds the wheels into the map (its odometry is in metres)
drv9 = SelfDriver(Base(RL(ANG, cast((0.0, 0.0, 0.0), ANG), 21.0),
                       {'odl': 0.0, 'odr': 0.0}), CV())
ticking(drv9)     # no planner thread: this harness ticks it
drv9._tick()                                   # establishes the odometer's origin
check("no travel yet -> the map has not been told to move",
      drv9.memory.pose == (0.0, 0.0, 0.0), drv9.memory.pose_status())
drv9._base.rl = RL(ANG, cast((0.0, 1.0, 0.0), ANG), 22.0)
drv9._base.base_data = {'odl': -1.0, 'odr': -1.0}   # raw counts down going forward
drv9._tick()
check("the planner hands the wheel travel and the scan stamp to the map",
      drv9.memory.motion == 'accepted' and abs(drv9.memory.pose[1] - 1.0) < 0.05,
      drv9.memory.pose_status())
check("and reports where it thinks it is", drv9.status()['map']['motion'] == 'accepted'
      and drv9.status()['map']['pose'][1] > 0.9, drv9.status()['map'])
drv9.stop()

# ...and it learns whether or not this planner is the one driving
drv10 = SelfDriver(Base(RL(ANG, cast((0.0, 0.0, 0.0), ANG), 31.0),
                        {'odl': 0.0, 'odr': 0.0}), CV())
empty_cells = drv10.memory.busy_cells
drv10._learn_only()                            # planner paused, robot driven by hand
check("a paused planner still learns the place it is in",
      drv10.memory.busy_cells > empty_cells,
      "%d -> %d" % (empty_cells, drv10.memory.busy_cells))
check("...and suggests nothing while doing it",
      drv10.suggested_turn == 0.0 and not drv10.halt)

# ...and it does that on its own thread, because /selfdrive switching self-drive
# off tears the planner thread down: the map must not stop learning then.
drv11 = SelfDriver(Base(RL(ANG, cast((0.0, 0.0, 0.0), ANG), 41.0),
                        {'odl': 0.0, 'odr': 0.0}), CV())
drv11.warm()                                   # boot: planner idle, map sampling
time.sleep(0.6)
check("the map learns at boot with self-drive off", drv11.memory.busy_cells > 0,
      drv11.memory.busy_cells)
drv11.disable()                                # what the /selfdrive route does
check("self-drive off still tears the planner down", drv11._thread is None
      and drv11._sampler is not None and drv11._sampler.is_alive())
drv11._base.rl = RL(ANG, cast((0.0, 1.0, 0.0), ANG), 42.0)
drv11._base.base_data = {'odl': -1.0, 'odr': -1.0}   # raw counts down going forward
time.sleep(0.8)
check("...and it follows the robot while a person drives it",
      drv11.memory.pose[1] > 0.9, drv11.memory.pose_status())


if FAILS:
    print("FAILED: " + ", ".join(FAILS))
    raise SystemExit(1)
print("ALL PROBES PASSED")
# which the fit guard rightly demands.
print("--- 7d. course-keeping cruise ---")
sm_pose_step = perception.pose_step             # the room scenarios move their own pose
ROOM_COURSE = (-3.0, 6.0, -3.0, 6.0)            # a room big enough to drive along +y
mem_course = spatial_memory.SpatialMemory()
pose = (0.0, 0.0, 0.0)                          # facing +y (map), robot frame 0
drv_course, _ = planner(mem=mem_course,
                        base_data={'odl': 0.0, 'odr': 0.0})
ticking(drv_course)
drv_course._base.rl = RL(ANG, cast(pose, ANG, ROOM_COURSE), 21.0)
drv_course._base.base_data = {'odl': 0.0, 'odr': 0.0}
drv_course._tick()                              # odometer origin
for i in range(1, 6):                           # drive 0.5 m along +y, through the driver
    pose = sm_pose_step(pose, 0.10, 0.10)
    drv_course._base.rl = RL(ANG, cast(pose, ANG, ROOM_COURSE),
                             21.0 + i)
    drv_course._base.base_data = {'odl': -i * 0.10, 'odr': -i * 0.10}
    drv_course._tick()
check("the course is the direction of recent travel",
      drv_course._course_map is not None and abs(drv_course._course_map - 0.0) < 12.0,
      (drv_course._course_map, mem_course.pose))
check("...and with everything clear, cruise keeps that course",
      abs(drv_course.suggested_turn) < 0.05,
      (drv_course.suggested_turn, drv_course.last_decision))
drv_course.stop()

# A pivot leaves the course alone: only translation rewrites it.  The wheels
# report a spin, the room agrees the robot only turned, and the course's map
# bearing survives — one drive after the pivot re-affirms it.
mem_turn = spatial_memory.SpatialMemory()
pose = (0.0, 0.0, 0.0)
drv_turn, _ = planner(mem=mem_turn, base_data={'odl': 0.0, 'odr': 0.0})
ticking(drv_turn)
drv_turn._base.rl = RL(ANG, cast(pose, ANG, ROOM_COURSE), 31.0)
drv_turn._base.base_data = {'odl': 0.0, 'odr': 0.0}
drv_turn._tick()
for i in range(1, 5):                           # drive 0.4 m to set a course
    pose = sm_pose_step(pose, 0.10, 0.10)
    drv_turn._base.rl = RL(ANG, cast(pose, ANG, ROOM_COURSE), 31.0 + i)
    drv_turn._base.base_data = {'odl': -i * 0.10, 'odr': -i * 0.10}
    drv_turn._tick()
before = drv_turn._course_map
spin = 0.25 * perception.TRACK_M / 2.0          # wheels for a ~29 deg pivot
pose = sm_pose_step(pose, -spin, spin)
drv_turn._base.rl = RL(ANG, cast(pose, ANG, ROOM_COURSE), 40.0)
drv_turn._base.base_data = {'odl': -4 * 0.10 + spin, 'odr': -4 * 0.10 - spin}
drv_turn._tick()                                # the pivot: no translation
check("a pivot does not rewrite the course",
      drv_turn._course_map == before and before is not None,
      (before, drv_turn._course_map, mem_turn.pose))
drv_turn.stop()

# The novelty term pulls toward unmapped floor — gently, and only at equal
# clearance: two headings equally clear, one mapped and one unknown, must not
# score the same, and neither may the term ever outrank safety.
print("--- 7e. novelty pulls toward unmapped floor ---")
# One room, one scan: the +30 deg cone sees a wall 3 m away (its floor maps
# free), the -30 deg cone returns nothing at all — no-return rays leave floor
# unmapped.  The two headings are then equally clear but not equally known,
# which is exactly the tie the novelty term exists to break.
nov_angles = [raw(d) for d in range(-180, 180)]
nov_dists = [0.0] * 360                    # unknown side: no return
for d in range(10, 61):                    # known side: wall at 3 m
    nov_dists[d + 180] = 3000.0
mem_nov = spatial_memory.SpatialMemory()
mem_nov.observe_lidar([perception.robot_angle(a) for a in nov_angles],
                      nov_dists, odom=None, scan_id=1)
check("novelty reads the mapped side as known, the other as unknown",
      mem_nov.novelty(math.radians(30)) < 0.2
      and mem_nov.novelty(math.radians(-30)) > 0.5,
      (mem_nov.novelty(math.radians(30)), mem_nov.novelty(math.radians(-30))))
drv, _ = planner(scan_pair=(nov_angles, nov_dists), mem=mem_nov)
ticking(drv)
drv._tick()
nov_scores = dict(drv.last_scores)
check("unmapped side outscores the mapped side at equal clearance",
      (nov_scores.get(-30) or -9) > (nov_scores.get(30) or -9), nov_scores)
drv.stop()

# Coverage: the floor the robot has driven over is recorded, and the count
# grows as it moves — the Roomba metric, one number.
print("--- 7f. covered floor ---")
mem_cov = spatial_memory.SpatialMemory()
check("a robot that never moved has covered nothing", mem_cov.covered_cells == 0)
pose = (0.0, 0.0, 0.0)
mem_cov.observe_lidar(ANG, cast(pose, ANG), odom=None, scan_id=1)
first = mem_cov.covered_cells
check("standing still covers the disk around the robot", first > 0, first)
for sid in range(2, 7):                          # drive half a metre
    mem_cov.observe_lidar(ANG, cast(pose, ANG), odom=(0.10, 0.10), scan_id=sid)
    pose = sm_pose_step(pose, 0.10, 0.10)
after = mem_cov.covered_cells
check("driving on covers more floor than standing still", after > first,
      (first, after))

# Battery sag: the guard parks the robot and speaks before the undervoltage
# cutoff parks the Pi.  Pure state machine in battery_guard.py; the announce
# text is the corpus's own alarm Minionese.
print("--- 8. battery sag parks and announces ---")
import battery_guard as bg

g = bg.BatteryGuard()
# a hill or a reverse burst: one soft-dip reading must not park the robot
check("a single soft dip does not park", g.update(9.7, 0.0) == 'ok', g.state)
check("...and it does not linger: a healthy reading clears it",
      g.update(10.4, 1.0) == 'ok' and g.low_sag_since is None, g.state)
check("sag below the threshold for the sustain window parks",
      g.update(9.7, 2.0) == 'ok'
      and g.update(9.7, 8.2) == 'low' and g.latched, g.state)
check("the announce is the corpus's own alarm",
      "Bee do bee do" in bg.battery_guard_announce(), bg.ANNOUNCE)
# recovery: latched means latched — one glance at a charged pack is not enough
check("latched holds through soft readings",
      all(g.update(9.9, t) == 'low' for t in (10.0, 20.0, 30.0)), g.state)
check("latched holds through a brief healthy glance",
      g.update(10.8, 31.0) == 'low', g.state)
check("a sustained rest above the threshold re-arms",
      all(g.update(10.8, t) == 'low' for t in range(32, 60))
      and g.update(10.8, 61.1) == 'ok' and not g.latched, g.state)
# the deep dip: no grace period
check("one critical reading parks at once",
      g.update(9.2, 62.0) == 'critical' and g.latched, g.state)
check("a missing voltage reading is not news",
      g.update(None, 63.0) == 'critical' and g.update(0, 64.0) == 'critical',
      g.state)

def volt(base, v):
    b = types.SimpleNamespace()
    b.base_data = {'odl': 0.0, 'odr': 0.0, 'v': v}
    return b

# the guard reads the voltage the app actually carries (the flipped sign must
# not have touched it) and refuses nothing when the pack is healthy
guard, _ = planner()
check("the planner's voltage feed is untouched by the odometry flip",
      guard._wheels.step(volt(guard._base, 11.5)) is None
      and robot_state.wheel_odometry(volt(guard._base, 11.5))['voltage'] == 11.5,
      robot_state.wheel_odometry(volt(guard._base, 11.5)))

print()
print("RESULT: %d failures" % len(FAILS))
if FAILS:
    print("FAILED: " + ", ".join(FAILS))
    raise SystemExit(1)
print("ALL PROBES PASSED")
