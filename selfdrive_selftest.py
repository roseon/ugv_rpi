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
    def __init__(self, angles, dists, stamp=None):
        self.lidar_angles_show = angles
        self.lidar_distances_show = dists
        self.lidar_scan_time = stamp   # moves once per revolution, like the real one


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
                  'target', 'map'},
      sorted(st))
check("wheels read from the ESP32 frame", st['wheels'] ==
      {'odl': -10398.7, 'odr': -9916.4, 'voltage': 11.48})
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
check("an unobserved object disk stops blocking, keeping only the memory of it",
      peak_after == spatial_memory.MEM_FLOOR and not ocs.blocked(0.0, 1.0),
      peak_after)

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
memF = spatial_memory.SpatialMemory()
for sid in (401, 402):
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
check("a remembered obstacle refuses the heading into it and turns instead",
      dict(drv.last_scores)[0] is None and not drv.halt
      and drv.suggested_turn != 0.0, drv.last_decision)
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
drv9._base.base_data = {'odl': 1.0, 'odr': 1.0}
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
drv11._base.base_data = {'odl': 1.0, 'odr': 1.0}   # driven by hand, off self-drive
time.sleep(0.8)
check("...and it follows the robot while a person drives it",
      drv11.memory.pose[1] > 0.9, drv11.memory.pose_status())

print()
print("RESULT: %d failures" % len(FAILS))
if FAILS:
    print("FAILED: " + ", ".join(FAILS))
    raise SystemExit(1)
print("ALL PROBES PASSED")
