"""spatial_memory.py — what the robot has learned about its surroundings.

Single owner of the learned map:

  * an **occupancy grid in a map frame** — a place, not the robot's current
    viewpoint.  Every LIDAR ray marks the cell it hit as occupied and the cells
    it passed through as free, so the map tells apart the three things a driver
    needs: blocked, seen-and-clear, and not seen yet;
  * a camera-object memory: each detection gets a bearing (perception) and a
    range from the front LIDAR sector — a single camera has no depth of its own
    — and close objects are fused into the grid as small occupied disks, so
    walls and objects share one avoidance model;
  * persistence, in surroundings.json next to the app.

**Where the robot is** comes from the chassis wheel odometry
(``robot_state.WheelStep``, metres per wheel, measured on this robot) turned
into a pose by ``perception.pose_step``.  The map frame is the robot frame as it
was when the memory was last cleared.

**Why the odometry is not simply believed.**  Measured here: 2.0 s commanded at
0.30 came back as 0.589 m of wheel travel while the LIDAR saw the room move less
than 10 mm — wheels turning, chassis not (lifted or slipping).  Integrating that
would walk every remembered wall 0.59 m across the map, which is exactly the
smearing a robot-centric grid suffered.  So each step is *confirmed by the scan*:
the new pose is kept only if the scan fits the map about as well as the old pose
did.  Real motion carries the scan with it and is accepted; phantom motion is
rejected and the map stays where the room says it is.

Decay is unchanged and still short-horizon (HIT_MAX/DECAY_SEC): evidence for a
cell that stops being observed fades away, so this is a map of what the robot has
seen lately, and the persisted file is a starting picture rather than forever.
"""

import json
import math
import os
import threading
import time

from perception import (box_bearing_deg, names_match, point_from_scan,
                        pose_step, robot_to_world, world_to_robot)

MEM_VERSION = 3          # bumped when the map's frame changed: v2 was
                         # robot-centric, so one wall landed in different cells
                         # on every pass, and v1 was rotated 180°.  An old file
                         # is ignored rather than driven on.

GRID_SIZE    = 121       # cells per side (±6 m at 10 cm)
CELL_M       = 0.10      # metres per cell
MIN_HITS     = 3         # occupied evidence before a cell counts as blocked
MIN_FREE     = 3         # ray passes before a cell counts as seen-and-clear
HIT_MAX      = 5         # occupied evidence saturates here.  The count used to
                         # be capped at 65535, so a cell observed for a minute
                         # held ~400 hits and the 1-per-DECAY_SEC decay could
                         # never clear it: after a drive the grid read "blocked"
                         # in every direction and the learned map stopped
                         # contributing to steering at all.
FREE_MAX     = 5         # ...and so does free evidence
DECAY_SEC    = 6.0       # stale evidence loses a count after this long
OBJ_RANGE_MAX = 3.0      # metres — objects beyond this are remembered, not avoided
OBJ_GRID_R   = 0.18      # metres — fused disk radius for an object in the grid
OBJ_DEDUPE_M = 0.5       # metres — merge a sighting into a known object within this
DET_MIN_CONF = 0.30      # ignore camera boxes below this confidence

# ── the frame guard ───────────────────────────────────────────────────────────
FIT_TOL_M     = 0.08     # a scan point this close to remembered evidence counts
FIT_EVERY     = 3        # fit every Nth ray; 120 of ~360 is just as decisive
FIT_MIN_CELLS = 40       # below this the map has nothing to contradict with
FIT_SLACK     = 0.12     # the new pose may fit this much worse than the old one
                         # and still be believed
FREE_EVERY    = 2        # mark free space every Nth ray (the occupied end of
                         # every ray is always marked); halves the cost of the
                         # pass-through marking with no loss of coverage


class SpatialMemory:
    """Occupancy grid in the map frame + camera-object memory, persisted."""

    def __init__(self, path=None):
        self._hits = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
        self._free = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
        self._last = [[0.0] * GRID_SIZE for _ in range(GRID_SIZE)]
        self.objects = []          # [{'name','bearing_deg','range_m','confidence','t'}]
        self.pose = (0.0, 0.0, 0.0)   # map frame: x = left, y = forward, theta
        self.motion = 'unknown'    # accepted | rejected | unchecked | waiting
        self.motion_m = 0.0        # how far the last wheel step claimed
        self._pending = (0.0, 0.0)     # wheel travel not yet checked against a scan
        self._scan_id = None           # the LIDAR revolves about once a second
        self._prev_pose = None         # pose when the previous scan was taken
        self._prev_bins = None         # ...and that scan, as range by bearing
        self._lock = threading.Lock()
        self.path = path
        self.load()

    # ── grid accessors ────────────────────────────────────────────────────
    def _cell(self, wx, wy):
        cx = GRID_SIZE // 2 + int(round(wx / CELL_M))
        cy = GRID_SIZE // 2 - int(round(wy / CELL_M))   # +y forward -> up
        return max(0, min(GRID_SIZE - 1, cx)), max(0, min(GRID_SIZE - 1, cy))

    def _blocked_locked(self, wx, wy, min_hits=MIN_HITS):
        cx, cy = self._cell(wx, wy)
        return self._hits[cx][cy] >= min_hits

    def blocked(self, mx, my, min_hits=MIN_HITS):
        """Is this point — robot frame, metres — remembered as occupied?

        Robot-frame in and place-frame out, through the pose: the caller asks
        about "40 cm to my left" and the map answers about the place that is.
        """
        with self._lock:
            wx, wy = robot_to_world(self.pose, mx, my)
            return self._blocked_locked(wx, wy, min_hits)

    def clearance(self, heading_rad, r, half_width_rad=0.35):
        """0..1 — fraction of a 12-sample arc at radius r that is not blocked.

        A true fraction, not a raw count: half the arc blocked reads 0.5, so
        thresholds like "don't drive into this heading" keep meaning something.
        """
        samples = 12
        blocked = 0
        with self._lock:
            x, y, theta = self.pose
            for i in range(samples):
                a = heading_rad + (i - samples / 2.0) / (samples / 2.0) * half_width_rad
                wx, wy = robot_to_world((x, y, theta), r * math.sin(a), r * math.cos(a))
                if self._blocked_locked(wx, wy):
                    blocked += 1
        return 1.0 - blocked / float(samples)

    @property
    def busy_cells(self):
        """Cells with any occupied evidence (what the UI has always shown)."""
        with self._lock:
            return sum(1 for row in self._hits for h in row if h > 0)

    @property
    def free_cells(self):
        """Cells the robot has looked through and found clear."""
        with self._lock:
            return sum(1 for row in self._free for f in row if f > 0)

    def pose_status(self):
        with self._lock:
            return {'pose': [round(v, 3) for v in self.pose],
                    'motion': self.motion,
                    'motion_m': round(self.motion_m, 3)}

    def cells(self, min_hits=1):
        """[(wx, wy, hits)] in the map frame, metres."""
        out = []
        half = GRID_SIZE // 2
        with self._lock:
            for x in range(GRID_SIZE):
                for y in range(GRID_SIZE):
                    if self._hits[x][y] >= min_hits:
                        out.append(((x - half) * CELL_M, (half - y) * CELL_M,
                                    self._hits[x][y]))
        return out

    def find_object(self, name):
        """Most recent remembered sighting of `name`, or None."""
        with self._lock:
            best = None
            for obj in self.objects:
                if names_match(obj.get('name', ''), name):
                    if best is None or obj.get('t', 0) > best.get('t', 0):
                        best = obj
            return dict(best) if best else None

    # ── learning ──────────────────────────────────────────────────────────
    def observe_lidar(self, angles, distances, odom=None, scan_id=None, range_m=6.0):
        """Feed the freshest scan (angles in the robot frame) and the wheel travel.

        `odom` is (d_left_m, d_right_m) since the previous call, or None when the
        chassis cannot say where it went.  `scan_id` identifies the revolution:
        the planner ticks faster than the LIDAR revolves, so a scan that has not
        changed is not fed in twice -- the wheel travel is collected instead and
        checked when there is something new to check it against.
        """
        now = time.time()
        fresh = scan_id is None or scan_id != self._scan_id
        with self._lock:
            self.motion, self.motion_m = 'none', 0.0
            if odom is not None:
                self._pending = (self._pending[0] + float(odom[0]),
                                 self._pending[1] + float(odom[1]))
            if not fresh:
                if self._pending != (0.0, 0.0):
                    self.motion = 'waiting'
                    self.motion_m = 0.5 * sum(self._pending)
                return
            self._scan_id = scan_id
            self._step_pose(angles, distances)
            x, y, theta = self.pose
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            for index, (a, d) in enumerate(zip(angles, distances)):
                if d <= 0 or d > range_m * 1000.0:
                    continue
                mx, my = point_from_scan(a, d)
                if index % FREE_EVERY == 0:
                    # The ray proves every cell along it is clear; its own end is
                    # the obstacle.  Every cell, not every other one: a striped
                    # corridor reads as half-unknown forever, which is what the
                    # frontier term and the clearance both act on.
                    cells_to_hit = max(1, int(d / 1000.0 / CELL_M))
                    for i in range(1, cells_to_hit):
                        f = i / float(cells_to_hit)
                        fx, fy = mx * f, my * f
                        cx, cy = self._cell(x + fx * cos_t + fy * sin_t,
                                            y - fx * sin_t + fy * cos_t)
                        if self._free[cx][cy] < FREE_MAX:
                            self._free[cx][cy] += 1
                        self._last[cx][cy] = now
                cx, cy = self._cell(x + mx * cos_t + my * sin_t,
                                    y - mx * sin_t + my * cos_t)
                if self._hits[cx][cy] < HIT_MAX:
                    self._hits[cx][cy] += 1
                self._last[cx][cy] = now
            self._decay_locked(now)

    def _step_pose(self, angles, distances):
        """Move the pose by the wheel travel since the last scan, if the scan agrees.

        Called with the lock held, once per revolution.
        """
        claimed = self._pending
        self._pending = (0.0, 0.0)
        self.motion_m = 0.5 * (claimed[0] + claimed[1])
        if self._prev_bins is None:
            # The first scan has nothing to be checked against.
            self.pose = pose_step(self.pose, claimed[0], claimed[1])
            self.motion = 'unchecked'
            self._remember_scan(angles, distances)
            return
        if abs(claimed[0]) < 1e-4 and abs(claimed[1]) < 1e-4:
            self.motion = 'accepted'          # claimed nothing, nothing to check
            self._remember_scan(angles, distances)
            return
        candidate = pose_step(self.pose, claimed[0], claimed[1])
        fits_new = self._fit_locked(candidate, angles, distances)
        fits_now = self._fit_locked(self.pose, angles, distances)
        if fits_new >= fits_now - FIT_SLACK:
            self.pose = candidate
            self.motion = 'accepted'
        else:
            # The room says the robot did not move.  Drop the step and keep the
            # map where the scan puts it; the alternative is walls that walk.
            self.motion = 'rejected'
        self._remember_scan(angles, distances)

    def _remember_scan(self, angles, distances):
        """Keep this scan as range-by-bearing, for the next step to be checked against."""
        bins = [None] * 360
        for a, d in zip(angles, distances):
            if d <= 0 or d > 6000:
                continue
            i = int(round(math.degrees(a))) % 360
            if bins[i] is None or d < bins[i]:
                bins[i] = float(d)
        self._prev_pose, self._prev_bins = self.pose, bins

    def _fit_locked(self, pose, angles, distances, tol_m=FIT_TOL_M):
        """How well this scan, seen from `pose`, reproduces the previous scan.

        The check is against the *previous scan*, not against the accumulated
        map, and that distinction is the whole guard: the map holds stale walls,
        and stale walls agree with "the robot never moved" no matter what the
        robot does, so a map-based fit would reject every real motion.  The
        previous scan is the room as it looked one revolution ago, so only a
        pose the robot is actually at can reproduce it.
        """
        hits = total = 0
        for a, d in zip(angles[::FIT_EVERY], distances[::FIT_EVERY]):
            if d <= 0 or d > 6000:
                continue
            mx, my = point_from_scan(a, d)
            wx, wy = robot_to_world(pose, mx, my)
            mx, my = world_to_robot(self._prev_pose, wx, wy)
            was = self._prev_bins[int(round(math.degrees(math.atan2(mx, my)))) % 360]
            if was is None:
                continue
            total += 1
            if abs(math.hypot(mx, my) * 1000.0 - was) <= tol_m * 1000.0:
                hits += 1
        return hits / float(total) if total else 0.0

    def _decay_locked(self, now):
        for x in range(GRID_SIZE):
            for y in range(GRID_SIZE):
                if now - self._last[x][y] <= DECAY_SEC:
                    continue
                if self._hits[x][y] > 0:
                    self._hits[x][y] -= 1
                if self._free[x][y] > 0:
                    self._free[x][y] -= 1
                self._last[x][y] = now - DECAY_SEC

    def observe_object(self, name, bearing_deg, range_m, confidence):
        """Fuse one sighting into the object memory + grid."""
        now = time.time()
        with self._lock:
            for obj in self.objects:
                if obj['name'] == name and abs(obj['bearing_deg'] - bearing_deg) < 15.0 \
                        and abs(obj['range_m'] - range_m) < OBJ_DEDUPE_M:
                    obj.update(bearing_deg=bearing_deg, range_m=range_m,
                               confidence=confidence, t=now)
                    break
            else:
                self.objects.append({'name': name, 'bearing_deg': bearing_deg,
                                     'range_m': range_m, 'confidence': confidence,
                                     't': now})
            if range_m <= OBJ_RANGE_MAX:
                r = max(CELL_M, OBJ_GRID_R)
                steps = 8
                for i in range(steps):
                    ang = 2 * math.pi * i / steps
                    fx = range_m * math.sin(math.radians(bearing_deg)) + r * math.sin(ang)
                    fy = range_m * math.cos(math.radians(bearing_deg)) + r * math.cos(ang)
                    wx, wy = robot_to_world(self.pose, fx, fy)
                    cx, cy = self._cell(wx, wy)
                    # Same HIT_MAX ceiling as lidar hits, but objects still count
                    # double so one sighting stands out.
                    self._hits[cx][cy] = min(HIT_MAX, self._hits[cx][cy] + 2)
                    self._last[cx][cy] = now
        return len(self.objects)

    def observe_detections(self, dets, front_range_m, min_conf=DET_MIN_CONF):
        """Fuse a batch of camera detections seen from `front_range_m`."""
        for det in dets or []:
            name = det.get('name', '')
            box = det.get('box')
            confidence = float(det.get('confidence', 0.0))
            if not name or not box or confidence < min_conf:
                continue
            bearing = box_bearing_deg(box)
            self.observe_object(name, round(bearing, 1), round(front_range_m, 2),
                                confidence)

    # ── persistence ───────────────────────────────────────────────────────
    def load(self):
        if not self.path or not os.path.exists(self.path):
            return False
        try:
            with open(self.path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
            if data.get('version') != MEM_VERSION:
                print("[memory] surroundings memory predates the place-referenced "
                      "map — relearning from scratch")
                return False
            with self._lock:
                raw = data.get('grid')
                if raw:
                    # Clamp on load too: a file written before HIT_MAX existed
                    # carries counts in the hundreds and would otherwise stay
                    # saturated for as long as it takes to decay them.
                    self._hits = [[min(int(v), HIT_MAX) for v in row]
                                  for row in raw]
                raw_free = data.get('free')
                if raw_free:
                    self._free = [[min(int(v), FREE_MAX) for v in row]
                                  for row in raw_free]
                self.objects = data.get('objects', [])
                pose = data.get('pose')
                if isinstance(pose, list) and len(pose) == 3:
                    self.pose = (float(pose[0]), float(pose[1]), float(pose[2]))
            print("[memory] loaded surroundings memory: "
                  f"{self.busy_cells} busy cells, {self.free_cells} known clear, "
                  f"{len(self.objects)} objects, pose {self.pose_status()['pose']}")
            return True
        except Exception as e:
            print(f"[memory] failed to load {self.path}: {e}")
            return False

    # Back-compat alias: older call sites used the private name.
    _load = load

    def save(self):
        if not self.path:
            return False
        try:
            with self._lock:
                payload = {'version': MEM_VERSION, 'grid': self._hits,
                           'free': self._free, 'objects': self.objects,
                           'pose': list(self.pose)}
            tmp = self.path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(payload, fh)
            os.replace(tmp, self.path)
            print(f"[memory] saved surroundings memory: "
                  f"{self.busy_cells} busy cells, {len(self.objects)} objects")
            return True
        except Exception as e:
            print(f"[memory] save failed: {e}")
            return False

    def clear(self):
        with self._lock:
            self._hits = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
            self._free = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
            self._last = [[0.0] * GRID_SIZE for _ in range(GRID_SIZE)]
            self.objects = []
            self.pose = (0.0, 0.0, 0.0)
            self.motion, self.motion_m = 'unknown', 0.0
            self._pending = (0.0, 0.0)
            self._scan_id = None
            self._prev_pose, self._prev_bins = None, None
        print("[memory] surroundings memory cleared")
        return True
