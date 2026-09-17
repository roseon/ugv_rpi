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
smearing a robot-centric grid suffered.  So wheel travel is *confirmed by the
scan* before it is believed, and the pose only ever moves to a place the room
agrees about.

**Why the check accumulates.**  This sensor publishes a revolution every 0.10 s
(measured: 1798 frames/s, scan stamps 0.101 s apart), so one revolution of this
chassis is 20 mm of travel at the slow speed and 35 mm at cruise.  A step that
small is inside the sensor's own repeatability, so a per-scan test cannot tell
real motion from a phantom one at any speed this robot drives: it is now judged
once the wheels claim FIT_EVERY_M of travel, where the two are 30x apart on the
robot's own scans (at the true pose the fit is 0.992; a phantom 100 mm step
scores 0.042).  Held travel is 10 cm — one cell — so the map is never marked
from a pose the room has not agreed to.

Evidence decays to MEM_FLOOR and stops there, so the place the robot has driven
through is *kept* — in memory and in surroundings.json — instead of being
forgotten within seconds of leaving the sensor's reach.  What decays away is the
right to refuse a heading: blocking is decided by MIN_HITS of *recent* evidence,
so a cell the robot saw minutes ago is remembered without being able to veto a
heading, which is what keeps a chair that has since moved from steering forever.
Measured before the floor existed: after a 5 m drive, 0 of 810 remembered cells
lay beyond the LIDAR's own 6 m reach — the map was the current view, not a map.

Only *confirmed* cells are floored.  A cell earns the floor by reaching
MIN_HITS, i.e. by being seen on three separate revolutions; a cell that has only
ever had one or two hits is not evidence of anything and decays to nothing, as
it always did.  Without that distinction the floor made every stray return
permanent: measured on the robot, 7,380 of 12,449 remembered cells had exactly
one hit, with a median distance of 5.88 m — noise at the edge of the sensor's
range, and all of it drawn on the operator's map.  Confirmed cells are the
opposite shape: 97% of the cells within 2 m of the robot had three hits or more,
so the room's real surfaces were already confirmed and the floor kept exactly
the wrong half of the map.

**The grid is a window on the frame, and the window follows the robot.**  A
bounded array cannot hold an unbounded place frame, so when the robot drives
more than CENTRE_KEEP_M from the middle of the grid the cells are shifted by a
whole number of cells — so the lattice is not resampled and every cell keeps the
place it had — and the pose moves by exactly the same amount.  What falls off
the far edge is what the LIDAR can no longer reach from here.  Without it the
robot drives out of its own map: measured live, the pose had reached cell
(152, 143) of a 0..120 grid, with 0 of 9,624 remembered cells within 3 m of the
robot and 0 of 267 live returns landing inside the grid at all, so nothing the
sensors saw could be stored or steer anything.
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
CENTRE_KEEP_M = 3.0      # the robot is kept this close to the grid's centre;
                         # beyond it the window slides under the robot (below)
MIN_HITS     = 3         # occupied evidence before a cell counts as blocked
MIN_FREE     = 3         # ray passes before a cell counts as seen-and-clear
HIT_MAX      = 5         # occupied evidence saturates here.  The count used to
                         # be capped at 65535, so a cell observed for a minute
                         # held ~400 hits and the 1-per-DECAY_SEC decay could
                         # never clear it: after a drive the grid read "blocked"
                         # in every direction and the learned map stopped
                         # contributing to steering at all.
MEM_FLOOR    = 1         # ...and decays to this and stays: the cell is
                         # remembered as seen (busy_cells, the saved file) but is
                         # under MIN_HITS, so it no longer blocks a heading.
FREE_MAX     = 5         # ...and so does free evidence
DECAY_SEC    = 6.0       # stale evidence loses a count after this long
OBJ_RANGE_MAX = 3.0      # metres — objects beyond this are remembered, not avoided
OBJ_GRID_R   = 0.18      # metres — fused disk radius for an object in the grid
OBJ_DEDUPE_M = 0.5       # metres — merge a sighting into a known object within this
OBJ_TTL_S = 30.0         # an entry no batch has refreshed in this long is dropped
                         # (a detection is a claim about now — and without expiry one
                         # drive added 20 entries for a handful of things, then the
                         # file kept reloading the ghosts into every later session)
COVER_FLOOR_M = 0.25     # radius (m) of the floor disk around the robot that a
                         # revolution marks as covered — how much of the room it
                         # has actually patrolled, the number "learning like a
                         # Roomba" is judged by
DET_MIN_CONF = 0.30      # ignore camera boxes below this confidence

# ── the frame guard ───────────────────────────────────────────────────────────
# Measured on this robot against real scans with the robot stationary (so any
# fit below the ceiling is noise): at the true pose the fit is 0.992 with
# FIT_TOL_M and 0.983 even at 15 mm, while a phantom 35 mm step scores 0.525, a
# phantom 100 mm step 0.042 and a phantom 10 cm + 5 deg turn 0.109.  The
# allowance used to be 80 mm, which put the truth at 1.000 and a phantom 35 mm
# step at 0.933 — above the acceptance threshold, so every step this chassis
# can take was believed.
FIT_TOL_M     = 0.025    # a scan point this close to remembered evidence counts
FIT_EVERY_M   = 0.10     # wheel travel to accumulate before the scan judges it
FIT_EVERY     = 3        # fit every Nth ray; 120 of ~360 is just as decisive
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
        self._sure = [bytearray(GRID_SIZE) for _ in range(GRID_SIZE)]
        self._covered = [bytearray(GRID_SIZE) for _ in range(GRID_SIZE)]
                                 # floor the robot has patrolled near (session-scoped:
                                 # a reloaded map is remembered, not visited)
                                   # this cell reached MIN_HITS once: a real
                                   # surface, so its evidence is kept (MEM_FLOOR)
                                   # instead of decaying to nothing like noise
        self.objects = []          # [{'name','bearing_deg','range_m','confidence',
                                   #   't','wx','wy'}] — the last two are where it
                                   #   stands, which is what makes two sightings
                                   #   one object
        self._det_cells = set()    # cells the last camera batch marked: an
                                   # object that moves gives them back
        self.pose = (0.0, 0.0, 0.0)   # map frame: x = left, y = forward, theta
        self.motion = 'unknown'    # accepted | rejected | unchecked | waiting
        self.motion_m = 0.0        # how far the last wheel step claimed
        self._pending = (0.0, 0.0)     # wheel travel since the last scan was fed
        self._scan_id = None           # identifies the revolution (0.10 s apart)
        self._held = (0.0, 0.0)        # wheel travel not yet judged by a scan
        self._ref_pose = None          # pose the reference scan was taken from
        self._ref_bins = None          # ...and that scan, as range by bearing
        self._lock = threading.Lock()
        self.path = path
        self.load()

    # ── grid accessors ────────────────────────────────────────────────────
    def _cell(self, wx, wy):
        cx = GRID_SIZE // 2 + int(round(wx / CELL_M))
        cy = GRID_SIZE // 2 - int(round(wy / CELL_M))   # +y forward -> up
        return max(0, min(GRID_SIZE - 1, cx)), max(0, min(GRID_SIZE - 1, cy))

    def _recentre_locked(self):
        """Slide the grid under the robot once it drifts more than CENTRE_KEEP_M.

        The shift is a whole number of cells, so nothing is resampled: a cell
        keeps the place it had, and the pose moves by exactly the same amount.
        Called with the lock held, and only from `_remember_scan` (which
        re-anchors the reference from the shifted pose) and `load`.
        """
        kx = int(round(self.pose[0] / CELL_M))
        ky = int(round(self.pose[1] / CELL_M))
        keep = int(round(CENTRE_KEEP_M / CELL_M))
        if abs(kx) <= keep and abs(ky) <= keep:
            return False

        def shifted(grid, make):
            out = [make() for _ in range(GRID_SIZE)]
            for x in range(GRID_SIZE):
                sx = x + kx
                if not 0 <= sx < GRID_SIZE:
                    continue
                src, dst = grid[sx], out[x]
                for y in range(GRID_SIZE):
                    sy = y - ky
                    if 0 <= sy < GRID_SIZE:
                        dst[y] = src[sy]
            return out

        self._hits = shifted(self._hits, lambda: [0] * GRID_SIZE)
        self._free = shifted(self._free, lambda: [0] * GRID_SIZE)
        self._last = shifted(self._last, lambda: [0.0] * GRID_SIZE)
        self._sure = shifted(self._sure, lambda: bytearray(GRID_SIZE))
        self._covered = shifted(self._covered, lambda: bytearray(GRID_SIZE))
        self.pose = (self.pose[0] - kx * CELL_M,
                     self.pose[1] - ky * CELL_M, self.pose[2])
        return True

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

    def novelty(self, heading_rad, r=0.9, half_width_rad=0.35):
        """0..1 — how much of this heading's arc is floor the map has not seen.

        Cells whose rays are fresh count as mapped, so driving past something
        stops making the way ahead look unknown (that wobble was why the first
        frontier term was removed — the fix is the freshness gate, not the
        removal).  At radius r ahead of the robot, same convention as clearance.
        """
        samples = 12
        unknown = 0
        with self._lock:
            x, y, theta = self.pose
            for i in range(samples):
                a = heading_rad + (i - samples / 2.0) / (samples / 2.0) * half_width_rad
                wx, wy = robot_to_world((x, y, theta), r * math.sin(a), r * math.cos(a))
                cx, cy = self._cell(wx, wy)
                if self._free[cx][cy] <= 0 or (self._last[cx][cy]
                                               and time.time() - self._last[cx][cy] > 10.0):
                    unknown += 1
        return unknown / float(samples)

    @property
    def covered_cells(self):
        """How many cells of floor the robot has actually patrolled near."""
        with self._lock:
            return sum(1 for row in self._covered for c in row if c)

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
                if self._hits[cx][cy] >= MIN_HITS:
                    self._sure[cx][cy] = 1     # a real surface, not one stray ray
            self._decay_locked(now)

    def _step_pose(self, angles, distances):
        """Move the pose by the wheel travel since the last scan, if the scan agrees.

        The wheels' claim is held until it is big enough to judge (FIT_EVERY_M),
        because one revolution is under the sensor's own repeatability — see the
        module docstring.  Called with the lock held, once per revolution.
        """
        claimed = self._pending
        self._pending = (0.0, 0.0)
        self.motion_m = 0.5 * (claimed[0] + claimed[1])
        if self._ref_bins is None:
            # The first scan has nothing to be checked against.
            self.pose = pose_step(self.pose, claimed[0], claimed[1])
            self.motion = 'unchecked'
            self._remember_scan(angles, distances)
            return
        if abs(claimed[0]) < 1e-4 and abs(claimed[1]) < 1e-4:
            # Claimed nothing, nothing to check.  The reference stays the room
            # as it looks now, so a scan kept from minutes ago cannot judge a
            # move made after somebody has since moved the furniture.
            self.motion = 'accepted'
            if self._held == (0.0, 0.0):
                self._remember_scan(angles, distances)
            return
        self._held = (self._held[0] + claimed[0], self._held[1] + claimed[1])
        if 0.5 * (abs(self._held[0]) + abs(self._held[1])) < FIT_EVERY_M:
            self.motion = 'waiting'           # too small for the scan to judge
            return
        candidate = pose_step(self.pose, self._held[0], self._held[1])
        fits_new = self._fit_locked(candidate, angles, distances)
        fits_ref = self._fit_locked(self.pose, angles, distances)
        self._held = (0.0, 0.0)
        if fits_new >= fits_ref - FIT_SLACK:
            self.pose = candidate
            self.motion = 'accepted'
        else:
            # The room says the robot did not move.  Drop the travel and keep
            # the map where the scan puts it; the alternative is walls that
            # walk, which is the smearing this whole frame exists to stop.
            self.motion = 'rejected'
        self._remember_scan(angles, distances)

    def _mark_covered_locked(self):
        """Record the floor the robot is standing on as covered.

        Coverage is the Roomba kind: the floor the robot itself has driven
        over, a small disk around its own pose, not the surfaces its sensor
        happens to see from afar.  Once per revolution, lock held.
        """
        x, y, theta = self.pose
        r_cells = int(math.ceil(COVER_FLOOR_M / CELL_M))
        cx0, cy0 = self._cell(x, y)
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                cx, cy = cx0 + dx, cy0 - dy
                if 0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE \
                        and math.hypot(dx * CELL_M, dy * CELL_M) <= COVER_FLOOR_M:
                    self._covered[cx][cy] = 1

    def _remember_scan(self, angles, distances):
        """Keep this scan as range-by-bearing, for the next move to be judged against."""
        self._recentre_locked()      # the window has to stay under the robot
        self._mark_covered_locked()  # every completion path of _step_pose comes here
        bins = [None] * 360
        for a, d in zip(angles, distances):
            if d <= 0 or d > 6000:
                continue
            i = int(round(math.degrees(a))) % 360
            if bins[i] is None or d < bins[i]:
                bins[i] = float(d)
        self._ref_pose, self._ref_bins = self.pose, bins

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
            mx, my = world_to_robot(self._ref_pose, wx, wy)
            was = self._ref_bins[int(round(math.degrees(math.atan2(mx, my)))) % 360]
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
                if self._hits[x][y] > (MEM_FLOOR if self._sure[x][y] else 0):
                    self._hits[x][y] -= 1
                if self._free[x][y] > 0:
                    self._free[x][y] -= 1
                self._last[x][y] = now - DECAY_SEC

    def observe_object(self, name, bearing_deg, range_m, confidence):
        """Fuse one sighting into the object memory + grid."""
        with self._lock:
            self._fuse_object_locked(name, bearing_deg, range_m, confidence,
                                     time.time())
        return len(self.objects)

    def _fuse_object_locked(self, name, bearing_deg, range_m, confidence, now):
        """The object list entry and its grid disk.  Called with the lock held.

        A sighting is merged into the object that is standing in the same place,
        not into the one at the same bearing: as the robot turns and drives, the
        same chair reappears at bearing after bearing, and matching on the
        bearing turned one chair into an entry per viewpoint — measured live, 133
        entries after a drive, for a room holding a handful of things.  Entries
        written before places were kept (or by `observe_object`, which has no box
        to range) fall back to that bearing-and-range rule.
        """
        wx, wy = robot_to_world(self.pose,
                                range_m * math.sin(math.radians(bearing_deg)),
                                range_m * math.cos(math.radians(bearing_deg)))
        for obj in self.objects:
            if obj['name'] != name:
                continue
            if obj.get('wx') is None:
                same = (abs(obj['bearing_deg'] - bearing_deg) < 15.0
                        and abs(obj['range_m'] - range_m) < OBJ_DEDUPE_M)
            else:
                same = math.hypot(obj['wx'] - wx, obj['wy'] - wy) <= OBJ_DEDUPE_M
            if same:
                obj.update(bearing_deg=bearing_deg, range_m=range_m,
                           confidence=confidence, t=now, wx=wx, wy=wy)
                break
        else:
            self.objects.append({'name': name, 'bearing_deg': bearing_deg,
                                 'range_m': range_m, 'confidence': confidence,
                                 't': now, 'wx': wx, 'wy': wy})
        if range_m <= OBJ_RANGE_MAX:
            # Same HIT_MAX ceiling as lidar hits, but objects still count
            # double so one sighting stands out.  An object deliberately does
            # not make a cell *sure*: it can walk away, and what keeps a place
            # remembered at MEM_FLOOR is the LIDAR seeing it again and again.
            for cx, cy in self._disk_cells(bearing_deg, range_m):
                self._hits[cx][cy] = min(HIT_MAX, self._hits[cx][cy] + 2)
                self._last[cx][cy] = now

    def _disk_cells(self, bearing_deg, range_m):
        """The grid cells an object at this bearing/range covers (lock held)."""
        r = max(CELL_M, OBJ_GRID_R)
        steps = 8
        out = []
        for i in range(steps):
            ang = 2 * math.pi * i / steps
            fx = range_m * math.sin(math.radians(bearing_deg)) + r * math.sin(ang)
            fy = range_m * math.cos(math.radians(bearing_deg)) + r * math.cos(ang)
            out.append(self._cell(*robot_to_world(self.pose, fx, fy)))
        return out

    def observe_detections(self, dets, front_range_m, min_conf=DET_MIN_CONF,
                           range_for=None):
        """Fuse a batch of camera detections seen from `front_range_m`.

        `range_for(bearing_deg, box)` — when the caller can offer one — is that
        detection's own distance, measured by the LIDAR at the bearing the box
        is actually at; `front_range_m` is the fallback for a detection nothing
        answers about.  The front cone used to be applied to every detection, so
        a chair 30° off to the side was remembered 0.5 m ahead of the robot and
        could refuse a heading the chair was nowhere near.

        A detection is a claim about where things are *now*, so the cells the
        previous batch marked and this one does not are released as soon as the
        batch arrives: a person who has walked on must not keep refusing the
        heading they were last seen at, and a trail of such disks would fence
        the robot in.  The grid's own decay runs at the LIDAR's 6 s timescale,
        which is far too slow for something that walks.

        The entry list obeys the same clock.  An object no batch has refreshed
        within OBJ_TTL_S is a claim nobody is making any more and is dropped,
        so the count is what the camera actually sees: without expiry one view
        left 20 entries for a handful of things and the map file reloaded the
        ghosts into every later session, where their old world positions kept
        them apart forever (four `person` entries at one bearing, live).
        """
        fresh = set()
        now = time.time()
        with self._lock:
            for det in dets or []:
                name = det.get('name', '')
                box = det.get('box')
                confidence = float(det.get('confidence', 0.0))
                if not name or not box or confidence < min_conf:
                    continue
                bearing = round(box_bearing_deg(box), 1)
                own = None if range_for is None else range_for(bearing, box)
                range_m = round(front_range_m if own is None else own, 2)
                self._fuse_object_locked(name, bearing, range_m, confidence, now)
                fresh.update(self._disk_cells(bearing, range_m))
            self._release_object_cells(fresh)
            cutoff = now - OBJ_TTL_S
            self.objects = [o for o in self.objects if o.get('t', now) >= cutoff]

    def _release_object_cells(self, fresh):
        """Forget the object evidence in cells no object claims any more.

        An object disk is a claim about where things are *now*, so the moment
        the camera stops covering a cell the claim goes with it: a person who has
        walked on must stop refusing the heading they were seen at, and a trail
        of such disks would fence the robot in.  A cell the LIDAR has confirmed
        is left alone, because that is a place and the camera only ever saw it
        behind whatever was standing there.
        """
        for cx, cy in self._det_cells - fresh:
            if self._sure[cx][cy]:
                # A place, not a claim: keep the cell, but it must stop refusing
                # headings on the strength of a thing that has gone.  The next
                # revolution or two puts its own evidence back.
                self._hits[cx][cy] = min(self._hits[cx][cy], MIN_HITS - 1)
            else:
                self._hits[cx][cy] = 0
        self._det_cells = fresh

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
                raw_sure = data.get('sure')
                if raw_sure:
                    self._sure = [bytearray(1 if v else 0 for v in row)
                                  for row in raw_sure]
                else:
                    # A file written before cells were distinguished this way:
                    # what it has confirmed is what its counts already say.
                    self._sure = [bytearray(1 if v >= MIN_HITS else 0 for v in row)
                                  for row in self._hits]
                self.objects = data.get('objects', [])
                # Entries written before objects were placed carry no wx/wy;
                # they would fail a place match forever and multiply, so the
                # first view of each of them re-places it through _fuse_object_locked.
                for obj in self.objects:
                    obj.setdefault('wx', None)
                    obj.setdefault('wy', None)
                pose = data.get('pose')
                if isinstance(pose, list) and len(pose) == 3:
                    self.pose = (float(pose[0]), float(pose[1]), float(pose[2]))
                # A file written before the window followed the robot can carry
                # a pose outside the array; bring the window under it before
                # anything reads a cell through that pose.
                self._recentre_locked()
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
                           'free': self._free,
                           'sure': [list(row) for row in self._sure],
                           'objects': self.objects, 'pose': list(self.pose)}
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
            self._sure = [bytearray(GRID_SIZE) for _ in range(GRID_SIZE)]
            self._covered = [bytearray(GRID_SIZE) for _ in range(GRID_SIZE)]
            self.objects = []
            self.pose = (0.0, 0.0, 0.0)
            self.motion, self.motion_m = 'unknown', 0.0
            self._pending = (0.0, 0.0)
            self._held = (0.0, 0.0)
            self._scan_id = None
            self._ref_pose, self._ref_bins = None, None
        print("[memory] surroundings memory cleared")
        return True
