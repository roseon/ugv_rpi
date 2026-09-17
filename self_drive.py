"""self_drive.py — the self-drive planner thread.

This module owns the thread lifecycle (start/stop/pause/resume and the
composite transitions enable/disable/pursue/warm), one planning tick, the
heading choice, and the status payload. Everything it needs has a module of
its own:

    perception.py        robot-frame conventions + pure scan helpers
    robot_state.py       live LIDAR scan + ESP32 wheel odometry
    spatial_memory.py    learned occupancy grid + object memory (surroundings.json)
    detection_source.py  live detections + throttled open-vocabulary top-up
    target_pursuit.py    what object is being chased + the pursuit rules

Where a change lands
    a new sensor reading        -> robot_state.py
    an angle/bearing convention -> perception.py (the only place either lives)
    what the robot remembers    -> spatial_memory.py
    which detections it sees    -> detection_source.py
    the target's state/rules    -> target_pursuit.py
    the heading that gets picked, the veto, the executor contract
                                -> SelfDriver._choose_heading / _tick (here)

The planner never writes wheel commands. It publishes suggested_turn (-1..1,
+ = left) and halt; app.LidarAvoider remains the executor and its emergency
states (SLOW/EVADE/REVERSE) stay authoritative, as does this module's refusal
to drive forward with something inside TOO_CLOSE_MM.

A surfaced detail: pending-halt. When the object is reached the planner sets
halt so the executor stops; that is the only way it can stop the robot, and
the avoider is free to override it (which is what safety wants).
"""

import math
import threading
import time

from detection_source import DetectionSource
from perception import arc_clearance, box_span_deg, side_wall
from robot_state import (LidarScan, WheelStep, dense_sector_min_mm,
                         wheel_odometry)
from spatial_memory import MEM_VERSION, OBJ_RANGE_MAX, SpatialMemory   # re-exported
from target_pursuit import PursuitPolicy, Target

# ── planner tunables ──────────────────────────────────────────────────────────
HEADINGS      = [0, 15, -15, 30, -30, 45, -45, 60, -60, 80, -80, 110, -110]
LOOP_HZ       = 7.0
SAVE_INTERVAL = 60.0        # seconds between auto-saves while active
TOO_CLOSE_MM  = 250         # inside this the planner refuses to drive forward
FRONT_CONE_DEG = 45         # safety cone for TOO_CLOSE_MM
OBJECT_CONE_DEG = 35       # front sector a detection is ranged with for memory
OBJECT_HALF_MIN_DEG = 5    # ...and how wide the look at a detection's own
OBJECT_HALF_MAX_DEG = 20   #    bearing is, at least and at most (see
                           #    _detection_range: the sensor samples ~2°)
                           # (the *arrival* range follows the object's own
                           # bearing — see target_pursuit.TARGET_CONE_DEG)
VETO_HALF_WIDTH = 0.4      # rad — live arc sampled per candidate heading

# heading score weights (mirror the desktop AutoPilot, plus object avoidance)
W_LIDAR = 2.0
W_MEM   = 1.2
W_GOAL  = 0.6
W_NOVEL = 0.5             # gentle pull toward unmapped floor: enough to break a
                          # tie between two clear headings, never enough to beat
                          # a heading that is actually safer (clearance weighs 2.0)
MEM_RADII = (0.55, 1.1)  # metres — remembered occupancy is sampled near and far
MEM_BLOCK_CLEAR = 0.34   # below this fraction of a near arc the heading is refused

# Roomba-style course keeping: the cruise goal is no longer "whatever is
# straight ahead of the bumper" but "the direction I have been driving".
# Without it every local block swung the robot toward the widest gap anywhere
# in the room — headings of +-80 and even +-110 deg chosen while cruising,
# pirouetting in place instead of covering floor.  With it the robot turns the
# minimum a block demands, then carries on along its old line.
COURSE_HOLD_S = 6.0       # a course older than this is stale and re-established
COURSE_BIAS   = 1.8       # weight of the held course inside the goal term.
                          # 1.0 let the memory's early cornering win near walls
                          # (mean |turn| 0.359 vs 0.194 in the open); 1.8 makes
                          # leaving the line cost double, so only a clearly
                          # safer or clearly freer heading outbids it
COURSE_MAX_DEG = 130.0    # beyond this from the held course the bias falls off
                          # (90 made a 45-deg detour lose half its course pull;
                          # 130 keeps mid-angle headings partially anchored)

# ── wall following (how it stops swivelling beside a wall) ───────────────────
# Beside a wall the clearance-maximising score and the held course fight every
# tick: clearance says the heading angled off the wall is always safer, the
# course says come back, and the winner alternates — the left/right swivel the
# wall-adjacent probes kept measuring (mean |turn| 0.359 vs 0.194 in the open).
# A Roomba does neither: it picks a standoff and holds a line parallel to the
# wall.  So when a side wall is in reach, the wall regulator SUPPLIES the
# course anchor — "parallel to the wall, at the standoff" — and all the
# damping already built (COURSE_BIAS, EMA, slew, deadband) shapes the output.
WALL_RANGE_M   = 0.85     # standoff the follower holds from the wall
WALL_TILT_GAIN = 1.2      # metres of standoff error per radian of tilt
                          # (0.45 commanded a 35 deg lurch for 0.28 m of
                          # error — a lurch is what this replaces; 1.2 keeps
                          # the whole working range under ~20 deg)
WALL_TILT_MAX_RAD = math.radians(20)   # cap the parallel-line tilt
WALL_HOLD_S    = 1.2      # keep following this side this long without a reading
                          # (sensor holes at a gap shouldn't drop the line
                          # every second tick)
WALL_GOAL_MULT = 2.5      # the wall anchor's authority in the score: W_MEM and
                          # W_LIDAR both prefer headings angled OFF a wall in
                          # reach, which at the plain goal weight let the robot
                          # drift out of reach instead of reeling back in.
                          # The multiplier lets gentle alignment win, while a
                          # genuinely blocked heading still loses: scan AND
                          # memory then agree against it and outweigh it.

# ── what the learned map is for ──────────────────────────────────────────────
# The grid is place-referenced now, which is what makes it worth driving on: a
# wall or a chair the robot passed a second ago is still remembered *there*
# when the scan has swung away from it, so a heading into remembered occupancy
# is scored down — and refused outright when the near arc is solid — and the
# robot goes around what it already knows about instead of re-finding it.
#
# It reads evidence that DECAY_SEC expires, so a chair that has moved stops
# steering within a few seconds of the scan no longer seeing it.
#
# A frontier term ("prefer the nearest unmapped space") was built and then
# removed: with any weight that made it matter it outranked the straight-ahead
# goal preference in an open room and took the robot off the proven straight
# line (the cruise probes below caught it choosing +45° in a clear room), which
# is exploration — a behaviour nobody asked for — at the cost of one that was
# proven live.

# ── steering smoothing (what stops the left/right wobble) ─────────────────────
# HEADINGS is a discrete set and the score gap between neighbours is routinely
# smaller than the scan-to-scan noise, so the winner flipped between e.g. +0 and
# -15 every tick and `best / 90.0` answered each flip with a full differential.
# The robot wove down a corridor that was straight ahead. Three guards, in
# order, each covering a failure the one before it cannot:
#
#   HEADING_SMOOTHING   an exponential moving average of the chosen heading.
#                       Symmetric flicker averages out to straight, while a real
#                       obstacle wins by a large, sustained margin and still
#                       turns the robot within about a second. (A
#                       hold-until-beaten-by-margin rule was tried first and
#                       rejected: in a clear room "straight" only beats a held
#                       30 deg by 0.10, so the robot kept its old heading and
#                       never straightened out at all.)
#   TURN_DEADBAND       snap a residual turn to zero, so coming out of a turn
#                       ends on exactly-equal wheel commands rather than a
#                       permanent fraction of a turn.
#   TURN_SLEW_PER_TICK  hard rate limit on suggested_turn, so no single tick can
#                       ask the wheels for a full lock.
HEADING_SMOOTHING  = 0.35
TURN_DEADBAND      = 0.06
TURN_SLEW_PER_TICK = 0.08


class SelfDriver:
    """Planner thread: learns surroundings, picks a cruise or pursuit heading."""

    def __init__(self, base, cvf, memory=None, detections=None, policy=None):
        self._base = base
        self._cvf = cvf
        self.memory = memory if memory is not None else SpatialMemory()
        self.detections = detections if detections is not None \
            else DetectionSource(cvf)
        self.policy = policy if policy is not None else PursuitPolicy()
        self.target = Target()

        self._active = False
        self._stop_evt = threading.Event()
        self._thread = None
        self._last_save = time.time()
        self._wheels = WheelStep()   # wheel travel since the last planning tick

        self.suggested_turn = 0.0   # -1..1, + = left (avoider convention)
        self.halt = False           # executor hint: stop the wheels now
        self.last_scores = []       # [(deg, score)] for the UI
        self.last_decision = "off"
        self._heading_smooth = 0.0  # EMA of the chosen heading, in degrees
        self._course_map = None     # map-frame bearing of recent actual travel
        self._wall_side = 0         # which side is being followed: -1, 0, +1
        self._wall_until = 0.0      # monotonic time the wall hold expires
        self._course_t = 0.0        # ...and when that was last confirmed
        self._last_course_xy = None # the pose the course was last read from
        self._learned = False       # something new since the last save
        self._sampler = None        # the thread that keeps the map current
        self._sampler_stop = threading.Event()

    # ── lifecycle (the only way the thread and its flags change) ──────────
    def warm(self):
        """Boot state: the planner exists but stays idle, and the map samples.

        Started *inactive*: this used to start it active and pause it from the
        calling thread, so the loop could get one planning tick in before the
        pause landed and boot would briefly decide a heading nobody asked for.
        """
        self.start(active=False)
        self.pause(save=False)
        self._start_sampler()

    def start(self, active=True):
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._active = active
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="self-drive")
        self._thread.start()

    # ── the map's own sampler ─────────────────────────────────────────────
    # Learning is deliberately not the planner's job.  ``disable()`` tears the
    # planner thread down (what the /selfdrive route does when self-drive is
    # switched off), and the learned map is what the robot knows about the
    # place, so it has to keep growing while the robot is driven by hand or
    # sitting still.  Before this, turning self-drive off stopped the memory
    # dead, and a hand-driven robot taught it nothing.
    def _start_sampler(self):
        if self._sampler and self._sampler.is_alive():
            return
        self._sampler_stop.clear()
        self._sampler = threading.Thread(target=self._sample_loop, daemon=True,
                                         name="self-drive-map")
        self._sampler.start()

    def _sample_loop(self):
        interval = 1.0 / LOOP_HZ
        while not self._sampler_stop.is_set():
            t0 = time.time()
            if not self._active:          # the planner tick learns while it runs
                try:
                    self._learn_only()
                except Exception as e:
                    print(f"[memory] sample error: {e}")
            if self._learned and time.time() - self._last_save > SAVE_INTERVAL:
                self.memory.save()
                self._last_save = time.time()
                self._learned = False
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def stop(self):
        self._active = False
        self.suggested_turn = 0.0
        self.halt = False
        self.last_decision = "off"
        self._heading_smooth = 0.0
        self._course_map = None
        self._wall_side = 0
        self._wall_until = 0.0
        self._stop_evt.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None

    def pause(self, save=True):
        self._active = False
        self.suggested_turn = 0.0
        self.halt = False
        self.last_decision = "paused"
        self._heading_smooth = 0.0
        self._course_map = None
        self._wall_side = 0
        self._wall_until = 0.0
        if save:
            self.memory.save()

    def resume(self):
        # A resume after stop() must bring the thread back: the /selfdrive
        # routes and app.py's watchdog both call this on a planner that may
        # have been stopped, and without it the planner would stay dead while
        # claiming to be active.
        if not (self._thread and self._thread.is_alive()):
            self.start()
        self._active = True
        self.halt = False
        self.last_decision = (f"looking for {self.target.name}" if self.target.name
                              else "planning")

    def enable(self):
        """Turn driving on (keeps any current target)."""
        self.resume()

    def disable(self, save=True):
        """Turn driving off and drop the chase: the composite every caller wants."""
        self.pause(save=save)
        self.target.clear()
        self.stop()

    def pursue(self, name):
        """Drive toward one named object; False if the name is blank."""
        if not self.target.set(name, memory=self.memory):
            return False
        self._heading_smooth = 0.0  # a chase steers by bearing, not by this
        self.enable()
        return True

    def clear_target(self):
        self.target.clear()
        self._heading_smooth = 0.0

    # ── memory shortcuts (the map's owner is SpatialMemory) ───────────────
    def save_memory(self):
        return self.memory.save()

    def load_memory(self):
        return self.memory.load()

    def clear_memory(self):
        cleared = self.memory.clear()
        # The pose starts again from the robot's own frame, so the odometer has
        # to start again from the counters as they are now.
        self._wheels.reset()
        return cleared

    @property
    def active(self):
        return self._active

    # ── reporting ─────────────────────────────────────────────────────────
    def status(self):
        scan = LidarScan.read(self._base)
        return {
            'active': self._active,
            'suggested_turn': round(self.suggested_turn, 3),
            'front_mm': scan.front_min_mm(FRONT_CONE_DEG),
            'halt': self.halt,
            'wheels': wheel_odometry(self._base),
            'decision': self.last_decision,
            'busy_cells': self.memory.busy_cells,
            'free_cells': self.memory.free_cells,
            'objects': self.memory.objects[-20:][::-1],
            'scores': self.last_scores,
            'target': self.target.status(),
            'map': self.memory.pose_status(),
            'course_deg': (None if self._course_map is None
                           else round(self._course_map, 1)),
            'wall_side': self._wall_side,   # 0 none, +1 left, -1 right
            'covered_cells': self.memory.covered_cells,
        }

    # ── planning ──────────────────────────────────────────────────────────
    def _loop(self):
        interval = 1.0 / LOOP_HZ
        while not self._stop_evt.is_set():
            t0 = time.time()
            if self._active:
                try:
                    self._tick()
                except Exception as e:
                    print(f"[planner] tick error: {e}")
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def _detection_range(self, scan):
        """Where each detection really is: the LIDAR returns at its own bearing.

        A camera box says where a thing is *sideways*; only the LIDAR says how
        far it is.  One front cone was applied to every detection, so an object
        30° off to the side was stored at whatever was nearest straight ahead —
        live, one `oven` was held twice at the same bearing 0.46 m and 1.7 m
        apart, and a view of a few things produced 20 entries.

        The distance comes from the 1-degree occupancy bins rather than from the
        one revolution `scan` holds: that revolution arrives in patches on this
        kit's wire, and a hole in it answers with the room *behind* the thing
        (measured live, objects landed up to 2.5 m beyond the sensor's own return
        at their bearing).  The look at a detection's bearing is as wide as the
        box is, so a small box still finds cells, and is clamped so a huge box
        does not claim half the room.  Nothing fresh in that sector means the
        front cone stays the honest fallback: the object is still recorded, just
        not at a distance nothing measured.
        """
        def range_for(bearing_deg, box):
            half_deg = max(OBJECT_HALF_MIN_DEG,
                           min(OBJECT_HALF_MAX_DEG, box_span_deg(box) / 2.0))
            mm = dense_sector_min_mm(self._base, bearing_deg, half_deg)
            if not mm:
                return None
            return max(0.2, min(mm / 1000.0, OBJ_RANGE_MAX))
        return range_for

    def _learn(self, scan):
        """Fold this scan (and the camera's view of it) into the learned map.

        The wheel travel is read every tick and collected until a *new*
        revolution is there to check it against; the map only moves its pose
        when the scan agrees the robot really went there.
        """
        self.memory.observe_lidar(scan.angles, scan.distances,
                                  odom=self._wheels.step(self._base),
                                  scan_id=scan.stamp)
        dets = self.detections.for_target(self.target.name)
        self.memory.observe_detections(dets, self._object_range_m(scan),
                                       range_for=self._detection_range(scan))
        self._learned = True
        self._course_from_pose()
        return dets

    def _course_from_pose(self):
        """The held course: the bearing of recent actual travel, or None.

        Read from the map's pose (x left, y forward), not from wheel commands,
        so a turn-in-place leaves it untouched and a block that turns the robot
        does not lose the line it was driving.  Goes stale after
        COURSE_HOLD_S of no travel so a pick-up-and-carry starts fresh.
        """
        pose = self.memory.pose_status().get('pose')
        if not pose:
            self._course_map = None
            return
        x, y, theta = pose[0], pose[1], pose[2]
        if self._last_course_xy is not None:
            dx, dy = x - self._last_course_xy[0], y - self._last_course_xy[1]
            if math.hypot(dx, dy) >= 0.05:
                # Map-frame bearing of the actual travel.  The robot frame it
                # is steered in rotates with every pivot, so the conversion to
                # a robot-frame heading happens at scoring time, not here.
                self._course_map = math.degrees(math.atan2(dx, dy))
                self._course_t = time.time()
            elif time.time() - self._course_t > COURSE_HOLD_S:
                self._course_map = None
        self._last_course_xy = (x, y)

    def _learn_only(self):
        """Idle: keep learning the place from the LIDAR anyway.

        The map is what the robot knows about where it is, so it has to grow
        from however the robot got there — driven by hand from the UI, or
        pushed — not only while this planner is the one holding the sticks.
        Nothing is decided here: no heading, no halt, no wheel suggestion.
        """
        scan = LidarScan.read(self._base)
        if not scan.empty:
            self._learn(scan)
        self._course_from_pose()

    def _tick(self):
        """One planning step: read, learn, then decide (safety first)."""
        scan = LidarScan.read(self._base)
        if scan.empty:
            # No scan means no cruise: the avoider goes IDLE without angles, so
            # nothing needs stopping here — just refuse to suggest a heading.
            self._hold("no lidar data", stop=False)
            return
        dets = self._learn(scan)

        front = scan.front_min_mm(FRONT_CONE_DEG)
        if front is not None and front < TOO_CLOSE_MM:
            # The avoider's emergency states own anything this close, and the
            # planner also refuses to drive forward, so a CRUISE state can
            # never push the robot into a <250 mm obstacle.
            self._hold(f"too close {front/1000:.2f}m — avoider owns it")
            return

        self.target.observe(dets, scan)
        if self.policy.arrival_hold(self.target):
            self.target.mark_arrived()
            self._hold(f"arrived at {self.target.name} "
                       f"({self.target.range_m:.2f}m) — stopped")
            return

        self._choose_heading(scan)

    def _slew(self, aim):
        """Move suggested_turn toward `aim` by at most one slew step."""
        step = aim - self.suggested_turn
        step = max(-TURN_SLEW_PER_TICK, min(TURN_SLEW_PER_TICK, step))
        return round(self.suggested_turn + step, 4)

    def _wall_course(self, scan):
        """Map-frame bearing of the wall-parallel line, or None with no wall.

        A P-regulator on the standoff: too close bears away, too far bears in,
        and a nose angling toward the wall adds its closing component, so the
        line is caught and levelled instead of overshooting.  The side is held
        WALL_HOLD_S past the last reading so a gap in the returns does not drop
        the line every other tick; once expired, either side may pick it up.
        """
        now = time.monotonic()
        if self._wall_side != 0 and now >= self._wall_until:
            self._wall_side = 0
        if self._wall_side == 0:
            left = side_wall(scan.angles, scan.distances, +1)
            right = side_wall(scan.angles, scan.distances, -1)
            if left is None and right is None:
                return None
            if right is None or (left is not None and left[0] <= right[0]):
                self._wall_side = +1      # follow the nearer wall
            else:
                self._wall_side = -1
        reading = side_wall(scan.angles, scan.distances, self._wall_side)
        if reading is not None:
            self._wall_until = now + WALL_HOLD_S
            standoff, offset = reading
        else:
            standoff, offset = WALL_RANGE_M, 0.0   # gap: hold the line level
        # Positive err_m means the robot must move TOWARD the wall's standoff
        # (too far, or the nose is opening away); negative means it is pushing
        # at the wall (too close, or the nose is closing on it).  The nose term
        # converts the toward-wall bearing offset into metres at the current
        # standoff: offset's sign is toward-wall only after the side flip
        # (left wall nose-in reads offset<0, right wall offset>0), so the
        # product is side-symmetric.  The response multiplies by the side
        # again: +tilt steers left, which is away from a right wall and into
        # a left one — six geometric cases checked (close/far/level-nose,
        # each side).
        err_m = (standoff - WALL_RANGE_M) \
            + 0.5 * self._wall_side * offset * standoff
        tilt = max(-WALL_TILT_MAX_RAD,
                   min(WALL_TILT_MAX_RAD,
                       self._wall_side * err_m / WALL_TILT_GAIN))
        return math.degrees(self.memory.pose[2] + tilt) % 360.0

    def _choose_heading(self, scan):
        """Pick the heading to suggest — the one place heading policy lives."""
        target, policy = self.target, self.policy
        pursuing = target.name is not None
        bearing = target.aim_bearing          # None when not in view
        self.halt = False

        best, best_score = 0.0, float('-inf')
        # The wall anchor is read once per tick (it mutates hold state), then
        # captured read-only by the scorer below.
        wall_course = self._wall_course(scan)
        wall_mult = WALL_GOAL_MULT if wall_course is not None else 1.0

        def score_heading(heading, map_veto):
            """This heading's score, or None when something in the way refuses it."""
            heading_rad = math.radians(heading)
            lidar_clear = arc_clearance(scan.angles, scan.distances, heading_rad,
                                        VETO_HALF_WIDTH)
            if pursuing and lidar_clear < policy.veto_clear:
                return None                      # live obstacle in the way
            near = self.memory.clearance(heading_rad, MEM_RADII[0])
            if map_veto and near < MEM_BLOCK_CLEAR \
                    and lidar_clear < policy.veto_clear:
                # Remembered obstacle in the way — but only when the live scan
                # also refuses the heading.  On its own, the veto made the robot
                # lurch toward far-side gaps it did not need (+80..110 deg wins
                # with the front cone clean): the memory's view of a wall-edge
                # outranked a live-clear corridor, and the wobble the EMA and
                # slew were built to smooth was the score flip that followed.
                # Measured live: mean |turn| near walls 0.359 vs 0.194 in the
                # open, all wide turns with the front cone clear.
                return None
            mem_clear = min(near, self.memory.clearance(heading_rad, MEM_RADII[1]))
            goal_term = policy.goal_term(heading, bearing)
            if goal_term is None:
                # Cruise: keep the course the robot has been driving (Roomba's
                # minimum-turn rule), pulled gently toward unmapped floor so
                # open room gets covered instead of patrolled in circles.  The
                # course is a map-frame bearing; it is re-expressed against the
                # robot's current heading here, so a pivot rotates what "back
                # on course" means instead of freezing it.  When a side wall
                # is in reach the wall-parallel line REPLACES the driven
                # course as the anchor — re-derived from the live scan every
                # tick, so odometry drift never bends it.  Its goal weight is
                # raised (wall_mult): the plain weight let W_LIDAR and W_MEM's
                # preference for headings angled off the wall win and the
                # robot drift out of the wall's reach instead of reeling back
                # to its standoff.
                course = wall_course
                if course is None:
                    course = self._course_map
                if course is not None:
                    course = (course - math.degrees(self.memory.pose[2])
                              + 180.0) % 360.0 - 180.0
                novel = self.memory.novelty(heading_rad)
                if course is None:
                    goal, weight = (-abs(heading) / 180.0
                                    + W_NOVEL / W_GOAL * novel, W_GOAL)
                else:
                    off = abs((heading - course + 180.0) % 360.0 - 180.0)
                    keep = 1.0 - off / COURSE_MAX_DEG
                    goal, weight = (max(-1.0, keep * COURSE_BIAS
                                        - abs(heading) / 180.0 * 0.2)
                                    + W_NOVEL / W_GOAL * novel, W_GOAL)
            else:
                goal, weight = goal_term
            return (W_LIDAR * lidar_clear + W_MEM * mem_clear
                    + wall_mult * weight * goal)

        scores = [(h, score_heading(h, True)) for h in HEADINGS]
        if not pursuing and all(s is None for _, s in scores):
            # Every heading refused is the remembered map's own mistake — a bad
            # pose, or a chair that has since moved.  Driving on what the live
            # scan says beats parking on what the map says.
            scores = [(h, score_heading(h, False)) for h in HEADINGS]
        self.last_scores = [(h, None if s is None else round(s, 3)) for h, s in scores]
        for heading, score in scores:
            if score is not None and score > best_score:
                best_score, best = score, float(heading)

        if best_score == float('-inf'):
            # Every heading is blocked right now — hold still and let the
            # avoider's emergency states work it out.
            self._heading_smooth = 0.0
            self._hold("surrounded — holding position")
            return

        # Average the chosen heading over time. Neighbours' scores differ by
        # less than the scan noise, so the winner flips left/right every tick;
        # averaging turns that flicker into the straight line it really is,
        # while a heading that is genuinely better stays chosen and still wins
        # through within ~1s.
        self._heading_smooth += HEADING_SMOOTHING * (best - self._heading_smooth)
        self.suggested_turn = self._slew(
            float(max(-1.0, min(1.0, self._heading_smooth / 90.0))))
        # Snap a residual turn away: ramping toward straight approaches 0 without
        # reaching it (0.167 -> 0.087 -> 0.007 -> ...), which would leave the two
        # wheels permanently a fraction apart. Below the deadband the wheels get
        # exactly equal commands, so "straight" really is straight.
        if abs(self.suggested_turn) < TURN_DEADBAND:
            self.suggested_turn = 0.0

        if pursuing and bearing is None:
            self.suggested_turn = target.next_scan_turn()
            self.last_decision = (f"looking for {target.name} — sweeping "
                                  f"{'left' if self.suggested_turn > 0 else 'right'} "
                                  f"cells {self.memory.busy_cells} "
                                  f"objs {len(self.memory.objects)}")
            return

        if pursuing and policy.aim_clear(scan, bearing):
            self.suggested_turn = policy.aim_turn(bearing)
            self.last_decision = (f"approaching {target.name} "
                                  f"bearing {bearing:+.0f}° "
                                  f"range {target.range_m:.2f}m "
                                  f"turn {self.suggested_turn:+.2f}")
            return

        if pursuing:
            self.last_decision = (f"approaching {target.name} "
                                  f"bearing {bearing:+.0f}° "
                                  f"range {target.range_m:.2f}m "
                                  f"heading {best:+.0f}° "
                                  f"turn {self.suggested_turn:+.2f} "
                                  f"(going around)")
            return

        self.last_decision = (f"heading {best:+.0f}° turn {self.suggested_turn:+.2f} "
                             f"cells {self.memory.busy_cells} "
                             f"objs {len(self.memory.objects)}")

    def _hold(self, decision, stop=True):
        """Stop suggesting motion (empty scan, too close, arrived, surrounded).

        `stop` asks the executor to halt the wheels; it is the only way the
        planner can stop the robot, because the avoider owns the motors.
        """
        self.suggested_turn = 0.0
        self.halt = stop
        self.last_decision = decision

    def _object_range_m(self, scan):
        """Range to attribute to a camera object: the front LIDAR sector."""
        front = scan.front_min_mm(OBJECT_CONE_DEG)
        range_m = (front / 1000.0) if front else 3.0
        return max(0.2, min(range_m, 3.0))
