"""robot_state.py — reading what the robot's sensors currently say.

One owner of "the robot's live readings". Nothing here decides anything: it
reads the LIDAR scan (applying the raw->robot-frame angle correction exactly
once, via perception.robot_angle) and the ESP32 wheel odometer counters.

The LIDAR list is published by base_ctrl's receive thread and re-bound as a
fresh list on every revolution, so every read snapshots it into a list before
use — the planner must not walk a list that is being replaced underneath it.
"""

import math
import time

from perception import (POSE_MAX_STEP_M, front_min_mm, robot_angle, sector_min)

# A bin this old says nothing about where anything is now (base_ctrl stamps each
# 1-degree cell as it is filled, and /lidar_points serves the same 3 s window).
DENSE_BIN_AGE_S = 3.0


def dense_sector_min_mm(base, center_deg, half_deg, max_age_s=DENSE_BIN_AGE_S):
    """Smallest fresh return in a sector, from the 1-degree occupancy bins.

    `LidarScan` reads the last revolution, and this kit's marginal wire often
    delivers it in patches: a sector can simply be a hole, where a far return
    survives and the near one never arrived.  Asking that scan where a camera
    box is therefore answers with the room behind the thing — measured live,
    objects were placed up to 2.5 m beyond the sensor's own return at their
    bearing.  The bins are built from every valid packet of the last few seconds
    (`base_ctrl.lidar_data_recv`), are the dense picture the operator's radar is
    drawn from, and are stamped, so a stale cell is skipped rather than trusted.

    Returns mm, or None when the sector holds no fresh return — the caller's
    own fallback is more honest than a distance nothing measured.
    """
    rl = getattr(base, 'rl', None)
    bins = getattr(rl, 'lidar_bins', None)
    if not bins:
        return None
    now = time.time()
    angles, distances = [], []
    for deg in range(int(math.ceil(center_deg - half_deg)),
                     int(math.floor(center_deg + half_deg)) + 1):
        # The bins are indexed the way base_ctrl parses the wire: with the +180
        # the sensor's mount bakes in (perception.LIDAR_ANGLE_OFFSET).
        dist, stamp = bins[int((deg + 180) % 360)]
        if dist > 0 and now - stamp <= max_age_s:
            angles.append(math.radians(deg))
            distances.append(dist)
    if not angles:
        return None
    best = sector_min(angles, distances, center_deg, half_deg)
    return None if best == float('inf') else best


class LidarScan:
    """One LIDAR scan, angles already in the robot frame (0 = forward, + = left).

    `stamp` identifies the revolution it came from (the sender's own timestamp,
    which moves only when a new scan is published).  The planner ticks faster
    than the LIDAR revolves, so without it the same revolution would be folded
    into the map seven times and a stationary wall would look seven times as
    solid as a real one.
    """

    __slots__ = ('angles', 'distances', 'stamp', '_front_cache')

    def __init__(self, angles, distances, stamp=None):
        self.angles = angles
        self.distances = distances
        self.stamp = stamp
        self._front_cache = {}

    @classmethod
    def read(cls, base):
        """Snapshot the freshest scan from a BaseController."""
        rl = getattr(base, 'rl', None)
        raw_angles = list(getattr(rl, 'lidar_angles_show', None) or [])
        distances = list(getattr(rl, 'lidar_distances_show', None) or [])
        return cls([robot_angle(a) for a in raw_angles], distances,
                   getattr(rl, 'lidar_scan_time', None))

    @property
    def empty(self):
        return not self.angles

    def front_min_mm(self, half_deg):
        """Smallest range inside a forward cone of +-half_deg (cached per cone)."""
        if half_deg not in self._front_cache:
            self._front_cache[half_deg] = front_min_mm(self.angles,
                                                       self.distances, half_deg)
        return self._front_cache[half_deg]


def wheel_odometry(base):
    """Latest ESP32 wheel odometer counters: signed travel per wheel, in metres,
    with forward positive — the convention every consumer here assumes.

    The planner only suggests headings, so without these there is no way to
    tell "steering not delivered" from "chassis cannot move" — the counters
    changing is the only hard proof the wheels actually turned.

    Magnitude measured on this robot: 2.0 s commanded at 0.30 came back as
    0.589 m on the left and 0.588 on the right.  Direction measured live while
    cruising forward on self-drive: both counters *decrease* (odl -1999.6 ->
    -2144.6 over about 9 m of forward travel), so the chassis counts down
    going forward.  The sign is flipped here, once, at the only place the raw
    counters enter this code: fed through raw, the pose gate kept proposing a
    backward move (motion_m negative) and the scan rightly rejected it —
    13.5 m driven live, pose moved 0.000 m, the map learned nothing.
    """
    data = getattr(base, 'base_data', None) or {}
    odl, odr = data.get('odl'), data.get('odr')
    if isinstance(odl, (int, float)):
        odl = -float(odl)
    if isinstance(odr, (int, float)):
        odr = -float(odr)
    return {'odl': odl, 'odr': odr, 'voltage': data.get('v')}


class WheelStep:
    """How far each wheel has turned since the previous step, in metres.

    The counters are cumulative, so a difference of two readings is the motion
    of one planning tick.  A reading that is missing, or that moved further
    than any wheel could in one tick, is reported as unknown rather than
    guessed: a wrong pose is worse for the map than no pose update at all.
    """

    def __init__(self, max_step_m=POSE_MAX_STEP_M):
        self._last = None
        self.max_step_m = float(max_step_m)

    def reset(self):
        self._last = None

    def step(self, base):
        """(d_left, d_right) in metres since the last call, or None if unknown."""
        now = wheel_odometry(base)
        left, right = now.get('odl'), now.get('odr')
        if not isinstance(left, (int, float)) or not isinstance(right, (int, float)):
            self._last = None
            return None
        previous, self._last = self._last, (float(left), float(right))
        if previous is None:
            return None                      # first reading establishes the origin
        d_left, d_right = float(left) - previous[0], float(right) - previous[1]
        if max(abs(d_left), abs(d_right)) > self.max_step_m:
            return None                      # not a wheel measurement — a reboot, say
        return (d_left, d_right)
