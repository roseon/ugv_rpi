"""perception.py — raw sensor readings turned into robot-frame quantities.

This module is the single owner of the conventions every other module relies
on. Both of them have already caused real bugs in this project, so they live
in exactly one place:

  * LIDAR angles carry a +180° hardware offset: base_ctrl.parse_lidar_frame
    bakes it in, and the robot frame is 0 = forward, + = left. robot_angle()
    applies the correction. Anything comparing a raw angle against a direction
    must go through it, or "front" silently means the robot's back.
    robot_state.LidarScan is the only place that reads raw angles, so the
    correction is applied once, at the boundary, and every helper below
    expects angles already in the robot frame.

  * Camera boxes map to bearings with box_bearing_deg(), and to elevations with
    box_elevation_deg(). Image x grows to the right, so an object on the LEFT of
    the frame yields a POSITIVE bearing; image y grows downward, so an object
    BELOW the axis yields a POSITIVE elevation. Both are angles in the robot
    frame, which is the point: anything that points at a detection (the gaze, the
    pursuit) can map the two axes with one rule instead of one rule per axis.

Also here: the pure scan helpers built on those angles (sector_min, turn_bias,
arc_clearance) and object-label identity (norm_name, names_match), because
"is this the same object" is just as much a perception decision as bearing.

Nothing in this module holds state or reads hardware.
"""

import math

LIDAR_ANGLE_OFFSET = math.pi   # +180° baked in by base_ctrl.parse_lidar_frame
FRAME_WIDTH_PX = 640.0         # detection boxes live in this pixel space
FRAME_HEIGHT_PX = 480.0        # ...and this aspect is what sets the vertical FOV
H_FOV_DEG = 60.0               # horizontal FOV assumed when turning x into a bearing


# ── angles ────────────────────────────────────────────────────────────────────

def normalize_angle(a):
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def robot_angle(raw):
    """Raw LIDAR angle (radians) -> robot frame (0 = forward, + = left)."""
    return normalize_angle(raw - LIDAR_ANGLE_OFFSET)


def box_bearing_deg(box, frame_width=FRAME_WIDTH_PX, fov_deg=H_FOV_DEG):
    """Camera box -> bearing in the robot frame (+ = left, 0 = straight ahead)."""
    x1, _y1, x2, _y2 = box
    return (0.5 - ((x1 + x2) / 2.0) / frame_width) * fov_deg


def box_span_deg(box, frame_width=FRAME_WIDTH_PX, fov_deg=H_FOV_DEG):
    """Camera box -> how wide it is, as an angle (+-x of the same pinhole).

    What the box itself occupies in the camera's field of view, so a thing the
    camera can see can be asked about in the LIDAR's own terms: how wide a look
    at its bearing should be taken before believing no beams came back.
    """
    x1, _y1, x2, _y2 = box
    return abs(x2 - x1) / float(frame_width) * fov_deg


def v_fov_deg(frame_width=FRAME_WIDTH_PX, frame_height=FRAME_HEIGHT_PX,
              h_fov_deg=H_FOV_DEG):
    """The vertical FOV of the same pinhole.

    Square pixels and one focal length: f = (w/2)/tan(h_fov/2), so the vertical
    half-angle is atan((h/2)/f) and the aspect ratio alone decides it.  For the
    robot's 640x480 capture at a 60° horizontal FOV that is 46.8°, NOT 60° — a
    camera that sees 60° across sees less than that down.
    """
    return math.degrees(2.0 * math.atan(
        math.tan(math.radians(h_fov_deg) / 2.0) * (frame_height / frame_width)))


def box_elevation_deg(box, frame_height=FRAME_HEIGHT_PX, frame_width=FRAME_WIDTH_PX,
                      h_fov_deg=H_FOV_DEG):
    """Camera box -> elevation in the robot frame (+ = below the axis).

    The vertical counterpart of box_bearing_deg, and the same kind of quantity:
    an angle, so the gaze's two axes can be mapped onto the panels by one rule.
    They were not, and the mismatch was measured live on the robot: x went
    through an angle (0.83 aim units per degree, the panel's 120° cone) while y
    was the raw pixel fraction (2.14 units per degree), which made the vertical
    2.56x as sensitive as the horizontal - a head a little above the axis swung
    the pupils far further up the screen than it should, and the two axes
    disagreed about where the same object was.
    """
    _x1, y1, _x2, y2 = box
    cy = (y1 + y2) / 2.0 / frame_height
    return (cy - 0.5) * v_fov_deg(frame_width, frame_height, h_fov_deg)


# ── scan helpers (angles must already be in the robot frame) ──────────────

def side_wall(angles_rad, distances_mm, side, min_mm=150, max_mm=1300):
    """The wall alongside the robot: range (m) and heading offset (rad), or None.

    Reads the side sector (±15 deg around abeam at `side`·90°, + = left) and
    picks the most abeam solid return within reach — the bearing a Roomba-like
    follower actually steers against.  Positive heading offset means the nose
    points TOWARD the wall (nearer ahead than behind); the range is the
    standoff the follower holds.

    Returns None when the side is open: too far to matter, or no solid return
    (furniture gaps, sensor holes) — the caller then falls back to its other
    anchors instead of steering against a wall it cannot see.
    """
    beam = side * math.pi / 2.0
    lo = beam - math.radians(15.0)
    hi = beam + math.radians(15.0)
    best = None
    best_abeam = 0.0
    for a, d in zip(angles_rad, distances_mm):
        if not (lo <= a <= hi) or not (min_mm <= d <= max_mm):
            continue
        abeam = math.cos(normalize_angle(a - beam))   # 1.0 exactly abeam
        if abeam < best_abeam:
            continue
        best_abeam = abeam
        best = (d / 1000.0, normalize_angle(a - beam))
    return best


def sector_min(angles_rad, distances_mm, center_deg, half_deg, min_mm=50):
    """Smallest valid distance (mm) inside an angular sector; inf = no reading."""
    lo = math.radians(center_deg - half_deg)
    hi = math.radians(center_deg + half_deg)
    best = float('inf')
    for a, d in zip(angles_rad, distances_mm):
        if lo <= a <= hi and d >= min_mm and d < best:
            best = d
    return best


def turn_bias(angles_rad, distances_mm, half_deg, danger_mm, min_mm=50):
    """Weighted turn direction in [-1, +1], for steering away from an obstacle.

    +1 = turn LEFT (clearer on the left), -1 = turn RIGHT.
    """
    left_w = right_w = 0.0
    lo = math.radians(-half_deg)
    hi = math.radians(half_deg)
    for a, d in zip(angles_rad, distances_mm):
        if not (lo <= a <= hi):
            continue
        if d < min_mm or d > danger_mm:
            continue
        w = (danger_mm - d) / danger_mm
        if a < 0:
            right_w += w   # obstacle on the right -> push left
        else:
            left_w += w    # obstacle on the left  -> push right
    total = left_w + right_w
    if total < 1e-6:
        return 0.0
    return float(max(-1.0, min(1.0, (right_w - left_w) / total)))


def arc_clearance(angles_rad, distances_mm, heading_rad, half_width_rad,
                  blocked_mm=1100):
    """Fraction (0..1) of the samples in a heading's arc that are clear.

    A true fraction, not a raw count: half the arc blocked must read 0.5 so
    thresholds like "don't drive into this heading" mean something.
    """
    total = blocked = 0
    for a, d in zip(angles_rad, distances_mm):
        if abs(normalize_angle(a - heading_rad)) > half_width_rad:
            continue
        if d <= 0:
            continue
        total += 1
        if d < blocked_mm:
            blocked += 1
    if total == 0:
        return 1.0
    return 1.0 - blocked / float(total)


# ── pose: where the robot is, in the frame its map is kept in ────────────────
# Wheel odometry reports signed travel per wheel in metres (measured on this
# robot: 2.0 s at 0.30 came back as 0.589 m).  The chassis turns about its
# centre, so the wheels' difference is a rotation and their mean is a step along
# the heading.  The pose is kept in the map frame, which is the robot frame at
# the moment the map was started: x = left, y = forward, theta = heading
# (counter-clockwise, 0 = +y), matching the angle convention above.
TRACK_M = 0.20          # wheel track (left-right): the one dimension the wheel
                        # odometry cannot supply.  The UGV Rover body is 230 mm
                        # wide with the wheels outboard of it.
POSE_MAX_STEP_M = 1.5   # a step bigger than this is not a wheel measurement


def pose_step(pose, d_left_m, d_right_m, track_m=TRACK_M):
    """Move `pose` (x, y, theta) by one odometry step.  Returns the new pose."""
    x, y, theta = pose
    d_left_m = float(d_left_m or 0.0)
    d_right_m = float(d_right_m or 0.0)
    turn = (d_right_m - d_left_m) / max(1e-6, track_m)   # right wheel further = left turn
    step = 0.5 * (d_left_m + d_right_m)
    mid = theta + turn / 2.0                             # integrate on the mid-step heading
    return (x + step * math.sin(mid), y + step * math.cos(mid),
            normalize_angle(theta + turn))


def robot_to_world(pose, mx, my):
    """A point in the robot frame (x left, y forward, metres) -> map frame."""
    x, y, theta = pose
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (x + mx * cos_t + my * sin_t, y - mx * sin_t + my * cos_t)


def world_to_robot(pose, wx, wy):
    """The inverse of robot_to_world (used by every map query)."""
    x, y, theta = pose
    dx, dy = wx - x, wy - y
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t)


def point_from_scan(a, d_mm):
    """One LIDAR reading (robot-frame angle, mm) -> robot-frame point in metres."""
    d = d_mm / 1000.0
    return (d * math.sin(a), d * math.cos(a))


def front_min_mm(angles_rad, distances_mm, half_deg):
    """Smallest positive range inside a forward cone, in mm (None if none)."""
    return min((d for a, d in zip(angles_rad, distances_mm)
                if d > 0 and abs(a) < math.radians(half_deg)),
               default=None)


# ── object labels ─────────────────────────────────────────────────────────────

def norm_name(name):
    """Lower-case and singularize a class label ('chairs' -> 'chair')."""
    n = (name or "").strip().lower()
    if n.endswith("s") and not n.endswith("ss"):
        n = n[:-1]
    return n


def names_match(det_name, target):
    """Loose match so "drive to the chair" hits a 'chair' or 'chairs' box."""
    a, b = norm_name(det_name), norm_name(target)
    if not a or not b:
        return False
    return a == b or a in b or b in a
