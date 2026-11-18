"""CO SHOW 2026 floor track (rulebook 5.1, drawing "R1185 ~ R3005", straight 2000).

Only constants and the lane model offsets. No global coordinates are used while driving:
the runner tracks its own lane relative to the robot (lane_tracker.py).
Python 3.8 compatible (shared with the Jetson side).
"""
import math

LANE_W = 0.90                 # m, each lane
R_INFIELD = 1.185             # m, inner edge of lane 2 (infield yellow starts inside)
R_LANE12 = R_INFIELD + LANE_W  # 2.085 m, line between lane 2 and lane 1
R_OUTER = 3.005               # m, outer edge of lane 1 (dark grey board outside)
STRAIGHT = 2.0                # m, between the two semicircle centres
DIRECTION = +1                # counter-clockwise seen from above (arrows): every curve is a LEFT turn

# lane centre radii and one-lap length along the centre
R_CENTER = {1: (R_LANE12 + R_OUTER) / 2.0, 2: (R_INFIELD + R_LANE12) / 2.0}
LAP_LEN = {k: 2 * STRAIGHT + 2 * math.pi * r for k, r in R_CENTER.items()}   # 1: ~20.0 m, 2: ~14.3 m
CURVATURE = {k: DIRECTION / r for k, r in R_CENTER.items()}                  # 1: 0.39, 2: 0.61 1/m


def boundary_offsets(lane):
    """Lateral offsets (m, + = left) of the floor boundaries seen from the lane centre.

    Lane 1 (outer): left = lane1/lane2 line, right = outer edge, far left = infield edge.
    Lane 2 (inner): left = infield edge (yellow), right = lane line, far right = outer edge.
    """
    h = LANE_W / 2.0
    if lane == 1:
        return [(+h, 'lane12'), (-h, 'outer'), (+3 * h, 'infield')]
    if lane == 2:
        return [(+h, 'infield'), (-h, 'lane12'), (-3 * h, 'outer')]
    raise ValueError('lane must be 1 or 2')


def min_avg_speed(lane, extra=1.0, limit_s=60.0, start_delay=1.5):
    """Average speed needed to finish one lap (+ extra to get the rear feet over the line) in limit_s."""
    return (LAP_LEN[lane] + extra) / (limit_s - start_delay)


# ---------------------------------------------------------------------------------------------- walls
# Not in the rulebook as a wall: the 0.6 m wall stands on the edge of the 9000 x 7000 floor plate (rulebook 5.1,
# margin 495 mm around the 8010 x 6010 track), no gaps. Measured 11-18 from the XT16 (bags 133542/134210):
# 9.00 x 7.01 m between the inner faces. (By eye it looked like 0.30 m from the outline; the lidar says 0.5 m.)
# Track frame: origin = track centre, x = long axis (straights parallel to x), z up; counter-clockwise travel.
WALL_GAP = 0.495
WALL_H = 0.60
WALL_LX = 2 * (STRAIGHT / 2 + R_OUTER) + 2 * WALL_GAP     # 8.61 m, inner face to inner face
WALL_LY = 2 * R_OUTER + 2 * WALL_GAP                       # 6.61 m


def lane_frame(lane, x, y):
    """Track-frame point -> (lateral offset from the lane centre, + = left of travel; travel heading rad).

    Counter-clockwise travel. The lane centre is a stadium of radius R_CENTER[lane] around (+-STRAIGHT/2, 0).
    """
    r = R_CENTER[lane]
    cx = max(-STRAIGHT / 2, min(STRAIGHT / 2, x))
    dx, dy = x - cx, y
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return r, 0.0
    # outward normal (dx, dy)/d; CCW tangent = normal rotated +90 deg; left of travel = inward
    heading = math.atan2(dx / d, -dy / d)
    return r - d, heading
