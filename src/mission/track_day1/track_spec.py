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
