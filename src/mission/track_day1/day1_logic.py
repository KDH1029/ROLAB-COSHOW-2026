"""Day-1 decisions free of ROS (shared by the runner node and the offline sim).

LightGate   start only after GREEN is seen continuously for need_s and no RED within block_s.
LapEnd      distance from the built-in odometry (already scaled in the odom TF) -> where to cut the path.
RppLike     the core of Nav2 Regulated Pure Pursuit, only for the offline sim. On the robot the real
            nav2_regulated_pure_pursuit_controller follows the same path (config/day1_rpp.yaml).
"""
import math

import numpy as np

import track_spec as T


class LightGate(object):
    def __init__(self, need_s=0.5, block_s=1.0, stale_s=1.0):
        self.need_s, self.block_s, self.stale_s = need_s, block_s, stale_s
        self.green_since = None
        self.last_red = -1e9
        self.last_msg = -1e9

    def feed(self, state, t):
        self.last_msg = t
        if state == 'GREEN':
            if self.green_since is None:
                self.green_since = t
        else:
            self.green_since = None
            if state == 'RED':
                self.last_red = t

    def go(self, t):
        if t - self.last_msg > self.stale_s:
            self.green_since = None
            return False
        return (self.green_since is not None and t - self.green_since >= self.need_s
                and t - self.last_red >= self.block_s)


class LapEnd(object):
    """One lap of the lane centre + extra so the rear feet surely cross the start line.

    Overshoot costs no time (timing stops when the rear feet cross), so extra is generous.
    """

    def __init__(self, lane, extra=1.2, lap_len=None):
        self.total = (lap_len if lap_len is not None else T.LAP_LEN[lane]) + extra
        self.dist = 0.0
        self.prev = None

    def feed(self, x, y):
        if self.prev is not None:
            self.dist += math.hypot(x - self.prev[0], y - self.prev[1])
        self.prev = (x, y)
        return self.remaining()

    def remaining(self):
        return self.total - self.dist


def speed_check(lane, v, extra=1.2):
    need = T.min_avg_speed(lane, extra)
    return v * 0.9 >= need, 'lane %d: lap %.1f m needs avg >= %.2f m/s; desired %.2f m/s' % (
        lane, T.LAP_LEN[lane], need, v)


class RppLike(object):
    """Nav2 RPP core (no rotate-to-heading, no collision check): lookahead point, curvature,
    curvature regulation, approach slow-down, min speeds."""

    def __init__(self, v=0.5, lookahead=0.8, min_radius=0.9, min_speed=0.25, approach_dist=0.6,
                 min_approach=0.05, max_w=1.0, max_acc=0.5, dt=0.02):
        self.v_des, self.L, self.min_r, self.min_v = v, lookahead, min_radius, min_speed
        self.approach, self.min_app, self.max_w, self.max_acc, self.dt = approach_dist, min_approach, max_w, max_acc, dt
        self.v = 0.0

    def step(self, path_robot, remaining_len):
        """path_robot: Nx2 path points in the robot frame (ahead of the closest point)."""
        if path_robot is None or len(path_robot) < 2:
            self.v = max(0.0, self.v - self.max_acc * self.dt * 2)
            return self.v, 0.0
        d = np.hypot(path_robot[:, 0], path_robot[:, 1])
        idx = np.nonzero(d >= self.L)[0]
        P = path_robot[idx[0]] if len(idx) else path_robot[-1]
        L2 = max(P[0] ** 2 + P[1] ** 2, 1e-6)
        curv = 2.0 * P[1] / L2
        v = self.v_des
        if abs(curv) > 1e-6 and 1.0 / abs(curv) < self.min_r:
            v = max(self.v_des * (1.0 / abs(curv)) / self.min_r, self.min_v)
        if remaining_len < self.approach:
            v = max(v * remaining_len / self.approach, self.min_app)
        if remaining_len < 0.05:
            v = 0.0
        dv = max(-self.max_acc * self.dt * 2, min(self.max_acc * self.dt, v - self.v))
        self.v = max(0.0, self.v + dv)
        w = max(-self.max_w, min(self.max_w, self.v * curv))
        return self.v, w
