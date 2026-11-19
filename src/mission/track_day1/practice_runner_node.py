#!/usr/bin/env python3
"""Practice runner: start ANYWHERE on the route and stop at the route's start (the start mark) — test only, a fork
(day1_runner_node.py and `day1.sh run` are not changed).

    scripts/practice_run.sh [route=rec105326_lap1] [hoop=in] [v=0.8] [...day1.sh run arguments]   (alias dhere)

e.g. 11-20 practice 1: the cones were moved and not taught yet, so start past the cone field (right side, facing up
the right wall), drive the rest of the lap (arch, zigzag) and stop at the start mark, before the cones.
Differences from the runner:
  - start lock: no '1 m from the route start' rule; the 180 deg wall twin is the one that puts the robot on the route
    (smallest lateral + heading error), not the one nearest the route start; limits max_lat 0.5 m / max_head 30 deg
  - lap end: at the route's start station + end_extra (0 m) instead of one lap + extra
  - zigzag_v=<m/s>: the zigzag zone's speed for this run only (the zones file keeps its 0.4)
Everything else (light, zones, hula hoop, keys, debug) is the runner's.
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import day1_runner_node as R  # noqa: E402
import track_path as TP  # noqa: E402


class AnyStartLock(TP.StartLock):
    """StartLock without the 'near the route start' rule; twin = the one that lies on the route."""

    def fit(self, tw):
        q = TP.compose(self.off_inv, tw)
        lat, _, s = TP.lateral_of(self.loop, q[0], q[1])
        a, b = self.loop.pose_at(s), self.loop.pose_at(s + 0.5)
        head = TP.wrap(q[2] - math.atan2(b[1] - a[1], b[0] - a[0]))
        return abs(lat) + 0.5 * abs(head)

    def feed(self, wall, odom):
        if not wall.get('ok'):
            self.why = 'wall: ' + wall.get('why', '?')
            return None
        tw = (wall['x'], wall['y'], wall['yaw'])
        twin = (-tw[0], -tw[1], TP.wrap(tw[2] + math.pi))
        if not self.buf:
            if self.fit(twin) < self.fit(tw):
                tw = twin
        else:
            p = self.buf[0][1]
            if math.hypot(tw[0] - p[0], tw[1] - p[1]) > math.hypot(twin[0] - p[0], twin[1] - p[1]):
                tw = twin
        tt = TP.compose(self.off_inv, tw)
        self.buf.append((TP.compose(tt, TP.inverse(odom)), tw, odom))
        self.buf = self.buf[-self.n:]
        if len(self.buf) < self.n:
            self.why = 'pooling %d/%d' % (len(self.buf), self.n)
            return None
        A = np.array([b[0] for b in self.buf])
        yc = math.atan2(np.sin(A[:, 2]).mean(), np.cos(A[:, 2]).mean())
        spread = np.hypot(A[:, 0] - A[:, 0].mean(), A[:, 1] - A[:, 1].mean()).max()
        yspread = np.abs([TP.wrap(a - yc) for a in A[:, 2]]).max()
        if spread > self.tol_m or yspread > self.tol:
            self.why = 'not steady: %.1f cm / %.2f deg over %d poses' % (100 * spread, math.degrees(yspread), self.n)
            return None
        Tto = (float(A[:, 0].mean()), float(A[:, 1].mean()), yc)
        pose = TP.compose(Tto, self.buf[-1][2])
        lat, _, s = TP.lateral_of(self.loop, pose[0], pose[1])
        a, b = self.loop.pose_at(s), self.loop.pose_at(s + 0.5)
        head = TP.wrap(pose[2] - math.atan2(b[1] - a[1], b[0] - a[0]))
        if abs(lat) > self.max_lat or abs(head) > self.max_head:
            self.why = 'placement: lateral %+.2f m, heading %+.1f deg (limits %.2f m, %.0f deg) — stand on the route, facing along it' % (
                lat, math.degrees(head), self.max_lat, math.degrees(self.max_head))
            return None
        self.why = ''
        return dict(T_track_odom=Tto, pose=pose, s=s, lateral=lat, heading_err=head)


class PracticeRunner(R.Day1Runner):
    def __init__(self):
        super().__init__()

        def p(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.end_extra = float(p('end_extra', 0.0))
        self.zigzag_v = float(p('zigzag_v', 0.0))         # 0 = the zones file's value
        self.zone_override()
        if self.mode != 'wall' or not self.path_file:
            raise SystemExit('practice runner: a route is needed (route=<name>)')
        self.wlock = AnyStartLock(self.lane, n=self.lock_n, max_lat=self.max_lat, max_head_deg=self.max_head,
                                  loop=self.loop, offset=self.track_off)
        self.end_set = False
        self.get_logger().info('★ PRACTICE: start anywhere on %s, stop at its start (+%.1f m)'
                               % (os.path.basename(self.path_file), self.end_extra))

    def zone_override(self):
        if self.zigzag_v > 0:
            for z in self.zones:
                if z.get('name') == 'zigzag':
                    z['v'] = self.zigzag_v
                    self.get_logger().info('★ PRACTICE: zigzag v %.2f for this run (zones file unchanged)' % self.zigzag_v)

    def cb_lidar(self, m):
        super().cb_lidar(m)
        if self.plan is not None and not self.end_set:
            with self.lock:
                L = self.loop.length
                s0 = self.plan.s_start
                to_start = (self.loop.s0 - s0) % L          # route start station ahead of where it stands
                if to_start < 1.0:                           # standing at the start already: a whole lap
                    to_start += L
                self.plan.s_end = s0 + to_start + self.end_extra
                self.end_set = True
            self.get_logger().info('★ PRACTICE: from station %.1f m, stop at the route start: %.1f m to drive'
                                   % (s0 % L, self.plan.s_end - s0))

    def look_switch(self, name, why, s_r):
        end_left = self.plan.s_end - s_r
        super().look_switch(name, why, s_r)
        self.zone_override()                         # the new branch's zones were read from its file
        if self.plan is not None and self.end_set:   # new branch: still stop at the start (same distance to go)
            sn = self.prog.s
            L = self.loop.length
            self.plan.s_end = sn + ((self.loop.s0 - sn) % L) + self.end_extra
            self.get_logger().info('★ PRACTICE: branch %s, %.1f m to the start (was %.1f)' % (name, self.plan.s_end - sn, end_left))


def main():
    rclpy.init()
    node = PracticeRunner()
    if node.keys:
        threading.Thread(target=R.key_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.stop('Ctrl+C')
        time.sleep(0.2)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
