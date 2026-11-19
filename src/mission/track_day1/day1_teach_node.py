#!/usr/bin/env python3
"""Teach the day-1 route by driving it once with the remote (NUC, ROS 2 Jazzy). Nothing is sent to the motors.

  day1.sh up (FAST-LIO odom TF) running, then  day1.sh teach in|out name=<route>
  1. robot standing on its start mark: wall_align lock, the same as the runner (lane and placement checked)
  2. drive one lap with the remote around the cones; when it comes back past the start the lap is saved
     (Enter = stop and save now, x = discard, nothing else needed)
  -> logs/day1/routes/<name>.csv (pins, the runner's path_file format), <name>.png, <name>_raw.csv
  Run it:  day1.sh run in route=<name>
The route is stored in the track frame (from the wall), so a later start placed a bit differently still uses it.
"""
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import track_path as TP  # noqa: E402
import track_spec as T  # noqa: E402
import wall_align as WA  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, '../../..'))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Teach(Node):
    def __init__(self):
        super().__init__('day1_teach')

        def p(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        lane = str(p('lane', 2).value).strip().lower()
        self.lane = {'1': 1, 'out': 1, 'outer': 1, '2': 2, 'in': 2, 'inner': 2}.get(lane)
        if self.lane is None:
            raise SystemExit('lane must be in or out')
        self.name = str(p('name', time.strftime('route_%m%d_%H%M')).value)
        self.odom_frame = str(p('odom_frame', 'odom').value)
        self.base_frame = str(p('base_frame', 'base_link').value)
        self.max_s = float(p('max_s', 180.0).value)
        self.out_dir = os.path.join(os.environ.get('DAY1_LOG', os.path.join(ROOT, 'logs/day1')), 'routes')
        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tf, self)
        self.wlock = TP.StartLock(self.lane, max_lat=0.45, max_head_deg=30.0)
        self.state = 'LOCK'
        self.T_to = None
        self.traj = []
        self.n = 0
        self.t_rec = None
        self.done = False
        self.sub = self.create_subscription(PointCloud2, '/lidar_points', self.cb_lidar, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.report)
        self.get_logger().info('teach %s course (lane %d) -> %s/%s.csv — stand still on the start mark'
                               % ('in' if self.lane == 2 else 'out', self.lane, self.out_dir, self.name))

    def pose(self):
        try:
            tr = self.tf.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception:
            return None
        t = tr.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def cb_lidar(self, m):
        if self.state != 'LOCK':
            return
        self.n += 1
        if self.n % 2:
            return
        cur = self.pose()
        if cur is None:
            self.get_logger().warn('no odom TF — day1.sh up running?', throttle_duration_sec=2.0)
            return
        P = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        res = self.wlock.feed(WA.align([WA.lidar_to_base(P)], seed=self.n), cur)
        if res is None:
            self.get_logger().info('wall lock: ' + self.wlock.why, throttle_duration_sec=1.0)
            return
        self.T_to = res['T_track_odom']
        self.destroy_subscription(self.sub)
        self.state, self.t_rec = 'REC', time.time()
        self.traj.append(res['pose'][:2])
        self.get_logger().info('★ start found (station %.2f, lateral %+.2f m, heading %+.1f deg) — drive the lap now '
                               'with the remote. Enter = save now, x = discard'
                               % (res['s'], res['lateral'], math.degrees(res['heading_err'])))

    def tick(self):
        if self.state != 'REC':
            return
        cur = self.pose()
        if cur is None:
            return
        x, y, _ = TP.compose(self.T_to, cur)
        if math.hypot(x - self.traj[-1][0], y - self.traj[-1][1]) > 0.02:
            self.traj.append((x, y))
        P = np.array(self.traj)
        dist = float(np.hypot(*np.diff(P, axis=0).T).sum()) if len(P) > 1 else 0.0
        r = math.hypot(x - P[0, 0], y - P[0, 1])
        self.dist, self.r0 = dist, r
        # back past the start: far enough along, close to the start and moving away again
        if dist > 0.6 * T.LAP_LEN[self.lane] and r < 0.6:
            self.near = min(getattr(self, 'near', 9.0), r)
            if r > self.near + 0.10:
                self.save('lap closed')
        if time.time() - self.t_rec > self.max_s:
            self.save('time limit %.0f s' % self.max_s)

    def report(self):
        if self.state == 'REC':
            self.get_logger().info('REC %.1f m · %.0f s · start %.2f m away' % (self.dist if hasattr(self, 'dist') else 0.0,
                                                                              time.time() - self.t_rec, getattr(self, 'r0', 0.0)))

    def save(self, why):
        if self.state != 'REC':
            return
        self.state = 'SAVED'
        P = np.array(self.traj)
        if len(P) < 20:
            self.get_logger().error('too short (%d points) — nothing saved' % len(P))
            self.done = True
            return
        pins, info = TP.route_from_trajectory(P, self.lane)
        os.makedirs(self.out_dir, exist_ok=True)
        base = os.path.join(self.out_dir, self.name)
        TP.save_route_csv(base + '.csv', self.lane, pins)
        np.savetxt(base + '_raw.csv', P, delimiter=',', fmt='%.4f', header='x_m,y_m (track frame, 2 cm steps)', comments='')
        lp = TP.PinLoop(pins[:, 1:3], s0=0.0)
        rb = TP.RulebookLoop(self.lane)
        lat = np.array([TP.lateral_of(rb, *lp.pose_at(s)[:2])[0] for s in np.arange(0, lp.length, 0.1)])
        self.get_logger().info('★ saved (%s): %s.csv — %d pins, %.2f m (rulebook lap %.2f), closed %s, gap %.2f m, '
                               'dropped %.1f m; off the lane centre max %.2f m'
                               % (why, base, len(pins), info['length_m'], T.LAP_LEN[self.lane], info['closed'],
                                  info['gap_m'], info['dropped_m'], np.abs(lat).max()))
        if not info['closed']:
            self.get_logger().warn('★ the drive did not come back to the start: the route is closed with a straight '
                                   '%.2f m line — teach again if that is long' % info['gap_m'])
        try:
            self.picture(base + '.png', pins, lp)
        except Exception as e:
            self.get_logger().warn('picture failed: %s' % e)
        self.get_logger().info('run it:  day1.sh run %s route=%s v=0.4 lookahead=0.5'
                               % ('in' if self.lane == 2 else 'out', self.name))
        self.done = True

    def picture(self, path, pins, lp):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        sys.path.insert(0, os.path.join(HERE, 'tools'))
        import plot_plan
        plot_plan._font()
        fig, ax = plt.subplots(figsize=(10, 8))
        plot_plan.draw_track(ax)
        raw = np.array(self.traj)
        ax.plot(raw[:, 0], raw[:, 1], color='#888', lw=0.8, label='리모컨 주행 (원본)')
        ax.plot(pins[:, 1], pins[:, 2], color=plot_plan.COL[self.lane], lw=2.4, label='저장된 경로 (%.2f m)' % lp.length)
        ax.plot(pins[0, 1], pins[0, 2], 'o', ms=12, mfc='#27ae60', mec='white', mew=2, label='출발')
        for s in np.arange(1.0, lp.length, 2.0):
            x, y, a = lp.pose_at(s)
            ax.annotate('', xy=(x + 0.12 * math.cos(a), y + 0.12 * math.sin(a)), xytext=(x, y),
                        arrowprops=dict(arrowstyle='-|>', color=plot_plan.COL[self.lane], lw=1.4))
        ax.legend(loc='upper right', fontsize=9)
        ax.set_title('%s — %s코스 가르친 경로' % (self.name, '인' if self.lane == 2 else '아웃'), fontsize=14)
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
        self.get_logger().info('picture: %s' % path)


def key_loop(node):
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        return
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok() and not node.done:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch in ('\n', '\r', 's', 'S', 'q', 'Q'):
                node.save('key')
            elif ch in ('x', 'X'):
                node.get_logger().warn('discarded')
                node.state, node.done = 'SAVED', True
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = Teach()
    threading.Thread(target=key_loop, args=(node,), daemon=True).start()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.save('Ctrl+C')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
