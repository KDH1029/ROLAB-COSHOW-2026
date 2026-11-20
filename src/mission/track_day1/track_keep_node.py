#!/usr/bin/env python3
"""Keep the track position from the start mark on — debug / practice tool (the runners are not changed).

    scripts/track_keep.sh [route=rec105326_lap1]        (alias dkeep) — robot standing on the start mark, `dup` running

1. at the start mark, where the wall fit is sure (11-19 bag: 93 % vs 2 % on the map), the same wall lock as the runner
   gives track <- odom (FAST-LIO's odom frame)
2. saved to logs/day1/track_keep.json with the FAST-LIO process it belongs to; then, until q, every second: the robot's
   track position, its station / lateral on the route, and how far it is from the route start
3. `dhere lock=keep` (practice runner) starts from this instead of a wall fit — anywhere, also where the walls are
   misread (right in front of the zigzag). Valid while the same FAST-LIO runs: do not restart `dup`, walk the robot
   with the remote (do not carry it: knocks break FAST-LIO). FAST-LIO drifts ~4 cm / 0.8 deg in 4 min (11-05).
Also published: TF track -> odom (static) and /track/robot_pose (PoseStamped, frame track) for RViz.
"""
import atexit
import json
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
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import track_path as TP  # noqa: E402
import wall_align as WA  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
LOG = os.environ.get('DAY1_LOG', os.path.join(ROOT, 'logs', 'day1'))
KEEP = os.path.join(LOG, 'track_keep.json')


def fastlio_pid():
    try:
        return int(open(os.path.join(LOG, 'fastlio.pid')).read().strip())
    except Exception:
        return None


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Keep(Node):
    def __init__(self):
        super().__init__('track_keep')

        def p(n, d):
            return self.declare_parameter(n, d, ParameterDescriptor(dynamic_typing=True)).value
        self.path = str(p('path_file', ''))
        self.loop = TP.load_pins(self.path, 2)
        self.lock = TP.StartLock(2, n=5, max_lat=0.5, max_head_deg=30.0, loop=self.loop)   # the runner's rule: near the start
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.stb = StaticTransformBroadcaster(self)
        self.pub = self.create_publisher(PoseStamped, '/track/robot_pose', 5)
        self.sub = self.create_subscription(PointCloud2, '/lidar_points', self.cb_lidar, qos_profile_sensor_data)
        self.T = None
        self.n = 0
        self.t0 = time.time()
        self.last_odom = None
        self.done = False
        self.create_timer(0.1, self.tick)
        self.get_logger().info('track keep on %s — robot standing on the start mark, q = quit' % os.path.basename(self.path))

    def odom(self):
        try:
            tr = self.tf.lookup_transform('odom', 'base_link', Time())
        except Exception:
            return None
        t = tr.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def cb_lidar(self, m):
        if self.T is not None:
            return
        self.n += 1
        if self.n % 2:
            return
        o = self.odom()
        if o is None:
            self.get_logger().warn('no odom TF — dup first', throttle_duration_sec=2.0)
            return
        P = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        res = self.lock.feed(WA.align([WA.lidar_to_base(P)], seed=self.n), o)
        if res is None:
            self.get_logger().info('wall lock: ' + self.lock.why, throttle_duration_sec=1.0)
            if time.time() - self.t0 > 30:
                self.get_logger().error('★ no start in 30 s: %s' % self.lock.why)
                self.t0 = time.time()
            return
        self.T = res['T_track_odom']
        self.destroy_subscription(self.sub)
        json.dump(dict(T_track_odom=list(self.T), fastlio_pid=fastlio_pid(), time=time.time(),
                       route=self.path, start=dict(s=res['s'], lateral=res['lateral'],
                                                   heading_deg=math.degrees(res['heading_err']))),
                  open(KEEP, 'w'), indent=1)
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id, ts.child_frame_id = 'track', 'odom'
        ts.transform.translation.x, ts.transform.translation.y = float(self.T[0]), float(self.T[1])
        ts.transform.rotation.z, ts.transform.rotation.w = math.sin(self.T[2] / 2), math.cos(self.T[2] / 2)
        self.stb.sendTransform(ts)
        self.get_logger().info('■ kept: start lateral %+.3f m, heading %+.1f deg -> %s (FAST-LIO pid %s). '
                               'Walk it anywhere with the remote; dhere starts from this (dkeep may be closed)'
                               % (res['lateral'], math.degrees(res['heading_err']), KEEP, fastlio_pid()))

    def tick(self):
        if self.T is None:
            return
        o = self.odom()
        if o is None:
            return
        if self.last_odom is not None and math.hypot(o[0] - self.last_odom[0], o[1] - self.last_odom[1]) > 1.0:
            self.get_logger().error('★ odom jumped %.1f m in 0.1 s — FAST-LIO reset? the kept position is no good'
                                    % math.hypot(o[0] - self.last_odom[0], o[1] - self.last_odom[1]))
        self.last_odom = o
        tp = TP.compose(self.T, o)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'track'
        ps.pose.position.x, ps.pose.position.y = float(tp[0]), float(tp[1])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(tp[2] / 2), math.cos(tp[2] / 2)
        self.pub.publish(ps)
        if int(time.time() * 10) % 10 == 0:
            lat, _, s = TP.lateral_of(self.loop, tp[0], tp[1])
            print('track (%+.2f, %+.2f) heading %4.0f deg · route %5.2f m of %.1f, lateral %+.2f m · kept %.0f s ago'
                  % (tp[0], tp[1], math.degrees(tp[2]), s % self.loop.length, self.loop.length, lat,
                     time.time() - json.load(open(KEEP))['time']), flush=True)


def key_loop(node):
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        return
    old = termios.tcgetattr(fd)
    # 데몬 스레드라 프로그램이 끝날 때 아래 finally 가 안 돌 수 있다 → 터미널이 글자 안 보이는 모드로 남음(11-20). 끝날 때 꼭 되돌림
    atexit.register(termios.tcsetattr, fd, termios.TCSADRAIN, old)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if r and sys.stdin.read(1) in ('q', 'Q'):
                rclpy.shutdown()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    n = Keep()
    threading.Thread(target=key_loop, args=(n,), daemon=True).start()
    try:
        rclpy.spin(n)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
