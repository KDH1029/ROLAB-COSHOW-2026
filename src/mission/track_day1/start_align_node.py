#!/usr/bin/env python3
"""Start alignment, run on its own BEFORE the run command: puts the robot on the route's start line.

    scripts/day1_align.sh route=rec105326_lap1        (then scripts/day1.sh run route=... as usual)

The runner starts from wherever the robot stands and eases onto the route over blend_m (2 m). In the 11-19 cone
field the first cone is ~1 m after the start mark, so a robot put down 0.2 m off the taught start and 14 deg
askew touched it (runs 17:28-17:33). This program, separate from the runner (day1_runner_node.py and day1.sh are
not changed):

  1. gait: BalanceStand -> ClassicWalk (the runner's gait), response checked
  2. where am I: the same wall lock as the runner (XT16 /lidar_points, wall_align + StartLock, route twin), limits
     relaxed (0.6 m / 60 deg) — it is here to fix the placement
  3. turn in place to the route's direction (next 0.5 m from the robot's closest point), then step sideways onto
     the route (vy, body frame), check, repeat at most 3 rounds; FAST-LIO odometry (odom -> base_link TF)
  4. stop (zero /cmd_vel, StopMove, motor bridge deactivated) and print the result; the runner's own start lock
     then shows lateral / heading near 0

Needs `day1.sh up` (FAST-LIO, motor bridge configured) and `day1.sh xt16`. Robot standing on the start mark.
★ It moves the robot: run it yourself; space / x / q / Ctrl+C = stop.
Parameters: path_file (route csv), lane 2, tol_lat 0.03 m, tol_head 3 deg, max_lat 0.6, max_head 60, gait classic|none
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
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from unitree_api.msg import Request, Response

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import track_path as TP  # noqa: E402
import wall_align as WA  # noqa: E402

API_BALANCESTAND = 1002
API_STOPMOVE = 1003
API_CLASSICWALK = 2049


# ---------------------------------------------------------------------------------------------- the logic (no ROS)
def route_error(loop, tp):
    """(lateral + = left of the route, heading error = route direction - robot yaw, station) at the closest point."""
    lat, _, s = TP.lateral_of(loop, tp[0], tp[1])
    a, b = loop.pose_at(s), loop.pose_at(s + 0.5)                 # direction of the next 0.5 m (no kink)
    head = TP.wrap(math.atan2(b[1] - a[1], b[0] - a[0]) - tp[2])
    return lat, head, s


class Aligner:
    """turn -> settle -> side -> settle -> check (-> turn ...). tick() gives (vx, vy, wz) in the body frame, or None
    when finished (self.result says how)."""
    W_MIN, W_MAX = 0.30, 0.60        # rad/s: Go2 ignores tiny turn commands
    V_MIN, V_MAX = 0.12, 0.25        # m/s sideways
    SETTLE = 0.6                     # s standing still before measuring
    STEP_TIMEOUT = 8.0
    ROUNDS = 3

    def __init__(self, loop, tol_lat=0.03, tol_head_deg=3.0):
        self.loop, self.tol_lat, self.tol_head = loop, tol_lat, math.radians(tol_head_deg)
        self.step, self.t_step, self.next, self.round = 'check', None, None, 0
        self.result = None

    def go(self, step, t):
        self.step, self.t_step = step, t

    def tick(self, t, tp):
        if self.t_step is None:
            self.t_step = t
        lat, head, _ = route_error(self.loop, tp)
        dt = t - self.t_step
        if self.step == 'settle':
            if dt >= self.SETTLE:
                self.go(self.next, t)
            return 0.0, 0.0, 0.0
        if self.step == 'check':
            if abs(head) <= 1.5 * self.tol_head and abs(lat) <= 1.5 * self.tol_lat:
                self.result = 'aligned: lateral %+.3f m, heading %+.1f deg' % (lat, -math.degrees(head))
                return None
            if self.round >= self.ROUNDS:
                self.result = ('★ not better after %d rounds: lateral %+.3f m, heading %+.1f deg'
                               % (self.ROUNDS, lat, -math.degrees(head)))
                return None
            self.round += 1
            self.go('turn' if abs(head) > self.tol_head else 'side', t)
        if self.step == 'turn':
            if abs(head) <= self.tol_head or dt > self.STEP_TIMEOUT:
                self.next = 'side'
                self.go('settle', t)
                return 0.0, 0.0, 0.0
            w = min(self.W_MAX, max(self.W_MIN, 1.2 * abs(head)))
            return 0.0, 0.0, math.copysign(w, head)
        if self.step == 'side':
            if abs(lat) <= self.tol_lat or dt > self.STEP_TIMEOUT:
                self.next = 'check'
                self.go('settle', t)
                return 0.0, 0.0, 0.0
            # move by -lat along the route's left normal, expressed in the body frame (heading ~ route already)
            v = min(self.V_MAX, max(self.V_MIN, 1.2 * abs(lat)))
            return 0.0, -math.copysign(v, lat) * math.cos(head), 0.0
        return 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------------------------- the node
class StartAlign(Node):
    def __init__(self):
        super().__init__('start_align')

        def p(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        path = str(p('path_file', ''))
        if not path:
            raise SystemExit('path_file:=<route.csv> needed (scripts/day1_align.sh route=<name>)')
        self.lane = int(p('lane', 2))
        self.loop = TP.load_pins(path, self.lane)
        self.gait = str(p('gait', 'classic')).strip().lower() != 'none'
        self.odom_frame, self.base_frame = str(p('odom_frame', 'odom')), str(p('base_frame', 'base_link'))
        self.bridge = str(p('bridge_node', '/go2_cmd_vel_to_sport'))
        self.aligner = Aligner(self.loop, float(p('tol_lat', 0.03)), float(p('tol_head', 3.0)))
        self.wlock = TP.StartLock(self.lane, n=5, max_lat=float(p('max_lat', 0.6)), max_head_deg=float(p('max_head', 60.0)),
                                  loop=self.loop)
        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tf, self)
        self.pub_req = self.create_publisher(Request, '/api/sport/request', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
        self.create_subscription(Response, '/api/sport/response', self.cb_resp, 10)
        self.cli_bridge = self.create_client(ChangeState, self.bridge + '/change_state')
        self.req_id = int(time.time() * 1000) % 1000000000
        self.state = 'CHECK'          # the stack first (odom TF + motor bridge): nothing is sent to the robot before
        self.check_t0 = None
        self.gait_step, self.gait_t, self.gait_tries = 0, 0.0, 0
        self.sub_lidar = None
        self.n_frames = 0
        self.T_to = None
        self.t0 = self.now_s()
        self.done = False
        self.create_timer(0.05, self.tick)
        self.get_logger().info('start align on %s (%.2f m/lap) — space/x/q = stop' % (os.path.basename(path), self.loop.length))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sport(self, api_id, parameter='{}'):
        r = Request()
        self.req_id += 1
        r.header.identity.id = self.req_id
        r.header.identity.api_id = api_id
        r.parameter = parameter
        self.pub_req.publish(r)

    def twist(self, vx=0.0, vy=0.0, wz=0.0):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = float(vx), float(vy), float(wz)
        self.pub_cmd.publish(tw)

    def bridge_state(self, activate):
        if not self.cli_bridge.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('motor bridge lifecycle service not ready (day1.sh up?)')
            return False
        req = ChangeState.Request()
        req.transition.id = 3 if activate else 4
        self.cli_bridge.call_async(req)
        return True

    def pose_at(self):
        try:
            tr = self.tf.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception:
            return None
        q = tr.transform.rotation
        return (tr.transform.translation.x, tr.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def cb_resp(self, m):
        if m.header.identity.api_id == API_CLASSICWALK and self.state == 'GAIT' and self.gait_step == 2:
            if m.header.status.code == 0:
                self.get_logger().info('gait classic OK')
                self.state = 'LOCK'
                self.start_lock()
            else:
                self.get_logger().warn('gait classic refused (code %d) — BalanceStand again' % m.header.status.code)
                self.gait_step = 0

    def start_lock(self):
        self.sub_lidar = self.create_subscription(PointCloud2, '/lidar_points', self.cb_lidar, qos_profile_sensor_data)
        self.t0 = self.now_s()

    def cb_lidar(self, m):
        if self.state != 'LOCK':
            return
        self.n_frames += 1
        if self.n_frames % 2:
            return
        pose = self.pose_at()
        if pose is None:
            self.get_logger().warn('no odom TF (%s -> %s) — day1.sh up?' % (self.odom_frame, self.base_frame),
                                   throttle_duration_sec=2.0)
            return
        P = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        res = self.wlock.feed(WA.align([WA.lidar_to_base(P)], seed=self.n_frames), pose)
        if res is None:
            self.get_logger().info('wall lock: ' + self.wlock.why, throttle_duration_sec=1.0)
            return
        self.T_to = res['T_track_odom']
        self.destroy_subscription(self.sub_lidar)
        self.sub_lidar = None
        self.get_logger().info('start found: lateral %+.3f m, heading %+.1f deg (as the runner counts them)'
                               % (res['lateral'], math.degrees(res['heading_err'])))
        if self.bridge_state(True):
            self.state, self.t_act = 'ACT', self.now_s()
        else:
            self.finish('★ motor bridge not reachable — nothing moved')

    def tick(self):
        if self.done:
            return
        t = self.now_s()
        if self.state == 'CHECK':
            if self.check_t0 is None:
                self.check_t0 = t
            tf_ok = self.pose_at() is not None
            br_ok = self.cli_bridge.service_is_ready()
            if tf_ok and br_ok:
                self.get_logger().info('stack OK (odom TF, motor bridge)')
                self.state = 'GAIT' if self.gait else 'LOCK'
                if self.state == 'LOCK':
                    self.start_lock()
            elif t - self.check_t0 > 5.0:
                self.finish('★ %s — run dup first (after a robot reboot: dxt, dlight, dhoopon, dup). Nothing moved'
                            % ' and '.join(w for w, ok in (('no odom TF (FAST-LIO / odom_tf)', tf_ok),
                                                           ('no motor bridge', br_ok)) if not ok))
            return
        if self.state == 'GAIT':
            if t - self.gait_t < 0.5:
                return
            if self.gait_step == 0:
                self.gait_tries += 1
                if self.gait_tries > 3:
                    self.finish('★ gait refused 3 times — nothing moved')
                    return
                self.sport(API_BALANCESTAND)
                self.gait_step, self.gait_t = 1, t
            elif self.gait_step == 1:
                self.sport(API_CLASSICWALK, '{"data": true}')
                self.gait_step, self.gait_t = 2, t
            elif self.gait_step == 2 and t - self.gait_t > 1.5:
                self.get_logger().warn('no ClassicWalk response — retry')
                self.gait_step = 0
            return
        if self.state == 'LOCK':
            if t - self.t0 > 30.0:
                self.finish('★ no start in 30 s: %s — nothing moved' % self.wlock.why)
            return
        if self.state == 'ACT':                # give the bridge its activation
            self.twist()
            if t - self.t_act > 0.5:
                self.state = 'ALIGN'
            return
        if self.state == 'ALIGN':
            cur = self.pose_at()
            if cur is None:
                self.twist()
                return
            if t - self.t_act > 45.0:
                self.finish('★ took over 45 s — stopped')
                return
            tp = TP.compose(self.T_to, cur)
            cmd = self.aligner.tick(t, tp)
            if cmd is None:
                self.finish(self.aligner.result)
                return
            if self.aligner.step != getattr(self, '_last_step', None):
                lat, head, _ = route_error(self.loop, tp)
                self.get_logger().info('%s (round %d): lateral %+.3f m, heading %+.1f deg'
                                       % (self.aligner.step, self.aligner.round, lat, -math.degrees(head)))
                self._last_step = self.aligner.step
            self.twist(*cmd)

    def finish(self, why):
        if self.done:
            return
        self.done = True
        for _ in range(3):
            self.twist()
        if self.state in ('ACT', 'ALIGN'):
            self.sport(API_STOPMOVE)
            self.bridge_state(False)
        self.state = 'DONE'
        self.get_logger().info(why)
        self.get_logger().info('next: scripts/day1.sh run route=... (its start lock prints lateral / heading)')
        threading.Timer(0.5, lambda: rclpy.ok() and rclpy.shutdown()).start()


def key_loop(node):
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        return
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok() and not node.done:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r and sys.stdin.read(1) in (' ', 'x', 'X', 'q', 'Q'):
                node.finish('STOP (key)')
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = StartAlign()
    threading.Thread(target=key_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.finish('STOP (Ctrl+C)')
        time.sleep(0.3)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
