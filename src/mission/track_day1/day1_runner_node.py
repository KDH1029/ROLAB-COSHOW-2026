#!/usr/bin/env python3
"""Day-1 track race runner (NUC, ROS 2 Jazzy). Run it in your own terminal: space / x = emergency stop.

  camera edges (/track/edges, internal PC) --TF odom<-base_link at capture--> LanePath (odom frame)
  --> nav_msgs/Path --> Nav2 controller_server FollowPath (Regulated Pure Pursuit) --> /cmd_vel --> go2 bridge

States
  GAIT   BalanceStand -> ClassicWalk {"data": true}, wait for code 0 (the yaw trim is measured for ClassicWalk)
  LOCK   pool `init_frames` edge frames while standing, find where the lane is (+-0.35 m, +-12 deg)
  WAIT   keep refining the path; GREEN continuously for `green_s` and no RED within 1 s
  RUN    send the path to FollowPath (re-sent as it grows); lap distance from the odom TF
  FINISH path cut at one lap + extra; Nav2 stops at its end
  STOP   emergency (key), goal cancelled, bridge deactivated, StopMove
"""
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
from rcl_interfaces.msg import ParameterDescriptor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from unitree_api.msg import Request, Response

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import day1_logic as DL  # noqa: E402
import lane_path as LP  # noqa: E402
import track_spec as T  # noqa: E402

API_BALANCESTAND = 1002
API_STOPMOVE = 1003
API_CLASSICWALK = 2049


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Day1Runner(Node):
    def __init__(self):
        super().__init__('day1_runner')
        def p(name, default):   # numbers typed as 1 or 1.0 both work
            return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        self.lane = int(p('lane', 2).value)
        self.v = float(p('v', 0.5).value)
        self.extra = float(p('extra', 1.2).value)
        self.light = str(p('light', 'model').value)          # model | none (test only)
        self.green_s = float(p('green_s', 0.5).value)
        self.gait = bool(p('gait', True).value)
        self.init_frames = int(p('init_frames', 3).value)
        self.edges_topic = str(p('edges_topic', '/track/edges').value)
        self.odom_frame = str(p('odom_frame', 'odom').value)
        self.base_frame = str(p('base_frame', 'base_link').value)
        self.net_latency = float(p('net_latency', 0.003).value)
        self.bridge = str(p('bridge_node', '/go2_cmd_vel_to_sport').value)
        self.keys = bool(p('keys', True).value)
        self.coast_s = float(p('coast_s', 1.0).value)        # lane unseen this long: stop growing the path

        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tf, self)
        self.path = LP.LanePath(self.lane)
        self.gate = DL.LightGate(need_s=self.green_s)
        self.lap = DL.LapEnd(self.lane, extra=self.extra)
        self.state = 'GAIT' if self.gait else 'LOCK'
        self.init_pts = []
        self.gait_step = 0
        self.gait_t = 0.0
        self.gait_tries = 0
        self.req_id = int(time.time() * 1000) % 1000000000
        self.goal_handle = None
        self.goal_busy = False
        self.last_goal_t = 0.0
        self.t_go = None
        self.n_frames = self.n_ok = 0
        self.last_edge_t = None
        self.light_state = 'NONE'
        self.lock_res = None
        self.lock = threading.Lock()

        q_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, self.edges_topic, self.cb_edges, q_be)
        self.create_subscription(String, '/mission/traffic_light_state', self.cb_light, 10)
        self.create_subscription(Response, '/api/sport/response', self.cb_resp, 10)
        self.pub_req = self.create_publisher(Request, '/api/sport/request', 10)
        self.pub_path = self.create_publisher(Path, '/track/path', 2)
        self.pub_cam = self.create_publisher(Bool, '/mission/camera_active', 2)
        self.pub_status = self.create_publisher(String, '/track/status', 2)
        self.ac = ActionClient(self, FollowPath, 'follow_path')
        self.cli_bridge = self.create_client(ChangeState, self.bridge + '/change_state')
        self.cli_param = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.report)

        ok, msg = DL.speed_check(self.lane, self.v, self.extra)
        self.get_logger().info(('' if ok else '★ ') + msg)
        self.get_logger().info('lane %d · v %.2f · light %s · gait %s — space/x = STOP, q = quit'
                               % (self.lane, self.v, self.light, self.gait))
        self.set_rpp_speed(self.v)

    # ------------------------------------------------------------------ helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_at(self, t_s):
        try:
            tr = self.tf.lookup_transform(self.odom_frame, self.base_frame,
                                          Time(seconds=t_s) if t_s else Time(), timeout=Duration(seconds=0.0))
        except Exception:
            try:
                tr = self.tf.lookup_transform(self.odom_frame, self.base_frame, Time())
            except Exception:
                return None
            if t_s and abs(Time.from_msg(tr.header.stamp).nanoseconds * 1e-9 - t_s) > 0.3:
                return None
        tt = tr.transform
        return tt.translation.x, tt.translation.y, yaw_of(tt.rotation)

    def sport(self, api_id, parameter='{}'):
        r = Request()
        self.req_id += 1
        r.header.identity.id = self.req_id
        r.header.identity.api_id = api_id
        r.parameter = parameter
        self.pub_req.publish(r)

    def set_rpp_speed(self, v):
        if not self.cli_param.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('controller_server parameters not reachable — RPP keeps its yaml speed')
            return
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name='FollowPath.desired_linear_vel',
                                   value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))]
        self.cli_param.call_async(req)

    def bridge_state(self, activate):
        if not self.cli_bridge.service_is_ready():
            self.get_logger().warn('motor bridge lifecycle service not ready')
            return
        req = ChangeState.Request()
        req.transition.id = 3 if activate else 4      # ACTIVATE / DEACTIVATE
        self.cli_bridge.call_async(req)

    # ------------------------------------------------------------------ callbacks
    def cb_light(self, m):
        st = m.data.split('|')[0].strip().upper()
        self.light_state = st
        self.gate.feed(st, self.now_s())

    def cb_resp(self, m):
        if m.header.identity.api_id == API_CLASSICWALK and self.state == 'GAIT' and self.gait_step == 2:
            if m.header.status.code == 0:
                self.get_logger().info('ClassicWalk OK')
                self.state = 'LOCK'
            else:
                self.get_logger().warn('ClassicWalk refused (code %d) — BalanceStand again' % m.header.status.code)
                self.gait_step = 0

    def cb_edges(self, m):
        recv = self.now_s()
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.n_frames += 1
        if not d.get('ok') or not d.get('pts'):
            return
        pts = np.asarray(d['pts'], np.float64).reshape(-1, 4)
        t_cap = recv - float(d.get('age', 0.05)) - self.net_latency
        pose = self.pose_at(t_cap)
        if pose is None:
            return
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        P = pts.copy()
        P[:, 0] = x + c * pts[:, 0] - s * pts[:, 1]
        P[:, 1] = y + s * pts[:, 0] + c * pts[:, 1]
        P[:, 2] = pts[:, 2] + yaw
        with self.lock:
            if self.state == 'LOCK':
                self.init_pts.append(P)
                if len(self.init_pts) >= self.init_frames:
                    kap = (0.0, T.CURVATURE[self.lane])
                    res = self.path.initialize(np.vstack(self.init_pts), x, y, yaw, kappas=kap)
                    self.init_pts = []
                    if res is not None:
                        self.lock_res = res
                        self.get_logger().info('lane found: centre %+.2f m, heading %+.1f deg, curvature %.2f, %d pts'
                                               % (res[0], math.degrees(res[1]), res[2], res[3]))
                        self.state = 'WAIT'
                        self.last_edge_t = recv
                        self.pub_cam.publish(Bool(data=True))
                        self.bridge_state(True)
                        self.set_rpp_speed(self.v)
                    else:
                        self.get_logger().warn('lane not found in %d frames — is the robot in lane %d?'
                                               % (self.init_frames, self.lane))
            elif self.state in ('WAIT', 'RUN'):
                cur = self.pose_at(None)
                s_r = self.path.robot_station(cur[0], cur[1])[0] if cur else None
                if self.path.update(P, s_r):
                    self.n_ok += 1
                    self.last_edge_t = recv

    # ------------------------------------------------------------------ main loop
    def tick(self):
        t = self.now_s()
        if self.state == 'GAIT':
            if t - self.gait_t < 0.5:
                return
            if self.gait_step == 0:
                self.gait_tries += 1
                if self.gait_tries > 3:
                    self.get_logger().error('ClassicWalk refused 3 times — stopping (yaw trim is for ClassicWalk)')
                    self.state = 'STOP'
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
        cur = self.pose_at(None)
        if cur is None:
            return
        with self.lock:
            if self.path.pts is not None:
                fresh = self.last_edge_t is not None and t - self.last_edge_t < self.coast_s
                self.path.advance(cur[0], cur[1], extend=fresh)
            if self.state == 'WAIT':
                go = self.gate.go(t) if self.light != 'none' else True
                if go:
                    self.state, self.t_go = 'RUN', t
                    self.lap.feed(cur[0], cur[1])
                    self.pub_cam.publish(Bool(data=False))
                    self.get_logger().info('GREEN — go')
            elif self.state == 'RUN':
                rem = self.lap.feed(cur[0], cur[1])
                if rem < LP.DEFAULTS['ahead']:
                    self.path.truncate_ahead(cur[0], cur[1], rem)
                if rem < 0.25:      # the path ends here; RPP stops within its goal tolerance
                    self.state = 'FINISH'
                    self.get_logger().info('lap done: %.2f m in %.1f s (odom)' % (self.lap.dist, t - self.t_go))
            pts = None if self.path.pts is None else self.path.pts.copy()
        if pts is None:
            return
        msg = self.make_path(pts)
        self.pub_path.publish(msg)
        if self.state == 'RUN' and (t - self.last_goal_t > 0.2) and not self.goal_busy:
            self.send_goal(msg)
            self.last_goal_t = t
        if self.state == 'FINISH' and (t - self.last_goal_t > 0.2) and not self.goal_busy and self.goal_handle is None:
            self.send_goal(msg)        # final, truncated path: RPP stops at its end
            self.last_goal_t = t

    def make_path(self, pts):
        msg = Path()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        d = np.diff(pts, axis=0)
        th = np.arctan2(d[:, 1], d[:, 0])
        th = np.concatenate([th, th[-1:]])
        for (x, y), a in zip(pts, th):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = math.sin(a / 2)
            ps.pose.orientation.w = math.cos(a / 2)
            msg.poses.append(ps)
        return msg

    def send_goal(self, path_msg):
        if not self.ac.server_is_ready():
            self.get_logger().warn('follow_path action not ready (controller_server up?)', throttle_duration_sec=2.0)
            return
        g = FollowPath.Goal()
        g.path = path_msg
        g.controller_id = 'FollowPath'
        g.goal_checker_id = 'goal_checker'
        self.goal_busy = True
        fut = self.ac.send_goal_async(g)
        fut.add_done_callback(self._goal_sent)

    def _goal_sent(self, fut):
        self.goal_busy = False
        h = fut.result()
        if h is None or not h.accepted:
            self.get_logger().warn('FollowPath goal rejected')
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        st = fut.result().status
        if self.state == 'FINISH' and st == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('stopped at the lap end')
            self.state = 'DONE'
            self.sport(API_STOPMOVE)

    def stop(self, why):
        self.state = 'STOP'
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.bridge_state(False)
        self.sport(API_STOPMOVE)
        self.get_logger().warn('STOP (%s)' % why)

    def report(self):
        cur = self.pose_at(None)
        off = '-'
        if cur is not None and self.path.pts is not None:
            off = '%+.3f' % self.path.robot_station(cur[0], cur[1])[1]
        L = self.path.last
        line = '%s | off %s m · pts %s %s rms %s · frames %d ok %d · light %s · lap %.1f/%.1f m' % (
            self.state, off, L['n'], L['per'], '%.3f' % L['rms'] if L['rms'] is not None else '-',
            self.n_frames, self.n_ok, self.light_state, self.lap.dist, self.lap.total)
        self.get_logger().info(line)
        self.pub_status.publish(String(data=json.dumps(dict(state=self.state, off=off, n=L['n'], lap=self.lap.dist))))


def key_loop(node):
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        return
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch in (' ', 'x', 'X'):
                node.stop('key')
            elif ch in ('q', 'Q'):
                node.stop('quit')
                time.sleep(0.3)
                rclpy.shutdown()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = Day1Runner()
    if node.keys:
        threading.Thread(target=key_loop, args=(node,), daemon=True).start()
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
