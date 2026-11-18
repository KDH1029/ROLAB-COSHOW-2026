#!/usr/bin/env python3
"""Day-1 track race runner (NUC, ROS 2 Jazzy). Run it in your own terminal: space / x = emergency stop.

mode wall (default): no camera, no map.
  XT16 /lidar_points while standing --wall_align (0.6 m rectangular wall)--> robot pose in the track frame
  + odom TF (FAST-LIO /Odometry via odom_tf) at that moment = track <- odom, fixed for the run
  rulebook lane centre (track_path) --> odom frame --> nav_msgs/Path --> Nav2 FollowPath (RPP) --> /cmd_vel --> bridge
mode vision (spare): camera edges (/track/edges, internal PC) --> LanePath (odom frame) --> the same Nav2 / bridge.

States
  GAIT   BalanceStand -> ClassicWalk {"data": true}, wait for code 0 (the yaw trim is measured for ClassicWalk)
  LOCK   wall: `lock_n` steady wall poses, lane and placement checked (+-max_lat, +-max_head)
         vision: pool `init_frames` edge frames, find where the lane is (+-0.35 m, +-12 deg)
  WAIT   GREEN continuously for `green_s` and no RED within 1 s
  RUN    send the path to FollowPath (re-sent every 0.2 s from the robot onwards); lap = station along the lane
  FINISH path ends at one lap + extra; Nav2 stops at its end
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from unitree_api.msg import Request, Response

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import day1_logic as DL  # noqa: E402
import lane_path as LP  # noqa: E402
import track_path as TP  # noqa: E402
import track_spec as T  # noqa: E402
import wall_align as WA  # noqa: E402

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
        lane = str(p('lane', 2).value).strip().lower()       # 1 / out = outer lane, 2 / in = inner lane
        self.lane = {'1': 1, 'out': 1, 'outer': 1, '2': 2, 'in': 2, 'inner': 2}.get(lane)
        if self.lane is None:
            raise SystemExit('lane must be in (2) or out (1), got %r' % lane)
        self.mode = str(p('mode', 'wall').value)               # wall | vision
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
        self.lidar_topic = str(p('lidar_topic', '/lidar_points').value)
        self.lock_n = int(p('lock_n', 5).value)
        self.max_lat = float(p('max_lat', 0.35).value)       # m from the lane centre at the start
        self.max_head = float(p('max_head', 15.0).value)     # deg
        self.blend_m = float(p('blend_m', 2.0).value)        # ease from the start offset onto the centre
        # drawn track centre vs wall-rectangle centre, start-relative frame (README: from the 4 gaps to the wall)
        self.track_off = (float(p('track_dx', 0.0).value), float(p('track_dy', 0.0).value),
                          math.radians(float(p('track_dyaw_deg', 0.0).value)))
        self.path_file = str(p('path_file', '').value)       # '' = rulebook lane centre
        if self.mode not in ('wall', 'vision'):
            raise SystemExit('mode must be wall or vision')

        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tf, self)
        self.path = LP.LanePath(self.lane)          # vision mode
        self.loop = TP.load_pins(self.path_file, self.lane) if self.path_file else TP.RulebookLoop(self.lane)
        self.wlock = TP.StartLock(self.lane, n=self.lock_n, max_lat=self.max_lat, max_head_deg=self.max_head,
                                  loop=self.loop, offset=self.track_off)
        self.plan = self.prog = self.T_to = self.T_ot = None   # wall mode: plan in the track frame, track <- odom
        self.sub_lidar = None
        self.lock_t0 = None
        self.cam_pub_t = 0.0      # camera_active is re-sent every second (a single message can be lost)
        self.force_go = False     # key g while waiting: start without the light
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
        if self.mode == 'vision':
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
        self.get_logger().info('lane %d (%s course, counter-clockwise, one lap + %.1f m) · v %.2f · mode %s · light %s · gait %s'
                               ' — space/x = STOP, g = 강제 출발(신호등 대기 중), q = quit'
                               % (self.lane, 'in' if self.lane == 2 else 'out', self.extra, self.v, self.mode, self.light, self.gait))
        self.set_rpp_speed(self.v)
        if self.mode == 'wall':
            self.get_logger().info('path: %s, %.2f m/lap (rulebook %.2f) · track offset dx %+.3f dy %+.3f m dyaw %+.2f deg'
                                   % (self.loop.name, self.loop.length, T.LAP_LEN[self.lane], self.track_off[0],
                                      self.track_off[1], math.degrees(self.track_off[2])))
        if self.state == 'LOCK':
            self.start_lock()

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
                self.start_lock()
            else:
                self.get_logger().warn('ClassicWalk refused (code %d) — BalanceStand again' % m.header.status.code)
                self.gait_step = 0

    def start_lock(self):
        self.lock_t0 = self.now_s()
        if self.mode == 'wall' and self.sub_lidar is None:
            # only while standing: a 32k-point cloud at 20 Hz is too much for Python while driving
            self.sub_lidar = self.create_subscription(PointCloud2, self.lidar_topic, self.cb_lidar, qos_profile_sensor_data)
            self.get_logger().info('wall lock: listening to %s' % self.lidar_topic)

    def cb_lidar(self, m):
        if self.state != 'LOCK' or self.plan is not None:
            return
        self.n_frames += 1
        if self.n_frames % 2:          # 10 Hz is plenty, and wall_align takes ~60 ms
            return
        pose = self.pose_at(None)      # standing still: the latest odom pose is the pose of the scan
        if pose is None:
            self.get_logger().warn('wall lock: no odom TF (%s -> %s)' % (self.odom_frame, self.base_frame),
                                   throttle_duration_sec=2.0)
            return
        P = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        r = WA.align([WA.lidar_to_base(P)], seed=self.n_frames)
        if r.get('ok'):
            self.n_ok += 1
        res = self.wlock.feed(r, pose)
        if res is None:
            self.get_logger().info('wall lock: ' + self.wlock.why, throttle_duration_sec=1.0)
            return
        with self.lock:
            self.T_to = res['T_track_odom']
            self.T_ot = TP.inverse(self.T_to)
            self.plan = TP.LanePlan(self.lane, res['s'], res['lateral'], extra=self.extra, blend_m=self.blend_m,
                                    loop=self.loop)
            self.prog = TP.Progress(self.lane, res['s'], loop=self.loop)
            self.lock_res = res
            self.state = 'WAIT'
        self.get_logger().info('start found: lane %d, station %.2f m, lateral %+.3f m, heading %+.1f deg '
                               '(walls %d, dims %.2f x %.2f, rms %.3f) — path %.1f m'
                               % (self.lane, res['s'], res['lateral'], math.degrees(res['heading_err']),
                                  r['walls_seen'], r['dims'][0], r['dims'][1], r['rms'],
                                  self.plan.s_end - self.plan.s_start))
        self.destroy_subscription(self.sub_lidar)
        self.sub_lidar = None
        threading.Thread(target=self.save_plan_png, args=(res,), daemon=True).start()
        self.pub_cam.publish(Bool(data=True))
        self.bridge_state(True)
        self.set_rpp_speed(self.v)

    def save_plan_png(self, res):
        """Picture of this run: start and end on the lane centre line, the robot where it actually stands."""
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))
            import plot_plan
            root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))
            out = os.path.join(os.environ.get('DAY1_LOG', os.path.join(root, 'logs/day1')),
                               'plan_%s.png' % time.strftime('%H%M%S'))
            plot_plan.save_plan(out, self.lane, res['s'], self.extra, self.loop, res['pose'], res['lateral'], self.blend_m,
                                title='%s코스 (%d레인): 출발 station %.2f m, 옆 %+.2f m, 방향 %+.1f°' % (
                                    '인' if self.lane == 2 else '아웃', self.lane, res['s'], res['lateral'],
                                    math.degrees(res['heading_err'])))
            self.get_logger().info('plan picture: %s' % out)
        except Exception as e:      # a picture must never stop the run
            self.get_logger().warn('plan picture failed: %s' % e)

    def track_pose(self, cur):
        return TP.compose(self.T_to, cur)

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
        if self.mode == 'wall':
            self.tick_wall(t, cur)
            return
        with self.lock:
            if self.path.pts is not None:
                fresh = self.last_edge_t is not None and t - self.last_edge_t < self.coast_s
                self.path.advance(cur[0], cur[1], extend=fresh)
            if self.state == 'WAIT':
                go = self.force_go or (self.gate.go(t) if self.light != 'none' else True)
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

    def tick_wall(self, t, cur):
        with self.lock:
            if self.plan is None:
                return
            tp = self.track_pose(cur)
            if self.state == 'WAIT':
                s_r = self.plan.s_start
                if t - self.cam_pub_t > 1.0:            # light node: judge the light now
                    self.pub_cam.publish(Bool(data=True))
                    self.cam_pub_t = t
                go = self.force_go or (self.gate.go(t) if self.light != 'none' else True)
                if go:
                    self.state, self.t_go = 'RUN', t
                    self.pub_cam.publish(Bool(data=False))
                    self.get_logger().info('GREEN — go')
            elif self.state in ('RUN', 'FINISH'):
                if self.state == 'RUN' and t - self.t_go < 3.0 and t - self.cam_pub_t > 1.0:
                    self.pub_cam.publish(Bool(data=False))   # light no longer needed (repeat: one can be lost)
                    self.cam_pub_t = t
                s_r = self.prog.feed(tp[0], tp[1])
                if self.state == 'RUN' and self.plan.s_end - s_r < 0.25:
                    self.state = 'FINISH'
                    self.get_logger().info('lap done: %.2f m along the lane in %.1f s'
                                           % (s_r - self.plan.s_start, t - self.t_go))
            else:
                s_r = self.prog.s
            pts = TP.apply(self.T_ot, self.plan.points(s_r - 0.3))
        msg = self.make_path(pts)
        self.pub_path.publish(msg)
        if self.state == 'RUN' and (t - self.last_goal_t > 0.2) and not self.goal_busy:
            self.send_goal(msg)
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
        if self.mode == 'wall':
            if self.state == 'LOCK' and self.plan is None and self.lock_t0 is not None and self.now_s() - self.lock_t0 > 5.0:
                self.get_logger().warn('★ no start yet after %.0f s: %s' % (self.now_s() - self.lock_t0, self.wlock.why),
                                       throttle_duration_sec=5.0)
            if self.state == 'WAIT' and self.light != 'none' and self.now_s() - self.gate.last_msg > 3.0:
                self.get_logger().warn('★ 신호등 메시지 없음 (/mission/traffic_light_state) — day1.sh light 로 내부 PC YOLO 를 켰나요? '
                                       '초록을 봐야만 출발합니다 (g = 강제 출발)', throttle_duration_sec=3.0)
            lat, s_done = '-', 0.0
            if cur is not None and self.plan is not None:
                tp = self.track_pose(cur)
                lat = '%+.3f' % TP.lateral_of(self.loop, tp[0], tp[1])[0]
                s_done = self.prog.s - self.plan.s_start
            total = (self.plan.s_end - self.plan.s_start) if self.plan else 0.0
            self.get_logger().info('%s | lateral %s m · lap %.1f/%.1f m · scans %d ok %d · light %s'
                                   % (self.state, lat, s_done, total, self.n_frames // 2, self.n_ok, self.light_state))
            self.pub_status.publish(String(data=json.dumps(dict(state=self.state, off=lat, lap=s_done))))
            return
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
            elif ch in ('g', 'G') and node.state == 'WAIT':
                node.force_go = True
                node.get_logger().warn('★ g — 신호등 없이 강제 출발')
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
