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
import atexit
import json
import math
import os
import random
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
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
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
# walking gaits (official ros2_sport_client): name -> (api_id, parameter)
GAITS = {'classic': (2049, '{"data": true}'), 'freewalk': (2045, '{}'), 'trotrun': (1062, '{}'),
         'static': (1061, '{}'), 'economic': (1063, '{}')}


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
        # debug: practice / tests only. Without it (competition) nobody picks the light or the hoop at the look point
        # by hand: no g / 1 / 2 keys, no light=none. (hoop=in|out, given before the run, is allowed in both)
        self.debug = str(p('debug', False).value).strip().lower() in ('true', '1', 'yes')
        self.green_s = float(p('green_s', 0.5).value)
        g = str(p('gait', 'classic').value).strip().lower()         # classic | freewalk | trotrun | static | economic | none
        g = {'true': 'classic', '1': 'classic', 'false': 'none', '0': 'none'}.get(g, g)
        if g != 'none' and g not in GAITS:
            raise SystemExit('gait must be one of %s or none, got %r' % (', '.join(GAITS), g))
        self.gait_name = g
        self.gait = g != 'none'
        self.gait_api, self.gait_param = GAITS.get(g, (0, '{}'))
        self.lookahead = float(p('lookahead', 0.0).value)        # m, 0 = keep the yaml (0.8)
        self.min_radius = float(p('min_radius', 0.0).value)      # m, RPP slows below this radius; 0 = yaml (0.9)
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
        # hula hoop: the colour given before the run; at the zone file's look point the robot stops, faces the hoops,
        # asks the hoop vision once and continues on the route that goes through that colour's hoop
        self.hoop = str(p('hoop', '').value).strip().lower()      # '' | red | yellow | in | out
        if not self.debug:
            if self.light == 'none':
                raise SystemExit('light=none needs debug=true (competition: the start must come from the light)')
        self.look_timeout = float(p('look_timeout', 10.0).value)   # s without the hoop vision, then a random branch
        self.light_timeout = float(p('light_timeout', 0.0).value)   # s waiting for GREEN, then go anyway; 0 = never (default, 11-20)
        # this run only, the zones file unchanged (11-20 speed tests): zigzag_v / cones_v m/s, 0 = the file's value
        self.zone_v = {'zigzag': float(p('zigzag_v', 0.0).value), 'cones': float(p('cones_v', 0.0).value)}
        self.wait_t = None
        self.finish_t = None
        if self.mode not in ('wall', 'vision'):
            raise SystemExit('mode must be wall or vision')

        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tf, self)
        self.path = LP.LanePath(self.lane)          # vision mode
        self.loop = TP.load_pins(self.path_file, self.lane) if self.path_file else TP.RulebookLoop(self.lane)
        # zones next to the route (<route>.zones.json, scripts/route_corridor.py): other driving inside a box
        self.zones, self.zone = [], None
        zf = self.path_file[:-4] + '.zones.json' if self.path_file.endswith('.csv') else ''
        if zf and os.path.exists(zf):
            self.zones = json.load(open(zf))
        self.apply_zone_v()
        # hoop=in|out: the branch is known before the run — take that route now, no stop, no vision
        if self.hoop in ('in', 'inner', 'out', 'outer'):
            lz = next((z for z in self.zones if z.get('type') == 'look'), None)
            if lz is None:
                raise SystemExit('hoop=%s but %s has no hula-hoop look point' % (self.hoop, os.path.basename(self.path_file)))
            # inner = the hoop nearer the track centre
            order = sorted(lz['branches'], key=lambda n: math.hypot(*lz['branches'][n]))
            pick = order[0] if self.hoop in ('in', 'inner') else order[-1]
            f = os.path.join(os.path.dirname(self.path_file), pick + '.csv')
            if os.path.abspath(f) != os.path.abspath(self.path_file):
                self.path_file = f
                self.loop = TP.load_pins(f, self.lane)
                zf = f[:-4] + '.zones.json'
                self.zones = json.load(open(zf)) if os.path.exists(zf) else []
                self.apply_zone_v()
            self.look_done = True
            self.get_logger().info('hoop %s: route %s from the start (no look, no vision)' % (self.hoop, pick))
        self.wlock = TP.StartLock(self.lane, n=self.lock_n, max_lat=self.max_lat, max_head_deg=self.max_head,
                                  loop=self.loop, offset=self.track_off)
        self.plan = self.prog = self.T_to = self.T_ot = None   # wall mode: plan in the track frame, track <- odom
        self.sub_lidar = None
        self.lock_t0 = None
        self.cam_pub_t = 0.0      # camera_active is re-sent every second (a single message can be lost)
        self.force_go = False     # key g while waiting: start without the light
        self.look = None          # pending look point (zone of type 'look'), its station, sub-step
        self.look_done = self.hoop in ('in', 'inner', 'out', 'outer')   # branch fixed before the run: no look
        self.hoop_seen = []       # (time, world bearing of the wanted colour)
        self.hoop_pick = None     # key 1 / 2 while looking
        self.legs = None          # narrow corridor: corner-to-corner driving (zone 'corners', mode 'legs')
        self.legs_done = set()
        self.last_rot = None
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
        # hoop vision contract: /mission/hoop_active Bool (True while looking, re-sent 1 Hz) ->
        # /mission/hoop_state String JSON {"red": {"bearing_deg": +left, "conf": 0..1}, "yellow": {...}}
        # (a colour may be missing; "side": "left"/"right" instead of bearing_deg also works)
        self.pub_hoop = self.create_publisher(Bool, '/mission/hoop_active', 2)
        self.create_subscription(String, '/mission/hoop_state', self.cb_hoop, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 5)
        self.pub_status = self.create_publisher(String, '/track/status', 2)
        self.ac = ActionClient(self, FollowPath, 'follow_path')
        self.cli_bridge = self.create_client(ChangeState, self.bridge + '/change_state')
        self.cli_param = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.report)

        ok, msg = DL.speed_check(self.lane, self.v, self.extra)
        self.get_logger().info(('' if ok else '★ ') + msg)
        self.get_logger().info(('★ DEBUG — g / 1 / 2 keys and light=none allowed (not for the competition)'
                                if self.debug else 'competition mode — light and hoop only from the vision'))
        keys = ' — space/x = STOP, q = quit' + (', g = 강제 출발, 1 / 2 = 훌라후프 갈래 (debug)' if self.debug else '')
        self.get_logger().info('lane %d (%s course, counter-clockwise, one lap + %.1f m) · v %.2f · mode %s · light %s · gait %s%s'
                               % (self.lane, 'in' if self.lane == 2 else 'out', self.extra, self.v, self.mode, self.light,
                                  self.gait_name, keys))
        self.bench = dict(lat=[], rp=[], max_vx=None)
        self.cli_bget = self.create_client(GetParameters, self.bridge + '/get_parameters')
        self.ask_bridge_limit()
        self.set_rpp_speed(self.v)
        if self.mode == 'wall':
            self.get_logger().info('path: %s, %.2f m/lap (rulebook %.2f) · track offset dx %+.3f dy %+.3f m dyaw %+.2f deg'
                                   ' · RPP lookahead %s min_radius %s · zones %s'
                                   % (self.loop.name, self.loop.length, T.LAP_LEN[self.lane], self.track_off[0],
                                      self.track_off[1], math.degrees(self.track_off[2]),
                                      self.lookahead or 'yaml', self.min_radius or 'yaml',
                                      ', '.join(z['name'] for z in self.zones) or 'none'))
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
        self.last_rot = tt.rotation
        return tt.translation.x, tt.translation.y, yaw_of(tt.rotation)

    def sport(self, api_id, parameter='{}'):
        r = Request()
        self.req_id += 1
        r.header.identity.id = self.req_id
        r.header.identity.api_id = api_id
        r.parameter = parameter
        self.pub_req.publish(r)

    def set_rpp(self, values):
        """FollowPath parameters by name (float or bool), fire and forget."""
        if not self.cli_param.service_is_ready():
            self.get_logger().warn('controller_server parameters not reachable — zone settings not applied')
            return
        req = SetParameters.Request()
        for k, v in values.items():
            pv = (ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v) if isinstance(v, bool)
                  else ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
            req.parameters.append(ParamMsg(name='FollowPath.' + k, value=pv))
        self.cli_param.call_async(req)

    def apply_zone_v(self):
        """zigzag_v / cones_v given on the command line: that zone's speed for this run (the file keeps its value)."""
        for z in self.zones:
            v = self.zone_v.get(z.get('name'), 0.0)
            if v > 0 and z.get('v') != v:
                self.get_logger().info('★ zone %s: v %.2f for this run (file: %s)' % (z['name'], v, z.get('v')))
                z['v'] = v

    def zone_at(self, x, y, margin=0.3):
        for z in self.zones:
            if 'box' not in z:          # a look point is not a driving zone
                continue
            b = z['box']
            if b[0] - margin < x < b[2] + margin and b[1] - margin < y < b[3] + margin:
                return z
        return None

    def zone_switch(self, z):
        """Entering a zone: its speed, look-ahead and turning in place at corners. Leaving: the run's own settings."""
        if z is self.zone:
            return
        if z is not None:
            self.set_rpp({'desired_linear_vel': min(self.v, z.get('v', self.v)),
                          'lookahead_dist': z.get('lookahead', 0.3),
                          'use_rotate_to_heading': bool(z.get('rotate', True)),
                          'rotate_to_heading_min_angle': math.radians(z.get('rotate_min_deg', 35.0))})
            self.get_logger().info('★ zone %s: v %.2f, look-ahead %.2f m, turn in place over %.0f deg'
                                   % (z['name'], min(self.v, z.get('v', self.v)), z.get('lookahead', 0.3), z.get('rotate_min_deg', 35.0)))
        else:
            self.set_rpp({'desired_linear_vel': self.v, 'lookahead_dist': self.lookahead if self.lookahead > 0 else 0.8,
                          'use_rotate_to_heading': False, 'rotate_to_heading_min_angle': 0.785})
            self.get_logger().info('zone %s left: back to v %.2f' % (self.zone['name'], self.v))
        self.zone = z

    def set_rpp_speed(self, v):
        if not self.cli_param.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('controller_server parameters not reachable — RPP keeps its yaml speed')
            return
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name='FollowPath.desired_linear_vel',
                                   value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))]
        # always the run's own look-ahead and no turning in place: a run stopped inside a zone leaves its settings
        # in controller_server (its parameters outlive the runner)
        req.parameters.append(ParamMsg(name='FollowPath.lookahead_dist', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.lookahead if self.lookahead > 0 else 0.8))))
        req.parameters.append(ParamMsg(name='FollowPath.use_rotate_to_heading', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=False)))
        req.parameters.append(ParamMsg(name='FollowPath.rotate_to_heading_min_angle', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=0.785)))
        if self.min_radius > 0:
            req.parameters.append(ParamMsg(name='FollowPath.regulated_linear_scaling_min_radius', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.min_radius))))
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

    def regait_tick(self, t):
        """After the look point: BalanceStand, then the run's gait again, then on (still standing meanwhile)."""
        g = self.regait
        self.pub_cmd.publish(Twist())
        if g['step'] == 0:
            self.sport(API_BALANCESTAND)
            g.update(step=1, t=t)
        elif g['step'] == 1 and t - g['t'] > 0.5:
            self.sport(self.gait_api, self.gait_param)
            g.update(step=2, t=t)
        elif g['step'] == 2 and (g['ok'] is not None or t - g['t'] > 1.5):
            if g['ok'] is False and g['tries'] < 2:
                g.update(step=0, t=t, tries=g['tries'] + 1, ok=None)
                return
            self.get_logger().info('gait %s again after the look point: %s' % (
                self.gait_name, {True: 'OK', False: '★ refused', None: 'no answer (sent)'}[g['ok']]))
            self.state = 'RUN'
            self.last_goal_t = 0.0

    def cb_resp(self, m):
        if (m.header.identity.api_id == self.gait_api and self.state == 'REGAIT'
                and getattr(self, 'regait', None) and self.regait['step'] == 2):
            self.regait['ok'] = m.header.status.code == 0
            return
        if m.header.identity.api_id == self.gait_api and self.state == 'GAIT' and self.gait_step == 2:
            if m.header.status.code == 0:
                self.get_logger().info('gait %s (%d) OK' % (self.gait_name, self.gait_api))
                self.state = 'LOCK'
                self.start_lock()
            else:
                self.get_logger().warn('gait %s refused (code %d) — BalanceStand again' % (self.gait_name, m.header.status.code))
                self.gait_step = 0

    def ask_bridge_limit(self):
        """Motor bridge max_vx: it clips vx but not wz, so a v above it over-steers on curves -> RPP speed = min(v, max_vx)."""
        if not self.cli_bget.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('motor bridge parameters not reachable — v %.2f used as is' % self.v)
            return
        req = GetParameters.Request()
        req.names = ['max_vx']

        def done(fut):
            try:
                mx = fut.result().values[0].double_value
            except Exception:
                return
            self.bench['max_vx'] = mx
            if mx > 0 and self.v > mx:
                self.get_logger().warn('★ v %.2f > bridge max_vx %.2f: the bridge would clip speed but not turning '
                                       '(over-steer, 11-18) -> v = %.2f. Faster: MAX_VX=... day1.sh up' % (self.v, mx, mx))
                self.v = mx
                self.set_rpp_speed(self.v)
        self.cli_bget.call_async(req).add_done_callback(done)

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
            self.look_setup()
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

    def bench_report(self, dist, dur):
        """One line per run in logs/day1/bench.csv (tools/bench_table.py makes the table)."""
        lat = np.abs(np.array(self.bench['lat'] or [0.0]))
        rp = np.degrees(np.array(self.bench['rp'] or [(0.0, 0.0)]))
        res = self.lock_res or {}
        row = dict(time=time.strftime('%m-%d %H:%M:%S'), gait=self.gait_name, lane=self.lane, v=self.v,
                   max_vx=self.bench['max_vx'], lap_s=round(dur, 2), dist_m=round(dist, 2), mean_mps=round(dist / max(dur, 1e-3), 3),
                   lat_max=round(float(lat.max()), 3), lat_mean=round(float(lat.mean()), 3),
                   roll_std=round(float(rp[:, 0].std()), 2), pitch_std=round(float(rp[:, 1].std()), 2),
                   start_lat=round(float(res.get('lateral', 0.0)), 3), start_head=round(math.degrees(res.get('heading_err', 0.0)), 1))
        self.get_logger().info('BENCH ' + json.dumps(row))
        try:
            root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))
            path = os.path.join(os.environ.get('DAY1_LOG', os.path.join(root, 'logs/day1')), 'bench.csv')
            new = not os.path.exists(path)
            with open(path, 'a') as f:
                if new:
                    f.write(','.join(row) + '\n')
                f.write(','.join(str(v) for v in row.values()) + '\n')
        except Exception as e:
            self.get_logger().warn('bench.csv: %s' % e)

    def cb_hoop(self, m):
        if self.look is None or self.look['step'] != 'see' or not self.hoop:
            return
        try:
            d = json.loads(m.data).get(self.hoop)
        except (ValueError, AttributeError):
            return
        if not d:
            return
        if 'bearing_deg' in d:
            self.hoop_seen.append((self.now_s(), self.look['yaw_now'] + math.radians(float(d['bearing_deg']))))
        elif d.get('side') in ('left', 'right'):
            self.hoop_seen.append((self.now_s(), d['side']))

    def look_setup(self):
        """The first 'look' zone of the route, if a hoop colour was given: where to stop (station) and the branches."""
        if not self.hoop or self.look_done:
            if not self.hoop and any(z.get('type') == 'look' for z in self.zones):
                self.get_logger().warn('★ this route has a hula-hoop look point but no hoop=red|yellow was given — '
                                       'no look, staying on %s' % os.path.basename(self.path_file))
            return
        for z in self.zones:
            if z.get('type') == 'look' and self.hoop in ('red', 'yellow'):
                ax, ay = z['at']
                s = self.loop.station(ax, ay)
                s += math.ceil((self.plan.s_start + 0.5 - s) / self.loop.length) * self.loop.length   # ahead of the start
                self.look = dict(zone=z, s=s, step='go', t=0.0, yaw_now=0.0)
                self.get_logger().info('hoop %s: look point (%.2f, %.2f) at %.1f m, face %.0f deg, branches %s'
                                       % (self.hoop, ax, ay, s - self.plan.s_start, z['yaw_deg'], ', '.join(z['branches'])))
                return

    def look_decide(self):
        """Branch whose hoop lies in the seen direction (or on the seen side); None if nothing yet."""
        z = self.look['zone']
        names = list(z['branches'])
        if self.hoop_pick is not None:
            return names[min(self.hoop_pick, len(names) - 1)], 'key'
        if len(self.hoop_seen) < 3:
            return None, ''
        x, y, _ = self.look['pose']
        bear = {n: math.atan2(p[1] - y, p[0] - x) for n, p in z['branches'].items()}
        seen = [v for _, v in self.hoop_seen[-5:]]
        if all(isinstance(v, float) for v in seen):
            b = math.atan2(np.mean(np.sin(seen)), np.mean(np.cos(seen)))
            n = min(bear, key=lambda k: abs(TP.wrap(bear[k] - b)))
            return n, 'bearing %.0f deg (hoops at %s)' % (math.degrees(b), ', '.join('%s %.0f' % (k, math.degrees(v)) for k, v in bear.items()))
        sides = [v for v in seen if isinstance(v, str)]
        side = max(set(sides), key=sides.count)
        left = max(bear, key=lambda k: TP.wrap(bear[k] - self.look['yaw_now']))     # hoop more to the left
        right = min(bear, key=lambda k: TP.wrap(bear[k] - self.look['yaw_now']))
        return (left if side == 'left' else right), 'side %s' % side

    def look_switch(self, name, why, s_r):
        """Continue on route `name` (same folder): shared up to here, so only the rest of the lap changes."""
        self.pub_hoop.publish(Bool(data=False))
        cur = os.path.splitext(os.path.basename(self.path_file))[0]
        self.get_logger().info('★ hoop %s -> %s (%s)' % (self.hoop, name, why))
        if name != cur:
            f = os.path.join(os.path.dirname(self.path_file), name + '.csv')
            try:
                loop = TP.load_pins(f, self.lane)
            except Exception as e:
                self.get_logger().error('★ cannot load %s (%s) — staying on %s' % (f, e, cur))
                return
            done = s_r - self.plan.s_start
            st = self.lock_res['pose']
            s0 = loop.station(st[0], st[1])
            self.loop, self.path_file = loop, f
            zf = f[:-4] + '.zones.json'
            self.zones = json.load(open(zf)) if os.path.exists(zf) else []
            self.apply_zone_v()
            self.zone = None
            self.plan = TP.LanePlan(self.lane, s0, self.lock_res['lateral'], extra=self.extra, blend_m=self.blend_m, loop=loop)
            tp = self.track_pose(self.pose_at(None))
            sn = loop.station(tp[0], tp[1])
            sn += round((s0 + done - sn) / loop.length) * loop.length
            self.prog = TP.Progress(self.lane, sn, loop=loop)
            self.get_logger().info('route now %s: %.2f m/lap, %.1f m done, %.1f m to go' % (name, loop.length, sn - s0, self.plan.s_end - sn))
        self.look['step'] = 'done'
        self.look_done = True
        self.last_goal_t = 0.0
        if self.gait:
            # 11-20: standing at the look point the motor bridge sent StopMove, which may reset most sport settings —
            # put the gait (ClassicWalk) back before walking on: BalanceStand -> 0.5 s -> gait -> answer (or 1.5 s)
            self.regait = dict(step=0, t=self.now_s(), tries=0, ok=None)
            self.state = 'REGAIT'
        else:
            self.state = 'RUN'

    def look_tick(self, t, tp, s_r):
        """LOOK: turn to face the hoops (direct /cmd_vel), ask the vision, pick the branch."""
        lk, z = self.look, self.look['zone']
        if lk['step'] == 'turn':
            err = TP.wrap(math.radians(z['yaw_deg']) - tp[2])
            if abs(err) < math.radians(4) or t - lk['t'] > 8.0:
                self.pub_cmd.publish(Twist())
                lk.update(step='see', t=t, yaw_now=tp[2], pose=tp)
                self.hoop_seen = []
                self.get_logger().info('looking for the %s hoop (heading %.0f deg, %.0f off) — 1 / 2 picks %s'
                                       % (self.hoop, math.degrees(tp[2]), math.degrees(err), ' / '.join(z['branches'])))
                return
            tw = Twist()
            w = max(-0.6, min(0.6, 1.2 * err))
            tw.angular.z = math.copysign(max(abs(w), 0.3), w)      # Go2 ignores tiny turn commands
            self.pub_cmd.publish(tw)
        elif lk['step'] == 'see':
            if t - self.cam_pub_t > 1.0:
                self.pub_hoop.publish(Bool(data=True))
                self.cam_pub_t = t
            name, why = self.look_decide()
            if name is None and t - lk['t'] > float(z.get('timeout_s', self.look_timeout)):
                name = random.choice(list(z['branches']))
                why = '★ no %s hoop seen in %.0f s — random pick' % (self.hoop, t - lk['t'])
            if name is not None:
                self.look_switch(name, why, s_r)

    # ------------------------------------------------------------------ narrow corridor: corner to corner
    LEG_TURN_DEG = 15.0      # turn in place first when the next leg is this far off
    LEG_AHEAD = 0.25         # m, aim point ahead on the leg line
    LEG_REACHED = 0.03       # m before the corner = reached

    def twist(self, vx=0.0, wz=0.0):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(vx), float(wz)
        self.pub_cmd.publish(tw)

    def legs_start(self, t, tp, z):
        """RPP off: from where the robot is, straight to each corner of the zone, turning in place at the corners."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.twist()
        C = np.array(z['corners'], float)
        # skip corners already behind (re-entry, or entering past the first one)
        k = 0
        while k + 1 < len(C) and np.dot(np.array(tp[:2]) - C[k], C[k + 1] - C[k]) > 0:
            k += 1
        pts = np.vstack([np.array(tp[:2])[None, :], C[k:]])
        self.legs = dict(zone=z, pts=pts, i=0, t0=t, t=t, step=None, v=float(z.get('v', 0.25)))
        self.state = 'LEGS'
        self.legs_next_step(t, tp)
        self.get_logger().info('★ zone %s: corner to corner, %d legs (RPP off)' % (z['name'], len(pts) - 1))

    def legs_next_step(self, t, tp):
        L = self.legs
        a, b = L['pts'][L['i']], L['pts'][L['i'] + 1]
        err = TP.wrap(math.atan2(b[1] - a[1], b[0] - a[0]) - tp[2])
        L['step'] = 'turn' if abs(err) > math.radians(self.LEG_TURN_DEG) else 'drive'
        L['t'] = t

    def legs_end(self, t, why):
        z = self.legs['zone']
        self.twist()
        self.legs_done.add(z['name'])
        self.legs = None
        self.state = 'RUN'
        self.last_goal_t = 0.0      # RPP takes over from where the robot is
        self.get_logger().info('zone %s: %s — back to the route' % (z['name'], why))

    def legs_tick(self, t, tp):
        L = self.legs
        if t - L['t0'] > 60.0:
            self.get_logger().warn('★ zone %s took over 60 s' % L['zone']['name'])
            self.legs_end(t, 'timeout')
            return
        a, b = L['pts'][L['i']], L['pts'][L['i'] + 1]
        d = b - a
        Ld = max(1e-6, float(np.hypot(*d)))
        u = d / Ld
        hd = math.atan2(u[1], u[0])
        p = np.array(tp[:2])
        if L['step'] == 'turn':
            err = TP.wrap(hd - tp[2])
            if abs(err) < math.radians(6) or t - L['t'] > 8.0:
                self.twist()
                L['step'], L['t'] = 'drive', t
                return
            self.twist(0.0, math.copysign(min(0.6, max(0.3, 1.2 * abs(err))), err))
            return
        along = float(np.dot(p - a, u))
        rem = Ld - along
        if rem < self.LEG_REACHED:
            L['i'] += 1
            if L['i'] >= len(L['pts']) - 1:
                self.legs_end(t, 'last corner reached in %.1f s' % (t - L['t0']))
                return
            self.twist()
            self.legs_next_step(t, tp)
            return
        tgt = a + u * min(Ld, max(0.0, along) + self.LEG_AHEAD)
        ang = TP.wrap(math.atan2(tgt[1] - p[1], tgt[0] - p[0]) - tp[2])
        wz = max(-0.6, min(0.6, 1.5 * ang))
        vx = L['v'] if rem > 0.3 else max(0.1, L['v'] * rem / 0.3)
        if abs(ang) > math.radians(30):      # far off the line: turn before walking on
            vx = 0.0
            wz = math.copysign(max(0.3, abs(wz)), wz)
        self.twist(vx, wz)

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
                    self.get_logger().error('gait %s refused 3 times — stopping' % self.gait_name)
                    self.state = 'STOP'
                    return
                self.sport(API_BALANCESTAND)
                self.gait_step, self.gait_t = 1, t
            elif self.gait_step == 1:
                self.sport(self.gait_api, self.gait_param)
                self.gait_step, self.gait_t = 2, t
            elif self.gait_step == 2 and t - self.gait_t > 1.5:
                self.get_logger().warn('no %s response — retry' % self.gait_name)
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
                if self.wait_t is None:
                    self.wait_t = t
                go = self.force_go or (self.gate.go(t) if self.light != 'none' else True)
                if not go and self.light_timeout > 0 and t - self.wait_t > self.light_timeout:
                    go = True
                    self.get_logger().warn('★ no GREEN in %.0f s — going anyway' % self.light_timeout)
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
                if self.wait_t is None:
                    self.wait_t = t
                go = self.force_go or (self.gate.go(t) if self.light != 'none' else True)
                if not go and self.light_timeout > 0 and t - self.wait_t > self.light_timeout:
                    go = True
                    self.get_logger().warn('★ no GREEN in %.0f s — going anyway' % self.light_timeout)
                if go:
                    self.state, self.t_go = 'RUN', t
                    self.pub_cam.publish(Bool(data=False))
                    self.get_logger().info('GREEN — go')
            elif self.state == 'LOOK':
                s_r = self.prog.feed(tp[0], tp[1])
                self.look_tick(t, tp, s_r)
                return
            elif self.state == 'LEGS':
                self.prog.feed(tp[0], tp[1])
                self.legs_tick(t, tp)
                return
            elif self.state == 'REGAIT':
                self.prog.feed(tp[0], tp[1])
                self.regait_tick(t)
                return
            elif self.state in ('RUN', 'FINISH'):
                if self.state == 'RUN' and t - self.t_go < 3.0 and t - self.cam_pub_t > 1.0:
                    self.pub_cam.publish(Bool(data=False))   # light no longer needed (repeat: one can be lost)
                    self.cam_pub_t = t
                s_r = self.prog.feed(tp[0], tp[1])
                if self.zones and self.state == 'RUN':
                    self.zone_switch(self.zone_at(tp[0], tp[1]))
                    z = self.zone
                    if (z is not None and z.get('corners') and z.get('mode', 'legs') == 'legs'
                            and z['name'] not in self.legs_done):
                        self.legs_start(t, tp, z)
                        return
                if self.state == 'RUN' and s_r - self.plan.s_start > 2.5 and self.last_rot is not None:   # past the blend
                    self.bench['lat'].append(TP.lateral_of(self.loop, tp[0], tp[1])[0])
                    q = self.last_rot
                    self.bench['rp'].append((math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                                             math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))))
                if self.state == 'RUN' and self.look is not None and self.look['step'] == 'go' and self.look['s'] - s_r < 0.15:
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                        self.goal_handle = None
                    self.pub_cmd.publish(Twist())
                    self.state = 'LOOK'
                    self.look.update(step='turn', t=t)
                    self.get_logger().info('at the look point (%.1f m): turning to %.0f deg' % (s_r - self.plan.s_start, self.look['zone']['yaw_deg']))
                    return
                if self.state == 'RUN' and self.plan.s_end - s_r < 0.25:
                    self.state = 'FINISH'
                    self.finish_t = t
                    self.get_logger().info('lap done: %.2f m along the lane in %.1f s'
                                           % (s_r - self.plan.s_start, t - self.t_go))
                    self.bench_report(s_r - self.plan.s_start, t - self.t_go)
                # 11-20: the runner stops the robot itself at the lap end — RPP alone never called the goal reached when
                # the robot ended 0.10-0.15 m beside the path end, and turned in place into a cone
                if self.state == 'FINISH' and (self.plan.s_end - s_r < 0.12 or t - self.finish_t > 2.5):
                    self.end_stop('at the end' if self.plan.s_end - s_r < 0.12 else '%.1f s after the lap' % (t - self.finish_t))
                    return
            else:
                s_r = self.prog.s
            # only 6 m ahead: the whole lap + extra ends where the robot is 1.2 m after the start, and RPP calls a
            # goal 'reached' as soon as the robot is within 0.1 m of the path's last pose (stalled the run, 11-19)
            s_to = s_r + 6.0
            if self.look is not None and self.look['step'] == 'go':
                s_to = min(s_to, self.look['s'])                    # stop at the look point
            pts = TP.apply(self.T_ot, self.plan.points(s_r - 0.3, s_to))
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
            self.end_stop('RPP reached the end')

    def end_stop(self, why):
        """Lap over: cancel the path, zero /cmd_vel, StopMove, motor bridge off — nothing moves the robot after this."""
        if self.state == 'DONE':
            return
        self.state = 'DONE'
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        for _ in range(3):
            self.pub_cmd.publish(Twist())
        self.sport(API_STOPMOVE)
        self.bridge_state(False)
        self.get_logger().info('■ stopped at the lap end (%s) — motor bridge off. q = quit' % why)

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
                                       '초록을 봐야만 출발합니다' + (' (g = 강제 출발)' if self.debug else ''), throttle_duration_sec=3.0)
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
    # 데몬 스레드라 프로그램이 끝날 때 아래 finally 가 안 돌 수 있다 → 터미널이 글자 안 보이는 모드로 남음(11-20). 끝날 때 꼭 되돌림
    atexit.register(termios.tcsetattr, fd, termios.TCSADRAIN, old)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch in (' ', 'x', 'X'):
                node.stop('key')
            elif ch in ('1', '2', 'g', 'G') and not node.debug:
                node.get_logger().warn('★ key %s ignored — debug=true only (competition: no hand picks)' % ch)
            elif ch in ('1', '2') and node.state == 'LOOK':
                node.hoop_pick = int(ch) - 1
                node.get_logger().warn('★ key %s — branch picked by hand' % ch)
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
