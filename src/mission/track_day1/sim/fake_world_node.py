#!/usr/bin/env python3
"""Fake Go2 + D435 + traffic light for the isolated ROS integration test (never on the robot domain).

  /api/sport/request  -> Move / StopMove / BalanceStand / ClassicWalk (answers /api/sport/response)
  /utlidar/robot_odom <- built-in odometry that under-reads distance (x 1/1.189) like the real one
  /track/edges        <- rendered D435 frames through band_vision_core (same JSON as the internal PC node)
  /mission/traffic_light_state <- RED until t_green, then GREEN
  truth: foot margin to the lane lines, lap time (green -> rear feet over the start line), printed at the end
"""
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from unitree_api.msg import Request, Response

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
import band_vision_core as BV  # noqa: E402
import floor_world as W  # noqa: E402
import track_spec as T  # noqa: E402
from sim_day1 import FEET, FOOT_R, BAND, pose_at_s, track_s  # noqa: E402


class FakeWorld(Node):
    def __init__(self):
        super().__init__('fake_world')
        def p(name, default):   # numbers typed as 1 or 1.0 both work
            return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        self.lane = int(p('lane', 2).value)
        self.t_green = float(p('t_green', 8.0).value)
        off0 = float(p('off0', 0.0).value)
        yaw0 = math.radians(float(p('yaw0', 0.0).value))
        s0 = float(p('s0', 0.4).value)
        self.latency = float(p('latency', 0.12).value)
        self.drift = float(p('drift', 0.012).value)
        self.R = T.R_CENTER[self.lane]
        self.lap = T.LAP_LEN[self.lane]
        self.x, self.y, self.yaw = pose_at_s(s0 - 0.30, self.R, off0)
        self.yaw += yaw0
        self.on_straight = (s0 % self.lap) < T.STRAIGHT
        self.start_lines = [(self.lane, pose_at_s(s0, self.R)[0], -1 if pose_at_s(s0, self.R)[1] < 0 else 1)] \
            if self.on_straight else []
        self.need = self.lap + 0.30 + 0.21
        self.rng = np.random.default_rng(1)
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = 0.0
        self.va = [0.0, 0.0, 0.0]
        self.ox, self.oy, self.oyaw = 0.0, 0.0, 0.3
        self.balanced = False
        self.walk = False
        self.t0 = time.time()
        self.t_first_req = None      # the light clock starts at the runner's first request (gait handshake)
        self.t_go = self.t_fin = None
        self.prog = 0.0
        self.s_prev = track_s(self.x, self.y, self.R)
        self.min_margin = 1e9
        self.viol = 0
        self.in_viol = False
        self.lock = threading.Lock()
        self.rig = W.Rig(pitch_deg=float(p('pitch', 20.0).value), ds=2)
        self.vis = BV.BandVision()
        self.create_subscription(Request, '/api/sport/request', self.cb_req, 20)
        self.pub_resp = self.create_publisher(Response, '/api/sport/response', 10)
        self.pub_odom = self.create_publisher(Odometry, '/utlidar/robot_odom', 20)
        self.pub_edges = self.create_publisher(String, '/track/edges', QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.pub_light = self.create_publisher(String, '/mission/traffic_light_state', 10)
        self.create_subscription(String, '/track/status', self.cb_status, 5)
        self.runner_state = ''
        self.create_timer(0.02, self.step)
        self.create_timer(0.1, self.light)
        threading.Thread(target=self.camera_loop, daemon=True).start()
        self.get_logger().info('fake world: lane %d, green at %.1f s' % (self.lane, self.t_green))

    def cb_status(self, m):
        try:
            self.runner_state = json.loads(m.data).get('state', '')
        except ValueError:
            pass

    def cb_req(self, m):
        a = m.header.identity.api_id
        if self.t_first_req is None:
            self.t_first_req = time.time() - self.t0
        if a == 1008:
            d = json.loads(m.parameter)
            self.cmd = (float(d['x']), float(d['y']), float(d['z']))
            self.cmd_t = time.time()
        elif a == 1003:
            self.cmd = (0.0, 0.0, 0.0)
        elif a == 1002:
            self.balanced = True
        elif a == 2049:
            r = Response()
            r.header.identity.id = m.header.identity.id
            r.header.identity.api_id = 2049
            r.header.status.code = 0 if self.balanced else -1     # real Go2: refused unless BalanceStand
            self.walk = self.balanced
            self.pub_resp.publish(r)

    def light(self):
        t = time.time() - self.t0
        green = self.t_first_req is not None and t >= self.t_first_req + self.t_green
        self.pub_light.publish(String(data='GREEN|0.90' if green else 'RED|0.90'))
        if green and self.t_go is None:
            self.t_go = t

    def step(self):
        dt = 0.02
        with self.lock:
            vx, vy, wz = self.cmd if time.time() - self.cmd_t < 0.6 else (0.0, 0.0, 0.0)
            a = dt / 0.25
            self.va[0] += (vx * 0.97 - self.va[0]) * a
            self.va[1] += (vy * 0.9 - self.va[1]) * a
            moving = abs(self.va[0]) > 0.05
            self.va[2] += (wz + (self.drift if moving else 0.0) - self.va[2]) * (dt / 0.15)
            dxb = self.va[0] * (1 + self.rng.normal(0, 0.03)) * dt
            dyb = (self.va[1] + self.rng.normal(0, 0.01) * moving) * dt
            dth = (self.va[2] + self.rng.normal(0, 0.02) * moving) * dt
            self.x += math.cos(self.yaw) * dxb - math.sin(self.yaw) * dyb
            self.y += math.sin(self.yaw) * dxb + math.cos(self.yaw) * dyb
            self.yaw += dth
            k = 1 / 1.189
            self.ox += (math.cos(self.oyaw) * dxb - math.sin(self.oyaw) * dyb) * k
            self.oy += (math.sin(self.oyaw) * dxb + math.cos(self.oyaw) * dyb) * k
            self.oyaw += dth + 0.002 * dt
            # truth
            s_now = track_s(self.x, self.y, self.R)
            ds = s_now - self.s_prev
            ds += self.lap if ds < -self.lap / 2 else (-self.lap if ds > self.lap / 2 else 0)
            self.s_prev = s_now
            t = time.time() - self.t0
            if self.t_go is not None:
                self.prog += ds
                if self.t_fin is None and self.prog >= self.need:
                    self.t_fin = t
            lo, hi = BAND[self.lane]
            m = 1e9
            for fx, fy in FEET:
                px = self.x + math.cos(self.yaw) * fx - math.sin(self.yaw) * fy
                py = self.y + math.sin(self.yaw) * fx + math.cos(self.yaw) * fy
                rf = W.stadium_r(np.array([px]), np.array([py]))[0]
                m = min(m, rf - lo, hi - rf)
            m -= FOOT_R
            self.min_margin = min(self.min_margin, m)
            if m < 0 and not self.in_viol:
                self.viol += 1
            self.in_viol = m < 0
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.ox, self.oy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.oyaw / 2), math.cos(self.oyaw / 2)
        o.twist.twist.linear.x = self.va[0] / 1.189
        o.twist.twist.angular.z = self.va[2]
        self.pub_odom.publish(o)

    def camera_loop(self):
        while rclpy.ok():
            t0 = time.time()
            with self.lock:
                x, y, yaw = self.x, self.y, self.yaw
            bgr, dep = self.rig.render_pair(x, y, yaw, rng=self.rng, start_lines=self.start_lines)
            out = self.vis.process(bgr, dep, self.rig.color.K, self.rig.depth.K)
            # deliver `latency` after capture; age tells the NUC how old the frame is
            wait = self.latency - (time.time() - t0)
            if wait > 0:
                time.sleep(wait)
            d = dict(ok=bool(out['ok']), why=out.get('why', ''), age=round(time.time() - t0, 4))
            if out['ok']:
                d['pts'] = [round(float(v), 3) for v in out['pts'].reshape(-1)]
            self.pub_edges.publish(String(data=json.dumps(d)))
            rest = 0.1 - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)

    def summary(self):
        lap = (self.t_fin - self.t_go) if (self.t_fin and self.t_go) else None
        return dict(lap_s=lap, margin_cm=round(self.min_margin * 100, 1), viol=self.viol,
                    overshoot_m=round(self.prog - self.need, 2), runner=self.runner_state)


def main():
    rclpy.init()
    n = FakeWorld()
    t_end = time.time() + float(os.environ.get('FAKE_T_MAX', '90'))
    try:
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.runner_state in ('DONE', 'STOP') or (n.t_fin and time.time() - n.t0 > n.t_fin + 6):
                break
    except KeyboardInterrupt:
        pass
    print('RESULT ' + json.dumps(n.summary()), flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
