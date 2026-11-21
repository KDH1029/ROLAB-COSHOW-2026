#!/usr/bin/env python3
"""Fake Go2 + D435 + traffic light for the isolated ROS integration test (never on the robot domain).

  /api/sport/request  -> Move / StopMove / BalanceStand / ClassicWalk (answers /api/sport/response)
  /utlidar/robot_odom <- built-in odometry that under-reads distance (x 1/1.189) like the real one
  /track/edges        <- rendered D435 frames through band_vision_core (same JSON as the internal PC node)  [mode vision]
  /lidar_points       <- XT16-like rays on the floor, the 0.6 m wall (9.00 x 7.01 m) and the other robot  [mode wall]
  /Odometry           <- FAST-LIO-like pose (own origin, small noise and yaw drift)                          [mode wall]
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from unitree_api.msg import Request, Response

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
import band_vision_core as BV  # noqa: E402
import floor_world as W  # noqa: E402
import track_spec as T  # noqa: E402
import track_path as TP  # noqa: E402
import wall_align as WA  # noqa: E402
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
        self.mode = str(p('mode', 'wall').value)
        self.other = bool(p('other', True).value)            # a second robot in the other lane
        self.wall_dims = (9.00, 7.01)                        # measured 11-18 (nominal 9.00 x 7.00)
        self.lio0 = (-0.8, 1.6, 2.0)                         # FAST-LIO origin: wherever it started
        self.lio_yaw_drift = math.radians(float(p('lio_yaw_drift_dps', 0.03).value))
        self.light_on = bool(p('light_on', True).value)      # false: no light messages at all (runner must wait)
        self.path_check = None                              # the runner's path vs the true lane centre
        # fake hula-hoop vision: hoop_red_at / hoop_yellow_at = 'x,y' (track frame), hoop_mode bearing | side | none
        self.hoops = {}
        for c in ('red', 'yellow'):
            v = str(p('hoop_%s_at' % c, '').value)
            if v:
                self.hoops[c] = [float(t) for t in v.split(',')]
        self.hoop_mode = str(p('hoop_mode', 'bearing').value)
        self.hoop_on = False
        self.hoop_asked = 0
        self.trail = []                                     # truth path (for the route-following check)
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
        self.pub_lio = self.create_publisher(Odometry, '/Odometry', 20)
        self.pub_cloud = self.create_publisher(PointCloud2, '/lidar_points', 5)
        self.pub_light = self.create_publisher(String, '/mission/traffic_light_state', 10)
        self.create_subscription(String, '/track/status', self.cb_status, 5)
        self.create_subscription(Path, '/track/path', self.cb_path, 2)
        self.create_subscription(Bool, '/mission/hoop_active', self.cb_hoop_on, 5)
        self.pub_hoop = self.create_publisher(String, '/mission/hoop_state', 10)
        self.create_timer(0.1, self.hoop_vision)
        self.runner_state = ''
        self.create_timer(0.02, self.step)
        self.create_timer(0.1, self.light)
        if self.mode == 'vision':
            threading.Thread(target=self.camera_loop, daemon=True).start()
        else:
            el = np.radians(np.arange(-15.0, 15.01, 2.0))
            az = np.radians(np.arange(0.0, 360.0, 1.0))
            E, A = np.meshgrid(el, az, indexing='ij')
            self.rays_l = np.column_stack([(np.cos(E) * np.cos(A)).ravel(), (np.cos(E) * np.sin(A)).ravel(), np.sin(E).ravel()])
            threading.Thread(target=self.lidar_loop, daemon=True).start()
        self.get_logger().info('fake world: lane %d, mode %s, other robot %s, green at %.1f s'
                               % (self.lane, self.mode, self.other, self.t_green))

    def other_pose(self, t):
        lane = 1 if self.lane == 2 else 2
        return TP.pose_at(lane, 3.0 + 0.5 * t)

    def lidar_loop(self):
        h_base = 0.31
        while rclpy.ok():
            t0 = time.time()
            with self.lock:
                x, y, yaw = self.x, self.y, self.yaw
            c, s = math.cos(yaw), math.sin(yaw)
            D = self.rays_l @ WA.R_LIDAR.T                   # base frame directions
            o = WA.T_LIDAR
            ox, oy, oz = x + c * o[0] - s * o[1], y + s * o[0] + c * o[1], h_base + o[2]
            dx, dy, dz = c * D[:, 0] - s * D[:, 1], s * D[:, 0] + c * D[:, 1], D[:, 2]
            best = np.full(len(D), np.inf)
            with np.errstate(divide='ignore', invalid='ignore'):
                tf = np.where(dz < -1e-6, -oz / dz, np.inf)
                best = np.minimum(best, tf)
                for ax, half in ((0, self.wall_dims[0] / 2), (1, self.wall_dims[1] / 2)):
                    oo, dd = (ox, dx) if ax == 0 else (oy, dy)
                    for sg in (-1, 1):
                        tw = (sg * half - oo) / dd
                        zh = oz + tw * dz
                        ok = (tw > 0) & (zh >= 0) & (zh <= T.WALL_H)
                        best = np.where(ok & (tw < best), tw, best)
                if self.other:
                    qx, qy, _ = self.other_pose(t0 - self.t0)
                    fx, fy = ox - qx, oy - qy
                    a = dx * dx + dy * dy
                    b = 2 * (fx * dx + fy * dy)
                    cc = fx * fx + fy * fy - 0.22 ** 2
                    disc = b * b - 4 * a * cc
                    tc = np.where(disc > 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
                    zh = oz + tc * dz
                    ok = (tc > 0) & (zh >= 0) & (zh <= 0.45)
                    best = np.where(ok & (tc < best), tc, best)
            keep = np.isfinite(best) & (best > 0.3) & (best < 15.0)
            r = best[keep] + self.rng.normal(0, 0.01, keep.sum())
            P = self.rays_l[keep] * r[:, None]                # lidar frame
            hdr = Odometry().header
            hdr.stamp = self.get_clock().now().to_msg()
            hdr.frame_id = 'hesai_lidar'
            self.pub_cloud.publish(pc2.create_cloud_xyz32(hdr, P.astype(np.float32)))
            rest = 0.1 - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)

    def cb_status(self, m):
        try:
            self.runner_state = json.loads(m.data).get('state', '')
        except ValueError:
            pass

    def cb_path(self, m):
        """First path while the runner waits: shift (translation fit) and rms against the true lane centre."""
        if self.path_check is not None or self.runner_state != 'WAIT' or len(m.poses) < 60:
            return
        t = time.time() - self.t0
        lio0 = (self.lio0[0], self.lio0[1], self.lio0[2] + self.lio_yaw_drift * t)
        P = TP.apply(TP.inverse(lio0), [(q.pose.position.x, q.pose.position.y) for q in m.poses])[30:]   # past the blend
        loop = TP.RulebookLoop(self.lane)
        sh = np.zeros(2)
        for _ in range(8):
            Q = P - sh
            C = np.array([loop.pose_at(loop.station(x, y))[:2] for x, y in Q])
            sh += (Q - C).mean(0)
        Q = P - sh
        C = np.array([loop.pose_at(loop.station(x, y))[:2] for x, y in Q])
        self.path_check = dict(path_shift_cm=[round(100 * float(v), 1) for v in sh],
                               path_rms_cm=round(100 * float(np.sqrt(((Q - C) ** 2).sum(1).mean())), 2),
                               path_len_m=round(float(np.hypot(*np.diff(P, axis=0).T).sum()), 2))

    def cb_hoop_on(self, m):
        if m.data and not self.hoop_on:
            self.hoop_asked += 1
        self.hoop_on = bool(m.data)

    def hoop_vision(self):
        if not self.hoop_on or not self.hoops or self.hoop_mode == 'none':
            return
        with self.lock:
            x, y, yaw = self.x, self.y, self.yaw
        b = {c: math.degrees(math.atan2(math.sin(math.atan2(h[1] - y, h[0] - x) - yaw), math.cos(math.atan2(h[1] - y, h[0] - x) - yaw)))
             for c, h in self.hoops.items()}
        if self.hoop_mode == 'side' and len(b) == 2:
            hi = max(b, key=b.get)
            out = {c: {'side': 'left' if c == hi else 'right', 'conf': 0.9} for c in b}
        else:
            out = {c: {'bearing_deg': round(v + self.rng.normal(0, 1.0), 2), 'conf': 0.9} for c, v in b.items()}
        self.pub_hoop.publish(String(data=json.dumps(out)))

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
        elif a in (2049, 2045, 1061, 1062, 1063):     # walking gaits (classic, free, static, trot run, economic)
            r = Response()
            r.header.identity.id = m.header.identity.id
            r.header.identity.api_id = a
            r.header.status.code = 0 if self.balanced else -1     # real Go2: refused unless BalanceStand
            self.walk = self.balanced
            self.pub_resp.publish(r)

    def light(self):
        t = time.time() - self.t0
        green = self.t_first_req is not None and t >= self.t_first_req + self.t_green
        if not self.light_on:
            return
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
            if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.05:
                self.trail.append((self.x, self.y))
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
        if self.mode == 'wall':
            t = time.time() - self.t0
            lio0 = (self.lio0[0], self.lio0[1], self.lio0[2] + self.lio_yaw_drift * t)
            px, py, pyaw = TP.compose(lio0, (self.x + self.rng.normal(0, 0.004), self.y + self.rng.normal(0, 0.004), self.yaw))
            q = Odometry()
            q.header.stamp = o.header.stamp
            q.header.frame_id = 'camera_init'
            q.child_frame_id = 'body'
            q.pose.pose.position.x, q.pose.pose.position.y = px, py
            q.pose.pose.orientation.z, q.pose.pose.orientation.w = math.sin(pyaw / 2), math.cos(pyaw / 2)
            self.pub_lio.publish(q)

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
        d = dict(lap_s=lap, margin_cm=round(self.min_margin * 100, 1), viol=self.viol,
                 overshoot_m=round(self.prog - self.need, 2), runner=self.runner_state, moved_m=round(self.prog, 2))
        d.update(self.path_check or {})
        d['hoop_asked'] = self.hoop_asked
        tf = os.environ.get('FAKE_TRAIL')
        if tf:
            np.savetxt(tf, np.array(self.trail), delimiter=',', fmt='%.4f', header='x_m,y_m', comments='')
        return d


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
