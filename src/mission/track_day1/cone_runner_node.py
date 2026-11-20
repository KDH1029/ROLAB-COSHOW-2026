#!/usr/bin/env python3
"""Day-1 runner that also avoids cones that are not where the route was taught (a fork: day1_runner_node.py is
not changed and `day1.sh run` never reads any of this).

    scripts/day1_cones.sh run route=<name> [everything day1.sh run takes] [cone_clear=0.30 ...]

Everything is the original runner (start lock, light, zones, hula hoop, lap end, keys) except the path it sends:
in a cone zone (<route>.cones.json, boxes in the track frame, scripts/route_cones.py) the route ahead is checked
against the cones the LiDAR sees (/scan of the cloud bridge: 360 bins, floor and ramps already removed, 20 Hz,
small) and compared with the course map (cone_map: where the cones stood when the route was taught). Nothing
new near the route -> the taught route, unchanged. A moved / new cone on it -> the route is shifted sideways
around everything seen (cone_avoid.ConePlanner) and rejoins the taught route at the zone's end; while shifted
the robot slows to avoid_v. No way through -> it stops and waits; after block_s it takes the least-bad path.

Parameters (besides the runner's):
  cones          true | false   false = exactly the original runner (the scan is not used)
  cone_map       maps/coshow_1119 | none   course map (map.yaml + map.pgm, track frame); none = every cell is new
  cone_new_tol   0.12   m, a scan cell farther than this from the map's cells is new (moved cone); 11-19 replay:
                        scan vs map p95 4 cm, cone edges / walls up to 0.10-0.17 m
  cone_clear     0.20   m from the path to a scan cell (half width 0.155 + a little: the field is that tight)
  cone_keep_min  0.15   m, the taught route is left when any cell (even a mapped one) gets nearer than this
  cone_soft      0.40   m, nearer than this costs (keeps to the middle of a gap when it has to shift)
  cone_min_clear 0.17   m, last retry before 'blocked'
  cone_dmax      0.90   m sideways from the taught route at most (and never out of the zone box)
  cone_slope     0.80   m sideways per m forward at most
  cone_memory_s  6.0    s a seen cell is kept (the scan is blind closer than 0.4 m: a cone beside the robot)
  cone_range     3.5    m, farther scan returns are not used
  avoid_v        0.30   m/s while shifted (and the run's v if lower)
  avoid_lookahead 0.40  m RPP look-ahead while shifted
  block_s        4.0    s stopped with no way through, then the least-bad path
Log: logs/day1/cones_HHMMSS.jsonl (2 Hz: robot, mode, obstacle cells, path, track frame).
"""
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cone_avoid as CA  # noqa: E402
import day1_runner_node as R  # noqa: E402
import track_path as TP  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
LOG = os.environ.get('DAY1_LOG', os.path.join(ROOT, 'logs', 'day1'))


def rot2(q):
    """Upper-left 2x2 of the rotation of quaternion q (scan points lie in the z = 0 plane of their frame)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z)]])


class ConeRunner(R.Day1Runner):
    def __init__(self):
        super().__init__()

        def p(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.cone_on = str(p('cones', True)).strip().lower() not in ('false', '0', 'no', 'off')
        self.planner = CA.ConePlanner(clear=float(p('cone_clear', 0.20)), soft=float(p('cone_soft', 0.40)),
                                      min_clear=float(p('cone_min_clear', 0.17)), keep_min=float(p('cone_keep_min', 0.15)),
                                      dmax=float(p('cone_dmax', 0.90)), slope=float(p('cone_slope', 0.80)))
        self.cone_map = str(p('cone_map', os.path.join(ROOT, 'maps', 'coshow_1119')))
        self.new_tol = float(p('cone_new_tol', 0.12))
        self.mem = CA.ObstacleMemory(memory_s=float(p('cone_memory_s', 6.0)))
        self.cone_range = float(p('cone_range', 3.5))
        self.avoid_v = float(p('avoid_v', 0.30))
        self.avoid_lookahead = float(p('avoid_lookahead', 0.40))
        self.block_s = float(p('block_s', 4.0))
        self.scan_topic = str(p('scan_topic', '/scan'))
        self.replan_dt = 0.1
        self.ds = 0.1
        self.cz, self.cz_file = [], None
        self.cone_reset()
        self.n_scans = 0
        self.map_tree = None
        self.load_cone_zones()
        if self.cone_on:
            self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, qos_profile_sensor_data)
        self.pub_obs = self.create_publisher(PointCloud2, '/cone/obstacles', 2)
        self.create_timer(1.0, self.cone_report)
        os.makedirs(LOG, exist_ok=True)
        self.dump_f = os.path.join(LOG, 'cones_%s.jsonl' % time.strftime('%H%M%S'))
        self.dump = open(self.dump_f, 'w') if self.cone_on and self.cz else None
        self.dump_t = 0.0
        self.dump_route()
        self.wait_checked = False

    # ------------------------------------------------------------------ zones / state
    def cone_reset(self):
        self.cone = dict(mode='-', t=-1.0, pts=None, prev=None, block_t=None, hold=False, avoiding=False,
                         switch_t=0.0, res=None)

    def load_cone_zones(self):
        self.cz, self.cz_file = CA.load_zones(self.path_file, os.path.join(ROOT, 'routes')) if self.path_file else ([], None)
        if not self.cone_on:
            self.get_logger().info('cones=false — the original runner (no scan, no avoiding)')
        elif not self.cz:
            self.get_logger().warn('★ no cone zones for %s (<route>.cones.json, scripts/day1_cones.sh zone) — '
                                   'drives exactly like day1.sh run' % (os.path.basename(self.path_file) or 'the lane'))
        else:
            self.load_cone_map()
            self.get_logger().info('cone zones from %s: %s · clear %.2f m (min %.2f) · avoid v %.2f · scan %s'
                                   % (self.cz_file, ', '.join('%s [%.2f %.2f %.2f %.2f]' % ((z.get('name', 'cones'),) + tuple(z['box']))
                                                              for z in self.cz),
                                      self.planner.clear, self.planner.min_clear, self.avoid_v, self.scan_topic))

    def load_cone_map(self):
        """Occupied cells of the course map near the cone zones (track frame): what was there when taught."""
        self.map_tree = None
        if self.cone_map.lower() in ('', 'none', 'false'):
            self.get_logger().warn('★ cone_map none — every scan cell counts as a moved cone (taught route only if clear)')
            return
        try:
            y = {}
            for line in open(os.path.join(self.cone_map, 'map.yaml')):
                if ':' in line and not line.startswith('#'):
                    k, v = line.split(':', 1)
                    y[k.strip()] = v.split('#')[0].strip()
            raw = open(os.path.join(self.cone_map, y['image']), 'rb').read().split(b'\n', 3)
            w, h = map(int, raw[1].split())
            occ = np.frombuffer(raw[3], np.uint8)[:w * h].reshape(h, w)[::-1] < 100
            org = [float(v) for v in y['origin'].strip('[]').split(',')]
            res = float(y['resolution'])
            jj, ii = np.nonzero(occ)
            M = np.column_stack([org[0] + (ii + 0.5) * res, org[1] + (jj + 0.5) * res])
            M = M[CA.in_boxes(M, [z['box'] for z in self.cz], margin=0.8)]
            self.map_tree = CA.cKDTree(M) if len(M) else None
            self.get_logger().info('cone map %s: %d cells near the zones (scan cells farther than %.2f m from them = moved)'
                                   % (self.cone_map, len(M), self.new_tol))
        except Exception as e:
            self.get_logger().error('★ cone map %s not loaded (%s) — every scan cell counts as moved' % (self.cone_map, e))

    def new_mask(self, cells):
        if self.map_tree is None or not len(cells):
            return np.ones(len(cells), bool)
        return self.map_tree.query(cells, distance_upper_bound=self.new_tol + 0.01)[0] > self.new_tol

    def dump_route(self):
        if getattr(self, 'dump', None) is not None:
            self.dump.write(json.dumps(dict(route=self.path_file, zones=self.cz_file)) + '\n')
            self.dump.flush()

    def look_switch(self, name, why, s_r):
        super().look_switch(name, why, s_r)
        self.load_cone_zones()
        self.dump_route()
        if self.cone['avoiding']:
            self.avoid_rpp(False, self.now_s())
        self.cone_reset()

    # ------------------------------------------------------------------ scan -> obstacle cells (track frame)
    def cb_scan(self, m):
        T_to = self.T_to
        if not self.cz or T_to is None:
            return
        try:
            tr = self.tf.lookup_transform(self.odom_frame, m.header.frame_id, Time.from_msg(m.header.stamp),
                                          timeout=Duration(seconds=0.0))
        except Exception:
            try:
                tr = self.tf.lookup_transform(self.odom_frame, m.header.frame_id, Time())
            except Exception:
                return
        r = np.asarray(m.ranges, np.float64)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > max(m.range_min, 0.05)) & (r < min(m.range_max, self.cone_range))
        if not ok.any():
            self.mem.add(self.now_s(), np.zeros((0, 2)))
            return
        xy = np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
        t3 = tr.transform.translation
        P = xy @ rot2(tr.transform.rotation).T + np.array([t3.x, t3.y])
        P = TP.apply(T_to, P)
        self.mem.add(self.now_s(), P[CA.in_boxes(P, [z['box'] for z in self.cz], margin=0.5)])
        self.n_scans += 1

    # ------------------------------------------------------------------ planning
    def cone_window(self, s_r, s_to):
        ss = np.arange(s_r - 1.5, s_to + 1.5 + 1e-9, self.ds)
        ss = ss[(ss >= self.plan.s_start - 0.5) & (ss <= self.plan.s_end)]
        if len(ss) < 5:
            return None
        P = np.array([self.loop.pose_at(s, self.plan.off(s))[:2] for s in ss])
        N = CA.axis_normals(P, self.ds, self.planner.sigma_m)
        w = (ss >= s_r - 1e-9) & (ss <= s_to + 1e-9)
        if w.sum() < 3:
            return None
        return ss[w], P[w], N[w]

    def cone_plan(self, s_r, s_to, tp, t, force=False):
        win = self.cone_window(s_r, s_to)
        if win is None:
            return None
        ss, P, N = win
        boxes = [z['box'] for z in self.cz]
        inside = CA.in_boxes(P, boxes)
        if not inside.any() and not CA.in_boxes(np.array(tp[:2]), boxes, margin=0.3)[0]:
            return None
        cells = self.mem.cells(t)
        new = self.new_mask(cells)
        res = self.planner.plan(P, N, inside, tp[:2], cells, boxes, new=new, stations=ss,
                                prev=self.cone['prev'], force=force)
        res['n_cells'], res['n_new'] = len(cells), int(new.sum())
        res['ss'] = ss
        return res

    def avoid_rpp(self, on, t):
        self.cone['avoiding'], self.cone['switch_t'] = on, t
        if self.zone is not None:           # a route zone (zigzag) has its own settings
            return
        if on:
            self.set_rpp({'desired_linear_vel': min(self.v, self.avoid_v), 'lookahead_dist': self.avoid_lookahead})
            self.get_logger().info('★ cones: shifting around — v %.2f, look-ahead %.2f m' % (min(self.v, self.avoid_v), self.avoid_lookahead))
        else:
            self.set_rpp({'desired_linear_vel': self.v, 'lookahead_dist': self.lookahead if self.lookahead > 0 else 0.8})
            self.get_logger().info('cones: back on the taught route — v %.2f' % self.v)

    def cone_pts(self):
        """Path to send (odom frame) or None = the taught one (outside the cone zones)."""
        t = self.now_s()
        c = self.cone
        if c['pts'] is not None and t - c['t'] < self.replan_dt:
            return c['pts']
        cur = self.pose_at(None)
        if cur is None or self.T_to is None:
            return None
        tp = self.track_pose(cur)
        s_r = self.prog.s
        s_to = s_r + 6.0
        if self.look is not None and self.look['step'] in ('go', 'move'):
            s_to = min(s_to, self.look['fork'])     # look point (look_mode=stop) / fork while undecided (move)
        res = self.cone_plan(s_r, s_to, tp, t)
        if res is None:                                     # not near a cone zone
            if c['avoiding']:
                self.avoid_rpp(False, t)
            c.update(mode='-', pts=None, prev=None, block_t=None, hold=False, res=None)
            return None
        if res['mode'] == 'blocked':
            if c['block_t'] is None:
                c['block_t'] = t
                self.get_logger().warn('★ cones: no way through with %.2f m clearance — stopped (least-bad path after %.0f s)'
                                       % (self.planner.min_clear, self.block_s))
            if t - c['block_t'] >= self.block_s:
                res = self.cone_plan(s_r, s_to, tp, t, force=True) or res
                if res['mode'] == 'forced' and c['mode'] != 'forced':
                    self.get_logger().warn('★ cones: still blocked after %.0f s — least-bad path (min clearance %.2f m)'
                                           % (self.block_s, res['min_clear']))
        else:
            c['block_t'] = None
        hold = res['mode'] == 'blocked'
        if c['hold'] and not hold:
            self.last_goal_t = 0.0                          # send the new path right away
        c['hold'] = hold
        shifted = res['mode'] in ('avoid', 'forced') and float(np.abs(res['off']).max()) > 0.05
        if shifted != c['avoiding'] and (shifted or t - c['switch_t'] > 1.0):
            self.avoid_rpp(shifted, t)
        c['prev'] = (res['ss'], res['off']) if res['mode'] in ('avoid', 'forced') else None
        if res['mode'] != c['mode']:
            self.get_logger().info('cones: %s -> %s (min clearance %.2f m, max shift %.2f m, %d cells, %d new)'
                                   % (c['mode'], res['mode'], res['min_clear'], float(np.abs(res['off']).max()),
                                      res['n_cells'], res['n_new']))
        pts = res['pts']
        back = pts[0] - 0.3 * (pts[1] - pts[0]) / max(1e-6, float(np.hypot(*(pts[1] - pts[0]))))
        out = TP.apply(self.T_ot, np.vstack([back, pts]))
        c.update(mode=res['mode'], pts=out, t=t, res=res)
        self.cone_dump(t, tp, res)
        return out

    def cone_dump(self, t, tp, res):
        if self.dump is None or t - self.dump_t < 0.5:
            return
        self.dump_t = t
        cells = self.mem.cells(t)
        rec = dict(t=round(t, 2), state=self.state, s=round(self.prog.s - self.plan.s_start, 2),
                   robot=[round(v, 3) for v in tp], mode=res['mode'], used=round(res['used'], 3),
                   new=np.nonzero(self.new_mask(cells))[0].tolist(),
                   min_clear=None if not math.isfinite(res['min_clear']) else round(res['min_clear'], 3),
                   max_shift=round(float(np.abs(res['off']).max()), 3),
                   obs=np.round(cells, 3).tolist(), path=np.round(res['pts'], 3).tolist())
        self.dump.write(json.dumps(rec) + '\n')
        self.dump.flush()

    # ------------------------------------------------------------------ hooks into the original runner
    def make_path(self, pts):
        if self.mode == 'wall' and self.cone_on and self.cz and self.state == 'RUN' and self.plan is not None:
            try:
                new = self.cone_pts()
            except Exception as e:      # never stop the run for the planner: the taught route
                self.get_logger().error('★ cone planner failed (%s) — taught route' % e, throttle_duration_sec=2.0)
                new = None
            if new is not None:
                pts = new
        return super().make_path(pts)

    def send_goal(self, path_msg):
        if self.cone['hold'] and self.state == 'RUN':
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.pub_cmd.publish(Twist())
            return
        super().send_goal(path_msg)

    # ------------------------------------------------------------------ report
    def cone_report(self):
        if not self.cone_on or not self.cz or self.T_to is None:
            return
        t = self.now_s()
        cells = self.mem.cells(t)
        if self.T_ot is not None and len(cells):
            h = Header(frame_id=self.odom_frame, stamp=self.get_clock().now().to_msg())
            P = TP.apply(self.T_ot, cells)
            self.pub_obs.publish(pc2.create_cloud_xyz32(h, np.column_stack([P, np.zeros(len(P))]).astype(np.float32)))
        if self.state == 'WAIT' and not self.wait_checked and self.n_scans >= 20 and self.plan is not None:
            self.wait_checked = True
            cur = self.pose_at(None)
            if cur is not None:
                tp = self.track_pose(cur)
                res = self.cone_plan(self.plan.s_start, self.plan.s_start + 6.0, tp, t)
                if res is None:
                    self.get_logger().info('cones: the zone is more than 6 m ahead (%d scans, %d cells so far)'
                                           % (self.n_scans, len(cells)))
                elif res['mode'] == 'taught':
                    self.get_logger().info('cones at the start: %d cells seen, %d not in the map — taught route clear for '
                                           'the first 6 m (min %.2f m)' % (res['n_cells'], res['n_new'], res['min_clear']))
                else:
                    self.get_logger().warn('★ cones at the start: %d cells, %d not in the map — taught route NOT clear '
                                           '(%s, shift up to %.2f m): cones moved? it will go around'
                                           % (res['n_cells'], res['n_new'], res['mode'], float(np.abs(res['off']).max())))
        if self.state == 'RUN' and self.cone['mode'] not in ('-',):
            r = self.cone['res']
            if r is not None:
                self.get_logger().info('cones: %s · %d cells (%d new) · min clearance %s · max shift %.2f m'
                                       % (r['mode'], r['n_cells'], r['n_new'], '%.2f m' % r['min_clear'] if math.isfinite(r['min_clear']) else '-',
                                          float(np.abs(r['off']).max())))


def main():
    rclpy.init()
    node = ConeRunner()
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
        if node.dump is not None:
            node.dump.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
