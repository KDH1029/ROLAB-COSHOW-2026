#!/usr/bin/env python3
"""Practice runner: start ANYWHERE on the route and stop at the route's start (the start mark) — test only, a fork
(day1_runner_node.py and `day1.sh run` are not changed).

    scripts/practice_run.sh [route=rec105326_lap1] [hoop=in] [v=0.8] [...day1.sh run arguments]   (alias dhere)

e.g. 11-20 practice 1: the cones were moved and not taught yet, so start past the cone field (right side, facing up
the right wall), drive the rest of the lap (arch, zigzag) and stop at the start mark, before the cones.
Differences from the runner:
  - start lock: no '1 m from the route start' rule; limits max_lat 0.5 m / max_head 30 deg. The walls alone cannot
    tell the two 180 deg twins apart, and this route is nearly point-symmetric (up the right wall ~ down the left
    one), so the twin is the one whose scan lands on the MAP's inside structures (zigzag walls, arch posts, cones),
    not on the walls: fraction of inside scan points within 0.12 m of a map cell. Neither twin on the map = the wall
    fit is wrong there: it does not start and says so (11-19 bag: start mark 93 vs 2 %, past the cones 94 vs 9 %,
    right in front of the zigzag the wall fit is 1.9 m off — 1 m further back it is fine, 85 vs 4 %)
  - lap end: at the route's start station + end_extra (0 m) instead of one lap + extra
  - zigzag_v / cones_v=<m/s>: now the runner's own (day1_runner_node), works here too
  - lock=auto (default): the position kept by dkeep (track_keep_node, from the start mark) if it is from this FAST-LIO
    run and < 15 min old, else the wall fit; lock=keep / lock=walls to force one
Everything else (light, zones, hula hoop, keys, debug) is the runner's.
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from scipy.spatial import cKDTree
from sensor_msgs_py import point_cloud2 as pc2
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import day1_runner_node as R  # noqa: E402
import track_path as TP  # noqa: E402
import wall_align as WA  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))


def map_inside_tree(d, margin=0.4):
    """KD-tree of the map's occupied cells away from the arena walls (the asymmetric structures)."""
    y = {}
    for line in open(os.path.join(d, 'map.yaml')):
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1)
            y[k.strip()] = v.split('#')[0].strip()
    raw = open(os.path.join(d, y['image']), 'rb').read().split(b'\n', 3)
    w, h = map(int, raw[1].split())
    occ = np.frombuffer(raw[3], np.uint8)[:w * h].reshape(h, w)[::-1] < 100
    org = [float(v) for v in y['origin'].strip('[]').split(',')]
    res = float(y['resolution'])
    jj, ii = np.nonzero(occ)
    M = np.column_stack([org[0] + (ii + 0.5) * res, org[1] + (jj + 0.5) * res])
    M = M[(np.abs(M[:, 0]) < 4.5 - margin) & (np.abs(M[:, 1]) < 3.5 - margin)]
    return cKDTree(M) if len(M) > 20 else None


class AnyStartLock(TP.StartLock):
    """StartLock without the 'near the route start' rule; twin by the map (inside structures), else the route fit."""
    scan = None        # (N, 3) base_link points of the latest frame (set by the runner)
    mtree = None       # map_inside_tree
    twin_info = ''

    def fit(self, tw):
        """How badly the robot would sit on the route for wall result tw: |lateral| + 0.5 |heading| (m, rad)."""
        q = TP.compose(self.off_inv, tw)
        lat, _, s = TP.lateral_of(self.loop, q[0], q[1])
        a, b = self.loop.pose_at(s), self.loop.pose_at(s + 0.5)
        head = TP.wrap(q[2] - math.atan2(b[1] - a[1], b[0] - a[0]))
        return abs(lat) + 0.5 * abs(head)

    def map_score(self, tw):
        if self.scan is None or self.mtree is None:
            return None
        q = TP.compose(self.off_inv, tw)
        P = self.scan
        r = np.hypot(P[:, 0], P[:, 1])
        P = P[(P[:, 2] > -0.15) & (P[:, 2] < 0.6) & (r > 0.5) & (r < 6.0)]
        c, s = math.cos(q[2]), math.sin(q[2])
        X = q[0] + c * P[:, 0] - s * P[:, 1]
        Y = q[1] + s * P[:, 0] + c * P[:, 1]
        inside = (np.abs(X) < 4.1) & (np.abs(Y) < 3.1)
        if inside.sum() < 30:
            return 0.0
        d = self.mtree.query(np.column_stack([X[inside], Y[inside]]), distance_upper_bound=0.2)[0]
        return float((d < 0.12).mean())

    def pick_twin(self, tw, twin):
        """(chosen, None) or (None, why). With a map: only the map decides — if neither twin lands on the map's inside
        structures, the wall fit itself is off (11-19 bag: right in front of the zigzag the corridor walls pass for arena
        walls, 1.9 m wrong) and nothing is accepted. Without a map: the route fit, only if clear-cut."""
        a, b = self.map_score(tw), self.map_score(twin)
        fa, fb = self.fit(tw), self.fit(twin)
        if a is not None:
            self.twin_info = 'map match %.0f %% vs %.0f %% (twin)' % (100 * max(a, b), 100 * min(a, b))
            if max(a, b) >= 0.25 and abs(a - b) >= 0.15:
                return (tw if a > b else twin), None
            return None, ('the walls do not match the map here (%s): the wall fit is off — put it ~1 m further back '
                          'along the route, away from the zigzag walls' % self.twin_info)
        self.twin_info = 'no map: route fit %.2f vs %.2f' % (min(fa, fb), max(fa, fb))
        if abs(fa - fb) >= 0.3:
            return (tw if fa < fb else twin), None
        return None, 'twin ambiguous (%s) — stand closer to the route' % self.twin_info

    def feed(self, wall, odom):
        if not wall.get('ok'):
            self.why = 'wall: ' + wall.get('why', '?')
            return None
        tw = (wall['x'], wall['y'], wall['yaw'])
        twin = (-tw[0], -tw[1], TP.wrap(tw[2] + math.pi))
        if not self.buf:
            tw, why = self.pick_twin(tw, twin)
            if tw is None:
                self.why = why
                return None
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


class KeptLock(TP.StartLock):
    """lock=keep: track <- odom kept by track_keep_node (dkeep) since the start mark; no wall fit needed here."""

    def __init__(self, T, info, *a, **k):
        super().__init__(*a, **k)
        self.T, self.twin_info = tuple(T), info

    def feed(self, wall, odom):
        for key, v in (('walls_seen', 0), ('dims', (0.0, 0.0)), ('rms', 0.0)):   # the runner logs these
            wall.setdefault(key, v)
        pose = TP.compose(self.T, odom)
        lat, _, s = TP.lateral_of(self.loop, pose[0], pose[1])
        a, b = self.loop.pose_at(s), self.loop.pose_at(s + 0.5)
        head = TP.wrap(pose[2] - math.atan2(b[1] - a[1], b[0] - a[0]))
        if abs(lat) > self.max_lat or abs(head) > self.max_head:
            self.why = 'placement (kept position): lateral %+.2f m, heading %+.1f deg (limits %.2f m, %.0f deg)' % (
                lat, math.degrees(head), self.max_lat, math.degrees(self.max_head))
            return None
        self.why = ''
        return dict(T_track_odom=self.T, pose=pose, s=s, lateral=lat, heading_err=head)


class PracticeRunner(R.Day1Runner):
    def __init__(self):
        super().__init__()

        def p(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.end_extra = float(p('end_extra', 0.0))
        if self.mode != 'wall' or not self.path_file:
            raise SystemExit('practice runner: a route is needed (route=<name>)')
        self.wlock = AnyStartLock(self.lane, n=self.lock_n, max_lat=self.max_lat, max_head_deg=self.max_head,
                                  loop=self.loop, offset=self.track_off)
        # lock: auto (default) = the position kept by dkeep if it is from this FAST-LIO run and < 15 min old, else walls
        #       keep = only the kept one (refuse otherwise) · walls = wall fit + map twin check
        mode = str(p('lock', 'auto')).strip().lower()
        if mode in ('auto', 'keep'):
            kept, why = self.kept()
            if kept is not None:
                T, info = kept
                self.wlock = KeptLock(T, info, self.lane, n=1, max_lat=self.max_lat, max_head_deg=self.max_head,
                                      loop=self.loop, offset=self.track_off)
                self.get_logger().info('★ PRACTICE: position kept by dkeep — ' + info)
            elif mode == 'keep':
                raise SystemExit('lock=keep: ' + why)
            else:
                self.get_logger().info('★ PRACTICE: walls (%s)' % why)
        mapd = str(p('lock_map', os.path.join(ROOT, 'maps', 'coshow_1119')))
        try:
            if isinstance(self.wlock, AnyStartLock):
                self.wlock.mtree = map_inside_tree(mapd)
        except Exception as e:
            self.get_logger().warn('★ lock map %s not loaded (%s) — twin by the route fit only' % (mapd, e))
        self.end_set = False
        self.get_logger().info('★ PRACTICE: start anywhere on %s, stop at its start (+%.1f m)'
                               % (os.path.basename(self.path_file), self.end_extra))

    def kept(self):
        """((T, info), '') from logs/day1/track_keep.json, or (None, why not)."""
        import json
        f = os.path.join(os.environ.get('DAY1_LOG', os.path.join(ROOT, 'logs', 'day1')), 'track_keep.json')
        if not os.path.exists(f):
            return None, 'no kept position (dkeep on the start mark keeps one)'
        k = json.load(open(f))
        try:
            pid = int(open(os.path.join(os.path.dirname(f), 'fastlio.pid')).read().strip())
        except Exception:
            pid = None
        if pid is None or pid != k.get('fastlio_pid'):
            return None, 'the kept position is from another FAST-LIO run (pid %s, now %s) — dkeep again' % (k.get('fastlio_pid'), pid)
        age = time.time() - k['time']
        if age > 900:
            return None, 'the kept position is %.0f min old (FAST-LIO drifts) — dkeep again' % (age / 60)
        return (k['T_track_odom'], 'kept %.0f s ago, FAST-LIO pid %s' % (age, pid)), ''

    def cb_lidar(self, m):
        if self.state == 'LOCK' and self.plan is None and self.n_frames % 2 == 1:     # the frame the runner will use
            self.wlock.scan = WA.lidar_to_base(pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True))
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
            self.get_logger().info('★ PRACTICE: from station %.1f m, stop at the route start: %.1f m to drive · twin: %s'
                                   % (s0 % L, self.plan.s_end - s0, self.wlock.twin_info))

    def look_switch(self, name, why, s_r):
        end_left = self.plan.s_end - s_r
        super().look_switch(name, why, s_r)
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
