#!/usr/bin/env python3
"""Built-in odometry against the wall-aligned track pose on a recording (no ROS graph, reads the bag only).

Truth: wall_align on every `step`-th /lidar_points frame (track frame, ~1 cm). Odometry: /utlidar/robot_odom,
anchored once at the start (mean wall pose while standing still), xy scaled by `scale`. This is what the runner
would do with a start-only alignment: how far does the odometry walk away from the truth in one lap?
    python3 tools/odom_vs_wall.py <bag dir> [step=2] [csv=out.csv] [gt=cache.npy] [traj=other.csv] [scales=1.0,1.189]
traj: any odometry as CSV with columns t,x,y,...,yaw (bag time, e.g. FAST-LIO /Odometry from tests/fastlio_replay.sh)
instead of /utlidar/robot_odom. gt: wall poses are cached there (computing them takes ~30 s per minute of bag).
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import track_spec as T          # noqa: E402
import wall_align as W          # noqa: E402
from wall_align_bag import frames   # noqa: E402


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def read_odom(bag, topic='/utlidar/robot_odom'):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while r.has_next():
        _, data, t = r.read_next()
        m = deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        out.append((t * 1e-9, p.x, p.y, yaw))
    return np.array(out)


def truth(bag, step):
    rows, prior = [], None
    for i, (t, P) in enumerate(frames(bag)):
        if i % step:
            continue
        r = W.align([P], seed=i, prior=prior)
        if r['ok']:
            prior = (r['x'], r['y'], r['yaw'])
            rows.append((t, r['x'], r['y'], r['yaw'], r['rms']))
    return np.array(rows)


def clean(G, win=7, tol_m=0.15, tol_deg=5.0):
    """Drop single wall poses that disagree with their neighbours (a wrong fit now and then, ~0.5 %)."""
    keep = np.ones(len(G), bool)
    h = win // 2
    for i in range(len(G)):
        a, b = max(0, i - h), min(len(G), i + h + 1)
        mx, my = np.median(G[a:b, 1]), np.median(G[a:b, 2])
        dy = np.degrees(np.abs(wrap(G[i, 3] - np.median(np.unwrap(G[a:b, 3]))))) if b - a > 2 else 0.0
        keep[i] = math.hypot(G[i, 1] - mx, G[i, 2] - my) < tol_m and dy < tol_deg
    return G[keep], int((~keep).sum())


def anchored(O, i0, gt0, scale):
    """Odometry rows -> track frame, with the odometry pose at index i0 put on gt0 = (x, y, yaw)."""
    x0, y0, a0 = O[i0, 1:4]
    c, s = math.cos(-a0), math.sin(-a0)
    dx, dy = O[:, 1] - x0, O[:, 2] - y0
    u, v = scale * (c * dx - s * dy), scale * (s * dx + c * dy)        # in the start body frame
    C, S = math.cos(gt0[2]), math.sin(gt0[2])
    return np.column_stack([O[:, 0], gt0[0] + C * u - S * v, gt0[1] + S * u + C * v, wrap(gt0[2] + O[:, 3] - a0)])


def errors(A, G, lane):
    """A: anchored odometry rows, G: truth rows. Errors at the truth times (odometry interpolated)."""
    x = np.interp(G[:, 0], A[:, 0], A[:, 1])
    y = np.interp(G[:, 0], A[:, 0], A[:, 2])
    yaw = np.interp(G[:, 0], A[:, 0], np.unwrap(A[:, 3]))
    ex, ey = x - G[:, 1], y - G[:, 2]
    # lateral (cross-track) part: the lane offset of the odometry point minus that of the truth
    lat = np.array([T.lane_frame(lane, a, b)[0] for a, b in zip(x, y)]) - np.array([T.lane_frame(lane, a, b)[0] for a, b in G[:, 1:3]])
    return np.hypot(ex, ey), lat, np.degrees(wrap(yaw - G[:, 3]))


def main():
    bag = sys.argv[1].rstrip('/')
    kv = dict(a.split('=') for a in sys.argv[2:])
    step = int(kv.get('step', 2))
    if 'gt' in kv and os.path.exists(kv['gt']):
        G = np.load(kv['gt'])
    else:
        G = truth(bag, step)
        if 'gt' in kv:
            np.save(kv['gt'], G)
    if 'traj' in kv:
        D = np.genfromtxt(kv['traj'], delimiter=',', names=True)
        O = np.column_stack([D['t'], D['x'], D['y'], D['yaw']])
        O = O[np.concatenate([[True], np.diff(O[:, 0]) > 1e-6])]
    else:
        O = read_odom(bag)
    name = os.path.basename(bag) + (' [%s]' % os.path.basename(kv['traj']) if 'traj' in kv else '')
    if len(G) < 20:
        print('%s: too few wall poses (%d)' % (name, len(G)))
        return
    G, n_drop = clean(G)
    # start of motion: odometry speed over 0.5 s above 0.05 m/s
    sp = np.hypot(np.gradient(O[:, 1], O[:, 0]), np.gradient(O[:, 2], O[:, 0]))
    k = np.convolve(sp, np.ones(75) / 75, mode='same')
    mv = np.nonzero(k > 0.05)[0]
    t_move = O[mv[0], 0] if len(mv) else O[-1, 0]
    still = G[(G[:, 0] < t_move - 0.2) & (G[:, 0] > t_move - 3.0)]
    if len(still) < 3:
        still = G[:5]
    gt0 = (still[:, 1].mean(), still[:, 2].mean(), math.atan2(np.sin(still[:, 3]).mean(), np.cos(still[:, 3]).mean()))
    t_anchor = still[-1, 0]
    i0 = int(np.searchsorted(O[:, 0], t_anchor))
    lane = 1 if abs(T.lane_frame(1, gt0[0], gt0[1])[0]) < abs(T.lane_frame(2, gt0[0], gt0[1])[0]) else 2
    run = G[G[:, 0] >= t_anchor]
    path = float(np.hypot(np.diff(run[:, 1]), np.diff(run[:, 2])).sum())
    dur = run[-1, 0] - t_anchor
    jit = np.hypot(still[:, 1] - gt0[0], still[:, 2] - gt0[1])
    print('%s: lane %d, wall poses %d, run %.1f s, path %.2f m (truth), start jitter %.1f cm (n=%d), start lateral %+.3f m, dropped %d'
          % (name, lane, len(G), dur, path, 100 * jit.max(), len(still), T.lane_frame(lane, gt0[0], gt0[1])[0], n_drop))
    best = None
    for s in np.arange(1.00, 1.351, 0.005):
        e, _, _ = errors(anchored(O, i0, gt0, s), run, lane)
        rms = float(np.sqrt(np.mean(e ** 2)))
        if best is None or rms < best[0]:
            best = (rms, s)
    scales = [float(v) for v in kv.get('scales', '1.0,1.189').split(',')]
    for s in scales + [best[1]]:
        e, lat, eyaw = errors(anchored(O, i0, gt0, s), run, lane)
        print('  scale %.3f: pos err p50 %.1f / p95 %.1f / max %.1f / end %.1f cm, lateral max |%.1f| cm, yaw err max |%.1f| deg'
              % (s, 100 * np.median(e), 100 * np.percentile(e, 95), 100 * e.max(), 100 * e[-1], 100 * np.abs(lat).max(), np.abs(eyaw).max()))
    if 'csv' in kv:
        A = anchored(O, i0, gt0, 1.189)
        e, lat, eyaw = errors(A, run, lane)
        with open(kv['csv'], 'w') as f:
            f.write('t,gx,gy,gyaw,ox,oy,err,lat,yaw_err\n')
            for row, ee, ll, yy in zip(run, e, lat, eyaw):
                ox = np.interp(row[0], A[:, 0], A[:, 1])
                oy = np.interp(row[0], A[:, 0], A[:, 2])
                f.write('%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n' % (row[0] - t_anchor, row[1], row[2], row[3], ox, oy, ee, ll, yy))


if __name__ == '__main__':
    main()
