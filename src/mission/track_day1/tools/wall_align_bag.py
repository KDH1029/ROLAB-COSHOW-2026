#!/usr/bin/env python3
"""wall_align on a recording: XT16 /lidar_points -> base frame (bridge_params extrinsics) -> track pose.

Every `step`-th frame is aligned on its own (one frame, no motion model), so the output is a track-frame
trajectory: start pose, how often the gates pass, lane offset along the way.
    python3 tools/wall_align_bag.py <bag dir> [step=4] [csv=out.csv]
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import wall_align as W   # noqa: E402

# go2_bridge bridge_params.yaml: hesai_lidar in base_link
EXT = dict(x=0.25, y=0.0, z=0.1588, roll=0.012330, pitch=0.005579, yaw=1.57079632679)


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


R_EXT = rot(EXT['roll'], EXT['pitch'], EXT['yaw'])
T_EXT = np.array([EXT['x'], EXT['y'], EXT['z']])


def frames(bag, topic='/lidar_points'):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while r.has_next():
        _, data, t = r.read_next()
        m = deserialize_message(data, PointCloud2)
        P = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(float)
        yield t * 1e-9, P @ R_EXT.T + T_EXT


def main():
    bag = sys.argv[1]
    kv = dict(a.split('=') for a in sys.argv[2:])
    step = int(kv.get('step', 4))
    rows, t0, prior = [], None, None
    for i, (t, P) in enumerate(frames(bag)):
        if t0 is None:
            t0 = t
        if i % step:
            continue
        r = W.align([P], seed=i, prior=prior)
        if r['ok']:
            prior = (r['x'], r['y'], r['yaw'])
        rows.append((t - t0, r))
    ok = [(t, r) for t, r in rows if r['ok']]
    print('%s: %d frames, ok %d (%.0f%%)' % (os.path.basename(bag.rstrip('/')), len(rows), len(ok), 100.0 * len(ok) / max(1, len(rows))))
    why = {}
    for _, r in rows:
        if not r['ok']:
            k = r['why'].split('(')[0].strip()
            why[k] = why.get(k, 0) + 1
    if why:
        print('  fail:', why)
    if ok:
        t, r = ok[0]
        print('  first: t=%.1f lane %d  x %.3f y %.3f yaw %.1f  lateral %+.3f m heading %+.1f deg  dims %.2f x %.2f rms %.3f walls %d floor_h %.3f'
              % (t, r['lane'], r['x'], r['y'], math.degrees(r['yaw']), r['lateral'], math.degrees(r['heading_err']),
                 r['dims'][0], r['dims'][1], r['rms'], r['walls_seen'], r['floor_h']))
        D = np.array([r['dims'] for _, r in ok])
        rm = np.array([r['rms'] for _, r in ok])
        print('  dims median %.3f x %.3f (p5-p95 %.3f-%.3f / %.3f-%.3f)  rms median %.3f'
              % (np.median(D[:, 0]), np.median(D[:, 1]), *np.percentile(D[:, 0], [5, 95]), *np.percentile(D[:, 1], [5, 95]), np.median(rm)))
        lat = np.array([r['lateral'] for _, r in ok])
        lanes = [r['lane'] for _, r in ok]
        print('  lateral p5/p50/p95 %+.3f/%+.3f/%+.3f m, lanes %s' % (*np.percentile(lat, [5, 50, 95]), {k: lanes.count(k) for k in set(lanes)}))
    if 'csv' in kv:
        with open(kv['csv'], 'w') as f:
            f.write('t,ok,x,y,yaw,lane,lateral,heading_err,dim_u,dim_v,rms,walls\n')
            for t, r in rows:
                if r['ok']:
                    f.write('%.3f,1,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n' % (t, r['x'], r['y'], r['yaw'], r['lane'], r['lateral'], r['heading_err'], r['dims'][0], r['dims'][1], r['rms'], r['walls_seen']))
                else:
                    f.write('%.3f,0,,,,,,,,,,\n' % t)


if __name__ == '__main__':
    main()
