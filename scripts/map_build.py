#!/usr/bin/env python3
"""FAST-LIO output (/cloud_registered, /Odometry recorded by scripts/map_from_bag.sh) -> arena map in the track frame.

  1. 2 cm voxels of every registered scan (count >= min_count: drops single stray points)
  2. level: RANSAC floor plane of the whole map -> floor z = 0 (FAST-LIO's frame inherits the start tilt)
  3. align: wall_align on the levelled map -> track frame (origin = track centre, x = long axis, the mapping start on
     the lower half y < 0, as in day 1). Measured wall spacing printed (arena 9.00 x 7.00)
  4. outputs in <out>: map_track.pcd (x y z intensity=count), map_track.npz, map.pgm/map.yaml (Nav2, 5 cm, occupied =
     points 0.05..0.60 m above the local floor standing >= 0.10 m tall in the cell), map_top.png, report.txt
    python3 scripts/map_build.py <lio bag dir> <original bag dir> <out dir>
    python3 scripts/map_build.py --regrid <out dir>      # only the 2D grid and picture again, from map_track.npz
    python3 scripts/map_build.py --clean <out dir>       # arena-only cleaned map_clean.npz/pcd (+ grid, picture)
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import track_path as TP   # noqa: E402
import track_spec as T    # noqa: E402
import wall_align as W    # noqa: E402

VOX = 0.02
MIN_COUNT = 2
GRID = 0.05
BAND = (0.05, 0.60)                # robot (with the XT16) is ~0.5 m tall: higher things (arch beam) do not block


def reader(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while r.has_next():
        yield r.read_next()


def accumulate(bag):
    keys, cnts, odo, n_scan = [], [], [], 0
    buf = []

    def flush():
        if not buf:
            return
        k = np.concatenate(buf)
        u, c = np.unique(k, return_counts=True)
        keys.append(u)
        cnts.append(c)
        buf.clear()
    for topic, data, t in reader(bag, ['/cloud_registered', '/Odometry']):
        if topic == '/Odometry':
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odo.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            continue
        m = deserialize_message(data, PointCloud2)
        P = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        P = P[np.isfinite(P).all(1) & (np.abs(P) < 50).all(1)]
        g = np.floor(P / VOX).astype(np.int64) + 2 ** 20
        buf.append((g[:, 0] << 42) | (g[:, 1] << 21) | g[:, 2])
        n_scan += 1
        if len(buf) >= 50:
            flush()
    flush()
    k = np.concatenate(keys)
    c = np.concatenate(cnts)
    u, inv = np.unique(k, return_inverse=True)
    cs = np.bincount(inv, weights=c).astype(np.int64)
    g = np.column_stack([(u >> 42) & (2 ** 21 - 1), (u >> 21) & (2 ** 21 - 1), u & (2 ** 21 - 1)]) - 2 ** 20
    xyz = (g + 0.5) * VOX
    return xyz, cs, np.array(odo), n_scan


def fit_floor(xyz, rng):
    lo = xyz[xyz[:, 2] < np.percentile(xyz[:, 2], 30)]
    if len(lo) > 200000:
        lo = lo[rng.choice(len(lo), 200000, replace=False)]
    best, bn = None, 0
    for _ in range(300):
        a, b, c = lo[rng.choice(len(lo), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if n[2] < 0:
            n = -n
        if n[2] < 0.95:
            continue
        k = int((np.abs((lo - a) @ n) < 0.02).sum())
        if k > bn:
            best, bn = (n, a), k
    n, a = best
    inl = lo[np.abs((lo - a) @ n) < 0.02]
    cc = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - cc, full_matrices=False)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    return n, cc


def make_grid(M, hz, span=0.10, min_pts=3):
    """5 cm cells. Occupied = points between BAND that STAND UP: height span >= span within the cell (walls, cones,
    tyres, posts). A thin sheet (floor seen a few cm too high while the robot pitched, ramp surface) stays free."""
    x0, y0 = -T.WALL_LX / 2 - 0.5, -T.WALL_LY / 2 - 0.5
    nx, ny = int((T.WALL_LX + 1.0) / GRID), int((T.WALL_LY + 1.0) / GRID)
    gi = np.floor((M[:, 0] - x0) / GRID).astype(int)
    gj = np.floor((M[:, 1] - y0) / GRID).astype(int)
    ok = (gi >= 0) & (gi < nx) & (gj >= 0) & (gj < ny)
    ob = ok & (hz > BAND[0]) & (hz < BAND[1])
    cid = gj[ob] * nx + gi[ob]
    hmax = np.full(nx * ny, -9.0)
    hmin = np.full(nx * ny, 9.0)
    cnt = np.zeros(nx * ny, int)
    np.maximum.at(hmax, cid, hz[ob])
    np.minimum.at(hmin, cid, hz[ob])
    np.add.at(cnt, cid, 1)
    occupied = ((cnt >= min_pts) & (hmax - hmin >= span)).reshape(ny, nx)
    seen = np.zeros((ny, nx), bool)
    fb = ok & (np.abs(hz) < 0.04)
    seen[gj[fb], gi[fb]] = True
    info = ('grid %d x %d at %.2f m: occupied %d cells (points %.2f..%.2f m above the local floor standing >= %.2f m tall '
            'in the cell), free %d' % (nx, ny, GRID, int(occupied.sum()), BAND[0], BAND[1], span, int((seen & ~occupied).sum())))
    return occupied, seen, x0, y0, info


def write_grid(out, occupied, seen, x0, y0):
    ny, nx = occupied.shape
    img = np.full((ny, nx), 205, np.uint8)
    img[seen] = 254
    img[occupied] = 0
    with open(os.path.join(out, 'map.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (nx, ny))
        f.write(img[::-1].tobytes())                         # pgm rows top = +y
    with open(os.path.join(out, 'map.yaml'), 'w') as f:
        f.write('image: map.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n# track frame (track centre, x = long axis), floor z = 0\n'
                % (GRID, x0, y0))


def clean(M, hz, cnt, margin=0.10, zmax=1.2, low=(0.03, 0.25), low_count=6, base_count=3, min_nb=4):
    """Arena-only map: inside the wall (+margin), floor..zmax, the 3-25 cm layer only where seen >= low_count times
    (floor streaks from pitch lag are seen by few scans; ramps, tyres and cone bases by many), then voxels with fewer
    than min_nb of their 26 neighbours dropped (lone specks). Returns the kept mask."""
    hx, hy = T.WALL_LX / 2 + margin, T.WALL_LY / 2 + margin
    keep = (np.abs(M[:, 0]) < hx) & (np.abs(M[:, 1]) < hy) & (hz > -0.03) & (hz < zmax) & (cnt >= base_count)
    lay = (hz >= low[0]) & (hz < low[1])
    keep &= ~lay | (cnt >= low_count)
    idx = np.nonzero(keep)[0]
    g = np.floor(M[idx] / VOX).astype(np.int64) + 2 ** 20
    key = (g[:, 0] << 42) | (g[:, 1] << 21) | g[:, 2]
    ks = np.sort(key)
    nb = np.zeros(len(idx), int)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == dy == dz == 0:
                    continue
                q = key + (dx << 42) + (dy << 21) + dz
                pos = np.searchsorted(ks, q)
                pos[pos >= len(ks)] = len(ks) - 1
                nb += ks[pos] == q
    keep[idx[nb < min_nb]] = False
    return keep


def write_pcd(path, M, val):
    with open(path, 'wb') as f:
        f.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                 'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (len(M), len(M))).encode())
        f.write(np.column_stack([M, val]).astype(np.float32).tobytes())


def clean_map(out):
    """map_track.npz -> map_clean.npz / map_clean.pcd (intensity = height above floor) + 2D grid from the clean points."""
    d = np.load(os.path.join(out, 'map_track.npz'))
    M, hz, cnt, traj = d['xyz'].astype(np.float64), d['height'].astype(np.float64), d['count'], d['traj']
    k = clean(M, hz, cnt)
    np.savez_compressed(os.path.join(out, 'map_clean.npz'), xyz=M[k].astype(np.float32), height=hz[k].astype(np.float32),
                        count=cnt[k].astype(np.int32), traj=traj)
    write_pcd(os.path.join(out, 'map_clean.pcd'), M[k], hz[k])
    occupied, seen, x0, y0, info = make_grid(M[k], hz[k])
    write_grid(out, occupied, seen, x0, y0)
    picture(os.path.join(out, 'map_top.png'), M[k], hz[k], traj, occupied, x0, y0, os.path.basename(out.rstrip('/')) + ' (clean)')
    msg = 'clean: kept %d of %d voxels (inside the wall, floor..1.2 m, 3-25 cm layer seen >= 6x, lone specks dropped)' % (k.sum(), len(k))
    lines = [l for l in open(os.path.join(out, 'report.txt')).read().splitlines() if not l.startswith(('grid ', 'clean:'))]
    with open(os.path.join(out, 'report.txt'), 'w') as f:
        f.write('\n'.join(lines + [msg, info]) + '\n')
    print(msg)
    print(info)


def regrid(out):
    """Rebuild map.pgm/map.yaml/map_top.png from map_track.npz (no replay)."""
    d = np.load(os.path.join(out, 'map_track.npz'))
    M, hz, traj = d['xyz'].astype(np.float64), d['height'].astype(np.float64), d['traj']
    occupied, seen, x0, y0, info = make_grid(M, hz)
    write_grid(out, occupied, seen, x0, y0)
    picture(os.path.join(out, 'map_top.png'), M, hz, traj, occupied, x0, y0, os.path.basename(out.rstrip('/')))
    lines = [l for l in open(os.path.join(out, 'report.txt')).read().splitlines() if not l.startswith('grid ')] + [info]
    with open(os.path.join(out, 'report.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(info)


def rot_to_z(n):
    """Rotation taking unit vector n onto +z."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s, c = np.linalg.norm(v), float(n @ z)
    if s < 1e-12:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / s ** 2)


def main():
    lio, orig, out = sys.argv[1], sys.argv[2], sys.argv[3]
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(0)
    xyz, cnt, odo, n_scan = accumulate(lio)
    keep = cnt >= MIN_COUNT
    xyz, cnt = xyz[keep], cnt[keep]
    rep = ['source %s' % os.path.basename(orig.rstrip('/')), 'scans %d, voxels %d (2 cm, seen >= %d times)' % (n_scan, len(xyz), MIN_COUNT)]

    # level: floor plane -> z = 0 (keep the robot start height above it for wall_align)
    n, c0 = fit_floor(xyz, rng)
    R = rot_to_z(n)
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    L = (xyz - c0) @ R.T                                     # floor through the origin of z
    o_lev = (np.zeros(3) - c0) @ R.T                         # camera_init origin in the levelled frame
    rep.append('floor tilt corrected %.2f deg, mapping start %.3f m above the floor' % (tilt, o_lev[2]))

    # align to the wall: run wall_align with the mapping start as the "robot"
    Q = L - o_lev                                            # origin at the start, floor at z = -height
    sub = Q[rng.choice(len(Q), min(len(Q), 600000), replace=False)]
    wa = W.align([sub], cfg=dict(max_pts=8000, r_range=(0.3, 15.0)))
    if not wa.get('ok'):
        sys.exit('wall_align failed on the map: %s' % wa.get('why'))
    Tt = (wa['x'], wa['y'], wa['yaw'])                       # start pose in the track frame
    rep.append('wall fit: spacing %.3f x %.3f m (arena 9.00 x 7.00), rms %.3f m, walls %d; mapping start at track (%.2f, %.2f) yaw %.1f deg'
               % (wa['dims'][0], wa['dims'][1], wa['rms'], wa['walls_seen'], Tt[0], Tt[1], math.degrees(Tt[2])))
    XY = TP.apply(Tt, Q[:, :2])
    M = np.column_stack([XY, L[:, 2]])                       # track frame, floor z = 0

    # trajectory in the same frame
    if len(odo):
        Po = (odo[:, 1:4] - c0) @ R.T - o_lev
        traj = np.column_stack([TP.apply(Tt, Po[:, :2]), Po[:, 2] + o_lev[2]])
        d = float(np.hypot(*np.diff(traj[:, :2], axis=0).T).sum())
        rep.append('trajectory %.1f m, %.0f s, start-end %.2f m, height %.2f..%.2f m'
                   % (d, odo[-1, 0] - odo[0, 0], float(np.hypot(*(traj[-1, :2] - traj[0, :2]))), traj[:, 2].min(), traj[:, 2].max()))
    else:
        traj = np.zeros((0, 3))

    # local floor (0.5 m cells) and the 2D grid
    fl = M[np.abs(M[:, 2]) < 0.10]
    cell = 0.5
    fk = np.floor(fl[:, :2] / cell).astype(int)
    floor_h = {}
    order = np.lexsort((fk[:, 1], fk[:, 0]))
    fk, fz = fk[order], fl[order, 2]
    brk = np.nonzero(np.any(np.diff(fk, axis=0) != 0, axis=1))[0] + 1
    for a, b in zip(np.r_[0, brk], np.r_[brk, len(fk)]):
        if b - a >= 20:
            floor_h[tuple(fk[a])] = float(np.median(fz[a:b]))
    fh = np.array([floor_h.get(tuple(k), 0.0) for k in np.floor(M[:, :2] / cell).astype(int)])
    hz = M[:, 2] - fh
    flat = fl[:, 2] - np.array([floor_h.get(tuple(k), 0.0) for k in np.floor(fl[:, :2] / cell).astype(int)])
    rep.append('floor: %d cells, local floor heights %.3f..%.3f m, floor thickness (p5..p95 around it) %.3f m'
               % (len(floor_h), min(floor_h.values()), max(floor_h.values()), np.percentile(flat, 95) - np.percentile(flat, 5)))
    occupied, seen, x0, y0, info = make_grid(M, hz)
    write_grid(out, occupied, seen, x0, y0)
    rep.append(info)

    # point cloud
    with open(os.path.join(out, 'map_track.pcd'), 'wb') as f:
        f.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                 'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (len(M), len(M))).encode())
        f.write(np.column_stack([M, cnt]).astype(np.float32).tobytes())
    np.savez_compressed(os.path.join(out, 'map_track.npz'), xyz=M.astype(np.float32), count=cnt.astype(np.int32),
                        height=hz.astype(np.float32), traj=traj.astype(np.float32), voxel=VOX,
                        T_level=R, floor_point=c0, start_track=np.array(Tt), wall_dims=np.array(wa['dims']))

    picture(os.path.join(out, 'map_top.png'), M, hz, traj, occupied, x0, y0, os.path.basename(orig.rstrip('/')))
    with open(os.path.join(out, 'report.txt'), 'w') as f:
        f.write('\n'.join(rep) + '\n')
    print('\n'.join(rep))
    clean_map(out)
    print('-> %s' % out)


def picture(path, M, hz, traj, occupied, x0, y0, name):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
    import plot_plan
    plot_plan._font()
    fig, axs = plt.subplots(1, 2, figsize=(19, 7.6))
    ax = axs[0]
    sel = (hz > 0.03) & (hz < 1.5)
    order = np.argsort(hz[sel])
    sc = ax.scatter(M[sel, 0][order], M[sel, 1][order], c=hz[sel][order], s=0.3, cmap='turbo', vmin=0, vmax=0.8)
    plt.colorbar(sc, ax=ax, fraction=0.03, label='바닥 위 높이 (m)')
    for r in (T.R_OUTER, T.R_LANE12, T.R_INFIELD):
        p = plot_plan._stadium(r)
        ax.plot(p[:, 0], p[:, 1], color='k', lw=0.6, alpha=0.6)
    if len(traj):
        ax.plot(traj[:, 0], traj[:, 1], color='m', lw=0.8, label='로봇 경로')
        ax.plot(traj[0, 0], traj[0, 1], 'o', color='g', ms=8, label='매핑 시작')
        ax.legend(loc='upper right', fontsize=9)
    ax.set_aspect('equal')
    ax.set_title('%s — 바닥 위 물체 (높이 색), 트랙 선은 규정 치수' % name)
    ax = axs[1]
    ax.imshow(occupied[::-1], cmap='gray_r', extent=(x0, x0 + occupied.shape[1] * GRID, y0, y0 + occupied.shape[0] * GRID))
    for r in (T.R_OUTER, T.R_LANE12, T.R_INFIELD):
        p = plot_plan._stadium(r)
        ax.plot(p[:, 0], p[:, 1], color='tab:blue', lw=0.6, alpha=0.6)
    ax.set_title('2D 지도 (5 cm, 바닥 위 %.2f~%.2f m 점유)' % BAND)
    for a in axs:
        a.set_xlim(-T.WALL_LX / 2 - 0.4, T.WALL_LX / 2 + 0.4)
        a.set_ylim(-T.WALL_LY / 2 - 0.4, T.WALL_LY / 2 + 0.4)
        a.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--regrid':
        regrid(sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == '--clean':
        clean_map(sys.argv[2])
    else:
        main()
