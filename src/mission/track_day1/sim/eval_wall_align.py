#!/usr/bin/env python3
"""Synthetic check of wall_align: XT16-like scans in the walled arena, random start poses.

XT16: 16 rings -15..+15 deg every 2 deg, 0.2 deg azimuth (1800 per ring), 2 cm range noise, 20 Hz.
Lidar 0.25 m ahead of the base, base `base_h` above the floor (lidar ~0.46 m). The wall (0.6 m) stands
WALL_GAP outside the track outline; each face gets its own placement error (wall_err) so the solver
cannot rely on the nominal size. Scene extras: the other team's robot (box 0.70 x 0.31 x 0.40 m) in the
other lane near us, our own legs (points within 0.4 m), people/booths behind the wall (rays that clear
the wall top hit something 0.5-3 m behind it, up to 1.8 m high, with probability p_crowd).

Start poses: either lane, anywhere along it, lateral +-0.30 m, heading +-10 deg off the travel direction.
Reports pose error percentiles over `n` trials per condition, and failures (ok=False) separately.
    python3 sim/eval_wall_align.py [n=200]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import track_spec as T          # noqa: E402
import wall_align as W          # noqa: E402

RINGS = np.deg2rad(np.arange(-15, 16, 2))
AZ = np.deg2rad(np.arange(0, 360, 0.2))
LIDAR_X = 0.25


def lane_point(lane, s):
    """Point and CCW heading at arc length s along the lane centre (start: bottom straight, x=-1)."""
    r = T.R_CENTER[lane]
    L = T.STRAIGHT
    segs = [L, math.pi * r, L, math.pi * r]
    s = s % sum(segs)
    if s < segs[0]:
        return -L / 2 + s, -r, 0.0
    s -= segs[0]
    if s < segs[1]:
        a = -math.pi / 2 + s / r
        return L / 2 + r * math.cos(a), r * math.sin(a), a + math.pi / 2
    s -= segs[1]
    if s < segs[2]:
        return L / 2 - s, r, math.pi
    s -= segs[2]
    a = math.pi / 2 + s / r
    return -L / 2 + r * math.cos(a), r * math.sin(a), a + math.pi / 2


def scan(pose, base_h, walls, other, rng, p_crowd=0.5, noise=0.02):
    """One XT16 frame in the robot base frame. walls = (xmin, xmax, ymin, ymax) inner faces, track frame."""
    x0, y0, yaw = pose
    lx, ly = x0 + LIDAR_X * math.cos(yaw), y0 + LIDAR_X * math.sin(yaw)
    hz = base_h + 0.159
    az = AZ[None, :] + yaw
    el = RINGS[:, None]
    dx, dy = np.cos(az), np.sin(az)
    dx, dy = np.broadcast_to(dx, (len(RINGS), len(AZ))), np.broadcast_to(dy, (len(RINGS), len(AZ)))
    el = np.broadcast_to(el, dx.shape)
    big = 1e9
    tx = np.where(dx > 1e-9, (walls[1] - lx) / np.where(dx > 1e-9, dx, 1), np.where(dx < -1e-9, (walls[0] - lx) / np.where(dx < -1e-9, dx, 1), big))
    ty = np.where(dy > 1e-9, (walls[3] - ly) / np.where(dy > 1e-9, dy, 1), np.where(dy < -1e-9, (walls[2] - ly) / np.where(dy < -1e-9, dy, 1), big))
    t_wall = np.minimum(tx, ty)
    z_wall = hz + t_wall * np.tan(el)
    t = np.full(dx.shape, np.inf)
    hit_wall = (z_wall >= 0) & (z_wall <= T.WALL_H)
    t = np.where(hit_wall, t_wall, t)
    # floor
    t_floor = np.where(el < 0, hz / np.tan(-np.minimum(el, -1e-9)), np.inf)
    t = np.where(t_floor < t, t_floor, t)
    # crowd behind the wall
    over = (z_wall > T.WALL_H) & (rng.random(dx.shape) < p_crowd)
    t_crowd = t_wall + rng.uniform(0.5, 3.0, dx.shape)
    z_crowd = hz + t_crowd * np.tan(el)
    t = np.where(over & (z_crowd < 1.8) & (t_crowd < t), t_crowd, t)
    # other robot: box in the track frame (cx, cy, heading), 0.70 x 0.31 x 0.40
    if other is not None:
        ox, oy, oth = other
        c, s = math.cos(-oth), math.sin(-oth)
        px, py = c * (lx - ox) - s * (ly - oy), s * (lx - ox) + c * (ly - oy)
        ddx, ddy = c * dx - s * dy, s * dx + c * dy
        with np.errstate(divide='ignore', invalid='ignore'):
            t1x, t2x = (-0.35 - px) / ddx, (0.35 - px) / ddx
            t1y, t2y = (-0.155 - py) / ddy, (0.155 - py) / ddy
        tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
        tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
        zb = hz + tmin * np.tan(el)
        hb = (tmax >= tmin) & (tmin > 0) & (zb >= 0.05) & (zb <= 0.45) & (tmin < t)
        t = np.where(hb, tmin, t)
    ok = np.isfinite(t) & (t < 30)
    t = t + rng.normal(0, noise, t.shape)
    # points in the base frame (lidar at (LIDAR_X, 0, 0.159))
    rel_az = AZ[None, :] + 0 * el
    px = LIDAR_X + t * np.cos(el) * np.cos(rel_az)
    py = t * np.cos(el) * np.sin(rel_az)
    pz = 0.159 + t * np.sin(el)
    P = np.stack([px[ok], py[ok], pz[ok]], 1)
    # own legs: a few hundred points near the body
    legs = np.stack([rng.uniform(-0.3, 0.3, 300), rng.uniform(-0.2, 0.2, 300), rng.uniform(-base_h, -0.05, 300)], 1)
    return np.vstack([P, legs])


def trial(rng, cond):
    lane = int(rng.integers(1, 3))
    s = rng.uniform(0, T.LAP_LEN[lane])
    cx, cy, th = lane_point(lane, s)
    lat = rng.uniform(-0.30, 0.30)
    # lateral offset: + = left of travel = inward
    nx, ny = -math.sin(th), math.cos(th)
    x, y = cx + lat * nx, cy + lat * ny
    yaw = th + math.radians(rng.uniform(-10, 10))
    e = rng.uniform(-cond['wall_err'], cond['wall_err'], 4)
    walls = (-T.WALL_LX / 2 + e[0], T.WALL_LX / 2 + e[1], -T.WALL_LY / 2 + e[2], T.WALL_LY / 2 + e[3])
    other = None
    if cond['other']:
        ol = 3 - lane
        os_ = s * T.LAP_LEN[ol] / T.LAP_LEN[lane] + rng.uniform(-0.5, 0.5)
        ox, oy, oth = lane_point(ol, os_)
        other = (ox, oy, oth)
    base_h = rng.uniform(0.27, 0.33)
    frames = [scan((x, y, yaw), base_h, walls, other, rng, p_crowd=cond['crowd']) for _ in range(cond['frames'])]
    r = W.align(frames, seed=int(rng.integers(1 << 30)))
    if not r['ok']:
        return None, r['why']
    # the 180 deg twin is equally right: compare against the nearer of (x, y, yaw) and (-x, -y, yaw+pi)
    best = None
    for sx in (1, -1):
        ex = math.hypot(r['x'] - sx * x, r['y'] - sx * y)
        ey = abs(math.degrees(math.atan2(math.sin(r['yaw'] - yaw - (0 if sx == 1 else math.pi)),
                                         math.cos(r['yaw'] - yaw - (0 if sx == 1 else math.pi)))))
        if best is None or ex < best[0]:
            best = (ex, ey)
    lat_true = T.lane_frame(lane, x, y)[0]
    return (best[0], best[1], abs(r['lateral'] - lat_true), r['lane'] == lane), ''


def main():
    n = int(sys.argv[1].split('=')[-1]) if len(sys.argv) > 1 else 200
    conds = [
        dict(name='clean, 1 frame', other=False, crowd=0.0, wall_err=0.0, frames=1),
        dict(name='wall +-5 cm, crowd', other=False, crowd=0.5, wall_err=0.05, frames=1),
        dict(name='+ other robot', other=True, crowd=0.5, wall_err=0.05, frames=1),
        dict(name='+ other robot, 5 frames', other=True, crowd=0.5, wall_err=0.05, frames=5),
        dict(name='wall +-10 cm, other, crowd 0.9', other=True, crowd=0.9, wall_err=0.10, frames=3),
    ]
    rng = np.random.default_rng(7)
    print('%-34s %5s %20s %20s %16s %6s' % ('condition', 'fail', 'pos p50/p95/max cm', 'yaw p50/p95/max deg',
                                            'lat p95/max cm', 'lane'))
    for c in conds:
        E, fails = [], {}
        for _ in range(n):
            e, why = trial(rng, c)
            if e is None:
                fails[why.split('(')[0].strip()] = fails.get(why.split('(')[0].strip(), 0) + 1
            else:
                E.append(e)
        A = np.array(E, float) if E else np.zeros((0, 4))
        def pc(v):
            return '%5.1f/%5.1f/%5.1f' % (np.percentile(v, 50), np.percentile(v, 95), v.max()) if len(v) else '-'
        print('%-34s %5d %20s %20s %16s %5.0f%%' % (
            c['name'], n - len(E), pc(A[:, 0] * 100), pc(A[:, 1]),
            ('%5.1f/%5.1f' % (np.percentile(A[:, 2] * 100, 95), A[:, 2].max() * 100)) if len(A) else '-',
            100 * A[:, 3].mean() if len(A) else 0))
        if fails:
            print('    failures:', fails)


if __name__ == '__main__':
    main()
