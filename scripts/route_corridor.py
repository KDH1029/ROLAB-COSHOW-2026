#!/usr/bin/env python3
"""Narrow corridor (zigzag): replace the routes' stretch inside a box with the corridor centre line from the map,
and write a zone file the runner switches its driving on (slow, short look-ahead, turn in place at corners).

    python3 scripts/route_corridor.py <route.csv> [<route2.csv> ...] zone=x0,y0,x1,y1 [name=zigzag]
        [v=0.25] [look=0.3] [rot_deg=35] [map=maps/coshow_1119] [png=out.png]

Centre line: shortest path on a 2.5 cm grid of the map's tall obstacles (0.25..0.6 m above the floor: walls, not the
low ramp/tyre/arch-bar parts), each step costing more the closer it is to a wall, never closer than the robot half
width + 2 cm. Then straight legs (corners kept sharp, the robot turns there). Every route gets the same legs between
its own entry and exit of the box. Old route files kept as .bak_HHMMSS.csv; zones in <route>.zones.json.
"""
import heapq
import json
import math
import os
import shutil
import sys
import time

import numpy as np
from scipy import ndimage

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import track_path as TP   # noqa: E402

RES = 0.025
HALF_W = 0.155


def load_pins(path):
    D = np.genfromtxt(path, delimiter=',', names=True)
    return int(D['lane'][0]), np.column_stack([D['x_m'], D['y_m']])


def save_pins(path, lane, P):
    shutil.copy2(path, path[:-4] + time.strftime('.bak_%H%M%S.csv'))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    nxt = np.roll(P, -1, 0)
    yaw = np.degrees(np.arctan2(nxt[:, 1] - P[:, 1], nxt[:, 0] - P[:, 0]))
    with open(path, 'w') as f:
        f.write('lane,station_m,x_m,y_m,yaw_deg\n')
        for a, (x, y), b in zip(s, P, yaw):
            f.write('%d,%.3f,%.4f,%.4f,%.1f\n' % (lane, a, x, y, b))


def wall_grid(mapd, box, pad=0.4):
    z = np.load(os.path.join(mapd, 'map_clean.npz'))
    X, H = z['xyz'], z['height']
    x0, y0, x1, y1 = box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad
    nx, ny = int(math.ceil((x1 - x0) / RES)), int(math.ceil((y1 - y0) / RES))
    m = (H > 0.25) & (H < 0.6) & (X[:, 0] > x0) & (X[:, 0] < x1) & (X[:, 1] > y0) & (X[:, 1] < y1)
    gi = ((X[m, 0] - x0) / RES).astype(int)
    gj = ((X[m, 1] - y0) / RES).astype(int)
    cnt = np.zeros((ny, nx), int)
    np.add.at(cnt, (gj, gi), 1)
    wall = cnt >= 2
    dist = ndimage.distance_transform_edt(~wall) * RES
    return dist, x0, y0


def centre_path(dist, x0, y0, a, b, min_room, target=0.35, box=None):
    ny, nx = dist.shape
    ok = dist >= min_room
    if box is not None:     # through the corridor, not around the outside of the structure
        xs = x0 + (np.arange(nx) + 0.5) * RES
        ys = y0 + (np.arange(ny) + 0.5) * RES
        ok &= (xs[None, :] > box[0] - 0.1) & (xs[None, :] < box[2] + 0.1) & (ys[:, None] > box[1] - 0.1) & (ys[:, None] < box[3] + 0.1)
    cell = lambda p: (int((p[1] - y0) / RES), int((p[0] - x0) / RES))

    def nearest_ok(c):
        jj, ii = np.nonzero(ok)
        k = np.argmin((jj - c[0]) ** 2 + (ii - c[1]) ** 2)
        return int(jj[k]), int(ii[k])
    s, g = nearest_ok(cell(a)), nearest_ok(cell(b))
    cost = 1.0 + 4.0 * (target / np.maximum(dist, 1e-3)) ** 2
    best = np.full(dist.shape, np.inf)
    prev = {}
    best[s] = 0.0
    q = [(0.0, s)]
    steps = [(dj, di, math.hypot(dj, di)) for dj in (-1, 0, 1) for di in (-1, 0, 1) if dj or di]
    while q:
        d, c = heapq.heappop(q)
        if c == g:
            break
        if d > best[c]:
            continue
        for dj, di, L in steps:
            n = (c[0] + dj, c[1] + di)
            if 0 <= n[0] < ny and 0 <= n[1] < nx and ok[n]:
                nd = d + L * 0.5 * (cost[c] + cost[n])
                if nd < best[n]:
                    best[n] = nd
                    prev[n] = c
                    heapq.heappush(q, (nd, n))
    if g not in prev and g != s:
        sys.exit('no way through the corridor with %.2f m room — box or map wrong?' % min_room)
    path, c = [g], g
    while c != s:
        c = prev[c]
        path.append(c)
    path = path[::-1]
    return np.array([(x0 + (i + 0.5) * RES, y0 + (j + 0.5) * RES) for j, i in path])


def rdp(P, tol):
    if len(P) < 3:
        return P
    a, b = P[0], P[-1]
    ab = b - a
    n = np.hypot(*ab)
    d = np.abs(np.cross(ab, P - a)) / n if n > 1e-9 else np.hypot(*(P - a).T)
    k = int(np.argmax(d))
    if d[k] > tol:
        return np.vstack([rdp(P[:k + 1], tol)[:-1], rdp(P[k:], tol)])
    return np.vstack([a, b])


def main():
    paths = [a for a in sys.argv[1:] if '=' not in a]
    kv = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    box = [float(v) for v in kv['zone'].split(',')]
    name = kv.get('name', 'zigzag')
    mapd = kv.get('map', os.path.join(ROOT, 'maps/coshow_1119'))
    inbox = lambda P: (P[:, 0] > box[0]) & (P[:, 0] < box[2]) & (P[:, 1] > box[1]) & (P[:, 1] < box[3])
    routes = []
    for p in paths:
        lane, pins = load_pins(p)
        lp = TP.PinLoop(pins, s0=0.0)
        s = np.arange(0.0, lp.length, 0.05)
        S = np.array([lp.pose_at(v)[:2] for v in s])
        m = inbox(S)
        if not m.any():
            sys.exit('%s does not enter the zone' % p)
        if m[0] or m[-1]:
            sys.exit('%s starts inside the zone — not supported' % p)
        k0, k1 = int(np.argmax(m)), len(m) - 1 - int(np.argmax(m[::-1]))
        routes.append(dict(path=p, lane=lane, pins=pins, lp=lp, s=s, S=S, k0=k0, k1=k1))
    dist, x0, y0 = wall_grid(mapd, box)
    r0 = routes[0]
    a, b = r0['S'][r0['k0']], r0['S'][r0['k1']]
    raw = centre_path(dist, x0, y0, a, b, HALF_W + 0.02, box=box)
    legs = rdp(raw, 0.05)
    room = lambda P: np.array([dist[int((y - y0) / RES), int((x - x0) / RES)] for x, y in P])
    dense = []
    for p, q in zip(legs[:-1], legs[1:]):
        n = max(1, int(math.ceil(np.hypot(*(q - p)) / 0.25)))
        dense += [p + (q - p) * t for t in np.linspace(0, 1, n, endpoint=False)]
    dense.append(legs[-1])
    dense = np.array(dense)
    turns = [math.degrees(abs(TP.wrap(math.atan2(*(c - b_)[::-1]) - math.atan2(*(b_ - a_)[::-1]))))
             for a_, b_, c in zip(legs[:-2], legs[1:-1], legs[2:])]
    rd = room(dense)
    k = int(np.argmin(rd))
    print('%s: corridor centre line %.2f m, %d legs, corner turns %s deg, room to walls min %.2f m at (%.2f, %.2f) '
          '(old route min %.2f)' % (name, np.hypot(*np.diff(dense, axis=0).T).sum(), len(legs) - 1,
                                    ', '.join('%.0f' % t for t in turns), rd.min(), dense[k, 0], dense[k, 1],
                                    room(r0['S'][r0['k0']:r0['k1'] + 1]).min()))
    for r in routes:
        before = r['pins'][[r['lp'].station(*p) < r['s'][r['k0']] - 0.15 for p in r['pins']]]
        after = r['pins'][[r['lp'].station(*p) > r['s'][r['k1']] + 0.15 for p in r['pins']]]
        P = np.vstack([before, dense, after])
        save_pins(r['path'], r['lane'], P)
        zf = r['path'][:-4] + '.zones.json'
        zones = json.load(open(zf)) if os.path.exists(zf) else []
        zones = [z for z in zones if z.get('name') != name]
        zones.append(dict(name=name, box=box, mode='legs', v=float(kv.get('v', 0.25)), lookahead=float(kv.get('look', 0.3)),
                          rotate=True, rotate_min_deg=float(kv.get('rot_deg', 35)), corners=legs.round(3).tolist()))
        json.dump(zones, open(zf, 'w'), indent=1)
        print('  %s: %d -> %d pins, zone file %s' % (os.path.basename(r['path']), len(r['pins']), len(P), os.path.basename(zf)))
    if 'png' in kv:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
        import plot_plan
        plot_plan._font()
        fig, ax = plt.subplots(figsize=(11, 10))
        ext = (x0, x0 + dist.shape[1] * RES, y0, y0 + dist.shape[0] * RES)
        ax.imshow(np.minimum(dist, 0.6), origin='lower', extent=ext, cmap='magma')
        ax.contour(np.linspace(ext[0], ext[1], dist.shape[1]), np.linspace(ext[2], ext[3], dist.shape[0]), dist,
                   levels=[HALF_W], colors='cyan', linewidths=0.8)
        old = r0['S'][max(0, r0['k0'] - 10):r0['k1'] + 10]
        ax.plot(old[:, 0], old[:, 1], color='w', lw=1.2, ls='--', label='예전 경로 (리모컨)')
        ax.plot(dense[:, 0], dense[:, 1], color='#2ecc71', lw=2.4, label='통로 가운데 선')
        ax.plot(legs[:, 0], legs[:, 1], 'o', color='#2ecc71', mec='k', ms=8, label='모서리 (제자리 회전)')
        ax.add_patch(__import__('matplotlib.patches', fromlist=['Rectangle']).Rectangle(
            (box[0], box[1]), box[2] - box[0], box[3] - box[1], fill=False, ec='y', ls=':', lw=1.5, label='구간'))
        ax.set_xlim(ext[0], ext[1])
        ax.set_ylim(ext[2], ext[3])
        ax.set_aspect('equal')
        ax.legend(loc='upper right', fontsize=9)
        ax.set_title('%s: 벽까지 거리(밝을수록 멂, 하늘색 = 로봇 반폭 0.155 m 선)' % name, fontsize=13)
        fig.tight_layout()
        fig.savefig(kv['png'], dpi=110)
        print('picture %s' % kv['png'])


if __name__ == '__main__':
    main()
