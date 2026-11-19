#!/usr/bin/env python3
"""A driven route from a recording, in the track frame of a built map (scripts/map_build.py).

  inputs: the recording (XT16 /lidar_points), its FAST-LIO odometry (tests/fastlio_replay.sh CSV, bag time), the map dir
  1. track <- odom from the wall: wall_align on every `step`-th frame, each gives wall o odom^-1; median over the run
     (spread printed = how well FAST-LIO and the wall agree over the whole drive)
  2. the wall cannot tell the two 180 deg twins apart: take the one whose drive crosses fewer occupied map cells
  3. outputs in <map>/routes/<name>: route.csv (0.25 m pins: station, x, y, yaw, t, speed), raw.csv (every pose),
     route.png (drive over the map, time-coloured, stops), summary.txt
    python3 scripts/route_from_bag.py <bag dir> <lio.csv> <map dir> <name> [step=5]
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
import track_path as TP   # noqa: E402
import track_spec as T    # noqa: E402
import wall_align as W    # noqa: E402
from wall_align_bag import frames   # noqa: E402


def load_grid(d):
    y = {}
    for line in open(os.path.join(d, 'map.yaml')):
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1)
            y[k.strip()] = v.split('#')[0].strip()
    raw = open(os.path.join(d, y['image']), 'rb').read().split(b'\n', 3)
    w, h = map(int, raw[1].split())
    img = np.frombuffer(raw[3], np.uint8).reshape(h, w)[::-1]
    org = [float(v) for v in y['origin'].strip('[]').split(',')]
    return img == 0, float(y['resolution']), org[0], org[1]


def main():
    bag, lio, mapd, name = sys.argv[1:5]
    kv = dict(a.split('=') for a in sys.argv[5:])
    step = int(kv.get('step', 5))
    D = np.genfromtxt(lio, delimiter=',', names=True)
    L = np.column_stack([D['t'], D['x'], D['y'], D['yaw']])
    L = L[np.concatenate([[True], np.diff(L[:, 0]) > 1e-6])]
    yu = np.unwrap(L[:, 3])

    # 1. track <- odom from every step-th frame
    Ts, prior, n = [], None, 0
    for i, (t, P) in enumerate(frames(bag)):
        if i % step or t < L[0, 0] or t > L[-1, 0]:
            continue
        n += 1
        r = W.align([P], seed=i, prior=prior)
        if not r['ok']:
            continue
        prior = (r['x'], r['y'], r['yaw'])
        od = (np.interp(t, L[:, 0], L[:, 1]), np.interp(t, L[:, 0], L[:, 2]), np.interp(t, L[:, 0], yu))
        Ts.append(TP.compose(prior, TP.inverse(od)))
    A = np.array(Ts)
    yc = math.atan2(np.sin(A[:, 2]).mean(), np.cos(A[:, 2]).mean())
    # one twin for all: flip any sample on the other side of the mean heading
    for k in range(len(A)):
        if abs(TP.wrap(A[k, 2] - yc)) > math.pi / 2:
            A[k] = TP.compose((0.0, 0.0, math.pi), tuple(A[k]))
    yc = math.atan2(np.sin(A[:, 2]).mean(), np.cos(A[:, 2]).mean())
    Tm = (float(np.median(A[:, 0])), float(np.median(A[:, 1])), yc)
    dev = np.hypot(A[:, 0] - Tm[0], A[:, 1] - Tm[1])
    ydev = np.degrees(np.abs([TP.wrap(a - yc) for a in A[:, 2]]))

    # 2. twin: fewer drive points on occupied map cells
    occ, res, x0, y0 = load_grid(mapd)

    def on_occ(Tq):
        XY = TP.apply(Tq, L[:, 1:3])
        i = np.floor((XY[:, 0] - x0) / res).astype(int)
        j = np.floor((XY[:, 1] - y0) / res).astype(int)
        ok = (i >= 0) & (i < occ.shape[1]) & (j >= 0) & (j < occ.shape[0])
        return int(occ[j[ok], i[ok]].sum()) + int((~ok).sum())
    twin = TP.compose((0.0, 0.0, math.pi), Tm)
    c0, c1 = on_occ(Tm), on_occ(twin)
    if c1 < c0:
        Tm = twin
    XY = TP.apply(Tm, L[:, 1:3])
    yaw = yu + Tm[2]
    t = L[:, 0] - L[0, 0]

    # 3. route, stops, outputs
    out = os.path.join(mapd, 'routes', name)
    os.makedirs(out, exist_ok=True)
    seg = np.hypot(*np.diff(XY, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    sp = np.convolve(np.concatenate([[0.0], seg / np.maximum(np.diff(t), 1e-3)]), np.ones(10) / 10, mode='same')
    stops, i = [], 0
    while i < len(t):
        if sp[i] < 0.05:
            j = i
            while j + 1 < len(t) and sp[j + 1] < 0.05:
                j += 1
            if t[j] - t[i] >= 1.0:
                stops.append((t[i], t[j], XY[i]))
            i = j + 1
        else:
            i += 1
    sq = np.arange(0.0, s[-1], 0.25)
    Q = np.column_stack([np.interp(sq, s, XY[:, 0]), np.interp(sq, s, XY[:, 1])])
    w = 3
    if len(Q) > 2 * w:
        k = np.ones(2 * w + 1) / (2 * w + 1)
        Qs = Q.copy()
        Qs[w:-w] = np.column_stack([np.convolve(Q[:, j], k, mode='valid') for j in (0, 1)])
        Q = Qs
    qy = np.degrees(np.arctan2(np.gradient(Q[:, 1]), np.gradient(Q[:, 0])))
    qt = np.interp(sq, s, t)
    qv = np.interp(sq, s, sp)
    with open(os.path.join(out, 'route.csv'), 'w') as f:
        f.write('station_m,x_m,y_m,yaw_deg,t_s,speed_mps\n')
        for a, (x, y), b, c, d in zip(sq, Q, qy, qt, qv):
            f.write('%.2f,%.4f,%.4f,%.1f,%.2f,%.3f\n' % (a, x, y, b, c, d))
    np.savetxt(os.path.join(out, 'raw.csv'), np.column_stack([t, XY, np.degrees(np.arctan2(np.sin(yaw), np.cos(yaw))), sp]),
               delimiter=',', fmt='%.4f', header='t_s,x_m,y_m,yaw_deg,speed_mps', comments='')
    rep = ['source %s, map %s' % (os.path.basename(bag.rstrip('/')), os.path.basename(mapd.rstrip('/'))),
           'wall poses %d of %d frames; track<-odom spread over the drive: median %.1f cm / max %.1f cm, yaw median %.2f / max %.2f deg'
           % (len(A), n, 100 * np.median(dev), 100 * dev.max(), np.median(ydev), ydev.max()),
           'twin by the map: drive on occupied cells %d vs other twin %d' % (min(c0, c1), max(c0, c1)),
           'drive %.1f m in %.1f s (moving avg %.2f m/s, max %.2f), start (%.2f, %.2f) yaw %.0f, end (%.2f, %.2f), start-end %.2f m'
           % (s[-1], t[-1], s[-1] / max(1e-3, t[-1] - sum(b - a for a, b, _ in stops)), sp.max(), XY[0, 0], XY[0, 1],
              math.degrees(TP.wrap(yaw[0])), XY[-1, 0], XY[-1, 1], float(np.hypot(*(XY[-1] - XY[0])))),
           'stops >= 1 s: ' + (', '.join('%.0f-%.0fs at (%.2f,%.2f)' % (a, b, p[0], p[1]) for a, b, p in stops) or 'none')]
    with open(os.path.join(out, 'summary.txt'), 'w') as f:
        f.write('\n'.join(rep) + '\n')
    print('\n'.join(rep))
    picture(os.path.join(out, 'route.png'), mapd, XY, t, stops, name)
    print('-> %s' % out)


def picture(path, mapd, XY, t, stops, name):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import plot_plan
    plot_plan._font()
    z = np.load(os.path.join(mapd, 'map_clean.npz'))
    M, hz = z['xyz'], z['height']
    fig, ax = plt.subplots(figsize=(13, 10))
    sel = (hz > 0.05)
    ax.scatter(M[sel, 0], M[sel, 1], c='#9aa3ad', s=0.4)
    for r in (T.R_OUTER, T.R_LANE12, T.R_INFIELD):
        p = plot_plan._stadium(r)
        ax.plot(p[:, 0], p[:, 1], color='#c9ced6', lw=0.8)
    sc = ax.scatter(XY[:, 0], XY[:, 1], c=t, cmap='viridis', s=3)
    plt.colorbar(sc, ax=ax, fraction=0.03, label='시간 (s)')
    for k in range(0, len(XY) - 10, 60):
        a, b = XY[k], XY[k + 10]
        if np.hypot(*(b - a)) > 0.05:
            ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='-|>', color='k', lw=1.0))
    ax.plot(*XY[0], 'o', ms=14, mfc='#27ae60', mec='white', mew=2, label='시작')
    ax.plot(*XY[-1], 's', ms=13, mfc='#c0392b', mec='white', mew=2, label='끝')
    for a, b, p in stops:
        ax.plot(*p, 'x', color='k', ms=10, mew=2)
        ax.annotate('%.0f~%.0fs' % (a, b), p, xytext=(6, 6), textcoords='offset points', fontsize=8)
    ax.plot([], [], 'x', color='k', label='1초 넘게 멈춤')
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    ax.set_xlim(-T.WALL_LX / 2 - 0.2, T.WALL_LX / 2 + 0.2)
    ax.set_ylim(-T.WALL_LY / 2 - 0.2, T.WALL_LY / 2 + 0.2)
    ax.grid(alpha=0.2)
    ax.set_title('%s — 지나간 경로 (회색 = 지도 장애물, 트랙 좌표)' % name, fontsize=14)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    main()
