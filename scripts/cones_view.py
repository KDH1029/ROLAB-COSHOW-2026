#!/usr/bin/env python3
"""Picture of the cone zones: map, route(s), zone boxes; with a run log also what the robot saw and the path it took.

    python3 scripts/cones_view.py <route> [<route> ...] [log=logs/day1/cones_HHMMSS.jsonl] [at=max|<t>] [out=png]
                                                                                       (day1_cones.sh show ...)
at=max (default) = the moment of the largest shift; at=<t> = the record nearest that time (s, as in the log).
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import cone_avoid as CA  # noqa: E402
import track_path as TP  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
import plot_plan  # noqa: E402


def route_file(r):
    if r.endswith('.csv'):
        return r
    for d in (os.path.join(ROOT, 'logs/day1/routes'), os.path.join(ROOT, 'routes')):
        f = os.path.join(d, r + '.csv')
        if os.path.exists(f):
            return f
    sys.exit('★ no route %s' % r)


def map_cells(d):
    y = {}
    for line in open(os.path.join(d, 'map.yaml')):
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1)
            y[k.strip()] = v.split('#')[0].strip()
    raw = open(os.path.join(d, y['image']), 'rb').read().split(b'\n', 3)
    w, h = map(int, raw[1].split())
    occ = np.frombuffer(raw[3], np.uint8).reshape(h, w)[::-1] == 0
    org = [float(v) for v in y['origin'].strip('[]').split(',')]
    res = float(y['resolution'])
    jj, ii = np.nonzero(occ)
    return np.column_stack([org[0] + (ii + 0.5) * res, org[1] + (jj + 0.5) * res])


def main():
    routes = [a for a in sys.argv[1:] if '=' not in a]
    kv = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    if not routes and 'log' in kv:          # the routes the run used (header lines of the log)
        for line in open(kv['log']):
            q = json.loads(line) if line.strip() else {}
            if 'route' in q and q['route'] not in routes:
                routes.append(q['route'])
    if not routes:
        sys.exit(__doc__)
    plot_plan._font()
    fig, ax = plt.subplots(figsize=(12, 9.5))
    M = map_cells(kv.get('map', os.path.join(ROOT, 'maps/coshow_1119')))
    ax.scatter(M[:, 0], M[:, 1], s=1, c='0.6', label='지도 (매핑 때)')
    zones = []
    for r, col in zip(routes, ['tab:blue', 'tab:red', 'tab:green']):
        f = route_file(r)
        D = np.genfromtxt(f, delimiter=',', names=True)
        lp = TP.PinLoop(np.column_stack([D['x_m'], D['y_m']]), s0=0.0)
        S = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length, 0.05)])
        ax.plot(S[:, 0], S[:, 1], '-', c=col, lw=1.5, label='%s (가르친 경로)' % os.path.basename(f)[:-4])
        zs, zf = CA.load_zones(f, os.path.join(ROOT, 'routes'))
        zones += [(z, zf) for z in zs if not any(z['box'] == q['box'] for q, _ in zones)]
    for z, zf in zones:
        b = z['box']
        ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], fill=False, ec='tab:orange', lw=2, ls='--'))
        ax.text(b[0], b[3] + 0.05, '콘 구간 %s [%.2f, %.2f, %.2f, %.2f]' % ((z.get('name', 'cones'),) + tuple(b)), color='tab:orange')
    title = '콘 구간 (%s)' % ', '.join(routes)
    if 'log' in kv:
        recs = [q for q in (json.loads(line) for line in open(kv['log']) if line.strip()) if 't' in q]
        if recs:
            at = kv.get('at', 'max')
            rec = (max(recs, key=lambda q: q['max_shift']) if at == 'max'
                   else min(recs, key=lambda q: abs(q['t'] - recs[0]['t'] - float(at))))
            R = np.array([q['robot'] for q in recs])
            ax.plot(R[:, 0], R[:, 1], ':', c='k', lw=1, label='로봇이 간 길 (기록)')
            O = np.array(rec['obs']).reshape(-1, 2)
            nw = np.zeros(len(O), bool)
            nw[[i for i in rec.get('new', []) if i < len(O)]] = True
            if (~nw).any():
                ax.scatter(O[~nw, 0], O[~nw, 1], s=8, c='tab:red', marker='s', label='그때 본 장애물 칸 (지도에 있음)')
            if nw.any():
                ax.scatter(O[nw, 0], O[nw, 1], s=22, c='magenta', marker='s', label='지도에 없던 칸 = 옮겨진 콘')
            Pth = np.array(rec['path'])
            ax.plot(Pth[:, 0], Pth[:, 1], '-', c='tab:purple', lw=2.5, label='그때 보낸 경로 (%s)' % rec['mode'])
            ax.plot(*rec['robot'][:2], 'ko', ms=8)
            modes = {}
            for q in recs:
                modes[q['mode']] = modes.get(q['mode'], 0) + 1
            title += ' · %s, %.1f s: %s, 최소 여유 %s m, 최대 비킴 %.2f m · 기록 %s' % (
                os.path.basename(kv['log']), rec['t'] - recs[0]['t'], rec['mode'], rec['min_clear'], rec['max_shift'],
                ', '.join('%s %d' % kv2 for kv2 in modes.items()))
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(title, fontsize=10)
    tag = os.path.basename(kv['log'])[:-6] if 'log' in kv else os.path.basename(routes[0]).replace('.csv', '')
    out = kv.get('out', os.path.join(ROOT, 'logs/day1', 'cones_%s.png' % tag))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=110, bbox_inches='tight')
    print(out)


if __name__ == '__main__':
    main()
