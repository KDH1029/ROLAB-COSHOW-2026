#!/usr/bin/env python3
"""Cones moved, the rest of the course did not: replace only the cone stretch of the routes (and of the map).

    python3 scripts/cone_splice.py route <raw.csv> [route=rec105326_lap1] [other=rec105326_lap2] [pin=0.25]
    python3 scripts/cone_splice.py map <new map dir> [map=maps/coshow_1119]
                                                                      (scripts/cone_splice.sh <bag> does both)
route: <raw.csv> = a short drive with the remote from the start mark through the new cones until out of the cone
  box (route_from_bag.py output, track frame). The routes' pins from the start up to where they leave the cone box
  (the 'cones' zone of <route>.zones.json) are replaced by that drive, resampled every `pin` m (light smoothing,
  +-1 pin); the rest stays. The old file is kept as .bak_HHMMSS.csv (route_edit / route_sync convention) and the
  shared stretch is copied to `other` with scripts/route_sync.py. New start = where the robot stood (the start mark).
map: inside the cone box (+0.3 m) the occupied cells of map.pgm and the points of map_clean.npz are replaced by the
  new map's (map_from_bag.sh on the same drive). Checked first: the new map's walls must sit on the old ones
  (median <= 6 cm outside the box). The old map is copied once to <map>_cones0.
"""
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import track_path as TP  # noqa: E402

RDIR = os.environ.get('SPLICE_ROUTES', os.path.join(ROOT, 'logs/day1/routes'))   # env: a scratch copy for a dry run


def in_box(P, b, m=0.0):
    return (P[:, 0] > b[0] - m) & (P[:, 0] < b[2] + m) & (P[:, 1] > b[1] - m) & (P[:, 1] < b[3] + m)


def cone_box(route):
    for z in json.load(open(os.path.join(RDIR, route + '.zones.json'))):
        if z.get('name') == 'cones' and 'box' in z:
            return z['box']
    sys.exit('★ no "cones" zone in %s.zones.json' % route)


def load_pins(f):
    D = np.genfromtxt(f, delimiter=',', names=True)
    return int(D['lane'][0]), np.column_stack([D['x_m'], D['y_m']])


def save_pins(f, lane, P):
    shutil.copy2(f, f[:-4] + time.strftime('.bak_%H%M%S.csv'))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    nxt = np.roll(P, -1, 0)
    yaw = np.degrees(np.arctan2(nxt[:, 1] - P[:, 1], nxt[:, 0] - P[:, 0]))
    with open(f, 'w') as o:
        o.write('lane,station_m,x_m,y_m,yaw_deg\n')
        for a, (x, y), b in zip(s, P, yaw):
            o.write('%d,%.3f,%.4f,%.4f,%.1f\n' % (lane, a, x, y, b))


def resample(R, step):
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(R, axis=0).T))])
    q = np.arange(0.0, s[-1], step)
    Q = np.column_stack([np.interp(q, s, R[:, 0]), np.interp(q, s, R[:, 1])])
    if len(Q) > 4:                       # +-1 pin: keeps the weave between cones, removes the remote's wobble
        Q[1:-1] = (Q[:-2] + Q[1:-1] + Q[2:]) / 3.0
    return Q


def do_route(raw, route, other, step):
    D = np.genfromtxt(raw, delimiter=',', names=True)
    R = np.column_stack([D['x_m'], D['y_m']])
    box = cone_box(route)
    f = os.path.join(RDIR, route + '.csv')
    lane, P = load_pins(f)
    # the old cone stretch: pin 0 (start) .. last pin before the route leaves the box
    k = 1
    while k < len(P) and in_box(P[k:k + 1], box)[0]:
        k += 1
    if k >= len(P) - 3:
        sys.exit('★ %s never leaves the cone box %s' % (route, box))
    # the new drive: from where it stood (start mark) until it leaves the box for good, plus the first point outside
    if not in_box(R[:1], box)[0]:
        sys.exit('★ the drive does not start in the cone box %s (start %.2f, %.2f) — wrong recording?' % (box, *R[0]))
    # where it leaves: the first time it is out of the box near where the old route leaves it (P[k]) — brushing the
    # box edge elsewhere does not count, and a whole lap (back to the start, inside the box again) works too
    inside = in_box(R, box)
    near_exit = (~inside) & (np.hypot(R[:, 0] - P[k, 0], R[:, 1] - P[k, 1]) < 0.8)
    if not near_exit.any():
        sys.exit('★ the drive never leaves the cone box near where %s does (%.2f, %.2f) — drive on until past the cones'
                 % (route, P[k, 0], P[k, 1]))
    last_in = int(np.argmax(near_exit)) - 1
    d0 = float(np.hypot(*(R[0] - P[0])))
    if d0 > 1.0:
        sys.exit('★ the drive starts %.2f m from the route start — not the same start mark?' % d0)
    N = resample(R[:last_in + 1], step)                # the old pins take over from the first one outside the box
    gap = float(np.hypot(*(N[-1] - P[k])))
    if gap > 1.2:
        sys.exit('★ the drive leaves the box %.2f m from where %s does — different exit?' % (gap, route))
    NEW = np.vstack([N, P[k:]])
    old_len, new_len = TP.PinLoop(P).length, TP.PinLoop(NEW).length
    save_pins(f, lane, NEW)
    print('%s: cone stretch pins 1-%d (%d pins) -> %d new pins from the drive; start %.2f m from the old start; '
          'joint %.2f m; lap %.2f -> %.2f m' % (route, k, k, len(N), d0, gap, old_len, new_len))
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts/route_sync.py'), f,
                        os.path.join(RDIR, other + '.csv')], capture_output=True, text=True)
    print((r.stdout + r.stderr).strip())
    picture(route, other, P, R[:last_in + 2], box)


def picture(route, other, P_old, R, box):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
    import plot_plan
    plot_plan._font()
    sys.path.insert(0, os.path.join(ROOT, 'scripts'))
    import cones_view as V
    fig, ax = plt.subplots(figsize=(12, 9.5))
    M = V.map_cells(os.path.join(ROOT, 'maps/coshow_1119'))
    ax.scatter(M[:, 0], M[:, 1], s=1, c='0.6', label='지도')
    Lo = TP.PinLoop(P_old)
    S = np.array([Lo.pose_at(s)[:2] for s in np.arange(0, Lo.length, 0.05)])
    ax.plot(S[:, 0], S[:, 1], ':', c='0.3', lw=1, label='%s 전' % route)
    for r, c in ((route, 'tab:blue'), (other, 'tab:red')):
        _, Q = load_pins(os.path.join(RDIR, r + '.csv'))
        L = TP.PinLoop(Q)
        S = np.array([L.pose_at(s)[:2] for s in np.arange(0, L.length, 0.05)])
        ax.plot(S[:, 0], S[:, 1], '-', c=c, lw=1.5, label='%s 후' % r)
    ax.plot(R[:, 0], R[:, 1], '-', c='tab:green', lw=0.8, alpha=0.7, label='리모컨 주행 (녹화)')
    ax.add_patch(plt.Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1], fill=False, ec='tab:orange', ls='--'))
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='upper right')
    ax.set_title('콘 구간 바꿔 끼우기 (%s, %s)' % (route, other))
    out = os.path.join(ROOT, 'logs/day1/cone_splice_%s.png' % time.strftime('%H%M%S'))
    fig.savefig(out, dpi=105, bbox_inches='tight')
    print('picture:', out)


def grid(d):
    y = {}
    for line in open(os.path.join(d, 'map.yaml')):
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1)
            y[k.strip()] = v.split('#')[0].strip()
    raw = open(os.path.join(d, y['image']), 'rb').read().split(b'\n', 3)
    w, h = map(int, raw[1].split())
    img = np.frombuffer(raw[3], np.uint8)[:w * h].reshape(h, w).copy()
    org = [float(v) for v in y['origin'].strip('[]').split(',')]
    return img, raw, float(y['resolution']), org


def cells(img, res, org):
    jj, ii = np.nonzero(img[::-1] == 0)
    return np.column_stack([org[0] + (ii + 0.5) * res, org[1] + (jj + 0.5) * res])


def do_map(newd, mapd, route):
    box = cone_box(route)
    oimg, oraw, ores, oorg = grid(mapd)
    nimg, _, nres, norg = grid(newd)
    O, N = cells(oimg, ores, oorg), cells(nimg, nres, norg)
    # the new map must sit on the old one where nothing changed (walls, zigzag, arch posts)
    No = N[~in_box(N, box, 0.5)]
    if len(No) < 50:
        sys.exit('★ the new map has only %d cells outside the cone box — too little to check the alignment' % len(No))
    d = cKDTree(O).query(No)[0]
    med = float(np.median(d))
    print('new map outside the cone box: %d cells, distance to the old map median %.3f m, p90 %.3f m'
          % (len(No), med, float(np.percentile(d, 90))))
    if med > 0.06:
        sys.exit('★ the new map does not sit on the old one (median %.2f m) — not patched' % med)
    bak = mapd.rstrip('/') + '_cones0'
    if not os.path.exists(bak):
        shutil.copytree(mapd, bak)
        print('old map kept:', bak)
    # 2D grid: inside the box (+0.3) -> free, then the new occupied cells
    h, w = oimg.shape
    jj, ii = np.mgrid[0:h, 0:w]
    X = oorg[0] + (ii + 0.5) * ores
    Y = oorg[1] + ((h - 1 - jj) + 0.5) * ores
    inb = (X > box[0] - 0.3) & (X < box[2] + 0.3) & (Y > box[1] - 0.3) & (Y < box[3] + 0.3)
    nin = in_box(N, box, 0.3)
    n_old = int(((oimg == 0) & inb).sum())
    oimg[inb & (oimg == 0)] = 254
    for x, y in N[nin]:
        i = int((x - oorg[0]) / ores)
        j = h - 1 - int((y - oorg[1]) / ores)
        if 0 <= i < w and 0 <= j < h:
            oimg[j, i] = 0
    with open(os.path.join(mapd, 'map.pgm'), 'wb') as f:
        f.write(b'\n'.join(oraw[:3]) + b'\n' + oimg.tobytes())
    print('map.pgm: cone box cells %d old -> %d new' % (n_old, int(nin.sum())))
    # 3D points for the editor
    oz, nz = np.load(os.path.join(mapd, 'map_clean.npz')), np.load(os.path.join(newd, 'map_clean.npz'))
    keep = ~in_box(oz['xyz'][:, :2], box, 0.3)
    take = in_box(nz['xyz'][:, :2], box, 0.3)
    out = {k: oz[k] for k in oz.files}
    for k in ('xyz', 'height', 'count'):
        out[k] = np.concatenate([oz[k][keep], nz[k][take]])
    np.savez(os.path.join(mapd, 'map_clean.npz'), **out)
    print('map_clean.npz: cone box points %d old -> %d new' % (int((~keep).sum()), int(take.sum())))


def main():
    a = sys.argv[1:]
    kv = dict(x.split('=', 1) for x in a if '=' in x)
    pos = [x for x in a if '=' not in x]
    route = kv.get('route', 'rec105326_lap1')
    if pos[:1] == ['route'] and len(pos) == 2:
        do_route(pos[1], route, kv.get('other', 'rec105326_lap2'), float(kv.get('pin', 0.25)))
    elif pos[:1] == ['map'] and len(pos) == 2:
        do_map(pos[1], os.path.join(ROOT, kv.get('map', 'maps/coshow_1119')), route)
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main()
