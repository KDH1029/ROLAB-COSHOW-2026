#!/usr/bin/env python3
"""Cones moved, the rest of the course did not: replace only the cone stretch of the routes (and of the map).

    python3 scripts/cone_splice.py route <raw.csv> [route=rec105326_lap1] [other=rec105326_lap2] [pin=0.25] [pass=1]
    python3 scripts/cone_splice.py passes <raw.csv> <new map dir>     every pass through the cones: time, length,
                                                                      nearest map cell (the cones) — to choose pass=N
    python3 scripts/cone_splice.py map <new map dir> [map=maps/coshow_1119]
                                                                      (scripts/cone_splice.sh <bag> does both)
route: <raw.csv> = a drive with the remote from the start mark through the new cones until out of the cone box
  (several passes in one recording are fine: back to the start mark and through again; pass=N picks one).
  Old text: <raw.csv> = a short drive with the remote from the start mark through the new cones until out of the cone
  box (route_from_bag.py output, track frame). The routes' pins from the start up to where they leave the cone box
  (the 'cones' zone of <route>.zones.json) are replaced by that drive, resampled every `pin` m (light smoothing,
  +-1 pin); the rest stays. The old file is kept as .bak_HHMMSS.csv (route_edit / route_sync convention) and the
  shared stretch is copied to `other` with scripts/route_sync.py. New start = where the robot stood (the start mark).
map: inside the cone box (+0.3 m) the occupied cells of map.pgm and the points of map_clean.npz are replaced by the
  new map's (map_from_bag.sh on the same drive). Checked first: the new map's walls must sit on the old ones
  (median <= 6 cm outside the box). The old map is copied once to <map>_cones0.
"""
import glob
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


FIELD = [-0.8, -3.15, 3.75, -0.75]     # the cone field (11-19/20), fixed: passes and the stretch to replace use it


def field_passes(R, speed=None, r0=0.6):
    """[(start, last index in the field)] for every pass of a drive: leaves the place it first stood (the start mark,
    within r0) and goes out of the cone FIELD (any side). Independent of the route, so the numbering never changes."""
    at = np.hypot(R[:, 0] - R[0, 0], R[:, 1] - R[0, 1]) < r0
    out_f = ~in_box(R, FIELD)
    res, i = [], 0
    while i < len(R):
        st = np.nonzero(at[i:])[0]
        if not len(st):
            break
        a = i + int(st[0])
        ex = np.nonzero(out_f[a:])[0]
        if not len(ex):
            break
        e = a + int(ex[0])
        j = e - 1
        while j > i and not at[j]:
            j -= 1
        j0 = j
        while j0 > i and at[j0 - 1]:
            j0 -= 1
        run = np.arange(j0, j + 1)
        if speed is not None and (speed[run] < 0.05).any():
            a = int(run[speed[run] < 0.05].max())
        else:
            a = int(run[np.argmin(np.hypot(R[run, 0] - R[0, 0], R[run, 1] - R[0, 1]))])
        res.append((a, e - 1))
        i = e + 1
    return res


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


def passes(R, P, box, k, speed=None):
    """[(start index, last index inside the box)] for every pass: leaves the start mark (within 0.6 m of the route
    start) and goes out of the cone box near where the old route leaves it (P[k])."""
    at_start = np.hypot(R[:, 0] - P[0, 0], R[:, 1] - P[0, 1]) < 0.6
    near_exit = (~in_box(R, box)) & (np.hypot(R[:, 0] - P[k, 0], R[:, 1] - P[k, 1]) < 0.8)
    out, i = [], 0
    while i < len(R):
        st = np.nonzero(at_start[i:])[0]
        if not len(st):
            break
        a = i + int(st[0])
        ex = np.nonzero(near_exit[a:])[0]
        if not len(ex):
            break
        e = a + int(ex[0])
        j = e - 1                                             # the stay at the start mark right before this pass
        while j > i and not at_start[j]:
            j -= 1
        j0 = j
        while j0 > i and at_start[j0 - 1]:
            j0 -= 1
        run = np.arange(j0, j + 1)
        if speed is not None and (speed[run] < 0.05).any():
            a = int(run[speed[run] < 0.05].max())               # where it last stood still (the start mark it was put on)
        else:
            a = int(run[np.argmin(np.hypot(R[run, 0] - P[0, 0], R[run, 1] - P[0, 1]))])
        out.append((a, e - 1))
        i = e + 1
    return out


def do_route(raw, route, other, step, pass_no=1):
    D = np.genfromtxt(raw, delimiter=',', names=True)
    R = np.column_stack([D['x_m'], D['y_m']])
    box = FIELD
    f = os.path.join(RDIR, route + '.csv')
    lane, P = load_pins(f)
    # the old cone stretch: pin 0 (start) .. last pin before the route leaves the cone field
    k = 1
    while k < len(P) and in_box(P[k:k + 1], box)[0]:
        k += 1
    if k >= len(P) - 3:
        sys.exit('★ %s never leaves the cone field %s' % (route, box))
    ps = field_passes(R, D['speed_mps'] if 'speed_mps' in D.dtype.names else None)
    if pass_no > len(ps):
        sys.exit('★ pass %d not found (%d passes from the start mark out of the cone field)' % (pass_no, len(ps)))
    a0, last_in = ps[pass_no - 1]
    R = R[a0:]
    last_in -= a0
    d0 = float(np.hypot(*(R[0] - P[0])))
    if d0 > 1.0:
        sys.exit('★ the drive starts %.2f m from the route start — not the same start mark?' % d0)
    N = resample(R[:last_in + 1], step)                # the old pins take over from the first one outside the box
    # where to rejoin the route: a pin AHEAD of where the pass leaves the field (in its exit direction), near and well
    # aligned (score = distance x (2 - cos)^2); the plain 'first pin out of the field' can lie behind it (pass exits elsewhere)
    ex = N[-1] - N[max(0, len(N) - 4)]
    ex = ex / max(1e-9, float(np.hypot(*ex)))
    best = None
    for j in range(k, min(len(P), k + 8)):
        v = P[j] - N[-1]
        dist = float(np.hypot(*v))
        cos = float(np.dot(v, ex) / max(1e-9, dist))
        if dist < 0.15 or cos > 0.3:
            sc = dist * (2.0 - cos) ** 2
            if best is None or sc < best[0]:
                best = (sc, j)
    if best is not None and best[1] != k:
        print('rejoin at pin %d (%.2f, %.2f) instead of %d: ahead of the exit' % (best[1] + 1, P[best[1], 0], P[best[1], 1], k + 1))
        k = best[1]
    gap = float(np.hypot(*(N[-1] - P[k])))
    if gap > 1.6:
        sys.exit('★ the drive leaves the cone field %.2f m from where %s does — different exit?' % (gap, route))
    if gap > 0.6:
        print('★ the drive leaves the field %.2f m from where the route went on: the joint is a straight guess — '
              'look at it (dpasses / deditin) and move pins around any cone there' % gap)
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


def show_passes(raw, newd, route):
    D = np.genfromtxt(raw, delimiter=',', names=True)
    R = np.column_stack([D['x_m'], D['y_m']])
    box = cone_box(route)
    _, P = load_pins(os.path.join(RDIR, route + '.csv'))
    k = 1
    while k < len(P) and in_box(P[k:k + 1], box)[0]:
        k += 1
    img, _, res, org = grid(newd)
    C = cells(img, res, org)
    C = C[in_box(C, box, 0.0) & (np.abs(C[:, 0]) < 4.2) & (np.abs(C[:, 1]) < 3.2)]
    tree = cKDTree(C) if len(C) else None
    box = FIELD
    ps = field_passes(R, D['speed_mps'] if 'speed_mps' in D.dtype.names else None)
    print('%d pass(es) from the start mark out of the cone field:' % len(ps))
    for f in glob.glob(os.path.join(os.path.dirname(raw), 'pass_*.csv')):
        os.remove(f)
    for n, (a, e) in enumerate(ps, 1):          # for the editor: dpasses (route_edit.py drives=...)
        np.savetxt(os.path.join(os.path.dirname(raw), 'pass_%d.csv' % n), R[a:e + 1], delimiter=',', fmt='%.4f',
                   header='x_m,y_m', comments='')
    for n, (a, e) in enumerate(ps, 1):
        seg = R[a:e + 1]
        L = float(np.hypot(*np.diff(seg, axis=0).T).sum())
        dmin = float(tree.query(seg)[0].min()) if tree is not None else float('nan')
        print('  pass=%d: %.0f s, %.1f m, nearest cone (map cell) %.2f m%s'
              % (n, D['t_s'][e] - D['t_s'][a], L, dmin, '  ★ close' if dmin < 0.25 else ''))
    print('  the route uses pass=1 unless given: dsplice <bag> pass=N (the slow steps are not repeated)')
    # picture: every pass over the new map's cone box, numbered
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
    import plot_plan
    plot_plan._font()
    fig, ax = plt.subplots(figsize=(13, 8))
    Cn = cells(img, res, org)
    ax.scatter(Cn[:, 0], Cn[:, 1], s=4, c='0.45', marker='s', label='새 지도 (이번 녹화)')
    Lo = TP.PinLoop(P)
    S = np.array([Lo.pose_at(q)[:2] for q in np.arange(0, Lo.length, 0.05)])
    ax.plot(S[:, 0], S[:, 1], ':', c='0.5', lw=1, label='지금 경로 (어제 콘 기준)')
    cols = ['tab:blue', 'tab:red', 'tab:green', 'tab:purple', 'tab:orange', 'tab:brown']
    for n, (a, e) in enumerate(ps, 1):
        seg = R[a:e + 1]
        dmin = float(tree.query(seg)[0].min()) if tree is not None else float('nan')
        c = cols[(n - 1) % len(cols)]
        ax.plot(seg[:, 0], seg[:, 1], '-', c=c, lw=2, label='pass=%d  (콘과 최소 %.2f m, %.1f m)' % (n, dmin, float(np.hypot(*np.diff(seg, axis=0).T).sum())))
        m = len(seg) // 2
        ax.text(seg[m, 0], seg[m, 1] + 0.08, str(n), color=c, fontsize=14, weight='bold')
    ax.add_patch(plt.Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1], fill=False, ec='tab:orange', ls='--'))
    ax.set_xlim(box[0] - 0.4, box[2] + 0.6)
    ax.set_ylim(box[1] - 0.4, box[3] + 1.2)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc='upper left')
    ax.set_title('콘 밭 통과별 경로 — 하나를 골라 dsplice <bag> pass=N')
    out = os.path.join(ROOT, 'logs/day1/cone_passes_%s.png' % time.strftime('%H%M%S'))
    fig.savefig(out, dpi=105, bbox_inches='tight')
    print('  picture:', out)


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
    patch_3d(newd, mapd, box)


def patch_3d(newd, mapd, box):
    """The full 3D map too (map_track.npz/pcd, what RViz and map_build use): inside the cone box (+0.3 m) the new
    drive's points, then map_build --clean rebuilds map_clean.*, map.pgm and map_top.png from it, so every display
    shows the same cones."""
    ot, nt = np.load(os.path.join(mapd, 'map_track.npz')), np.load(os.path.join(newd, 'map_track.npz'))
    keep = ~in_box(ot['xyz'][:, :2], box, 0.3)
    take = in_box(nt['xyz'][:, :2], box, 0.3)
    out = {k: ot[k] for k in ot.files}
    for k in ('xyz', 'count', 'height'):
        out[k] = np.concatenate([ot[k][keep], nt[k][take]])
    np.savez(os.path.join(mapd, 'map_track.npz'), **out)
    sys.path.insert(0, os.path.join(ROOT, 'scripts'))
    import map_build
    map_build.write_pcd(os.path.join(mapd, 'map_track.pcd'), out['xyz'], out['count'])
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts/map_build.py'), '--clean', mapd], capture_output=True, text=True)
    print('map_track: cone box points %d old -> %d new; map_build --clean: %s'
          % (int((~keep).sum()), int(take.sum()), (r.stdout + r.stderr).strip().splitlines()[-1:] or ''))


def main():
    a = sys.argv[1:]
    kv = dict(x.split('=', 1) for x in a if '=' in x)
    pos = [x for x in a if '=' not in x]
    route = kv.get('route', 'rec105326_lap1')
    if pos[:1] == ['route'] and len(pos) == 2:
        do_route(pos[1], route, kv.get('other', 'rec105326_lap2'), float(kv.get('pin', 0.25)), int(kv.get('pass', 1)))
    elif pos[:1] == ['passes'] and len(pos) == 3:
        show_passes(pos[1], pos[2], route)
    elif pos[:1] == ['map'] and len(pos) == 2:
        do_map(pos[1], os.path.join(ROOT, kv.get('map', 'maps/coshow_1119')), route)
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main()
