#!/usr/bin/env python3
"""After editing one of two routes that share stretches: copy the shared stretches to the other route.

    python3 scripts/route_sync.py <edited.csv> <other.csv>

Which stretches are shared is read from the edited route's previous version (newest .bak_*.csv, written by the editor
when it saved): the other route's pins lying on that old line (within 1 cm) are the shared ones, the rest is its own
branch. The other route becomes: the edited route up to where the branch leaves + its own branch + the edited route
from where the branch rejoins. Moves, added and deleted pins on the shared stretches all carry over; edits right at
the parting points are reported. The other route's old file is kept as .bak_HHMMSS.csv.
"""
import glob
import os
import shutil
import sys
import time

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import track_path as TP   # noqa: E402


def load(path):
    D = np.genfromtxt(path, delimiter=',', names=True)
    return int(D['lane'][0]), np.column_stack([D['x_m'], D['y_m']])


def save(path, lane, P):
    shutil.copy2(path, path[:-4] + time.strftime('.bak_%H%M%S.csv'))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    nxt = np.roll(P, -1, 0)
    yaw = np.degrees(np.arctan2(nxt[:, 1] - P[:, 1], nxt[:, 0] - P[:, 0]))
    with open(path, 'w') as f:
        f.write('lane,station_m,x_m,y_m,yaw_deg\n')
        for a, (x, y), b in zip(s, P, yaw):
            f.write('%d,%.3f,%.4f,%.4f,%.1f\n' % (lane, a, x, y, b))


def main():
    pe, po = sys.argv[1], sys.argv[2]
    baks = sorted(glob.glob(pe[:-4] + '.bak_*.csv'), key=os.path.getmtime)
    baks = [b for b in baks if os.path.getmtime(b) <= os.path.getmtime(pe)]
    if not baks:
        sys.exit('no previous version of %s (.bak) — nothing to compare with' % pe)
    old = baks[-1]
    _, E_old = load(old)
    _, E_new = load(pe)
    lane_o, O = load(po)
    lo = TP.PinLoop(E_old, s0=0.0)
    ln = TP.PinLoop(E_new, s0=0.0)
    on_old = np.array([abs(TP.lateral_of(lo, x, y)[0]) < 0.01 for x, y in O])
    if on_old.all():
        sys.exit('%s lies on %s everywhere — no own branch found' % (po, os.path.basename(old)))
    if not on_old.any():
        sys.exit('%s shares nothing with %s' % (po, os.path.basename(old)))
    # the own branch: the run of off-line pins (start pin is shared)
    n = len(O)
    k0 = int(np.argmax(~on_old))
    k1 = k0
    while k1 + 1 < n and not on_old[k1 + 1]:
        k1 += 1
    if (~on_old[k1 + 1:]).any():
        print('★ %s has more than one own stretch — only the first (pins %d-%d) is kept as its own' % (po, k0 + 1, k1 + 1))
    a, b = O[k0 - 1], O[(k1 + 1) % n]             # last shared pin before, first shared after
    sa_old, sb_old = lo.station(*a), lo.station(*b)
    pa, pb = np.array(lo.pose_at(sa_old)[:2]), np.array(lo.pose_at(sb_old)[:2])
    sa, sb = ln.station(*pa), ln.station(*pb)
    moved = [float(np.hypot(*(np.array(ln.pose_at(s)[:2]) - p))) for s, p in ((sa, pa), (sb, pb))]
    st = np.array([ln.station(x, y) for x, y in E_new])
    st[0] = 0.0
    before = E_new[st <= sa + 0.02]     # pins are >= 0.25 m apart: 2 cm keeps the parting / joining pin itself
    after = E_new[st >= sb - 0.02]
    if sb < sa:
        sys.exit('the parting point comes after the joining point on the edited route — edited across the branch?')
    P = np.vstack([before, O[k0:k1 + 1], after])
    save(po, lane_o, P)
    print('%s -> %s: shared pins copied %d (before %d + after %d), own branch kept %d pins (%d-%d of %d)'
          % (os.path.basename(pe), os.path.basename(po), len(before) + len(after), len(before), len(after),
             k1 - k0 + 1, k0 + 1, k1 + 1, n))
    print('  compared with %s; parting/joining points moved %.2f / %.2f m by the edit%s'
          % (os.path.basename(old), moved[0], moved[1],
             '  ★ edited near where the routes part — check the joint in the editor' if max(moved) > 0.05 else ''))


if __name__ == '__main__':
    main()
