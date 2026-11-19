#!/usr/bin/env python3
"""Make two routes identical where they run together (the shared stretches become one common line).

    python3 scripts/route_merge.py <A.csv> <B.csv> [thr=0.25] [blend=0.6] [pin=0.6] [map=maps/coshow_1119] [png=out.png]

Shared = A's points within `thr` of B. There the common line is the midpoint of A and B; both routes get exactly the
same pins on it (`pin` m apart, default 0.6). Where they part, each eases from the common line onto its own over `blend` metres.
Both files are rewritten (old ones kept as .bak_HHMMSS.csv); pin 1 (the start) stays on the common line.
"""
import os
import shutil
import sys
import time

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import track_path as TP   # noqa: E402


def load(path):
    D = np.genfromtxt(path, delimiter=',', names=True)
    return int(D['lane'][0]), TP.PinLoop(np.column_stack([D['x_m'], D['y_m']]), s0=0.0)


def save(path, lane, P):
    shutil.copy2(path, path[:-4] + time.strftime('.bak_%H%M%S.csv'))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    nxt = np.roll(P, -1, 0)
    yaw = np.degrees(np.arctan2(nxt[:, 1] - P[:, 1], nxt[:, 0] - P[:, 0]))
    with open(path, 'w') as f:
        f.write('lane,station_m,x_m,y_m,yaw_deg\n')
        for a, (x, y), b in zip(s, P, yaw):
            f.write('%d,%.3f,%.4f,%.4f,%.1f\n' % (lane, a, x, y, b))


def smooth(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3 - 2 * u)


def main():
    pa, pb = sys.argv[1], sys.argv[2]
    kv = dict(a.split('=', 1) for a in sys.argv[3:])
    thr, blend, ds = float(kv.get('thr', 0.25)), float(kv.get('blend', 0.6)), 0.05
    pin = float(kv.get('pin', 0.6))
    lane_a, A = load(pa)
    lane_b, B = load(pb)
    s1 = np.arange(0.0, A.length, ds)
    PA = np.array([A.pose_at(s)[:2] for s in s1])
    proj = [TP.lateral_of(B, x, y) for x, y in PA]
    d = np.abs([p[0] for p in proj])
    s2 = np.array([p[2] for p in proj])
    PB = np.array([B.pose_at(s)[:2] for s in s2])
    shared = d < thr
    if shared.all() or not shared.any():
        sys.exit('routes are %s everywhere — nothing to merge' % ('together' if shared.all() else 'apart'))
    # the parted stretches on A's stations (wrap-around aware: start inside a shared stretch)
    k0 = int(np.argmax(shared))
    order = np.r_[k0:len(s1), 0:k0]
    runs, cur = [], None
    for k in order:
        if not shared[k] and cur is None:
            cur = [k, k]
        elif not shared[k]:
            cur[1] = k
        elif cur is not None:
            runs.append(tuple(cur))
            cur = None
    if cur is not None:
        runs.append(tuple(cur))
    runs = [r for r in runs if (r[1] - r[0]) % len(s1) * ds > 0.3]      # ignore specks
    C = 0.5 * (PA + PB)
    # weight of the own line: 1 inside a parted stretch, easing to 0 over `blend` around it
    w = np.zeros(len(s1))
    for a, b in runs:
        for k in range(len(s1)):
            da = (s1[a] - s1[k]) % A.length            # distance before the parting point
            db = (s1[k] - s1[b]) % A.length            # distance after the joining point
            inside = (s1[k] - s1[a]) % A.length <= (s1[b] - s1[a]) % A.length
            if inside:
                w[k] = 1.0
            else:
                w[k] = max(w[k], smooth(1 - min(da, db) / blend))
    newA = C + w[:, None] * (PA - C)
    # B: the same common pins where w == 0; its own line in its parted stretches; the same easing
    core = w == 0
    pinsA, pinsB = [], []
    step = int(round(pin / ds))
    k = 0
    while k < len(s1):
        if core[k]:
            j = k
            while j < len(s1) and core[j]:
                j += 1
            idx = list(range(k, j, step))
            pinsA += [newA[i] for i in idx]
            pinsB += [C[i] for i in idx]
            k = j
        else:
            j = k
            while j < len(s1) and not core[j]:
                j += 1
            pinsA += [newA[i] for i in range(k, j, step)]
            # B over the same stretch: from its point at s2[k] to s2[j-1], along B, eased to C at both ends
            sb0, sb1 = s2[k], s2[j - 1] if j - 1 < len(s1) else s2[-1]
            span = (sb1 - sb0) % B.length
            nb = max(2, int(round(span / pin)))
            for t in np.linspace(0.0, 1.0, nb, endpoint=False):
                sb = sb0 + t * span
                q = np.array(B.pose_at(sb)[:2])
                # easing weight: the same as A's at the matching place along the stretch
                kk = k + int(round(t * (j - 1 - k)))
                pinsB.append(C[kk] + w[kk] * (q - C[kk]))
            k = j
    pinsA, pinsB = np.array(pinsA), np.array(pinsB)
    # report
    from route_edit import clearance, load_map
    mapd = kv.get('map', os.path.join(ROOT, 'maps/coshow_1119'))
    _, _, cells = load_map(mapd)

    def stats(P):
        lp = TP.PinLoop(P, s0=0.0)
        S = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length, 0.05)])
        return lp, S, clearance(S[::2], cells)
    la0, SA0, ca0 = stats(np.array([A.pose_at(s)[:2] for s in np.arange(0, A.length, 0.25)]))
    lb0, SB0, cb0 = stats(np.array([B.pose_at(s)[:2] for s in np.arange(0, B.length, 0.25)]))
    la, SA, ca = stats(pinsA)
    lb, SB, cb = stats(pinsB)
    same = int(core.sum())
    shift = np.hypot(*(C - PA).T)[core]
    print('shared %.1f m of %.1f (A), parted %s; common line = midpoint, moved A by max %.2f m (median %.2f)'
          % (same * ds, A.length, ', '.join('s %.1f-%.1f' % (s1[a], s1[b]) for a, b in runs), shift.max(), np.median(shift)))
    print('A: %.2f -> %.2f m, %d pins, obstacle room min %.2f -> %.2f m' % (A.length, la.length, len(pinsA), ca0.min(), ca.min()))
    print('B: %.2f -> %.2f m, %d pins, obstacle room min %.2f -> %.2f m' % (B.length, lb.length, len(pinsB), cb0.min(), cb.min()))
    # check: on the shared stretch both new routes lie on one line
    dd = np.array([abs(TP.lateral_of(lb, x, y)[0]) for x, y in SA[::4]])
    print('A to B on the shared stretch after the merge: max %.3f m' % dd[dd < 0.1].max())
    save(pa, lane_a, pinsA)
    save(pb, lane_b, pinsB)
    print('saved %s, %s (old .bak kept)' % (pa, pb))
    if 'png' in kv:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1/tools'))
        import plot_plan
        plot_plan._font()
        z = np.load(os.path.join(mapd, 'map_clean.npz'))
        M, hz = z['xyz'], z['height']
        fig, ax = plt.subplots(figsize=(13, 10))
        m = (hz > 0.05) & (hz < 0.6)
        ax.scatter(M[m, 0], M[m, 1], c='#9aa3ad', s=0.4)
        ax.plot(SA0[:, 0], SA0[:, 1], color='#1f6fb2', lw=0.8, ls=':', label='%s 전' % os.path.basename(pa)[:-4])
        ax.plot(SB0[:, 0], SB0[:, 1], color='#c0392b', lw=0.8, ls=':', label='%s 전' % os.path.basename(pb)[:-4])
        ax.plot(SA[:, 0], SA[:, 1], color='#1f6fb2', lw=2.2, label='%s 후' % os.path.basename(pa)[:-4])
        ax.plot(SB[:, 0], SB[:, 1], color='#c0392b', lw=2.2, label='%s 후' % os.path.basename(pb)[:-4])
        cc = C[core]
        ax.plot(cc[:, 0], cc[:, 1], '.', color='#8e44ad', ms=3, label='공통 구간')
        ax.plot(*pinsA[0], 'o', ms=14, mfc='#27ae60', mec='white', mew=2, label='출발 (공통)')
        ax.set_aspect('equal')
        ax.legend(loc='upper right')
        ax.grid(alpha=0.2)
        ax.set_title('두 경로 겹치는 구간을 한 선으로 (점선 = 전, 실선 = 후)', fontsize=14)
        fig.tight_layout()
        fig.savefig(kv['png'], dpi=110)
        print('picture %s' % kv['png'])


if __name__ == '__main__':
    main()
