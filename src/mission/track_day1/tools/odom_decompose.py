#!/usr/bin/env python3
"""Why the built-in odometry walks away: split its error into forward scale, sideways slip and yaw.

Reference = FAST-LIO /Odometry of the same recording (tests/fastlio_replay.sh, ~2 cm against the wall poses).
Both trajectories -> body-frame velocity (forward, left) and yaw rate over `win` s, time-aligned by
cross-correlation of the speed. Then:
  forward:  odom forward speed vs reference (slope = 1 / scale)
  sideways: reference left speed that the odometry does not report (slip, the robot drifting left/right)
  yaw:      odom yaw rate vs reference (gyro scale) and heading difference over the run
    python3 tools/odom_decompose.py <bag dir> <lio.csv> [win=0.5]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_vs_wall import read_odom   # noqa: E402


def body_vel(t, x, y, yaw, tq, win):
    """Body-frame velocity and yaw rate at times tq (central difference over win)."""
    yu = np.unwrap(yaw)
    def at(tt):
        return np.interp(tt, t, x), np.interp(tt, t, y), np.interp(tt, t, yu)
    xa, ya, aa = at(tq - win / 2)
    xb, yb, ab = at(tq + win / 2)
    am = 0.5 * (aa + ab)
    vx, vy = (xb - xa) / win, (yb - ya) / win
    c, s = np.cos(am), np.sin(am)
    return c * vx + s * vy, -s * vx + c * vy, (ab - aa) / win


def main():
    bag, lio = sys.argv[1].rstrip('/'), sys.argv[2]
    kv = dict(a.split('=') for a in sys.argv[3:])
    win = float(kv.get('win', 0.5))
    O = read_odom(bag)
    D = np.genfromtxt(lio, delimiter=',', names=True)
    L = np.column_stack([D['t'], D['x'], D['y'], D['yaw']])
    L = L[np.concatenate([[True], np.diff(L[:, 0]) > 1e-6])]
    t0, t1 = max(O[0, 0], L[0, 0]) + 1.0, min(O[-1, 0], L[-1, 0]) - 1.0
    tq = np.arange(t0, t1, 0.05)
    lf, ll, lw = body_vel(*L.T, tq, win)
    # time offset: odom speed vs LIO speed
    best = None
    for dt in np.arange(-0.5, 0.501, 0.01):
        of, ol, ow = body_vel(*O.T, tq + dt, win)
        c = np.corrcoef(np.hypot(of, ol), np.hypot(lf, ll))[0, 1]
        if best is None or c > best[0]:
            best = (c, dt)
    dt = best[1]
    of, ol, ow = body_vel(*O.T, tq + dt, win)
    mv = np.hypot(lf, ll) > 0.15                       # moving
    st = mv & (np.abs(lw) < 0.15)                      # moving straight
    cu = mv & (np.abs(lw) > 0.25)                      # curving
    name = os.path.basename(bag)
    print('%s: odom time offset %+.2f s (corr %.3f), moving %.0f s (straight %.0f / curve %.0f)'
          % (name, dt, best[0], mv.sum() * 0.05, st.sum() * 0.05, cu.sum() * 0.05))
    for lab, m in (('all', mv), ('straight', st), ('curve', cu)):
        if m.sum() < 20:
            continue
        k = np.sum(lf[m] * of[m]) / np.sum(of[m] ** 2)          # lio_fwd = k * odom_fwd
        kw = np.sum(lw[m] * ow[m]) / max(1e-9, np.sum(ow[m] ** 2))
        print('  %-8s fwd: LIO %.3f m/s, odom %.3f m/s -> scale %.3f | left: LIO %+.3f, odom %+.3f m/s (slip %+.3f) | yaw rate: LIO %+.3f, odom %+.3f rad/s (ratio %.3f)'
              % (lab, lf[m].mean(), of[m].mean(), k, ll[m].mean(), ol[m].mean(), (ll[m] - ol[m]).mean(),
                 lw[m].mean(), ow[m].mean(), kw if lab != 'straight' else float('nan')))
    # what each part costs over the run (integrated in the LIO frame, metres)
    a = np.interp(tq, L[:, 0], np.unwrap(L[:, 3]))
    c, s = np.cos(a), np.sin(a)
    k = np.sum(lf[mv] * of[mv]) / np.sum(of[mv] ** 2)
    slip_xy = np.cumsum(np.column_stack([-s * (ll - ol), c * (ll - ol)]) * 0.05, axis=0)
    scale_xy = np.cumsum(np.column_stack([c * (lf - k * of), s * (lf - k * of)]) * 0.05, axis=0)
    yaw_drift = np.degrees((np.interp(tq[-1] + dt, O[:, 0], np.unwrap(O[:, 3])) - np.interp(tq[0] + dt, O[:, 0], np.unwrap(O[:, 3])))
                           - (a[-1] - a[0]))
    print('  over the run: sideways slip unseen by odom %.2f m (max), forward error left after scale %.3f: %.2f m (max), heading drift %+.1f deg'
          % (np.hypot(*slip_xy.T).max(), k, np.hypot(*scale_xy.T).max(), yaw_drift))


if __name__ == '__main__':
    main()
