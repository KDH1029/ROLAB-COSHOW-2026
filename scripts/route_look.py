#!/usr/bin/env python3
"""Look point for the hula hoops: written into each route's <route>.zones.json (type 'look').

    python3 scripts/route_look.py <route1.csv> <route2.csv> at=x,y [arch_x=0.15] [yaw=deg] [timeout=5] [default=0]

The robot (run with hoop=red|yellow) stops at `at`, turns to `yaw` (default: towards the middle of the two hoops),
asks the hoop vision once, and goes on along the route whose hoop has the wanted colour. Each route's hoop = where it
crosses the arch line x = arch_x. `at` must lie where the routes are still the same (before they part).
"""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import track_path as TP   # noqa: E402


def main():
    paths = [a for a in sys.argv[1:] if '=' not in a]
    kv = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    at = [float(v) for v in kv['at'].split(',')]
    ax = float(kv.get('arch_x', 0.15))
    hoops, loops = {}, {}
    for p in paths:
        D = np.genfromtxt(p, delimiter=',', names=True)
        lp = TP.PinLoop(np.column_stack([D['x_m'], D['y_m']]), s0=0.0)
        S = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length, 0.02)])
        k = int(np.argmin(np.abs(S[:, 0] - ax) + 10 * (S[:, 1] < 0)))      # the crossing on the upper side
        name = os.path.splitext(os.path.basename(p))[0]
        hoops[name] = [round(float(S[k, 0]), 3), round(float(S[k, 1]), 3)]
        loops[name] = lp
    names = list(hoops)
    d_at = [abs(TP.lateral_of(loops[n], at[0], at[1])[0]) for n in names]
    if max(d_at) > 0.10:
        print('★ the look point is %.2f m off a route — put it where the routes are still the same' % max(d_at))
    mid = np.mean([hoops[n] for n in names], axis=0)
    yaw = float(kv['yaw']) if 'yaw' in kv else math.degrees(math.atan2(mid[1] - at[1], mid[0] - at[0]))
    zone = dict(name='hoop', type='look', at=at, yaw_deg=round(yaw, 1), branches=hoops,
                timeout_s=float(kv.get('timeout', 5)), default=int(kv.get('default', 0)))
    for p in paths:
        zf = p[:-4] + '.zones.json'
        zones = json.load(open(zf)) if os.path.exists(zf) else []
        zones = [z for z in zones if z.get('type') != 'look'] + [zone]
        json.dump(zones, open(zf, 'w'), indent=1)
    sep = ['%s %.0f deg %.1f m' % (n, math.degrees(math.atan2(hoops[n][1] - at[1], hoops[n][0] - at[0])),
                                   math.hypot(hoops[n][0] - at[0], hoops[n][1] - at[1])) for n in names]
    print('look point (%.2f, %.2f) facing %.0f deg; hoops: %s; written to %d zone files' % (at[0], at[1], yaw, '; '.join(sep), len(paths)))


if __name__ == '__main__':
    main()
