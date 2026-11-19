#!/usr/bin/env python3
"""Cone zone for the cone-avoiding runner: a box in the track frame -> <route>.cones.json (day1.sh run ignores it).

    python3 scripts/route_cones.py <route> [<route> ...] box=x0,y0,x1,y1 [name=cones]     (day1_cones.sh zone ...)

<route> = a name (logs/day1/routes/<name>.csv, the repo's routes/<name>.csv — written next to every copy) or a csv.
The box is where the robot may go around cones: start it ~0.5 m before the first cone, end it where the route
leaves the cones, keep it off the walls. Several zones: run once per zone with different name= (same name
replaces). box=none removes the zone.
"""
import json
import os
import sys

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src/mission/track_day1'))
import track_path as TP   # noqa: E402


def copies(r):
    if r.endswith('.csv'):
        return [r]
    out = [os.path.join(d, r + '.csv') for d in (os.path.join(ROOT, 'logs/day1/routes'), os.path.join(ROOT, 'routes'))]
    return [f for f in out if os.path.exists(f)]


def main():
    routes = [a for a in sys.argv[1:] if '=' not in a]
    kv = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    if not routes or 'box' not in kv:
        sys.exit(__doc__)
    name = kv.get('name', 'cones')
    box = None
    if kv['box'] != 'none':
        b = [float(v) for v in kv['box'].split(',')]
        box = [round(min(b[0], b[2]), 3), round(min(b[1], b[3]), 3), round(max(b[0], b[2]), 3), round(max(b[1], b[3]), 3)]
    n = 0
    for r in routes:
        fs = copies(r)
        if not fs:
            print('★ no route %s' % r)
        for f in fs:
            zf = f[:-4] + '.cones.json'
            zones = [z for z in (json.load(open(zf)) if os.path.exists(zf) else []) if z.get('name') != name]
            if box is not None:
                zones.append(dict(name=name, box=box))
                D = np.genfromtxt(f, delimiter=',', names=True)
                lp = TP.PinLoop(np.column_stack([D['x_m'], D['y_m']]), s0=0.0)
                S = np.array([lp.pose_at(s)[:2] for s in np.arange(0, lp.length, 0.05)])
                inb = (S[:, 0] > box[0]) & (S[:, 0] < box[2]) & (S[:, 1] > box[1]) & (S[:, 1] < box[3])
                print('%s: zone %s %s, %.1f m of the route inside' % (zf, name, box, 0.05 * inb.sum()))
            else:
                print('%s: zone %s removed' % (zf, name))
            json.dump(zones, open(zf, 'w'), indent=1)
            n += 1
    print('%d file(s) written' % n)


if __name__ == '__main__':
    main()
