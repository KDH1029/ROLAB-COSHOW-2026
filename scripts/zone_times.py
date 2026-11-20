#!/usr/bin/env python3
"""Time spent in each zone (zigzag, cones) per run, from the runner logs — to compare speed tests.

    python3 scripts/zone_times.py [N=6 | log files...]        (alias dzones) — the last N run logs
Per run: route, v, zigzag_v / cones_v if given, each zone's seconds and speed set, lap time.
"""
import glob
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
LOG = os.path.join(ROOT, 'logs', 'day1')


def one(f):
    t = open(f, errors='replace').read()
    ev = [(float(a), b) for a, b in re.findall(r'\[(\d+\.\d+)\] \[day1_runner\]: (.*)', t)]
    route = re.search(r'path: pins (\S+)\.csv', t)
    v = re.search(r' · v ([\d.]+) ', t)
    ov = re.findall(r'★ zone (\w+): v ([\d.]+) for this run', t)
    zones, cur = [], None
    for ts, msg in ev:
        m = re.match(r'★ zone (\w+): v ([\d.]+)', msg)
        if m and 'for this run' not in msg:
            if cur:
                zones.append((cur[0], ts - cur[2], cur[1]))
            cur = (m.group(1), m.group(2), ts)
        elif msg.startswith('zone ') and ' left' in msg and cur:
            zones.append((cur[0], ts - cur[2], cur[1]))
            cur = None
    done = re.search(r'lap done: ([\d.]+) m along the lane in ([\d.]+) s', t)
    if not zones and not done:
        return None
    return '%s  %s v %s%s | %s | %s' % (
        os.path.basename(f), route.group(1) if route else '?', v.group(1) if v else '?',
        (' (' + ', '.join('%s_v %s' % o for o in ov) + ')') if ov else '',
        ' · '.join('%s %.1f s @%s' % z for z in zones) or 'no zone',
        'lap %s m in %s s' % done.groups() if done else 'not finished')


def main():
    args = sys.argv[1:]
    if args and not args[0].isdigit():
        files = args
    else:
        n = int(args[0]) if args else 6
        files = sorted(glob.glob(os.path.join(LOG, 'run_*.log')), key=os.path.getmtime)[-n:]
    for f in files:
        r = one(f)
        if r:
            print(r)


if __name__ == '__main__':
    main()
