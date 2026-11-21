#!/usr/bin/env python3
"""Table of day-1 runs from logs/day1/bench.csv (the runner appends one line per finished lap).
    python3 tools/bench_table.py [bench.csv] [since=MM-DD]
Grouped by gait, course and commanded speed: best lap, mean speed, lateral error past the start blend, body sway."""
import csv
import os
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 and '=' not in sys.argv[1] else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../../../../logs/day1/bench.csv')
kv = dict(a.split('=') for a in sys.argv[1:] if '=' in a)
if not os.path.exists(path):
    sys.exit('no %s yet — finish a run first (day1.sh run ...)' % path)
rows = [r for r in csv.DictReader(open(path)) if r['time'] >= kv.get('since', '')]
grp = defaultdict(list)
for r in rows:
    grp[(r['gait'], int(r['lane']), float(r['v']))].append(r)
print('| 걸음새 | 코스 | 명령 v | 회수 | 최단 한 바퀴 | 평균 속도 | 옆 오차 최대 / 평균 | 몸 흔들림 roll / pitch (표준편차) |')
print('|---|---|---|---|---|---|---|---|')
for (g, lane, v), rs in sorted(grp.items(), key=lambda k: (k[0][1], k[0][0], k[0][2])):
    best = min(rs, key=lambda r: float(r['lap_s']))
    f = lambda k: sum(float(r[k]) for r in rs) / len(rs)
    print('| %s | %s | %.2f | %d | %.1f s | %.2f m/s | %.3f / %.3f m | %.2f / %.2f° |' % (
        g, '인' if lane == 2 else '아웃', v, len(rs), float(best['lap_s']), f('mean_mps'),
        max(float(r['lat_max']) for r in rs), f('lat_mean'), f('roll_std'), f('pitch_std')))
