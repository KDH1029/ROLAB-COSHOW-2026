#!/usr/bin/env bash
# Everything that runs today, on the fake robot: 6 runs in parallel, each on its own ROS domain (loopback only,
# nothing reaches the robot), then a pass / fail table. Run after any change: day1.sh selftest  (~3 min)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
cd "$ROOT"
OUT=$ROOT/logs/selftest; rm -rf "$OUT"; mkdir -p "$OUT"
R=$ROOT/routes                               # the committed routes (not the working copies)
H="-p hoop_red_at:=0.15,1.519 -p hoop_yellow_at:=0.148,2.326"
RT="lane=1 off0=0.265 s0=0.81 yaw0=9 v=0.4 other=false"
RA="-p lookahead:=0.5 -p min_radius:=0.4 -p path_file:=$R/rec105326_lap1.csv"
run() { local n=$1 d=$2; shift 2
  ( env TEST_DOMAIN=$d DAY1_TEST_LOG=$OUT/$n FAKE_T_MAX=220 FAKE_TRAIL=$OUT/trail_$n.csv "$@" > $OUT/out_$n.txt 2>&1 ) & }
echo "selftest: 6 fake-robot runs in parallel (domains 84-89), ~3 min ..."
run track_in 84 timeout 200 tests/day1_ros_integration.sh lane=in v=0.5
run track_out 85 timeout 200 tests/day1_ros_integration.sh lane=out v=0.8
run hoop_in 86 RUNNER_ARGS="$RA -p hoop:=in" timeout 280 tests/day1_ros_integration.sh $RT
run hoop_out 87 RUNNER_ARGS="$RA -p hoop:=out" timeout 280 tests/day1_ros_integration.sh $RT
run hoop_red 88 FAKE_ARGS="$H" RUNNER_ARGS="$RA -p hoop:=red" timeout 280 tests/day1_ros_integration.sh $RT
run hoop_yellow_side 89 FAKE_ARGS="$H -p hoop_mode:=side" RUNNER_ARGS="$RA -p hoop:=yellow" timeout 280 tests/day1_ros_integration.sh $RT
wait
python3 - "$OUT" <<'PY'
import sys, os, re, json, numpy as np
out = sys.argv[1]
# expected arch crossing: inner hoop y 1.52, outer 2.33 (None = track run)
want = dict(track_in=None, track_out=None, hoop_in=1.52, hoop_out=2.33, hoop_red=1.52, hoop_yellow_side=2.33)
bad = 0
print('| run | result | lap | arch y | check |')
print('|---|---|---|---|---|')
for n, w in want.items():
    txt = open(os.path.join(out, 'out_%s.txt' % n), errors='replace').read()
    m = re.search(r'RESULT (\{.*\})', txt)
    r = json.loads(m.group(1)) if m else {}
    lap = None
    log = os.path.join(out, n, 'runner.log')
    if os.path.exists(log):
        mm = re.search(r'lap done: [0-9.]+ m along the lane in ([0-9.]+) s', open(log, errors='replace').read())
        lap = float(mm.group(1)) if mm else None
    y = None
    tf = os.path.join(out, 'trail_%s.csv' % n)
    if w is not None and os.path.exists(tf) and os.path.getsize(tf) > 30:
        T = np.genfromtxt(tf, delimiter=',', skip_header=1)
        k = (np.abs(T[:, 0] - 0.15) < 0.06) & (T[:, 1] > 0)
        y = float(np.median(T[k, 1])) if k.any() else None
    ok = r.get('runner') == 'DONE' and lap is not None
    if w is None:
        ok = ok and r.get('viol', 1) == 0
    else:
        ok = ok and y is not None and abs(y - w) < 0.15
    bad += not ok
    print('| %s | %s | %s | %s | %s |' % (n, r.get('runner', 'no result'), '%.1f s' % lap if lap else '-',
                                         '%.2f' % y if y is not None else '-', 'OK' if ok else '★ FAIL'))
print('\nALL OK' if not bad else '\n★ %d FAILED — logs in %s' % (bad, out))
sys.exit(1 if bad else 0)
PY
