#!/usr/bin/env bash
# One-shot setup before the runs: XT16 -> traffic light (camera + YOLO) -> hula hoop -> up (FAST-LIO, bridge, RPP),
# then a status check. Parts already running are kept. Robot standing still, internal PC on.      (alias dsetup)
# Then, per run: dalign -> dred / dyellow / din ...
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
D1=$ROOT/scripts/day1.sh
LOG=${DAY1_LOG:-$ROOT/logs/day1}
alive(){ [ -f "$LOG/$1.pid" ] && kill -0 "$(cat "$LOG/$1.pid")" 2>/dev/null; }
step(){ echo; echo "── $*"; }
warn=()

ping -c1 -W1 192.168.123.18 >/dev/null 2>&1 || { echo "★ internal PC 192.168.123.18 not reachable — robot power / LAN"; exit 1; }
ping -c1 -W1 192.168.123.20 >/dev/null 2>&1 || { echo "★ XT16 192.168.123.20 not reachable — power / cable"; exit 1; }

step "1/4 XT16"
"$D1" xt16 || { echo "★ XT16 not working — stopped here"; exit 1; }

step "2/4 traffic light (camera + YOLO)"
"$D1" light || warn+=("traffic light node")

step "3/4 hula hoop"
"$D1" hoop start || warn+=("hula-hoop node")

step "4/4 FAST-LIO · motor bridge · RPP (robot standing still)"
if alive fastlio && alive odom_tf && alive bridge && alive nav2; then
  echo "already up (FAST-LIO pid $(cat "$LOG/fastlio.pid")) — kept"
else
  "$D1" up || { echo "★ up failed. If it says 'already running': dkill, then dsetup again"; exit 1; }
fi

step "status"
S=$("$D1" status 2>/dev/null)
rate(){ echo "$S" | grep -A1 "^$1 " | grep -o 'average rate: [0-9.]*' | grep -o '[0-9.]*$'; }
for t in /lidar_points /Odometry /lowstate; do
  r=$(rate $t); printf '  %-16s %s\n' "$t" "${r:-★ none}"
  [ -n "$r" ] || warn+=("$t")
done
"$D1" hoop status 2>/dev/null | grep -m1 "카메라:" | sed 's/^/  /'
echo
if [ ${#warn[@]} -eq 0 ]; then
  echo "■ ready — per run: dalign, then dred / dyellow / din (add light_timeout=10 if the light is off)"
else
  echo "★ ready with problems: ${warn[*]} — fix those before a competition run"
fi
