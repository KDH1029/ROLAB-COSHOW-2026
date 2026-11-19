#!/usr/bin/env bash
# Put the NUC's hula-hoop model (src/vision/start_light/hoop_384x640.onnx, set by `day1.sh hoop use`) on the internal
# PC and make the node use it. ★ Robot LYING DOWN (the engine build loads the robot's own PC, ~10 min). Run once.
#   hoop_update.sh            back up the old engine -> push -> stop node -> build FP16 engine -> start node (alias dhoopup)
#   hoop_update.sh rollback   put the backed-up engine back and restart the node (old model, ~10 s, no build)
# (`day1.sh hoop deploy` leaves an already running node on the OLD engine: it does not restart it.)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
ROBOT=${ROBOT:-unitree@192.168.123.18}
SSH=(ssh -o "ControlPath=$HOME/.ssh/go2_cm_%r@%h:%p" -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6)
D1=$ROOT/scripts/day1.sh
E='~/junsu_vision/hoop_384x640_fp16.engine'
ping -c1 -W1 192.168.123.18 >/dev/null 2>&1 || { echo "★ internal PC (192.168.123.18) not reachable — robot power / LAN"; exit 1; }
if [ "${1:-}" = rollback ]; then
  "${SSH[@]}" $ROBOT "ls -l $E.prev" || { echo "★ no backup engine on the internal PC"; exit 1; }
  "$D1" hoop stop; sleep 1
  "${SSH[@]}" $ROBOT "cp $E.prev $E" && "$D1" hoop start && echo "■ back on the previous hula-hoop engine"
  exit $?
fi
"${SSH[@]}" $ROBOT "pgrep -f '[t]rt_build.py'" >/dev/null && { echo "★ an engine build is already running on the internal PC — wait for it"; exit 1; }
"${SSH[@]}" $ROBOT "[ -f $E ] && cp $E $E.prev && echo 'old engine kept: $E.prev'"
"$D1" hoop push || exit 1
"$D1" hoop stop; sleep 1
"$D1" hoop build || { echo "★ build failed — the old engine is in $E.prev (hoop_update.sh rollback)"; exit 1; }
"$D1" hoop start && echo "■ hula-hoop node on the new engine ($(md5sum $ROOT/src/vision/start_light/hoop_384x640.onnx | cut -c1-8))"
