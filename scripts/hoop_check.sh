#!/usr/bin/env bash
# Hula-hoop vision on the real camera, robot does not move: hoop_check.sh [seconds=20]   (alias d1hc)
#   hoop_check.sh engine   the internal PC's FP16 engine vs the labelled pictures (FP32 on the NUC: rh 98-100 %, yh 100 %)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
ROBOT=${ROBOT:-unitree@192.168.123.18}
SSH=(ssh -o "ControlPath=$HOME/.ssh/go2_cm_%r@%h:%p" -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6)
if [ "${1:-}" = engine ]; then
  T=/tmp/hoop_engine_check; D=$ROOT/logs/hoop_train/data; L=$(mktemp -d)
  mkdir -p $L/img $L/lab
  for lb in $(grep -l . $D/labels/*/*.txt); do      # pictures that have a card (labels 0 rh, 1 yh)
    cp "$lb" $L/lab/; im=${lb/\/labels\//\/images\/}; cp "${im%.txt}.jpg" $L/img/ 2>/dev/null
  done
  cp "$ROOT/src/vision/start_light/hoop_engine_check.py" $L/
  echo "pictures: $(ls $L/img | wc -l) -> internal PC $T"
  "${SSH[@]}" $ROBOT "rm -rf $T" || exit 1
  scp -q -r -o "ControlPath=$HOME/.ssh/go2_cm_%r@%h:%p" $L $ROBOT:$T || exit 1
  rm -rf $L
  "${SSH[@]}" $ROBOT "cd ~/junsu_vision && nice -n 19 python3 $T/hoop_engine_check.py $T \$(cat hoop_classes.txt)"
  exit $?
fi
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
python3 "$ROOT/scripts/hoop_check.py" "$@"
