#!/usr/bin/env bash
# YOLO test: the internal PC's camera with every YOLO box drawn, live in a window on the NUC (test only — its own
# node ~/yolo_test on the internal PC, nothing shared with the traffic-light / hula-hoop nodes). Robot does not move.
#   yolo_view.sh            hula-hoop engine (tl6_1119: tl_red tl_green rh yh unknown cone)      (alias d1yolo)
#   yolo_view.sh tl         traffic-light engine (tl_cone_1104)
#   yolo_view.sh <engine>   any engine file on the internal PC
# Window keys: c capture one full-size frame (on the robot) · r capture every frame on/off · s save the view · q quit.
# Closing the window (q) stops the test node and copies the captures to logs/yolo_test/captures/ (jpg + YOLO txt + json).
# The camera is left running.
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
ROBOT=${ROBOT:-unitree@192.168.123.18}
SSH=(ssh -o "ControlPath=$HOME/.ssh/go2_cm_%r@%h:%p" -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6)
case "${1:-}" in
  '') E='~/junsu_vision/hoop_384x640_fp16.engine';;
  tl) E='~/junsu_vision/tl_cone_1104_384x640_fp16.engine';;
  *)  E=$1;;
esac
ping -c1 -W1 192.168.123.18 >/dev/null 2>&1 || { echo "★ internal PC (192.168.123.18) not reachable — robot power / LAN (robot-br)"; exit 1; }
T=$(mktemp -d)
cp "$ROOT/src/vision/yolo_test/"* "$T/"
cp "$ROOT/src/vision/start_light/tl_yolo_common.py" "$ROOT/src/vision/start_light/trt_engine.py" "$T/"   # read-only copies
rsync -a -e "${SSH[*]}" "$T/" "$ROBOT:yolo_test/" || { echo "★ copy to the internal PC failed"; exit 1; }
rm -rf "$T"
"${SSH[@]}" $ROBOT "bash ~/yolo_test/run.sh start $E" || exit 1
fin(){
  "${SSH[@]}" $ROBOT "bash ~/yolo_test/run.sh stop"
  mkdir -p "$ROOT/logs/yolo_test/captures"
  if "${SSH[@]}" $ROBOT "ls ~/yolo_test/captures/*/*.jpg" >/dev/null 2>&1; then
    rsync -a -e "${SSH[*]}" "$ROBOT:yolo_test/captures/" "$ROOT/logs/yolo_test/captures/" \
      && echo "captures -> $ROOT/logs/yolo_test/captures ($(ls "$ROOT"/logs/yolo_test/captures/*/*.jpg 2>/dev/null | wc -l) frames in all)"
  fi
}
trap fin EXIT
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export DISPLAY=${DISPLAY:-:0}
python3 "$ROOT/scripts/yolo_view.py"
