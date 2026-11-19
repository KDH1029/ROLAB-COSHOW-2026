#!/usr/bin/env bash
# Start alignment, BEFORE the run command (separate program; day1.sh and the runner are not changed):
# turn in place to the route's direction, then step sideways onto the route; then stop.
#   day1_align.sh [route=rec105326_lap1] [tol_lat=0.03] [tol_head=3] [gait=none]
# Needs `day1.sh up` and `day1.sh xt16`; robot standing on the start mark. ★ moves the robot — space/x/q = stop.
# Afterwards: day1.sh run route=... as usual (its start lock prints lateral / heading, ~0 after this).
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
TD=$ROOT/src/mission/track_day1
LOG=${DAY1_LOG:-$ROOT/logs/day1}; mkdir -p "$LOG"
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

r=rec105326_lap1; A=()
for kv in "$@"; do
  case $kv in
    route=*) r=${kv#*=};;
    *=*) A+=(-p "${kv%%=*}:=${kv#*=}");;
    *) echo "★ unknown argument $kv (key=value)"; exit 1;;
  esac
done
f=$LOG/routes/$r.csv; [ -f "$f" ] || f=$ROOT/routes/$r.csv
[ -f "$f" ] || { echo "★ no route $r in $LOG/routes or $ROOT/routes"; exit 1; }
echo "route: $f"
python3 "$TD/start_align_node.py" --ros-args -p "path_file:=$f" "${A[@]}" 2>&1 | tee "$LOG/align_$(date +%H%M%S).log"
