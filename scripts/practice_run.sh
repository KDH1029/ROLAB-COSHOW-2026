#!/usr/bin/env bash
# Practice: start ANYWHERE on the route (stand on it, facing along it) and stop at the start mark — test only, a fork
# of the runner (day1.sh run is not changed). Needs `day1.sh up`.  ★ moves the robot: space/x = STOP, q = quit.
#   practice_run.sh [route=rec105326_lap1] [hoop=in] [v=0.8] [debug=true light=none] [...]      (alias dhere)
# Defaults: route rec105326_lap1, hoop=in, v=0.8, lookahead 0.5, min_radius 0.4, max_lat 0.5, max_head 30,
#   debug=true light=none (mid-course the traffic light is not in view: it goes as soon as it has found itself).
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
TD=$ROOT/src/mission/track_day1
LOG=${DAY1_LOG:-$ROOT/logs/day1}; mkdir -p "$LOG"
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
has(){ printf '%s\n' "$@" | grep -q "^$1="; }
r=rec105326_lap1; A=()
for kv in "$@"; do
  case $kv in
    route=*) r=${kv#*=};;
    *=*) A+=(-p "${kv%%=*}:=${kv#*=}");;
    *) echo "★ unknown argument $kv (key=value)"; exit 1;;
  esac
done
f=$LOG/routes/$r.csv; [ -f "$f" ] || f=$ROOT/routes/$r.csv
[ -f "$f" ] || { echo "★ no route $r"; exit 1; }
echo "route: $f"
D=()
printf '%s\n' "$@" | grep -q '^hoop=' || D+=(-p hoop:=in)
printf '%s\n' "$@" | grep -q '^v=' || D+=(-p v:=0.8)
printf '%s\n' "$@" | grep -q '^lookahead=' || D+=(-p lookahead:=0.5)
printf '%s\n' "$@" | grep -q '^min_radius=' || D+=(-p min_radius:=0.4)
printf '%s\n' "$@" | grep -q '^max_lat=' || D+=(-p max_lat:=0.5)
printf '%s\n' "$@" | grep -q '^max_head=' || D+=(-p max_head:=30.0)
printf '%s\n' "$@" | grep -q '^light=' || D+=(-p light:=none)
printf '%s\n' "$@" | grep -q '^debug=' || D+=(-p debug:=true)
python3 "$TD/practice_runner_node.py" --ros-args -p "path_file:=$f" "${D[@]}" "${A[@]}" 2>&1 | tee "$LOG/run_practice_$(date +%H%M%S).log"
