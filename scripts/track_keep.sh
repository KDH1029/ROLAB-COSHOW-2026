#!/usr/bin/env bash
# Keep the track position from the start mark (robot standing on it, `dup` running): track_keep.sh [route=...] (alias dkeep)
# Then walk the robot anywhere with the remote and start practice runs with: dhere lock=keep
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
r=rec105326_lap1; for kv in "$@"; do case $kv in route=*) r=${kv#*=};; esac; done
f=$ROOT/logs/day1/routes/$r.csv; [ -f "$f" ] || f=$ROOT/routes/$r.csv
python3 "$ROOT/src/mission/track_day1/track_keep_node.py" --ros-args -p "path_file:=$f"
