#!/usr/bin/env bash
# Which branch red / yellow would take now (the runner's own choice, live hoop vision). Robot does not move.
#   hoop_which.sh [seconds=15] [route=rec105326_lap1] [pose=x,y,yaw_deg]      (alias d1which)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
python3 "$ROOT/scripts/hoop_which.py" "$@"
