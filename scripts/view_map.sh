#!/usr/bin/env bash
# Look at a built map in RViz (own ROS domain, loopback only — nothing reaches the robot).
#   scripts/view_map.sh [maps/coshow_1119] [step=1]     closing RViz stops everything
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
MAP=${1:-$ROOT/maps/coshow_1119}; STEP=${2:-step=1}
[ -f "$MAP/map_track.npz" ] || { echo "★ no $MAP/map_track.npz (scripts/map_from_bag.sh first)"; exit 1; }
export ROS_DOMAIN_ID=${VIEW_DOMAIN:-88}
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 "$ROOT/scripts/map_view.py" "$MAP" "$STEP" & PUB=$!
trap 'kill $PUB 2>/dev/null' EXIT
rviz2 -d "$ROOT/scripts/map_view.rviz"
