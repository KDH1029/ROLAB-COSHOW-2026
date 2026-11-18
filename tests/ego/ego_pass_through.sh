#!/usr/bin/env bash
# Isolated test of ego pass-through: fake robot (tests/ego/fake_route_node.py) + real ego_planner_node.
# Route (2,0) (4,0) (4,2) (2,2) in camera_init. mode=single sends one /goal_pose per point (old behaviour),
# mode=pass sends /goal_path + one /goal_pose. Loopback only, domain 81 — nothing reaches the robot.
#   tests/ego/ego_pass_through.sh            # runs both modes
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
LOG=$ROOT/logs/ego_pt_test; rm -rf "$LOG"; mkdir -p "$LOG"
export ROS_DOMAIN_ID=81
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
pgrep -f "[i]nstall/ego_planner/lib/ego_planner/ego_planner_node" >/dev/null && { echo "★ another ego_planner_node is running — stop it first"; exit 1; }
CFG=$ROOT/src/navigation/ego_planner/config/ego_planner_params.yaml
for MODE in ${MODES:-single pass}; do
  "$ROOT/install/ego_planner/lib/ego_planner/ego_planner_node" --ros-args --params-file "$CFG" > "$LOG/ego_$MODE.log" 2>&1 < /dev/null &
  EGO=$!
  /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher --frame-id camera_init --child-frame-id base_link > /dev/null 2>&1 < /dev/null &
  TFP=$!
  for _ in $(seq 20); do ros2 lifecycle set /ego_planner_node configure >/dev/null 2>&1 && break; sleep 0.5; done
  ros2 lifecycle set /ego_planner_node activate >/dev/null 2>&1 || echo "★ activate failed"
  python3 "$ROOT/tests/ego/fake_route_node.py" --ros-args -p mode:=$MODE -p t_max:=60.0 2>&1 < /dev/null | tee "$LOG/fake_$MODE.log" | grep RESULT
  kill -TERM $EGO $TFP 2>/dev/null; sleep 1; kill -9 $EGO $TFP 2>/dev/null; wait $EGO $TFP 2>/dev/null
  grep -c "pass-through" "$LOG/ego_$MODE.log" | sed "s/^/  ego log lines with pass-through: /"
done
