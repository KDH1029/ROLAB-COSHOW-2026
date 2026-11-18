#!/usr/bin/env bash
# Isolated ROS integration test of day 1: real odom TF + motor bridge + Nav2 controller_server(RPP) + runner,
# fake Go2 / D435 / traffic light (src/mission/track_day1/sim/fake_world_node.py).
# Loopback only, domain 81 — nothing reaches the robot.     tests/day1_ros_integration.sh [lane=2] [off0=0] [yaw0=0] [v=0.5]
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
LANE=2; OFF0=0.0; YAW0=0.0; V=0.5; S0=0.4
for kv in "$@"; do case $kv in lane=*) LANE=${kv#*=};; off0=*) OFF0=${kv#*=};; yaw0=*) YAW0=${kv#*=};; v=*) V=${kv#*=};; s0=*) S0=${kv#*=};; esac; done
export ROS_DOMAIN_ID=81
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
export DAY1_LOG=$ROOT/logs/day1_test; rm -rf "$DAY1_LOG"; mkdir -p "$DAY1_LOG"
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cleanup() { [ -n "${FAKE:-}" ] && kill $FAKE 2>/dev/null; "$ROOT/scripts/day1.sh" down >/dev/null 2>&1; [ -n "${RUNNER:-}" ] && kill -INT $RUNNER 2>/dev/null; sleep 0.5; [ -n "${RUNNER:-}" ] && kill -9 $RUNNER 2>/dev/null; true; }
trap cleanup EXIT
FAKE_T_MAX=120 python3 "$ROOT/src/mission/track_day1/sim/fake_world_node.py" --ros-args -p lane:=$LANE -p off0:=$OFF0 \
  -p yaw0:=$YAW0 -p s0:=$S0 -p t_green:=6.0 > "$DAY1_LOG/fake.log" 2>&1 &
FAKE=$!
sleep 2
"$ROOT/scripts/day1.sh" up || exit 1
python3 "$ROOT/src/mission/track_day1/day1_runner_node.py" --ros-args -p lane:=$LANE -p v:=$V -p keys:=false \
  > "$DAY1_LOG/runner.log" 2>&1 &
RUNNER=$!
wait $FAKE
grep -E 'RESULT|Traceback|Error' "$DAY1_LOG/fake.log"
echo "--- runner"; grep -E 'lane found|GREEN|lap done|stopped|STOP|★|WARN|ERROR|refused' "$DAY1_LOG/runner.log" | head -20
echo "--- cmd type"; ros2 topic info /cmd_vel 2>/dev/null | head -2
