#!/usr/bin/env bash
# Isolated ROS integration test of day 1: real odom TF + motor bridge + Nav2 controller_server(RPP) + runner,
# fake Go2 / XT16 walls + FAST-LIO pose (mode wall) or D435 (mode vision) / traffic light (sim/fake_world_node.py).
# Loopback only, domain 81 — nothing reaches the robot.
#   tests/day1_ros_integration.sh [lane=2|in|out] [off0=0] [yaw0=0] [v=0.5] [mode=wall] [other=true] [light_on=true]
#   env: RUNNER_ARGS='-p track_dx:=0.05 ...' (runner -p), VIA_SH=1 (start the runner through scripts/day1.sh run), FAKE_T_MAX,
#        FAKE_ARGS='-p hoop_red_at:=x,y ...' (fake world -p), FAKE_TRAIL=<csv> (truth path written at the end)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
LANE=2; OFF0=0.0; YAW0=0.0; V=0.5; S0=0.4; MODE=wall; OTHER=true; LIGHT_ON=true
for kv in "$@"; do case $kv in lane=*) LANE=${kv#*=};; off0=*) OFF0=${kv#*=};; yaw0=*) YAW0=${kv#*=};; v=*) V=${kv#*=};; s0=*) S0=${kv#*=};;
  mode=*) MODE=${kv#*=};; other=*) OTHER=${kv#*=};; light_on=*) LIGHT_ON=${kv#*=};; esac; done
case $LANE in in) FLANE=2;; out) FLANE=1;; *) FLANE=$LANE;; esac
[ "$MODE" = wall ] && export ODOM=fake || export ODOM=builtin
export ROS_DOMAIN_ID=${TEST_DOMAIN:-81}
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
export DAY1_LOG=$ROOT/logs/day1_test; rm -rf "$DAY1_LOG"; mkdir -p "$DAY1_LOG"
[ -n "${ROUTES_FROM:-}" ] && cp -r "$ROUTES_FROM" "$DAY1_LOG/routes"     # for VIA_SH=1 SH_ARGS=route=...
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cleanup() { [ -n "${FAKE:-}" ] && kill $FAKE 2>/dev/null; "$ROOT/scripts/day1.sh" down >/dev/null 2>&1; [ -n "${RUNNER:-}" ] && kill -INT -- -$RUNNER 2>/dev/null; sleep 0.5; [ -n "${RUNNER:-}" ] && kill -9 -- -$RUNNER 2>/dev/null; true; }
trap cleanup EXIT
FAKE_T_MAX=${FAKE_T_MAX:-120} python3 "$ROOT/src/mission/track_day1/sim/fake_world_node.py" --ros-args -p lane:=$FLANE -p off0:=$OFF0 \
  -p yaw0:=$YAW0 -p s0:=$S0 -p t_green:=6.0 -p mode:=$MODE -p other:=$OTHER -p light_on:=$LIGHT_ON ${FAKE_ARGS:-} > "$DAY1_LOG/fake.log" 2>&1 &
FAKE=$!
sleep 2
"$ROOT/scripts/day1.sh" up || exit 1
if [ "${VIA_SH:-0}" = 1 ]; then   # the way it is run on site: day1.sh run in|out key=value ...
  case $LANE in 2) SL=in;; 1) SL=out;; *) SL=$LANE;; esac
  setsid "$ROOT/scripts/day1.sh" run $SL v=$V keys=false mode=$MODE ${SH_ARGS:-} > "$DAY1_LOG/runner.log" 2>&1 < /dev/null &
else
  setsid python3 "$ROOT/src/mission/track_day1/day1_runner_node.py" --ros-args -p lane:=$LANE -p v:=$V -p keys:=false -p mode:=$MODE ${RUNNER_ARGS:-} \
    > "$DAY1_LOG/runner.log" 2>&1 < /dev/null &
fi
RUNNER=$!
wait $FAKE
grep -E 'RESULT|Traceback|Error' "$DAY1_LOG/fake.log"
echo "--- runner"; grep -E 'lane [0-9] \(|path:|lane found|start found|GREEN|lap done|stopped|STOP|★|WARN|ERROR|refused|Traceback|zone|look|hoop|route now' "$DAY1_LOG/runner.log" | head -30
[ -f "$DAY1_LOG/bench.csv" ] && { echo "--- bench"; tail -1 "$DAY1_LOG/bench.csv"; [ -n "${BENCH_OUT:-}" ] && tail -1 "$DAY1_LOG/bench.csv" >> "$BENCH_OUT"; }
echo "--- cmd type"; ros2 topic info /cmd_vel 2>/dev/null | head -2
