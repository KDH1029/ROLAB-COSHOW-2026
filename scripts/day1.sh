#!/usr/bin/env bash
# Day 1 track race (NUC). Commands that move the robot are run by you in your own terminal.
#   day1.sh up            FAST-LIO (imu/cloud bridge + fastlio_mapping, no map) -> odom TF + motor bridge
#                         (min_wz/min_vy 0, inactive) + Nav2 controller_server(RPP).  ★ robot standing still (IMU init)
#                         ODOM=builtin: built-in odometry x1.189 instead (vision spare), ODOM=fake: test only
#   day1.sh vision push   copy the camera edge node to the internal PC (~/coshow_track)
#   day1.sh vision start  camera (if not running) + edge node on the internal PC
#   day1.sh vision snap   camera + bird's-eye check images -> logs/snap/
#   day1.sh light         start the YOLO traffic-light node (~/junsu_vision on the internal PC)
#   day1.sh status        rates of the topics that matter
#   day1.sh run lane=2 v=0.5 [light=none] [extra=1.2] [mode=vision]   ★ stand the robot first; space/x = STOP
#   day1.sh down          stop what `up` started
#   day1.sh test          offline checks (vision, closed-loop sim)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
TD=$ROOT/src/mission/track_day1
LOG=${DAY1_LOG:-$ROOT/logs/day1}; mkdir -p "$LOG"
ROBOT=${ROBOT:-unitree@192.168.123.18}
SSH=(ssh -o "ControlPath=$HOME/.ssh/go2_cm_%r@%h:%p" -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6)
ODOM=${ODOM:-lio}            # lio | builtin | fake
ODOM_SCALE=${ODOM_SCALE:-1.189}
NODES="imu_bridge cloud_bridge fastlio nav2_lm nav2 bridge odom_tf"

env_ros() {
  # rmw_cyclonedds_cpp for Jazzy is built in ~/unitree_ros2/cyclonedds_ws on this NUC (not from apt)
  set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
  source "$ROOT/install/setup.bash"; set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
}

bg() {  # bg <name> <cmd...>
  local n=$1; shift
  setsid nohup "$@" > "$LOG/$n.log" 2>&1 < /dev/null &
  echo $! > "$LOG/$n.pid"
}

alive() { [ -f "$LOG/$1.pid" ] && kill -0 "$(cat "$LOG/$1.pid")" 2>/dev/null; }

lc() {  # lc <node> <transition>, retries until the node is up
  for _ in $(seq 20); do ros2 lifecycle set "$1" "$2" >/dev/null 2>&1 && return 0; sleep 0.5; done
  echo "★ $1 $2 failed"; return 1
}

case "${1:-}" in
  up)
    env_ros
    CHK="cmd_vel_to_sport_node controller_server odom_tf_node"
    [ "$ODOM" = lio ] && CHK="$CHK imu_bridge_node cloud_bridge_node fastlio_mapping"
    for p in $CHK; do   # only processes on our ROS domain (unset = 0) clash with ours
      for o in $(pgrep -f "[/]$p"); do
        d=$(tr '\0' '\n' < /proc/$o/environ 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p')
        [ "${d:-0}" = "$ROS_DOMAIN_ID" ] || continue
        echo "★ $p already running on domain $ROS_DOMAIN_ID (pid $o, not started by day1.sh?) — stop it first (FORCE=1 to ignore)"
        [ "${FORCE:-0}" = 1 ] || exit 1
      done
    done
    BP=$ROOT/src/core/go2_bridge/config/bridge_params.yaml
    case $ODOM in
      lio)
        timeout 5 ros2 topic echo --once /lidar_points sensor_msgs/msg/PointCloud2 --field header >/dev/null 2>&1 \
          || { echo "★ no /lidar_points in 5 s — XT16 driver running? (ping 192.168.123.20)"; exit 1; }
        bg imu_bridge ros2 run go2_bridge imu_bridge_node --ros-args --params-file "$BP"
        bg cloud_bridge ros2 run go2_bridge cloud_bridge_node --ros-args --params-file "$BP"
        lc /go2_imu_bridge configure && lc /go2_imu_bridge activate || exit 1
        lc /go2_cloud_bridge configure && lc /go2_cloud_bridge activate || exit 1
        bg fastlio ros2 run fast_lio fastlio_mapping --ros-args --params-file "$ROOT/config/fastlio/fastlio_xt16_odometry.yaml"
        timeout 20 ros2 topic echo --once /Odometry nav_msgs/msg/Odometry --field header >/dev/null 2>&1 \
          || { echo "★ no /Odometry from FAST-LIO in 20 s — see $LOG/fastlio.log"; exit 1; }
        OT=(-p odom_topic:=/Odometry -p xy_scale:=1.0);;
      fake) OT=(-p odom_topic:=/Odometry -p xy_scale:=1.0);;
      builtin) OT=(-p xy_scale:=$ODOM_SCALE);;
      *) echo "ODOM=lio|builtin|fake"; exit 1;;
    esac
    bg odom_tf ros2 run go2_bridge odom_tf_node --ros-args -r __node:=go2_odom_tf --params-file "$BP" "${OT[@]}"
    bg bridge ros2 run go2_bridge cmd_vel_to_sport_node --ros-args -r __node:=go2_cmd_vel_to_sport --params-file "$BP" \
      -p min_wz:=0.0 -p min_vy:=0.0
    lc /go2_odom_tf configure && lc /go2_odom_tf activate
    lc /go2_cmd_vel_to_sport configure    # activated by the runner once the lane is found
    # the local costmap needs odom -> base_link before it configures (built-in odometry must be flowing)
    timeout 10 ros2 topic echo --once /tf tf2_msgs/msg/TFMessage >/dev/null 2>&1 \
      || { echo "★ no odom TF in 10 s (ODOM=$ODOM)"; exit 1; }
    bg nav2 ros2 run nav2_controller controller_server --ros-args --params-file "$ROOT/config/day1_nav2.yaml"
    bg nav2_lm ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_day1 \
      -p autostart:=true -p "node_names:=['controller_server']" -p bond_timeout:=0.0
    for _ in $(seq 30); do ros2 lifecycle get /controller_server 2>/dev/null | grep -q '^active \[3\]' && break; sleep 0.5; done
    ros2 lifecycle get /controller_server 2>/dev/null | grep -q '^active \[3\]' \
      && echo "■ up: odom $ODOM · bridge (inactive) · controller_server(RPP) active" \
      || { echo "★ controller_server not active — see $LOG/nav2.log"; exit 1; };;
  down)
    for n in $NODES; do
      if alive $n; then kill -INT -- -"$(cat "$LOG/$n.pid")" 2>/dev/null; fi
    done
    sleep 1
    for n in $NODES; do alive $n && kill -TERM -- -"$(cat "$LOG/$n.pid")" 2>/dev/null; rm -f "$LOG/$n.pid"; done
    echo "■ down";;
  vision)
    case "${2:-}" in
      push)
        "${SSH[@]}" $ROBOT 'mkdir -p ~/coshow_track' || exit 1
        rsync -a -e "${SSH[*]}" "$TD/band_vision_core.py" "$TD/band_vision_node.py" "$TD/track_spec.py" "$TD/jetson_track.sh" \
          $ROBOT:coshow_track/ && "${SSH[@]}" $ROBOT 'cd ~/coshow_track && md5sum *.py *.sh' && echo "■ pushed";;
      start|stop|status) "${SSH[@]}" $ROBOT "bash ~/coshow_track/jetson_track.sh ${2}";;
      snap)
        env_ros
        ros2 topic pub --once /track/snap std_msgs/msg/Bool '{data: true}' >/dev/null; sleep 1.5
        mkdir -p "$LOG/snap"; rsync -a -e "${SSH[*]}" $ROBOT:coshow_track/snap/ "$LOG/snap/" && ls -t "$LOG/snap" | head -2;;
      *) echo "vision push|start|stop|status|snap";;
    esac;;
  light) "${SSH[@]}" $ROBOT 'bash ~/junsu_vision/jetson_vision.sh start';;
  status)
    env_ros
    for tp in /lidar_points /Odometry /mission/traffic_light_state /utlidar/robot_odom /cmd_vel /lowstate; do
      printf '%-32s ' $tp; timeout 4 ros2 topic hz $tp --window 20 2>/dev/null | grep -m1 average || echo '-'
    done
    ros2 lifecycle get /go2_cmd_vel_to_sport 2>/dev/null; ros2 lifecycle get /controller_server 2>/dev/null;;
  run)
    env_ros; shift
    A=(); for kv in "$@"; do A+=(-p "${kv%%=*}:=${kv#*=}"); done
    python3 "$TD/day1_runner_node.py" --ros-args "${A[@]}" 2>&1 | tee "$LOG/run_$(date +%H%M%S).log";;
  test)
    cd "$TD" && python3 sim/eval_vision.py base,glare && python3 sim/sim_day1.py --only lane2,lane1,lane2_place;;
  *) sed -n 2,12p "$0";;
esac
