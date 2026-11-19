#!/usr/bin/env bash
# Force everything off on the NUC (domain 0 only), also what day1.sh did not start ("already running" from `up`):
#   runners / align / tools -> motor bridge, RPP, odom TF, FAST-LIO, bridges -> XT16 driver.
# The robot gets StopMove first. Not touched: ros2 bag record (a recording in progress), other ROS domains (isolated tests).
#   kill_all.sh          NUC                                                     (alias dkill)
#   kill_all.sh robot    + the internal PC's traffic-light / hula-hoop / yolo-test nodes and the camera
# Then: dxt, dlight, dhoopon, dup as usual.
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
LOG=${DAY1_LOG:-$ROOT/logs/day1}
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source "$ROOT/install/setup.bash" 2>/dev/null; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
export ROS_DOMAIN_ID=0
PATS="day1_runner_node.py practice_runner_node.py cone_runner_node.py start_align_node.py hoop_check.py hoop_which.py yolo_view.py \
cmd_vel_to_sport_node controller_server lifecycle_manager odom_tf_node imu_bridge_node cloud_bridge_node fastlio_mapping \
hesai_ros_driver_node"
dom0(){ local d; d=$(tr '\0' '\n' < /proc/$1/environ 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p'); [ "${d:-0}" = 0 ]; }
pids(){ for p in $PATS; do for o in $(pgrep -f "[/]$p|^python3 .*$p"); do
          [ "$o" = $$ ] && continue; ps -o args= -p $o 2>/dev/null | grep -q "ros2 bag" && continue
          dom0 $o && echo "$o"; done; done | sort -u; }
P=$(pids)
if [ -n "$P" ]; then
  timeout 3 ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 1, api_id: 1003}}}" >/dev/null 2>&1 \
    && echo "robot: StopMove sent"
  echo "stopping: $(for o in $P; do ps -o comm= -p $o; done | sort | uniq -c | awk '{printf "%s%s ", $2, ($1>1?"x"$1:"")}')"
  kill -INT $P 2>/dev/null; sleep 2
  L=$(for o in $P; do kill -0 $o 2>/dev/null && echo $o; done); [ -n "$L" ] && { kill -TERM $L 2>/dev/null; sleep 1; }
  L=$(for o in $P; do kill -0 $o 2>/dev/null && echo $o; done); [ -n "$L" ] && { kill -KILL $L 2>/dev/null; echo "killed hard: $L"; }
else
  echo "nothing running on the NUC (domain 0)"
fi
for n in imu_bridge cloud_bridge fastlio nav2_lm nav2 bridge odom_tf; do rm -f "$LOG/$n.pid"; done
R=$(pgrep -af "ros2 bag record" | grep -v pgrep | head -2); [ -n "$R" ] && echo "★ left alone (recording): $R"
if [ "${1:-}" = robot ]; then
  SSH=(ssh -o "ControlPath=$HOME/.ssh/go2_cm_%r@%h:%p" -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6)
  "${SSH[@]}" unitree@192.168.123.18 'bash ~/junsu_vision/jetson_vision.sh stop; bash ~/junsu_vision/jetson_vision.sh hoop-stop; \
    [ -x ~/yolo_test/run.sh ] && bash ~/yolo_test/run.sh stop; bash ~/junsu_vision/jetson_vision.sh stop-camera' 2>&1 | grep -v '^$' \
    || echo "★ internal PC not reachable"
fi
ros2 daemon stop >/dev/null 2>&1
echo "■ off — left: $(pids | wc -w) process(es). Next: dxt, dlight, dhoopon, dup"
