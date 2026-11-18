#!/usr/bin/env bash
# FAST-LIO on a recording, isolated (loopback only, own domain): cloud/imu bridge + fastlio_mapping, /Odometry -> CSV.
#   tests/fastlio_replay.sh <bag dir> <out.csv> [domain=81] [rate=1.0] [-p fast-lio overrides ...]
#   e.g. tests/fastlio_replay.sh ~/bags/gosu_..._rec_inline /tmp/a.csv domain=82 -p leg.vel_en:=false
# Plays only the three inputs (/lidar_points /lowstate /utlidar/robot_odom). Nothing reaches the robot.
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
BAG=${1:?bag}; OUT=${2:?out.csv}; shift 2
DOM=81; RATE=1.0; EXTRA=()
while [ $# -gt 0 ]; do case $1 in domain=*) DOM=${1#*=};; rate=*) RATE=${1#*=};; -p) EXTRA+=(-p "$2"); shift;; esac; shift; done
export ROS_DOMAIN_ID=$DOM
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LOG=${OUT%.csv}.log; : > "$LOG"
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; sleep 1; for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done; }
trap cleanup EXIT
BP=$ROOT/src/core/go2_bridge/config/bridge_params.yaml
setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file "$BP" -p use_sim_time:=true >> "$LOG" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file "$BP" -p use_sim_time:=true >> "$LOG" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run fast_lio fastlio_mapping --ros-args --params-file "$ROOT/config/fastlio/fastlio_xt16_odometry.yaml" \
  -p use_sim_time:=true "${EXTRA[@]}" >> "$LOG" 2>&1 < /dev/null & PIDS+=($!)
setsid python3 - "$OUT" >> "$LOG" 2>&1 < /dev/null <<'EOF' &
import math, sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('odom_csv'); f = open(sys.argv[1], 'w'); f.write('t,x,y,z,yaw\n')
def cb(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    f.write('%.4f,%.4f,%.4f,%.4f,%.5f\n' % (m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z, yaw)); f.flush()
n.create_subscription(Odometry, '/Odometry', cb, 50)
rclpy.spin(n)
EOF
PIDS+=($!)
sleep 3
for n in /go2_cloud_bridge /go2_imu_bridge; do   # lifecycle nodes: nothing comes out until active
  for s in configure activate; do ros2 lifecycle set $n $s >> "$LOG" 2>&1 || { echo "lifecycle $n $s failed"; exit 1; }; done
done
sleep 1
ros2 bag play "$BAG" --topics /lidar_points /lowstate /utlidar/robot_odom --clock 1000 -r "$RATE" >> "$LOG" 2>&1 < /dev/null
sleep 2
echo "$(basename "$BAG"): $(($(wc -l < "$OUT") - 1)) odometry rows -> $OUT"
