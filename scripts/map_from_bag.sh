#!/usr/bin/env bash
# Map from a recording, isolated (loopback only, own ROS domain): bag at real speed -> imu/cloud bridge -> FAST-LIO
# (full undistorted scans) -> ros2 bag record /cloud_registered /Odometry -> scripts/map_build.py (2 cm voxels,
# floor levelled, aligned to the arena wall = track frame) -> maps/<name>/
#   scripts/map_from_bag.sh <bag dir> <name> [domain=86]
# ★ Real speed only: a slowed bag without sim time breaks the IMU timing (11-19).
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
BAG=${1:?bag}; NAME=${2:?name}; DOM=${3:-86}; DOM=${DOM#domain=}
WORK=$ROOT/logs/map_$NAME; OUT=$ROOT/maps/$NAME
rm -rf "$WORK"; mkdir -p "$WORK" "$OUT"
export ROS_DOMAIN_ID=$DOM
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source "$ROOT/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done; }
trap cleanup EXIT
BP=$ROOT/src/core/go2_bridge/config/bridge_params.yaml
setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file "$BP" > "$WORK/cloud_bridge.log" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file "$BP" > "$WORK/imu_bridge.log" 2>&1 < /dev/null & PIDS+=($!)
setsid ros2 run fast_lio fastlio_mapping --ros-args --params-file "$ROOT/config/fastlio/fastlio_xt16_odometry.yaml" \
  -p publish.dense_publish_en:=true -p publish.scan_publish_en:=true > "$WORK/fastlio.log" 2>&1 < /dev/null & PIDS+=($!)
sleep 3
for n in /go2_cloud_bridge /go2_imu_bridge; do
  for s in configure activate; do ros2 lifecycle set $n $s > /dev/null 2>&1 || { echo "★ lifecycle $n $s failed"; exit 1; }; done
done
setsid ros2 bag record -s mcap -o "$WORK/lio" /cloud_registered /Odometry > "$WORK/record.log" 2>&1 < /dev/null & REC=$!; PIDS+=($REC)
sleep 3
echo "playing $(basename "$BAG") at real speed ..."
ros2 bag play "$BAG" --topics /lidar_points /lowstate /utlidar/robot_odom > "$WORK/play.log" 2>&1 < /dev/null
sleep 3
kill -TERM -- -$REC 2>/dev/null; sleep 3          # bag record ignores SIGINT in the background
grep -E "PAUSED|No point|diverg" "$WORK/fastlio.log" | head -3
python3 "$ROOT/scripts/map_build.py" "$WORK/lio" "$BAG" "$OUT"
