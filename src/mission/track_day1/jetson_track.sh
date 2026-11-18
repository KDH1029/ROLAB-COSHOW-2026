#!/usr/bin/env bash
# Internal PC (Jetson, Foxy): camera + floor edge node.   jetson_track.sh start|stop|status
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
set +u; source /opt/ros/foxy/setup.bash; [ -f ~/cyclonedds_ws/install/setup.bash ] && source ~/cyclonedds_ws/install/setup.bash; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[ -f ~/cyclonedds_ws/cyclonedds.xml ] && export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
CORES=${CORES:-4,5}; HZ=${HZ:-10}
mkdir -p $D/logs
node_pid(){ pgrep -f "[b]and_vision_node.py"; }
cam_pid(){ pgrep -f "[r]ealsense2_camera_node"; }
case "${1:-status}" in
  start)
    if [ -z "$(cam_pid)" ]; then
      P=848x480x30   # same as the traffic-light / recording launch, so either can start the camera
      setsid nohup ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_infra1:=false \
        enable_infra2:=false enable_gyro:=false enable_accel:=false rgb_camera.profile:=$P rgb_camera.color_profile:=$P \
        depth_module.profile:=$P depth_module.depth_profile:=$P > $D/logs/camera_$(date +%H%M%S).log 2>&1 < /dev/null &
      echo "camera started ($P)"; sleep 6
    else echo "camera already running: $(cam_pid | tr '\n' ' ')"; fi
    [ -n "$(node_pid)" ] && { echo "edge node already running: $(node_pid)"; exit 0; }
    L=$D/logs/band_$(date +%H%M%S).log
    setsid nohup nice -n 5 taskset -c $CORES python3 $D/band_vision_node.py --ros-args -p hz:=$HZ > $L 2>&1 < /dev/null &
    sleep 4; tail -2 $L; [ -n "$(node_pid)" ] && echo "■ edge node up ($L)" || { echo "★ edge node did not start — $L"; exit 1; };;
  stop) p=$(node_pid); [ -n "$p" ] && kill $p && echo "edge node stopped" || echo "not running";;
  status) echo "camera: $(cam_pid | tr '\n' ' ')  edge node: $(node_pid | tr '\n' ' ')"; tail -2 $(ls -t $D/logs/band_*.log 2>/dev/null | head -1) 2>/dev/null;;
esac
