#!/usr/bin/env bash
# On the internal PC, ~/yolo_test/run.sh start [engine] | stop | status   (called by scripts/yolo_view.sh on the NUC)
# Test only: its own node and log; the camera is started only if it is not running (same arguments as the recording).
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
set +u; source /opt/ros/foxy/setup.bash; source ~/cyclonedds_ws/install/setup.bash; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
mkdir -p $D/logs
pid(){ pgrep -f "[y]olo_view_node.py"; }
cam_pid(){ pgrep -f "[r]ealsense2_camera_node"; }
case "${1:-status}" in
  start)
    E=${2:-$HOME/junsu_vision/hoop_384x640_fp16.engine}
    [ -f "$E" ] || { echo "★ engine $E not found"; ls ~/junsu_vision/*.engine; exit 1; }
    if [ -z "$(cam_pid)" ]; then
      P=848x480x30
      setsid nohup ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_infra1:=false \
        enable_infra2:=false enable_gyro:=false enable_accel:=false rgb_camera.profile:=$P rgb_camera.color_profile:=$P \
        depth_module.profile:=$P depth_module.depth_profile:=$P > $D/logs/camera_$(date +%H%M%S).log 2>&1 < /dev/null &
      echo "camera started ($P)"; sleep 6
    else echo "camera already running: pid $(cam_pid | tr '\n' ' ')"; fi
    [ -n "$(pid)" ] && pkill -INT -f "[y]olo_view_node.py" && sleep 1
    L=$D/logs/view_$(date +%H%M%S).log
    setsid nohup nice -n 19 taskset -c ${CORES:-6,7} python3 $D/yolo_view_node.py --ros-args -p engine:=$E > $L 2>&1 < /dev/null &
    sleep 6; grep -v -e FutureWarning -e hasattr $L | tail -2
    [ -n "$(pid)" ] && echo "■ yolo test node on ($L)" || { echo "★ node did not start — $L"; exit 1; };;
  stop) pkill -INT -f "[y]olo_view_node.py" && echo "yolo test node off" || echo "no yolo test node";;
  status) echo "yolo test node: $(pid | tr '\n' ' ')  camera: $(cam_pid | tr '\n' ' ')";;
esac
