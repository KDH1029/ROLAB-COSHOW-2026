#!/usr/bin/env bash
# Cone-avoiding runner (a fork — day1.sh and day1_runner_node.py are not changed; `day1.sh run` stays as it was).
# Same start, light, zones, hula hoop and lap end as `day1.sh run`; in a cone zone it also goes around cones that
# are not where the route was taught (LiDAR /scan). Needs `day1.sh up` (and xt16 / light) like day1.sh run.
#   day1_cones.sh run route=R [hoop=..] [v=..] [cone_clear=0.30] [avoid_v=0.30] [cones=false] ...
#                         everything `day1.sh run` takes, plus the cone_* parameters (cone_runner_node.py)
#   day1_cones.sh zone R [R2 ...] box=x0,y0,x1,y1 [name=cones]   cone zone (track frame) -> R.cones.json
#   day1_cones.sh show R [R2 ...] [log=logs/day1/cones_HHMMSS.jsonl] [at=max|<s>]   picture -> logs/day1/cones_R.png
#   day1_cones.sh last    picture of the last run's log (the moment of the largest shift)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
TD=$ROOT/src/mission/track_day1
LOG=${DAY1_LOG:-$ROOT/logs/day1}; mkdir -p "$LOG"

env_ros() {   # the same environment as day1.sh
  set +u; source /opt/ros/jazzy/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
  source "$ROOT/install/setup.bash"; set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$ROOT/config/cyclonedds_go2.xml}
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
}

case "${1:-}" in
  run)
    env_ros; shift
    ZZ=0; B=(); for kv in "$@"; do [ "$kv" = zz ] && ZZ=1 || B+=("$kv"); done; set -- "${B[@]}"   # zz = zigzag-edited copy of the route (R_lapN -> Rzz_lapN), the original route untouched
    A=(); for kv in "$@"; do
      case $kv in
        in|out) A+=(-p "lane:=$kv");;
        route=*) r=${kv#*=}; [ "$ZZ" = 1 ] && r=${r/_lap/zz_lap}; f=$LOG/routes/$r.csv
                 [ -f "$f" ] || f=$ROOT/routes/$r.csv
                 [ -f "$f" ] || { echo "★ no route $r in $LOG/routes or $ROOT/routes"; exit 1; }
                 echo "route: $f"; A+=(-p "path_file:=$f");;
        cv=*) A+=(-p "cones_v:=${kv#*=}");;          # cv = cones_v  (콘 구간 속도, 이번 주행만)
        zv=*) A+=(-p "zigzag_v:=${kv#*=}");;         # zv = zigzag_v (지그재그 구간 속도, 이번 주행만)
        *=*) A+=(-p "${kv%%=*}:=${kv#*=}");;
        *) echo "★ unknown argument $kv (in|out key=value)"; exit 1;;
      esac
    done
    if printf '%s\n' "$@" | grep -q '^route='; then       # the same defaults as day1.sh run for a taught route
      printf '%s\n' "$@" | grep -q '^lookahead=' || A+=(-p lookahead:=0.5)
      printf '%s\n' "$@" | grep -q '^min_radius=' || A+=(-p min_radius:=0.4)
    fi
    python3 "$TD/cone_runner_node.py" --ros-args "${A[@]}" 2>&1 | tee "$LOG/run_cones_$(date +%H%M%S).log";;
  zone) shift; python3 "$ROOT/scripts/route_cones.py" "$@";;
  show) shift; python3 "$ROOT/scripts/cones_view.py" "$@";;
  last)
    l=$(ls -t "$LOG"/cones_*.jsonl 2>/dev/null | head -1); [ -n "$l" ] || { echo "★ no cones_*.jsonl in $LOG"; exit 1; }
    python3 "$ROOT/scripts/cones_view.py" log="$l" "${@:2}";;
  *) sed -n 2,10p "$0";;
esac
