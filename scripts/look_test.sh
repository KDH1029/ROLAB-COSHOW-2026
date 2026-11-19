#!/usr/bin/env bash
# Test only: drive the route to the hula-hoop look point the way a real run does, and STAY there facing the arches,
# to see what the camera sees (d1yolo in another terminal; r = record). Nothing new runs: it is `day1.sh run` with
#   debug=true light=none        no traffic light (go at once)
#   hoop=red + hoop node OFF     at the look point (pin 15, 2.69 1.78, facing 177 deg) no answer comes, so it waits
#   look_timeout=600             ... 10 minutes, then it would pick a branch at random and drive on: stop before (space)
# The hula-hoop node is turned back on when this ends (also on Ctrl+C).  ★ moves the robot: space/x = STOP, q = quit.
#   look_test.sh [route=rec105326_lap1] [v=0.8] [look_timeout=600]        (alias d1look; d1align first)
#   look_test.sh here [...]    start past the cone field instead (practice_run.sh: anywhere on the route, no light;
#                              put it where d1here starts — logs/day1/practice_start.png)      (alias d1lookhere)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
D1=$ROOT/scripts/day1.sh
RUN=("$D1" run debug=true light=none)
if [ "${1:-}" = here ]; then shift; RUN=("$ROOT/scripts/practice_run.sh"); fi
printf '%s\n' "$@" | grep -q '^route=' || set -- route=rec105326_lap1 "$@"
printf '%s\n' "$@" | grep -q '^v=' || set -- "$@" v=0.8
printf '%s\n' "$@" | grep -q '^look_timeout=' || set -- "$@" look_timeout=600
"$D1" hoop stop
trap 'echo "hula-hoop node back on:"; "$D1" hoop start' EXIT
"${RUN[@]}" hoop=red "$@"
