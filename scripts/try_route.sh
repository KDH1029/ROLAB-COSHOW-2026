#!/usr/bin/env bash
# Run the mission course on one of the 11-20 cone-field variants (recording passes 1 / 2 / 3), the real runner.
#   try_route.sh <1|2|3> [day1.sh run arguments]        (alias dtry)   e.g. dtry 2 hoop=red light_timeout=10
# Defaults (unless given): hoop=in v=0.8. Routes rec1120p<N>_lap1 / _lap2 (inner / outer arch), same start mark as the
# default rec105326_* (which is pass 3): dalign first as usual. Practice from mid-course: dhere route=rec1120p<N>_lap1
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
n=${1:-}; case "$n" in 1|2|3) shift;; *) echo "★ which pass: dtry 1 | dtry 2 | dtry 3 [args]"; exit 1;; esac
A=("route=rec1120p${n}_lap1")
printf '%s\n' "$@" | grep -q '^hoop=' || A+=(hoop=in)
printf '%s\n' "$@" | grep -q '^v=' || A+=(v=0.8)
exec "$ROOT/scripts/day1.sh" run "${A[@]}" "$@"
