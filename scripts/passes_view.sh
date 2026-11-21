#!/usr/bin/env bash
# The lap1 route editor with the latest cone recording's passes drawn in their own colours (pass 1 blue, pass 2 red, ...)
# and the whole drive (thin orange, returns included). r = hide/show them. s saves the route as usual.
#   passes_view.sh [route=rec105326_lap1] [splice=<maps/coshow_1119/routes/splice_*>]          (alias dpasses)
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
r=rec105326_lap1; d=""
for kv in "$@"; do case $kv in route=*) r=${kv#*=};; splice=*) d=${kv#*=};; esac; done
[ -n "$d" ] || d=$(ls -dt "$ROOT"/maps/coshow_1119/routes/splice_* 2>/dev/null | head -1)
[ -n "$d" ] && ls "$d"/pass_*.csv >/dev/null 2>&1 || { echo "★ no passes yet — dsplice <bag> first"; exit 1; }
P=$(ls "$d"/pass_*.csv | sort -V | paste -sd, -)
echo "passes: $(ls "$d"/pass_*.csv | wc -l) from $(basename "$d")"
exec "$ROOT/scripts/day1.sh" edit "$r" raw="$d/raw.csv" drives="$P"
