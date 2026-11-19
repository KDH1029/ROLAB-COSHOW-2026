#!/usr/bin/env bash
# Cones moved, the rest of the course did not: from a short remote drive (start mark -> through the new cones -> out of
# the cone box), replace only the cone stretch of rec105326_lap1 / lap2 and of the map. Robot not needed (bag replay,
# isolated: loopback + own domains 83/84). ~5-10 min.
#   scripts/cone_splice.sh <bag dir>        e.g. ~/bags/gosu_20261120_..._cones2          (alias dsplice)
# Afterwards: deditin to look (s saves), dself for the fake-robot check, then real runs.
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
BAG=$(readlink -f "${1:?bag dir}"); NAME=$(basename "$BAG")
W=$ROOT/logs/cone_splice/$NAME; mkdir -p "$W"
echo "1/4 FAST-LIO on the drive (real speed, isolated)"
bash "$ROOT/tests/fastlio_replay.sh" "$BAG" "$W/lio.csv" domain=83 > "$W/replay.log" 2>&1 || { tail -5 "$W/replay.log"; exit 1; }
echo "2/4 drive in the track frame (walls)"
python3 "$ROOT/scripts/route_from_bag.py" "$BAG" "$W/lio.csv" "$ROOT/maps/coshow_1119" "splice_$NAME" > "$W/route.log" 2>&1 \
  || { tail -5 "$W/route.log"; exit 1; }
tail -3 "$W/route.log"
echo "3/4 routes: cone stretch replaced"
python3 "$ROOT/scripts/cone_splice.py" route "$ROOT/maps/coshow_1119/routes/splice_$NAME/raw.csv" || exit 1
echo "4/4 map: cone box replaced (old map kept as maps/coshow_1119_cones0)"
bash "$ROOT/scripts/map_from_bag.sh" "$BAG" "cones_$NAME" 84 > "$W/map.log" 2>&1 || { tail -5 "$W/map.log"; echo "★ map not patched (routes are done)"; exit 1; }
python3 "$ROOT/scripts/map_build.py" --clean "$ROOT/maps/cones_$NAME" >> "$W/map.log" 2>&1
python3 "$ROOT/scripts/cone_splice.py" map "$ROOT/maps/cones_$NAME" || echo "★ map not patched (routes are done)"
echo "■ done — look: deditin · check: dself"
