#!/usr/bin/env bash
# Cones moved, the rest of the course did not: from a short remote drive (start mark -> through the new cones -> out of
# the cone box), replace only the cone stretch of rec105326_lap1 / lap2 and of the map. Robot not needed (bag replay,
# isolated: loopback + own domains 83/84). ~5-10 min.
#   scripts/cone_splice.sh <bag dir> [pass=N]   e.g. ~/bags/gosu_20261120_..._cones2          (alias dsplice)
# Several passes in one recording (back to the start mark, through again): all of them go into the map, the route takes
# pass=N (1 = the first). Run again with another pass=N: the replays already done are kept (logs/cone_splice/<bag>).
# Afterwards: deditin to look (s saves), dself for the fake-robot check, then real runs.
set -uo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
set +u; source /opt/ros/jazzy/setup.bash; source "$ROOT/install/setup.bash" 2>/dev/null; set -u   # rosbag2_py (bag readers)
BAG=$(readlink -f "${1:?bag dir}"); NAME=$(basename "$BAG"); PASS=${2:-pass=1}
W=$ROOT/logs/cone_splice/$NAME; mkdir -p "$W"
RAW=$ROOT/maps/coshow_1119/routes/splice_$NAME/raw.csv
NEWMAP=$ROOT/maps/cones_$NAME
if [ -s "$W/lio.csv" ]; then echo "1/5 FAST-LIO replay: kept from before"; else
  echo "1/5 FAST-LIO on the drive (real speed, isolated)"
  bash "$ROOT/tests/fastlio_replay.sh" "$BAG" "$W/lio.csv" domain=83 > "$W/replay.log" 2>&1 || { tail -5 "$W/replay.log"; exit 1; }
fi
if [ -s "$RAW" ]; then echo "2/5 drive in the track frame: kept from before"; else
  echo "2/5 drive in the track frame (walls)"
  python3 "$ROOT/scripts/route_from_bag.py" "$BAG" "$W/lio.csv" "$ROOT/maps/coshow_1119" "splice_$NAME" > "$W/route.log" 2>&1 \
    || { tail -5 "$W/route.log"; exit 1; }
  tail -3 "$W/route.log"
fi
if [ -f "$NEWMAP/map_clean.npz" ]; then echo "3/5 map of the drive: kept from before"; else
  echo "3/5 map of the drive (real speed, isolated)"
  bash "$ROOT/scripts/map_from_bag.sh" "$BAG" "cones_$NAME" 84 > "$W/map.log" 2>&1 || { tail -5 "$W/map.log"; exit 1; }
  python3 "$ROOT/scripts/map_build.py" --clean "$NEWMAP" >> "$W/map.log" 2>&1
fi
echo "4/5 passes"
python3 "$ROOT/scripts/cone_splice.py" passes "$RAW" "$NEWMAP"
echo "5/5 routes: cone stretch <- $PASS; map: cone box replaced (old map kept once as maps/coshow_1119_cones0)"
python3 "$ROOT/scripts/cone_splice.py" route "$RAW" "$PASS" || exit 1
[ -f "$W/map_patched" ] || { python3 "$ROOT/scripts/cone_splice.py" map "$NEWMAP" && touch "$W/map_patched" || echo "★ map not patched (routes are done)"; }
echo "■ done — look: deditin · check: dself"
