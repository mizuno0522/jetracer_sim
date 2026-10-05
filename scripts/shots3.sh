#!/bin/bash
# 3 つの版 (HDRP・URP・Built-in) を同じ位置・同じ画質で撮り、1 枚に並べて HDRP との色の差を出す (docs/hdrp.md)。
# 絵作りを変えたら、これで 3 版が同じ絵のままかを確かめる。
#   ./scripts/shots3.sh minicar            # → shots/cmp3_minicar_<日時>/sheet.png (左から HDRP・URP・Built-in)
#   ./scripts/shots3.sh fuji rx7
#   QUALITY=low ./scripts/shots3.sh minicar
# 環境: QUALITY (既定 medium)、OUT (出力先)、ほかは scripts/shots.sh と同じ (SHOTS・SHOTVIEWS・SHOTSIZE)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
COURSE="${1:-minicar}"; CAR="${2:-rx7}"
OUT="${OUT:-$HERE/../shots/cmp3_${COURSE}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
if [ "$COURSE" = "fuji" ]; then : "${SHOTS:=0}"; : "${SHOTVIEWS=carfront,carside,panasonic,scenic}"; fi
export QUALITY="${QUALITY:-medium}" BENCH="${BENCH:-0}" SHOTSIZE="${SHOTSIZE:-1280x720}" SHOTS SHOTVIEWS
B="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}"
U="${JETRACER_UNITY_PLAYER_URP:-$HOME/jetracer/unity/player_urp}"
H="${JETRACER_UNITY_PLAYER_HDRP:-$HOME/jetracer/unity/player_hdrp}"
DIRS=()
for pair in "hdrp:$H" "urp:$U" "builtin:$B"; do
  n="${pair%%:*}"; p="${pair#*:}"
  [ -x "$p/MinicarSim.x86_64" ] || { echo "★$n 版のプレイヤーが無い: $p (飛ばす)" >&2; continue; }
  JETRACER_UNITY_PLAYER="$p" OUT="$OUT/$n" "$HERE/shots.sh" "$COURSE" "$CAR" >/dev/null 2>&1 || true
  DIRS+=("$OUT/$n")
done
python3 "$HERE/../tools/shot_compare3.py" "${DIRS[@]}" --out "$OUT/sheet.png"
