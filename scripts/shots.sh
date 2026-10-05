#!/bin/bash
# Unity プレイヤーを撮影・計測モードで起動し (ROS 不要)、撮った画像を 1 枚に並べる。
#   ./scripts/shots.sh fuji rx7            # 富士・自車 RX-7 (ライバル 787B・ND)。出力 shots/fuji_rx7_<日時>/
#   ./scripts/shots.sh minicar             # ミニカーの会場
#   QUALITY=high SHOTS="0,1250" BENCH=0 ./scripts/shots.sh fuji nd
# 環境: JETRACER_UNITY_PLAYER (既定 ~/jetracer/unity/player)、SHOTS (既定 0,1250,2600,3300 / ミニカーは 0,8,15,22)、
#       SHOTVIEWS (富士の既定 grandstand,panasonic,scenic,carfront,carside,carrear。空で撮らない)、
#       SHOTSIZE (1920x1080)、BENCH (600 フレーム。0 で計らない)、LAYOUT (aic|chase|rviz。計測時の画面)、QUALITY (low|medium|high。既定 low)
#       JETRACER_TCP_PORT (既定 10001。ROS には繋がない前提なので、誰も待っていないポートを渡す。プレイヤーの既定 10000 は
#       minicarbattle2026 の sim が使っていて、繋がると車がそちらの座標へ飛び、相手の検証にも割り込む)
# 画面が要る (-batchmode / -nographics では描けない)。リモートなら VNC か DISPLAY を用意する。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
COURSE="${1:-fuji}"
CAR="${2:-rx7}"
PLAYER="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}/MinicarSim.x86_64"
[ -x "$PLAYER" ] || { echo "★プレイヤーが無い: $PLAYER (scripts/build_unity.sh)" >&2; exit 1; }
case "$CAR" in rx7) PROFILE=real_rx7; R1=b787; R2=nd;; nd) PROFILE=real_nd; R1=rx7; R2=b787;; b787) PROFILE=real_b787; R1=rx7; R2=nd;; *) PROFILE=real_rx7; R1=b787; R2=nd;; esac
if [ "$COURSE" = "fuji" ]; then
  JSON="course_fuji_${PROFILE}.json"; DEF_SHOTS="0,1250,2600,3300"; DEF_VIEWS="grandstand,panasonic,scenic,carfront,carside,carrear"
else
  JSON="course.json"; DEF_SHOTS="0,8,15,22"; DEF_VIEWS=""
fi
OUT="${OUT:-$HERE/../shots/${COURSE}_${CAR}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
export DRI_PRIME="${DRI_PRIME:-1}"   # 内蔵 + 単体 GPU の PC で単体 GPU を使う (1 枚だけの PC では無視される)
echo "撮影: $JSON  自車 $CAR  → $OUT"
"$PLAYER" -course "$JSON" -owncar "$CAR" -rivalcar "$R1" -rival2car "$R2" -sound off -rosport "${JETRACER_TCP_PORT:-10001}" \
  -layout "${LAYOUT:-aic}" -quality "${QUALITY:-low}" \
  -shots "${SHOTS:-$DEF_SHOTS}" -shotdir "$OUT" -shotsize "${SHOTSIZE:-1920x1080}" -shotviews "${SHOTVIEWS-$DEF_VIEWS}" -bench "${BENCH:-600}" \
  -screen-width 1920 -screen-height 1080 -logFile "$OUT/player.log" || true
grep -E "\[Shots\]|\[SimBridge\] course built|\[CourseBuilder\]" "$OUT/player.log" | tail -8 || true
python3 "$HERE/../tools/shot_sheet.py" "$OUT"
[ -f "$OUT/bench.md" ] && cat "$OUT/bench.md"
