#!/bin/bash
# course.py / sim.yaml / vehicle_profile → unity/course.json を書き出し、Unity プロジェクトへコピーする。
# Unity プロジェクト (minicarbattle2026/unity/MinicarSim) へのコピーは UNITY_PROJ を明示したときだけ
# (★既存車両の作業コピーの course.json を黙って上書きしないため)。
#   ./scripts/export_course.sh [profile]                                   # unity/course.json を更新するだけ
#   UNITY_PROJ=~/minicarbattle2026/unity/MinicarSim ./scripts/export_course.sh   # Unity プロジェクトへもコピー
#   COURSE=fuji ./scripts/export_course.sh real_rx7      # サーキット → unity/course_fuji_real_rx7.json
#     (sim_host.launch.py course:=fuji vehicle_profile:=real_rx7 は Unity に -course course_fuji_real_rx7.json を渡す)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PROFILE="${1:-jetracer_tt02}"
COURSE="${COURSE:-minicar}"
UNITY_PROJ="${UNITY_PROJ:-}"
# 参照線 (ミニマップ表示用)。ROUTE=<route.yaml> で指定。未指定なら config/route_${PROFILE}.yaml があればそれ
ROUTE="${ROUTE:-}"
[ -z "$ROUTE" ] && [ -f "$HERE/../ros_ws/src/minicar_sim/config/route_${PROFILE}.yaml" ] && ROUTE="$HERE/../ros_ws/src/minicar_sim/config/route_${PROFILE}.yaml"
ROUTE_ARG=(); [ -n "$ROUTE" ] && ROUTE_ARG=(-r "$ROUTE")
export PYTHONNOUSERSITE=1
export PYTHONPATH="$HERE/../ros_ws/src/jetracer_common:$PYTHONPATH"
OUT_NAME=course.json
if [ "$COURSE" != "minicar" ]; then
  OUT_NAME="course_${COURSE}_$(basename "$PROFILE" .yaml).json"
  ROUTE_ARG=()
fi
python3 "$HERE/../ros_ws/src/minicar_sim/scripts/export_unity_course.py" -p "$PROFILE" -c "$COURSE" -o "$HERE/../unity/$OUT_NAME" "${ROUTE_ARG[@]}"
if [ -n "$UNITY_PROJ" ] && [ -d "$UNITY_PROJ/Assets" ]; then
  mkdir -p "$UNITY_PROJ/Assets/StreamingAssets"
  cp "$HERE/../unity/$OUT_NAME" "$UNITY_PROJ/Assets/StreamingAssets/$OUT_NAME"
  echo "copied → $UNITY_PROJ/Assets/StreamingAssets/$OUT_NAME (次に unity/build_player.sh でビルドし直す。ビルド済みなら Build/*_Data/StreamingAssets へコピーでもよい)"
else
  echo "unity/$OUT_NAME を更新した (Unity プロジェクトへコピーするには UNITY_PROJ=<path> を付ける)"
fi
