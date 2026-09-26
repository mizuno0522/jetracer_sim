#!/bin/bash
# course.py / sim.yaml / vehicle_profile → unity/course.json を書き出し、Unity プロジェクトへコピーする。
# Unity プロジェクト (minicarbattle2026/unity/MinicarSim) へのコピーは UNITY_PROJ を明示したときだけ
# (★既存車両の作業コピーの course.json を黙って上書きしないため)。
#   ./scripts/export_course.sh [profile]                                   # unity/course.json を更新するだけ
#   UNITY_PROJ=~/minicarbattle2026/unity/MinicarSim ./scripts/export_course.sh   # Unity プロジェクトへもコピー
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PROFILE="${1:-jetracer_tt02}"
UNITY_PROJ="${UNITY_PROJ:-}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="$HERE/../ros_ws/src/jetracer_common:$PYTHONPATH"
python3 "$HERE/../ros_ws/src/minicar_sim/scripts/export_unity_course.py" -p "$PROFILE" -o "$HERE/../unity/course.json"
if [ -n "$UNITY_PROJ" ] && [ -d "$UNITY_PROJ/Assets" ]; then
  mkdir -p "$UNITY_PROJ/Assets/StreamingAssets"
  cp "$HERE/../unity/course.json" "$UNITY_PROJ/Assets/StreamingAssets/course.json"
  echo "copied → $UNITY_PROJ/Assets/StreamingAssets/course.json (次に unity/build_player.sh でビルドし直す)"
else
  echo "unity/course.json を更新した (Unity プロジェクトへコピーするには UNITY_PROJ=<path> を付ける)"
fi
