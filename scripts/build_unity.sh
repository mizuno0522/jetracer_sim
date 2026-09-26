#!/bin/bash
# JetRacer 用 Unity プロジェクト (既定 unity/MinicarSim = 既存 MinicarSim の複製 + JetRacer 向けの変更) を
# バッチモードでビルドし、setup_unity_player.sh でプレイヤーを配置して course.json を書き直す。
#   ./scripts/build_unity.sh            # 初回は Library の生成 (数分)
# 環境: UNITY (Editor のパス), JETRACER_UNITY_PROJ (プロジェクト), JETRACER_UNITY_PLAYER (配置先)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
UNITY="${UNITY:-$HOME/Unity/Hub/Editor/6000.0.83f1/Editor/Unity}"
PROJ="${JETRACER_UNITY_PROJ:-$HERE/../unity/MinicarSim}"
LOG="$PROJ/Logs/build_player.log"
mkdir -p "$PROJ/Logs" "$PROJ/Assets/StreamingAssets"
"$HERE/export_course.sh" jetracer_tt02 >/dev/null
cp "$HERE/../unity/course.json" "$PROJ/Assets/StreamingAssets/course.json"
echo "Unity ビルド中 ($PROJ)... ログ: $LOG"
if "$UNITY" -batchmode -nographics -projectPath "$PROJ" \
     -executeMethod Minicar.EditorTools.MinicarBuild.SetupAndBuild -logFile "$LOG"; then
  echo "ビルド完了: $PROJ/Build/MinicarSim.x86_64"
else
  echo "★ビルド失敗。エラー:" >&2
  grep -E "error CS|\[MinicarBuild\]|Error building|Exception" "$LOG" | head -20 >&2
  exit 1
fi
"$HERE/setup_unity_player.sh" "$PROJ/Build"
