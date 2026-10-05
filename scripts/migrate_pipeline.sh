#!/bin/bash
# 描画の仕組みを替えた版のプロジェクト (URP 版 unity/MinicarSimURP・HDRP 版 unity/MinicarSimHDRP) を unity/MinicarSim から作り
# (何度実行してもよい。毎回スクリプトを同期し直す)、パッケージを入れて設定し、プレイヤーをビルドして配置する。
# 元の unity/MinicarSim (Built-in・学習用) は変えない。docs/hdrp.md
#
#   ./scripts/migrate_pipeline.sh urp           # 同期 → 1 パッケージ → 2 設定 → 3 ビルド → 配置 (migrate_urp.sh と同じ)
#   ./scripts/migrate_pipeline.sh hdrp          # (migrate_hdrp.sh と同じ)
#   ./scripts/migrate_pipeline.sh urp --shots   # さらに Built-in とその版を同じ位置で撮って左右に並べる (画面が要る)
#   STEPS="2 3" ./scripts/migrate_pipeline.sh urp   # 途中の段だけやり直す (sync 1 2 3 install の中から)
# 環境: UNITY (Editor)、JETRACER_URP_PROJ / JETRACER_HDRP_PROJ (プロジェクト)、
#       JETRACER_UNITY_PLAYER_URP / JETRACER_UNITY_PLAYER_HDRP (配置先。既定 ~/jetracer/unity/player_urp・player_hdrp)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
UNITY="${UNITY:-$HOME/Unity/Hub/Editor/6000.0.83f1/Editor/Unity}"
SRC="$ROOT/unity/MinicarSim"
PIPE="${1:-}"; shift || true
case "$PIPE" in
  hdrp) NAME=HDRP; DST="${JETRACER_HDRP_PROJ:-$ROOT/unity/MinicarSimHDRP}"; PLAYER="${JETRACER_UNITY_PLAYER_HDRP:-$HOME/jetracer/unity/player_hdrp}"
        PHASE1=Minicar.EditorTools.HdrpMigration.Phase1; PHASE2=Minicar.EditorTools.HdrpSetup.Phase2 ;;
  urp)  NAME=URP; DST="${JETRACER_URP_PROJ:-$ROOT/unity/MinicarSimURP}"; PLAYER="${JETRACER_UNITY_PLAYER_URP:-$HOME/jetracer/unity/player_urp}"
        PHASE1=Minicar.EditorTools.UrpMigration.Phase1; PHASE2=Minicar.EditorTools.UrpSetup.Phase2 ;;
  *) echo "使い方: $0 urp|hdrp [--shots]" >&2; exit 1 ;;
esac
STEPS="${STEPS:-sync 1 2 3 install}"
SHOTS=0; [ "${1:-}" = "--shots" ] && SHOTS=1
[ -x "$UNITY" ] || { echo "★Unity が無い: $UNITY (UNITY=<Editor のパス>)" >&2; exit 1; }
mkdir -p "$DST/Logs"

run_unity() {   # $1 = 段の名前, $2 = メソッド, $3 = 追加の引数
  local log="$DST/Logs/${PIPE}_$1.log"
  echo "== $1: $2 (ログ $log)"
  if ! "$UNITY" -batchmode -nographics -projectPath "$DST" -executeMethod "$2" $3 -logFile "$log"; then
    echo "★$1 失敗。エラー:" >&2
    grep -E "error CS|Error|Exception|\[Hdrp|\[Urp|\[MinicarBuild\]" "$log" | grep -v "^UnityEngine\." | head -30 >&2
    exit 1
  fi
  grep -E "\[HdrpMigration\]|\[HdrpSetup\]|\[UrpMigration\]|\[UrpSetup\]|\[MinicarBuild\]" "$log" | tail -6 || true
}

for step in $STEPS; do
  case "$step" in
  sync)
    echo "== sync: $SRC → $DST"
    # 管理しているもの (Assets の 3 つ・ProjectSettings・Packages/manifest.json) だけを写す。
    # その版が作ったもの (Assets/MinicarHDRP・Assets/MinicarURP・Global Settings など) と Library は消さない
    for d in Minicar Scenes StreamingAssets; do
      mkdir -p "$DST/Assets/$d"
      rsync -a --delete "$SRC/Assets/$d/" "$DST/Assets/$d/"
      cp "$SRC/Assets/$d.meta" "$DST/Assets/$d.meta"
    done
    mkdir -p "$DST/ProjectSettings" "$DST/Packages"
    rsync -a "$SRC/ProjectSettings/" "$DST/ProjectSettings/"
    cp "$SRC/Packages/manifest.json" "$DST/Packages/manifest.json"
    rm -f "$DST/Packages/packages-lock.json"
    ;;
  1) run_unity phase1 "$PHASE1" ;;          # パッケージ + スクリプト定義 (MINICAR_HDRP / MINICAR_URP)
  2) run_unity phase2 "$PHASE2" ;;          # 設定アセット・色空間・Global Settings・実行時に使う材質
  3) "$ROOT/scripts/export_course.sh" jetracer_tt02 >/dev/null
     cp "$ROOT/unity/course.json" "$DST/Assets/StreamingAssets/course.json"
     run_unity build Minicar.EditorTools.MinicarBuild.SetupAndBuild
     grep -iE "global settings|(HDRP|URP).*(missing|not found)" "$DST/Logs/${PIPE}_build.log" | head -5 || true ;;
  install)
     JETRACER_UNITY_PLAYER="$PLAYER" "$HERE/setup_unity_player.sh" "$DST/Build"
     echo "$NAME 版のプレイヤー: $PLAYER/MinicarSim.x86_64" ;;
  *) echo "知らない段: $step" >&2; exit 1 ;;
  esac
done

if [ "$SHOTS" = 1 ]; then
  T=$(date +%Y%m%d_%H%M%S)
  OUT="$ROOT/shots/fuji_rx7_builtin_$T" "$HERE/shots.sh" fuji rx7
  JETRACER_UNITY_PLAYER="$PLAYER" OUT="$ROOT/shots/fuji_rx7_${PIPE}_$T" "$HERE/shots.sh" fuji rx7
  python3 "$ROOT/tools/shot_sheet.py" "$ROOT/shots/fuji_rx7_builtin_$T" "$ROOT/shots/fuji_rx7_${PIPE}_$T" --out "$ROOT/shots/compare_builtin_${PIPE}_$T.png"
fi
