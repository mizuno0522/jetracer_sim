#!/bin/bash
# ビルド済みの Unity プレイヤーを GitHub Release から取ってきて配置し、course.json を今の profile で書き直す。
# Unity Editor が無くても sim を Unity 描画で動かせる。
#   ./scripts/get_unity_player.sh              # 既定の版 (下の VERSION)
#   ./scripts/get_unity_player.sh v0.1.0
# 出力: $JETRACER_UNITY_PLAYER (既定 ~/jetracer/unity/player)/MinicarSim.x86_64
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VERSION="${1:-v0.1.0}"
DST="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}"
NAME="jetracer_sim_player_${VERSION}_linux_x86_64.tar.gz"
URL="https://github.com/mizuno0522/jetracer_sim/releases/download/${VERSION}/${NAME}"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
if [ -n "${PLAYER_TARBALL:-}" ]; then           # 手元の tar を使う (Release に上げる前の確認用)
  cp "$PLAYER_TARBALL" "$TMP/$NAME"
else
  echo "download $URL"
  curl -fL --progress-bar -o "$TMP/$NAME" "$URL"
fi
tar -C "$TMP" -xzf "$TMP/$NAME"
mkdir -p "$(dirname "$DST")"
rm -rf "${DST:?}"
mv "$TMP/jetracer_sim_player" "$DST"
# 数値 (カメラ・コース・見た目) は course.json に入っているので、リポジトリの profile で書き直す
"$HERE/export_course.sh" "${PROFILE:-jetracer_tt02}" >/dev/null
cp "$HERE/../unity/course.json" "$DST/MinicarSim_Data/StreamingAssets/course.json"
echo "完了: $DST/MinicarSim.x86_64"
echo "起動: ros2 launch minicar_sim sim_host.launch.py unity_player:=$DST/MinicarSim.x86_64"
