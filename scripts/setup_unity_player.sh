#!/bin/bash
# ビルド済み Unity プレイヤーをコピーし、course.json を JetRacer 用 (vehicle_profile のカメラ) に差し替える。
# 再ビルド不要 (プレイヤーは Build/MinicarSim_Data/StreamingAssets/course.json を実行時に読む)。
# 既存プロジェクトの course.json は触らない (docs/unity.md)。
#   ./scripts/setup_unity_player.sh                       # 既存 sim のビルドから (~/minicarbattle2026/unity/MinicarSim/Build)
#   ./scripts/setup_unity_player.sh <Build ディレクトリ>   # JetRacer 用に複製したプロジェクトのビルドから
# 出力: $JETRACER_UNITY_PLAYER (既定 ~/jetracer/unity/player)/MinicarSim.x86_64
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="${1:-$HOME/minicarbattle2026/unity/MinicarSim/Build}"
DST="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}"
PROFILE="${PROFILE:-jetracer_tt02}"
[ -x "$SRC/MinicarSim.x86_64" ] || { echo "プレイヤーが無い: $SRC (Unity でビルドしてから)" >&2; exit 1; }
mkdir -p "$DST"
rsync -a --delete "$SRC/" "$DST/"
"$HERE/export_course.sh" "$PROFILE" >/dev/null
cp "$HERE/../unity/course.json" "$DST/MinicarSim_Data/StreamingAssets/course.json"
python3 - "$DST/MinicarSim_Data/StreamingAssets/course.json" <<'PY'
import json, sys
c = json.load(open(sys.argv[1]))['camera']
print(f"course.json: camera {c['width']}x{c['height']} fov {c['fov_deg']:.0f}° {c['rate_hz']:.0f} Hz")
PY
echo "完了: $DST/MinicarSim.x86_64"
echo "起動: ros2 launch minicar_sim sim_host.launch.py unity_player:=$DST/MinicarSim.x86_64 tcp_port:=10001"
