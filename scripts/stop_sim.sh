#!/bin/bash
# jetracer_sim の起動物 (launch・その子ノード・Unity プレイヤー) を全部止める。
# 既存 sim (~/minicarbattle2026) には触らない: パターンは実行ファイルのパスで先頭固定 (^) にする。
# ★ pkill -f を緩い文字列で使うと、その文字列を含む別のシェル (自分の端末) まで殺す。
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../ros_ws" && pwd)"
PLAYER="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}"
launches=$(pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch (minicar_sim|jetracer_stack|jetracer_bridge) ")
for pid in $launches; do kill -INT "$pid" 2>/dev/null; done
[ -n "$launches" ] && sleep 4
for pid in $launches; do pkill -9 -P "$pid" 2>/dev/null; kill -9 "$pid" 2>/dev/null; done
pkill -9 -f "^(/usr/bin/)?python3 $WS/install/" 2>/dev/null
pkill -9 -f "^$PLAYER/MinicarSim" 2>/dev/null
sleep 0.5
echo "残り $(pgrep -f "^(/usr/bin/)?python3 ($WS/install/|/opt/ros/humble/bin/ros2 launch (minicar_sim|jetracer_stack|jetracer_bridge) )" | wc -l) プロセス"
