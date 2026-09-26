#!/bin/bash
# 1 台で閉ループを回す (Unity 無し・OpenCV 描画・ground truth の教師)。Jetson 単体でも動く。
#   ./scripts/run_sim_local.sh            # Ctrl-C で止める
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$HERE/../ros_ws/install/setup.bash"
# ★ set -m: スクリプト (非対話シェル) のバックグラウンド job は SIGINT を無視する設定を継承する。
#   ジョブ制御を有効にしてから起動しないと、trap の kill -INT が launch に届かず wait が永久に止まる。
set -m
ros2 launch minicar_sim sim_host.launch.py camera_backend:=opencv unity_player:=none rviz:=false &
SIM=$!
sleep 5
ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true auto_run:=true &
STK=$!
set +m
cleanup() {
  kill -INT $STK $SIM 2>/dev/null
  for _ in $(seq 1 15); do kill -0 $SIM 2>/dev/null || kill -0 $STK 2>/dev/null || break; sleep 1; done
  "$HERE/stop_sim.sh" >/dev/null 2>&1 || true
}
trap cleanup INT TERM EXIT
wait
