#!/bin/bash
# 1 台で閉ループを回す (Unity 無し・OpenCV 描画・ground truth の教師)。Jetson 単体でも動く。
#   ./scripts/run_sim_local.sh            # Ctrl-C で止める
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$HERE/../ros_ws/install/setup.bash"
ros2 launch minicar_sim sim_host.launch.py camera_backend:=opencv unity_player:=none rviz:=false &
SIM=$!
sleep 5
ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true auto_run:=true &
STK=$!
trap "kill $STK $SIM 2>/dev/null; wait" INT TERM
wait
