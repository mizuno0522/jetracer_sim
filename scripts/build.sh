#!/bin/bash
# ワークスペースをビルドする (PC / Jetson 共通)。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1          # ~/.local の empy 4.x / numpy 2.x を見ない (既存記録の落とし穴)
source /opt/ros/humble/setup.bash
cd "$HERE/../ros_ws"
colcon build --symlink-install "$@"
echo "source $HERE/../ros_ws/install/setup.bash"
