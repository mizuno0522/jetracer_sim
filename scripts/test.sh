#!/bin/bash
# 走行ゼロで落とせる枝: 単体テスト (ROS 不要) と /sim/ 購読禁止の不変量。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1
cd "$HERE/../ros_ws/src/jetracer_common" && python3 -m pytest -q test
echo "unit tests OK"
