#!/bin/bash
# sim PC 用: Unity 中継 (ros_tcp_endpoint) を ros_ws/src に取り込む。Jetson では実行しない (要らない)。
#   ./scripts/setup_ws.sh                                     # GitHub から vcs import
#   ./scripts/setup_ws.sh ~/ros2_unity_ws/src/ROS-TCP-Endpoint   # 手元の checkout を symlink
# そのあと ./scripts/build.sh。ros_tcp_endpoint が無くても build.sh は通る (無いと camera_backend:=unity だけ使えない)。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../ros_ws/src"
if [ -n "$1" ]; then
  ln -sfn "$(realpath "$1")" "$SRC/ros_tcp_endpoint"
elif [ ! -e "$SRC/ros_tcp_endpoint" ]; then
  command -v vcs >/dev/null || { echo "vcstool が無い: sudo apt install python3-vcstool" >&2; exit 1; }
  vcs import "$SRC" < "$HERE/../ros_ws/deps.repos"
fi
ls -ld "$SRC/ros_tcp_endpoint"
echo "次: ./scripts/build.sh"
