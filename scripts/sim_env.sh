# 2 ホスト構成 (sim PC ↔ Jetson) の共通環境。両方の全端末で source する。
#   source scripts/sim_env.sh
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../ros_ws/install/setup.bash" 2>/dev/null
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$HOME/cyclonedds.xml}
echo "jetracer_sim env: ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION CYCLONEDDS_URI=$CYCLONEDDS_URI"
