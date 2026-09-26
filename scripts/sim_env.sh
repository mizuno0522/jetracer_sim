# 2 ホスト構成 (sim PC ↔ Jetson) の共通環境。両方の全端末で source する。
#   source scripts/sim_env.sh
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../ros_ws/install/setup.bash" 2>/dev/null
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# DDS 設定: 有線 (setup_host.sh が ~/.jetracer_wired_nic に記録) がリンク UP なら有線限定、そうでなければ指定なし (1 台用)。
# 有線限定のままケーブルを抜くと探索が全部失敗するので、ここで毎回選び直す。
_NIC=$(cat "$HOME/.jetracer_wired_nic" 2>/dev/null)
if [ -n "$_NIC" ] && [ "$(cat /sys/class/net/$_NIC/carrier 2>/dev/null)" = "1" ] && [ -f "$HOME/cyclonedds-wired.xml" ]; then
  export CYCLONEDDS_URI=file://$HOME/cyclonedds-wired.xml
elif [ -f "$HOME/cyclonedds-local.xml" ]; then
  export CYCLONEDDS_URI=file://$HOME/cyclonedds-local.xml
else
  export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://$HOME/cyclonedds.xml}
fi
echo "jetracer_sim env: ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION CYCLONEDDS_URI=$CYCLONEDDS_URI"
