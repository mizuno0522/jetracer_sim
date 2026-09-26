#!/bin/bash
# ホストの環境を一気に作る (Jetson / sim PC 共通)。何度実行しても同じ状態になる (冪等)。
#
#   sudo ./scripts/setup_host.sh jetson            # Jetson Orin Nano: 192.168.10.3・chrony client
#   sudo ./scripts/setup_host.sh pc                # sim PC:          192.168.10.2・chrony server
#   sudo ./scripts/setup_host.sh pc --nic enp5s0   # 有線 NIC を明示 (省略時は最初の有線を自動検出)
#   sudo ./scripts/setup_host.sh jetson --no-net   # apt と sysctl だけ (固定 IP・chrony を触らない)
#
# やること (docs/setup.md Step 0〜1 に相当):
#   1. apt: ROS 2 Humble の apt ソース (無ければ)・ros-humble-ros-base・rmw_cyclonedds_cpp・cv_bridge・rosbag2 (mcap)・
#      colcon・rosdep・numpy/scipy/yaml/opencv・chrony・(pc のみ) python3-vcstool
#   2. sysctl: net.core.rmem_max / rmem_default = 16 MB (画像の取りこぼし対策)
#   3. 固定 IP: 有線 NIC に 192.168.10.{2|3}/24、ゲートウェイ・DNS 無し (インターネットは無線のまま)
#   4. chrony: pc = server (allow 192.168.10.0/24)、jetson = client (server 192.168.10.2)。ufw が有効なら直結サブネットを許可
#   5. 呼び出したユーザーの ~/cyclonedds.xml (NIC 名入り) と ~/.bashrc の環境変数 (無ければ追記)
# やらないこと: Docker・Unity・NVIDIA Container Toolkit (docs/docker.md・docs/unity.md)、ワークスペースのビルド (scripts/build.sh)。
set -euo pipefail

ROLE="${1:-}"
shift || true
NIC=""
DO_NET=1
while [ $# -gt 0 ]; do
  case "$1" in
    --nic) NIC="$2"; shift 2 ;;
    --no-net) DO_NET=0; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
case "$ROLE" in
  jetson) IP=192.168.10.3; PEER=192.168.10.2 ;;
  pc)     IP=192.168.10.2; PEER=192.168.10.3 ;;
  *) echo "usage: sudo $0 {jetson|pc} [--nic <if>] [--no-net]" >&2; exit 2 ;;
esac
[ "$(id -u)" = 0 ] || { echo "sudo で実行すること" >&2; exit 2; }
USER_NAME="${SUDO_USER:-$USER}"
USER_HOME=$(getent passwd "$USER_NAME" | cut -d: -f6)
HERE="$(cd "$(dirname "$0")" && pwd)"
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
[ "$CODENAME" = jammy ] || { echo "★ Ubuntu 22.04 (jammy) ではない ($CODENAME)。Humble が揃わない" >&2; exit 1; }
echo "== setup_host: role=$ROLE ip=$IP user=$USER_NAME home=$USER_HOME"

# ---- 1. apt ---------------------------------------------------------------
if [ ! -f /etc/apt/sources.list.d/ros2.sources ] && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  echo "== ROS 2 apt ソースを登録"
  apt-get update -qq
  apt-get install -y -qq software-properties-common curl >/dev/null
  add-apt-repository -y universe >/dev/null
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}')
  curl -sL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${CODENAME}_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb >/dev/null
fi
echo "== apt install"
apt-get update -qq
PKGS="ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-robot-state-publisher ros-humble-cv-bridge \
      ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap python3-colcon-common-extensions python3-rosdep \
      python3-numpy python3-scipy python3-yaml python3-opencv chrony ethtool"
[ "$ROLE" = pc ] && PKGS="$PKGS python3-vcstool"
apt-get install -y -qq $PKGS >/dev/null
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null 2>&1 || true

# ---- 2. sysctl ------------------------------------------------------------
echo "== sysctl (rmem 16 MB)"
cat > /etc/sysctl.d/60-cyclonedds.conf <<'SYSCTL'
net.core.rmem_max=16777216
net.core.rmem_default=16777216
SYSCTL
sysctl --system >/dev/null

# ---- 3. 固定 IP / 4. chrony ----------------------------------------------
if [ "$DO_NET" = 1 ]; then
  if [ -z "$NIC" ]; then
    NIC=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2=="ethernet"{print $1; exit}')
  fi
  [ -n "$NIC" ] || { echo "★ 有線 NIC が見つからない (--nic で指定)" >&2; exit 1; }
  CON="jetracer-link"
  echo "== 固定 IP: $NIC → $IP/24 (接続名 $CON。ゲートウェイ・DNS 無し)"
  if ! nmcli -t -f NAME con show | grep -qx "$CON"; then
    nmcli con add type ethernet ifname "$NIC" con-name "$CON" >/dev/null
  fi
  nmcli con mod "$CON" ipv4.method manual ipv4.addresses "$IP/24" ipv4.gateway "" ipv4.dns "" \
        ipv4.never-default yes ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 10
  nmcli con up "$CON" >/dev/null 2>&1 || echo "  (リンクが無いので up は保留。ケーブルを挿せば自動で上がる)"

  echo "== chrony ($ROLE)"
  CONF=/etc/chrony/chrony.conf
  sed -i '/^# jetracer_sim/,/^# \/jetracer_sim/d' "$CONF"
  if [ "$ROLE" = pc ]; then
    cat >> "$CONF" <<CHRONY
# jetracer_sim (setup_host.sh)
allow 192.168.10.0/24
local stratum 10
# /jetracer_sim
CHRONY
  else
    sed -i 's/^pool /#pool /; s/^server /#server /' "$CONF"
    cat >> "$CONF" <<CHRONY
# jetracer_sim (setup_host.sh)
server $PEER iburst prefer
# /jetracer_sim
CHRONY
  fi
  systemctl enable --now chrony >/dev/null 2>&1 || systemctl enable --now chronyd >/dev/null 2>&1
  systemctl restart chrony 2>/dev/null || systemctl restart chronyd

  # ufw が有効だと NTP (UDP 123) と DDS のユニキャストが落ちる。直結サブネットだけ丸ごと許可する
  if command -v ufw >/dev/null && ufw status 2>/dev/null | grep -q "^Status: active"; then
    echo "== ufw: 192.168.10.0/24 からの受信を許可 (NTP・DDS)"
    ufw allow from 192.168.10.0/24 >/dev/null
  fi
fi

# ---- 5. ユーザー側: cyclonedds.xml と .bashrc --------------------------------
# 2 枚作る: 有線限定 (直結用) と、インターフェース指定なし (1 台用。有線がリンク DOWN だと限定版は探索に失敗する)。
# どちらを使うかは scripts/sim_env.sh が有線のリンク状態 (carrier) で自動で選ぶ。
echo "== $USER_HOME/cyclonedds-wired.xml (NIC ${NIC:-未設定}) / cyclonedds-local.xml"
sed "s/name=\"eth0\"/name=\"${NIC:-eth0}\"/" "$HERE/../config/cyclonedds.xml" > "$USER_HOME/cyclonedds-wired.xml"
sed '/<Interfaces>/d' "$HERE/../config/cyclonedds.xml" > "$USER_HOME/cyclonedds-local.xml"
cp "$USER_HOME/cyclonedds-local.xml" "$USER_HOME/cyclonedds.xml"      # .bashrc の既定 (sim_env.sh が上書き選択)
echo "${NIC:-}" > "$USER_HOME/.jetracer_wired_nic"
chown "$USER_NAME:" "$USER_HOME/cyclonedds-wired.xml" "$USER_HOME/cyclonedds-local.xml" "$USER_HOME/cyclonedds.xml" "$USER_HOME/.jetracer_wired_nic"
if ! grep -q "jetracer_sim env" "$USER_HOME/.bashrc"; then
  cat >> "$USER_HOME/.bashrc" <<'BASHRC'
# jetracer_sim env (setup_host.sh)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
export PYTHONNOUSERSITE=1
BASHRC
  echo "== ~/.bashrc に環境変数を追記 (新しい端末から有効)"
fi

echo
echo "== 完了。確認:"
echo "   source ~/.bashrc && ros2 doctor --report | grep -iE 'rmw|domain'     # rmw_cyclonedds_cpp / 42"
echo "   sysctl net.core.rmem_max                                            # 16777216"
[ "$DO_NET" = 1 ] && echo "   ping -c 2 $PEER && chronyc tracking | grep -E 'Reference|System time'   # 相手が居れば"
echo "   ./scripts/p9_check.sh                                               # 2 ホスト直結の起動前チェック"
