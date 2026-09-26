# 環境構築 — 2 ホスト (sim PC ＋ Jetson Orin Nano)

| ホスト | OS / ROS | 役割 | IP (有線直結・ゲートウェイ無し) | chrony |
|---|---|---|---|---|
| sim PC | Ubuntu 22.04.5・AMD Ryzen・**AMD GPU (CUDA 無し)** / Humble (desktop 導入済み・`rmw_cyclonedds_cpp` は apt で追加) | Unity (素) ＋ Docker (物理・imu_sim・endpoint)。学習器は別 PC | 192.168.10.2 (有線 enp5s0) | server |
| Jetson Orin Nano | JetPack 6.2.1 (L4T 36.x・Ubuntu 22.04) / Humble | 推論・ブリッジ・カメラ | 192.168.10.3 | client |

各ステップの最後の「確認」が通らないうちは次へ進まない。**P9 (Jetson ↔ PC の実接続) はまだ誰も試していない**ので、Step 5 を最初に通す半日を取ること。

## Step 0 — 全ホスト: ROS 2 Humble

```bash
lsb_release -cs                    # jammy
sudo apt update && sudo apt install -y software-properties-common curl && sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb && sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-robot-state-publisher \
                    ros-humble-cv-bridge ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap \
                    python3-colcon-common-extensions python3-rosdep python3-vcstool python3-numpy python3-scipy python3-yaml python3-opencv
sudo rosdep init 2>/dev/null; rosdep update
```

環境 (全ホスト同じ値。`scripts/sim_env.sh` にまとめてある):

```bash
cp config/cyclonedds.xml ~/cyclonedds.xml     # NetworkInterface name を ip link の有線名 (enp3s0 等) に書き換える
cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
export PYTHONNOUSERSITE=1        # ~/.local の empy 4.x / numpy 2.x 混入対策
