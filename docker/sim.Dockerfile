# sim PC の ROS 側 (物理・imu_sim・camera_info・ros_tcp_endpoint) を 1 コンテナで。Unity は素で動かす。
# 既存記録の empy 4.x / numpy 2.x の混入問題を Docker の壁で消す。
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp ros-humble-robot-state-publisher ros-humble-cv-bridge \
      ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap \
      python3-pip python3-numpy python3-scipy python3-yaml python3-opencv git \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 \
    CYCLONEDDS_URI=file:///cfg/cyclonedds.xml PYTHONNOUSERSITE=1
WORKDIR /ws
# ROS-TCP-Endpoint (Unity との TCP 中継。DDS ではない)
RUN git clone -b main-ros2 --depth 1 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git /ws/src/ros_tcp_endpoint
COPY ros_ws/src/ /ws/src/
RUN . /opt/ros/humble/setup.sh && rosdep install --from-paths src -y --ignore-src -r || true
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
RUN echo "source /ws/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["source /ws/install/setup.bash && ros2 launch minicar_sim sim_host.launch.py unity_player:=none sim_mode:=${SIM_MODE:-realtime} camera_backend:=${CAMERA_BACKEND:-unity}"]
