# 学習器 (PyTorch ＋ ROS 2 Humble)。rosbag を読んで学習し ONNX を書き出す。GPU は --gpus all。
FROM pytorch/pytorch:2.4.0-cuda12.1-cudnn9-runtime
ENV DEBIAN_FRONTEND=noninteractive TZ=Asia/Tokyo
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release locales \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap \
       ros-humble-cv-bridge python3-colcon-common-extensions python3-opencv \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir onnx onnxruntime-gpu pyyaml scipy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 PYTHONNOUSERSITE=1
WORKDIR /ws
COPY ros_ws/src/minicar_msgs /ws/src/minicar_msgs
COPY ros_ws/src/minicar_sim_msgs /ws/src/minicar_sim_msgs
COPY ros_ws/src/jetracer_common /ws/src/jetracer_common
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
RUN echo "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["bash"]
