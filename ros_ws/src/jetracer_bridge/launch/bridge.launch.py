#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
実機バックエンド (Jetson): jetracer_bridge ＋ camera_node。推論スタックは vehicle_stack.launch.py。

  ros2 launch jetracer_bridge bridge.launch.py
  ros2 launch jetracer_bridge bridge.launch.py camera:=false     # I2C だけ試すとき
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('jetracer_bridge'), 'config', 'jetracer_bridge.yaml')
    profile = LaunchConfiguration('vehicle_profile')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_profile', default_value='jetracer_tt02'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(package='jetracer_bridge', executable='jetracer_bridge', name='jetracer_bridge', output='screen',
             parameters=[params, {'vehicle_profile_file': profile,
                                  'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
        Node(package='jetracer_bridge', executable='csi_camera_node', name='camera_node', output='screen',
             parameters=[params, {'vehicle_profile_file': profile}],
             condition=IfCondition(LaunchConfiguration('camera'))),
    ])
