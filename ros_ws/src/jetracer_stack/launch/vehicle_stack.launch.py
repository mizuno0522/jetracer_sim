#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson 側 (推論スタック)。sim 接続でも実機でも同じ。

  ros2 launch jetracer_stack vehicle_stack.launch.py                    # policy_net (model_file が要る)
  ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true      # sim: ground truth の教師で走らせる
  ros2 topic pub --once /run std_msgs/Bool "data: true"

★ 推論スタックは /sim/ を購読してはいけない (teacher:=true は sim 専用の例外。scripts/check_no_sim_topics.sh)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('jetracer_stack')
    params = os.path.join(pkg, 'config', 'stack.yaml')
    sim_pkg = get_package_share_directory('minicar_sim')
    imu_yaml = os.path.join(sim_pkg, 'config', 'imu_sim.yaml')
    profile = LaunchConfiguration('vehicle_profile')
    teacher = LaunchConfiguration('teacher')
    model = LaunchConfiguration('model_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = ParameterValue(Command([FindExecutable(name='python3'), ' -m jetracer_stack.urdf_gen --profile ',
                                   profile, ' --imu ', imu_yaml]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_profile', default_value='jetracer_tt02'),
        DeclareLaunchArgument('teacher', default_value='false',
                              description='true=gt_teacher (sim の ground truth) で /lookahead を出す'),
        DeclareLaunchArgument('model_file', default_value='', description='policy_net の ONNX'),
        DeclareLaunchArgument('auto_run', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='lockstep のとき true'),
        DeclareLaunchArgument('failsafe', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             output='screen', parameters=[{'robot_description': urdf,
                                           'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}]),
        Node(package='jetracer_stack', executable='cmd_shaper', name='cmd_shaper', output='screen',
             parameters=[params, {'vehicle_profile_file': profile,
                                  'auto_run': ParameterValue(LaunchConfiguration('auto_run'), value_type=bool),
                                  'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}]),
        Node(package='jetracer_stack', executable='failsafe', name='failsafe', output='screen',
             parameters=[params, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('failsafe'))),
        Node(package='jetracer_stack', executable='gt_teacher', name='gt_teacher', output='screen',
             parameters=[params, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
             condition=IfCondition(teacher)),
        Node(package='jetracer_stack', executable='policy_net', name='policy_net', output='screen',
             parameters=[params, {'model_file': model,
                                  'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
             condition=UnlessCondition(teacher)),
    ])
