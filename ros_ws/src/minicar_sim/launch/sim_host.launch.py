#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim PC 側 (実車に無いもの) を起動する。JetRacer sim 版。

  vehicle_sim      物理 100 Hz (vehicle_profile で TT-02 4WD)。/sim/body_state・/sim/ground_truth
  imu_sim          ★新規。物理状態から /imu (100 Hz) を合成 (指令からは作らない)
  camera_info_pub  /camera/camera_info を画像と同じ stamp で
  ros_tcp_endpoint + Unity プレイヤー (camera_backend:=unity)。opencv なら vehicle_sim が描く
  sim_viz / rviz2  任意

  ros2 launch minicar_sim sim_host.launch.py                          # Unity で描く (既定)
  ros2 launch minicar_sim sim_host.launch.py camera_backend:=opencv   # Unity 無しで動かす (Jetson 単体試験)
  ros2 launch minicar_sim sim_host.launch.py sim_mode:=lockstep       # 強化学習用 (/sim/step)

Jetson 側は jetracer_stack/vehicle_stack.launch.py。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_pkg = get_package_share_directory('minicar_sim')
    sim_params = os.path.join(sim_pkg, 'config', 'sim.yaml')
    imu_params = os.path.join(sim_pkg, 'config', 'imu_sim.yaml')
    rviz_dir = os.path.join(sim_pkg, 'rviz')

    profile = LaunchConfiguration('vehicle_profile')
    sim_mode = LaunchConfiguration('sim_mode')
    camera_backend = LaunchConfiguration('camera_backend')
    unity_player = LaunchConfiguration('unity_player')
    use_rviz = LaunchConfiguration('rviz')
    use_viz = LaunchConfiguration('viz')
    use_shortcut = LaunchConfiguration('use_shortcut')
    narrow_divider = LaunchConfiguration('narrow_divider')
    arrow_dir = LaunchConfiguration('arrow_dir')
    seed = LaunchConfiguration('seed')
    route_file = LaunchConfiguration('route_file')
    tcp_port = LaunchConfiguration('tcp_port')
    use_unity = PythonExpression(["'", camera_backend, "' == 'unity'"])
    lockstep = PythonExpression(["'", sim_mode, "' == 'lockstep'"])

    return LaunchDescription([
        DeclareLaunchArgument('vehicle_profile', default_value='jetracer_tt02',
                              description='config/vehicle_profile/<name>.yaml (m05 で旧車両)'),
        DeclareLaunchArgument('sim_mode', default_value='realtime',
                              description='realtime (壁時計) / lockstep (/sim/step で進める・/clock)'),
        DeclareLaunchArgument('camera_backend', default_value='unity',
                              description='unity=Unity プレイヤーが描く / opencv=vehicle_sim が射影描画'),
        DeclareLaunchArgument('unity_player',
                              default_value=os.path.expanduser('~/minicarbattle2026/unity/MinicarSim/Build/MinicarSim.x86_64'),
                              description='Unity プレイヤー。none で起動しない (手動起動やエディタ Play のとき)'),
        DeclareLaunchArgument('record', default_value='',
                              description='Unity の画面を録画する mp4 のパス (例 ~/Videos/run.mp4)。空なら録画しない。'
                                          'ffmpeg が要る。camera_backend:=unity のときだけ効く'),
        DeclareLaunchArgument('record_fps', default_value='30'),
        DeclareLaunchArgument('window_width', default_value='1024',
                              description='Unity のウィンドウ幅。録画サイズは min(record_width, ウィンドウ幅) なので大きく録るならここも上げる'),
        DeclareLaunchArgument('window_height', default_value='768'),
        DeclareLaunchArgument('record_width', default_value='1280',
                              description='録画の幅 [px]。フル解像度のままだと符号化が重く、画像配信のレートが落ちる'),
        DeclareLaunchArgument('tcp_port', default_value='10000',
                              description='ros_tcp_endpoint のポート。同じ PC で既存 sim (M-05・10000) も動かすなら 10001 等に'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value='sim.rviz'),
        DeclareLaunchArgument('viz', default_value='false'),
        DeclareLaunchArgument('use_shortcut', default_value='true'),
        DeclareLaunchArgument('narrow_divider', default_value='true'),
        DeclareLaunchArgument('arrow_dir', default_value='center'),
        DeclareLaunchArgument('seed', default_value='0', description='エピソードの seed (imu_sim の乱択化)'),
        DeclareLaunchArgument('route_file', default_value='route_jetracer_tt02.yaml',
                              description='参照線 (tools/make_route.py の yaml。名前だけなら config/ から)。'
                                          '空ならコース中心線 (TT-02 では R_min を割る区間がある)'),
        DeclareLaunchArgument('lookahead_m', default_value='0.5',
                              description='先行注視点の基本距離 [m] (ld = lookahead_m + 0.3 v)。教師の追従の締まり'),
        DeclareLaunchArgument('inject_stuck_at_s', default_value='-1.0'),
        DeclareLaunchArgument('start_lateral_m', default_value='0.0'),
        DeclareLaunchArgument('start_offset_m', default_value='0.0'),

        Node(package='minicar_sim', executable='vehicle_sim.py', name='vehicle_sim', output='screen',
             parameters=[sim_params, {
                 'vehicle_profile_file': profile,
                 'sim_mode': sim_mode,
                 'use_camera': True,
                 'camera_backend': camera_backend,
                 'use_shortcut': use_shortcut,
                 'narrow_divider': narrow_divider,
                 'arrow_dir': arrow_dir,
                 'episode_seed': ParameterValue(seed, value_type=int),
                 'route_file': ParameterValue(route_file, value_type=str),
                 'lookahead_m': ParameterValue(LaunchConfiguration('lookahead_m'), value_type=float),
                 'inject_stuck_at_s': ParameterValue(LaunchConfiguration('inject_stuck_at_s'), value_type=float),
                 'start_lateral_m': ParameterValue(LaunchConfiguration('start_lateral_m'), value_type=float),
                 'start_offset_m': ParameterValue(LaunchConfiguration('start_offset_m'), value_type=float),
                 'use_sim_time': ParameterValue(lockstep, value_type=bool),
             }]),

        Node(package='imu_sim', executable='imu_sim_node', name='imu_sim', output='screen',
             parameters=[{'config_file': imu_params,
                          'seed': ParameterValue(seed, value_type=int),
                          'use_sim_time': ParameterValue(lockstep, value_type=bool)}]),

        Node(package='minicar_sim', executable='camera_info_pub.py', name='camera_info_pub', output='screen',
             parameters=[{'vehicle_profile_file': profile,
                          'use_sim_time': ParameterValue(lockstep, value_type=bool)}]),

        # Unity: ROS ⇔ Unity の TCP 中継 (同じ PC。DDS ではない) とプレイヤー
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint', name='ros_tcp_endpoint',
             output='screen', parameters=[{'ROS_IP': '127.0.0.1',
                                           'ROS_TCP_PORT': ParameterValue(tcp_port, value_type=int)}],
             condition=IfCondition(use_unity)),
        ExecuteProcess(
            # -seed: 起動時のエピソード seed (照明・床・観戦者の乱択化)。/sim/episode は LATCHED だが
            #        endpoint 経由では接続前の latched が届かないので引数でも渡す
            # -record: 空なら録画しない。Unity が描いた画面をそのまま ffmpeg (libx264 ultrafast) へ流すので
            #          デスクトップ録画 (x11grab) と違い Wayland でも黒画面にならず、描画も止まらない
            cmd=[unity_player, '-rosip', '127.0.0.1', '-rosport', tcp_port, '-layout', 'aic', '-laps', '0', '-seed', seed,
                 '-record', LaunchConfiguration('record'),
                 '-recordfps', LaunchConfiguration('record_fps'),
                 '-recordwidth', LaunchConfiguration('record_width'),
                 '-screen-width', LaunchConfiguration('window_width'),
                 '-screen-height', LaunchConfiguration('window_height'),
                 '-logFile', os.path.expanduser('~/.ros/log/jetracer_unity_player.log')],
            name='unity_player', output='screen',
            condition=IfCondition(PythonExpression(
                ["(", use_unity, ") and '", unity_player, "' not in ('', 'none')"]))),

        Node(package='minicar_sim', executable='sim_viz.py', name='sim_viz', output='screen',
             parameters=[sim_params, {'use_shortcut': use_shortcut, 'narrow_divider': narrow_divider}],
             condition=IfCondition(PythonExpression(
                 ["'", use_rviz, "' == 'true' or '", use_viz, "' == 'true'"]))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', PythonExpression(
                 ["__import__('os').path.join('", rviz_dir, "', '", LaunchConfiguration('rviz_config'), "')"])],
             condition=IfCondition(use_rviz)),
    ])
