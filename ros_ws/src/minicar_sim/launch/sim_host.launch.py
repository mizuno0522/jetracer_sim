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
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class UnityPlayerPath(Substitution):
    """unity_player:=auto のとき、scripts/pick_unity_player.sh で GPU に合う版 (Built-in / URP / HDRP) を選ぶ。

    ★URP 版・HDRP 版はセンサ画像の見え方が Built-in と違う。画像で走る方策・検出器の評価・学習 (mlagents) では auto を使わない。
    """

    def __init__(self, value, mlagents):
        super().__init__()
        self._value, self._mlagents = value, mlagents

    def describe(self):
        return 'UnityPlayerPath()'

    def perform(self, context):
        import subprocess
        v = context.perform_substitution(self._value)
        if v != 'auto':
            return v
        builtin = os.path.join(os.environ.get('JETRACER_UNITY_PLAYER', os.path.expanduser('~/jetracer/unity/player')), 'MinicarSim.x86_64')
        if context.perform_substitution(self._mlagents) == 'true':
            return builtin                       # 学習は常に Built-in
        # install 先からはリポジトリの場所が分からないので、JETRACER_SIM_ROOT か既定の置き場所で探す
        root = os.environ.get('JETRACER_SIM_ROOT', os.path.expanduser('~/jetracer/jetracer_sim'))
        script = os.path.join(root, 'scripts', 'pick_unity_player.sh')
        try:
            return subprocess.run([script], check=True, capture_output=True, text=True).stdout.strip() or builtin
        except Exception as e:                   # 選べなければ Built-in
            print(f'[sim_host] unity_player:=auto: {script} を実行できない ({e})。Built-in 版を使う')
            return builtin


def generate_launch_description():
    sim_pkg = get_package_share_directory('minicar_sim')
    sim_params = os.path.join(sim_pkg, 'config', 'sim.yaml')
    course = LaunchConfiguration('course')
    # 実車スケールのコースは IMU の取付・レンジ・路面振動が違う (imu_sim_real.yaml)
    imu_params = PythonExpression(["'", os.path.join(sim_pkg, 'config', 'imu_sim.yaml'), "' if '", course,
                                   "' == 'minicar' else '", os.path.join(sim_pkg, 'config', 'imu_sim_real.yaml'), "'"])
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
    use_unity = PythonExpression(["'", camera_backend, "' == 'unity' and '",
                                  LaunchConfiguration('use_camera'), "' == 'true'"])
    lockstep = PythonExpression(["'", sim_mode, "' == 'lockstep'"])

    return LaunchDescription([
        DeclareLaunchArgument('course', default_value='minicar',
                              description='minicar (規約 p.24) | fuji (富士スピードウェイ・実車スケール。'
                                          'vehicle_profile:=real_nd|real_rx7|real_b787 と組み合わせる。docs/fuji.md)'),
        DeclareLaunchArgument('ml_maxsteps', default_value='1800',
                              description='ML-Agents の 1 エピソードの判断回数の上限 (15 Hz。富士は 3600 = 4 分)'),
        DeclareLaunchArgument('vehicle_profile', default_value='jetracer_tt02',
                              description='config/vehicle_profile/<name>.yaml (m05 で旧車両)'),
        DeclareLaunchArgument('sim_mode', default_value='realtime',
                              description='realtime (壁時計) / lockstep (/sim/step で進める・/clock)'),
        DeclareLaunchArgument('camera_backend', default_value='unity',
                              description='unity=Unity プレイヤーが描く / opencv=vehicle_sim が射影描画'),
        DeclareLaunchArgument('unity_player',
                              default_value=os.path.expanduser('~/minicarbattle2026/unity/MinicarSim/Build/MinicarSim.x86_64'),
                              description='Unity プレイヤー。none で起動しない (手動起動やエディタ Play のとき)。'
                                          'auto で GPU に合う版 (Built-in / URP / HDRP) を選ぶ (見せる用。画像で走る方策・評価には使わない)'),
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
        DeclareLaunchArgument('arrow_dir', default_value='center',
                              description='⑥矢印信号: center (既定) / random / alternate (周回ごとに左右を入れ替える) / left / right'),
        DeclareLaunchArgument('arrow_follow', default_value='false',
                              description='複数台のレースで、中継で届く /sim/arrow_master の矢印に従う (tools/race/race3.sh の 2・3 台目)'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='false で画像を出さない (カメラを使わない教師の相手役。Unity も endpoint も起動しない)'),
        DeclareLaunchArgument('unity_fps', default_value='60',
                              description='Unity の描画の上限 [FPS] (-fps)。3 台レースで PC が詰まるときに下げる'),
        DeclareLaunchArgument('seed', default_value='0', description='エピソードの seed (imu_sim の乱択化)'),
        # 見た目・音・ML-Agents (Unity だけに効く。物理は変わらない)
        DeclareLaunchArgument('car', default_value='',
                              description='自車のボディ: b787 | nd | rx7 (空なら従来の見た目)'),
        DeclareLaunchArgument('rival_car', default_value='', description='相手 (黄) のボディ'),
        DeclareLaunchArgument('rival2_car', default_value='', description='3 台目 (緑) のボディ'),
        DeclareLaunchArgument('sound', default_value='auto',
                              description='エンジン音: on | off | auto (ボディを選んだときだけ on)'),
        DeclareLaunchArgument('quality', default_value='auto',
                              description='画質: low | medium | high | auto (GPU を見て選ぶ。mlagents:=true なら low。docs/hdrp.md)'),
        DeclareLaunchArgument('mlagents', default_value='false',
                              description='true で Unity に ML-Agents のエージェントを作る (sim_mode:=lockstep と '
                                          'tools/mlagents/mlagents_gateway.py が要る。docs/mlagents.md)'),
        DeclareLaunchArgument('ml_port', default_value='5004', description='学習器 (mlagents-learn) のポート'),
        DeclareLaunchArgument('demo', default_value='',
                              description='ML-Agents: 人の運転を <名前>.demo に記録する (mlagents:=true のとき)'),
        DeclareLaunchArgument('demo_dir', default_value=os.path.join(os.getcwd(), 'demos'),
                              description='.demo を書く場所 (既定: 起動したフォルダの demos/)'),
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
                 'course': course,
                 'sim_mode': sim_mode,
                 'use_camera': ParameterValue(LaunchConfiguration('use_camera'), value_type=bool),
                 'arrow_follow': ParameterValue(LaunchConfiguration('arrow_follow'), value_type=bool),
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
             parameters=[{'config_file': ParameterValue(imu_params, value_type=str),
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
            cmd=[UnityPlayerPath(unity_player, LaunchConfiguration('mlagents')), '-rosip', '127.0.0.1', '-rosport', tcp_port, '-layout', 'aic', '-laps', '0', '-seed', seed,
                 '-fps', LaunchConfiguration('unity_fps'),
                 '-owncar', LaunchConfiguration('car'),
                 '-rivalcar', LaunchConfiguration('rival_car'),
                 '-rival2car', LaunchConfiguration('rival2_car'),
                 '-sound', LaunchConfiguration('sound'),
                 '-quality', LaunchConfiguration('quality'),
                 PythonExpression(["'-mlagents' if '", LaunchConfiguration('mlagents'), "' == 'true' else '-nomlagents'"]),
                 '-demo', LaunchConfiguration('demo'),
                 # ビルドしたプレイヤーは、この引数が無いと学習器 (mlagents-learn) に繋ぎに行かない (エディタだけ既定で 5004 に繋ぐ)
                 '--mlagents-port', LaunchConfiguration('ml_port'),
                 '-maxsteps', LaunchConfiguration('ml_maxsteps'),
                 # コースの描画は course.json (ミニカー) か course_<コース>_<profile>.json (StreamingAssets の中。
                 # COURSE=fuji ./scripts/export_course.sh <profile> が作る。カメラの取付が車ごとに違うので profile ごと)
                 '-course', PythonExpression(["'course.json' if '", course, "' == 'minicar' else 'course_", course, "_", profile, ".json'"]),
                 '-demodir', LaunchConfiguration('demo_dir'),
                 '-record', LaunchConfiguration('record'),
                 '-recordfps', LaunchConfiguration('record_fps'),
                 '-recordwidth', LaunchConfiguration('record_width'),
                 '-screen-width', LaunchConfiguration('window_width'),
                 '-screen-height', LaunchConfiguration('window_height'),
                 '-logFile', os.path.expanduser('~/.ros/log/jetracer_unity_player.log')],
            name='unity_player', output='screen',
            # ノート PC (内蔵 + 単体 GPU) では何も指定しないと内蔵 GPU で動く。Mesa に単体 GPU を選ばせる (1 枚だけの PC では無視される)
            additional_env={'DRI_PRIME': os.environ.get('DRI_PRIME', '1')},
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
