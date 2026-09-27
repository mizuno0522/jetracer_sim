#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
車両シミュレータ (JetRacer sim 版)。

既存 sim (minicarbattle2026) の vehicle_sim を JetRacer 機 (TT-02 4WD ＋ CSI カメラ ＋ 6 軸 IMU) に
向けたもの。物理 (摩擦限界つき自転車モデル・100 Hz) はそのまま。足したもの:
  - /actuator_cmd を minicar_msgs/ActuatorCmd (δ rad・v m/s・mode) で受ける (凍結する境界。P4)
  - vehicle_profile (yaml) で諸元を切り替え。drivetrain: 4wd_locked (シャフト 4WD・センターデフ無し)
  - アクチュエータ模型 (jetracer_common.actuator_model): サーボの遅れ・TBLE-02S の後退ロック・モータ上限
  - /sim/body_state (100 Hz): imu_sim へ渡す剛体状態。IMU は指令からではなく **ここから** 合成する (P8)
  - /sim/ground_truth (100 Hz): 学習ラベル (先行注視点の画像座標・横偏差・方位誤差・曲率・区間)
  - sim_mode:=lockstep: /sim/step・/sim/reset サービスで 1/30 s ずつ進める (/clock を出す)。物理は同一コード (P13)

元の説明:

模擬する範囲:
  - 自転車モデルによる車両運動 (Tamiya M-05 相当。前輪駆動)
  - VL53L1X x4 / HC-SR04 x2 のレイキャスト
  - RPM センサ由来の車輪距離 (低μ区間ではスリップを注入)
  - IMU (ヨーレート / ピッチ / 振動 / 横加速度)
  - カメラ由来の LaneInfo と ArrowSign と /camera/brightness

  - 合成カメラ画像 (use_camera:=true)。床・ウォール・矢印LEDボード・他車を
    描いて /camera/image_raw に流し、本物の arrow_detector / lane_detector を
    sim 内で回せるようにする。既定 (use_camera:=false) では画像を作らず、
    LaneInfo / ArrowSign をコース真値から直接生成する。

模擬しないもの:
  - 実機カメラの魚眼歪み・露出制御。合成側は広角ピンホールなので、
    BEV は専用の bev_sim.yaml (scripts/make_bev_sim.py が生成) を使う。
    実写に対する画像処理の検証は録画データで行うこと (docs/testing.md 参照)。

起動時は minicar_hw / minicar_localization / minicar_perception を止めて、
本ノードがそれらのトピックを肩代わりする。
"""

import math
import os
import time

# BLAS/OpenMP のスレッド並列は、ここで扱うような小さな行列には割に合わない。
# スピン待ちで CPU を食い潰し、100Hz の物理ステップを遅らせるだけなので、
# numpy を import する前に 1 スレッドへ落とす。
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from std_msgs.msg import Float32, Float64, Bool, Float64MultiArray
from sensor_msgs.msg import Range, LaserScan, Image, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, TransformStamped
from tf2_ros import TransformBroadcaster

from minicar_msgs.msg import (DriveCommand, VehicleStatus, LaneInfo,
                              ArrowSign, OpponentInfo, ActuatorCmd)
from minicar_sim_msgs.msg import BodyState, GroundTruth, StepInfo
from minicar_sim_msgs.srv import Reset as ResetSrv, Step as StepSrv
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32
from rosgraph_msgs.msg import Clock
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from jetracer_common.actuator_model import ServoModel, EscModel, MotorModel
from jetracer_common.cam_geom import CamGeom
from jetracer_common.reference_line import ReferenceLine

from course import (default_course, raycast, raycast_multi,
                    GIMMICK_AREAS, PARKING_SLOTS, surface_at, ARROW_SIGN)


# 矢印信号の向き。CENTER(直進=イエロー) は ArrowSign に DIR_CENTER が
# 入るまで arrow_detector と同じ内部コード 3 を使う。
_ARROW_DIR_CODES = {'left': ArrowSign.DIR_LEFT,
                    'right': ArrowSign.DIR_RIGHT,
                    'center': 3}
_ARROW_DIR_NAMES = {v: k.upper() for k, v in _ARROW_DIR_CODES.items()}



# race_manager がレース開始時に 1 回だけ送る制御トピック用。
# 購読側も TRANSIENT_LOCAL にしないと、起動が間に合わなかったときに
# 取りこぼす (実測: lap_counter が 0.296 秒遅れて enable を失った)。
LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST)

class VehicleSim(Node):

    def __init__(self):
        super().__init__('vehicle_sim')

        self.declare_parameter('rate_hz', 100.0)
        # 実時間積分。true なら 1 ステップの dt を実測時間から取る。
        # 固定 dt だと、CPU が混んだときにシム時間だけが遅れ、
        # 「センサ更新 1 回あたりに車が進む距離」が実機より短くなる。
        # そのぶん制御が有利になり、ノードを足すたびに走りが変わってしまう。
        self.declare_parameter('realtime', True)
        self.declare_parameter('max_step_dt_s', 0.03)
        self.declare_parameter('wheelbase_m', 0.210)      # M-05
        self.declare_parameter('max_steer_rad', 0.42)
        self.declare_parameter('max_speed_mps', 3.0)
        self.declare_parameter('accel_time_constant', 0.35)
        self.declare_parameter('sensor_noise_m', 0.004)
        # 後方センサ: 超音波 x1 (真後ろ) + 45 度 ToF x2 (左後ろ/右後ろ)
        self.declare_parameter('rear_sonar_max_m', 2.0)
        self.declare_parameter('publish_ground_truth', True)
        # 配信レートは物理の刻みと分けて持つ。rclpy はコールバック 1 回に
        # 約 1ms かかるので、100Hz の /odom を 9 ノードが購読すると
        # 受信側だけで 1 コアの 77% を使ってしまう (実測)。制御は 20〜50Hz
        # なので、物理は 100Hz のまま配信だけ落とす。
        self.declare_parameter('odom_rate_hz', 50.0)     # /odom, TF, IMU
        self.declare_parameter('slow_rate_hz', 20.0)     # /distance, /sim/*

        # --- 路面ごとの摩擦係数 (4輪それぞれの位置で引く) ---
        # ここが走りを決める。④は外が人工芝(高μ)・内が滑り板(低μ)で、
        # 境界を跨ぐと左右で μ が変わりヨーモーメントが立つ。
        self.declare_parameter('mu_carpet', 0.85)      # パンチカーペット
        self.declare_parameter('mu_turf', 1.05)        # ④人工芝 (高μ)
        self.declare_parameter('mu_slip_plate', 0.25)  # ④滑り板 (低μ)
        self.declare_parameter('mu_rough', 0.55)       # ⑦でこぼこ (接地が断続)
        self.declare_parameter('mu_slope', 0.70)       # ②坂道の白ボード
        # 駆動方式。M-05 は前輪駆動 (FF) で、前輪が加速と旋回を兼ねる。
        self.declare_parameter('drivetrain', 'front')      # front/rear/all
        self.declare_parameter('drive_load_share', 0.60)   # 静止時の駆動輪荷重割合
        # ★ 車両重量。摩擦限界 (a=μg) には質量が出てこないので、これが
        #   効くのは motor_force_n との組でモーター律速を作るところ。
        #   素の M-05 は 1.1〜1.3kg だが、Jetson / モバイルバッテリー /
        #   DC-DC / LiDAR / センサ / マウントプレートで 1kg 以上増える。
        self.declare_parameter('vehicle_mass_kg', 2.5)
        # ★ 駆動輪が路面に出せる力 [N]。実測して合わせること。
        #   既定 6.0N の根拠: 素のシャーシ (1.2kg) なら 5.0m/s² =
        #   カーペットの摩擦限界ちょうどに届く、という仮定。
        #   艤装後 2.5kg では 2.4m/s² となりモーター律速になる。
        #   実測方法: 直線で 0→2.0m/s の到達時間 t を測り F = m・(2.0/t)。
        self.declare_parameter('motor_force_n', 6.0)
        # ★ 重心高 [m]。前後の荷重移動 (ΔW/W = a_x・h/(g・L)) に効く。
        #   FF は加速すると駆動輪の荷重が抜けるので、高いほど加速が鈍る。
        #   Jetson とプレートが上に載るぶん素のシャーシより高い。
        self.declare_parameter('cog_height_m', 0.06)
        self.declare_parameter('vehicle_length_m', 0.40)   # ヨー慣性の計算に使う
        self.declare_parameter('slide_damp_ratio', 0.35)   # 横滑りを止める力 / μg
        self.declare_parameter('slide_share', 0.35)        # 横力不足のうち横滑りに回る割合
        self.declare_parameter('rough_drag_per_s', 0.9)    # ⑦の速度比例抵抗
        self.declare_parameter('rough_yaw_noise', 0.12)    # ⑦のヨー外乱 (rad/s)
        self.declare_parameter('slope_grade', 0.08)        # ②の勾配 (dz/dx)
        self.declare_parameter('vehicle_half_width_m', 0.11)
        # --- 壁の剛体衝突 (minicarbattle2026 の vehicle_sim 2026-09-27 と同じ式) ---
        # 以前は壁が車体の運動に作用せず、余裕が collision_clear_m 未満で「衝突」の印を立てるだけで
        # 壁を突き抜けられた。車体を 3 枚のディスク (後軸から wall_disc_offsets_m、半径 wall_disc_radius_m)
        # で表し、壁に達したら押し戻して法線速度を消し、接線速度はクーロン摩擦 (μ=wall_friction ×
        # 法線衝撃) で減らす。接触点が重心から外れていれば回転も受ける (wall_yaw_couple)。
        # 失った速度は v・vy の変化として /sim/body_state に出るので、imu_sim の加速度に衝撃が乗る。
        self.declare_parameter('wall_collision', True)
        self.declare_parameter('wall_friction', 0.5)
        self.declare_parameter('wall_restitution', 0.0)
        self.declare_parameter('wall_disc_radius_m', 0.11)             # TT-02 (M-05 は 0.095)
        self.declare_parameter('wall_disc_offsets_m', [0.0, 0.13, 0.26])  # TT-02 (M-05 は -0.05/0.13/0.30)
        self.declare_parameter('wall_yaw_couple', 1.0)
        self.declare_parameter('wall_yaw_tau_s', 0.15)     # 接触で受けた回転の減衰時定数
        # --- 2 台走行の相手 (minicarbattle2026 の vehicle_sim_race.py と同じ考え方、2026-09-27) ---
        # 相手の姿勢 /sim/rival_state (相手の /sim/render_state の中継、またはゴースト tools/ghost_replay.py) が
        # 届いているときだけ効く。1 台で走るときは何も変わらない。
        #   車どうしの衝突: 車体に収まる円 (TT-02: 全長 0.43 m・幅 0.20 m) どうしの剛体衝突。2 台は同じ質量として
        #   衝撃の半分ずつを各 sim が受け持つ。失った速度は body_state に出るので imu_sim の加速度に衝撃が乗る。
        #   相手はカメラ (Unity が /sim/rival_state の車を描く) に写る。LiDAR・超音波は JetRacer に無い。
        self.declare_parameter('car_collision', True)
        self.declare_parameter('car_restitution', 0.2)
        self.declare_parameter('car_friction', 0.3)
        self.declare_parameter('car_body_length_m', 0.43)
        self.declare_parameter('car_body_width_m', 0.20)
        self.declare_parameter('rival_timeout_s', 0.3)
        self.declare_parameter('pose_log', os.environ.get('RACE_POSE_LOG', ''))   # 2 台の位置を CSV に (10 Hz)
        # 車輪スリップ率。ゴムタイヤは限界でも 10〜15% 程度しか滑らず、
        # 限界を超えたところで急に空転する。elastic が限界時のスリップ率、
        # runaway が限界超過ぶんの空転係数。
        # (cmd_arbiter のトラクションコントロールが tc_slip_target=0.12 を
        #  見ているので、ここが過大だと TC が常時介入してしまう)
        self.declare_parameter('slip_elastic', 0.12)
        self.declare_parameter('slip_runaway', 0.60)
        self.declare_parameter('inject_stuck_at_s', -1.0)  # 秒。負なら無効
        self.declare_parameter('stuck_duration_s', 3.0)
        # 縦方向の内訳ログ (発進しない件の追跡用)。既定 false。
        self.declare_parameter('debug_motion', False)

        # --- 仮想他車 (決勝の追い越し検証用) ---
        # spawn_opponent=True で、自車の少し前を低速で走る相手を1台配置する。
        self.declare_parameter('spawn_opponent', False)
        self.declare_parameter('opponent_speed_mps', 0.6)
        self.declare_parameter('opponent_lead_m', 0.8)     # 初期の車間
        self.declare_parameter('opponent_lateral_m', 0.0)  # コリドー中心からの横位置
        # 真値ベースで /opponent_info も publish するか
        # (opponent_detector を使わず BT/制御だけ検証したいとき true)
        self.declare_parameter('publish_opponent_info', True)
        # 走行ルート: true=②坂道ショートカット経由(③ライトかく乱を通らない)
        self.declare_parameter('use_shortcut', True)
        # ⑤狭い道の中央仕切り。試走会/予選/決勝すべてで設置 (△3 p.30-31)。false は比較検証用
        self.declare_parameter('narrow_divider', True)

        # --- LiDAR (北陽 UST-20LX 相当) ---
        # 無制限クラス向け。use_lidar=True で /scan を配信し、そのスキャンから
        # 角度セクタ最小で仮想 ToF を切り出して既存の ToF トピックに流す
        # (制御側 wall_follow はそのまま流用できる)。
        self.declare_parameter('use_lidar', False)
        self.declare_parameter('lidar_fov_deg', 270.0)     # UST-20LX
        self.declare_parameter('lidar_res_deg', 0.25)      # 角度分解能
        self.declare_parameter('lidar_rate_hz', 40.0)      # 40Hz(25ms/scan)
        self.declare_parameter('lidar_max_range_m', 20.0)
        self.declare_parameter('lidar_min_range_m', 0.06)
        # 取付位置 (base_link 基準)。実機は車体前方に載せるので、
        # sim もそこから射出しないと融合側の格子と食い違う。
        # ★ minicar_fusion/config/fusion.yaml の lidar_mount_* と必ず同じ値にすること。
        self.declare_parameter('lidar_mount_x_m', 0.0)
        self.declare_parameter('lidar_mount_y_m', 0.0)
        self.declare_parameter('lidar_sector_deg', 30.0)   # 仮想ToF切り出し窓(全幅)

        # --- 合成カメラ (非魚眼・ピンホール) ---
        # use_camera=True で /camera/image_raw を配信し、本物の
        # arrow_detector / lane_detector 等をsim内で検証できるようにする。
        # まずは床＋矢印LEDボード＋他車を描画する (壁/レーンは順次追加)。
        self.declare_parameter('use_camera', False)
        # 画像の描き手。opencv = このノードで射影描画 (既定) /
        # unity = Unity (unity/MinicarSim) が /sim/render_state の姿勢で描いて
        # /camera/image_raw を出す。このノードは描かず、輝度だけ受け取った画像から取る。
        self.declare_parameter('camera_backend', 'opencv')
        # スタート位置をコース中心線から横へずらす [m, 左正]。
        # 複数台を同じスタートラインに横並びで置くとき用 (unity/race2.sh)。
        self.declare_parameter('start_lateral_m', 0.0)
        # スタート位置を中心線に沿って前へずらす [m] (負で後ろ)。複数台を縦に並べるとき用。
        # 周回判定 (lap_counter) は自分のスタートからの累積ヨーと距離で数えるので、
        # ずらしても周回タイムは公平なまま。
        self.declare_parameter('start_offset_m', 0.0)
        self.declare_parameter('cam_width', 320)
        self.declare_parameter('cam_height', 240)
        self.declare_parameter('cam_fov_deg', 120.0)   # 既定は sim.yaml と同値 (ピンホール等価の水平画角 = fx)
        self.declare_parameter('cam_vfov_deg', 0.0)    # 0 = 正方画素 (fy = fx)。取り込みを縮めて正方形にしていると fy ≠ fx
        self.declare_parameter('cam_k1', 0.0)          # 半径方向の歪み (OpenCV plumb_bob。k1 < 0 で樽型)
        self.declare_parameter('cam_k2', 0.0)
        self.declare_parameter('cam_mount_height_m', 0.12)
        self.declare_parameter('cam_pitch_deg', 12.0)      # 下向き
        # 実機 camera_node の crop_top_frac と一致させること
        self.declare_parameter('cam_crop_top_frac', 0.10)
        self.declare_parameter('cam_rate_hz', 20.0)
        # 真値 LaneInfo の配信。本物の lane_detector に明け渡すときは false。
        self.declare_parameter('publish_lane', True)
        # ウォール (SPF 1x4 = 19x89mm を立てた白板 + 下部の赤ストライプ) と
        # カーペットの描画。lane_detector は「床色の適応学習 + 赤/白ウォール
        # 検出」でコリドーを切り出すので、この 2 つが無いと sim で検証できない。
        self.declare_parameter('cam_draw_walls', True)
        self.declare_parameter('wall_height_m', 0.089)     # 板の高さ (SPF 1x4 の 89 mm)
        # 規約 p.34: 板は床から 30 mm 浮かせてあり、上端は約 120 mm。
        # 「制限部門マシンのタイヤ中心までの高さが 30mm。壁に衝突したときに滑り込まない隙間」
        self.declare_parameter('wall_base_m', 0.030)
        # 測距センサのビーム高さ。板の帯 (wall_base 〜 wall_base+wall_height) の外だと
        # 壁の下 (上) を抜けて見えない。実機の取付高さを入れて確認するための値。
        self.declare_parameter('tof_height_m', 0.060)
        self.declare_parameter('sonar_height_m', 0.060)
        self.declare_parameter('lidar_height_m', 0.100)
        self.declare_parameter('wall_tile_m', 0.45)        # 板 1 枚の長さ
        # 実物は白ボードの下側に赤ストライプが入る (2025 車載画像で確認)
        self.declare_parameter('wall_stripe_ratio', 0.38)
        self.declare_parameter('cam_render_range_m', 6.0)  # これより遠い壁は描かない
        # カーペットの斑点。BEV 位相相関の対地速度推定に模様が要る。
        self.declare_parameter('cam_floor_speckle', True)
        self.declare_parameter('floor_speckle_pitch_m', 0.06)

        # --- IMU / 地磁気 ---
        # ジャイロのバイアスとノイズ。積分ヨーは必ずドリフトするので、
        # pose_fusion (地磁気でバイアス推定) の効果を検証できる。
        self.declare_parameter('gyro_bias_dps', 0.7)
        self.declare_parameter('gyro_noise_dps', 0.4)
        # 地磁気 (AK8963 相当)。会場は鉄骨/SUS支柱で乱れるため、
        # 一様磁場 + ハードアイアン + 局所擾乱ゾーンで模擬する。
        self.declare_parameter('mag_field_ut', 46.0)       # 水平成分の強さ [uT]
        self.declare_parameter('mag_declination_deg', 0.0)  # 磁北とodom X軸のずれ
        self.declare_parameter('mag_noise_ut', 0.6)
        self.declare_parameter('mag_hard_iron_ut', [1.5, -2.0])
        # 擾乱ゾーン: (x, y, 半径, 付加磁場uT)。既定はトンネル(鉄骨)の八角コーナー
        self.declare_parameter('mag_disturb_zone', [9.6, 2.0, 1.2, 28.0])
        self.declare_parameter('publish_mag', True)

        # 矢印信号の向き。'random' は決勝どおり左右がランダムに決まる。
        # 'left'/'right'/'center' を指定すると固定でき、検証で両側を試せる。
        self.declare_parameter('arrow_dir', 'random')
        # random のとき、周回ごとに引き直すか (本番は毎周指示が変わる想定)
        self.declare_parameter('arrow_random_each_lap', True)

        # ================= JetRacer sim で足したもの =================
        # 時間の進め方: realtime (壁時計・流しっぱなし) / lockstep (/sim/step で 1/30 s ずつ)
        self.declare_parameter('sim_mode', 'realtime')
        self.declare_parameter('control_hz', 30.0)          # lockstep の 1 step = 1/control_hz
        self.declare_parameter('step_image_timeout_s', 0.5)  # lockstep で描画を待つ上限
        # /actuator_cmd の型: actuator = minicar_msgs/ActuatorCmd (既定) / drive = 旧 DriveCommand
        self.declare_parameter('cmd_msg', 'actuator')
        self.declare_parameter('cmd_timeout_s', 0.3)        # ブリッジと同じ 300 ms 失効
        self.declare_parameter('estop_hold_s', 1.0)         # ESTOP を受けたら RUN を無視する時間
        self.declare_parameter('vehicle_profile_name', 'jetracer_tt02')
        # --- アクチュエータ模型 (vehicle_profile) ---
        self.declare_parameter('servo_tau_s', 0.06)
        self.declare_parameter('servo_rate_limit_rad_s', 6.0)
        self.declare_parameter('esc_deadband_mps', 0.15)
        self.declare_parameter('esc_reverse_via_neutral_s', 0.12)
        self.declare_parameter('esc_brake_decel_mps2', 4.0)
        self.declare_parameter('coast_decel_mps2', 0.35)    # 惰行の減速 (転がり抵抗)
        self.declare_parameter('motor_v_free_mps', 4.5)     # 無負荷相当の車速 (駆動力が 0 になる点)
        # --- 4WD 拘束 (drivetrain: 4wd_locked) ---
        # 前後輪の回転が拘束されるので、旋回中は前輪 (経路が長い) が引きずられ後輪が押す。
        # 巻き込み量 r = 1/cosδ − 1 を slip_elastic で飽和させた割合 w = g·r / (g·r + slip_elastic) だけ
        # 前後力を使い、摩擦円の横方向の余力を減らし、抵抗として運動エネルギーを捨てる。★要較正
        # (δmax 27° で r = 0.12 → w ≈ 0.5。以前の min(1, r/slip_elastic) は 26.5° で w = 1 になり
        #  横グリップが 0・抵抗 4 m/s² で、フルロックの旋回で車が止まった。2026-09-26 の参照線試験)
        self.declare_parameter('windup_gain', 1.0)
        self.declare_parameter('windup_drag', 0.5)
        # --- 姿勢 (roll / pitch) の準静的な模型。imu_sim の重力投影に使う ---
        self.declare_parameter('roll_per_ms2', 0.020)       # rad / (m/s²) 横加速度 → 外側へロール
        self.declare_parameter('pitch_per_ms2', 0.012)      # rad / (m/s²) 加速で鼻上げ
        # --- ground truth ---
        self.declare_parameter('lookahead_m', 0.5)
        self.declare_parameter('lookahead_speed_gain', 0.3)  # ld = lookahead_m + gain × v
        self.declare_parameter('ground_truth_rate_hz', 100.0)
        self.declare_parameter('route_file', '')             # 空ならコース中心線。make_route.py の route.yaml も可
        self.declare_parameter('route_name', 'shortcut')
        self.declare_parameter('off_track_margin_m', 0.05)   # |cte| > 半幅 + margin で off_track
        self.declare_parameter('collision_clear_m', 0.10)    # 壁までの余裕がこれ未満で collision
        self.declare_parameter('episode_seed', 0)
        self.declare_parameter('car_id', 0)
        # ★ 車両の数値の定義元。名前 (config/vehicle_profile/<name>.yaml) か絶対パス。
        #   空なら sim.yaml / launch の値をそのまま使う (旧 sim と同じ)。
        self.declare_parameter('vehicle_profile_file', 'jetracer_tt02')

        p = self.get_parameter
        prof_name = str(p('vehicle_profile_file').value).strip()
        if prof_name:
            from jetracer_common.profile import find_profile, load_profile, vehicle_sim_overrides
            from rclpy.parameter import Parameter as _Param
            prof_path = find_profile(prof_name)
            self.vehicle_profile = load_profile(prof_path)
            ov = vehicle_sim_overrides(self.vehicle_profile)
            self.set_parameters([_Param(k, value=v) for k, v in ov.items()])
            self.get_logger().info(
                f"vehicle_profile: {self.vehicle_profile.get('name')} ({prof_path}) "
                f"WB={ov['wheelbase_m']:.3f} δmax={ov['max_steer_rad']:.2f} drive={ov['drivetrain']} "
                f"cam={ov['cam_width']}x{ov['cam_height']}@{ov['cam_rate_hz']:.0f}Hz")
        else:
            self.vehicle_profile = None
        self.dt = 1.0 / float(p('rate_hz').value)
        self.realtime = bool(p('realtime').value)
        self.max_step_dt = float(p('max_step_dt_s').value)
        self._last_step_t = None
        self._lag_warned = 0.0
        self.L = float(p('wheelbase_m').value)
        self.max_steer = float(p('max_steer_rad').value)
        self.max_speed = float(p('max_speed_mps').value)
        self.tau = float(p('accel_time_constant').value)
        self.noise = float(p('sensor_noise_m').value)
        self.rear_sonar_max = float(p('rear_sonar_max_m').value)
        self.odom_period = 1.0 / max(1e-3, float(p('odom_rate_hz').value))
        self.slow_period = 1.0 / max(1e-3, float(p('slow_rate_hz').value))
        self._t_odom_pub = 0.0
        self._t_slow_pub = 0.0
        self.mu_table = {
            None: float(p('mu_carpet').value),
            'TUNNEL': float(p('mu_carpet').value),
            'LIGHT': float(p('mu_carpet').value),
            'MU_HIGH': float(p('mu_turf').value),
            'MU_LOW': float(p('mu_slip_plate').value),
            'ROUGH': float(p('mu_rough').value),
            'SLOPE': float(p('mu_slope').value),
        }
        self.drive = str(p('drivetrain').value).strip().lower()
        self.drive_load = float(p('drive_load_share').value)
        self.mass = float(p('vehicle_mass_kg').value)
        self.motor_force = float(p('motor_force_n').value)
        self.cog_h = float(p('cog_height_m').value)
        self.veh_len = float(p('vehicle_length_m').value)
        self._a_x_prev = 0.0
        self.slide_damp = float(p('slide_damp_ratio').value)
        self.slide_share = float(p('slide_share').value)
        self.rough_drag = float(p('rough_drag_per_s').value)
        self.rough_yaw_noise = float(p('rough_yaw_noise').value)
        self.slope_grade = float(p('slope_grade').value)
        self.half_w = float(p('vehicle_half_width_m').value)
        self.wall_collision = bool(p('wall_collision').value)
        self.wall_mu = float(p('wall_friction').value)
        self.wall_e = float(p('wall_restitution').value)
        self.wall_r = float(p('wall_disc_radius_m').value)
        self.wall_offsets = [float(v) for v in p('wall_disc_offsets_m').value]
        self.wall_couple = float(p('wall_yaw_couple').value)
        self.wall_w_tau = max(1e-3, float(p('wall_yaw_tau_s').value))
        self._wall_w = 0.0         # 壁接触で受けた回転 (ヨーレートに足す、減衰)
        self._wall_contact = False
        self._wall_hits = 0        # 接触が始まった回数
        self.car_col = bool(p('car_collision').value)
        self.car_e = float(p('car_restitution').value)
        self.car_mu = float(p('car_friction').value)
        blen, bw = float(p('car_body_length_m').value), float(p('car_body_width_m').value)
        self.car_r = bw / 2.0
        rear = -(blen - self.L) / 2.0                     # 後軸基準の車体後端 (前後の張り出しは等分と仮定)
        front = self.L + (blen - self.L) / 2.0
        self.car_offsets = list(np.linspace(rear + self.car_r, front - self.car_r, 4))
        self.rival_timeout = float(p('rival_timeout_s').value)
        self._rival = None                 # (x, y, yaw, v, 受信時刻)
        self._rival_w = 0.0
        self._car_contact = False
        self._car_hits = 0
        # 左右μ差のヨーモーメント係数 = (トレッド/2) / (Iz/m)。
        # Iz/m ≒ (長さ² + 幅²)/12 なので、車体寸法から出す
        # (TT-02 で 4.9、M-05 は小さいぶん大きくなる)。
        self.yaw_split_k = self.half_w / max(
            1e-6, (self.veh_len ** 2 + (2 * self.half_w) ** 2) / 12.0)
        self.slip_elastic = float(p('slip_elastic').value)
        self.slip_runaway = float(p('slip_runaway').value)
        self.vy = 0.0            # 車体横方向の滑り速度 (左が正)
        self._slip_ratio = 0.0
        self._rough_w = 0.0      # ⑦のヨー外乱 (有色雑音)
        self._yaw_split = 0.0    # 左右μ差による持続的なヨー
        self._pitch = 0.0
        self._a_lat = 0.0
        self._rough_frac = 0.0
        self.stuck_at = float(p('inject_stuck_at_s').value)
        self.stuck_dur = float(p('stuck_duration_s').value)
        # 位置指定のスタック注入 (検証用): MINICAR_STUCK_AT_XY="x,y,t_min"。
        # sim 時刻 t_min 以降に (x,y) の 0.20 m 以内へ初めて入った時に注入する
        # (⑤狭い道の中など、時刻では狙えない場所用)。
        self._stuck_xy = None
        _sxy = os.environ.get('MINICAR_STUCK_AT_XY', '')
        if _sxy:
            self._stuck_xy = tuple(float(v) for v in _sxy.split(','))
        self.debug_motion = bool(p('debug_motion').value)
        self._dbg_t = 0.0
        self._dbg_wall0 = None
        self._dbg_simt0 = 0.0
        self._dbg_steps = 0

        self.spawn_opp = bool(p('spawn_opponent').value)
        self.opp_speed = float(p('opponent_speed_mps').value)
        self.opp_lead = float(p('opponent_lead_m').value)
        self.opp_lat = float(p('opponent_lateral_m').value)
        self.pub_opp_info = bool(p('publish_opponent_info').value)
        self.opp_s = None      # 他車のコース弧長 (spawn 後に初期化)

        self.use_lidar = bool(p('use_lidar').value)
        self.lidar_fov = math.radians(float(p('lidar_fov_deg').value))
        self.lidar_res = math.radians(float(p('lidar_res_deg').value))
        self.lidar_rate = float(p('lidar_rate_hz').value)
        self.lidar_max = float(p('lidar_max_range_m').value)
        self.lidar_min = float(p('lidar_min_range_m').value)
        self.lidar_mx = float(p('lidar_mount_x_m').value)
        self.lidar_my = float(p('lidar_mount_y_m').value)
        self.lidar_sector = math.radians(float(p('lidar_sector_deg').value))
        # 車体前方を中心とした 270° のビーム角 (車体座標)
        n_beams = int(round(self.lidar_fov / self.lidar_res)) + 1
        self.lidar_angles = np.linspace(-self.lidar_fov / 2.0,
                                        self.lidar_fov / 2.0, n_beams)

        self.use_camera = bool(p('use_camera').value)
        self.camera_backend = str(p('camera_backend').value).strip().lower()
        if self.camera_backend not in ('opencv', 'unity'):
            self.get_logger().warn(
                f"camera_backend='{self.camera_backend}' は不明。opencv として扱う")
            self.camera_backend = 'opencv'
        self.unity_camera = self.use_camera and self.camera_backend == 'unity'
        # Unity に姿勢を渡すのはカメラを使わない場合も (複数台レースの表示用)
        self.unity_render = self.camera_backend == 'unity'
        self.cam_w = int(p('cam_width').value)
        self.cam_h = int(p('cam_height').value)
        self.cam_pitch = math.radians(float(p('cam_pitch_deg').value))
        self.cam_hm = float(p('cam_mount_height_m').value)
        self.cam_rate = float(p('cam_rate_hz').value)
        # 合成カメラはピンホール描画なので画角 180 度以上は表現できない
        # (tan が発散・反転する)。実機の IMX219-222 は 222 度だが、sim では
        # その中心側だけを見ている扱いにして 170 度で頭打ちにする。
        fov_deg = float(p('cam_fov_deg').value)
        if fov_deg >= 170.0:
            self.get_logger().warn(
                f"cam_fov_deg={fov_deg:.0f} はピンホール描画では表現できない。"
                f"170 度に丸めた (魚眼の中心側だけを写す扱い)")
            fov_deg = 170.0
        fov_deg = max(10.0, fov_deg)
        # ★ カメラの幾何は CamGeom (jetracer_common) が定義元。ラベル・cmd_shaper・camera_info と同じ式
        self._cam_model = CamGeom(self.cam_w, self.cam_h, fov_deg, self.cam_hm,
                                  math.degrees(self.cam_pitch),
                                  vfov_deg=float(p('cam_vfov_deg').value),
                                  k1=float(p('cam_k1').value), k2=float(p('cam_k2').value))
        g = self._cam_model
        self.cam_f = g.fx                      # 互換 (ログ表示)
        self.cam_cx, self.cam_cy = g.cx, g.cy
        self._setup_render_canvas(g)
        # 実機 camera_node の crop_top_frac と同じ量を配信直前に捨てる。
        # 描画は cam_h のまま行い、切るのは publish_camera の最後だけ
        # (地平線や標識の描画式はクロップ前の cam_cy を前提にしている)。
        self._cam_crop_top_px = int(round(
            self.cam_h * min(0.6, max(0.0, float(p('cam_crop_top_frac').value)))))
        self.gyro_bias = math.radians(float(p('gyro_bias_dps').value))
        self.gyro_noise = math.radians(float(p('gyro_noise_dps').value))
        self.mag_field = float(p('mag_field_ut').value)
        self.mag_decl = math.radians(float(p('mag_declination_deg').value))
        self.mag_noise = float(p('mag_noise_ut').value)
        self.mag_hard_iron = [float(v) for v in p('mag_hard_iron_ut').value]
        self.mag_zone = [float(v) for v in p('mag_disturb_zone').value]
        self.pub_mag_on = bool(p('publish_mag').value)

        self.publish_lane = bool(p('publish_lane').value)
        self.draw_walls = bool(p('cam_draw_walls').value)
        self.wall_base = float(p('wall_base_m').value)
        self.wall_h = self.wall_base + float(p('wall_height_m').value)   # 上端の高さ
        self._sees_wall = {}
        for key in ('tof', 'sonar', 'lidar'):
            h = float(p(f'{key}_height_m').value)
            ok = self.wall_base <= h <= self.wall_h
            self._sees_wall[key] = ok
            if not ok:
                self.get_logger().error(
                    f'{key} のビーム高さ {h * 1000:.0f} mm は板の帯 '
                    f'{self.wall_base * 1000:.0f}〜{self.wall_h * 1000:.0f} mm の外。壁が見えない (最大距離を返す)')
        self.wall_tile = float(p('wall_tile_m').value)
        self.stripe_ratio = float(p('wall_stripe_ratio').value)
        self.render_range = float(p('cam_render_range_m').value)
        self.floor_speckle = bool(p('cam_floor_speckle').value)
        self.speckle_pitch = float(p('floor_speckle_pitch_m').value)

        self.course = default_course(
            use_shortcut=bool(self.get_parameter('use_shortcut').value),
            narrow_divider=bool(self.get_parameter('narrow_divider').value))
        self.segs = self.course.wall_segments()
        if self.use_camera:
            self._build_wall_tiles()

        # 矢印ゲートの世界座標 (⑥ 標識)。コースの ARROW_GATE 特徴位置を使う。
        self.arrow_gate_xyz = self._arrow_gate_position()

        # --- 車両状態 ---
        c, cum = self.course.center, self.course.cum_len
        s0 = float(self.get_parameter('start_offset_m').value) % self.course.total_length
        i = min(int(np.searchsorted(cum, s0, side='right')) - 1, len(c) - 2)
        f = (s0 - cum[i]) / max(1e-9, cum[i + 1] - cum[i])
        self.x = float(c[i][0] + f * (c[i + 1][0] - c[i][0]))
        self.y = float(c[i][1] + f * (c[i + 1][1] - c[i][1]))
        self.yaw = math.atan2(c[i + 1][1] - c[i][1], c[i + 1][0] - c[i][0])
        lat = float(self.get_parameter('start_lateral_m').value)
        self.x -= lat * math.sin(self.yaw)
        self.y += lat * math.cos(self.yaw)
        self.v = 0.0
        self.steer = 0.0
        self.distance = 0.0
        self.sim_time = 0.0

        self.cmd_throttle = 0.0
        self.cmd_steer = 0.0
        self.armed = False

        self.rng = np.random.default_rng(0)

        # ================= JetRacer sim: モード・アクチュエータ・参照線・カメラ幾何 =================
        self.sim_mode = str(p('sim_mode').value).strip().lower()
        self.lockstep = self.sim_mode == 'lockstep'
        if self.lockstep:
            self.realtime = False      # 固定 10 ms ティック。壁時計は見ない
        self.control_dt = 1.0 / float(p('control_hz').value)
        self.step_img_timeout = float(p('step_image_timeout_s').value)
        self.cmd_msg = str(p('cmd_msg').value).strip().lower()
        self.cmd_timeout = float(p('cmd_timeout_s').value)
        self.estop_hold = float(p('estop_hold_s').value)
        self.cmd_steer_rad = 0.0
        self.cmd_speed = 0.0
        self.cmd_mode = ActuatorCmd.MODE_IDLE
        self._cmd_time = -1.0          # sim 時刻
        self._estop_until = -1.0
        self.servo = ServoModel(float(p('servo_tau_s').value),
                                float(p('servo_rate_limit_rad_s').value), self.max_steer)
        self.esc = EscModel(float(p('esc_deadband_mps').value),
                            float(p('esc_reverse_via_neutral_s').value),
                            float(p('esc_brake_decel_mps2').value))
        self.brake_decel = float(p('esc_brake_decel_mps2').value)
        self.coast_decel = float(p('coast_decel_mps2').value)
        self.motor = MotorModel(self.motor_force, float(p('motor_v_free_mps').value))
        self.windup_gain = float(p('windup_gain').value)
        self.windup_drag = float(p('windup_drag').value)
        self.roll_k = float(p('roll_per_ms2').value)
        self.pitch_k = float(p('pitch_per_ms2').value)
        self._esc_target = None
        self._esc_brake = False
        self._esc_state = 'FWD'
        self._roll = 0.0
        self._pitch_dyn = 0.0
        self._wx = 0.0
        self._wy = 0.0
        self._vy_prev = 0.0
        self._v_prev = 0.0
        self._ax_kin = 0.0
        self._ay_kin = 0.0
        self._steer_prev = 0.0
        self._steer_rate = 0.0
        self.yaw_rate = 0.0
        self.step_id = 0
        self.car_id = int(p('car_id').value)
        self.lap = 0
        self._s_prev = None
        self._lap_done = False
        self._collision = False
        self._off_track = False
        self._min_clear = 9.9
        self.lookahead_m = float(p('lookahead_m').value)
        self.lookahead_gain = float(p('lookahead_speed_gain').value)
        self.gt_period = 1.0 / max(1e-3, float(p('ground_truth_rate_hz').value))
        self._t_gt_pub = 0.0
        self.off_track_margin = float(p('off_track_margin_m').value)
        self.collision_clear = float(p('collision_clear_m').value)
        self.episode_seed = int(p('episode_seed').value)
        route_file = str(p('route_file').value).strip()
        if route_file:
            if '/' not in route_file:          # 名前だけなら minicar_sim の config/ (install 側) から
                from ament_index_python.packages import get_package_share_directory
                route_file = os.path.join(get_package_share_directory('minicar_sim'), 'config', route_file)
            self.ref = ReferenceLine.from_yaml_route(os.path.expanduser(route_file),
                                                     str(p('route_name').value))
            self.get_logger().info(f"参照線: {route_file} ({p('route_name').value}) 全長 {self.ref.total:.2f} m")
        else:
            self.ref = ReferenceLine(self.course.center)
        g0 = self._cam_model
        self.cam_geom = CamGeom(self.cam_w, self.cam_h, fov_deg, self.cam_hm,
                                math.degrees(self.cam_pitch),
                                float(p('cam_crop_top_frac').value),
                                vfov_deg=float(p('cam_vfov_deg').value), k1=g0.k1, k2=g0.k2)
        # ★ vehicle_profile の drivetrain は front/rear/all/4wd_locked
        if self.drive not in ('front', 'rear', 'all', '4wd_locked'):
            self.get_logger().error(f"drivetrain='{self.drive}' は不明。all として扱う")
            self.drive = 'all'
        self._surface_ids = {None: BodyState.SURFACE_CARPET, 'TUNNEL': BodyState.SURFACE_TUNNEL,
                             'LIGHT': BodyState.SURFACE_LIGHT, 'MU_HIGH': BodyState.SURFACE_TURF,
                             'MU_LOW': BodyState.SURFACE_SLIP, 'ROUGH': BodyState.SURFACE_ROUGH,
                             'SLOPE': BodyState.SURFACE_SLOPE}
        self.pub_body = self.create_publisher(BodyState, '/sim/body_state', 10)
        self.pub_gt = self.create_publisher(GroundTruth, '/sim/ground_truth', 10)
        self.pub_episode = self.create_publisher(UInt32, '/sim/episode', LATCHED)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10) if self.lockstep else None
        self._cb_group_io = ReentrantCallbackGroup()
        self._cb_group_step = MutuallyExclusiveCallbackGroup()
        self._last_img = None
        self._img_event = threading.Event()
        self._imu_since_step = []
        # imu_sim にエピソードの seed を配る (LATCHED なので後から起動しても届く)
        self.pub_episode.publish(UInt32(data=self.episode_seed))

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # --- Publisher (実ノードの代替) ---
        self.pub_tof_l = self.create_publisher(Range, '/sensors/tof_left', sensor_qos)
        self.pub_tof_r = self.create_publisher(Range, '/sensors/tof_right', sensor_qos)
        # 前方45度の斜めToF (カーブ先読み用の追加2個 → 計4個構成)
        self.pub_tof_fl = self.create_publisher(Range, '/sensors/tof_fl', sensor_qos)
        self.pub_tof_fr = self.create_publisher(Range, '/sensors/tof_fr', sensor_qos)
        self.pub_sonar = self.create_publisher(Range, '/sensors/sonar_front', sensor_qos)
        # 後方センサは超音波 1 本だけ。前方マウントの LiDAR は後ろ 90 度が
        # 死角なので、後退時に見えるのはこのセンサだけになる。
        self.pub_sonar_rear = self.create_publisher(
            Range, '/sensors/sonar_rear', sensor_qos)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', sensor_qos)
        # Unity 描画時は Unity 側が /camera/image_raw の唯一の発行者になる
        if not self.unity_camera:
            self.pub_cam = self.create_publisher(Image, '/camera/image_raw', sensor_qos)
        # Unity に渡す描画状態 (姿勢と、画像に写る動的なもの)。並びは
        # RENDER_STATE_FIELDS。Unity は受け取った stamp を画像の stamp にするので、
        # 画像の鮮度は「その姿勢を配信した時刻」基準になる (描画遅延が年齢に乗る)。
        self.pub_render = (self.create_publisher(Float64MultiArray,
                                                 '/sim/render_state', 10)
                           if self.unity_render else None)
        self.pub_wheel = self.create_publisher(Float32, '/sensors/wheel_distance', 10)
        self.pub_status = self.create_publisher(VehicleStatus, '/vehicle_status', 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_dist = self.create_publisher(Float64, '/distance', 10)
        # シミュレーション時間 (壁時計ではなくこれで採点する。CPU が混むと
        # シム時間は実時間より遅れるので、壁時計だと負荷の重い構成が
        # 不当に遅く見える。実際にカメラ有りで 1.56 倍に伸びていた)
        self.pub_simtime = self.create_publisher(Float32, '/sim/time', 10)
        # 路面の摩擦モデルの真値 (HUD で見るための可視化用。実機には無い)
        self.pub_mu = self.create_publisher(Float32, '/sim/mu', 10)
        self.pub_sideslip = self.create_publisher(Float32, '/sim/sideslip', 10)
        self.pub_wslip = self.create_publisher(Float32, '/sim/wheel_slip', 10)
        self.pub_yaw = self.create_publisher(Float32, '/imu/yaw', 20)
        self.pub_yaw_rate = self.create_publisher(Float32, '/imu/yaw_rate', 20)
        self.pub_pitch = self.create_publisher(Float32, '/imu/pitch', 20)
        self.pub_vib = self.create_publisher(Float32, '/imu/vibration_rms', 10)
        self.pub_lat = self.create_publisher(Float32, '/imu/lateral_accel', 20)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10)

        self.pub_lane = self.create_publisher(LaneInfo, '/lane_info', 10)
        self.pub_arrow = self.create_publisher(ArrowSign, '/arrow_sign', 10)
        self.pub_bright = self.create_publisher(Float32, '/camera/brightness', 10)
        self.pub_gs = self.create_publisher(Float32, '/perception/ground_speed', 10)
        self.pub_opp = self.create_publisher(OpponentInfo, '/opponent_info', 10)

        self.tf = TransformBroadcaster(self)

        if self.cmd_msg == 'drive':
            self.create_subscription(DriveCommand, '/actuator_cmd', self.cb_cmd, 10)
        else:
            # 凍結する契約: reliable / depth 1。ブリッジと同じ QoS
            cmd_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(ActuatorCmd, '/actuator_cmd', self.cb_actuator, cmd_qos,
                                     callback_group=self._cb_group_io)
        self.create_subscription(Bool, '/odom/reset', self.cb_reset, LATCHED)

        if self.lockstep:
            # 時間は /sim/step が進める。画像と IMU は step の戻り値に載せるために購読する
            self.create_service(StepSrv, '/sim/step', self.srv_step,
                                callback_group=self._cb_group_step)
            self.create_service(ResetSrv, '/sim/reset', self.srv_reset,
                                callback_group=self._cb_group_step)
            self.create_subscription(Image, '/camera/image_raw', self._cb_step_image, sensor_qos,
                                     callback_group=self._cb_group_io)
            self.create_subscription(Imu, '/imu', self._cb_step_imu, sensor_qos,
                                     callback_group=self._cb_group_io)
            self._publish_clock()
            self.get_logger().info(
                f"lockstep: /sim/step で 1/{1.0 / self.control_dt:.0f} s ずつ進める (物理 {self.dt * 1000:.0f} ms ティック)")
        else:
            self.create_timer(self.dt, self.step)
        self.create_timer(0.05, self.publish_sensors)
        self.create_subscription(Float64MultiArray, '/sim/rival_state', self._cb_rival, 10)
        self._pose_log = None
        if p('pose_log').value:
            self._pose_log = open(f"{p('pose_log').value}_{os.environ.get('ROS_DOMAIN_ID', '0')}.csv", 'w')
            self._pose_log.write('t,x,y,yaw,v,rx,ry,ryaw,rv,dist\n')
            self.create_timer(0.1, self._log_pose)
        self.create_timer(0.05, self.publish_perception)
        if self.use_lidar:
            self.create_timer(1.0 / self.lidar_rate, self.publish_lidar)
        if self.unity_camera:
            # 描かない。/camera/brightness は Unity の画像の平均輝度から出す
            # (zone_estimator のトンネル判定を実際の見え方と整合させる)。
            self.create_subscription(Image, '/camera/image_raw',
                                     self.cb_unity_image, sensor_qos)
        elif self.use_camera:
            from cv_bridge import CvBridge
            import cv2 as _cv2
            self.cv2 = _cv2
            # OpenCV のスレッド並列は、他ノードと CPU を奪い合って
            # 物理ステップ (100Hz) を遅らせるだけなので切る。
            _cv2.setNumThreads(1)
            self.cam_bridge = CvBridge()
            # 毎フレームの正規乱数生成は重い (5ms) ので、あらかじめ数枚分を
            # int16 で作っておいて順に足す。見た目のランダムさは十分。
            if self.noise > 0:
                self._noise_bank = [
                    self.rng.normal(0, 3.0, (self.cam_h, self.cam_w, 1))
                    .repeat(3, axis=2).astype(np.int16)
                    for _ in range(8)]
                self._noise_i = 0
            if not self.lockstep:
                self.create_timer(1.0 / self.cam_rate, self.publish_camera)
        self._cam_acc = 0.0          # lockstep: 画像の周期 (1/cam_rate) を step で数える

        # 矢印はスタートごとにランダムで左右が決まる (決勝仕様)。
        # arrow_dir:=left|right|center を渡せば固定でき、検証で両側を試せる。
        self.arrow_each_lap = bool(p('arrow_random_each_lap').value)
        want = str(p('arrow_dir').value).strip().lower()
        if want in _ARROW_DIR_CODES:
            self.arrow_dir = _ARROW_DIR_CODES[want]
        else:
            if want not in ('', 'random'):
                self.get_logger().warn(
                    f"arrow_dir='{want}' は不明。random として扱う "
                    f"(有効値: {'/'.join(_ARROW_DIR_CODES)}/random)")
            self.arrow_dir = int(self.rng.integers(1, 3))
        self.arrow_random = self.arrow_dir not in _ARROW_DIR_CODES.values() or \
            want in ('', 'random')

        if self.unity_camera:
            self.get_logger().info(
                "カメラ画像は Unity が描画する (/sim/render_state → /camera/image_raw)")
        self.get_logger().info(
            f"VehicleSim started. コース全長 {self.course.total_length:.2f}m, "
            f"矢印={_ARROW_DIR_NAMES[self.arrow_dir]}, "
            f"測距={'LiDAR(UST-20LX相当)' if self.use_lidar else '固定ToF'}"
            f"{', 合成カメラON' if self.use_camera else ''}"
        )

    # -----------------------------------------------------------------
    def cb_cmd(self, msg: DriveCommand):
        """旧 DriveCommand (cmd_msg:=drive)。正規化値を ActuatorCmd 相当に写す。"""
        self.cmd_steer_rad = float(msg.steer_norm) * self.max_steer
        self.cmd_speed = (float(msg.throttle_norm) * self.max_speed if msg.use_raw_throttle
                          else float(msg.speed_mps))
        if msg.brake:
            self.cmd_speed = 0.0
        self.cmd_mode = ActuatorCmd.MODE_IDLE if msg.disarm else ActuatorCmd.MODE_RUN
        self._cmd_time = self.sim_time

    def cb_actuator(self, msg: ActuatorCmd):
        """凍結した契約 /actuator_cmd (δ rad・v m/s・mode)。"""
        if msg.mode == ActuatorCmd.MODE_ESTOP:
            self._estop_until = self.sim_time + self.estop_hold
        self.cmd_steer_rad = float(msg.steer_rad)
        self.cmd_speed = float(msg.speed_mps)
        self.cmd_mode = int(msg.mode)
        self._cmd_time = self.sim_time

    def cb_reset(self, msg: Bool):
        if msg.data:
            self.distance = 0.0
            self.sim_time = 0.0
            # 配信スケジュールも一緒に戻す。これを忘れると sim_time だけが
            # 巻き戻り、publish_odom の間引き判定
            #   sim_time - _t_odom_pub < odom_period
            # が「巻き戻したぶんの秒数」ずっと成立し続けて **/odom が丸ごと
            # 止まる**。race_manager はレース開始時に /odom/reset を出すので、
            # 起動から開始までの待ち時間 (run_sim.sh は 9 秒) と同じ長さだけ
            # 下流が盲目になり、実際には走っている車を BT がスタックと誤認して
            # 脱出動作に入る。
            self._t_odom_pub = 0.0
            self._t_slow_pub = 0.0

    # =================================================================
    # 路面の摩擦と運動方程式
    # =================================================================
    G = 9.81

    def _wheel_mu(self, dt):
        """
        4輪それぞれが乗っている路面から μ を引く。

        タイヤ位置は base_link (後軸) を原点に、前軸 +L / 左右 ±半車幅。
        返すのは (前軸μ, 後軸μ, 左μ, 右μ, 全体μ, でこぼこ接地率)。
        左右のμが違うとき (低μ板を跨いだとき) に、あとでヨーモーメントを立てる。
        """
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hw = self.half_w
        mus = {}
        rough = 0
        for ax, lx in (('f', self.L), ('r', 0.0)):
            for sd, ly in (('l', hw), ('r', -hw)):
                wx = self.x + lx * c - ly * s
                wy = self.y + lx * s + ly * c
                surf = surface_at(wx, wy)
                mus[ax + sd] = self.mu_table.get(surf, self.mu_table[None])
                if surf == 'ROUGH':
                    rough += 1
        mu_f = 0.5 * (mus['fl'] + mus['fr'])
        mu_r = 0.5 * (mus['rl'] + mus['rr'])
        mu_l = 0.5 * (mus['fl'] + mus['rl'])
        mu_rt = 0.5 * (mus['fr'] + mus['rr'])
        return mu_f, mu_r, mu_l, mu_rt, 0.5 * (mu_f + mu_r), rough / 4.0

    def _slope_pitch(self):
        """
        ②坂道の勾配から車体ピッチを出す。

        坂は x 方向に三角形 (前半が登り・後半が下り) なので、進行方向が
        逆でも登り/下りが自動的に入れ替わる。pitch>0 が鼻上げ。
        """
        for name, x0, y0, x1, y1 in GIMMICK_AREAS:
            if name != 'SLOPE':
                continue
            if not (x0 <= self.x <= x1 and y0 <= self.y <= y1):
                return 0.0
            grade = self.slope_grade if self.x < (x0 + x1) / 2.0 \
                else -self.slope_grade
            return math.atan(grade * math.cos(self.yaw))
        return 0.0

    def _dbg_motion(self, target_v, v, a_cmd, a_x_pure, a_x_cap, util_y,
                    mu_drive, a_x):
        """縦方向の内訳を 5Hz で出す (debug_motion:=true のときだけ)。

        「指令は出ているのに車が動かない」を追うための一時的な観測点。
        スロットル → target_v → a_cmd → a_x → v のどこで落ちるかを見る。
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._dbg_t < 0.2:
            return
        # シム時刻が実時間からどれだけ遅れたか。CPU が足りず step() が
        # 回らないと、車は「動いていない」のではなく「シム時間が進んで
        # いない」だけになる。両者は外から見分けがつかないので必ず出す。
        if self._dbg_wall0 is None:
            self._dbg_wall0 = now
            self._dbg_simt0 = self.sim_time
        lag = (now - self._dbg_wall0) - (self.sim_time - self._dbg_simt0)
        steps = self._dbg_steps
        self._dbg_steps = 0
        self._dbg_t = now
        self.get_logger().info(
            f"[MOTION] lag={lag:+.2f}s steps/s={steps / 0.2:.0f} "
            f"armed={int(self.armed)} thr={self.cmd_throttle:+.3f} "
            f"tgt_v={target_v:+.3f} v={v:+.4f} a_cmd={a_cmd:+.3f} "
            f"a_pure={a_x_pure:.3f} a_cap={a_x_cap:.3f} util_y={util_y:.3f} "
            f"mu_d={mu_drive:.3f} a_x={a_x:+.3f} pitch={self._pitch:+.4f} "
            f"x={self.x:.3f} y={self.y:.3f}")

    def _integrate_motion(self, dt, target_v, stuck):
        """
        タイヤの摩擦限界つき自転車モデルを 1 ステップ進める。

        運動学だけのモデルだと「舵を切れば必ず曲がる」ので、低μ板に乗っても
        でこぼこ道でも走りが変わらず、ルート設計の是非を検証できなかった。
        ここでは各輪の μ から次を作る:
          - 摩擦円: 横に使ったぶんだけ前後加速の上限が減る
          - 横力の飽和: 足りないぶんは横滑り速度 vy になり、外へ膨らむ
          - 前軸が先に飽和 → アンダー、後軸が先 → オーバー(巻き込み)
          - 左右μ差 (跨ぎ) → ヨーモーメント
          - ⑦でこぼこ: 速度比例の抵抗とヨー外乱
          - ②坂道: 重力の進行方向成分
        戻り値は接地基準の移動距離。
        """
        g = self.G
        mu_f, mu_r, mu_l, mu_rt, mu, rough = self._wheel_mu(dt)
        self._rough_frac = rough
        self._mu_now = mu
        v = self.v

        # --- 駆動輪の縦方向使用率 → その軸の横力が減る ---
        # M-05 は前輪駆動なので、前輪が加速と旋回を兼ねる。加速するほど
        # 前輪の横力の余力が減り、曲がらなくなる (FF のアンダーステア)。
        # RC の ESC ブレーキも駆動輪にしか効かないので減速側も同じ。
        # 前ステップの a_x を使うのは、a_x と横力が相互に依存して解けない
        # ため (100Hz なので 1 ステップの遅れは実質問題にならない)。
        mu_drive = mu_f if self.drive == 'front' else (
            mu_r if self.drive == 'rear' else mu)

        # --- 前後の荷重移動 ---
        # 加速すると荷重は後ろへ移る。**FF は駆動輪が前なので、加速するほど
        # 駆動輪の荷重が抜けて加速できなくなる**という負のループになる。
        # ΔW/W = a_x・h / (g・L)。drive_load_share は「静止時の」配分として扱う。
        # 質量そのものは約分されて消える (F=μmg → a=μg) が、この配分の
        # 移動量は重心高とホイールベースだけで決まるので効く。
        shift = self._a_x_prev * self.cog_h / (g * self.L)
        if self.drive == 'front':
            drive_share = self.drive_load - shift
        elif self.drive == 'rear':
            drive_share = self.drive_load + shift
        else:
            drive_share = self.drive_load
        # 荷重が完全に抜ける/全部乗ることは無いので端をクランプする
        drive_share = max(0.10, min(0.90, drive_share))

        a_x_pure = mu_drive * g * drive_share
        util_x_drive = min(1.0, abs(self._a_x_prev) / max(1e-6, a_x_pure))
        shrink = math.sqrt(max(0.0, 1.0 - util_x_drive ** 2))
        mu_f_eff = mu_f * (shrink if self.drive == 'front' else 1.0)
        mu_r_eff = mu_r * (shrink if self.drive == 'rear' else 1.0)
        if self.drive in ('all', '4wd_locked'):
            mu_f_eff, mu_r_eff = mu_f * shrink, mu_r * shrink
        # --- 4WD 拘束 (センターデフ無し): 旋回中の前後輪の回転差が拘束されて「突っ張る」 ---
        # 前輪の経路は後輪より 1/cosδ 長いので、その差ぶん前輪は引きずられ後輪は押す。
        # 使った前後力の割合 windup を摩擦円から差し引き、抵抗として捨てる。
        windup = 0.0
        if self.drive == '4wd_locked' and abs(v) > 0.05:
            ratio = self.windup_gain * (1.0 / max(1e-3, math.cos(self.steer)) - 1.0)
            windup = ratio / (ratio + max(1e-3, self.slip_elastic))     # 0 ≤ w < 1 (飽和形)
            wshrink = math.sqrt(max(0.0, 1.0 - windup ** 2))
            mu_f_eff *= wshrink
            mu_r_eff *= wshrink
        self._windup = windup

        # --- 横方向: 要求横加速度とタイヤが出せる横加速度 ---
        yaw_rate_kin = v / self.L * math.tan(self.steer)
        a_y_req = v * yaw_rate_kin
        a_y_cap = 0.5 * (mu_f_eff + mu_r_eff) * g
        a_y_tire = max(-a_y_cap, min(a_y_cap, a_y_req))
        util_y = min(1.0, abs(a_y_req) / max(1e-6, a_y_cap))

        # --- 前後方向: 駆動輪の摩擦円の残りが使える ---
        # ESC の状態で指令加速度を作る (惰行 / ブレーキ帯 / 駆動)
        if self._esc_target is None:
            a_cmd = -math.copysign(self.coast_decel, v) if abs(v) > 0.02 else -v / max(dt, 1e-3)
        elif self._esc_brake:
            a_cmd = -math.copysign(min(self.brake_decel, abs(v) / max(dt, 1e-3)), v) if abs(v) > 1e-3 else 0.0
        else:
            a_cmd = (target_v - v) / self.tau
        a_x_cap = a_x_pure * math.sqrt(max(0.0, 1.0 - util_y ** 2))
        # ★ モーターの出せる力による上限。ここが無いと「重くしても速度が
        #   変わらない」モデルになる (摩擦限界 a=μg には質量が出てこないため)。
        #   実車は Jetson / バッテリー / LiDAR / プレートで素のシャーシから
        #   1kg 以上増えるので、加速側はほぼこちらが律速する。
        #   減速は ESC ブレーキ + 転がり抵抗なので、この上限は掛けない。
        # DC モータの電流上限: 高速域ほど駆動力が落ちる (jetracer_common.MotorModel)
        a_motor = self.motor.force_cap(v) / max(0.1, self.mass)
        a_x_cap_accel = min(a_x_cap, a_motor)
        a_x = max(-a_x_cap, min(a_x_cap_accel, a_cmd))
        # ★ 制動中に前後＋横の要求が摩擦円を超えたら、横優先をやめて要求を同じ比で縮める
        #   (楕円の飽和)。横優先のままだと、旋回で横が限界に張り付くと util_y=1 → a_x_cap=0 で
        #   ブレーキが全く効かず、一定速度のまま外へ膨らんで壁に当たる (M-05 の坂道ヘアピン出口で発見)。
        #   実車は滑っているタイヤの摩擦が滑る向きに働くので、ブレーキを掛ければ減速し、そのぶん曲がらなくなる。
        if a_cmd < 0.0 and not stuck:
            a_y_lim = 0.5 * (mu_f + mu_r) * g
            r = math.hypot(a_cmd / max(1e-6, a_x_pure), a_y_req / max(1e-6, a_y_lim))
            if r > 1.0 and a_cmd / r < a_x:
                a_x = a_cmd / r
                a_y_tire = a_y_req / r
        self._a_x_prev = a_x
        # 重力と転がり抵抗はグリップと無関係にかかる
        self._pitch = self._slope_pitch()
        a_x += -g * math.sin(self._pitch)
        if rough > 0.0:
            a_x -= self.rough_drag * rough * v
        if getattr(self, '_windup', 0.0) > 0.0:
            a_x -= math.copysign(self.windup_drag * mu * g * self._windup, v)
        if stuck:
            a_x = -v / max(1e-3, self.tau)

        if self.debug_motion:
            self._dbg_motion(target_v, v, a_cmd, a_x_pure, a_x_cap, util_y,
                             mu_drive, a_x)

        # --- ヨー: 実際に曲がれる量 ---
        yaw_rate = a_y_tire / max(0.15, abs(v)) * (1.0 if v >= 0 else -1.0)
        if abs(v) < 0.15:
            yaw_rate = yaw_rate_kin
        # 後軸のほうが先に飽和したら巻き込む (オーバーステア)。
        # 前輪駆動では前軸の余力が先に減るので、通常はこちらは成立せず
        # アンダーステア側に出る。
        if abs(a_y_req) > mu_r_eff * g and mu_r_eff < mu_f_eff:
            over = (abs(a_y_req) - mu_r_eff * g) / max(1e-6, mu_r_eff * g)
            yaw_rate += math.copysign(min(2.0, over) * 0.6, a_y_req)

        # --- 左右μ差 (跨ぎ) によるヨーモーメント ---
        # 片輪あたりの前後力 (加速度換算 |a_x|/2) が弱い側の限界 μg/2 を
        # 超えると、その分だけ左右で押す力に差が出てヨーモーメントになる。
        # M = m・Δa・(トレッド/2)、Iz ≈ m(l²+w²)/12。係数は車体寸法から出す
        # (self.yaw_split_k)。
        # 一次遅れにしてあるのは、車が回り始めるとタイヤに横滑り角がついて
        # 戻す力が出るため (そのまま積分するとスピンし続けてしまう)。
        mu_weak, mu_strong = min(mu_l, mu_rt), max(mu_l, mu_rt)
        cap_weak = mu_weak * g / 2.0
        a_x_side = abs(a_x) / 2.0
        yaw_acc_split = 0.0
        if a_x_side > cap_weak and mu_strong > mu_weak:
            d_a = min(a_x_side - cap_weak, (mu_strong - mu_weak) * g / 2.0)
            side = 1.0 if mu_l > mu_rt else -1.0     # グリップの高い側
            # 加速中は高μ側が強く押して逆へ、減速中は高μ側が引いて同じ側へ
            yaw_acc_split = (-side * math.copysign(1.0, a_x)
                             * self.yaw_split_k * d_a)
        self._yaw_split += (yaw_acc_split - self._yaw_split / 0.12) * dt
        yaw_rate += self._yaw_split

        # --- ⑦でこぼこのヨー外乱 (有色雑音) ---
        if rough > 0.0:
            self._rough_w = 0.85 * self._rough_w + float(self.rng.normal(
                0.0, self.rough_yaw_noise)) * rough * min(1.0, abs(v))
            yaw_rate += self._rough_w
        else:
            self._rough_w *= 0.7

        # --- 横滑り: タイヤが出せなかったぶんが車体を外へ流す ---
        # ヨーレートを頭打ちにした時点で「曲がれない」ぶんの大半は表現できて
        # いるので、ここは残りの一部 (slide_share) だけを横滑りに回す。
        # 全部足すと二重計上になり、低μ路で不自然に横っ飛びする。
        a_slide = (a_y_req - a_y_tire) * self.slide_share
        self.vy -= a_slide * dt              # 曲がる向きと逆 = 外側へ
        damp = self.slide_damp * mu * g * dt
        if abs(self.vy) <= damp:
            self.vy = 0.0
        else:
            self.vy -= math.copysign(damp, self.vy)

        # --- 積分 ---
        self.v = v + a_x * dt
        if not self.armed:
            self.v = max(0.0, self.v)
        if self._wall_w != 0.0:
            self._wall_w *= max(0.0, 1.0 - dt / self.wall_w_tau)
            if abs(self._wall_w) < 1e-4:
                self._wall_w = 0.0
            yaw_rate += self._wall_w
        self.yaw_rate = yaw_rate
        self.yaw += yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.v * c - self.vy * s) * dt
        self.y += (self.v * s + self.vy * c) * dt

        # --- 壁の剛体衝突 ---
        if self.wall_collision:
            self._resolve_wall_contact(dt)
        # --- 車どうしの衝突 (2 台走行のときだけ) ---
        if self.car_col and self._rival is not None:
            self._resolve_car_contact(dt)

        # --- 車輪スリップ率 (オドメトリに乗る誤差) ---
        util_x = abs(a_cmd) / max(1e-6, a_x_pure)
        self._slip_ratio = min(0.9,
                               self.slip_elastic * min(1.0, util_x) ** 2
                               + self.slip_runaway * max(0.0, util_x - 1.0))
        self._a_lat = a_y_tire
        return math.hypot(self.v, self.vy) * dt * (1.0 if self.v >= 0 else -1.0)

    # =================================================================
    def _nearest_walls(self, px, py):
        """各点 (配列) から最も近い壁までの距離と、壁から点へ向く単位法線。"""
        w = np.asarray(self.segs, float)
        x0, y0, x1, y1 = w[:, 0], w[:, 1], w[:, 2], w[:, 3]
        dx, dy = x1 - x0, y1 - y0
        L2 = np.where(dx * dx + dy * dy < 1e-12, 1e-12, dx * dx + dy * dy)
        px = np.asarray(px, float)[:, None]
        py = np.asarray(py, float)[:, None]
        t = np.clip(((px - x0) * dx + (py - y0) * dy) / L2, 0.0, 1.0)
        qx, qy = x0 + t * dx, y0 + t * dy
        ex, ey = px - qx, py - qy
        d = np.hypot(ex, ey)
        i = np.argmin(d, axis=1)
        rows = np.arange(len(i))
        dmin = d[rows, i]
        nx, ny = ex[rows, i], ey[rows, i]
        nrm = np.where(dmin < 1e-9, 1.0, dmin)
        return dmin, nx / nrm, ny / nrm, i

    def _cb_rival(self, m):
        d = m.data
        if len(d) < 14:
            return
        now = time.monotonic()
        prev = self._rival
        if prev is not None and 1e-3 < now - prev[4] < 0.2:
            dy = math.atan2(math.sin(d[4] - prev[2]), math.cos(d[4] - prev[2]))
            self._rival_w = 0.7 * self._rival_w + 0.3 * dy / (now - prev[4])
        self._rival = (d[2], d[3], d[4], d[13], now)

    def _rival_pose(self):
        """受信からの経過ぶん進めた相手の姿勢 (x, y, yaw, v)。古ければ None。
        JetRacer の render_state の stamp は sim 時計なので、相手の sim と比べず受信時刻から進める。"""
        r = self._rival
        if r is None:
            return None
        age = time.monotonic() - r[4]
        if age > self.rival_timeout:
            return None
        rx, ry, ryaw, rv = r[0], r[1], r[2], r[3]
        w = self._rival_w
        if abs(w) > 1e-3:
            ny = ryaw + w * age
            rx += rv / w * (math.sin(ny) - math.sin(ryaw))
            ry += rv / w * (-math.cos(ny) + math.cos(ryaw))
            ryaw = ny
        else:
            rx += rv * math.cos(ryaw) * age
            ry += rv * math.sin(ryaw) * age
        return rx, ry, ryaw, rv

    def _resolve_car_contact(self, dt):
        """相手の車体と重なったら押し戻し・法線衝撃・摩擦 (壁と同じ剛体の式、同じ質量として半分ずつ)。"""
        pose = self._rival_pose()
        if pose is None:
            return
        rx, ry, ryaw, rv = pose
        R2 = 2.0 * self.car_r
        offs = self.car_offsets
        rc, rs = math.cos(ryaw), math.sin(ryaw)
        rpx = np.array([rx + rc * d for d in offs])
        rpy = np.array([ry + rs * d for d in offs])

        def contacts():
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            out = []
            for d in offs:
                px, py = self.x + c * d, self.y + s * d
                ex, ey = px - rpx, py - rpy
                dist = np.hypot(ex, ey)
                j = int(np.argmin(dist))
                pen = R2 - float(dist[j])
                if pen > 0.0:
                    nrm = max(1e-9, float(dist[j]))
                    out.append((d, float(ex[j]) / nrm, float(ey[j]) / nrm, pen))
            return out

        cs = contacts()
        if not cs:
            self._car_contact = False
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        v0 = self.v
        k_i = max(1e-6, (self.veh_len ** 2 + (2.0 * self.half_w) ** 2) / 12.0)
        x_cg = self.L * self.drive_load
        w = self.yaw_rate
        vbx, vby = self.v * c - self.vy * s, self.v * s + self.vy * c
        rcx, rcy = x_cg * c, x_cg * s
        vcx, vcy = vbx - w * rcy, vby + w * rcx
        vrx, vry = rv * rc, rv * rs
        vn_first = 0.0
        for _ in range(3):
            for d, n_x, n_y, pen in cs:
                self.x += n_x * pen * 0.5
                self.y += n_y * pen * 0.5
                p_x, p_y = self.x + c * d, self.y + s * d
                rxx, ryy = p_x - (self.x + rcx), p_y - (self.y + rcy)
                vpx, vpy = vcx - w * ryy - vrx, vcy + w * rxx - vry
                vn = vpx * n_x + vpy * n_y
                if vn < 0.0:
                    if vn_first == 0.0:
                        vn_first = vn
                    rn = rxx * n_y - ryy * n_x
                    jn = -0.5 * (1.0 + self.car_e) * vn / (1.0 + self.wall_couple * rn * rn / k_i)
                    vcx += jn * n_x
                    vcy += jn * n_y
                    w += self.wall_couple * rn * jn / k_i
                    t_x, t_y = -n_y, n_x
                    vpx, vpy = vcx - w * ryy - vrx, vcy + w * rxx - vry
                    vt = vpx * t_x + vpy * t_y
                    rt = rxx * t_y - ryy * t_x
                    jt = -0.5 * vt / (1.0 + self.wall_couple * rt * rt / k_i)
                    jt = max(-self.car_mu * jn, min(self.car_mu * jn, jt))
                    vcx += jt * t_x
                    vcy += jt * t_y
                    w += self.wall_couple * rt * jt / k_i
            cs = contacts()
            if not cs:
                break
        vbx, vby = vcx + w * rcy, vcy - w * rcx
        self.v = vbx * c + vby * s
        self.vy = -vbx * s + vby * c
        self._wall_w += w - self.yaw_rate
        self.yaw_rate = w
        if not self._car_contact:
            self._car_contact = True
            if vn_first < -0.05:
                self._car_hits += 1
                self.get_logger().info(
                    f"車両接触 #{self._car_hits}: 相対法線速度 {-vn_first:.2f} m/s, v={v0:.2f} → {self.v:.2f} m/s"
                    f" (自車 x={self.x:.2f}, y={self.y:.2f} / 相手 x={rx:.2f}, y={ry:.2f})")

    def _log_pose(self):
        pose = self._rival_pose()
        t = time.time()
        if pose is None:
            self._pose_log.write(f'{t:.3f},{self.x:.3f},{self.y:.3f},{self.yaw:.3f},{self.v:.3f},,,,,\n')
        else:
            rx, ry, ryaw, rv = pose
            self._pose_log.write(f'{t:.3f},{self.x:.3f},{self.y:.3f},{self.yaw:.3f},{self.v:.3f},'
                                 f'{rx:.3f},{ry:.3f},{ryaw:.3f},{rv:.3f},{math.hypot(rx - self.x, ry - self.y):.3f}\n')
        self._pose_log.flush()

    def _resolve_wall_contact(self, dt):
        """
        壁を剛体として扱う (minicarbattle2026 の vehicle_sim と同じ式)。
        車体の 3 ディスクのどれかが壁に食い込んでいたら、
          1. 車体ごと法線方向へ押し戻す
          2. 接触点の法線速度が壁へ向いていれば、それを消す衝撃を与える (反発係数 wall_restitution)
          3. 接線速度をクーロン摩擦 |J_t| ≤ μ J_n で減らす
        衝撃は重心まわりの剛体として並進と回転に配る (単位質量、Iz/m = (長さ²+幅²)/12)。
        回転は _wall_w として次ステップ以降のヨーレートに乗り、wall_yaw_tau_s で減衰する。
        """
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        v0 = self.v
        offs = self.wall_offsets
        px = [self.x + c * d for d in offs]
        py = [self.y + s * d for d in offs]
        dist, nx, ny, iw = self._nearest_walls(px, py)
        pen = self.wall_r - dist
        # 円の中心が壁の線を越えるほど食い込むと (角に斜めに速く当たる・薄い仕切り)、「中心 → 最寄りの壁の点」の
        # 向きが壁の向こうを向き、向こう側へ押し出されて通り抜けた。前のステップの中心と壁の同じ側に戻す
        # (minicarbattle2026 の vehicle_sim と同じ直し、2026-09-27)
        prev = getattr(self, '_disc_prev', None)
        if prev is not None and len(prev) == len(offs):
            w = np.asarray(self.segs, float)
            for k in range(len(offs)):
                x0, y0, x1, y1 = w[iw[k]]
                side = lambda qx, qy: (x1 - x0) * (qy - y0) - (y1 - y0) * (qx - x0)
                if side(*prev[k]) * side(px[k], py[k]) < 0.0 and dist[k] < self.wall_r * 2.0:
                    nx[k], ny[k] = -nx[k], -ny[k]
                    pen[k] = self.wall_r + dist[k]
        if not np.any(pen > 0.0):
            self._wall_contact = False
            self._disc_prev = list(zip(px, py))
            return
        k_i = max(1e-6, (self.veh_len ** 2 + (2.0 * self.half_w) ** 2) / 12.0)
        x_cg = self.L * self.drive_load             # 後軸から見た重心位置
        w = self.yaw_rate
        vbx, vby = self.v * c - self.vy * s, self.v * s + self.vy * c
        rcx, rcy = x_cg * c, x_cg * s
        vcx, vcy = vbx - w * rcy, vby + w * rcx
        vn_first = 0.0
        for _ in range(3):                          # 逐次衝撃法 (複数ディスク)
            for k, d in enumerate(offs):
                if pen[k] <= 0.0:
                    continue
                n_x, n_y = float(nx[k]), float(ny[k])
                self.x += n_x * pen[k]
                self.y += n_y * pen[k]
                p_x, p_y = self.x + c * d, self.y + s * d
                rx, ry = p_x - (self.x + rcx), p_y - (self.y + rcy)
                vpx, vpy = vcx - w * ry, vcy + w * rx
                vn = vpx * n_x + vpy * n_y
                if vn < 0.0:
                    if vn_first == 0.0:
                        vn_first = vn
                    rn = rx * n_y - ry * n_x
                    jn = -(1.0 + self.wall_e) * vn / (1.0 + self.wall_couple * rn * rn / k_i)
                    vcx += jn * n_x
                    vcy += jn * n_y
                    w += self.wall_couple * rn * jn / k_i
                    t_x, t_y = -n_y, n_x
                    vpx, vpy = vcx - w * ry, vcy + w * rx
                    vt = vpx * t_x + vpy * t_y
                    rt = rx * t_y - ry * t_x
                    jt = -vt / (1.0 + self.wall_couple * rt * rt / k_i)
                    jt = max(-self.wall_mu * jn, min(self.wall_mu * jn, jt))
                    vcx += jt * t_x
                    vcy += jt * t_y
                    w += self.wall_couple * rt * jt / k_i
            px = [self.x + c * d for d in offs]
            py = [self.y + s * d for d in offs]
            dist, nx, ny, iw = self._nearest_walls(px, py)
            pen = self.wall_r - dist
            if not np.any(pen > 1e-4):
                break
        vbx, vby = vcx + w * rcy, vcy - w * rcx
        self.v = vbx * c + vby * s
        self.vy = -vbx * s + vby * c
        self._wall_w += w - self.yaw_rate
        self.yaw_rate = w
        self._disc_prev = [(self.x + c * d, self.y + s * d) for d in offs]   # 押し戻した後の中心
        if not self._wall_contact:
            self._wall_contact = True
            if vn_first >= 0.0:
                return                    # 壁に触れているだけ (後退中など)。衝突には数えない
            self._wall_hits += 1
            self.get_logger().info(
                f"壁接触 #{self._wall_hits}: v={v0:.2f} m/s 法線速度 {-vn_first:.2f} m/s"
                f" → v={self.v:.2f} (x={self.x:.2f}, y={self.y:.2f})",
                throttle_duration_sec=0.5)

    def step(self):
        """realtime: タイマーから 1 ティック。"""
        self._tick()

    def _tick(self, dt=None):
        """物理 1 ティック。時間の進め方はここ 1 か所 (lockstep はこれを必要回数呼ぶ)。"""
        # --- 1 ステップの時間 ---
        dt = self.dt if dt is None else dt
        if self.realtime:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._last_step_t is not None:
                dt = now - self._last_step_t
                if dt > self.max_step_dt:
                    # 追いつけていない。積分が粗くなりすぎないよう頭打ちにする
                    # (この場合だけシム時間が実時間から遅れる)
                    if now - self._lag_warned > 5.0:
                        self.get_logger().warn(
                            f"シムが実時間に追いつけていません "
                            f"(dt={dt*1000:.0f}ms > {self.max_step_dt*1000:.0f}ms)")
                        self._lag_warned = now
                    dt = self.max_step_dt
                dt = max(dt, 1e-4)
            self._last_step_t = now
        self.sim_time += dt
        self._dbg_steps += 1

        # --- スタックを注入して脱出動作を検証する ---
        if self._stuck_xy is not None and self.sim_time >= self._stuck_xy[2] \
                and math.hypot(self.x - self._stuck_xy[0], self.y - self._stuck_xy[1]) < 0.20:
            self.stuck_at = self.sim_time
            self.get_logger().warn(f'スタック注入 @({self.x:.2f},{self.y:.2f}) t={self.sim_time:.1f}')
            self._stuck_xy = None
        stuck = (self.stuck_at > 0.0
                 and self.stuck_at <= self.sim_time < self.stuck_at + self.stuck_dur)

        # --- 指令 → アクチュエータ模型 (ここが「指令」と「実際に起きたこと」の境目) ---
        # 300 ms 失効・IDLE・ESTOP はブリッジと同じ挙動: 駆動を止め、舵は保持
        stale = (self._cmd_time < 0.0) or (self.sim_time - self._cmd_time > self.cmd_timeout)
        estop = self.sim_time < self._estop_until
        self.armed = (self.cmd_mode == ActuatorCmd.MODE_RUN) and not stale and not estop
        cmd_v = self.cmd_speed if self.armed else 0.0
        if stuck:
            cmd_v = 0.0
        if self.armed:
            tgt, brake, self._esc_state = self.esc.step(cmd_v, self.v, dt)
        else:
            self.esc.reset()
            tgt, brake = 0.0, True            # 失効時はブレーキ (実機の ESC は中立 = ブレーキ帯)
        self._esc_target, self._esc_brake = tgt, brake
        target_v = 0.0 if tgt is None else tgt
        # 舵はサーボの一次遅れとレート制限を通す (失効時は保持 = 指令を更新しない)
        self.steer = self.servo.step(self.cmd_steer_rad if not stale else self.servo.delta, dt)
        self._steer_rate = (self.steer - self._steer_prev) / dt
        self._steer_prev = self.steer
        self._v_prev = self.v
        self._vy_prev = self.vy
        ds_ground = self._integrate_motion(dt, target_v, stuck)
        # --- 剛体状態 (運動学的加速度と姿勢)。imu_sim の入力 ---
        vdot = (self.v - self._v_prev) / dt
        vydot = (self.vy - self._vy_prev) / dt
        self._ax_kin = vdot - self.yaw_rate * self.vy
        self._ay_kin = vydot + self.yaw_rate * self.v
        roll_new = self.roll_k * self._ay_kin
        pitch_new = self._pitch + self.pitch_k * self._ax_kin       # 坂の傾き + 加減速の鼻上げ/鼻下げ
        # 姿勢の変化率は一次遅れで平滑 (加速度の差分雑音を角速度に流し込まない)
        self._wx += ((roll_new - self._roll) / dt - self._wx) * min(1.0, dt / 0.05)
        self._wy += ((pitch_new - self._pitch_dyn) / dt - self._wy) * min(1.0, dt / 0.05)
        self._roll, self._pitch_dyn = roll_new, pitch_new

        # 車輪はグリップの使用率だけ空転し、オドメトリが多めに出る
        gim = self.course.gimmick_at(self.course.progress(self.x, self.y))
        slip = self._slip_ratio
        if stuck:
            slip = 0.9        # 空転している状態
            ds_wheel = 0.02 * dt
        else:
            ds_wheel = ds_ground / max(1e-6, (1.0 - slip))

        self.distance += abs(ds_ground)
        self._ds_wheel_pending = ds_wheel
        self._gimmick = gim
        self._slip = slip

        # --- 周回をまたいだら矢印信号を引き直す (本番は毎周指示が変わる) ---
        if self.arrow_random and self.arrow_each_lap:
            prog = self.course.progress(self.x, self.y)
            prev = getattr(self, '_prev_prog', prog)
            if prev > 0.8 and prog < 0.2:       # 1周した
                new_dir = int(self.rng.integers(1, 3))
                if new_dir != self.arrow_dir:
                    self.arrow_dir = new_dir
                self.get_logger().info(
                    f"矢印信号を更新: {_ARROW_DIR_NAMES[self.arrow_dir]}")
            self._prev_prog = prog

        self._update_opponent(dt)

        self.step_id += 1 if self.lockstep else 0
        self._update_track_state()
        self.publish_body_state()
        self.publish_odom()
        self.publish_ground_truth()

    # -----------------------------------------------------------------
    def _update_opponent(self, dt=None):
        """仮想他車をコースに沿って前進させ、真値ベースの相対量を更新する。"""
        self._opp_rel = None
        if not self.spawn_opp:
            return

        dt = self.dt if dt is None else dt
        own_s, _, _ = self.course.nearest(self.x, self.y)
        total = self.course.total_length
        if self.opp_s is None:
            self.opp_s = (own_s + self.opp_lead) % total

        # 他車を弧長方向へ前進
        self.opp_s = (self.opp_s + self.opp_speed * dt) % total

        # 車間 (弧長差, 周回ラップを考慮)
        gap = (self.opp_s - own_s) % total
        if gap > total / 2.0:
            gap -= total   # 後方にいる場合は負

        # 他車のワールド座標 (コース中心線 + 横オフセット)
        idx = int(np.searchsorted(self.course.cum_len, self.opp_s % total))
        idx = min(idx, len(self.course.center) - 1)
        ox, oy = self.course.center[idx]
        # 中心線の法線方向に横オフセット
        i2 = min(idx + 1, len(self.course.center) - 1)
        tx = self.course.center[i2][0] - self.course.center[idx][0]
        ty = self.course.center[i2][1] - self.course.center[idx][1]
        tn = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / tn, tx / tn
        ox += self.opp_lat * nx
        oy += self.opp_lat * ny

        # 自車 base_link 座標へ変換
        dx, dy = ox - self.x, oy - self.y
        lx = dx * math.cos(-self.yaw) - dy * math.sin(-self.yaw)
        ly = dx * math.sin(-self.yaw) + dy * math.cos(-self.yaw)

        self._opp_world = (ox, oy, math.atan2(ty, tx))
        self._opp_rel = {
            'gap': gap, 'lx': lx, 'ly': ly,
            'ahead': gap > 0.0 and lx > 0.0,
        }

    # =================================================================
    def publish_odom(self):
        # 物理は毎ステップ進めるが、配信は odom_rate_hz に間引く。
        # 次回時刻を period ずつ進める (sim_time で上書きすると、刻みの
        # 端数ぶん毎回遅れて実効レートが指定値を下回る)。
        # sim_time が巻き戻った場合の保険。呼び出し元が戻し忘れても
        # 配信が止まりっぱなしにならないようにする (止まると下流からは
        # 「車が消えた」ように見え、原因の特定が非常に難しい)。
        if self._t_odom_pub > self.sim_time:
            self._t_odom_pub = self.sim_time
            self._t_slow_pub = self.sim_time
        if self.sim_time - self._t_odom_pub < self.odom_period:
            return
        self._t_odom_pub += self.odom_period
        if self.sim_time - self._t_odom_pub > 2.0 * self.odom_period:
            self._t_odom_pub = self.sim_time      # 大きく遅れたら追従し直す
        slow = (self.sim_time - self._t_slow_pub) >= self.slow_period
        if slow:
            self._t_slow_pub += self.slow_period
            if self.sim_time - self._t_slow_pub > 2.0 * self.slow_period:
                self._t_slow_pub = self.sim_time
        stamp = self.get_clock().now().to_msg()
        q = (math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.yaw_rate
        self.pub_odom.publish(odom)

        # 走行距離は積算値なので低レートで足りる (購読は3ノード)
        if slow:
            d = Float64()
            d.data = self.distance
            self.pub_dist.publish(d)
            self._pub_f(self.pub_simtime, self.sim_time)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        self.tf.sendTransform(t)
        if self.pub_render is not None:
            self._publish_render_state(stamp)

        self._pub_f(self.pub_yaw, self.yaw)
        # ジャイロにはバイアスとノイズを載せる。積分だけでは必ずずれるので、
        # pose_fusion が地磁気でバイアスを推定できているかを検証できる。
        self._pub_f(self.pub_yaw_rate,
                    self.yaw_rate + self.gyro_bias
                    + float(self.rng.normal(0, self.gyro_noise)))
        # 横加速度はタイヤが実際に出した横力 (滑っている間は頭打ちになる)
        self._pub_f(self.pub_lat, getattr(self, '_a_lat', self.v * self.yaw_rate))

    # =================================================================
    # JetRacer sim: 剛体状態 / ground truth / lockstep
    # =================================================================
    def _now_msg(self):
        return self.get_clock().now().to_msg()

    def _publish_clock(self):
        if self.pub_clock is None:
            return
        c = Clock()
        c.clock.sec = int(self.sim_time)
        c.clock.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
        self.pub_clock.publish(c)

    def _surface_id(self):
        return self._surface_ids.get(surface_at(self.x, self.y), BodyState.SURFACE_CARPET)

    def publish_body_state(self):
        """物理 1 ティックごと (100 Hz)。imu_sim はこれだけを見て /imu を作る。"""
        self._publish_clock()
        m = BodyState()
        m.header.stamp = self._now_msg()
        m.header.frame_id = 'base_link'
        m.sim_time = float(self.sim_time)
        m.step_id = int(self.step_id)
        m.ax = float(self._ax_kin)
        m.ay = float(self._ay_kin)
        m.az = 0.0
        m.wx = float(self._wx)
        m.wy = float(self._wy)
        m.wz = float(self.yaw_rate)
        m.roll = float(self._roll)
        m.pitch = float(self._pitch_dyn)
        m.v = float(self.v)
        m.vy = float(self.vy)
        m.steer_rad = float(self.steer)
        m.surface = int(self._surface_id())
        m.rough_frac = float(getattr(self, '_rough_frac', 0.0))
        m.slip_ratio = float(self._slip_ratio)
        m.stuck = bool(self.stuck_at > 0.0 and
                       self.stuck_at <= self.sim_time < self.stuck_at + self.stuck_dur)
        self.pub_body.publish(m)

    def _update_track_state(self):
        """参照線に対する位置・周回・壁余裕 (ground_truth と StepInfo で共用)。"""
        s, d, psi, _ = self.ref.nearest(self.x, self.y)
        if self._s_prev is not None and self.ref.progress_delta(self._s_prev, s) > 0 \
                and self._s_prev > 0.8 * self.ref.total and s < 0.2 * self.ref.total:
            self.lap += 1
            self._lap_done = True
        self._s_prev = s
        self._track = (s, d, psi)
        self._min_clear = self.course.clearance(self.x, self.y)
        # 壁が剛体になったので余裕は collision_clear_m まで縮まないことがある。壁に接触している間も衝突とする
        self._collision = self._min_clear < self.collision_clear or self._wall_contact
        self._off_track = abs(d) > self.course.half + self.off_track_margin

    def publish_ground_truth(self):
        # 物理ティックの整数倍で間引く (sim_time の比較だと realtime の dt 揺れで 100 → 64〜87 Hz に落ちる)
        self._gt_tick = getattr(self, '_gt_tick', 0) + 1
        every = max(1, int(round(self.dt / self.gt_period)))
        if self._gt_tick % every:
            return
        s, d, psi = self._track
        ld = self.lookahead_m + self.lookahead_gain * max(0.0, self.v)
        lx, ly, lpsi, kappa = self.ref.point_at(s + ld)
        c, sn = math.cos(-self.yaw), math.sin(-self.yaw)
        bx = (lx - self.x) * c - (ly - self.y) * sn
        by = (lx - self.x) * sn + (ly - self.y) * c
        u, v, vis = self.cam_geom.project(bx, by, 0.0)
        m = GroundTruth()
        m.header.stamp = self._now_msg()
        m.header.frame_id = 'odom'
        m.sim_time = float(self.sim_time)
        m.step_id = int(self.step_id)
        m.x, m.y, m.yaw = float(self.x), float(self.y), float(self.yaw)
        m.v, m.vy, m.yaw_rate = float(self.v), float(self.vy), float(self.yaw_rate)
        m.steer_rad = float(self.steer)
        m.s_m = float(s)
        m.progress = float(s / self.ref.total)
        m.cte_m = float(d)
        m.heading_err_rad = float(self.ref.heading_error(self.yaw, psi))
        m.curvature = float(kappa)
        m.zone = int(self._surface_id())
        m.lap = int(self.lap)
        m.lookahead_m = float(ld)
        m.la_x, m.la_y = float(lx), float(ly)
        m.u = float(u) if vis else -1.0
        m.v_px = float(v) if vis else -1.0
        m.la_visible = bool(vis)
        m.min_wall_clear_m = float(self._min_clear)
        m.off_track = bool(self._off_track)
        m.collision = bool(self._collision)
        self.pub_gt.publish(m)

    # ----- lockstep -----
    def _cb_step_image(self, msg):
        self._last_img = msg
        self._img_event.set()

    def _cb_step_imu(self, msg):
        self._imu_since_step.append(msg)

    def _step_info(self, progress_m):
        s, d, psi = self._track
        info = StepInfo()
        info.header.stamp = self._now_msg()
        info.step_id = int(self.step_id)
        info.progress_m = float(progress_m)
        info.cte_m = float(d)
        info.heading_err_rad = float(self.ref.heading_error(self.yaw, psi))
        info.speed_mps = float(self.v)
        info.min_wall_clear_m = float(self._min_clear)
        info.steer_rate_rad_s = float(self._steer_rate)
        info.zone = int(self._surface_id())
        info.collision = bool(self._collision)
        info.off_track = bool(self._off_track)
        info.lap_done = bool(self._lap_done)
        info.terminated = bool(self._collision or self._off_track)
        info.truncated = False
        self._lap_done = False
        return info

    def srv_step(self, req, res):
        """1/control_hz ぶん進める。10 ms ティックの端数は累積器で持ち越す (3,3,4,3,3,4 …)。"""
        self.cb_actuator(req.action)
        self._cmd_time = self.sim_time          # step の指令は失効しない
        s0 = self._track[0] if self._s_prev is not None else self.ref.nearest(self.x, self.y)[0]
        self._imu_since_step = []
        self._img_event.clear()
        img_stamp0 = (self._last_img.header.stamp.sec + self._last_img.header.stamp.nanosec * 1e-9
                      if self._last_img is not None else -1.0)
        self._step_acc = getattr(self, '_step_acc', 0.0) + self.control_dt
        n_ticks = 0
        while self._step_acc >= self.dt - 1e-9:
            self._tick(self.dt)
            self._step_acc -= self.dt
            n_ticks += 1
        progress_m = self.ref.progress_delta(s0, self._track[0])
        # imu_sim (別プロセス) はティックごとに 1 サンプル出す。この step ぶんが届くまで少し待つ
        # (待たないと次の step に数えられ、学習側の窓がずれる)。取りこぼし (drop_prob) はタイムアウトで諦める
        t_end = time.monotonic() + 0.1
        while len(self._imu_since_step) < n_ticks and time.monotonic() < t_end:
            time.sleep(0.001)
        # 画像はカメラの周期 (1/cam_rate) でしか来ない。この step で 1 枚「来る予定」のときだけ待つ。
        # OpenCV 描画はここで同期的に描く。Unity は /sim/render_state (stamp = sim 時刻) を受けて描くので、
        # 新しい stamp の画像が返るまで待つ (step_image_timeout_s)。
        self._cam_acc += self.control_dt
        expect_image = False
        if self.use_camera and self._cam_acc >= 1.0 / self.cam_rate - 1e-9:
            self._cam_acc -= 1.0 / self.cam_rate
            expect_image = True
            if not self.unity_camera:
                self.publish_camera()
        if expect_image:
            t_end = time.monotonic() + self.step_img_timeout
            while time.monotonic() < t_end:
                if self._img_event.wait(0.005):
                    st = self._last_img.header.stamp
                    if st.sec + st.nanosec * 1e-9 > img_stamp0:
                        break
                    self._img_event.clear()
        res.image = self._last_img if self._last_img is not None else Image()
        res.imu = list(self._imu_since_step)
        res.info = self._step_info(progress_m)
        return res

    def srv_reset(self, req, res):
        """エピソードの開始。seed を配り、スポーン位置に戻す。"""
        self.episode_seed = int(req.seed)
        self.rng = np.random.default_rng(self.episode_seed)
        self.pub_episode.publish(UInt32(data=self.episode_seed))
        for k in req.override_keys:
            self.get_logger().warn(f"override '{k}' は未対応 (imu_sim の yaml を直接変えること)")
        spawn = (req.spawn or 'start').strip().lower()
        if spawn == 'random':
            s0 = float(self.rng.uniform(0.0, self.ref.total))
            lat = float(self.rng.uniform(-0.10, 0.10))
        elif spawn.startswith('zone:'):
            # 区間 ID の始点 (参照線上で最初にその区間になる弧長)
            want = int(spawn.split(':')[1])
            s0, lat = 0.0, 0.0
            for s_try in np.linspace(0.0, self.ref.total, 400, endpoint=False):
                px, py, _, _ = self.ref.point_at(s_try)
                if self._surface_ids.get(surface_at(px, py), 0) == want:
                    s0 = float(s_try)
                    break
        else:
            s0, lat = 0.0, 0.0
        px, py, psi, _ = self.ref.point_at(s0)
        self.x = px - lat * math.sin(psi)
        self.y = py + lat * math.cos(psi)
        self.yaw = psi
        self.v = self.vy = 0.0
        self._wall_w = 0.0
        self._wall_contact = False
        self.yaw_rate = 0.0
        self.steer = 0.0
        self.servo.reset()
        self.esc.reset()
        self.cmd_speed = 0.0
        self.cmd_steer_rad = 0.0
        self.cmd_mode = ActuatorCmd.MODE_IDLE
        self._a_x_prev = 0.0
        self._roll = self._pitch_dyn = self._wx = self._wy = 0.0
        self.distance = 0.0
        self.lap = 0
        self._s_prev = None
        self._lap_done = False
        self.stuck_at = float(self.get_parameter('inject_stuck_at_s').value)
        self._update_track_state()
        self.publish_body_state()
        self.publish_odom()
        self.publish_ground_truth()
        res.ok = True
        res.image = self._last_img if self._last_img is not None else Image()
        res.imu = list(self._imu_since_step[-50:])
        res.info = self._step_info(0.0)
        return res

    # =================================================================
    def publish_sensors(self):
        stamp = self.get_clock().now().to_msg()

        # --- 後方 (超音波 1 本)。LiDAR は前方マウントで後ろが死角なので、
        #     use_lidar の有無に関わらず必ず配信する。脱出の後退で使う。
        d_rear = raycast(self.segs, self.x, self.y, self.yaw + math.pi,
                         self.rear_sonar_max)
        opp = getattr(self, '_opp_rel', None)
        if opp is not None and not opp['ahead'] and abs(opp['ly']) < 0.18:
            d_rear = min(d_rear, math.hypot(opp['lx'], opp['ly']))
        self._pub_range(self.pub_sonar_rear, stamp, 'sonar_rear', d_rear,
                        self.rear_sonar_max, Range.ULTRASOUND)

        # 測距センサ。use_lidar のときは publish_lidar が /scan から仮想 ToF を
        # 生成して同じトピックに流すので、ここでは配信しない。
        if not self.use_lidar:
            # 左右 ToF (真横), 前方超音波
            d_l = raycast(self.segs, self.x, self.y, self.yaw + math.pi / 2, 1.3)
            d_r = raycast(self.segs, self.x, self.y, self.yaw - math.pi / 2, 1.3)
            d_f = raycast(self.segs, self.x, self.y, self.yaw, 4.0)
            # 前方45度の斜めToF。ビーム経路が長いので最大距離を延ばす (2.0m)。
            DIAG_MAX = 2.0
            d_fl = raycast(self.segs, self.x, self.y, self.yaw + math.pi / 4, DIAG_MAX)
            d_fr = raycast(self.segs, self.x, self.y, self.yaw - math.pi / 4, DIAG_MAX)
            if not self._sees_wall['tof']:        # ビームが板の帯の外 (壁の下を抜ける)
                d_l = d_r = 1.3
                d_fl = d_fr = DIAG_MAX
            if not self._sees_wall['sonar']:
                d_f = 4.0

            # 前方の他車を超音波に反映 (壁より近ければ他車が返る)
            opp = getattr(self, '_opp_rel', None)
            if opp is not None and opp['ahead'] and abs(opp['ly']) < 0.18:
                d_f = min(d_f, math.hypot(opp['lx'], opp['ly']))

            self._pub_range(self.pub_tof_l, stamp, 'tof_left', d_l, 1.3)
            self._pub_range(self.pub_tof_r, stamp, 'tof_right', d_r, 1.3)
            self._pub_range(self.pub_tof_fl, stamp, 'tof_fl', d_fl, DIAG_MAX)
            self._pub_range(self.pub_tof_fr, stamp, 'tof_fr', d_fr, DIAG_MAX)
            self._pub_range(self.pub_sonar, stamp, 'sonar_front', d_f, 4.0,
                            Range.ULTRASOUND)

        ds = getattr(self, '_ds_wheel_pending', 0.0)
        self._ds_wheel_pending = 0.0
        m = Float32()
        m.data = float(ds * 5.0)   # 20Hz publish 相当にまとめる
        self.pub_wheel.publish(m)

        st = VehicleStatus()
        st.header.stamp = stamp
        st.armed = self.armed
        st.tof_left_ok = True
        st.tof_right_ok = True
        st.sonar_ok = True
        st.rpm_ok = True
        slip = getattr(self, '_slip', 0.0)
        st.wheel_speed_mps = float(self.v / max(1e-6, 1.0 - slip))
        st.link_hz = 60.0
        self.pub_status.publish(st)

        self._pub_f(self.pub_gs, self.v)

        if self.pub_mag_on:
            self._publish_mag(stamp)

        # 振動とピッチ (運動モデルから出す)。
        # 振動は「でこぼこに乗っている車輪の割合 x 速度」で決まる。止まって
        # いれば揺れないので、surface_estimator が速度を見ずに判定していると
        # ここで露見する (実機と同じ)。
        gim = getattr(self, '_gimmick', None)
        rough = getattr(self, '_rough_frac', 0.0)
        vib = 0.4 + 3.2 * rough * min(1.0, abs(self.v) / 1.5)
        self._pub_f(self.pub_vib, vib + float(self.rng.normal(0, 0.1)))
        self._pub_f(self.pub_pitch, getattr(self, '_pitch', 0.0))
        self._pub_f(self.pub_mu, getattr(self, '_mu_now', 0.85))
        self._pub_f(self.pub_sideslip, self.vy)
        self._pub_f(self.pub_wslip, self._slip_ratio)

        if self.use_camera and getattr(self, '_cam_brightness', None) is not None:
            # 合成カメラを回しているときは、実際に描いた画像の平均輝度を出す
            bright = self._cam_brightness
        else:
            bright = 25.0 if gim == 'TUNNEL' else 130.0
            if gim == 'LIGHT_DISTURB':
                bright = 130.0 + 60.0 * math.sin(self.sim_time * 6.0)
        self._pub_f(self.pub_bright, bright)

        if self.spawn_opp and self.pub_opp_info:
            self._publish_opponent_info(stamp)

    def _publish_opponent_info(self, stamp):
        """真値ベースで /opponent_info を publish (opponent_detector の代替)。"""
        m = OpponentInfo()
        m.header.stamp = stamp
        m.header.frame_id = 'base_link'
        opp = getattr(self, '_opp_rel', None)

        # コリドー半幅 (中心線からの走行可能幅の半分)
        half = self.course.half
        if opp is None or not opp['ahead'] or opp['lx'] > 1.2:
            m.detected = False
            m.stable_detected = False
            self.pub_opp.publish(m)
            return

        lx, ly = opp['lx'], opp['ly']
        m.detected = True
        m.stable_detected = True
        m.stable_count = 10
        m.confidence = 0.9
        m.distance_m = float(math.hypot(lx, ly))
        m.bearing_rad = float(math.atan2(ly, max(0.05, lx)))
        m.lateral_offset_m = float(ly)
        m.opponent_width_m = 0.2
        # 他車の左右の空き = コリドー半幅から他車の端までの距離。
        # ly は他車の横位置(+左)。左の空き = 左壁(+half) - 他車左端。
        m.gap_left_m = float(max(0.0, (half - ly) - 0.1))
        m.gap_right_m = float(max(0.0, (half + ly) - 0.1))
        closing = self.v - self.opp_speed
        m.closing_speed_mps = float(closing)
        m.rel_speed_mps = float(self.opp_speed)
        if ly > 0.06:
            m.rel_position = OpponentInfo.RELPOS_LEFT
        elif ly < -0.06:
            m.rel_position = OpponentInfo.RELPOS_RIGHT
        else:
            m.rel_position = OpponentInfo.RELPOS_AHEAD
        self.pub_opp.publish(m)

    # =================================================================
    # 合成カメラ (非魚眼ピンホール投影)
    # =================================================================
    def _arrow_gate_position(self):
        """
        ⑥矢印標識 (LED 掲示板) の中心の世界座標 (x,y,z)。

        走行ラインの上ではなく、狭い道をまたぐ門型フレームの中央に来る
        (course.ARROW_SIGN)。支柱は左右の紅白の板の上に立つので、
        掲示板の中心は道の中央 = 仕切りの真上になる。
        """
        s = ARROW_SIGN
        y0, y1 = s['post_y']
        return np.array([s['x'], (y0 + y1) / 2.0,
                         s['board_z0'] + s['board_h'] / 2.0])

    def _setup_render_canvas(self, g):
        """OpenCV 描画のキャンバス。歪みがあると出力の端はピンホールでより外側を見ているので、
        その範囲 (CamGeom.undistorted_extent) をピンホールで描いてから cv2.remap で出力へ写す。"""
        if not g.distorted:
            self._rw, self._rh = self.cam_w, self.cam_h
            self._rfx, self._rfy, self._rcx, self._rcy = g.fx, g.fy, g.cx, g.cy
            self._remap = None
            return
        x0, x1, y0, y1 = g.undistorted_extent()
        pad = 2
        rw = int(math.ceil((x1 - x0) * g.fx)) + 2 * pad
        rh = int(math.ceil((y1 - y0) * g.fy)) + 2 * pad
        if rw > 4 * self.cam_w or rh > 4 * self.cam_h:
            self.get_logger().warn(f"歪みが強くキャンバスが大きい ({rw}×{rh})。4 倍で打ち切る (端が伸びる)")
            rw, rh = min(rw, 4 * self.cam_w), min(rh, 4 * self.cam_h)
        self._rw, self._rh = rw, rh
        self._rfx, self._rfy = g.fx, g.fy
        self._rcx, self._rcy = -x0 * g.fx + pad, -y0 * g.fy + pad
        us, vs = np.meshgrid(np.arange(self.cam_w, dtype=np.float64) + 0.5,
                             np.arange(self.cam_h, dtype=np.float64) + 0.5)
        xu, yu = g.undistort((us - g.cx) / g.fx, (vs - g.cy) / g.fy)
        self._remap = ((xu * g.fx + self._rcx - 0.5).astype(np.float32),
                       (yu * g.fy + self._rcy - 0.5).astype(np.float32))
        th, tv = g.true_fov_deg()
        self.get_logger().info(f"カメラ: fx {g.fx:.1f} fy {g.fy:.1f} k1 {g.k1:+.3f} k2 {g.k2:+.3f} → "
                               f"見込み角 水平 {th:.0f}°・垂直 {tv:.0f}° (キャンバス {rw}×{rh})")

    def _cam_basis(self):
        """カメラ姿勢: 車の yaw ＋ 下向き pitch。x=右, y=下, z=前 の正規直交基底。"""
        psi, phi = self.yaw, self.cam_pitch
        z = np.array([math.cos(psi) * math.cos(phi),
                      math.sin(psi) * math.cos(phi), -math.sin(phi)])
        x = np.array([math.sin(psi), -math.cos(psi), 0.0])
        y = np.cross(z, x)                      # 下向き
        C = np.array([self.x, self.y, self.cam_hm])
        return C, x, y, z

    def _project(self, pts):
        """世界点(Nx3) → 画像座標(Nx2) と カメラ前方深さ zc(N)。"""
        C, xh, yh, zh = self._cam_basis()
        v = np.asarray(pts, float) - C
        zc = v @ zh
        safe = np.where(np.abs(zc) < 1e-6, 1e-6, zc)
        u = self._rcx + self._rfx * (v @ xh) / safe
        w = self._rcy + self._rfy * (v @ yh) / safe
        return np.stack([u, w], axis=1), zc

    def _horizon_row(self):
        """地平線(遠方の地面)の画像行。上=背景, 下=カーペットで塗り分けるのに使う。"""
        far = np.array([[self.x + 500.0 * math.cos(self.yaw),
                         self.y + 500.0 * math.sin(self.yaw), 0.0]])
        uv, zc = self._project(far)
        if zc[0] <= 0:
            return self._rh // 3
        return int(np.clip(uv[0, 1], 0, self._rh))

    # -----------------------------------------------------------------
    # 赤白ウォールとカーペット
    # -----------------------------------------------------------------
    def _build_wall_tiles(self):
        """
        壁線分を描画用のタイルに刻んでおく (描画のたびに刻まないため)。

        色は course.WALL_COLORS (レギュレーション p.24 の図面から読んだ
        板ごとの色) を引き継ぐ。実物は赤板・白板を 1 枚ずつ並べたもので、
        1 枚の板が赤白に塗り分けられているわけではない。
        """
        a, b, jit, col = [], [], [], []
        colors = getattr(self.course, 'wall_colors', None)
        for i, (x0, y0, x1, y1) in enumerate(self.segs):
            c = 0 if (colors is not None and i < len(colors)
                      and colors[i] == 'red') else 1
            length = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(round(length / self.wall_tile)))
            for k in range(n):
                t0, t1 = k / n, (k + 1) / n
                a.append((x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0))
                b.append((x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1))
                # 板ごとの汚れ/日焼けのばらつき (検出器を一様色に頼らせない)
                jit.append(1.0 + 0.05 * math.sin(7.1 * i + 2.3 * k))
                col.append(c)
        self.wall_a = np.array(a, float)
        self.wall_b = np.array(b, float)
        self.wall_jit = np.array(jit, float)
        self.wall_col = np.array(col, int)      # 0=赤板 / 1=白板
        self.wall_mid = (self.wall_a + self.wall_b) / 2.0

    def _draw_walls(self, img):
        """
        ウォールを板ごとの四角形として遠い順に描く (画家のアルゴリズム)。

        色はレギュレーションどおり板ごとの単色 (赤板 / 白板)。
        lane_detector は赤(HSV)と白(HSV)を別々に壁として拾うので、
        どちらも画面に出ることが sim 検証の前提になる。
        """
        cv2 = self.cv2
        C, xh, yh, zh = self._cam_basis()

        # 近傍の板だけに絞る
        d2 = np.sum((self.wall_mid - np.array([self.x, self.y])) ** 2, axis=1)
        sel = np.where(d2 < self.render_range ** 2)[0]
        if sel.size == 0:
            return
        A, B = self.wall_a[sel], self.wall_b[sel]

        # カメラ前方 (zc>eps) の区間だけ残すよう線分をクリップする。
        # 板の上端の方が zc が小さいので、上端で判定すれば四隅とも前方になる。
        eps = 0.08
        base = -C[0] * zh[0] - C[1] * zh[1] + (self.wall_h - C[2]) * zh[2] - eps
        za = A[:, 0] * zh[0] + A[:, 1] * zh[1] + base
        zb = B[:, 0] * zh[0] + B[:, 1] * zh[1] + base
        dz = zb - za
        with np.errstate(divide='ignore', invalid='ignore'):
            t_cross = np.where(np.abs(dz) > 1e-9, -za / dz, np.inf)
        t_lo = np.where((za < 0) & (dz > 0), t_cross, 0.0)
        t_hi = np.where((zb < 0) & (dz < 0), t_cross, 1.0)
        keep = ((za > 0) | (zb > 0)) & (t_hi > t_lo)
        if not np.any(keep):
            return
        A, B = A[keep], B[keep]
        t_lo, t_hi = t_lo[keep][:, None], t_hi[keep][:, None]
        jit = self.wall_jit[sel][keep]
        cols = self.wall_col[sel][keep]
        P0 = A + (B - A) * t_lo
        P1 = A + (B - A) * t_hi

        # 板 1 枚 = 上端 2 点 + 下端 2 点
        n = P0.shape[0]
        z_top = np.full((n, 1), self.wall_h)
        z_bot = np.full((n, 1), self.wall_base)      # 板の下 30 mm は床が見える
        pts = np.concatenate([
            np.hstack([P0, z_top]), np.hstack([P1, z_top]),
            np.hstack([P1, z_bot]), np.hstack([P0, z_bot]),
        ], axis=1).reshape(n * 4, 3)
        uv, zc = self._project(pts)
        uv = uv.reshape(n, 4, 2)
        depth = zc.reshape(n, 4).mean(axis=1)

        # 画面外の板を捨てる
        umin, umax = uv[:, :, 0].min(axis=1), uv[:, :, 0].max(axis=1)
        vmin, vmax = uv[:, :, 1].min(axis=1), uv[:, :, 1].max(axis=1)
        on_screen = (umax > 0) & (umin < self._rw) & (vmax > 0) & (vmin < self._rh)

        order = np.argsort(-depth)
        for i in order:
            if not on_screen[i]:
                continue
            # 遠いほど暗く (露出とコントラストの手当てを検出器に試させる)
            shade = float(np.clip(1.05 - 0.06 * depth[i], 0.6, 1.0)) * jit[i]
            q = uv[i].astype(np.int32)
            if cols[i] == 0:
                c = (40 * shade, 40 * shade, 176 * shade)      # 赤板
            else:
                c = (236 * shade, 238 * shade, 238 * shade)    # 白板
            cv2.fillConvexPoly(img, q, c)

    # 路面の色 (BGR)。実物の見え方に寄せる。
    _AREA_COLOR = {
        'MU_HIGH': (60, 120, 60),      # 人工芝 (緑)
        'MU_LOW': (222, 224, 228),     # 滑り板 (白っぽい)
        'ROUGH': (237, 192, 76),       # でこぼこ道 (青い風呂マット4枚 p.31)
        'SLOPE': (196, 198, 200),      # 坂道の白ボード
    }
    _SLOT_COLOR = {'green': (70, 160, 70), 'red': (60, 60, 200),
                   'blue': (200, 90, 60)}

    def _build_ground_cells(self):
        """
        ギミック路面と駐車枠テープを 0.20m のセルに割って、世界座標のまま
        1 つの配列にまとめておく (静的なので起動時に 1 回だけ)。

        セルに割る理由は、大きな面がカメラ近接面をまたぐため (壁と同じ)。
        毎フレーム作り直すと 1 枚 41ms かかっていた (射影呼び出しが
        セルごとに 400 回以上)。ここでまとめておいて、描画側は
        **全セルを 1 回の射影で処理する**。
        """
        cell = 0.20
        areas = [(x0, y0, x1, y1, self._AREA_COLOR[n])
                 for n, x0, y0, x1, y1 in GIMMICK_AREAS
                 if n in self._AREA_COLOR]
        # ⑧駐車枠は「色テープの枠線」であって塗りつぶしではない (幅5cm)。
        # parking_detector はこのテープを見るので、面で塗ると検証にならない。
        tape = 0.05
        for name, col, sx0, sy0, sx1, sy1 in PARKING_SLOTS:
            c = self._SLOT_COLOR.get(col, (200, 200, 200))
            areas += [(sx0, sy0, sx0 + tape, sy1, c),
                      (sx1 - tape, sy0, sx1, sy1, c),
                      (sx0, sy0, sx1, sy0 + tape, c),
                      (sx0, sy1 - tape, sx1, sy1, c)]

        palette, quads, cidx = [], [], []
        for x0, y0, x1, y1, color in areas:
            if color not in palette:
                palette.append(color)
            ci = palette.index(color)
            nx = max(1, int(round((x1 - x0) / cell)))
            ny = max(1, int(round((y1 - y0) / cell)))
            xs = np.linspace(x0, x1, nx + 1)
            ys = np.linspace(y0, y1, ny + 1)
            for i in range(nx):
                for j in range(ny):
                    quads.append([[xs[i], ys[j]], [xs[i + 1], ys[j]],
                                  [xs[i + 1], ys[j + 1]], [xs[i], ys[j + 1]]])
                    cidx.append(ci)
        self.ground_quads = np.array(quads, float)          # (N,4,2)
        self.ground_center = self.ground_quads.mean(axis=1)  # (N,2)
        self.ground_cidx = np.array(cidx, int)
        self.ground_palette = palette

    def _draw_ground_areas(self, img):
        """
        ギミックの路面と駐車枠を地面に描く。

        床が一様グレーだと、lane_detector の床色モデル (直前の床から
        学習する) が④人工芝や②坂道の白ボードに乗ったときの挙動を
        検証できない。⑧駐車枠の色テープも同じ理由で要る。
        """
        cv2 = self.cv2
        if getattr(self, 'ground_quads', None) is None:
            self._build_ground_cells()

        # 自車が乗っている路面で、地平線より下の下地を塗る。
        # セル分割だけだと、カメラ近接面をまたぐ足元のセルが描けず
        # (四隅のどれかが後方になるため)、一番大きく写る手前がグレーの
        # ままになってしまう。
        here = surface_at(self.x, self.y)
        base = self._AREA_COLOR.get(here)
        if base is not None:
            horizon = int(np.clip(self._horizon_row(), 0, self._rh))
            img[horizon:] = base

        # 近いセルだけに絞る → 残り全部を 1 回で射影する
        d2 = np.sum((self.ground_center - np.array([self.x, self.y])) ** 2,
                    axis=1)
        idx = np.where(d2 < (self.render_range + 2.0) ** 2)[0]
        if idx.size == 0:
            return
        q = self.ground_quads[idx]                      # (M,4,2)
        pts = np.concatenate(
            [q.reshape(-1, 2), np.zeros((q.size // 2, 1))], axis=1)
        uv, zc = self._project(pts)
        uv = uv.reshape(-1, 4, 2)
        zc = zc.reshape(-1, 4)

        ok = np.all(zc > 0.08, axis=1)                  # 四隅ともカメラ前方
        umin, umax = uv[:, :, 0].min(axis=1), uv[:, :, 0].max(axis=1)
        vmin, vmax = uv[:, :, 1].min(axis=1), uv[:, :, 1].max(axis=1)
        ok &= (umax >= 0) & (umin <= self._rw) & \
              (vmax >= 0) & (vmin <= self._rh)
        if not np.any(ok):
            return
        # 遠いセルから描く (画家のアルゴリズム。重なりの前後を正しく)
        order = np.argsort(-d2[idx][ok])
        poly = np.clip(uv[ok][order], -1e4, 1e4).astype(np.int32)
        cols = self.ground_cidx[idx][ok][order]
        for ci, color in enumerate(self.ground_palette):
            sel = poly[cols == ci]
            if sel.size:
                cv2.fillPoly(img, sel, color)

    def _draw_floor_speckle(self, img):
        """
        パンチカーペットの斑点を世界座標に固定して描く。

        床が一様グレーだと lane_detector の位相相関 (対地速度) が効かない。
        模様が世界に固定されていることが重要なので、格子点のハッシュから
        濃淡を決める (フレーム間で同じ点は同じ濃さ)。
        """
        pitch = self.speckle_pitch
        rng_m = min(self.render_range, 2.4)
        i0 = int(math.floor((self.x - rng_m) / pitch))
        i1 = int(math.ceil((self.x + rng_m) / pitch))
        j0 = int(math.floor((self.y - rng_m) / pitch))
        j1 = int(math.ceil((self.y + rng_m) / pitch))
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1))
        ii, jj = ii.ravel(), jj.ravel()
        # 整数ハッシュ → [-1,1) の濃淡
        h = ((ii * 73856093) ^ (jj * 19349663)) & 0xFFFF
        shade = (h / 32768.0) - 1.0

        pts = np.stack([ii * pitch, jj * pitch, np.zeros(ii.size)], axis=1)
        uv, zc = self._project(pts)
        ok = (zc > 0.12)
        u = np.rint(uv[:, 0]).astype(int)
        v = np.rint(uv[:, 1]).astype(int)
        ok &= (u >= 0) & (u < self._rw) & (v >= 0) & (v < self._rh)
        if not np.any(ok):
            return
        u, v, shade = u[ok], v[ok], shade[ok]
        delta = (shade * 11.0).astype(np.int16)
        base = img[v, u].astype(np.int16)
        img[v, u] = np.clip(base + delta[:, None], 0, 255).astype(np.uint8)

    def _arrow_texture(self):
        """
        矢印LEDボードのテクスチャ。

        実物は 93cm x 19cm の LED 電光掲示板で、96x16 のドットマトリクス
        (parking-signal.pdf に点灯パターンの図がある)。ベタ塗りではなく
        **ドットの粒が見える** ので、そのように描く。粒の間の黒があることで
        検出器から見た彩度・輝度の分布も実物に近くなる。

        図柄 (parking-signal.pdf):
          左 = シアン … 左に斜めの頭 + 右へ伸びる帯
          右 = マゼンタ … その左右反転
          中 = イエロー … 上向き矢印 (直進)
        """
        cv2 = self.cv2
        cols, rows = 96, 16          # LED の並び
        cell = 4                     # 1 粒あたりの画素
        tw, th = cols * cell, rows * cell
        board = np.full((th, tw, 3), 8, np.uint8)     # 消灯部 (ほぼ黒)

        grid = np.zeros((rows, cols), np.uint8)

        def poly(pts):
            cv2.fillPoly(grid, [np.array([[int(x * cols), int(y * rows)]
                                          for x, y in pts], np.int32)], 1)

        d = self.arrow_dir
        if d == ArrowSign.DIR_LEFT:
            col = (255, 255, 0)                        # BGR: シアン
            poly([(0.04, 1.00), (0.21, 0.00), (0.21, 1.00)])
            poly([(0.21, 0.42), (0.96, 0.42), (0.96, 1.00), (0.21, 1.00)])
        elif d == ArrowSign.DIR_RIGHT:
            col = (255, 0, 255)                        # マゼンタ
            poly([(0.96, 1.00), (0.79, 0.00), (0.79, 1.00)])
            poly([(0.04, 0.42), (0.79, 0.42), (0.79, 1.00), (0.04, 1.00)])
        else:
            col = (0, 255, 255)                        # イエロー (直進)
            poly([(0.55, 0.00), (0.30, 0.62), (0.80, 0.62)])
            poly([(0.03, 0.62), (0.97, 0.62), (0.97, 0.72), (0.03, 0.72)])
            poly([(0.42, 0.62), (0.68, 0.62), (0.68, 1.00), (0.42, 1.00)])

        # 点灯セルを「粒」として描く (周囲 1px を黒く残す)
        ys, xs = np.nonzero(grid)
        for gy, gx in zip(ys, xs):
            y0, x0 = gy * cell, gx * cell
            board[y0:y0 + cell - 1, x0:x0 + cell - 1] = col
        return board

    def _draw_posts(self, img, bx):
        """
        矢印標識を支える SUS フレーム (支柱と上桟)。

        これが無いと掲示板が宙に浮いて見える。実物は 100cm+45cm の SUS 材を
        連結した高さ 155cm の門型フレームで、**支柱は狭い道の左右の紅白の板
        の上に立つ** (レギュレーション p.30 の写真: 30cm の足を板にクランプ)。
        走路の中には柱は無く、車は掲示板の下をくぐる。
        """
        cv2 = self.cv2
        post_h = ARROW_SIGN['post_h']
        r = ARROW_SIGN['post_r']        # SUS パイプの見かけの太さ (片側)
        py0, py1 = ARROW_SIGN['post_y']
        for py in (py0, py1):
            quad = np.array([[bx, py - r, post_h], [bx, py + r, post_h],
                             [bx, py + r, 0.0], [bx, py - r, 0.0]])
            uv, zc = self._project(quad)
            if np.any(zc <= 0.08):
                continue
            cv2.fillConvexPoly(img, uv.astype(np.int32), (150, 152, 155))
        # 上桟 (左右の支柱をつなぐ)
        top = np.array([[bx, py0, post_h], [bx, py1, post_h],
                        [bx, py1, post_h - 2 * r], [bx, py0, post_h - 2 * r]])
        uv, zc = self._project(top)
        if not np.any(zc <= 0.08):
            cv2.fillConvexPoly(img, uv.astype(np.int32), (150, 152, 155))

    def _draw_arrow_board(self, img):
        """矢印標識を世界に置いて画像へ透視貼り付け。"""
        cv2 = self.cv2
        bx, by, bz = self.arrow_gate_xyz
        hw, hh = ARROW_SIGN['board_w'] / 2.0, ARROW_SIGN['board_h'] / 2.0
        self._draw_posts(img, bx)
        # 車の左(-y側)がテクスチャ左になるよう TL,TR,BR,BL を並べる
        corners = np.array([
            [bx, by - hw, bz + hh],   # top-left  (車から見て左上)
            [bx, by + hw, bz + hh],   # top-right
            [bx, by + hw, bz - hh],   # bottom-right
            [bx, by - hw, bz - hh],   # bottom-left
        ])
        uv, zc = self._project(corners)
        if np.any(zc <= 0.05):
            return
        dst = uv.astype(np.float32)
        # 画面から大きく外れていれば描かない
        if dst[:, 0].max() < 0 or dst[:, 0].min() > self._rw or \
           dst[:, 1].max() < 0 or dst[:, 1].min() > self._rh:
            return
        # 裏側は黒いプラダン (レギュレーション p.30 の背面図)。表を貼ったままだと
        # 通過後に後ろから矢印が見えてしまい、検出器が誤った向きを掴む。
        # 掲示板は狭い道の入口を向いている (+x 側から読む)。
        if self.x < bx:
            tex = np.full((16, 96, 3), 26, np.uint8)
        else:
            tex = self._arrow_texture()
        th, tw = tex.shape[:2]
        src = np.array([[0, 0], [tw - 1, 0], [tw - 1, th - 1], [0, th - 1]], np.float32)
        Hm = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(tex, Hm, (self._rw, self._rh))
        mask = cv2.warpPerspective(np.full((th, tw), 255, np.uint8), Hm,
                                   (self._rw, self._rh))
        img[mask > 0] = warped[mask > 0]

    def _draw_opponents(self, img):
        """他車を暗い箱として描画 (追い抜き検証の土台)。"""
        opp = getattr(self, '_opp_rel', None)
        if opp is None or not opp['ahead']:
            return
        cv2 = self.cv2
        # 他車の世界座標を自車相対から復元
        lx, ly = opp['lx'], opp['ly']
        wx = self.x + lx * math.cos(self.yaw) - ly * math.sin(self.yaw)
        wy = self.y + lx * math.sin(self.yaw) + ly * math.cos(self.yaw)
        w2, hgt = 0.10, 0.14
        corners = np.array([
            [wx, wy - w2, hgt], [wx, wy + w2, hgt],
            [wx, wy + w2, 0.0], [wx, wy - w2, 0.0]])
        uv, zc = self._project(corners)
        if np.any(zc <= 0.05):
            return
        pts = uv.astype(np.int32)
        cv2.fillConvexPoly(img, pts, (40, 40, 45))

    def _scene_light(self):
        """
        区間ごとの見え方 (ゲインと色かぶり)。/camera/brightness と辻褄を合わせる。

        ①トンネル: 漆黒シートで暗く色も飛ぶ → lane_detector は valid=false を
        返し、BT が ToF の壁沿い走行へ切り替わるはず、という筋書きを検証できる。
        ③ライトかく乱: ステージライトで明滅＋色かぶり。
        """
        gim = getattr(self, '_gimmick', None)
        if gim == 'TUNNEL':
            return 0.16, (1.0, 1.0, 1.0), 0.75
        if gim == 'LIGHT_DISTURB':
            gain = 1.0 + 0.46 * math.sin(self.sim_time * 6.0)
            # 7 パターンの照明色を順に当てる (大会仕様)
            tints = [(1.0, 1.0, 1.0), (1.35, 0.85, 0.85), (0.85, 1.3, 0.85),
                     (0.85, 0.85, 1.35), (1.25, 1.2, 0.8), (0.8, 1.2, 1.25),
                     (1.3, 0.85, 1.25)]
            return gain, tints[int(self.sim_time / 2.0) % 7], 0.0
        return 1.0, (1.0, 1.0, 1.0), 0.0

    def publish_camera(self):
        cv2 = self.cv2
        # 誰も見ていないなら描かない (検出器を止めた実行を軽くするため)
        if self.pub_cam.get_subscription_count() == 0:
            return
        W, H = self._rw, self._rh
        img = np.empty((H, W, 3), np.uint8)
        horizon = int(np.clip(self._horizon_row(), 0, H))
        img[:horizon] = (110, 110, 110)      # 会場(中庸グレー)
        img[horizon:] = (95, 95, 92)         # パンチカーペット(濃いグレー)
        # ギミック路面と駐車枠 → その上に斑点 (カーペットの模様)
        self._draw_ground_areas(img)
        if self.floor_speckle:
            self._draw_floor_speckle(img)
        if self.draw_walls:
            self._draw_walls(img)
        self._draw_opponents(img)
        self._draw_arrow_board(img)
        # ピンホールのキャンバス → 歪んだ出力画像 (樽型・fx≠fy)。歪み無しなら恒等なので飛ばす
        if self._remap is not None:
            img = cv2.remap(img, self._remap[0], self._remap[1], cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_REPLICATE)

        # --- 区間ごとの照明 (①トンネルの暗転 / ③ライトかく乱) ---
        gain, tint, desat = self._scene_light()
        if gain != 1.0 or tint != (1.0, 1.0, 1.0) or desat > 0.0:
            f = img.astype(np.float32)
            if desat > 0.0:
                gray = f.mean(axis=2, keepdims=True)
                f = (1.0 - desat) * f + desat * gray
            f *= gain * np.array(tint, np.float32)
            img = np.clip(f, 0, 255).astype(np.uint8)

        # 軽いノイズ (検出器のロバスト性を試す)
        if self.noise > 0:
            n = self._noise_bank[self._noise_i]
            self._noise_i = (self._noise_i + 1) % len(self._noise_bank)
            img = cv2.add(img, n, dtype=cv2.CV_8U)

        # 実機の camera_node は見物人よけに上端を捨ててから配信する。
        # perception.yaml の roi_top/roi_bottom はクロップ後の高さ基準なので、
        # sim でも同じ割合を捨てないと矢印の ROI が sim と実機でずれる。
        if self._cam_crop_top_px:
            img = img[self._cam_crop_top_px:]

        # 輝度は合成画像の実測値を流す (zone_estimator のトンネル判定と整合させる)
        self._cam_brightness = float(img.mean())
        msg = self.cam_bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub_cam.publish(msg)

    # =================================================================
    # Unity 描画 (camera_backend:=unity)
    # =================================================================
    RENDER_STATE_FIELDS = (
        'stamp_sec', 'stamp_nanosec', 'x', 'y', 'yaw', 'sim_time',
        'arrow_dir', 'narrow_divider', 'pitch',
        'opp_valid', 'opp_x', 'opp_y', 'opp_yaw',
        'v', 'steer', 'a_lat',          # 車体の見た目 (タイヤ回転・前輪の切れ角・ロール)
        'distance',                     # 走行距離 (レース表示の順位用)
        'step_id', 'car_id')            # ★予約 (lockstep の描画同期と N 台並列用。Unity は当面無視してよい)

    def _publish_render_state(self, stamp):
        opp = getattr(self, '_opp_world', None) if self.spawn_opp else None
        m = Float64MultiArray()
        m.data = [float(stamp.sec), float(stamp.nanosec),
                  float(self.x), float(self.y), float(self.yaw),
                  float(self.sim_time), float(self.arrow_dir),
                  1.0 if self.course.narrow_divider else 0.0,
                  float(getattr(self, '_pitch', 0.0)),
                  1.0 if opp is not None else 0.0,
                  *(opp if opp is not None else (0.0, 0.0, 0.0)),
                  float(self.v), float(self.steer),
                  float(getattr(self, '_a_lat', 0.0)),
                  float(self.distance),
                  float(self.step_id), float(self.car_id)]
        self.pub_render.publish(m)

    def cb_unity_image(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8') or not msg.data:
            return
        # 平均輝度だけ欲しいので 4 画素おきに間引く (320x216 で 1/16)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        self._cam_brightness = float(img[::4, ::4].mean())

    # =================================================================
    def publish_lidar(self):
        """
        UST-20LX 相当の 2D スキャンを配信し、そこから角度セクタ最小で
        仮想 ToF (左右/斜め/前方) を切り出して既存トピックへ流す。

        セクタ最小 = その方向±窓内の最近点。単一ペンシルビームの真横 ToF と違い、
        ヘアピン頂点でも内壁・外壁の両方を確実に捉えられるので、wall_follow の
        両側センタリングが効き、外壁への吸い込みが起きにくい。
        """
        stamp = self.get_clock().now().to_msg()
        # 射出点は車体中心ではなく LiDAR の取付位置 (前方マウント)。
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        sx = self.x + self.lidar_mx * c - self.lidar_my * s
        sy = self.y + self.lidar_mx * s + self.lidar_my * c
        ranges = raycast_multi(self.segs, sx, sy,
                               self.yaw + self.lidar_angles, self.lidar_max)
        if not self._sees_wall['lidar']:          # スキャン面が板の帯の外
            ranges = np.full_like(ranges, self.lidar_max)

        # --- /scan (可視化・将来のフルスキャン利用向け) ---
        noisy = ranges + self.rng.normal(0.0, self.noise, size=ranges.shape)
        out = np.where(ranges >= self.lidar_max, float('inf'),
                       np.maximum(self.lidar_min, noisy))
        msg = LaserScan()
        msg.header.stamp = stamp
        # 実機の urg_node は frame_id='laser' で出し、lidar.launch.py が
        # base_link → laser の静的 TF を配る。sim は TF を増やさずに済むよう
        # base_link のまま出す (レイの起点には lidar_mount_* を既に適用済み
        # なので、sensor_fusion 側の扱いは実機と同じになる)。
        msg.header.frame_id = 'base_link'
        msg.angle_min = float(-self.lidar_fov / 2.0)
        msg.angle_max = float(self.lidar_fov / 2.0)
        msg.angle_increment = float(self.lidar_res)
        msg.time_increment = 0.0
        msg.scan_time = float(1.0 / self.lidar_rate)
        msg.range_min = float(self.lidar_min)
        msg.range_max = float(self.lidar_max)
        msg.ranges = [float(v) for v in out]
        self.pub_scan.publish(msg)

        # --- 角度セクタ最小で仮想 ToF を切り出す ---
        half = self.lidar_sector / 2.0

        def sector_min(center):
            m = (self.lidar_angles >= center - half) & \
                (self.lidar_angles <= center + half)
            return float(np.min(ranges[m])) if np.any(m) else float('inf')

        d_l = sector_min(math.pi / 2.0)
        d_r = sector_min(-math.pi / 2.0)
        d_fl = sector_min(math.pi / 4.0)
        d_fr = sector_min(-math.pi / 4.0)
        d_f = sector_min(0.0)

        # 前方の他車 (LiDAR は壁しか見ないので明示的に反映)
        opp = getattr(self, '_opp_rel', None)
        if opp is not None and opp['ahead'] and abs(opp['ly']) < 0.18:
            d_f = min(d_f, math.hypot(opp['lx'], opp['ly']))

        self._pub_range(self.pub_tof_l, stamp, 'tof_left', d_l, self.lidar_max)
        self._pub_range(self.pub_tof_r, stamp, 'tof_right', d_r, self.lidar_max)
        self._pub_range(self.pub_tof_fl, stamp, 'tof_fl', d_fl, self.lidar_max)
        self._pub_range(self.pub_tof_fr, stamp, 'tof_fr', d_fr, self.lidar_max)
        self._pub_range(self.pub_sonar, stamp, 'sonar_front', d_f,
                        self.lidar_max, Range.ULTRASOUND)

    # =================================================================
    def _publish_mag(self, stamp):
        """
        地磁気 (AK8963 相当) を車体座標で配信する。

        水平磁場を車体座標へ回し、ハードアイアン(固定オフセット)と
        ノイズを載せる。会場の鉄骨/SUS支柱を模した擾乱ゾーンでは
        方位がでたらめになるので、pose_fusion 側で弾けるかを試せる。
        """
        # 磁北方向 (odom 座標) に対する車体ヨー
        head = self.yaw - self.mag_decl
        bx = self.mag_field * math.cos(-head)
        by = self.mag_field * math.sin(-head)

        zx, zy, zr, zamp = self.mag_zone
        if zr > 0.0 and math.hypot(self.x - zx, self.y - zy) < zr:
            # 擾乱: 位置で決まる固定方向の大きな付加磁場
            ang = 2.7 * self.x + 1.9 * self.y
            bx += zamp * math.cos(ang)
            by += zamp * math.sin(ang)

        bx += self.mag_hard_iron[0] + float(self.rng.normal(0, self.mag_noise))
        by += self.mag_hard_iron[1] + float(self.rng.normal(0, self.mag_noise))

        m = MagneticField()
        m.header.stamp = stamp
        m.header.frame_id = 'imu_link'
        # ROS の MagneticField は [T]。uT から変換する。
        m.magnetic_field.x = bx * 1e-6
        m.magnetic_field.y = by * 1e-6
        m.magnetic_field.z = -20.0e-6      # 伏角ぶんの鉛直成分 (方位には使わない)
        self.pub_mag.publish(m)

    # =================================================================
    def publish_perception(self):
        """
        コースの真値から LaneInfo を生成する。
        lane_detector の代替なので、画像処理そのものは検証されない点に注意。
        """
        stamp = self.get_clock().now().to_msg()
        gim = getattr(self, '_gimmick', None)

        # 本物の lane_detector に /lane_info を明け渡している場合は出さない
        if not self.publish_lane:
            self._publish_arrow(stamp, gim)
            return

        lane = LaneInfo()
        lane.header.stamp = stamp
        lane.header.frame_id = 'base_link'

        # トンネル内はカメラが使えない設定にして、BT の ToF 切替を検証する
        if gim == 'TUNNEL':
            lane.valid = False
            lane.source = LaneInfo.SOURCE_RANGE
            lane.confidence = 0.0
            self.pub_lane.publish(lane)
            self._publish_arrow(stamp, gim)
            return

        s, d, psi = self.course.nearest(self.x, self.y)
        heading_err = math.atan2(math.sin(psi - self.yaw),
                                 math.cos(psi - self.yaw))

        lane.valid = True
        lane.source = LaneInfo.SOURCE_COLOR
        lane.confidence = 0.55 if gim == 'LIGHT_DISTURB' else 0.85
        lane.lookahead_m = 0.55
        lane.cross_track_error_m = float(-d)
        lane.heading_error_rad = float(heading_err)
        # 前方の見通し距離を実際の前方クリアランスで与える (カメラ相当)。
        # 直線では大きく (=速く走れる)、カーブ/ヘアピン手前では自然に短くなり
        # lane_follow の v_sight が手前で減速する (①早めの減速を暗黙に実現)。
        lane.free_distance_m = float(raycast(self.segs, self.x, self.y,
                                             self.yaw, 3.0))
        # コリドー幅と左右余裕は **実際の壁からレイキャストで** 出す。
        # ギミック名で 0.35m を返すやり方だと、予選ラウンドのように
        # ⑤狭い道の中央仕切りを外した条件で「狭くないのに狭い」と誤って
        # 伝わり、下流 (zone_estimator の NARROW 判定) が実際の測距と
        # 食い違ってしまう。
        m_l = float(raycast(self.segs, self.x, self.y,
                            self.yaw + math.pi / 2.0, 2.0))
        m_r = float(raycast(self.segs, self.x, self.y,
                            self.yaw - math.pi / 2.0, 2.0))
        lane.corridor_width_m = float(min(m_l + m_r, 2.0))
        lane.left_margin_m = m_l
        lane.right_margin_m = m_r

        # 前方の中心線サンプル
        pts = []
        for k in range(1, 8):
            ds = 0.12 * k
            idx = np.searchsorted(self.course.cum_len,
                                  (s + ds) % self.course.total_length)
            idx = min(idx, len(self.course.center) - 1)
            px, py = self.course.center[idx]
            dx, dy = px - self.x, py - self.y
            lx = dx * math.cos(-self.yaw) - dy * math.sin(-self.yaw)
            ly = dx * math.sin(-self.yaw) + dy * math.cos(-self.yaw)
            pts.append(Point(x=float(lx), y=float(ly), z=0.0))
        lane.centerline = pts

        if len(pts) >= 5:
            xs = np.array([p.x for p in pts])
            ys = np.array([p.y for p in pts])
            if xs.max() - xs.min() > 0.05:
                a, b, _ = np.polyfit(xs, ys, 2)
                lane.curvature_1pm = float(2 * a / (1 + b * b) ** 1.5)

        self.pub_lane.publish(lane)
        self._publish_arrow(stamp, gim)

    def _publish_arrow(self, stamp, gim):
        # use_camera のときは本物の arrow_detector が /arrow_sign を出すので
        # 偽の矢印は publish しない (トピック衝突を避ける)。
        if self.use_camera:
            return
        a = ArrowSign()
        a.header.stamp = stamp
        if gim == 'ARROW_GATE':
            a.detected = True
            a.direction = self.arrow_dir
            a.stable_direction = self.arrow_dir
            a.stable_count = 10
            a.confidence = 0.9
            a.distance_m = 2.0
        else:
            a.detected = False
            a.direction = ArrowSign.DIR_UNKNOWN
            a.stable_direction = ArrowSign.DIR_UNKNOWN
        self.pub_arrow.publish(a)

    # =================================================================
    def _pub_range(self, pub, stamp, frame, value, max_r,
                   rtype=Range.INFRARED):
        m = Range()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.radiation_type = rtype
        m.field_of_view = 0.3
        m.min_range = 0.02
        m.max_range = max_r
        noisy = value + float(self.rng.normal(0.0, self.noise))
        m.range = float('inf') if value >= max_r else float(max(0.02, noisy))
        pub.publish(m)

    def _pub_f(self, pub, value):
        m = Float32()
        m.data = float(value)
        pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSim()
    try:
        if node.lockstep:
            # step サービスの中で画像の到着を待つので、購読は別スレッドで回す
            ex = MultiThreadedExecutor(num_threads=3)
            ex.add_node(node)
            ex.spin()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                       # launch の SIGINT で ExternalShutdownException が出る (Traceback を出さない)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
