#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RViz2 用の可視化ノード。シミュレータの動作を目で確認するためのもの。

vehicle_sim には手を入れず、publish されているトピットと自前のコース定義
(default_course) だけから MarkerArray を組み立てる完全な独立ノード。
これにより「実車では何が見えているか」を RViz2 上に再現する。

表示するもの (すべて /sim/markers の 1 本の MarkerArray にまとめる):
  - コースの左右の壁               (odom フレーム, 茶)
  - ギミック区間のラベル            (odom フレーム, 区間ごとの色)
  - 自車の車体                     (base_link フレーム)
      橙のバンパー=前 / 暗赤=後 / 黄矢印=進行方向 で前後が分かる
      前輪タイヤは /actuator_cmd の舵角ぶん傾き、ステアリングが見える
  - 他車                          (base_link フレーム, 赤 / 検出時のみ)
  - 左右 ToF ・ 前方超音波の光線    (base_link フレーム, 緑 / 橙)
  - コリドー中心線 (/lane_info)     (base_link フレーム, 紫)
  - 走行軌跡                       (odom フレーム)
      各点の色が通過時の速度 (青:遅い→緑→赤:速い)。/odom の速度で着色
  - 行動状態ラベル                  (base_link フレーム, 状態・周回・現在速度)

RViz2 の Fixed Frame は odom。base_link フレームの Marker は
vehicle_sim が流す TF (odom→base_link) に追従して動く。
"""

import math
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from std_msgs.msg import ColorRGBA, String, Int32, Bool
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from minicar_msgs.msg import OpponentInfo, LaneInfo, DriveCommand

from collections import deque

from course import (default_course, GIMMICK_AREAS, LIGHT_POS, LIGHT_RIG,
                    ARROW_SIGN)


# ギミックごとの色 (RViz のラベル)
ZONE_COLORS = {
    'TUNNEL':        (0.30, 0.45, 0.85),
    'LIGHT_DISTURB': (0.90, 0.70, 0.20),
    'MU_LOW':        (0.55, 0.40, 0.75),
    'ROUGH':         (0.30, 0.75, 0.93),
    'ARROW_GATE':    (0.30, 0.70, 0.40),
    'NARROW':        (0.85, 0.35, 0.30),
    'SLOPE':         (0.40, 0.60, 0.85),
}


def rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def yaw_quat(yaw):
    return Quaternion(x=0.0, y=0.0,
                      z=float(math.sin(yaw * 0.5)), w=float(math.cos(yaw * 0.5)))


def speed_color(s, vmax):
    """通過速度を色に変換する。青(遅い)→緑(中)→赤(速い)。"""
    t = max(0.0, min(1.0, s / max(0.1, vmax)))
    if t < 0.5:
        a = t / 0.5     # 青 → 緑
        return rgba(0.20 + 0.10 * a, 0.40 + 0.40 * a, 0.90 - 0.60 * a)
    a = (t - 0.5) / 0.5  # 緑 → 赤
    return rgba(0.30 + 0.60 * a, 0.80 - 0.50 * a, 0.30 - 0.10 * a)



# race_manager がレース開始時に 1 回だけ送る制御トピック用。
# 購読側も TRANSIENT_LOCAL にしないと、起動が間に合わなかったときに
# 取りこぼす (実測: lap_counter が 0.296 秒遅れて enable を失った)。
LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST)

class SimViz(Node):

    def __init__(self):
        super().__init__('sim_viz')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('vehicle_length_m', 0.47)
        self.declare_parameter('vehicle_width_m', 0.22)
        self.declare_parameter('use_shortcut', True)
        # ⑤狭い道の中央仕切り。試走会/予選/決勝すべてで設置 (△3 p.30-31)。false は比較検証用
        self.declare_parameter('narrow_divider', True)
        self.declare_parameter('wheelbase_m', 0.210)      # M-05
        self.declare_parameter('max_steer_rad', 0.42)     # 舵角いっぱい
        self.declare_parameter('speed_color_max_mps', 2.5)  # 軌跡が赤になる速度
        self.declare_parameter('trail_max_points', 700)
        self.declare_parameter('trail_min_dist_m', 0.03)  # 何m進むごとに点を打つか

        p = self.get_parameter
        self.veh_l = float(p('vehicle_length_m').value)
        self.veh_w = float(p('vehicle_width_m').value)
        self.wheelbase = float(p('wheelbase_m').value)
        self.max_steer = float(p('max_steer_rad').value)
        self.speed_ref = float(p('speed_color_max_mps').value)
        self.trail_min_dist = float(p('trail_min_dist_m').value)

        self.course = default_course(
            use_shortcut=bool(self.get_parameter('use_shortcut').value),
            narrow_divider=bool(self.get_parameter('narrow_divider').value))

        # --- 入力 ---
        self.opp = None
        self.lane = None
        self.tof_l = None
        self.tof_r = None
        self.tof_fl = None
        self.tof_fr = None
        self.sonar = None
        # 後方 (超音波 x1)
        self.sonar_rear = None
        self.state = ''
        self.lap = 0
        self.steer_norm = 0.0
        self.speed_now = 0.0

        # 走行軌跡 (odom フレーム, 通過速度つき)
        self.trail = deque(maxlen=int(p('trail_max_points').value))
        self._last_trail = None

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(OpponentInfo, '/opponent_info', self.cb_opp, 10)
        self.create_subscription(LaneInfo, '/lane_info', self.cb_lane, 10)
        self.create_subscription(Range, '/sensors/tof_left', self.cb_tl, sensor_qos)
        self.create_subscription(Range, '/sensors/tof_right', self.cb_tr, sensor_qos)
        self.create_subscription(Range, '/sensors/tof_fl', self.cb_tfl, sensor_qos)
        self.create_subscription(Range, '/sensors/tof_fr', self.cb_tfr, sensor_qos)
        self.create_subscription(Range, '/sensors/sonar_front', self.cb_sn, sensor_qos)
        self.create_subscription(Range, '/sensors/sonar_rear', self.cb_snr, sensor_qos)
        self.create_subscription(String, '/behavior/state', self.cb_state, 10)
        self.create_subscription(Int32, '/lap_count', self.cb_lap, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(DriveCommand, '/actuator_cmd', self.cb_cmd, 10)
        self.create_subscription(Bool, '/odom/reset', self.cb_reset, LATCHED)

        self.pub = self.create_publisher(MarkerArray, '/sim/markers', 1)

        # 占有格子をマーカーで描く。RViz2 の Map 表示は AMD/Mesa の GPU で
        # シェーダ (indexed_8bit_image) のリンクに失敗して何も描かれない
        # (Jetson の NVIDIA では描ける)。GPU に依存しない CUBE_LIST にする。
        self._grid_period = 0.1
        self._grid_last = 0.0
        self.create_subscription(OccupancyGrid, '/fusion/local_map', self.cb_grid, 1)

        self.create_timer(1.0 / float(p('rate_hz').value), self.publish)
        # コース壁は動かないので一度だけ作って使い回す
        self._static = self._build_static_markers()
        self.get_logger().info("SimViz started. RViz2 の Fixed Frame を 'odom' にしてください。")

    # -----------------------------------------------------------------
    def cb_opp(self, msg):
        self.opp = msg

    def cb_lane(self, msg):
        self.lane = msg

    def cb_tl(self, msg):
        self.tof_l = msg.range

    def cb_tr(self, msg):
        self.tof_r = msg.range

    def cb_tfl(self, msg):
        self.tof_fl = msg.range

    def cb_tfr(self, msg):
        self.tof_fr = msg.range

    def cb_sn(self, msg):
        self.sonar = msg.range

    def cb_snr(self, msg):
        self.sonar_rear = msg.range

    def cb_state(self, msg):
        self.state = msg.data

    def cb_lap(self, msg):
        self.lap = msg.data

    def cb_cmd(self, msg):
        self.steer_norm = msg.steer_norm

    def cb_odom(self, msg):
        self.speed_now = msg.twist.twist.linear.x
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 一定距離進むごとに、その地点と通過速度を軌跡へ記録する
        if self._last_trail is None or \
                math.hypot(x - self._last_trail[0], y - self._last_trail[1]) \
                >= self.trail_min_dist:
            self.trail.append((x, y, abs(self.speed_now)))
            self._last_trail = (x, y)

    def cb_reset(self, msg):
        if msg.data:
            self.trail.clear()
            self._last_trail = None

    # -----------------------------------------------------------------
    def cb_grid(self, msg: OccupancyGrid):
        now = time.monotonic()
        if now - self._grid_last < self._grid_period:
            return
        self._grid_last = now
        info = msg.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.asarray(msg.data, np.int16).reshape(h, w)
        out = MarkerArray()
        # (値の条件, id, 色): 占有 = 赤紫の濃い面 / 空き = 薄い水色
        for mask, mid, col in ((data >= 50, 0, rgba(0.95, 0.15, 0.75, 0.85)),
                               ((data >= 0) & (data < 50), 1, rgba(0.30, 0.85, 0.95, 0.18))):
            m = self._base('fusion_grid', mid, Marker.CUBE_LIST)
            m.header.stamp = msg.header.stamp      # 格子を作った時点の車体位置に置く
            m.header.frame_id = msg.header.frame_id or 'base_link'
            m.scale.x = m.scale.y = res * 0.96
            m.scale.z = 0.004
            m.color = col
            jj, ii = np.nonzero(mask)
            m.points = [Point(x=float(ox + (i + 0.5) * res), y=float(oy + (j + 0.5) * res), z=0.01)
                        for j, i in zip(jj, ii)]
            if not m.points:
                m.action = Marker.DELETE
            out.markers.append(m)
        self.pub.publish(out)

    def _base(self, ns, mid, mtype, frame='base_link'):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    # -----------------------------------------------------------------
    def _build_static_markers(self):
        arr = []

        # ギミックの路面 (レギュレーション p.24 の実測領域)。
        # 「いま車がどの路面の上にいるか」を目で確かめられるようにする。
        # 特に④は外側が人工芝(高μ)・内側が滑り板(低μ)で、境界を跨ぐと
        # 左右輪の摩擦が変わってスピンするため、走路との関係が重要。
        # コースの基本床: パンチカーペット (グレー)。ギミック路面はこの上に
        # 不透明で重ねるので、①暗幕・④人工芝/滑り板・⑦マットが床と見分けられる。
        # 範囲は外周の壁 (course.WALLS_M) を少し広げた矩形。
        carpet = self._base('floor', 0, Marker.CUBE, frame='odom')
        carpet.pose.position.x = 5.175
        carpet.pose.position.y = 3.175
        # 床とギミック路面は z<0 に十分下げて描く。z=0 付近に不透明の面があると、
        # 同じ高さに半透明で描く占有格子 (/fusion/local_map, base_link の z=0) が
        # 奥行き精度の都合で隠れて見えなくなる (真上からの遠い視点で顕著)。
        carpet.pose.position.z = -0.030
        carpet.scale.x = 10.45
        carpet.scale.y = 6.55
        carpet.scale.z = 0.002
        carpet.color = rgba(0.40, 0.41, 0.42, 1.0)
        arr.append(carpet)

        # (色, 不透明度, ラベル)。並びは GIMMICK_AREAS の順で上に重なる
        # (人工芝の上に滑り板が載る)。
        area_style = {
            'MU_HIGH': ((0.16, 0.55, 0.20), 1.0, '④人工芝(高μ)'),
            'MU_LOW':  ((0.93, 0.93, 0.96), 1.0, '④滑り板(低μ)'),
            # ⑦は 40x80cm の青い風呂マット4枚 (p.32 の写真の色)
            'ROUGH':   ((0.20, 0.58, 0.95), 1.0, '⑦でこぼこ道'),
            'SLOPE':   ((0.95, 0.65, 0.25), 0.45, '②坂道'),
            # ①暗幕 (漆黒シート)。床を黒くして暗幕ゾーンの範囲を示す
            'TUNNEL':  ((0.03, 0.03, 0.035), 1.0, '①暗幕トンネル'),
            'LIGHT':   ((0.95, 0.85, 0.30), 0.22, '③ライトかく乱'),
        }
        for k, (name, x0, y0, x1, y1) in enumerate(GIMMICK_AREAS):
            rgb, alpha, label = area_style.get(
                name, ((0.6, 0.6, 0.6), 0.3, name))
            m = self._base('gimmick_area', k, Marker.CUBE, frame='odom')
            m.pose.position.x = float((x0 + x1) / 2.0)
            m.pose.position.y = float((y0 + y1) / 2.0)
            m.pose.position.z = -0.026 + 0.002 * k    # 重なり順を安定させる (床より上・格子より下)
            m.scale.x = float(x1 - x0)
            m.scale.y = float(y1 - y0)
            m.scale.z = 0.001
            m.color = rgba(*rgb, alpha)
            arr.append(m)
            t = self._base('gimmick_area', 100 + k, Marker.TEXT_VIEW_FACING,
                           frame='odom')
            t.pose.position.x = float((x0 + x1) / 2.0)
            t.pose.position.y = float((y0 + y1) / 2.0)
            t.pose.position.z = 0.16
            t.scale.z = 0.13
            # 黒い暗幕のラベルは床に埋もれるので明るい色で書く
            t.color = rgba(0.85, 0.85, 0.9, 0.95) if name == 'TUNNEL' else rgba(*rgb, 0.95)
            t.text = label
            arr.append(t)

        # 壁 (レギュレーション p.24 の SPF材ボード 1 枚ずつ)。
        # 実物どおり赤板と白板を板ごとの色で描く (course.WALL_COLORS)。
        # LINE_LIST は点ごとに色を持てるので、1 本のマーカーで塗り分ける。
        m = self._base('course', 0, Marker.LINE_LIST, frame='odom')
        m.scale.x = 0.035
        m.color = rgba(1.0, 1.0, 1.0)
        wall_colors = getattr(self.course, 'wall_colors', None)
        red_rgba = rgba(0.85, 0.13, 0.13)
        white_rgba = rgba(0.95, 0.95, 0.95)
        for i, (x0, y0, x1, y1) in enumerate(self.course.wall_segments()):
            # 板は床から 30 mm 浮き、上端 約 120 mm (規約 p.34)。線は帯の中央の高さに置く
            m.points.append(Point(x=float(x0), y=float(y0), z=0.075))
            m.points.append(Point(x=float(x1), y=float(y1), z=0.075))
            is_red = (wall_colors is not None and i < len(wall_colors)
                      and wall_colors[i] == 'red')
            c = red_rgba if is_red else white_rgba
            m.colors.append(c)
            m.colors.append(c)
        arr.append(m)

        # スタート/ゴールライン (コース始点)
        sx, sy = self.course.center[0]
        nx, ny = self._normal_at(0)
        line = self._base('course', 2, Marker.LINE_LIST, frame='odom')
        line.scale.x = 0.03
        line.color = rgba(0.95, 0.95, 0.95)
        h = self.course.half
        line.points = [Point(x=float(sx + h * nx), y=float(sy + h * ny), z=0.0),
                       Point(x=float(sx - h * nx), y=float(sy - h * ny), z=0.0)]
        arr.append(line)

        # ギミック区間のラベル
        for i, (name, lo, hi) in enumerate(self.course.gimmicks):
            mid = (lo + hi) / 2.0
            s = mid * self.course.total_length
            idx = int(np.searchsorted(self.course.cum_len, s))
            idx = min(idx, len(self.course.center) - 1)
            px, py = self.course.center[idx]
            t = self._base('zones', i, Marker.TEXT_VIEW_FACING, frame='odom')
            t.pose.position.x = float(px)
            t.pose.position.y = float(py)
            t.pose.position.z = 0.25
            t.scale.z = 0.16
            t.color = rgba(*ZONE_COLORS.get(name, (0.7, 0.7, 0.7)))
            t.text = name
            arr.append(t)

        # 駐車枠 P1(緑)/P2(赤)/P3(青) — 下辺中央にぶら下がる (⑧ 決勝のみ)
        slot_rgb = {'green': (0.20, 0.65, 0.30),
                    'red': (0.85, 0.25, 0.22),
                    'blue': (0.25, 0.40, 0.85)}
        for si, slot in enumerate(getattr(self.course, 'slots', [])):
            name, color, x0, y0, x1, y1 = slot
            box = self._base('slots', si, Marker.LINE_STRIP, frame='odom')
            box.scale.x = 0.025
            box.color = rgba(*slot_rgb.get(color, (0.8, 0.8, 0.8)))
            box.points = [Point(x=float(x0), y=float(y0), z=0.0),
                          Point(x=float(x1), y=float(y0), z=0.0),
                          Point(x=float(x1), y=float(y1), z=0.0),
                          Point(x=float(x0), y=float(y1), z=0.0),
                          Point(x=float(x0), y=float(y0), z=0.0)]
            arr.append(box)
            lbl = self._base('slot_label', si, Marker.TEXT_VIEW_FACING, frame='odom')
            lbl.pose.position.x = (x0 + x1) / 2.0
            lbl.pose.position.y = (y0 + y1) / 2.0
            lbl.pose.position.z = 0.12
            lbl.scale.z = 0.18
            lbl.color = rgba(*slot_rgb.get(color, (0.8, 0.8, 0.8)))
            lbl.text = name
            arr.append(lbl)

        # ③ライトかく乱のライト本体 (曲がり角中心、高さ40cm)。
        # 実物は SUS 材 2 本を十字に渡し、その交点にライトを載せる。脚は
        # 四方の壁の上なので、走路に立つ柱は無い (p.28)。
        lx, ly, lz = LIGHT_POS
        bar_z = LIGHT_RIG['bar_z']
        for k, (dx, dy) in enumerate(((LIGHT_RIG['bar_ew'] / 2.0, 0.0),
                                      (0.0, LIGHT_RIG['bar_ns'] / 2.0))):
            bar = self._base('gimmick_area', 202 + k, Marker.LINE_LIST,
                             frame='odom')
            bar.scale.x = 0.02
            bar.color = rgba(0.72, 0.74, 0.78, 0.85)
            bar.points = [Point(x=float(lx - dx), y=float(ly - dy), z=bar_z),
                          Point(x=float(lx + dx), y=float(ly + dy), z=bar_z)]
            for sgn in (-1.0, 1.0):     # 端の脚 (壁の上に立つ)
                ex, ey = lx + sgn * dx, ly + sgn * dy
                bar.points += [Point(x=float(ex), y=float(ey), z=bar_z),
                               Point(x=float(ex), y=float(ey), z=0.0)]
            arr.append(bar)
        lamp = self._base('gimmick_area', 200, Marker.SPHERE, frame='odom')
        lamp.pose.position.x = float(lx)
        lamp.pose.position.y = float(ly)
        lamp.pose.position.z = float(lz) + 0.03
        lamp.scale.x = lamp.scale.y = lamp.scale.z = 0.09
        lamp.color = rgba(0.95, 0.85, 0.30, 0.95)
        arr.append(lamp)
        lamp_t = self._base('gimmick_area', 201, Marker.TEXT_VIEW_FACING,
                            frame='odom')
        lamp_t.pose.position.x = float(lx)
        lamp_t.pose.position.y = float(ly)
        lamp_t.pose.position.z = float(lz) + 0.18
        lamp_t.scale.z = 0.12
        lamp_t.color = rgba(0.95, 0.85, 0.30, 0.95)
        lamp_t.text = 'ライト(h40cm)'
        arr.append(lamp_t)

        # ⑥矢印信号の門型フレーム。支柱は狭い道の左右の紅白の板の上。
        sx = ARROW_SIGN['x']
        py0, py1 = ARROW_SIGN['post_y']
        ph = ARROW_SIGN['post_h']
        bz0 = ARROW_SIGN['board_z0']
        bz1 = bz0 + ARROW_SIGN['board_h']
        bw = ARROW_SIGN['board_w'] / 2.0
        bc = (py0 + py1) / 2.0
        frame = self._base('gimmick_area', 204, Marker.LINE_LIST, frame='odom')
        frame.scale.x = 0.02
        frame.color = rgba(0.72, 0.74, 0.78, 0.9)
        frame.points = []
        for py in (py0, py1):           # 支柱
            frame.points += [Point(x=float(sx), y=float(py), z=0.0),
                             Point(x=float(sx), y=float(py), z=ph)]
        frame.points += [Point(x=float(sx), y=float(py0), z=ph),   # 上桟
                         Point(x=float(sx), y=float(py1), z=ph)]
        arr.append(frame)
        board = self._base('gimmick_area', 205, Marker.CUBE, frame='odom')
        board.pose.position.x = float(sx)
        board.pose.position.y = float(bc)
        board.pose.position.z = float((bz0 + bz1) / 2.0)
        board.scale.x = 0.03
        board.scale.y = float(2.0 * bw)
        board.scale.z = float(bz1 - bz0)
        board.color = rgba(0.12, 0.12, 0.14, 0.95)
        arr.append(board)
        board_t = self._base('gimmick_area', 206, Marker.TEXT_VIEW_FACING,
                             frame='odom')
        board_t.pose.position.x = float(sx)
        board_t.pose.position.y = float(bc)
        board_t.pose.position.z = float(ph + 0.12)
        board_t.scale.z = 0.12
        board_t.color = rgba(0.35, 0.75, 0.45, 0.95)
        board_t.text = '⑥矢印信号(h65-84cm)'
        arr.append(board_t)

        # 走行軌跡の色 = 速度 の凡例。コースの左外側に縦のカラーバーで置く。
        # 文字だけの凡例だと「どの色が何 m/s か」が読み取れないため。
        bar_x, bar_y0, bar_y1 = -0.75, 1.20, 4.20
        n_seg = 24
        bar = self._base('legend', 10, Marker.LINE_LIST, frame='odom')
        bar.scale.x = 0.16
        bar.color = rgba(1.0, 1.0, 1.0)
        for k in range(n_seg):
            v0 = self.speed_ref * k / n_seg
            v1 = self.speed_ref * (k + 1) / n_seg
            y0 = bar_y0 + (bar_y1 - bar_y0) * k / n_seg
            y1 = bar_y0 + (bar_y1 - bar_y0) * (k + 1) / n_seg
            c = speed_color(0.5 * (v0 + v1), self.speed_ref)
            bar.points.append(Point(x=bar_x, y=float(y0), z=0.01))
            bar.points.append(Point(x=bar_x, y=float(y1), z=0.01))
            bar.colors.append(c)
            bar.colors.append(c)
        arr.append(bar)
        title = self._base('legend', 11, Marker.TEXT_VIEW_FACING, frame='odom')
        title.pose.position.x = bar_x
        title.pose.position.y = bar_y1 + 0.30
        title.pose.position.z = 0.02
        title.scale.z = 0.16
        title.color = rgba(0.92, 0.92, 0.92)
        title.text = '走行軌跡の色 = 速度'
        arr.append(title)
        for k in range(4):
            v = self.speed_ref * k / 3.0
            t = self._base('legend', 20 + k, Marker.TEXT_VIEW_FACING,
                           frame='odom')
            t.pose.position.x = bar_x - 0.34
            t.pose.position.y = bar_y0 + (bar_y1 - bar_y0) * k / 3.0
            t.pose.position.z = 0.02
            t.scale.z = 0.14
            t.color = rgba(0.88, 0.88, 0.88)
            t.text = f'{v:.1f}'
            arr.append(t)
        unit = self._base('legend', 30, Marker.TEXT_VIEW_FACING, frame='odom')
        unit.pose.position.x = bar_x - 0.34
        unit.pose.position.y = bar_y0 - 0.28
        unit.pose.position.z = 0.02
        unit.scale.z = 0.13
        unit.color = rgba(0.75, 0.75, 0.75)
        unit.text = 'm/s'
        arr.append(unit)

        # 軌跡の色の意味を示す凡例 (コース近くに固定表示)
        lx, ly = float(np.min(self.course.center[:, 0])) - 0.3, \
            float(np.max(self.course.center[:, 1])) + 0.4
        legend = self._base('legend', 0, Marker.TEXT_VIEW_FACING, frame='odom')
        legend.pose.position.x = lx
        legend.pose.position.y = ly
        legend.pose.position.z = 0.2
        legend.scale.z = 0.16
        legend.color = rgba(0.9, 0.9, 0.9)
        legend.text = "軌跡の色 = 通過速度  (青:遅い → 赤:速い)"
        arr.append(legend)

        return arr

    def _normal_at(self, i):
        n = len(self.course.center)
        a = self.course.center[min(i + 1, n - 1)]
        b = self.course.center[max(i - 1, 0)]
        d = a - b
        norm = math.hypot(d[0], d[1]) or 1.0
        return -d[1] / norm, d[0] / norm

    # -----------------------------------------------------------------
    def publish(self):
        arr = MarkerArray()
        # 静的マーカーはスタンプだけ更新して再送 (RViz が後から起動しても見える)
        now = self.get_clock().now().to_msg()
        for m in self._static:
            m.header.stamp = now
        arr.markers.extend(self._static)

        arr.markers.extend(self._ego_markers())
        arr.markers.extend(self._sensor_rays())

        trail = self._trail_marker()
        if trail is not None:
            arr.markers.append(trail)

        cl = self._centerline()
        if cl is not None:
            arr.markers.append(cl)

        opp = self._opponent()
        if opp is not None:
            arr.markers.append(opp)
        else:
            arr.markers.append(self._delete('opponent', 0))

        arr.markers.append(self._state_label())

        self.pub.publish(arr)

    # -----------------------------------------------------------------
    def _ego_markers(self):
        """
        車体・前後の目印・4輪 (前輪は舵角ぶん傾く)・進行方向矢印。

        base_link は後軸付近を原点とする。前軸は +x に wheelbase。
        車体中心は両軸の中点に置く。
        """
        body_cx = self.wheelbase * 0.5
        front_x = self.wheelbase + (self.veh_l - self.wheelbase) * 0.5   # 前端付近
        rear_x = -(self.veh_l - self.wheelbase) * 0.5                    # 後端付近
        half_track = self.veh_w * 0.5 - 0.012
        steer = self.steer_norm * self.max_steer
        out = []

        # 車体 (青。幅は輪距より少し狭くしてタイヤを見せる)
        m = self._base('ego', 0, Marker.CUBE)
        m.pose.position.x = body_cx
        m.pose.position.z = 0.05
        m.scale.x = self.veh_l
        m.scale.y = self.veh_w * 0.82
        m.scale.z = 0.07
        m.color = rgba(0.18, 0.47, 0.77, 0.88)
        out.append(m)

        # 前バンパー (オレンジ = 前)
        nose = self._base('ego', 1, Marker.CUBE)
        nose.pose.position.x = front_x
        nose.pose.position.z = 0.075
        nose.scale.x = 0.035
        nose.scale.y = self.veh_w * 0.7
        nose.scale.z = 0.08
        nose.color = rgba(0.95, 0.55, 0.15)
        out.append(nose)

        # 後端 (暗い赤 = 後)
        tail = self._base('ego', 2, Marker.CUBE)
        tail.pose.position.x = rear_x
        tail.pose.position.z = 0.075
        tail.scale.x = 0.03
        tail.scale.y = self.veh_w * 0.7
        tail.scale.z = 0.06
        tail.color = rgba(0.55, 0.12, 0.10)
        out.append(tail)

        # 4輪。前輪 (id 10,11) は舵角ぶん傾ける。後輪 (12,13) は直進。
        wheels = [
            (10, self.wheelbase,  half_track, steer),
            (11, self.wheelbase, -half_track, steer),
            (12, 0.0,             half_track, 0.0),
            (13, 0.0,            -half_track, 0.0),
        ]
        for wid, wx, wy, wyaw in wheels:
            w = self._base('ego', wid, Marker.CUBE)
            w.pose.position.x = wx
            w.pose.position.y = wy
            w.pose.position.z = 0.03
            w.pose.orientation = yaw_quat(wyaw)
            w.scale.x = 0.075     # タイヤの長さ (転がり方向)
            w.scale.y = 0.028     # 幅
            w.scale.z = 0.06
            w.color = rgba(0.12, 0.12, 0.12)
            out.append(w)

        # 進行方向の矢印 (黄。前後がひと目で分かる)
        arrow = self._base('ego', 3, Marker.ARROW)
        arrow.pose.position.x = body_cx
        arrow.pose.position.z = 0.11
        arrow.scale.x = 0.33     # 長さ
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color = rgba(0.95, 0.9, 0.15)
        out.append(arrow)

        return out

    def _trail_marker(self):
        """走行軌跡。各点の色が通過速度 (青:遅→赤:速)。"""
        if len(self.trail) < 2:
            return None
        m = self._base('trail', 0, Marker.LINE_STRIP, frame='odom')
        m.scale.x = 0.035
        m.color = rgba(1.0, 1.0, 1.0)   # 既定色 (colors で上書きされる)
        for x, y, s in self.trail:
            m.points.append(Point(x=float(x), y=float(y), z=0.015))
            m.colors.append(speed_color(s, self.speed_ref))
        return m

    def _sensor_rays(self):
        out = []
        # (距離, 方位角[rad], 色, id)
        rays = [
            (self.sonar, 0.0, rgba(0.90, 0.55, 0.20), 0, 1.2),          # 前方
            (self.tof_l, math.pi / 2, rgba(0.30, 0.75, 0.40), 1, 1.2),  # 左
            (self.tof_r, -math.pi / 2, rgba(0.30, 0.75, 0.40), 2, 1.2), # 右
            (self.tof_fl, math.pi / 4, rgba(0.35, 0.65, 0.95), 3, 2.0), # 前左45度
            (self.tof_fr, -math.pi / 4, rgba(0.35, 0.65, 0.95), 4, 2.0),# 前右45度
            # 後方 (前方マウントの LiDAR の死角を埋める。脱出の後退で使う)
            (self.sonar_rear, math.pi, rgba(0.95, 0.45, 0.45), 5, 2.0),  # 真後ろ
        ]
        for dist, ang, col, i, dmax in rays:
            m = self._base('rays', i, Marker.LINE_LIST)
            m.scale.x = 0.012
            if dist is None or not math.isfinite(dist):
                # 無効値は薄く最大距離まで
                d = dmax
                col = rgba(col.r, col.g, col.b, 0.15)
            else:
                d = float(dist)
            m.color = col
            m.points = [Point(x=0.0, y=0.0, z=0.05),
                        Point(x=d * math.cos(ang), y=d * math.sin(ang), z=0.05)]
            # 先端に小さな点
            out.append(m)
        return out

    def _centerline(self):
        if self.lane is None or not self.lane.valid or not self.lane.centerline:
            return None
        m = self._base('centerline', 0, Marker.LINE_STRIP)
        m.scale.x = 0.02
        m.color = rgba(0.48, 0.35, 0.65)
        m.points = [Point(x=p.x, y=p.y, z=0.03) for p in self.lane.centerline]
        return m

    def _opponent(self):
        if self.opp is None or not self.opp.detected:
            return None
        m = self._base('opponent', 0, Marker.CUBE)
        d = self.opp.distance_m
        b = self.opp.bearing_rad
        m.pose.position.x = d * math.cos(b)
        m.pose.position.y = d * math.sin(b)
        m.pose.position.z = 0.05
        m.scale.x = 0.30
        m.scale.y = max(0.12, self.opp.opponent_width_m)
        m.scale.z = 0.10
        m.color = rgba(0.85, 0.25, 0.20, 0.9)
        return m

    def _state_label(self):
        m = self._base('state', 0, Marker.TEXT_VIEW_FACING)
        m.pose.position.z = 0.34
        m.scale.z = 0.14
        # 速度に応じてラベルの色も変える (軌跡と同じ配色)
        m.color = speed_color(abs(self.speed_now), self.speed_ref)
        m.text = (f"{self.state or 'IDLE'}  |  lap {self.lap}  |  "
                  f"{abs(self.speed_now):.2f} m/s")
        return m

    def _delete(self, ns, mid):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETE
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SimViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                       # launch の SIGINT で ExternalShutdownException が出る (Traceback を出さない)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
