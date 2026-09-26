#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
シミュレータ用のコース定義。

レギュレーション資料 p.24 の寸法図から、壁 (SPF材ボード) の座標を
PDF のベクター図形として抽出して再構成したもの。目視トレースではなく
図面の実座標なので、コース形状は p.24 とほぼ一致する。

抽出方法と較正:
  - p.24 の赤/白の棒 = 壁ボード、オレンジの丸 = 支柱(46本) として抽出
  - 全幅が 1030cm になるよう一様スケール (1.5399 cm/pt) を適用
  - 「スタート1/2/3」の縦長白枠は寸法が規格外(125cm)で、支柱に接続されて
    いないラベル枠なので壁から除外している (これを壁とみなすと下段レーンが
    塞がって周回できない)
  - 図面自体が部分的に非一様な模式図のため、板1枚が 186cm 相当ではなく
    192cm 相当で描かれている。全体寸法(1030cm)を優先して合わせてある。

コース構造 (下段から):
  L1 y≈1.21m  下段レーン  → 右へ (スタート/ゴール、下に駐車枠 P1/P2/P3)
  L2 y≈2.55m  第2レーン   → 左へ
  L3 y≈3.98m  第3レーン   → 右へ
  L4 y≈5.95m  上段レーン  → 左へ (⑤狭い道 / ⑥矢印信号 の仕切りあり)
  左チャネル x≈0.85m       → 下へ (⑦でこぼこ道)

走行方向は p.21 の矢印に一致させてある。

既定の走行ルートは ②坂道ショートカット経由:
  L1 → ①トンネルで上へ → L2 を少し左へ → ②坂道 (y≈3.21 仕切りの切れ目
  x 6.19-7.72) で L3 へ → ④低μ/高μ路で上へ → L4 を左へ (⑤狭い道 / ⑥矢印)
  → 左チャネルを下へ (⑦でこぼこ道) → L1
ショートカットにより L2/L3 の西側と左 U ターンを省略するため、
③ライトかく乱ゾーンは通らない (約 10m 短縮)。
外周経路 (③を通る) は default_course(use_shortcut=False) で選べる。

座標系: 原点は図面左下 (▲基準)、x 右、y 上、単位 m。
"""

import os
import math

import numpy as np


class Course:
    """壁 (線分の集合) と走行中心線で表したコース。"""

    def __init__(self, centerline, walls, width_m=0.60, gimmicks=None,
                 wall_colors=None,
                 slots=None):
        self.center = np.asarray(centerline, dtype=np.float64)
        self.walls = np.asarray(walls, dtype=np.float64)   # (N,4) x0,y0,x1,y1
        # 板ごとの色 ('red' / 'white')。合成カメラの描画に使う。
        self.wall_colors = (list(wall_colors) if wall_colors is not None
                            else ['white'] * len(self.walls))
        self.width = float(width_m)
        self.half = self.width / 2.0
        self.gimmicks = gimmicks or []
        self.slots = slots or []

        seg = np.diff(self.center, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum_len = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.total_length = float(self.cum_len[-1])

    # -----------------------------------------------------------------
    def wall_segments(self):
        """壁線分を [(x1,y1,x2,y2), ...] で返す (レイキャスト用)。"""
        return self.walls

    # -----------------------------------------------------------------
    def clearance(self, x, y):
        """点 (x,y) から最も近い壁までの距離 [m]。"""
        x0, y0, x1, y1 = (self.walls[:, 0], self.walls[:, 1],
                          self.walls[:, 2], self.walls[:, 3])
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        L2 = np.where(L2 < 1e-12, 1e-12, L2)
        t = np.clip(((x - x0) * dx + (y - y0) * dy) / L2, 0.0, 1.0)
        px, py = x0 + t * dx, y0 + t * dy
        return float(np.min(np.hypot(px - x, py - y)))

    # -----------------------------------------------------------------
    def nearest(self, x, y):
        """(弧長 s, 横偏差 d, 接線方位 psi) を返す。d は左が正。"""
        p = np.array([x, y])
        a = self.center[:-1]
        b = self.center[1:]
        ab = b - a
        ab_len2 = np.sum(ab * ab, axis=1)
        ab_len2[ab_len2 < 1e-12] = 1e-12
        t = np.clip(np.sum((p - a) * ab, axis=1) / ab_len2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        dist = np.hypot(proj[:, 0] - x, proj[:, 1] - y)
        i = int(np.argmin(dist))

        s = self.cum_len[i] + t[i] * self.seg_len[i]
        tangent = ab[i] / max(1e-9, math.hypot(ab[i, 0], ab[i, 1]))
        psi = math.atan2(tangent[1], tangent[0])
        nx, ny = -tangent[1], tangent[0]
        d = (x - proj[i, 0]) * nx + (y - proj[i, 1]) * ny
        return s, d, psi

    def progress(self, x, y):
        s, _, _ = self.nearest(x, y)
        return s / self.total_length

    def gimmick_at(self, s_ratio):
        """周回進捗 [0,1) に対応するギミック名。無ければ None。"""
        for name, lo, hi in self.gimmicks:
            if lo <= s_ratio < hi:
                return name
        return None


def raycast(segs, ox, oy, angle, max_range):
    """
    線分群に対する 2D レイキャスト。最も近い交点までの距離を返す。
    ToF / 超音波センサの模擬に使う。ROS 非依存なので単体テストできる。
    """
    dx, dy = math.cos(angle), math.sin(angle)
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    ex, ey = x2 - x1, y2 - y1

    denom = dx * ey - dy * ex
    valid = np.abs(denom) > 1e-12
    if not np.any(valid):
        return max_range

    t = np.full(len(segs), np.inf)
    u = np.full(len(segs), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        t[valid] = (((x1 - ox) * ey - (y1 - oy) * ex) / denom)[valid]
        u[valid] = (((x1 - ox) * dy - (y1 - oy) * dx) / denom)[valid]

    hit = valid & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    if not np.any(hit):
        return max_range
    return float(min(max_range, np.min(t[hit])))


def raycast_multi(segs, ox, oy, angles, max_range):
    """
    複数レイの一括レイキャスト (LiDAR スキャンの模擬)。
    angles(rad) 各方向について最近交点までの距離を返す (len(angles) の配列)。
    交点が無い方向は max_range を返す。ROS 非依存。
    """
    angles = np.asarray(angles, dtype=np.float64)
    dx = np.cos(angles)[:, None]                 # (B,1)
    dy = np.sin(angles)[:, None]
    x1 = segs[:, 0][None, :]                      # (1,S)
    y1 = segs[:, 1][None, :]
    ex = (segs[:, 2] - segs[:, 0])[None, :]
    ey = (segs[:, 3] - segs[:, 1])[None, :]

    denom = dx * ey - dy * ex                     # (B,S)
    num_t = (x1 - ox) * ey - (y1 - oy) * ex       # (1,S) → broadcast
    num_u = (x1 - ox) * dy - (y1 - oy) * dx       # (B,S)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = num_t / denom
        u = num_u / denom

    hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    t = np.where(hit, t, np.inf)
    r = np.min(t, axis=1)                         # (B,)
    return np.minimum(r, max_range)


# =====================================================================
# p.24 から抽出した壁ボード (単位 m)。x0,y0 - x1,y1 の線分。
# =====================================================================
WALLS_M = [
    # --- 外周: 左辺 ---
    (0.026, 0.516, 0.026, 2.436), (0.000, 2.445, 0.000, 4.365),
    (0.002, 4.348, 0.002, 5.298), (0.262, 6.294, 0.026, 5.374),
    # --- 外周: 下辺 (中央は駐車枠のため凹む) ---
    (2.034, 0.538, 0.114, 0.538), (0.856, 0.634, 0.184, 1.306),
    (7.892, 0.530, 5.972, 0.530), (9.910, 0.530, 7.990, 0.530),
    # --- 駐車枠 P1/P2/P3 (⑧ 決勝のみ) ---
    (4.044, -0.032, 2.124, -0.032), (6.023, -0.022, 4.103, -0.022),
    (2.113, 0.024, 2.113, 0.492), (3.428, 0.031, 3.428, 0.499),
    (4.707, 0.024, 4.707, 0.492), (5.984, 0.031, 5.984, 0.499),
    # --- 外周: 上辺 ---
    (0.210, 6.298, 1.160, 6.298), (3.005, 6.319, 1.085, 6.319),
    (4.950, 6.340, 3.030, 6.340), (6.966, 6.357, 5.046, 6.357),
    (8.894, 6.378, 6.974, 6.378),
    # --- 外周: 右辺 (八角コーナー) ---
    (9.798, 5.961, 8.959, 6.408), (10.293, 5.196, 9.787, 6.000),
    (10.279, 3.312, 10.279, 5.232), (10.288, 1.366, 10.288, 3.286),
    (10.351, 1.433, 9.814, 0.442),
    # --- 内壁: L1 / L2 の仕切り (y≈1.89) ---
    (2.612, 1.874, 1.790, 2.349), (4.656, 1.893, 2.736, 1.893),
    (6.731, 1.893, 4.811, 1.893), (8.739, 1.893, 6.819, 1.893),
    # --- 内壁: 左の縦仕切り (左チャネルと U ターン部を分ける) ---
    (1.731, 2.336, 1.731, 4.256),
    # --- 内壁: L2 / L3 の仕切り (中央の切れ目が ②坂道ショートカット) ---
    (5.230, 3.228, 3.310, 3.228), (6.189, 3.210, 5.239, 3.210),
    (9.640, 3.205, 7.720, 3.205),
    # --- 内壁: 右側の八角内コーナー (①トンネルの内壁) ---
    (10.243, 2.453, 9.699, 3.232), (10.199, 4.083, 9.716, 3.264),
    # --- 内壁: L3 / L4 の仕切り (y≈4.74) ---
    (1.750, 4.261, 2.573, 4.736), (4.513, 4.743, 2.593, 4.743),
    (6.442, 4.743, 4.522, 4.743), (8.404, 4.743, 6.484, 4.743),
    # --- 内壁: ⑤狭い道 の仕切り (⑥矢印信号のゲート) ---
    # レギュレーション △3 p.30-31: 「試走会/予選/決勝ラウンドすべてで設置予定」。
    # 予選でも仕切りはある (予選で違うのは矢印信号だけ)。
    #   default_course(narrow_divider=False) で外せる (仕切りなしの比較検証用)。
    (5.034, 5.557, 3.114, 5.557),
]

# 板の色 (WALLS_M と同じ並び)。レギュレーション p.24 の図面から
# 各板の描画色を読み取って割り当てた。実物は「赤板」「白板」を 1 枚ずつ
# 並べたもので、1 枚の中で赤白に塗り分けられているわけではない
# (SPF1x4 材: 赤180cm x10 / 白180cm x12 / 赤90cm x6 / 白90cm x6 / 白45cm x4)。
# 合成カメラ (vehicle_sim) はこの色でウォールを描画する。
# 注: ④低μ/高μ路まわり (右上の八角コーナー) の斜め板 2 枚 (#19, #33) は、
#     白板の両側に濃い縁が描かれていて自動判定が赤に振れていた。図面を
#     拡大して目視で白板と確認済み。これで赤は仕様どおり 16 枚になる
#     (白は 23 枚で仕様 22 枚より 1 枚多い = 壁線分の分割が 1 箇所多い)。
WALL_COLORS = (
    'white', 'red', 'white', 'red', 'red', 'red', 'red', 'white',
    'white', 'white', 'white', 'white', 'white', 'white', 'white',
    'white', 'red', 'white', 'red', 'white', 'white', 'red', 'white',
    'red', 'red', 'white', 'red', 'white', 'white', 'red', 'white',
    'red', 'red', 'white', 'red', 'white', 'red', 'white', 'white'
)

# =====================================================================
# ギミックの領域 (レギュレーション p.24 の図から実測)。可視化と、
# 「どの路面の上を走っているか」の判定に使う。(name, x0, y0, x1, y1)
# =====================================================================
# ③ライトかく乱のライト本体の位置 (曲がり角中心、高さ 40cm)。
# レギュレーション p.28「設置位置は曲がり角中心、高さ40cm地点」より。
#
# 実物は 190cm と (190+100)cm の SUS 材を十字に渡し、その交点にライトを
# 載せる (脚は四隅の壁に立つ)。写真の寸法は交点から壁まで
# 95cm / 140cm x2 で、これは内側ヘアピンの U ターン部
#   x 1.731 (仕切りの西端) 〜 3.310 (仕切り 29 の西端)
#   y 1.893 〜 4.743 (上下の仕切り)
# の中心と一致する。つまり柱は走路の外 (壁の上) で、ライトだけが
# 走路の上空 40cm に浮く。
#   x = 1.731 + 0.95 (西の仕切りから 95cm)
#   y = (1.893 + 4.743) / 2 → 上下の仕切りまで 140cm ずつ
LIGHT_POS = (2.68, 3.32, 0.40)
LIGHT_RIG = dict(bar_ew=1.90, bar_ns=2.85, bar_z=0.38, leg_r=0.012)

# ⑥矢印信号 (レギュレーション p.24 の図面と p.30 の寸法写真)。
# 掲示板は⑤狭い道をまたぐ門型フレームに吊られ、支柱は道の左右の
# 紅白の板 (外壁) の上に立つ。走路の中に柱は立たない。
#   LED 掲示板 93cm x 19cm / 下端 65cm / 支柱 155cm (100cm+45cm SUS を連結)
#   足元は 30cm の SUS 材を壁の上に載せてクランプで固定する。
# 図面実測: フレームの帯は x 4.64〜4.91 (中心 4.775)。掲示板は左右の
# 支柱の中央 = 狭い道の中央 (仕切り y=5.557 の上) に来る。
ARROW_SIGN = dict(
    x=4.775,
    post_y=(4.743, 6.340),
    board_w=0.93, board_h=0.19, board_z0=0.65,
    post_h=1.55, post_r=0.012,
)

GIMMICK_AREAS = (
    # ④ 低μ/高μ路: 外側が人工芝(高μ)、その内側に滑り板(低μ)が置かれる。
    #    境界を跨ぐと左右輪の摩擦が変わってスピンするので、走路は芝側に寄せる。
    ('MU_HIGH',  8.44, 3.24, 10.23, 6.33),   # 人工芝
    ('MU_LOW',   8.40, 3.93,  9.36, 5.52),   # 滑り板 (芝の内側)
    # ⑦ でこぼこ道: 内側のみ。外側を通れば避けられる。
    #    実物は 40x80cm の「お風呂すべり止めマット(ブルー)」4枚 = 80x160cm
    #    (レギュレーション p.31)。色は明るい水色。
    ('ROUGH',    0.92, 2.61,  1.71, 4.15),
    # ② 坂道ショートカット
    ('SLOPE',    6.14, 2.84,  7.81, 3.70),
    # ① トンネル (SUS骨組み+漆黒シート)。右下の八角部を覆う。
    ('TUNNEL',   8.43, 0.60, 10.20, 3.16),
    # ③ ライトかく乱ゾーン: 照らされる範囲はレギュレーション p.28 の赤枠。
    #    赤枠は内側ヘアピンの U ターン部そのもの (四方を仕切りに囲まれた
    #    区画) で、その中心にライトが吊られる。
    ('LIGHT',    1.73, 1.89,  3.31, 4.74),
)


def surface_at(x, y):
    """(x, y) がどのギミック路面の上か。重なりは内側 (低μ) を優先。"""
    hits = [n for n, x0, y0, x1, y1 in GIMMICK_AREAS
            if x0 <= x <= x1 and y0 <= y <= y1]
    for name in ('MU_LOW', 'ROUGH', 'SLOPE', 'TUNNEL', 'MU_HIGH', 'LIGHT'):
        if name in hits:
            return name
    return None


# =====================================================================
# 走行中心線。p.21 の矢印の向きに合わせた蛇行ライン。
# 2 通りのルートを用意してある (default_course(use_shortcut=...) で選択)。
# =====================================================================

# --- ショートカット経路 (既定) --------------------------------------
# ②坂道ショートカット (y≈3.21 の仕切りの切れ目 x 6.19-7.72) で
# L2 → L3 を直接つなぎ、左の U ターンごと ③ライトかく乱ゾーンを通らない。
# L2/L3 の西側 (x 2.1-7.0) を丸ごと省略するので約 10m 短くなる。
_WAYPOINTS_SHORTCUT = [
    # --- L1 下段レーンを右へ (★スタート/ゴール) ---
    (2.00, 1.21), (3.20, 1.21), (4.60, 1.21), (6.00, 1.21),
    (7.40, 1.21), (8.60, 1.22), (9.20, 1.30),
    # --- 右下で上へ (①トンネル) ---
    (9.55, 1.60), (9.62, 2.00), (9.55, 2.35),
    # --- L2 を少しだけ左へ ---
    (9.20, 2.55), (8.60, 2.55), (8.10, 2.58),
    # --- ②坂道ショートカットで L2 → L3 (切れ目の中央 x≈7.0 を north 抜け) ---
    (7.70, 2.70), (7.32, 2.95), (7.08, 3.25), (7.02, 3.58), (7.16, 3.86),
    # --- L3 を右へ ---
    (7.60, 3.98), (8.10, 3.98),
    # --- 右上で上へ (④低μ/高μ路) ---
    (8.90, 4.10), (9.35, 4.45), (9.42, 4.90), (9.30, 5.35),
    # --- L4 上段レーンを左へ (⑤狭い道 / ⑥矢印信号) ---
    (8.90, 5.70), (8.10, 5.80), (7.00, 5.85), (6.20, 5.88),
    (5.60, 5.92), (4.80, 5.95), (4.00, 5.95), (3.30, 5.93),
    (2.60, 5.88), (1.85, 5.80),
    # --- 左チャネルを下へ (⑦でこぼこ道) ---
    (1.20, 5.55), (0.92, 5.10), (0.86, 4.60), (0.85, 4.10),
    (0.85, 3.70), (0.85, 3.20), (0.86, 2.70), (0.90, 2.20),
    (1.05, 1.75), (1.35, 1.42), (1.70, 1.25),
]

# --- 外周経路 (比較用) ----------------------------------------------
# ショートカットを使わず左端まで回り込む。③ライトかく乱ゾーンを通る。
_WAYPOINTS_LONG = [
    (2.00, 1.21), (3.20, 1.21), (4.60, 1.21), (6.00, 1.21),
    (7.40, 1.21), (8.60, 1.22), (9.20, 1.30),
    (9.55, 1.60), (9.62, 2.00), (9.55, 2.35),
    (9.20, 2.55), (8.20, 2.55), (7.00, 2.55), (5.80, 2.55),
    (4.60, 2.55), (3.60, 2.55),
    # 左で U ターン (③ライトかく乱ゾーン)
    (3.00, 2.60), (2.50, 2.78), (2.15, 3.10), (2.10, 3.27),
    (2.18, 3.62), (2.55, 3.88), (3.05, 3.98),
    (4.20, 3.98), (5.50, 3.98), (6.95, 3.98), (8.10, 3.98),
    (8.90, 4.10), (9.35, 4.45), (9.42, 4.90), (9.30, 5.35),
    (8.90, 5.70), (8.10, 5.80), (7.00, 5.85), (6.20, 5.88),
    (5.60, 5.92), (4.80, 5.95), (4.00, 5.95), (3.30, 5.93),
    (2.60, 5.88), (1.85, 5.80),
    (1.20, 5.55), (0.92, 5.10), (0.86, 4.60), (0.85, 4.10),
    (0.85, 3.70), (0.85, 3.20), (0.86, 2.70), (0.90, 2.20),
    (1.05, 1.75), (1.35, 1.42), (1.70, 1.25),
]

# ギミックの代表位置 (m)。周回進捗の範囲は default_course で自動計算する。
# ルートごとに通過するギミックが違う。
_FEATURES_COMMON = {
    'TUNNEL':     (9.62, 2.00),   # ① 右下の八角コーナー
    'MU_LOW':     (9.42, 4.90),   # ④ 右上 (人工芝 / PTFE)
    'ARROW_GATE': (5.60, 5.92),   # ⑥ 矢印信号ゲートの手前
    'NARROW':     (4.20, 5.95),   # ⑤ 狭い道 (仕切り x 3.11-5.03)
    'ROUGH':      (0.85, 3.70),   # ⑦ でこぼこ道 (左チャネル)
}
# ショートカット経路: ②坂道を実際に通る / ③ライトかく乱は通らない
_FEATURES_SHORTCUT = dict(_FEATURES_COMMON,
                          SLOPE=(7.08, 3.25))
# 外周経路: ③ライトかく乱を通る / ②坂道は脇を通過するだけ
_FEATURES_LONG = dict(_FEATURES_COMMON,
                      LIGHT_DISTURB=(2.10, 3.27),
                      SLOPE=(6.95, 3.98))

# 駐車枠 (⑧)。(name, color, x0, y0, x1, y1)  ※p.24 実測
PARKING_SLOTS = [
    ('P1', 'green', 2.113, 0.00, 3.428, 0.50),
    ('P2', 'red',   3.428, 0.00, 4.707, 0.50),
    ('P3', 'blue',  4.707, 0.00, 5.984, 0.50),
]


# =====================================================================
# 寸法の較正 (2026-09-19)。上の座標は p.24 の図形を PDF から抜いて全幅 1030cm に
# 一様スケールしたものだが、p.24 は寸法と比例しない模式図で、縦方向が合っていなかった:
#   レーン幅 (壁〜壁)  図形 1.36 / 1.32 / 1.53 / 1.60 m  ←→ 記載寸法はすべて 140cm (計 560cm)
#   ⑤狭い道           図形 0.81 / 0.78 m               ←→ 記載寸法 70 / 70cm
# 判定に使う寸法は記載値から取る (図形は位相と x の配置だけを信じる)。
# y を区分線形で写す: 駐車枠の下端 0 / 下辺 0.50 / 仕切り 1.90, 3.30, 4.70 /
# ⑤仕切り 5.40 / 上辺 6.10。上辺は図形では 8cm 傾いているので 6.10 に揃える。
# x は寸法記載が飛び飛びで図形と突き合わせ切れないので触っていない (③の U ターン部は
# 図形 1.58 m ←→ 記載 190cm の差が残る。予選のショートカット経路は通らない)。
# =====================================================================
_Y_ANCHORS = ((0.000, 0.00), (0.533, 0.50), (1.893, 1.90), (3.215, 3.30),
              (4.743, 4.70), (5.557, 5.40), (6.290, 6.10))


def fix_y(y):
    """図形から読んだ y [m] を、記載寸法に合わせた y へ写す。"""
    return round(float(np.interp(y, [a for a, _ in _Y_ANCHORS],
                                 [b for _, b in _Y_ANCHORS])), 3)


WALLS_DRAWN_M = list(WALLS_M)          # 較正前 (図形そのまま)。比較用に残す

# ---------------------------------------------------------------------
# x 方向の較正 (2026-09-19)。図は x にも歪んでいる (右 280cm は約半分の縮尺) うえ、板を
# 固定サイズの記号で描いている (180cm 板 = 支柱芯々 186cm を 192cm 相当で描画)。
# 壁を「板の規格長 + 記載寸法」から組み直す。板の内訳は図と部材表が一致 (180x23 / 90x12 / 45x4)。
#   支柱芯々: 180cm 板 1.86 m / 90cm 板 0.95 m / 45cm 板 0.49 m (p.24: 板端とポストセンターは 20mm)
#   x の基準: 左壁 0、左の縦仕切り 1.70 (「170」)、右壁 10.30 (「1030」)
#   L1/L2・L3/L4 仕切り: 縦仕切りの端から 90cm 板を 30 度で 1 枚 (dx 0.823) + 180cm 板 3 枚
#       → 先端 x = 8.103 (記載チェーンの 800、トンネルヘアピンの開口 2.20 m = 記載 (220) と整合。図形は 8.74)
#     縦仕切りの長さ = 4.225 - 2.375 = 1.85 m で 180cm 板 1 枚と合う
#   L2/L3 仕切り: 西端 = 1.70 + 「190」 = 3.60 から 180 + 90cm 板、東側は右壁のくさび (90cm x 2) から 180cm 板
#       → 切れ目 (②坂道) は x 6.41〜7.95
#   駐車枠: 180cm 板 2 枚 = 3.72 m = (124) x 3。左端 1.86 (スタート 1/2/3 = 250/430/610 と整合)
#   上辺: 左上の斜め板の先 0.194 から 90 + 180 x 2、⑥ゲートの「10」cm の切れ目、180 x 2 → 東端 8.684
#   右の外周は板の長さでは閉じ切らない = 規約の「成り行きで設置してばらつきを吸収」。図の作者と同じ
#   解き方にする: 右壁 (180cm x 2) の下端を y=1.42 (図形どおり) に置くと、右上の斜め板 2 枚 (19, 20) は
#   規格長ちょうどで閉じる。足りないぶんは右下の斜め板 23 に寄せる (1.25 m。図も 1.13 m に伸ばして
#   描いている)。下辺の東端は記載の 945 で、最後の板 7 も 0.15 m 伸ばしてある。
# ---------------------------------------------------------------------
_YB, _Y1, _Y2, _Y3, _Y5, _YT = 0.50, 1.90, 3.30, 4.70, 5.40, 6.10
_XD, _TIP = 1.70, 1.70 + 0.823 + 3 * 1.86          # 縦仕切り / L1-L2・L3-L4 仕切りの先端 (8.103)
_RW, _RW_Y0 = 10.30, 1.42                          # 右壁 / 右壁の下端
# 右上の斜め板 2 枚の継ぎ目: 上辺の東端 (8.684, 6.10) と右壁の上端から 0.95 m ずつ、外側へ張り出す点
_ax, _ay = 8.684, _YT
_bx, _by = _RW, _RW_Y0 + 3.72
_h = math.hypot(_bx - _ax, _by - _ay) / 2.0
_o = math.sqrt(max(0.0, 0.95 ** 2 - _h ** 2))
_APEX = (round((_ax + _bx) / 2 + _o * (_ay - _by) / (2 * _h), 3),
         round((_ay + _by) / 2 + _o * (_bx - _ax) / (2 * _h), 3))
WALLS_M = [
    # --- 外周: 左辺 (0-3) ---
    (0.0, _YB, 0.0, 2.36), (0.0, 2.36, 0.0, 4.22), (0.0, 4.22, 0.0, 5.17), (0.194, _YT, 0.0, 5.17),
    # --- 外周: 下辺 (4-7)。5 は左下の内コーナーの斜め板 ---
    (1.86, _YB, 0.0, _YB), (0.856, 0.60, 0.184, 1.272),
    (7.44, _YB, 5.58, _YB), (9.45, _YB, 7.44, _YB),
    # --- 駐車枠 (8-13) ---
    (3.72, 0.0, 1.86, 0.0), (5.58, 0.0, 3.72, 0.0),
    (1.86, 0.0, 1.86, 0.49), (3.10, 0.0, 3.10, 0.49), (4.34, 0.0, 4.34, 0.49), (5.58, 0.0, 5.58, 0.49),
    # --- 外周: 上辺 (14-18)。16 と 17 の間の 10cm が ⑥ゲート ---
    (0.194, _YT, 1.144, _YT), (3.004, _YT, 1.144, _YT), (4.864, _YT, 3.004, _YT),
    (6.824, _YT, 4.964, _YT), (8.684, _YT, 6.824, _YT),
    # --- 外周: 右辺 (19-23) ---
    (_APEX[0], _APEX[1], 8.684, _YT), (_RW, _RW_Y0 + 3.72, _APEX[0], _APEX[1]),
    (_RW, _RW_Y0 + 1.86, _RW, _RW_Y0 + 3.72), (_RW, _RW_Y0, _RW, _RW_Y0 + 1.86),
    (_RW, _RW_Y0, 9.45, _YB),
    # --- 内壁: L1 / L2 の仕切り (24-27) ---
    (_XD + 0.823, _Y1, _XD, _Y1 + 0.475), (_XD + 0.823 + 1.86, _Y1, _XD + 0.823, _Y1),
    (_XD + 0.823 + 3.72, _Y1, _XD + 0.823 + 1.86, _Y1), (_TIP, _Y1, _XD + 0.823 + 3.72, _Y1),
    # --- 内壁: 左の縦仕切り (28) ---
    (_XD, _Y1 + 0.475, _XD, _Y3 - 0.475),
    # --- 内壁: L2 / L3 の仕切り (29-31)。切れ目 6.41〜7.95 が ②坂道 ---
    (5.46, _Y2, 3.60, _Y2), (6.41, _Y2, 5.46, _Y2), (9.812, _Y2, 7.952, _Y2),
    # --- 内壁: 右のくさび (32-33) ---
    (_RW, _Y2 - 0.815, 9.812, _Y2), (_RW, _Y2 + 0.815, 9.812, _Y2),
    # --- 内壁: L3 / L4 の仕切り (34-37) ---
    (_XD, _Y3 - 0.475, _XD + 0.823, _Y3), (_XD + 0.823 + 1.86, _Y3, _XD + 0.823, _Y3),
    (_XD + 0.823 + 3.72, _Y3, _XD + 0.823 + 1.86, _Y3), (_TIP, _Y3, _XD + 0.823 + 3.72, _Y3),
    # --- 内壁: ⑤狭い道 の仕切り (38)。東端が ⑥ゲート ---
    (4.964, _Y5, 3.104, _Y5),
]
assert len(WALLS_M) == len(WALLS_DRAWN_M) == len(WALL_COLORS)


def _fix_wp(x, y):
    """図形の座標で書いた中心線の点を、較正後の座標へ写す (y は fix_y、x は下の 3 か所だけ)。"""
    if 2.62 <= y <= 3.90 and 6.90 < x < 7.80:   # ②坂道: 切れ目の中央 6.955 → 7.18
        x += 0.22
    elif 2.20 < y < 4.10 and 1.731 < x < 3.70:     # ③U ターン (外周経路): 縦仕切り 1.731 → 1.70、仕切りの西端 3.31 → 3.60
        x = float(np.interp(x, [1.731, 3.31, 3.70], [_XD, 3.60, 3.80]))
    return (round(x, 3), fix_y(y))


def _rebuild(wps):
    """中心線を較正後の座標へ。①トンネルのヘアピン (図形の x>7.40, y<2.62 の点) は写像せず、
    L1 (y=1.20) と L2 (y=2.60) をつなぐ半径 0.70 m の半円を仕切りの先端の少し先に置き直す。
    図形の形を x に引き伸ばすと横長の楕円になって頂点の曲率がきつくなり、写像の継ぎ目で
    中心線が逆戻りした (2026-09-19: cte p95 0.17 → 0.20 m)。"""
    # 中心の x は先端 + 0.15 m (先端に寄せる)。A/B (2026-09-19、周回カウンタの実ラップ):
    #   +0.15 → 13.0 s/周・cte p95 0.24〜0.28 m / +0.45 → 13.7 s・0.20〜0.24 / +0.75 → 13.8 s・0.18〜0.19
    # 先端に寄せると ②坂道の入口 (x 7.18) まで直線が約 1 m で S 字が詰まり、車が内側を切るので cte は出るが、
    # 壁余裕は同じ (車体外周 min 0.21 m、p10 0.37 m)。判定の順 (安全 → タイム → 追従) で +0.15 を採る。
    # ⇒ 次の一手は、車が実際に走る線 (S 字の内側) を参照線にすること。
    cx, cy, r = _TIP + 0.15, (_YB + 2 * _Y1 + _Y2) / 4.0, 0.70      # (8.25, 1.90)
    arc = [(round(cx + r * math.cos(math.radians(a)), 3), round(cy + r * math.sin(math.radians(a)), 3))
           for a in range(-90, 91, 30)] + [(round(cx - 0.30, 3), round(cy + r, 3))]
    out, done = [], False
    for x, y in wps:
        if y < 2.62 and x > 7.40:
            if not done:
                out.extend(arc)
                done = True
            continue
        out.append(_fix_wp(x, y))
    return out


_WAYPOINTS_SHORTCUT = _rebuild(_WAYPOINTS_SHORTCUT)
_WAYPOINTS_LONG = _rebuild(_WAYPOINTS_LONG)
_FEATURES_SHORTCUT = {k: _fix_wp(x, y) for k, (x, y) in _FEATURES_SHORTCUT.items()}
_FEATURES_LONG = {k: _fix_wp(x, y) for k, (x, y) in _FEATURES_LONG.items()}
for _f in (_FEATURES_SHORTCUT, _FEATURES_LONG):
    _f['TUNNEL'] = (_TIP + 0.15 + 0.70, 1.90)       # ヘアピンの頂点
PARKING_SLOTS = [('P1', 'green', 1.86, 0.0, 3.10, 0.50), ('P2', 'red', 3.10, 0.0, 4.34, 0.50),
                 ('P3', 'blue', 4.34, 0.0, 5.58, 0.50)]
LIGHT_POS = (_XD + 0.95, fix_y(LIGHT_POS[1]), LIGHT_POS[2])      # 縦仕切りから 95cm (p.28)
ARROW_SIGN = dict(ARROW_SIGN, x=4.914, post_y=(_Y3, _YT))        # 上辺の 10cm の切れ目の中央
# ギミック領域のうち寸法の記載があるものは記載値で置く:
#   ④滑り板 (100)x(50+50)cm: 上辺から 90cm 下が北端、右の芝 (85)cm を残す (p.24 / p.29)
#   ②坂道 110 x 60cm (p.24 △1、p.27): L2/L3 仕切りの切れ目の中央に置く
_SPEC_AREAS = {
    'MU_LOW': (8.45, 4.20, 9.45, 5.20),
    'SLOPE':  (6.63, 3.00, 7.73, 3.60),            # 切れ目 6.41〜7.95 の中央
    'TUNNEL': (7.50, 0.56, 10.20, 3.25),           # グレーの左端 = 記載の 750
    'ROUGH':  (0.90, 2.65, 1.70, 4.25),            # 80 x 160cm、縦仕切り (1.70) に接する
    'LIGHT':  (1.70, 1.90, 3.60, 4.70),            # 縦仕切り〜L2/L3 仕切りの西端 (「190」)
}
GIMMICK_AREAS = tuple(
    (n, *_SPEC_AREAS[n]) if n in _SPEC_AREAS else (n, x0, fix_y(y0), x1, fix_y(y1))
    for n, x0, y0, x1, y1 in GIMMICK_AREAS)
if os.environ.get('MINICAR_PLATE_DXY', ''):          # 感度確認用: 滑り板の実物が地図からずれていた場合
    _pdx, _pdy = (float(v) for v in os.environ['MINICAR_PLATE_DXY'].split(','))
    GIMMICK_AREAS = tuple((n, x0 + _pdx, y0 + _pdy, x1 + _pdx, y1 + _pdy) if n == 'MU_LOW' else (n, x0, y0, x1, y1)
                          for n, x0, y0, x1, y1 in GIMMICK_AREAS)


def _resample(poly, step=0.06):
    """ポリラインを等間隔にリサンプルして滑らかにする。"""
    poly = np.asarray(poly, dtype=np.float64)
    seg = np.diff(poly, axis=0)
    seglen = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seglen)])
    s = np.arange(0.0, cum[-1], step)
    return np.column_stack([np.interp(s, cum, poly[:, 0]),
                            np.interp(s, cum, poly[:, 1])])


def _smooth(poly, iters=6, alpha=0.35):
    """閉ループを保ったまま移動平均で滑らかにする (急な折れを丸める)。"""
    p = poly.copy()
    for _ in range(iters):
        prev = np.roll(p, 1, axis=0)
        nxt = np.roll(p, -1, axis=0)
        p = (1 - alpha) * p + alpha * 0.5 * (prev + nxt)
    return p


NARROW_DIVIDER_INDEX = 38      # WALLS_M の中の ⑤狭い道 中央仕切り


def default_course(use_shortcut=True, narrow_divider=True):
    """
    レギュレーション p.24 の実寸形状を再現したコース。

    use_shortcut=True  (既定) ②坂道ショートカットを通る短い経路。
                              ③ライトかく乱ゾーンは通らない。
    use_shortcut=False        左端まで回り込む外周経路。
                              ③ライトかく乱ゾーンを通る。

    narrow_divider=False      ⑤狭い道の中央仕切りを外す (比較検証用)。
                              仕切りは試走会/予選/決勝のすべてで設置される
                              (レギュレーション △3 p.30-31) ので既定は True。

    壁は両ルートで共通なので、比較走行はルート指定を変えるだけでできる。
    """
    wp = _WAYPOINTS_SHORTCUT if use_shortcut else _WAYPOINTS_LONG
    features = _FEATURES_SHORTCUT if use_shortcut else _FEATURES_LONG

    pts = np.asarray(wp, dtype=np.float64)
    center = _smooth(_resample(np.vstack([pts, pts[:1]]), step=0.06))
    center = np.vstack([center, center[:1]])   # 閉ループ

    walls, colors = WALLS_M, WALL_COLORS
    # 感度確認用: 環境変数 MINICAR_NARROW_LANE_M で ⑤狭い道の外側レーン幅を指定する
    # (仕切りを外壁 y=6.10 側へ寄せる)。既定は記載寸法の 0.70 m (壁芯)。内法は板厚と
    # 支柱の張り出しで 0.66〜0.68 m になり得る。地図 (route.yaml) は変えない = 実物が
    # 地図より狭かった場合の再現。
    lane = os.environ.get('MINICAR_NARROW_LANE_M', '')
    if lane and narrow_divider:
        x0, y0, x1, y1 = WALLS_M[NARROW_DIVIDER_INDEX]
        yd = 6.10 - float(lane)             # 外壁 (上辺) からレーン幅だけ内側
        walls = list(WALLS_M)
        walls[NARROW_DIVIDER_INDEX] = (x0, yd, x1, yd)
    # 感度確認用: 環境変数 MINICAR_GEOM (JSON) で「実物が地図と違っていた場合」を作る。地図 (route.yaml と
    # make_route.py が見る WALLS_M) は変えず、sim の真値の壁だけを動かす。推定で置いた寸法 (x 方向・右の外周) 用。
    #   tip_dx     L1/L2・L3/L4 仕切りの先端を東へ [m] (最後の板の長さで吸収)
    #   rw_y0      右壁の下端の y [m] (既定 1.42)。右下と右上の斜め板の形が変わる
    #   top_end_dx 上辺の東端を東へ [m] (右上の斜め板の起点)
    # 滑り板の位置は MINICAR_PLATE_DXY="dx,dy" (モジュール読込時に GIMMICK_AREAS を動かす)。
    geom = os.environ.get('MINICAR_GEOM', '')
    if geom:
        import json
        g = json.loads(geom)
        walls = list(walls)
        dx = float(g.get('tip_dx', 0.0))
        for idx in (27, 37):
            x0, y0, x1, y1 = walls[idx]
            walls[idx] = (x0 + dx, y0, x1, y1)
        if 'rw_y0' in g or 'top_end_dx' in g:
            ry0 = float(g.get('rw_y0', _RW_Y0))
            tx = 8.684 + float(g.get('top_end_dx', 0.0))
            bx, by = _RW, ry0 + 3.72
            h = math.hypot(bx - tx, by - _YT) / 2.0
            o = math.sqrt(max(0.0, 0.95 ** 2 - h ** 2))
            ap = ((tx + bx) / 2 + o * (_YT - by) / (2 * h), (_YT + by) / 2 + o * (bx - tx) / (2 * h))
            x0, y0, x1, y1 = walls[18]
            walls[18] = (tx, y0, x1, y1)
            walls[19] = (ap[0], ap[1], tx, _YT)
            walls[20] = (_RW, by, ap[0], ap[1])
            walls[21] = (_RW, ry0 + 1.86, _RW, by)
            walls[22] = (_RW, ry0, _RW, ry0 + 1.86)
            walls[23] = (_RW, ry0, 9.45, _YB)
    if not narrow_divider:
        # 比較検証用: ⑤狭い道の中央仕切りを外す (色配列も添字を揃えて外す)
        walls = [w for i, w in enumerate(WALLS_M) if i != NARROW_DIVIDER_INDEX]
        colors = [c for i, c in enumerate(WALL_COLORS)
                  if i != NARROW_DIVIDER_INDEX]

    course = Course(center, walls, width_m=0.60, wall_colors=colors,
                    gimmicks=[], slots=PARKING_SLOTS)
    course.narrow_divider = bool(narrow_divider)
    course.use_shortcut = bool(use_shortcut)

    total = course.total_length
    window = 0.022        # 進捗 ±2.2% (≒ ±0.8m) を区間幅とする
    gims = []
    for name, (fx, fy) in features.items():
        d2 = ((course.center[:, 0] - fx) ** 2 + (course.center[:, 1] - fy) ** 2)
        s = course.cum_len[int(np.argmin(d2))] / total
        gims.append((name, max(0.0, s - window), min(1.0, s + window)))
    gims.sort(key=lambda g: g[1])
    course.gimmicks = gims
    return course
