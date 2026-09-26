# -*- coding: utf-8 -*-
"""
ピンホールカメラの幾何 (sim の OpenCV 描画・Unity 描画と同じ式)。

車体座標 (x 前・y 左・z 上、原点 = base_link = 後軸中心・床面) の点を
/camera/image_raw (上部クロップ後) の画素座標に写す、またはその逆。

  - vehicle_sim._cam_basis と同じ基底: 右 = (0,-1,0)、前 = (cosφ,0,-sinφ)、下 = 前 × 右
  - 焦点距離 f = (W/2) / tan(hfov/2)
  - クロップは描画後に上端を crop_top_px 行捨てるだけなので、主点 cy はクロップ前の値のまま

ground_truth の (u, v) と cmd_shaper の逆変換が **同じこの式** を使うことが、
「ラベルの点が走路の上に来る」ための条件。数値は vehicle_profile の camera から入れる。
"""

import math

import numpy as np


class CamGeom:

    def __init__(self, width, height, hfov_deg, mount_height_m, pitch_deg,
                 crop_top_frac=0.0, mount_x_m=0.0, mount_y_m=0.0):
        self.w = int(width)
        self.h = int(height)
        self.hfov = math.radians(float(hfov_deg))
        self.hm = float(mount_height_m)
        self.pitch = math.radians(float(pitch_deg))       # 下向きが正
        self.crop_px = int(round(self.h * min(0.6, max(0.0, float(crop_top_frac)))))
        self.mx = float(mount_x_m)
        self.my = float(mount_y_m)
        self.f = (self.w / 2.0) / math.tan(self.hfov / 2.0)
        self.cx = self.w / 2.0
        self.cy = self.h / 2.0
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        self.fwd = np.array([c, 0.0, -s])
        self.right = np.array([0.0, -1.0, 0.0])
        self.down = np.cross(self.fwd, self.right)
        self.C = np.array([self.mx, self.my, self.hm])

    @classmethod
    def from_profile(cls, cam):
        """vehicle_profile.yaml の camera 節 (dict) から作る。"""
        return cls(cam['width'], cam['height'], cam['hfov_deg'],
                   cam['mount_height_m'], cam['pitch_deg'],
                   cam.get('crop_top_frac', 0.0), cam.get('mount_x_m', 0.0),
                   cam.get('mount_y_m', 0.0))

    @property
    def out_height(self):
        """クロップ後の画像の高さ [px]。"""
        return self.h - self.crop_px

    def K(self):
        """クロップ後画像の内部パラメータ行列 (CameraInfo.k 用)。"""
        return np.array([[self.f, 0.0, self.cx],
                         [0.0, self.f, self.cy - self.crop_px],
                         [0.0, 0.0, 1.0]])

    # ------------------------------------------------------------------
    def project(self, xb, yb, zb=0.0):
        """車体座標の点 → (u, v, visible)。v はクロップ後の座標。"""
        p = np.array([float(xb), float(yb), float(zb)]) - self.C
        zc = float(p @ self.fwd)
        if zc <= 1e-6:
            return float('nan'), float('nan'), False
        u = self.cx + self.f * float(p @ self.right) / zc
        v = self.cy + self.f * float(p @ self.down) / zc - self.crop_px
        visible = (0.0 <= u < self.w) and (0.0 <= v < self.out_height)
        return u, v, visible

    def ground_from_pixel(self, u, v):
        """クロップ後の画素 (u, v) → 床面 (z=0) の車体座標 (xb, yb)。地平線より上なら None。"""
        xn = (float(u) - self.cx) / self.f
        yn = (float(v) + self.crop_px - self.cy) / self.f
        ray = self.fwd + xn * self.right + yn * self.down
        if ray[2] >= -1e-9:            # 下を向いていない = 床と交わらない
            return None
        t = -self.C[2] / ray[2]
        p = self.C + t * ray
        if p[0] <= 0.0:
            return None
        return float(p[0]), float(p[1])

    def horizon_row(self):
        """地平線の行 (クロップ後)。"""
        ray = self.fwd
        # 無限遠の床は fwd の水平成分方向。down 成分 = 0 の行
        p = np.array([1e6 * ray[0], 1e6 * ray[1], 0.0]) - self.C
        zc = float(p @ self.fwd)
        return self.cy + self.f * float(p @ self.down) / zc - self.crop_px
