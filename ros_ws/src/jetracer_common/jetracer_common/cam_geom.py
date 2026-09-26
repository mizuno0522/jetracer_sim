# -*- coding: utf-8 -*-
"""
カメラの幾何 (sim の OpenCV 描画・Unity 描画・ラベル・cmd_shaper が同じ式を使う)。

車体座標 (x 前・y 左・z 上、原点 = base_link = 後軸中心・床面) の点を
/camera/image_raw (上部クロップ後) の画素座標に写す、またはその逆。

モデルは OpenCV の plumb_bob (チェッカーボード較正 cv2.calibrateCamera の出力をそのまま入れられる形):
    xn = X/Z, yn = Y/Z                        正規化座標 (ピンホール)
    r² = xn² + yn²
    xd = xn (1 + k1 r² + k2 r⁴)              半径方向の歪み (k1 < 0 で樽型)。接線方向 p1,p2 と k3 は 0 に固定
    u = cx + fx xd,   v = cy + fy yd
  - fx は hfov_deg から fx = (W/2) / tan(hfov/2)。fy は vfov_deg があればそこから、無ければ fx (正方画素)。
    実機の取り込みが「4:3 や 16:9 を 224×224 に縮めている」なら fy ≠ fx になる (tools/camera_calib.py で測る)。
    profile に fx / fy が直接書いてあればそちらを優先する。
  - hfov_deg は「ピンホール等価の画角」(fx の言い換え)。樽型歪みがあると画像の端から端の実際の見込み角は
    これより広い (true_fov_deg() で出す)。
  - 基底は vehicle_sim._cam_basis と同じ: 右 = (0,-1,0)、前 = (cosφ,0,-sinφ)、下 = 前 × 右
  - クロップは描画後に上端を crop_top_px 行捨てるだけなので、主点 cy はクロップ前の値のまま

ground_truth の (u, v) と cmd_shaper の逆変換が **同じこの式** を使うことが、
「ラベルの点が走路の上に来る」ための条件。数値は vehicle_profile の camera から入れる。
"""

import math

import numpy as np


class CamGeom:

    def __init__(self, width, height, hfov_deg, mount_height_m, pitch_deg,
                 crop_top_frac=0.0, mount_x_m=0.0, mount_y_m=0.0,
                 vfov_deg=0.0, k1=0.0, k2=0.0, fx=0.0, fy=0.0, cx=None, cy=None):
        self.w = int(width)
        self.h = int(height)
        self.hm = float(mount_height_m)
        self.pitch = math.radians(float(pitch_deg))       # 下向きが正
        self.crop_px = int(round(self.h * min(0.6, max(0.0, float(crop_top_frac)))))
        self.mx = float(mount_x_m)
        self.my = float(mount_y_m)
        if fx and fx > 0:
            self.fx = float(fx)
        else:
            self.fx = (self.w / 2.0) / math.tan(math.radians(float(hfov_deg)) / 2.0)
        if fy and fy > 0:
            self.fy = float(fy)
        elif vfov_deg and vfov_deg > 0:
            self.fy = (self.h / 2.0) / math.tan(math.radians(float(vfov_deg)) / 2.0)
        else:
            self.fy = self.fx
        self.f = self.fx                                    # 互換 (ログ表示など)
        self.hfov = 2.0 * math.atan((self.w / 2.0) / self.fx)
        self.cx = self.w / 2.0 if cx is None or cx < 0 else float(cx)
        self.cy = self.h / 2.0 if cy is None or cy < 0 else float(cy)
        self.k1 = float(k1)
        self.k2 = float(k2)
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        self.fwd = np.array([c, 0.0, -s])
        self.right = np.array([0.0, -1.0, 0.0])
        self.down = np.cross(self.fwd, self.right)
        self.C = np.array([self.mx, self.my, self.hm])
        # 樽型 (k1<0) の順写像 r → r(1+k1 r²+k2 r⁴) が単調に増える範囲。これを超える点は折り返して写るので不可視扱い
        self.r_max = self._monotonic_limit()
        # 歪んだ側で届く最大半径。画像の四隅がこれを超えるなら、その k1/k2 と画角の組み合わせは物理的にあり得ない
        # (樽型の式が折り返す)。黙って変な画を出さないよう、ここで止める
        if math.isfinite(self.r_max):
            rm = self.r_max
            self.rd_max = rm * (1.0 + self.k1 * rm * rm + self.k2 * rm ** 4)
            corners = [((u - self.cx) / self.fx, (v - self.cy) / self.fy) for u in (0, self.w) for v in (0, self.h)]
            rc = max(math.hypot(x, y) for x, y in corners)
            if rc >= 0.98 * self.rd_max:
                raise ValueError(
                    f"カメラの歪みが強すぎる: 画像の四隅 (正規化半径 {rc:.2f}) が k1={self.k1:+.4f}, k2={self.k2:+.4f} の"
                    f"届く範囲 {self.rd_max:.2f} を超える。OpenCV の正規化座標 (x = X/Z) での値か、画角と組で確認すること")
        else:
            self.rd_max = float('inf')

    @classmethod
    def from_profile(cls, cam):
        """vehicle_profile.yaml の camera 節 (dict) から作る。"""
        return cls(cam['width'], cam['height'], cam['hfov_deg'],
                   cam['mount_height_m'], cam['pitch_deg'],
                   cam.get('crop_top_frac', 0.0), cam.get('mount_x_m', 0.0),
                   cam.get('mount_y_m', 0.0),
                   vfov_deg=cam.get('vfov_deg', 0.0) or 0.0,
                   k1=cam.get('k1', 0.0) or 0.0, k2=cam.get('k2', 0.0) or 0.0,
                   fx=cam.get('fx', 0.0) or 0.0, fy=cam.get('fy', 0.0) or 0.0,
                   cx=cam.get('cx', None), cy=cam.get('cy', None))

    # ------------------------------------------------------------------ 歪み
    @property
    def distorted(self):
        return self.k1 != 0.0 or self.k2 != 0.0

    def _monotonic_limit(self):
        if not self.distorted:
            return float('inf')
        # d/dr [r(1+k1 r²+k2 r⁴)] = 1 + 3k1 r² + 5k2 r⁴ が 0 になる最小の r
        rs = np.linspace(0.0, 20.0, 20001)
        d = 1.0 + 3.0 * self.k1 * rs ** 2 + 5.0 * self.k2 * rs ** 4
        bad = np.nonzero(d <= 0.0)[0]
        return float(rs[bad[0]]) if len(bad) else float('inf')

    def distort(self, xn, yn):
        """正規化座標 (ピンホール) → 歪んだ正規化座標。numpy 配列可。"""
        xn = np.asarray(xn, float)
        yn = np.asarray(yn, float)
        r2 = xn * xn + yn * yn
        g = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
        return xn * g, yn * g

    def undistort(self, xd, yd, iters=30):
        """歪んだ正規化座標 → ピンホールの正規化座標 (不動点反復。樽型の単調範囲で収束)。"""
        xd = np.asarray(xd, float)
        yd = np.asarray(yd, float)
        if not self.distorted:
            return xd.copy(), yd.copy()
        x, y = xd.copy(), yd.copy()
        lim2 = self.r_max ** 2
        for _ in range(iters):
            r2 = np.minimum(x * x + y * y, lim2)     # 折り返す範囲に出ないよう抑える (発散防止)
            g = np.maximum(1e-3, 1.0 + self.k1 * r2 + self.k2 * r2 * r2)
            x = xd / g
            y = yd / g
        return x, y

    # ------------------------------------------------------------------ 画像
    @property
    def out_height(self):
        """クロップ後の画像の高さ [px]。"""
        return self.h - self.crop_px

    def K(self):
        """クロップ後画像の内部パラメータ行列 (CameraInfo.k 用)。"""
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy - self.crop_px],
                         [0.0, 0.0, 1.0]])

    def D(self):
        """plumb_bob の歪み係数 [k1, k2, p1, p2, k3] (CameraInfo.d 用)。"""
        return [self.k1, self.k2, 0.0, 0.0, 0.0]

    def undistorted_extent(self):
        """出力画像の枠が、ピンホールの正規化座標でどこまで届くか (x_min, x_max, y_min, y_max)。
        歪みを後処理で掛ける描画器 (OpenCV・Unity) は、ピンホールでこの範囲を描いてから写像する。"""
        us = np.concatenate([np.linspace(0, self.w, 65), np.full(65, self.w), np.linspace(0, self.w, 65), np.zeros(65)])
        vs = np.concatenate([np.zeros(65), np.linspace(0, self.h, 65), np.full(65, self.h), np.linspace(0, self.h, 65)])
        xu, yu = self.undistort((us - self.cx) / self.fx, (vs - self.cy) / self.fy)
        return float(xu.min()), float(xu.max()), float(yu.min()), float(yu.max())

    def true_fov_deg(self):
        """画像の左端から右端・上端から下端の実際の見込み角 [deg] (歪み込み)。"""
        x0, x1, y0, y1 = self.undistorted_extent()
        # 主点を通る水平線・垂直線上で測る
        xl, _ = self.undistort(np.array([(0 - self.cx) / self.fx, (self.w - self.cx) / self.fx]), np.zeros(2))
        _, yt = self.undistort(np.zeros(2), np.array([(0 - self.cy) / self.fy, (self.h - self.cy) / self.fy]))
        h = math.degrees(math.atan(-xl[0]) + math.atan(xl[1]))
        v = math.degrees(math.atan(-yt[0]) + math.atan(yt[1]))
        return h, v

    # ------------------------------------------------------------------ 写像
    def project(self, xb, yb, zb=0.0):
        """車体座標の点 → (u, v, visible)。v はクロップ後の座標。"""
        p = np.array([float(xb), float(yb), float(zb)]) - self.C
        zc = float(p @ self.fwd)
        if zc <= 1e-6:
            return float('nan'), float('nan'), False
        xn = float(p @ self.right) / zc
        yn = float(p @ self.down) / zc
        if xn * xn + yn * yn > self.r_max ** 2:
            return float('nan'), float('nan'), False
        xd, yd = self.distort(xn, yn)
        u = self.cx + self.fx * float(xd)
        v = self.cy + self.fy * float(yd) - self.crop_px
        visible = (0.0 <= u < self.w) and (0.0 <= v < self.out_height)
        return u, v, visible

    def ground_from_pixel(self, u, v):
        """クロップ後の画素 (u, v) → 床面 (z=0) の車体座標 (xb, yb)。地平線より上なら None。"""
        xd = (float(u) - self.cx) / self.fx
        yd = (float(v) + self.crop_px - self.cy) / self.fy
        xn, yn = self.undistort(xd, yd)
        ray = self.fwd + float(xn) * self.right + float(yn) * self.down
        if ray[2] >= -1e-9:            # 下を向いていない = 床と交わらない
            return None
        t = -self.C[2] / ray[2]
        p = self.C + t * ray
        if p[0] <= 0.0:
            return None
        return float(p[0]), float(p[1])

    def horizon_row(self):
        """地平線の行 (クロップ後、画像の中央の列で)。歪みがあると地平線は曲線になる。"""
        # 無限遠の床の方向 (fwd の水平成分) のカメラ座標
        d = np.array([self.fwd[0], self.fwd[1], 0.0])
        zc = float(d @ self.fwd)
        yn = float(d @ self.down) / zc
        _, yd = self.distort(0.0, yn)
        return self.cy + self.fy * float(yd) - self.crop_px
