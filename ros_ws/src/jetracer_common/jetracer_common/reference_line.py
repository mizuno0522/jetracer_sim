# -*- coding: utf-8 -*-
"""
参照線 (閉ループの折れ線) と、それに対する車両の位置づけ。

/sim/ground_truth の (s, cte, heading_err, curvature, 先行注視点) と、
lockstep の StepInfo.progress_m は **同じこの関数** から出す (設計「参照線の弧長を返す関数を共用」)。
既定はコース中心線 (course.py)。make_route.py の route.yaml も読める。
"""

import math

import numpy as np


class ReferenceLine:

    def __init__(self, points, closed=True, resample_m=0.02):
        pts = np.asarray(points, float)
        if closed and np.hypot(*(pts[0] - pts[-1])) > 1e-6:
            pts = np.vstack([pts, pts[:1]])
        # 等間隔に再サンプル (曲率と先読みが安定する)
        seg = np.diff(pts, axis=0)
        L = np.hypot(seg[:, 0], seg[:, 1])
        cum = np.concatenate([[0.0], np.cumsum(L)])
        n = max(8, int(cum[-1] / resample_m))
        s = np.linspace(0.0, cum[-1], n, endpoint=False)
        x = np.interp(s, cum, pts[:, 0])
        y = np.interp(s, cum, pts[:, 1])
        self.pts = np.stack([x, y], axis=1)
        self.s = s
        self.total = float(cum[-1])
        self.closed = closed
        d = np.roll(self.pts, -1, axis=0) - self.pts
        self.psi = np.arctan2(d[:, 1], d[:, 0])
        dpsi = np.diff(np.unwrap(np.concatenate([self.psi, self.psi[:1]])))
        ds = np.hypot(d[:, 0], d[:, 1])
        kappa = dpsi / np.maximum(1e-6, ds)
        # 曲率は 0.3 m 程度の窓で平均 (折れ線の刻みを消す)
        win = max(1, int(0.3 / resample_m))
        k = np.concatenate([kappa[-win:], kappa, kappa[:win]])
        self.kappa = np.convolve(k, np.ones(2 * win + 1) / (2 * win + 1), 'same')[win:-win]

    @classmethod
    def from_yaml_route(cls, path, name='shortcut'):
        import yaml
        with open(path) as f:
            d = yaml.safe_load(f)
        return cls(d[name]['waypoints'])

    # ------------------------------------------------------------------
    def nearest(self, x, y):
        """(弧長 s, 横偏差 d [左正], 接線方位 psi, index)。"""
        dx = self.pts[:, 0] - x
        dy = self.pts[:, 1] - y
        i = int(np.argmin(dx * dx + dy * dy))
        psi = float(self.psi[i])
        nx, ny = -math.sin(psi), math.cos(psi)
        d = (x - self.pts[i, 0]) * nx + (y - self.pts[i, 1]) * ny
        # 接線方向へ射影して弧長を補正
        along = (x - self.pts[i, 0]) * math.cos(psi) + (y - self.pts[i, 1]) * math.sin(psi)
        s = (self.s[i] + along) % self.total
        return float(s), float(d), psi, i

    def point_at(self, s):
        s = s % self.total
        i = int(np.searchsorted(self.s, s, side='right') - 1)
        i = max(0, min(len(self.s) - 1, i))
        j = (i + 1) % len(self.s)
        ds = self.s[j] - self.s[i] if j > i else self.total - self.s[i]
        f = (s - self.s[i]) / max(1e-9, ds)
        p = self.pts[i] + (self.pts[j] - self.pts[i]) * f
        return float(p[0]), float(p[1]), float(self.psi[i]), float(self.kappa[i])

    def lookahead(self, x, y, dist):
        """最近点から dist 先の参照線上の点 (x, y, psi, kappa)。"""
        s, _, _, _ = self.nearest(x, y)
        return self.point_at(s + dist)

    def heading_error(self, yaw, psi):
        return math.atan2(math.sin(psi - yaw), math.cos(psi - yaw))

    def progress_delta(self, s_prev, s_now):
        """弧長の前進量 (周回のラップを考慮。後退は負)。"""
        d = (s_now - s_prev) % self.total
        if d > self.total / 2.0:
            d -= self.total
        return d
