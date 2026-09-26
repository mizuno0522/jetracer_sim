#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetRacer (TT-02) 用の参照線を引く。config/route_<profile>.yaml を作る (設計 未決⑦)。

コース中心線 (course.py) は M-05 (WB 0.210 m) 向けに描かれていて、TT-02 (WB 0.257 m・δmax 27°) の
最小回転半径 R_min = 0.506 m を割る区間がある (坂道の右ヘアピン、R 0.42 m)。ここでは中心線を出発点に

  1. 既存 make_route.py (minicar_planning) の意図を「優先線」として引き継ぐ:
       ④ 低μ/高μ  … 外側 (人工芝) に寄せて境界を跨がない。滑り板の南側の横断は板の手前を通る
       ⑦ でこぼこ … 内側だけなので外側へ避ける
       ② 坂道     … 直角に入り直角に出る (x 一定)
  2. その優先線の近くで、曲率が 1/(R_min×margin) を超えないように、壁余裕を保ったまま
     最小曲率で引き直す (scipy least_squares・疎ヤコビアン)。

    python3 tools/make_route.py                       # jetracer_tt02 で config/route_jetracer_tt02.yaml
    python3 tools/make_route.py --delta-max 0.52      # δmax の実測値で引き直す
    python3 tools/make_route.py --plot /tmp/route.png # 確認画像 (壁・中心線・優先線・結果・きつい所)

出来たら tools/corridor_check.py --route <yaml> で机上判定し、sim_host.launch.py route_file:=<yaml> で走らせる。
コース形状 (course.py) は真値なので触らない。ここで作るのは意図の側。
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml
from scipy import sparse
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.realpath(__file__))
SIM = os.path.join(HERE, '..', 'ros_ws', 'src', 'minicar_sim')
sys.path.insert(0, os.path.join(SIM, 'scripts'))
sys.path.insert(0, os.path.join(HERE, '..', 'ros_ws', 'src', 'jetracer_common'))
from course import default_course, GIMMICK_AREAS  # noqa: E402
from jetracer_common.profile import find_profile, load_profile  # noqa: E402
from jetracer_common.reference_line import ReferenceLine  # noqa: E402

STEP = 0.06            # 点の間隔 [m] (course.py の中心線と同じ)
GRID = 0.01            # 壁距離場の格子 [m]


# ----------------------------------------------------------------------------- 幾何
def resample(pts, step=STEP, closed=True):
    p = np.asarray(pts, float)
    if closed:
        p = np.vstack([p, p[:1]])
    seg = np.diff(p, axis=0)
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
    n = int(round(cum[-1] / step))
    s = np.linspace(0.0, cum[-1], n, endpoint=False)
    return np.column_stack([np.interp(s, cum, p[:, 0]), np.interp(s, cum, p[:, 1])])


def menger_curvature(p):
    """閉ループの各点の曲率 (3 点の外接円)。左曲がりが正。"""
    a = np.roll(p, 1, axis=0)
    c = np.roll(p, -1, axis=0)
    ab = p - a
    bc = c - p
    ac = c - a
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.hypot(*ab.T) * np.hypot(*bc.T) * np.hypot(*ac.T)
    return 2.0 * cross / np.maximum(1e-9, den)


class WallField:
    """壁までの距離場 (格子・双一次補間)。最適化の中で毎回 39 本の線分に当てないため。"""

    def __init__(self, walls, grid=GRID, pad=0.5):
        w = np.asarray(walls, float)
        self.x0 = w[:, [0, 2]].min() - pad
        self.y0 = w[:, [1, 3]].min() - pad
        x1 = w[:, [0, 2]].max() + pad
        y1 = w[:, [1, 3]].max() + pad
        self.g = grid
        nx, ny = int((x1 - self.x0) / grid) + 1, int((y1 - self.y0) / grid) + 1
        gx = self.x0 + np.arange(nx) * grid
        gy = self.y0 + np.arange(ny) * grid
        X, Y = np.meshgrid(gx, gy, indexing='ij')
        d = np.full(X.shape, np.inf)
        for xa, ya, xb, yb in w:
            dx, dy = xb - xa, yb - ya
            L2 = max(1e-12, dx * dx + dy * dy)
            t = np.clip(((X - xa) * dx + (Y - ya) * dy) / L2, 0.0, 1.0)
            d = np.minimum(d, np.hypot(X - (xa + t * dx), Y - (ya + t * dy)))
        self.d = d

    def __call__(self, p):
        ix = (p[:, 0] - self.x0) / self.g
        iy = (p[:, 1] - self.y0) / self.g
        return map_coordinates(self.d, [ix, iy], order=1, mode='nearest')


# ----------------------------------------------------------------------------- 優先線 (M-05 make_route.py の意図)
def _runs(flags):
    runs, i, n = [], 0, len(flags)
    while i < n:
        if flags[i]:
            j = i
            while j + 1 < n and flags[j + 1]:
                j += 1
            runs.append((i, j))
            i = j + 1
        else:
            i += 1
    return runs


def _shift(pts, inside, target, axis, taper_m):
    """inside の区間だけ座標 axis を target へ寄せ、前後 taper_m で経路に沿って滑らかに戻す。"""
    p = np.asarray(pts, float).copy()
    n = len(p)
    seg = np.hypot(*np.diff(p, axis=0).T)
    for i0, i1 in _runs(list(inside)):
        w = np.zeros(n)
        w[i0:i1 + 1] = 1.0
        d, k = 0.0, i0
        while k > 0 and d < taper_m:
            d += seg[k - 1]
            k -= 1
            w[k] = max(w[k], 1.0 - d / taper_m)
        d, k = 0.0, i1
        while k < n - 1 and d < taper_m:
            d += seg[k]
            k += 1
            w[k] = max(w[k], 1.0 - d / taper_m)
        p[:, axis] += w * (target - p[:, axis])
    return p


def in_box(p, box, m=0.0, heading=None, tol_deg=45.0):
    """箱の中 (margin m)。heading を与えたら、その向き (rad) ±tol の点だけ。
    蛇行コースでは同じ箱を別レーンも通る (M-05 の make_route.py で実際に起きた) ので、向きで区別する。"""
    ok = ((box[0] - m <= p[:, 0]) & (p[:, 0] <= box[2] + m)
          & (box[1] - m <= p[:, 1]) & (p[:, 1] <= box[3] + m))
    if heading is not None:
        d = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
        psi = np.arctan2(d[:, 1], d[:, 0])
        dpsi = np.abs(np.arctan2(np.sin(psi - heading), np.cos(psi - heading)))
        ok &= dpsi <= math.radians(tol_deg)
    return ok


N, E, S, W = math.pi / 2, 0.0, -math.pi / 2, math.pi


def preferred_line(center, half_w):
    """コース中心線にギミックごとの意図を織り込んだ優先線。"""
    area = {n: (x0, y0, x1, y1) for n, x0, y0, x1, y1 in GIMMICK_AREAS}
    plate = area['MU_LOW']                       # ④滑り板 (低μ)
    rough = area['ROUGH']                        # ⑦でこぼこ (内側のみ)
    slope = area['SLOPE']                        # ②坂道
    outer_x = 10.28                              # 右外壁
    p = resample(center)
    # ④ 北向きに芝を通る区間: 滑り板の外縁と外壁の中央 (左右の余裕を等しく)。北向きの点だけ
    turf = (8.90, plate[1] + 0.22, outer_x, plate[3] - 0.30)
    p = _shift(p, in_box(p, turf, 0.15, N), (plate[2] + outer_x) / 2.0, 0, 1.6)
    # ④ 滑り板の南側を東西に横切る区間: 板の南端の手前 (半車幅 + 0.20)。東向きの点だけ
    cross = (plate[0] - 0.20, plate[1] - 0.40, plate[2] + 0.20, plate[1] + 0.27)
    p = _shift(p, in_box(p, cross, 0.0, E), plate[1] - half_w - 0.12, 1, 0.9)
    # ⑦ でこぼこ: 外壁 x=0 と でこぼこ x0 の中央。南向きの点だけ
    p = _shift(p, in_box(p, rough, 0.15, S), rough[0] / 2.0, 0, 1.6)
    # ② 坂道: 中央を x 一定で直角に横断 (上下 0.55 m は直進)。北向きの点だけ
    sx = (slope[0] + slope[2]) / 2.0
    p = _shift(p, in_box(p, slope, 0.55, N), sx, 0, 1.2)
    return resample(p)


# ----------------------------------------------------------------------------- 最適化
def optimize(pref, field, kappa_soft, kappa_hard, clear_min, w=None, iters=200):
    """
    優先線 pref (N×2, 閉ループ) を出発点に、残差の二乗和を最小化:
      平滑   : 2 階差分 (曲率エネルギー)
      曲率   : |κ| が kappa_soft を超えた分 (hinge)。kappa_hard 超えはさらに重く
      壁余裕 : 壁距離が clear_min を割った分 (hinge)
      優先線 : pref からの距離 (弱く。意図を残す)
      間隔   : 隣接距離が STEP から外れた分 (点が寄り集まらないように)
    """
    w = dict(dict(smooth=1.0, kappa=30.0, hard=300.0, clear=200.0, pref=0.6, space=20.0), **(w or {}))
    n = len(pref)
    x0 = pref.reshape(-1)
    idx = np.arange(n)
    prev, nxt = np.roll(idx, 1), np.roll(idx, -1)

    def residuals(x):
        p = x.reshape(n, 2)
        d2 = p[nxt] - 2.0 * p + p[prev]
        k = menger_curvature(p)
        ak = np.abs(k)
        r_k = np.maximum(0.0, ak - kappa_soft)
        r_h = np.maximum(0.0, ak - kappa_hard)
        r_c = np.maximum(0.0, clear_min + 0.02 - field(p))
        r_p = p - pref
        r_s = np.hypot(*(p[nxt] - p).T) - STEP
        return np.concatenate([
            math.sqrt(w['smooth']) * d2.reshape(-1) / STEP ** 2 * STEP,   # ≈ κ·STEP のスケール
            math.sqrt(w['kappa']) * r_k,
            math.sqrt(w['hard']) * r_h,
            math.sqrt(w['clear']) * r_c,
            math.sqrt(w['pref']) * r_p.reshape(-1),
            math.sqrt(w['space']) * r_s / STEP,
        ])

    # 疎ヤコビアン: 各残差は高々 3 点 (6 変数) にしか依存しない
    rows, cols = [], []
    r = 0

    def dep(count, deps_of_i, per=1):
        nonlocal r
        for i in range(n):
            for q in range(per):
                for j in deps_of_i(i):
                    rows.extend([r] * 2)
                    cols.extend([2 * j, 2 * j + 1])
                r += 1
    three = lambda i: (prev[i], i, nxt[i])  # noqa: E731
    dep(n, three, per=2)          # smooth (x, y)
    dep(n, three)                 # kappa
    dep(n, three)                 # hard
    dep(n, lambda i: (i,))        # clear
    dep(n, lambda i: (i,), per=2)  # pref
    dep(n, lambda i: (i, nxt[i]))  # space
    J = sparse.coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(r, 2 * n)).tocsr()

    sol = least_squares(residuals, x0, jac_sparsity=J, method='trf', max_nfev=iters,
                        x_scale=0.05, ftol=1e-6, xtol=1e-6, verbose=0)
    return sol.x.reshape(n, 2), sol


# ----------------------------------------------------------------------------- 入出力
def report(name, pts, field, r_min, half_w):
    ref = ReferenceLine(pts)                       # sim と同じ 0.3 m 窓の曲率
    k_raw = np.abs(menger_curvature(resample(pts)))
    rad = 1.0 / np.maximum(1e-6, np.abs(ref.kappa))
    clear = field(ref.pts) - half_w
    i = int(np.argmin(rad))
    j = int(np.argmin(clear))
    print(f'{name}: 全長 {ref.total:.2f} m  最小半径 {rad.min():.3f} m @ ({ref.pts[i][0]:.2f},{ref.pts[i][1]:.2f}) '
          f'[生 {1.0 / max(1e-6, k_raw.max()):.3f} m]  R_min 割れ {(rad < r_min).sum() * 0.02:.2f} m  '
          f'壁余裕 (半車幅込) 最小 {clear.min():.3f} m @ ({ref.pts[j][0]:.2f},{ref.pts[j][1]:.2f})')
    return ref


def plot(path, course, center, pref, out, r_min, field, half_w):
    import cv2
    PX = 110
    W, H = int(11.0 * PX), int(7.0 * PX)
    img = np.full((H, W, 3), 30, np.uint8)
    P = lambda x, y: (int(x * PX) + 20, H - int(y * PX) - 20)  # noqa: E731
    for n, x0, y0, x1, y1 in GIMMICK_AREAS:
        cv2.rectangle(img, P(x0, y0), P(x1, y1), (70, 70, 40), -1)
        cv2.putText(img, n, P(x0 + 0.05, y1 - 0.15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 120), 1)
    for x0, y0, x1, y1 in course.walls:
        cv2.line(img, P(x0, y0), P(x1, y1), (230, 230, 230), 2)
    for pts, col in ((center, (0, 140, 0)), (pref, (0, 120, 255)), (out, (255, 255, 0))):
        q = np.vstack([pts, pts[:1]])
        for a, b in zip(q[:-1], q[1:]):
            cv2.line(img, P(*a), P(*b), col, 1 if col != (255, 255, 0) else 2)
    ref = ReferenceLine(out)
    rad = 1.0 / np.maximum(1e-6, np.abs(ref.kappa))
    for (x, y), rr in zip(ref.pts, rad):
        if rr < r_min * 1.15:
            cv2.circle(img, P(x, y), 3, (0, 0, 255) if rr < r_min else (0, 200, 255), -1)
    clear = field(ref.pts) - half_w
    for (x, y), c in zip(ref.pts, clear):
        if c < 0.12:
            cv2.circle(img, P(x, y), 3, (255, 0, 255), -1)
    cv2.putText(img, f'green=centerline  orange=preferred  cyan=route (R_min {r_min:.3f} m)  '
                     f'red=R<R_min  yellow=R<1.15 R_min  magenta=clear<0.12',
                (20, H - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.imwrite(path, img)
    print('plot', path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='jetracer_tt02')
    ap.add_argument('--delta-max', type=float, default=0.0, help='δmax [rad] を上書き (実測値)')
    ap.add_argument('--margin', type=float, default=1.20, help='目標半径 = R_min × margin')
    ap.add_argument('--clear', type=float, default=0.16, help='壁余裕の下限 (半車幅を引いた値) [m]')
    ap.add_argument('--out', default='')
    ap.add_argument('--plot', default='')
    ap.add_argument('--no-pref', action='store_true', help='ギミックの意図を織り込まず中心線から')
    a = ap.parse_args()

    prof = load_profile(find_profile(a.profile))
    L = float(prof['wheelbase_m'])
    dmax = a.delta_max if a.delta_max > 0 else float(prof['delta_max_rad'])
    r_min = L / math.tan(dmax)
    half_w = float(prof.get('width_m', 0.19)) / 2.0
    print(f'{prof["name"]}: WB {L:.3f} m, δmax {math.degrees(dmax):.1f}° → R_min {r_min:.3f} m, '
          f'目標 {r_min * a.margin:.3f} m, 壁余裕 ≥ {a.clear:.2f} m (+半車幅 {half_w:.3f})')

    course = default_course()
    field = WallField(course.walls)
    center = resample(course.center[:-1])
    pref = center if a.no_pref else preferred_line(course.center[:-1], half_w)
    report('中心線', center, field, r_min, half_w)
    report('優先線', pref, field, r_min, half_w)

    out, sol = optimize(pref, field, kappa_soft=1.0 / (r_min * a.margin), kappa_hard=1.0 / r_min,
                        clear_min=half_w + a.clear)
    print(f'最適化: {sol.nfev} 評価, cost {sol.cost:.4f}, {sol.message}')
    out = resample(out)
    ref = report('結果  ', out, field, r_min, half_w)

    path = a.out or os.path.join(SIM, 'config', f'route_{a.profile}.yaml')
    data = {'shortcut': {'waypoints': [[round(float(x), 3), round(float(y), 3)] for x, y in out]}}
    with open(path, 'w') as f:
        f.write(f'# tools/make_route.py が生成 (profile {a.profile}, δmax {math.degrees(dmax):.1f}°, '
                f'R_min {r_min:.3f} m × {a.margin}, 壁余裕 ≥ {a.clear} m)。手で編集しない。\n'
                f'# 全長 {ref.total:.2f} m。sim_host.launch.py route_file:=<このファイル> で使う。\n')
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
    print('wrote', os.path.normpath(path), f'({len(out)} 点)')
    if a.plot:
        plot(a.plot, course, center, pref, out, r_min, field, half_w)


if __name__ == '__main__':
    main()
