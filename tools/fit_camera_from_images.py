#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
実画像 (走行ログ) からカメラの幾何を推定する。チェッカーボードが無いときの第一近似。

  python3 tools/fit_camera_from_images.py <ログのディレクトリ> [--frames 8] [--plot out.jpg]
  例: python3 tools/fit_camera_from_images.py /tmp/260912_MEC_PS5 --plot /tmp/fit.jpg

考え方: コースの壁 (course.py の配置、板の下端 wall_base_m・上端 wall_base_m + wall_height_m) は寸法が分かっている。
実画像の赤白の板の領域と、「ある車の位置 (x, y, yaw) とカメラ (高さ・ピッチ・fx・fy・k1)」で sim が描く壁の領域が
一致するように、**複数フレームの車の位置とカメラの共通パラメータを同時に**合わせる。
  - 1 枚だけだと焦点距離とピッチが分離できない (床の平行線は f に依らない)。壁の高さと通路幅と板の長さが
    既知で、フレームごとに見え方が違うので、複数枚で分離できる
  - fy/fx (縦横比) も推定する。ロガーが 4:3 や 16:9 の取り込みを 224×224 に縮めていれば 1.33 / 1.78 になる
    (= 切り出しか縮小かが分かる)
  - 画像下端の車体の柱 (行 205 以降) は使わない。細い白テープ (スタートライン等) は形で除く
出力は vehicle_profile.camera に貼れる yaml と、実画像に推定した壁を重ねた確認画像。
**精度はチェッカーボード較正 (tools/camera_calib.py) に劣る**。実機で較正できたらそちらを正とする。
"""
import argparse
import csv
import glob
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.realpath(__file__))
SIM = os.path.join(HERE, '..', 'ros_ws', 'src', 'minicar_sim')
sys.path.insert(0, os.path.join(SIM, 'scripts'))
sys.path.insert(0, os.path.join(HERE, '..', 'ros_ws', 'src', 'jetracer_common'))
from course import default_course  # noqa: E402
from jetracer_common.cam_geom import CamGeom  # noqa: E402

W = H = 224
ROW_MAX = 205            # これより下は車体 (柱) なので使わない
SS = 2                   # 描画の超解像 (被覆率を滑らかにして最適化しやすくする)
COLOR = True             # 赤い板と白い板を別チャンネルで比べる (継ぎ目の位置が奥行きの目盛りになる)


# ----------------------------------------------------------------------------- 実画像の壁
def _clean(m):
    m = m.astype(np.uint8)
    m[ROW_MAX:] = 0
    # 細い横線 (床の白テープ) を消す: 縦 7 px の開演算。板は縦に厚い
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 1), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((1, 3), np.uint8))
    return m.astype(np.float32)


def wall_masks_real(img):
    """(赤の板, 白の板) の 2 チャンネル。"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = (hsv[..., i].astype(np.int32) for i in range(3))
    red = ((h <= 10) | (h >= 165)) & (s >= 90) & (v >= 70)
    white = (s <= 45) & (v >= 175)
    return np.stack([_clean(red), _clean(white)])


def wall_mask_real(img):
    return np.clip(wall_masks_real(img).sum(axis=0), 0, 1)


# ----------------------------------------------------------------------------- sim の壁
class WallRenderer:
    def __init__(self, walls, z0, z1, piece=0.08, colors=None):
        p0, p1, col = [], [], []
        colors = colors or ['white'] * len(walls)
        for (x0, y0, x1, y1), cname in zip(walls, colors):
            L = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(math.ceil(L / piece)))
            t = np.linspace(0.0, 1.0, n + 1)
            xs, ys = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            p0.append(np.stack([xs[:-1], ys[:-1]], 1))
            p1.append(np.stack([xs[1:], ys[1:]], 1))
            col.append(np.full(n, 0 if cname == 'red' else 1))
        p0, p1 = np.concatenate(p0), np.concatenate(p1)
        self.col = np.concatenate(col)
        n = len(p0)
        c = np.zeros((n, 4, 3))
        c[:, 0, :2], c[:, 0, 2] = p0, z0
        c[:, 1, :2], c[:, 1, 2] = p1, z0
        c[:, 2, :2], c[:, 2, 2] = p1, z1
        c[:, 3, :2], c[:, 3, 2] = p0, z1
        self.corners = c
        self.mid = 0.5 * (p0 + p1)

    def render(self, pose, cam, scale=SS, color=False):
        """pose = (x, y, yaw)、cam = dict(h, pitch_deg, fx, fy, k1, k2, mount_x) → 被覆率 (H, W) float。
        color=True なら (赤, 白) の 2 チャンネル (2, H, W)。"""
        x, y, yaw = pose
        phi = math.radians(cam['pitch_deg'])
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        fwd = np.array([cy_ * math.cos(phi), sy_ * math.cos(phi), -math.sin(phi)])
        right = np.array([sy_, -cy_, 0.0])
        down = np.cross(fwd, right)
        C = np.array([x + cam['mount_x'] * cy_, y + cam['mount_x'] * sy_, cam['h']])
        near = np.hypot(self.mid[:, 0] - C[0], self.mid[:, 1] - C[1]) < 7.0
        v = self.corners[near] - C
        cl = self.col[near]
        zc = v @ fwd
        ok = (zc > 0.03).all(axis=1)
        v, zc, cl = v[ok], zc[ok], cl[ok]
        xn = (v @ right) / zc
        yn = (v @ down) / zc
        k1, k2 = cam['k1'], cam.get('k2', 0.0)
        r2 = xn * xn + yn * yn
        if k1 < 0 or k2 < 0:
            # 樽型の順写像が単調な範囲 (1 + 3k1 r² + 5k2 r⁴ > 0) を超える角は折り返すので捨てる
            ok2 = (1.0 + 3.0 * k1 * r2 + 5.0 * k2 * r2 * r2 > 0.05).all(axis=1)
            xn, yn, r2, cl = xn[ok2], yn[ok2], r2[ok2], cl[ok2]
        g = 1.0 + k1 * r2 + k2 * r2 * r2
        u = (W / 2.0 + cam['fx'] * xn * g) * scale
        w = (H / 2.0 + cam['fy'] * yn * g) * scale
        chans = [np.zeros((H * scale, W * scale), np.uint8) for _ in range(2 if color else 1)]
        if len(u):
            pts = np.stack([u, w], axis=2)
            inside = ((pts[..., 0] > -4 * W * scale) & (pts[..., 0] < 5 * W * scale)
                      & (pts[..., 1] > -4 * H * scale) & (pts[..., 1] < 5 * H * scale)).all(axis=1)
            pts = np.round(pts[inside] * 4).astype(np.int32)       # 4 倍の固定小数点 (shift=2)
            cl = cl[inside]
            # 近い板が遠い板を隠す: 遠い順に塗り、色チャンネルでは上書き (他方のチャンネルを消す)
            if len(pts):
                if color:
                    depth = zc[ok2].mean(axis=1)[inside] if (k1 < 0 or k2 < 0) else zc.mean(axis=1)[inside]
                    for idx in np.argsort(-depth):
                        c = int(cl[idx])
                        cv2.fillPoly(chans[c], [pts[idx]], 1, lineType=cv2.LINE_8, shift=2)
                        cv2.fillPoly(chans[1 - c], [pts[idx]], 0, lineType=cv2.LINE_8, shift=2)
                else:
                    cv2.fillPoly(chans[0], list(pts), 1, lineType=cv2.LINE_8, shift=2)
        out = []
        for m in chans:
            m = cv2.resize(m.astype(np.float32), (W, H), interpolation=cv2.INTER_AREA) if scale != 1 else m.astype(np.float32)
            m[ROW_MAX:] = 0
            out.append(m)
        return np.stack(out) if color else out[0]


def soft_iou(a, b):
    inter = np.minimum(a, b).sum()
    union = np.maximum(a, b).sum()
    return inter / max(1e-6, union)


# ----------------------------------------------------------------------------- 最適化
def cam_from_vec(q, aspect_free=True, k1_free=True):
    h, pitch, hfov, aspect, k1 = q
    fx = (W / 2.0) / math.tan(math.radians(hfov) / 2.0)
    return dict(h=h, pitch_deg=pitch, fx=fx, fy=fx * aspect, k1=k1, k2=0.0, mount_x=0.0)


def grid_pose(rend, real, center, cam, lat=(-0.15, 0.0, 0.15), dyaw=(-24, -12, 0, 12, 24), step=3):
    """中心線に沿って粗く当てる (初期値)。"""
    best = (-1.0, None)
    d = np.roll(center, -1, axis=0) - center
    psi = np.arctan2(d[:, 1], d[:, 0])
    for i in range(0, len(center), step):
        nx, ny = -math.sin(psi[i]), math.cos(psi[i])
        for l in lat:
            for dy in dyaw:
                pose = (center[i, 0] + l * nx, center[i, 1] + l * ny, psi[i] + math.radians(dy))
                s = soft_iou(rend.render(pose, cam, scale=1, color=COLOR), real)
                if s > best[0]:
                    best = (s, pose)
    return best


def pick_frames(logdir, n, rng):
    """壁が十分に写っているフレームを選ぶ (壁の面積 6〜35 %)。セッションをまたいで散らす。"""
    imgs = sorted(glob.glob(os.path.join(logdir, 'images*', '*.jpg')))
    rng.shuffle(imgs)
    out = []
    for p in imgs:
        im = cv2.imread(p)
        if im is None or im.shape[:2] != (H, W):
            continue
        m2 = wall_masks_real(im)
        frac = np.clip(m2.sum(axis=0), 0, 1)[:ROW_MAX].mean()
        # 赤い板が写っているフレームを優先 (白だけだと会場の白い物と紛れる)
        if 0.06 <= frac <= 0.35 and m2[0][:ROW_MAX].mean() >= 0.015:
            out.append((p, im, m2 if COLOR else np.clip(m2.sum(axis=0), 0, 1)))
        if len(out) >= n:
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logdir')
    ap.add_argument('--frames', type=int, default=8)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--plot', default='')
    ap.add_argument('--aspects', default='1.0,1.333,1.778', help='試す fy/fx の初期値')
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    simcfg = yaml.safe_load(open(os.path.join(SIM, 'config', 'sim.yaml')))['vehicle_sim']['ros__parameters']
    z0 = float(simcfg.get('wall_base_m', 0.03))
    z1 = z0 + float(simcfg.get('wall_height_m', 0.089))
    course = default_course()
    from course import WALL_COLORS
    colors = list(WALL_COLORS) if len(WALL_COLORS) == len(course.walls) else None
    rend = WallRenderer(course.walls, z0, z1, colors=colors)
    center = course.center[:-1]
    print(f'壁 {len(course.walls)} 枚 (板 z {z0:.3f}〜{z1:.3f} m)、中心線 {len(center)} 点')

    frames = pick_frames(a.logdir, a.frames, rng)
    print(f'フレーム {len(frames)} 枚: ' + ', '.join(os.path.basename(p)[:8] for p, _, _ in frames))
    blur = lambda m: np.stack([cv2.GaussianBlur(c, (0, 0), 0.8) for c in m]) if m.ndim == 3 else cv2.GaussianBlur(m, (0, 0), 0.8)
    reals = [blur(m) for _, _, m in frames]

    # --- 1. 粗い探索: カメラ候補 × 各フレームの位置 -----------------------------
    t0 = time.time()
    cands = []
    for aspect in [float(s) for s in a.aspects.split(',')]:
        for hfov in (90.0, 120.0, 150.0):
            for pitch in (25.0, 40.0, 55.0):
                cam = cam_from_vec((0.10, pitch, hfov, aspect, 0.0))
                res = [grid_pose(rend, r, center, cam) for r in reals[:3]]
                score = float(np.mean([s for s, _ in res]))
                cands.append((score, (0.10, pitch, hfov, aspect, 0.0)))
    cands.sort(key=lambda c: -c[0])
    print(f'粗い探索 {time.time() - t0:.0f} s。上位:')
    for s, q in cands[:5]:
        print(f'  IoU {s:.3f}  h {q[0]:.2f} pitch {q[1]:.0f}° hfov {q[2]:.0f}° fy/fx {q[3]:.2f}')

    # --- 2. 上位の候補から、全フレームの位置とカメラを同時に詰める --------------------
    best = None
    for s0, q0 in cands[:3]:
        cam = cam_from_vec(q0)
        poses = [grid_pose(rend, r, center, cam)[1] for r in reals]
        x0 = np.concatenate([np.array(q0, float)] + [np.array(p, float) for p in poses])
        nf = len(reals)

        def f(xv):
            q = xv[:5]
            if not (0.04 < q[0] < 0.25 and 0 < q[1] < 80 and 40 < q[2] < 170 and 0.7 < q[3] < 2.2 and -0.3 < q[4] <= 0.02):
                return 2.0
            cam_ = cam_from_vec(q)
            return 1.0 - np.mean([soft_iou(rend.render(xv[5 + 3 * i:8 + 3 * i], cam_, color=COLOR), reals[i]) for i in range(nf)])

        t1 = time.time()
        r = minimize(f, x0, method='Powell', options=dict(maxiter=6000, xtol=1e-3, ftol=1e-4))
        print(f'  候補 (pitch {q0[1]:.0f} hfov {q0[2]:.0f} fy/fx {q0[3]:.2f}) → IoU {1 - r.fun:.3f} '
              f'({r.nfev} 評価, {time.time() - t1:.0f} s)')
        if best is None or r.fun < best.fun:
            best = r
    q = best.x[:5]
    cam = cam_from_vec(q)
    iou = 1.0 - best.fun
    g = CamGeom(W, H, q[2], q[0], q[1], vfov_deg=math.degrees(2 * math.atan((H / 2.0) / cam['fy'])), k1=q[4])
    th, tv = g.true_fov_deg()
    print('\n=== 推定結果 (実画像 %d 枚、平均 IoU %.3f) ===' % (len(reals), iou))
    print(f'  取付高さ {q[0]:.3f} m、ピッチ (下向き) {q[1]:.1f}°')
    print(f'  fx {cam["fx"]:.1f} px (hfov {q[2]:.1f}°)、fy {cam["fy"]:.1f} px (fy/fx {q[3]:.2f})、k1 {q[4]:+.3f}')
    print(f'  歪み込みの見込み角: 水平 {th:.0f}°・垂直 {tv:.0f}°。地平線 v = {g.horizon_row():.0f} 行')
    print('  fy/fx の読み: 1.0 付近 = 正方に切り出し、1.33 = 4:3 を縮小、1.78 = 16:9 を縮小')
    print('\n# vehicle_profile.camera に貼る (★画像からの推定。チェッカーボード較正で置き換える)')
    print(yaml.safe_dump({'hfov_deg': round(float(q[2]), 1), 'vfov_deg': round(float(g_vfov(cam)), 1),
                          'mount_height_m': round(float(q[0]), 3), 'pitch_deg': round(float(q[1]), 1),
                          'k1': round(float(q[4]), 4), 'k2': 0.0}, sort_keys=False).rstrip())
    if a.plot:
        tiles = []
        for i, (p, im, _) in enumerate(frames):
            pose = best.x[5 + 3 * i:8 + 3 * i]
            m = rend.render(pose, cam, color=COLOR)
            vis = im.copy()
            ms = m if m.ndim == 3 else m[None]
            rs = reals[i] if reals[i].ndim == 3 else reals[i][None]
            for ch, col in zip(range(len(ms)), [(0, 255, 255), (255, 255, 0)]):
                edge = cv2.Canny((ms[ch] > 0.5).astype(np.uint8) * 255, 50, 150) > 0
                vis[edge] = col
            real_e = cv2.Canny((np.clip(rs.sum(axis=0), 0, 1) > 0.5).astype(np.uint8) * 255, 50, 150) > 0
            vis[real_e & (vis.sum(axis=2) < 700)] = (255, 0, 255)
            vis = cv2.resize(vis, (336, 336), interpolation=cv2.INTER_NEAREST)
            cv2.putText(vis, f'IoU {soft_iou(m, reals[i]):.2f}', (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            tiles.append(vis)
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
        cv2.imwrite(a.plot, np.vstack(rows))
        print('plot', a.plot, '(黄 = 推定で描いた赤い板、水色 = 白い板、紫 = 実画像の壁の輪郭)')


def g_vfov(cam):
    return math.degrees(2 * math.atan((H / 2.0) / cam['fy']))


if __name__ == '__main__':
    main()
