# -*- coding: utf-8 -*-
"""
sim→real 画像変換 (CUT) の共通部品。ROS に依存しない (学習は Jetson やクラウドでも回せる)。

- 画像は 224×224 の BGR uint8 (ROS の /camera/image_raw と同じ並び) を基本にする
- 画面下端の車体の柱 (vehicle_profile.camera.realism.posts) は変換器に学ばせない:
  学習・推論の入力で両ドメインとも同じ定数色で塗り、変換後は入力 (Unity が描いた柱) の画素で上書きする。
  柱の位置は実機 camera_preproc のマスクと同値にすること (docs/unity.md)。
"""

import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
# 柱の位置を読む vehicle_profile。自分の車の profile を使うときは環境変数 SIM2REAL_PROFILE にパスか名前を入れる
_PDIR = os.path.join(REPO, 'ros_ws', 'src', 'minicar_sim', 'config', 'vehicle_profile')
_P = os.environ.get('SIM2REAL_PROFILE', 'jetracer_tt02')
PROFILE = _P if os.path.isfile(_P) else os.path.join(_PDIR, _P if _P.endswith('.yaml') else _P + '.yaml')
POST_FILL = 48          # 柱を塗る色 (実画像の柱 ≈ 40〜60)


def load_posts(profile=PROFILE):
    """[(x0, x1, y0), ...] (正規化座標)。profile に無ければ空 (柱の無いカメラは realism.posts: [] にする)。"""
    with open(profile) as f:
        vp = yaml.safe_load(f)['vehicle_profile']
    p = vp.get('camera', {}).get('realism', {}).get('posts', []) or []
    return [tuple(p[i:i + 3]) for i in range(0, len(p) - 2, 3)]


def post_mask(h, w, posts, pad_px=1):
    """柱の領域の bool マスク (h, w)。"""
    m = np.zeros((h, w), bool)
    for x0, x1, y0 in posts:
        c0 = max(0, int(np.floor(x0 * w)) - pad_px)
        c1 = min(w, int(np.ceil(x1 * w)) + pad_px)
        r0 = max(0, int(np.floor(y0 * h)) - pad_px)
        m[r0:, c0:c1] = True
    return m


def paint_posts(img, mask, value=POST_FILL):
    out = img.copy()
    out[mask] = value
    return out


def to_tensor_np(bgr):
    """BGR uint8 (H,W,3) → float32 (3,H,W) in [-1,1] (RGB 順)。"""
    rgb = bgr[:, :, ::-1].astype(np.float32)
    return (rgb / 127.5 - 1.0).transpose(2, 0, 1).copy()


def from_tensor_np(chw):
    """float (3,H,W) in [-1,1] (RGB) → BGR uint8 (H,W,3)。"""
    rgb = np.clip((chw.transpose(1, 2, 0) + 1.0) * 127.5, 0, 255).astype(np.uint8)
    return rgb[:, :, ::-1].copy()


# ---------------------------------------------------------------------------
# 幾何の保存の検証: 変換で壁・走路の縁が動いていないか (エッジの一致)
def _gray(bgr):
    return bgr.astype(np.float32) @ np.array([0.114, 0.587, 0.299], np.float32)


def edges(bgr, thresh=None):
    """Sobel の勾配の強い画素 (上位 8 %)。"""
    g = _gray(bgr)
    gx = np.zeros_like(g)
    gy = np.zeros_like(g)
    gx[:, 1:-1] = g[:, 2:] - g[:, :-2]
    gy[1:-1, :] = g[2:, :] - g[:-2, :]
    mag = np.hypot(gx, gy)
    t = np.percentile(mag, 92) if thresh is None else thresh
    return mag > max(t, 8.0)


def _dilate(m, r):
    out = m.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dy == 0 and dx == 0:
                continue
            sh = np.zeros_like(m)
            ys = slice(max(0, dy), m.shape[0] + min(0, dy))
            yd = slice(max(0, -dy), m.shape[0] + min(0, -dy))
            xs = slice(max(0, dx), m.shape[1] + min(0, dx))
            xd = slice(max(0, -dx), m.shape[1] + min(0, -dx))
            sh[yd, xd] = m[ys, xs]
            out |= sh
    return out


def edge_fscore(a_bgr, b_bgr, tol_px=2, region=None):
    """a (変換前) と b (変換後) のエッジの F 値 (許容 tol_px)。region: bool マスクで評価範囲を絞る。"""
    ea, eb = edges(a_bgr), edges(b_bgr)
    if region is not None:
        ea &= region
        eb &= region
    if ea.sum() == 0 or eb.sum() == 0:
        return float('nan')
    prec = (eb & _dilate(ea, tol_px)).sum() / eb.sum()
    rec = (ea & _dilate(eb, tol_px)).sum() / ea.sum()
    return float(2 * prec * rec / max(1e-9, prec + rec))
