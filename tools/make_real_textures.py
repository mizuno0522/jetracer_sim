#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
実カメラ画像 (2026-09-12 手動走行ログ) から Unity 用のテクスチャを作る。
幾何は Unity のまま、見た目 (色・粒・会場の背景) を実画像から取るためのもの。

  python3 tools/make_real_textures.py <画像ディレクトリ (images*/ を含む)> -o <StreamingAssets/textures>

出力:
  carpet.png    床のパンチカーペット。近景の床 (画面下・柱を避けた帯) から切り出し、鏡像で継ぎ目を消した
                256×256 のタイル。Unity の Carpet 材質に使う (kCarpet の色はこのタイルの平均に置き換わる)
  backdrop.png  壁の向こう (会場) の帯。各フレームの上部 (地平線より上) を横に並べた 4096×256 の帯。
                コースを囲む円筒に貼る (幾何的には正しくないが、CNN が見る「壁の上の雑然とした明るさ」の分布を再現する)
  stats.json    実画像の統計 (床・白壁・赤帯の平均色、周辺減光、柱の位置)。course.json の realism の初期値の根拠
"""

import argparse
import glob
import json
import os
import random

import cv2
import numpy as np


def load(files):
    return [cv2.imread(f) for f in files]


def carpet_tile(imgs, size=256):
    # 近景の床: 行 150〜200、列 70〜110 と 135〜175 (柱 54-57 / 121-125 を避ける)。
    # 滑り板 (青)・人工芝 (緑)・壁が入ったパッチは色で捨て、灰色のカーペットだけ使う。
    # 透視で縦に潰れているので縦だけ 2 倍に伸ばし、パッチを並べて鏡像タイルにする
    patches = []
    for im in imgs:
        for x0 in (70, 135):
            p = im[150:200, x0:x0 + 40].astype(np.float32)
            m = p.reshape(-1, 3).mean(0)
            if not (80 < m.mean() < 150) or (m.max() - m.min()) > 12 or p.std() > 22:
                continue
            p = cv2.resize(p, (40, 100), interpolation=cv2.INTER_LINEAR)
            # 露出差と周辺減光の傾き (パッチ内の低周波) を抜き、粒だけ残して平均 115 に揃える
            low = cv2.GaussianBlur(p, (0, 0), 12.0)
            patches.append(p - low + 115.0)
    if len(patches) < 21:
        raise SystemExit(f'カーペットのパッチが足りない ({len(patches)})')
    random.shuffle(patches)
    grid = []
    for r in range(3):
        row = np.concatenate([patches[r * 7 + c] for c in range(7)], axis=1)   # 280 × 100
        grid.append(row)
    tile = np.concatenate(grid, axis=0)                     # 280 × 300
    tile = cv2.GaussianBlur(tile, (0, 0), 1.2)               # パッチの継ぎ目をならす
    tile = cv2.resize(tile, (size // 2, size // 2), interpolation=cv2.INTER_AREA)
    top = np.concatenate([tile, tile[:, ::-1]], axis=1)      # 鏡像で継ぎ目を消す
    tile = np.concatenate([top, top[::-1, :]], axis=0)
    return np.clip(tile, 0, 255).astype(np.uint8)


def backdrop_strip(imgs, n=24, h=256, w_each=170):
    # 各フレームの上部 (行 0〜72 = 壁の上端より上) を切り出し、横に並べる。左右反転を混ぜて継ぎ目の目立ちを減らす
    picks = random.sample(imgs, n)
    cols = []
    for i, im in enumerate(picks):
        band = im[0:72, 16:208]              # 地平線 (行 ~95) より上: 壁の向こうの会場だけ
        band = cv2.resize(band, (w_each, h), interpolation=cv2.INTER_LINEAR)
        if i % 2:
            band = band[:, ::-1]
        cols.append(band)
    strip = np.concatenate(cols, axis=1)
    # 継ぎ目を横方向にぼかす
    strip = cv2.GaussianBlur(strip, (0, 0), sigmaX=2.0, sigmaY=0.5)
    # 露出後の実画像の上部 (平均 ~118) に合わせ少し持ち上げる
    return np.clip(strip.astype(np.float32) * 1.12, 0, 255).astype(np.uint8)


def stats(imgs):
    a = np.stack(imgs).astype(np.float32)
    gray = a.mean(axis=3)
    floor = a[:, 150:195, :, :].reshape(-1, 3)
    mid = a[:, 90:130, :, :].reshape(-1, 3)
    bright = mid[mid.mean(1) > 150]
    red = mid[(mid[:, 2] > 120) & (mid[:, 1] < 80)]
    prof = gray[:, 212:224, :].mean(axis=(0, 1))
    dark = np.where(prof < prof.mean() - 25)[0]
    runs = []
    if len(dark):
        s = p = int(dark[0])
        for c in dark[1:]:
            if c != p + 1:
                runs.append((s, p))
                s = int(c)
            p = int(c)
        runs.append((s, p))
    c = gray[:, 160:190, 100:124].mean()
    e = (gray[:, 160:190, 0:24].mean() + gray[:, 160:190, 200:224].mean()) / 2
    return dict(
        n=len(imgs),
        mean_bgr=[float(v) for v in a.reshape(-1, 3).mean(0)],
        gray_p5_50_95=[float(v) for v in np.percentile(gray, [5, 50, 95])],
        floor_bgr=[float(v) for v in floor.mean(0)],
        floor_std=[float(v) for v in floor.std(0)],
        wall_white_bgr=[float(v) for v in bright.mean(0)] if len(bright) else None,
        wall_red_bgr=[float(v) for v in red.mean(0)] if len(red) else None,
        vignette_center_over_edge=float(c / e),
        posts_cols=runs,
        row_profile=[float(v) for v in gray.mean(axis=(0, 2))[::8]],
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('-n', type=int, default=300, help='統計・テクスチャに使うサンプル枚数')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    random.seed(a.seed)
    files = sorted(glob.glob(os.path.join(a.src, 'images*', '*.jpg')))
    if not files:
        raise SystemExit(f'画像が無い: {a.src}/images*/*.jpg')
    sample = random.sample(files, min(a.n, len(files)))
    imgs = load(sample)
    os.makedirs(a.out, exist_ok=True)
    cv2.imwrite(os.path.join(a.out, 'carpet.png'), carpet_tile(imgs))
    cv2.imwrite(os.path.join(a.out, 'backdrop.png'), backdrop_strip(imgs))
    st = stats(imgs)
    with open(os.path.join(a.out, 'stats.json'), 'w') as f:
        json.dump(st, f, indent=1)
    print(f"wrote carpet.png / backdrop.png / stats.json → {a.out}")
    print(f"floor BGR {np.round(st['floor_bgr'],1)}  wall white {np.round(st['wall_white_bgr'],1)}  "
          f"red {np.round(st['wall_red_bgr'],1)}  vignette {st['vignette_center_over_edge']:.3f}  posts {st['posts_cols']}")


if __name__ == '__main__':
    main()
