#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
実カメラ画像と sim の /camera/image_raw の統計を並べて比べる (見た目合わせの当たり判定)。
  python3 tools/compare_images.py <実画像ディレクトリ (images*/ を含む)> <sim 画像ディレクトリ (*.png)>
"""

import glob
import os
import random
import sys

import cv2
import numpy as np


def stats(files):
    a = np.stack([cv2.imread(f) for f in files]).astype(np.float32)
    gray = a.mean(axis=3)
    floor = a[:, 150:195, 30:50, :].reshape(-1, 3)      # 柱を避けた近景の床 (左)
    floor2 = a[:, 150:195, 70:110, :].reshape(-1, 3)
    prof = gray[:, 212:224, :].mean(axis=(0, 1))
    c = gray[:, 160:190, 100:124].mean()
    e = (gray[:, 160:190, 0:24].mean() + gray[:, 160:190, 200:224].mean()) / 2
    lap = np.median([cv2.Laplacian(cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY), cv2.CV_64F).var() for f in files[:80]])
    return dict(
        n=len(files),
        mean=gray.mean(), p5=np.percentile(gray, 5), p50=np.percentile(gray, 50), p95=np.percentile(gray, 95),
        floor=np.concatenate([floor, floor2]).mean(0), floor_std=np.concatenate([floor, floor2]).std(0).mean(),
        top=gray[:, 0:60, :].mean(), band=gray[:, 60:100, :].mean(), bottom=gray[:, 190:224, :].mean(),
        vignette=c / e, posts=prof[54:58].mean() - prof[70:110].mean(), sharp=lap,
        rows=gray.mean(axis=(0, 2))[::16],
    )


def main():
    random.seed(0)
    real = sorted(glob.glob(os.path.join(sys.argv[1], 'images*', '*.jpg')))
    sim = sorted(glob.glob(os.path.join(sys.argv[2], '*.png')))
    real = random.sample(real, min(300, len(real)))
    r, s = stats(real), stats(sim)
    print(f"{'':14s} {'real':>22s} {'sim':>22s}")
    for k in ('n', 'mean', 'p5', 'p50', 'p95', 'top', 'band', 'bottom', 'floor_std', 'vignette', 'posts', 'sharp'):
        rv, sv = r[k], s[k]
        print(f"{k:14s} {rv:22.1f} {sv:22.1f}")
    print(f"{'floor BGR':14s} {np.round(r['floor'], 0)!s:>22s} {np.round(s['floor'], 0)!s:>22s}")
    print("row profile (every 16 rows):")
    print("  real", ' '.join(f'{v:5.0f}' for v in r['rows']))
    print("  sim ", ' '.join(f'{v:5.0f}' for v in s['rows']))


if __name__ == '__main__':
    main()
