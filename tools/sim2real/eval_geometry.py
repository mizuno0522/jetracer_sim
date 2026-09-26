#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
変換で幾何 (壁・走路の縁) が動いていないかを数で確かめる。変換前後のエッジの一致 (F 値) を画像ごとに出す。
ラベル (/sim/ground_truth の先行注視点 u, v) は変換前の幾何で付いているので、ここが崩れると学習が壊れる。

  ~/jetracer/venv_sim2real/bin/python tools/sim2real/eval_geometry.py --model runs/cut_001/G.onnx --sim ~/jetracer/data/sim_frames [-n 200] [--grid out.png]
合格の目安: 全体 F 値 (許容 2 px) の中央値 ≥ 0.6、床より下半分 ≥ 0.6 (実画像の質感で床のエッジが増える分は許す)。
"""
import argparse
import os
import random
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import edge_fscore, load_posts, post_mask  # noqa: E402
from translator import Translator  # noqa: E402
from train_cut import list_images  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--sim', required=True)
    ap.add_argument('-n', type=int, default=200)
    ap.add_argument('--grid', default='')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    random.seed(a.seed)
    files = list_images(a.sim)
    files = random.sample(files, min(a.n, len(files)))
    tr = Translator(os.path.expanduser(a.model))
    f_all, f_low, pairs = [], [], []
    for f in files:
        bgr = np.asarray(Image.open(f).convert('RGB'))[:, :, ::-1].copy()
        out = tr(bgr)
        h, w = bgr.shape[:2]
        keep = ~post_mask(h, w, load_posts(), pad_px=3)
        low = keep.copy()
        low[: h // 2] = False
        f_all.append(edge_fscore(bgr, out, 2, keep))
        f_low.append(edge_fscore(bgr, out, 2, low))
        if len(pairs) < 6:
            pairs.append(np.concatenate([bgr, out], 0))
    fa, fl = np.nanmedian(f_all), np.nanmedian(f_low)
    print(f'{len(files)} 枚: エッジ F 値 (2 px) 中央値 全体 {fa:.3f} / 下半分 {fl:.3f}, 全体の p10 {np.nanpercentile(f_all, 10):.3f}')
    print('判定:', 'OK' if fa >= 0.6 and fl >= 0.6 else 'NG (幾何が動いている)')
    if a.grid and pairs:
        Image.fromarray(np.concatenate(pairs, 1)[:, :, ::-1]).save(a.grid)
        print('grid', a.grid)


if __name__ == '__main__':
    main()
