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


def color_classes(bgr):
    """意味のある色の画素。blue = 滑り板の水色、white = 床のテープ、dark = 暗幕・トンネル、red = 壁の赤。"""
    b, g, r = [bgr[..., i].astype(int) for i in range(3)]
    lum = (r + g + b) / 3
    sat = bgr.max(-1).astype(int) - bgr.min(-1)
    return {'blue': b > r + 40, 'white': (lum > 190) & (sat < 40), 'dark': lum < 55,
            'red': (r > g + 40) & (r > b + 40)}


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
    kept = {k: [0, 0] for k in ('blue', 'white', 'dark')}
    red_add = [0, 0]
    for f in files:
        bgr = np.asarray(Image.open(f).convert('RGB'))[:, :, ::-1].copy()
        out = tr(bgr)
        h, w = bgr.shape[:2]
        keep = ~post_mask(h, w, load_posts(), pad_px=3)
        low = keep.copy()
        low[: h // 2] = False
        f_all.append(edge_fscore(bgr, out, 2, keep))
        f_low.append(edge_fscore(bgr, out, 2, low))
        ci, co = color_classes(bgr), color_classes(out)
        for k in kept:
            s = ci[k] & keep
            kept[k][0] += int((s & co[k]).sum())
            kept[k][1] += int(s.sum())
        s = ~ci['red'] & keep
        red_add[0] += int((s & co['red']).sum())
        red_add[1] += int(s.sum())
        if len(pairs) < 6:
            pairs.append(np.concatenate([bgr, out], 0))
    fa, fl = np.nanmedian(f_all), np.nanmedian(f_low)
    print(f'{len(files)} 枚: エッジ F 値 (2 px) 中央値 全体 {fa:.3f} / 下半分 {fl:.3f}, 全体の p10 {np.nanpercentile(f_all, 10):.3f}')
    print('判定:', 'OK' if fa >= 0.6 and fl >= 0.6 else 'NG (幾何が動いている)')
    # エッジの F 値は「色の入れ替わり」(暗幕 → 赤い壁、白テープが消える) を見逃すので併記する。前のモデルと比べて下がっていないか見る
    print('色の保持: ' + '  '.join(f'{k} {a / max(n, 1):.2f}' for k, (a, n) in kept.items())
          + f'  / sim に無い赤の追加 {100 * red_add[0] / max(red_add[1], 1):.1f} %')
    if a.grid and pairs:
        Image.fromarray(np.concatenate(pairs, 1)[:, :, ::-1]).save(a.grid)
        print('grid', a.grid)


if __name__ == '__main__':
    main()
