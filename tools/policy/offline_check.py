#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
学習した policy.onnx を走らせずに確かめる。
  1. 検証 bag ごとの誤差 (sim のまま / 実画像風に変換した *_s2r を分けて)
  2. 実画像 (ラベル無し) で注視点を予測し、人が操縦したときの舵 (ファイル名の st) との相関を見る。
     注視点 u が左右に動く向きと舵の向きが揃っていれば、実画像でも道の曲がりを読めている見込み。
  3. 実画像に予測した注視点を描いた一覧画像

  ~/jetracer/venv_sim2real/bin/python tools/policy/offline_check.py --model ~/jetracer/runs/policy_001/policy.onnx \
      --data ~/jetracer/data/policy --val pol_428,pol_429,pol_430 --real ~/jetracer/data/real_260912 --grid out.png
実画像には IMU が無い (ファイル名の ax.. は別系統) ので、IMU は学習データの平均 (静止に近い値) を入れる。
"""
import argparse
import glob
import os
import re

import numpy as np
import onnxruntime as ort
from PIL import Image, ImageDraw

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def prep(rgb):
    x = (rgb.astype(np.float32) / 255.0 - MEAN) / STD
    return x.transpose(0, 3, 1, 2).copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--data', default=os.path.expanduser('~/jetracer/data/policy'))
    ap.add_argument('--val', default='pol_428,pol_429,pol_430')
    ap.add_argument('--real', default='')
    ap.add_argument('--n-real', type=int, default=2000)
    ap.add_argument('--grid', default='')
    a = ap.parse_args()
    so = ort.SessionOptions()
    sess = ort.InferenceSession(os.path.expanduser(a.model), so, providers=['CPUExecutionProvider'])
    imu_mean = None
    for pre in [v for v in a.val.split(',') if v]:
        for npz in sorted(glob.glob(os.path.join(a.data, pre + '*.npz'))):
            d = np.load(npz)
            img = np.load(npz[:-4] + '_img.npy', mmap_mode='r')
            y = d['y']
            ok = y[:, 3] > 0.5
            imu_mean = d['imu'][ok].mean(0) if imu_mean is None else imu_mean
            preds = []
            idx = np.where(ok)[0]
            for s in range(0, len(idx), 64):
                j = idx[s:s + 64]
                preds.append(sess.run(None, {'image': prep(np.asarray(img[j])[:, :, :, ::-1]), 'imu': d['imu'][j]})[0])
            p = np.concatenate(preds)
            pu, pv = (p[:, 0] + 1) / 2 * 224, (p[:, 1] + 1) / 2 * 224
            print(f'{os.path.basename(npz)[:-4]:36s} {"s2r" if "_s2r" in npz else "sim"}  n={len(idx):5d}  '
                  f'|du| {np.mean(np.abs(pu - y[idx, 0])):.2f} px  |dv| {np.mean(np.abs(pv - y[idx, 1])):.2f} px  '
                  f'|ds| {np.mean(np.abs(p[:, 2] - y[idx, 2])):.3f}')
    if not a.real:
        return
    files = sorted(glob.glob(os.path.join(os.path.expanduser(a.real), '**', '*.jpg'), recursive=True))
    rng = np.random.default_rng(0)
    files = [files[i] for i in sorted(rng.choice(len(files), min(a.n_real, len(files)), replace=False))]
    imu = np.repeat(imu_mean[None].astype(np.float32), 1, 0)
    U, S, ST, shown = [], [], [], []
    for k, f in enumerate(files):
        m = re.search(r'_st([+-][\d.]+)', os.path.basename(f))
        if not m:
            continue
        im = np.asarray(Image.open(f).convert('RGB').resize((224, 224)))
        p = sess.run(None, {'image': prep(im[None]), 'imu': imu})[0][0]
        U.append((p[0] + 1) / 2 * 224); S.append(p[2]); ST.append(float(m[1]))
        if len(shown) < 24 and k % max(1, len(files) // 24) == 0:
            shown.append((im, (p[0] + 1) / 2 * 224, (p[1] + 1) / 2 * 224, float(m[1]), p[2]))
    U, ST = np.array(U), np.array(ST)
    r = np.corrcoef(U, ST)[0, 1]
    print(f'実画像 {len(U)} 枚: 予測 u と人の舵 st の相関 r = {r:+.3f}  (u 平均 {U.mean():.1f} px, 標準偏差 {U.std():.1f} px, s 平均 {np.mean(S):.3f})')
    for lo, hi in [(-1, -0.4), (-0.4, -0.1), (-0.1, 0.1), (0.1, 0.4), (0.4, 1)]:
        sel = (ST >= lo) & (ST < hi)
        if sel.any():
            print(f'  st {lo:+.1f}〜{hi:+.1f}: {sel.sum():4d} 枚, u 平均 {U[sel].mean():6.1f} px')
    if a.grid and shown:
        W = 6
        H = (len(shown) + W - 1) // W
        g = Image.new('RGB', (W * 224, H * 224))
        for i, (im, u, v, st, s) in enumerate(shown):
            t = Image.fromarray(im)
            dr = ImageDraw.Draw(t)
            dr.ellipse([u - 6, v - 6, u + 6, v + 6], outline=(0, 255, 0), width=3)
            dr.line([112, 223, u, v], fill=(0, 255, 0), width=2)
            dr.text((4, 4), f'st {st:+.2f}  s {s:.2f}', fill=(255, 255, 0))
            g.paste(t, ((i % W) * 224, (i // W) * 224))
        g.save(a.grid)
        print('grid', a.grid)


if __name__ == '__main__':
    main()
