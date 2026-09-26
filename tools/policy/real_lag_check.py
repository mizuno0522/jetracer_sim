#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
実走ログ (log.csv: 画像・人の舵・時刻) の 1 本を頭から順に policy.onnx に通し、
予測した注視点 u と人の舵の相関を、舵を時間方向にずらしながら測る (人は見てから少し遅れて切る)。
  ~/jetracer/venv_sim2real/bin/python tools/policy/real_lag_check.py --model runs/policy_001/policy.onnx \
      --log ~/jetracer/data/real_260912/260912_MEC_PS5/log_2.csv [--plot out.png]
IMU は実ログに角速度が無いため、sim の学習データの平均 (静止に近い値) を入れる。
"""
import argparse
import csv
import glob
import os

import numpy as np
import onnxruntime as ort
from PIL import Image

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', nargs='+', required=True)
    ap.add_argument('--log', required=True)
    ap.add_argument('--data', default=os.path.expanduser('~/jetracer/data/policy'))
    ap.add_argument('--plot', default='')
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.log)))
    base = os.path.dirname(a.log)
    suf = os.path.basename(a.log)[3:-4]          # log_2.csv → '_2' (画像は images_2/)
    path = lambda r: os.path.join(base, r['img_path'].replace('images/', f'images{suf}/', 1))
    n0 = len(rows)
    rows = [r for r in rows if os.path.exists(path(r))]      # 画像の欠けた行は除く (時刻でずらすので間は詰めない)
    t = np.array([float(r['t_frame']) for r in rows])
    st = np.array([float(r['steering']) for r in rows])
    imu = np.load(sorted(glob.glob(os.path.join(a.data, '*.npz')))[0])['imu'].mean(0, keepdims=True).astype(np.float32)
    dt = np.median(np.diff(t))
    print(f'{os.path.basename(a.log)}: {len(rows)} / {n0} 枚, {1 / dt:.1f} Hz, {t[-1] - t[0]:.0f} s')
    res = {}
    for m in a.model:
        sess = ort.InferenceSession(os.path.expanduser(m), providers=['CPUExecutionProvider'])
        U = []
        for s in range(0, len(rows), 32):
            ims = [np.asarray(Image.open(path(r)).convert('RGB').resize((224, 224)))
                   for r in rows[s:s + 32]]
            x = ((np.stack(ims).astype(np.float32) / 255 - MEAN) / STD).transpose(0, 3, 1, 2).copy()
            y = sess.run(None, {'image': x, 'imu': np.repeat(imu, len(ims), 0)})[0]
            U += list((y[:, 0] + 1) / 2 * 224)
        U = np.array(U)
        out = []
        Us = np.convolve(U, np.ones(5) / 5, mode='same')          # 5 枚 (0.33 s) の移動平均
        for lag in range(-15, 16):
            sh = t + lag * dt                                      # 舵を lag*dt 秒ずらす (負 = 人の舵が先)
            ok = (sh >= t[0]) & (sh <= t[-1])
            st_l = np.interp(sh, t, st)
            out.append((lag, lag * dt, np.corrcoef(U[ok], st_l[ok])[0, 1], np.corrcoef(Us[ok], st_l[ok])[0, 1]))
        best = max(out, key=lambda o: o[2])
        z = [o for o in out if o[0] == 0][0]
        bs = max(out, key=lambda o: o[3])
        print(f'{m}: ずれ 0 で r = {z[2]:+.3f}, 最大 r = {best[2]:+.3f} (舵を {best[1]:+.2f} s ずらす)'
              f' | 移動平均 0.33 s: ずれ 0 で {z[3]:+.3f}, 最大 {bs[3]:+.3f} ({bs[1]:+.2f} s)')
        res[m] = (U, out)
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 4))
        n = min(len(st), int(60 / dt))
        tt = t[:n] - t[0]
        ax.plot(tt, st[:n], 'k', lw=1, label='human steering')
        for m, (U, _) in res.items():
            ax.plot(tt, (U[:n] - 112) / 112 * 2.5, lw=1, label=f'pred u ({os.path.basename(os.path.dirname(m))}), scaled')
        ax.set_xlabel('s'); ax.legend(); fig.tight_layout(); fig.savefig(a.plot, dpi=90)
        print('plot', a.plot)


if __name__ == '__main__':
    main()
