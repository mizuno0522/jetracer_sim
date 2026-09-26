#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
静止ログ (10 分以上) の Allan 分散から imu_sim.yaml の noise.* を出す。

  python3 tools/imu_allan.py ~/bags/imu_static_0926_1200        # rosbag2
  python3 tools/imu_allan.py static.csv                          # t,ax,ay,az,gx,gy,gz

出力: 雑音密度 (τ=1 s の Allan 偏差 ≈ N)、バイアス不安定性 (最小点 / 0.664)、
ランダムウォーク係数 (τ=3 s の傾き +1/2 から K)。yaml 形式で印字するので imu_sim.yaml に貼る。
"""
import sys

import numpy as np

from bag_imu import load_imu


def allan_dev(x, fs, taus):
    x = np.asarray(x, float)
    out = []
    for tau in taus:
        m = int(round(tau * fs))
        if m < 1 or m > len(x) // 3:
            out.append(np.nan)
            continue
        n = len(x) // m
        means = x[:n * m].reshape(n, m).mean(axis=1)
        out.append(np.sqrt(0.5 * np.mean(np.diff(means) ** 2)))
    return np.asarray(out)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    d = load_imu(sys.argv[1])
    t = d['t']
    fs = 1.0 / np.median(np.diff(t))
    dur = t[-1] - t[0]
    print(f'# {len(t)} samples, {fs:.1f} Hz, {dur / 60:.1f} min')
    if dur < 300:
        print('# ★ 5 分未満。バイアス不安定性の最小点が出ない可能性')
    taus = np.logspace(-2, np.log10(max(1.0, dur / 10)), 40)
    res = {}
    for name, arr, unit in (('accel', d['acc'], 'm/s²'), ('gyro', d['gyr'], 'rad/s')):
        nd, rw, bi = [], [], []
        for k in range(3):
            ad = allan_dev(arr[:, k] - arr[:, k].mean(), fs, taus)
            i1 = np.nanargmin(np.abs(taus - 1.0))
            nd.append(ad[i1])                                   # N: τ=1 s
            imin = np.nanargmin(ad)
            bi.append(ad[imin] / 0.664)                          # B
            i3 = np.nanargmin(np.abs(taus - 3.0))
            rw.append(ad[i3] / np.sqrt(3.0 / 3.0) * np.sqrt(3.0) / 3.0 if not np.isnan(ad[i3]) else np.nan)  # K ≈ σ(τ)·√(3/τ)
        res[name] = (nd, rw, bi)
        print(f'# {name}: N (τ=1s) = {np.round(nd, 6)} {unit}/√Hz   B = {np.round(bi, 6)} {unit}   K = {np.round(rw, 6)} {unit}/√s')
    print('noise:')
    print(f"  accel_nd: {np.mean(res['accel'][0]):.6f}   # m/s²/√Hz")
    print(f"  gyro_nd: {np.mean(res['gyro'][0]):.7f}   # rad/s/√Hz")
    print(f"  accel_rw: {np.nanmean(res['accel'][1]):.6f}   # m/s²/√s")
    print(f"  gyro_rw: {np.nanmean(res['gyro'][1]):.7f}   # rad/s/√s")
    print('# turn_on_bias は電源入れ直し ×10 の初期値の分布から (このスクリプトでは出ない)')


if __name__ == '__main__':
    main()
