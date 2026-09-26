#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
車体を持ち上げてモータを回したログから、振動のスペクトルと imu_sim.yaml の vibration.* を出す。

  python3 tools/imu_spectrum.py ~/bags/imu_spin_0926_1210 [--speed 0.6]
  python3 tools/imu_spectrum.py spin.csv --speed 0.6

--speed を与えると motor_hz_per_mps (基本周波数 / 車速) を出す。複数段階で回して直線あてはめする。
"""
import argparse

import numpy as np

from bag_imu import load_imu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path')
    ap.add_argument('--speed', type=float, default=0.0, help='このログの指令車速 [m/s]')
    ap.add_argument('--top', type=int, default=6)
    a = ap.parse_args()
    d = load_imu(a.path)
    t = d['t']
    fs = 1.0 / np.median(np.diff(t))
    acc = d['acc'] - d['acc'].mean(axis=0)
    n = len(t)
    win = np.hanning(n)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    print(f'# {n} samples, {fs:.1f} Hz (ナイキスト {fs / 2:.0f} Hz。それ以上の成分は折り返して見える)')
    peaks_all = []
    for k, name in enumerate('xyz'):
        X = np.abs(np.fft.rfft(acc[:, k] * win)) * 2.0 / win.sum()
        rms = float(np.sqrt(np.mean(acc[:, k] ** 2)))
        idx = np.argsort(X[1:])[::-1][:a.top] + 1
        print(f'# axis {name}: rms {rms:.3f} m/s²  peaks: ' +
              ', '.join(f'{f[i]:.1f} Hz ({X[i]:.2f})' for i in sorted(idx, key=lambda i: f[i])))
        peaks_all += [(f[i], X[i]) for i in idx]
    peaks_all.sort(key=lambda p: -p[1])
    f0, amp0 = peaks_all[0]
    print('vibration:')
    if a.speed > 0:
        print(f'  motor_hz_per_mps: {f0 / a.speed:.1f}   # {f0:.1f} Hz @ {a.speed} m/s (複数段階で直線あてはめすること)')
    else:
        print(f'  # 基本周波数 {f0:.1f} Hz (--speed を付けると motor_hz_per_mps を出す)')
    harm = [(round(fp / f0, 1), ap_) for fp, ap_ in peaks_all if fp > 0.5 * f0]
    print(f'  harmonics: {[h for h, _ in harm[:3]]}')
    print(f'  amp_ms2: {[round(float(x), 2) for _, x in harm[:3]]}')


if __name__ == '__main__':
    main()
