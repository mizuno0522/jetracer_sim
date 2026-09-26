#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机上判定: TT-02 の δmax で参照線が通るか (設計 未決⑦)。走行ゼロで落とせる枝。

  python3 tools/corridor_check.py --profile jetracer_tt02
  python3 tools/corridor_check.py --profile jetracer_tt02 --route ../minicarbattle2026/jetson/ros_ws/src/minicar_planning/config/route.yaml

R_min = L / tan δmax と、参照線の曲率半径の最小値・その場所・壁までの余裕を並べる。
R_min > 参照線の最小半径 なら、その区間は幾何的に通れない (参照線を引き直すか δmax を増やす)。
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros_ws', 'src', 'minicar_sim', 'scripts'))
sys.path.insert(0, os.path.join(HERE, '..', 'ros_ws', 'src', 'jetracer_common'))
from course import default_course  # noqa: E402
from jetracer_common.profile import find_profile, load_profile  # noqa: E402
from jetracer_common.reference_line import ReferenceLine  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='jetracer_tt02')
    ap.add_argument('--route', default='', help='route.yaml (空ならコース中心線)')
    ap.add_argument('--route-name', default='shortcut')
    ap.add_argument('--delta-max', type=float, default=0.0, help='δmax [rad] を上書き (実測値)')
    a = ap.parse_args()
    prof = load_profile(find_profile(a.profile))
    L = float(prof['wheelbase_m'])
    dmax = a.delta_max if a.delta_max > 0 else float(prof['delta_max_rad'])
    r_min = L / math.tan(dmax)
    course = default_course()
    ref = (ReferenceLine.from_yaml_route(a.route, a.route_name) if a.route
           else ReferenceLine(course.center))
    kappa = np.abs(ref.kappa)
    radius = 1.0 / np.maximum(1e-6, kappa)
    half_w = float(prof.get('width_m', 0.19)) / 2.0
    print(f'{prof.get("name")}: WB {L:.3f} m, δmax {dmax:.3f} rad ({math.degrees(dmax):.1f}°) → R_min {r_min:.3f} m')
    print(f'参照線: 全長 {ref.total:.2f} m, 点数 {len(ref.pts)}')
    bad = radius < r_min
    tight = radius < r_min * 1.15
    print(f'最小曲率半径 {radius.min():.3f} m @ s={ref.s[int(np.argmin(radius))]:.2f} m '
          f'({ref.pts[int(np.argmin(radius))].round(2)})')
    print(f'R_min を割る区間: {bad.sum() * (ref.s[1] - ref.s[0]):.2f} m, 余裕 15% 未満: {tight.sum() * (ref.s[1] - ref.s[0]):.2f} m')
    # 壁余裕: 参照線上の各点から壁までの距離 − 半車幅
    clear = np.array([course.clearance(x, y) for x, y in ref.pts]) - half_w
    i = int(np.argmin(clear))
    print(f'壁余裕 (片側・半車幅を引いた値) の最小 {clear[i]:.3f} m @ s={ref.s[i]:.2f} m ({ref.pts[i].round(2)})')
    # 区間ごとの表
    print('\n s[m]   R[m]   clear[m]  判定')
    step = max(1, len(ref.s) // 40)
    for j in range(0, len(ref.s), step):
        flag = '★通れない' if bad[j] else ('△きつい' if tight[j] else '')
        print(f'{ref.s[j]:6.2f} {min(radius[j], 99.0):6.2f} {clear[j]:8.3f}  {flag}')
    print('\n判定:', '★ 参照線を引き直すか δmax を増やす改造が要る' if bad.any() else '幾何的には通れる (sim で走らせる前の机上判定として OK)')


if __name__ == '__main__':
    main()
