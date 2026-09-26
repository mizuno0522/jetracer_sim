#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
/sim/ground_truth を一定時間購読して、周回・追従誤差・壁余裕・衝突をまとめる (参照線や教師の評価用)。

  python3 tools/lap_eval.py --seconds 120
  python3 tools/lap_eval.py --seconds 120 --csv /tmp/gt.csv    # 全サンプルも書く

sim_host.launch.py と vehicle_stack.launch.py teacher:=true auto_run:=true を上げた状態で実行する。
"""
import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from minicar_sim_msgs.msg import GroundTruth


class Eval(Node):
    def __init__(self):
        super().__init__('lap_eval')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rows = []
        self.create_subscription(GroundTruth, '/sim/ground_truth', self.cb, qos)

    def cb(self, m):
        self.rows.append((m.sim_time, m.x, m.y, m.yaw, m.v, m.s_m, m.cte_m, m.heading_err_rad,
                          m.steer_rad, m.min_wall_clear_m, int(m.collision), int(m.off_track), int(m.lap),
                          int(m.zone), m.curvature))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=90.0)
    ap.add_argument('--csv', default='')
    a = ap.parse_args()
    rclpy.init()
    n = Eval()
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.1)
    rows = n.rows
    n.destroy_node()
    rclpy.shutdown()
    if len(rows) < 10:
        sys.exit(f'/sim/ground_truth が {len(rows)} 件しか来ていない (sim と teacher は上がっているか)')
    r = np.array(rows, float)
    t, v, cte, herr, steer, clear, col, off, lap = (r[:, 0], r[:, 4], r[:, 6], r[:, 7], r[:, 8],
                                                  r[:, 9], r[:, 10], r[:, 11], r[:, 12])
    moving = v > 0.2
    print(f'{len(r)} samples, sim {t[-1] - t[0]:.1f} s, 走行 {moving.mean() * 100:.0f} %')
    # 周回
    lap_t = [t[i] for i in range(1, len(r)) if lap[i] > lap[i - 1]]
    laps = np.diff([t[0]] + lap_t) if lap_t else []
    print(f'周回 {int(lap[-1] - lap[0])} 回' + (f', ラップ {" / ".join(f"{x:.1f}" for x in laps[1:])} s'
                                            f' (初回 {laps[0]:.1f} s はスタート込み)' if len(laps) > 1 else ''))
    if moving.any():
        c = cte[moving]
        print(f'横偏差 |cte|: 最大 {np.abs(c).max():.3f} m, rms {np.sqrt((c ** 2).mean()):.3f} m, '
              f'p95 {np.percentile(np.abs(c), 95):.3f} m')
        print(f'方位誤差 最大 {math.degrees(np.abs(herr[moving]).max()):.1f}°, '
              f'舵 |δ| 最大 {math.degrees(np.abs(steer).max()):.1f}°, 飽和 (>26°) {np.mean(np.abs(steer[moving]) > math.radians(26)) * 100:.1f} %')
        print(f'車速 平均 {v[moving].mean():.2f} m/s, 最大 {v.max():.2f} m/s')
    i = int(np.argmin(clear))
    print(f'壁余裕 最小 {clear[i]:.3f} m @ ({r[i, 1]:.2f},{r[i, 2]:.2f}) s={r[i, 5]:.1f} m, '
          f'衝突 {int(np.diff(col).clip(0).sum())} 回, コース外 {int(np.diff(off).clip(0).sum())} 回')
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'v', 's', 'cte', 'herr', 'steer', 'clear', 'col', 'off', 'lap', 'zone', 'kappa'])
            w.writerows(rows)
        print('csv', a.csv)


if __name__ == '__main__':
    main()
