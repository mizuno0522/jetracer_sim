#!/usr/bin/env python3
"""見せる用の簡単な自動運転: コースの中心線を追って /actuator_cmd を出す (富士など実車スケールのコース)。

  ros2 launch minicar_sim sim_host.launch.py course:=fuji vehicle_profile:=real_rx7 car:=rx7 ...   # 先に sim を上げる
  python3 tools/demo/centerline_driver.py --course unity/course_fuji_real_rx7.json [--vmax 50] [--alat 8]

位置は /sim/render_state (真値) を使う。学習や評価には使わない (見た目と音を確かめるためのもの)。
速度は「曲率から決まる上限」を先に作り、ブレーキが間に合うように手前へ伝える。舵は先を見て追う (pure pursuit)。
"""
import argparse
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from minicar_msgs.msg import ActuatorCmd

X, Y, YAW, V = 2, 3, 4, 13          # SimBridge.F と同じ並び


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--course', required=True)
    ap.add_argument('--vmax', type=float, default=50.0)
    ap.add_argument('--alat', type=float, default=8.0)
    ap.add_argument('--decel', type=float, default=6.0)
    ap.add_argument('--lateral', type=float, default=0.0, help='中心線から左へずらす量 [m] (複数台で並走するとき)')
    a = ap.parse_args()
    d = json.load(open(a.course))
    c = np.array(d['centerline_shortcut']).reshape(-1, 2)
    if np.linalg.norm(c[0] - c[-1]) < 1e-6:
        c = c[:-1]
    n = len(c)
    seg = np.linalg.norm(np.roll(c, -1, 0) - c, axis=1)
    t = (np.roll(c, -1, 0) - c) / seg[:, None]
    dth = np.arctan2(np.cross(np.roll(t, 1, 0), t), (np.roll(t, 1, 0) * t).sum(1))
    k = np.abs(np.convolve(np.r_[dth[-10:], dth, dth[:10]] / np.r_[seg[-10:], seg, seg[:10]], np.ones(21) / 21, 'same')[10:-10])
    vlim = np.minimum(a.vmax, np.sqrt(a.alat / np.maximum(k, 1e-5)))
    for _ in range(2):                  # 手前へ: v² ≤ v_next² + 2·decel·ds
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            vlim[i] = min(vlim[i], math.sqrt(vlim[j] ** 2 + 2 * a.decel * seg[i]))
    wb = d['vehicle']['wheelbase_m']
    c = c + np.stack([-t[:, 1], t[:, 0]], 1) * a.lateral       # 走る線 (中心線を横へずらす)

    rclpy.init()
    node = Node('centerline_driver')
    pub = node.create_publisher(ActuatorCmd, '/actuator_cmd', 10)
    st = {'i': 0}

    def cb(m):
        s = m.data
        if len(s) <= V:
            return
        p = np.array([s[X], s[Y]])
        idx = (st['i'] + np.arange(-20, 120)) % n
        i = int(idx[np.argmin(np.linalg.norm(c[idx] - p, axis=1))])
        st['i'] = i
        v = abs(s[V])
        look = 8.0 + 0.45 * v
        j, acc = i, 0.0
        while acc < look:
            acc += seg[j]
            j = (j + 1) % n
        dx, dy = c[j] - p
        alpha = math.atan2(dy, dx) - s[YAW]
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        cmd = ActuatorCmd()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.steer_rad = float(math.atan2(2 * wb * math.sin(alpha), max(1.0, math.hypot(dx, dy))))
        cmd.speed_mps = float(vlim[i])
        cmd.mode = 1
        pub.publish(cmd)

    node.create_subscription(Float64MultiArray, '/sim/render_state', cb, 10)
    node.get_logger().info(f'centerline_driver: {n} 点、上限 {a.vmax:.0f} m/s、最も遅い所 {vlim.min():.1f} m/s')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
