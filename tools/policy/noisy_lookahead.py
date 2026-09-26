#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
教師 (gt_teacher) の /lookahead に揺らぎを足して流す (学習データ用。戻りの場面を作る)。
  gt_teacher を /lookahead:=/lookahead_clean に付け替え、このノードが /lookahead を出す。
正解ラベルは /sim/ground_truth の真値から付けるので、走りが揺らいでもラベルは正しいまま。

揺らぎ: 注視点の横位置 u [px] に Ornstein-Uhlenbeck 雑音 (σ・τ) を足し、ときどき (確率 p/秒) 数百 ms の「押し出し」を入れる。
  python3 tools/policy/noisy_lookahead.py --sigma 18 --tau 0.6 --kick-rate 0.15 --seed 1
"""
import argparse
import math
import random

import rclpy
from rclpy.node import Node

from minicar_msgs.msg import LookAhead


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sigma', type=float, default=18.0, help='u の OU 雑音の定常 σ [px] (画像幅 224)')
    ap.add_argument('--tau', type=float, default=0.6, help='OU の時定数 [s]')
    ap.add_argument('--kick-rate', type=float, default=0.15, help='押し出しの頻度 [回/s]')
    ap.add_argument('--kick', type=float, default=45.0, help='押し出しの大きさ [px] (u に足す)')
    ap.add_argument('--width', type=float, default=224.0)
    ap.add_argument('--kick-s', type=float, default=0.4, help='押し出しの長さ [s]')
    ap.add_argument('--seed', type=int, default=0)
    a, _ = ap.parse_known_args()
    rnd = random.Random(a.seed)
    rclpy.init()
    n = Node('noisy_lookahead')
    pub = n.create_publisher(LookAhead, '/lookahead', 10)
    st = {'x': 0.0, 't': None, 'kick_until': -1.0, 'kick_v': 0.0}

    def cb(m):
        t = n.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if st['t'] is None else max(0.0, min(0.2, t - st['t']))
        st['t'] = t
        if dt > 0:
            k = math.exp(-dt / a.tau)
            st['x'] = st['x'] * k + a.sigma * math.sqrt(1 - k * k) * rnd.gauss(0, 1)
            if t > st['kick_until'] and rnd.random() < a.kick_rate * dt:
                st['kick_until'] = t + a.kick_s
                st['kick_v'] = a.kick * rnd.choice((-1, 1))
        extra = st['kick_v'] if t < st['kick_until'] else 0.0
        o = LookAhead()
        for f in m.get_fields_and_field_types():
            setattr(o, f, getattr(m, f))
        if o.valid:
            o.u = float(max(0.0, min(a.width - 1.0, o.u + st['x'] + extra)))
        pub.publish(o)

    n.create_subscription(LookAhead, '/lookahead_clean', cb, 10)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
