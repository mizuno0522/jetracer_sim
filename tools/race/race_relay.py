#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
複数台レースの中継 (minicarbattle2026 の race_relay.py と同じもの): 別の ROS_DOMAIN_ID で走る車の
/sim/render_state を、別のドメインへ /sim/rival_state (2 台目の相手は /sim/rival2_state) として転送する。

  python3 tools/race/race_relay.py --src-domain 82 --dst-domain 81
  python3 tools/race/race_relay.py --src-domain 83 --dst-domain 81 --dst-topic /sim/rival2_state
  python3 tools/race/race_relay.py --src-domain 81 --dst-domain 82 --msg int32 \
      --src-topic /sim/arrow_dir --dst-topic /sim/arrow_master          # 矢印信号を 1 つにそろえる

各車は sim_host + jetracer_stack を自分のドメインで動かす (トピック名が絶対パスなので、ドメインで分けるのが
一番確実)。受け取った sim (vehicle_sim) は相手の姿勢で車どうしの衝突を判定し、Unity (青のドメインの 1 本の
TCP 接続) は自車を /sim/render_state、相手を /sim/rival_state・/sim/rival2_state で受けて 3 台とも描く。
使い方の全体は tools/race/race3.sh。
"""

import argparse

import rclpy
try:
    import jetracer_common.rclpy_lean  # noqa: F401  QoS イベントを作らない (rclpy の CPU 対策)
except ImportError:
    pass
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, Int32

MSG_TYPES = {'state': Float64MultiArray, 'grid': OccupancyGrid, 'int32': Int32}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src-domain', type=int, required=True)
    ap.add_argument('--dst-domain', type=int, required=True)
    ap.add_argument('--src-topic', default='/sim/render_state')
    ap.add_argument('--dst-topic', default='/sim/rival_state')
    ap.add_argument('--msg', choices=sorted(MSG_TYPES), default='state',
                    help='state=描画状態 / grid=占有格子 (/fusion/local_map)')
    a = ap.parse_args()
    msg_type = MSG_TYPES[a.msg]

    src_ctx, dst_ctx = Context(), Context()
    rclpy.init(context=src_ctx, domain_id=a.src_domain)
    rclpy.init(context=dst_ctx, domain_id=a.dst_domain)
    src = rclpy.create_node('race_relay_src', context=src_ctx)
    dst = rclpy.create_node('race_relay_dst', context=dst_ctx)
    pub = dst.create_publisher(msg_type, a.dst_topic, 10)
    src.create_subscription(msg_type, a.src_topic, pub.publish, 10)
    src.get_logger().info(
        f'{a.src_topic}@{a.src_domain} → {a.dst_topic}@{a.dst_domain}')

    ex = SingleThreadedExecutor(context=src_ctx)
    ex.add_node(src)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        src.destroy_node()
        dst.destroy_node()
        rclpy.try_shutdown(context=src_ctx)
        rclpy.try_shutdown(context=dst_ctx)


if __name__ == '__main__':
    main()
