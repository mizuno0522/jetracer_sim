#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
複数ドメインの車へ /run を「同時に」送る (tools/race/race3.sh のスタート合図。minicarbattle2026 と同じもの)。

  python3 tools/race/race_start.py 81 82 83
  python3 tools/race/race_start.py --min-subs 1 81 82 83    # 各ドメインで /run の購読者がこの数そろうまで待つ

ドメインごとに `ros2 topic pub` を別々に叩くと、発見 (discovery) の待ちが
ドメインで違い、合図の到着が 2.7 秒ずれた (先に出た車が前の車に追いついて重なる)。
ここでは全ドメインの publisher を先に作り、どのドメインでも /run の購読者
(JetRacer では jetracer_stack の cmd_shaper 1 つ。--min-subs、既定 1) が揃ってから、同じループで
一斉に送る。取りこぼし対策に 0.5 秒おきに数回送る (再アームは無害)。

/run の前に /sim/countdown (std_msgs/Int32) で 5,4,3,2,1 を 1 秒おきに流し、
/run と同時に 0 (= GO) を送る。Unity の画面がこれでカウントダウンを出す
(自動運転 AI チャレンジのスタート表示と同じ)。--no-countdown で省く。
"""

import sys
import time

import rclpy
try:
    import jetracer_common.rclpy_lean  # noqa: F401  QoS イベントを作らない (rclpy の CPU 対策)
except ImportError:
    pass
from rclpy.context import Context
from std_msgs.msg import Bool, Int32

def main():
    argv = sys.argv[1:]
    min_subs = 1
    if '--min-subs' in argv:
        k = argv.index('--min-subs')
        min_subs = int(argv[k + 1])
        del argv[k:k + 2]
    args = [a for a in argv if a != '--no-countdown']
    countdown = '--no-countdown' not in argv
    domains = [int(d) for d in args]
    if not domains:
        print('usage: race_start.py DOMAIN [DOMAIN ...]')
        return 2
    pubs, cd_pubs, nodes, ctxs = [], [], [], []
    for d in domains:
        ctx = Context()
        rclpy.init(context=ctx, domain_id=d)
        node = rclpy.create_node(f'race_start_{d}', context=ctx)
        pubs.append(node.create_publisher(Bool, '/run', 10))
        cd_pubs.append(node.create_publisher(Int32, '/sim/countdown', 10))
        nodes.append(node)
        ctxs.append(ctx)

    t0 = time.time()
    while time.time() - t0 < 30.0:
        counts = [p.get_subscription_count() for p in pubs]
        if all(c >= min_subs for c in counts):
            break
        time.sleep(0.05)
    else:
        print(f'★購読者が揃わない: {dict(zip(domains, counts))}。そのまま送る')
    # 購読者側からもこちらを発見し終わるまで少し待つ (片側だけの発見だと落ちる)
    time.sleep(1.0)

    if countdown:
        for n in (5, 4, 3, 2, 1):
            for p in cd_pubs:
                p.publish(Int32(data=n))
            time.sleep(1.0)

    msg = Bool(data=True)
    for k in range(6):
        for p in pubs:
            p.publish(msg)
        if countdown and k < 2:
            for p in cd_pubs:
                p.publish(Int32(data=0))
        time.sleep(0.5)
    print(f'/run を同時送信: ドメイン {domains}')

    for n, c in zip(nodes, ctxs):
        n.destroy_node()
        rclpy.try_shutdown(context=c)
    return 0


if __name__ == '__main__':
    sys.exit(main())
