#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
/camera/image_raw と /imu の受信間隔 (抜け) と stamp→受信の遅延を一定時間測る (2 ホスト直結の切り分け用)。
どちらのホストでも同じ。送り元 (PC) で抜けが無く受け側 (Jetson) で出るなら有線区間か受信側。
  python3 tools/rate_probe.py [秒 (既定 20)]
"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rclpy.init(); n = Node('rate_probe'); qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
rec = {'/camera/image_raw': [], '/imu': []}
def mk(t):
    def cb(m):
        now = n.get_clock().now().nanoseconds * 1e-9
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rec[t].append((now, now - st))
    return cb
n.create_subscription(Image, '/camera/image_raw', mk('/camera/image_raw'), qos)
n.create_subscription(Imu, '/imu', mk('/imu'), qos)
t0 = time.monotonic()
while time.monotonic() - t0 < dur: rclpy.spin_once(n, timeout_sec=0.01)
for t, r in rec.items():
    if len(r) < 3: print(t, 'no data'); continue
    a = np.array(r); iv = np.diff(a[:, 0]); d = a[:, 1] * 1e3
    print(f"{t}: {len(r)/dur:.1f} Hz  間隔 平均 {iv.mean()*1e3:.1f} ms / p99 {np.percentile(iv,99)*1e3:.1f} / 最大 {iv.max()*1e3:.0f} ms, "
          f">200 ms の抜け {int((iv>0.2).sum())} 回;  遅延 stamp→受信 平均 {d.mean():.1f} ms / p95 {np.percentile(d,95):.1f} / 最大 {d.max():.1f} ms")
rclpy.shutdown()
