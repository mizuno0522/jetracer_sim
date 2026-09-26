#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
画像トピックを mp4 に録る (車が実際に見ている画。Unity 画面そのものではない)。

  python3 tools/record_video.py --seconds 60 -o ~/Videos/camera.mp4
  python3 tools/record_video.py --topic /camera/image_raw --scale 3 --hud   # 224×224 は小さいので 3 倍に

Unity の表示 (追従視点・HUD・ミニマップ) を録るなら sim_host.launch.py の record:= を使う (docs/unity.md)。
こちらは 2 ホストでも Jetson 側から録れる (受け取った画像そのものなので、経路の取りこぼしも映る)。
録画レートは受信レートに合わせる (既定 15 Hz)。--hud で sim 時刻・車速・横偏差を焼き込む。
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from minicar_sim_msgs.msg import GroundTruth


class Recorder(Node):
    def __init__(self, a):
        super().__init__('record_video')
        self.a = a
        self.bridge = CvBridge()
        self.writer = None
        self.n = 0
        self.gt = None
        self.t_first = None
        self.create_subscription(Image, a.topic, self.cb, qos_profile_sensor_data)
        if a.hud:
            self.create_subscription(GroundTruth, '/sim/ground_truth', self.cb_gt, qos_profile_sensor_data)

    def cb_gt(self, m):
        self.gt = m

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.a.scale != 1:
            img = cv2.resize(img, None, fx=self.a.scale, fy=self.a.scale, interpolation=cv2.INTER_NEAREST)
        if self.a.hud:
            img = img.copy()
            g = self.gt
            txt = (f't={g.sim_time:6.1f}s v={g.v:4.2f} cte={g.cte_m:+.3f} lap={g.lap}'
                   if g is not None else 'no /sim/ground_truth')
            cv2.rectangle(img, (0, img.shape[0] - 18), (img.shape[1], img.shape[0]), (0, 0, 0), -1)
            cv2.putText(img, txt, (4, img.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)
        if self.writer is None:
            h, w = img.shape[:2]
            self.writer = cv2.VideoWriter(self.a.out, cv2.VideoWriter_fourcc(*'mp4v'), self.a.fps, (w, h))
            if not self.writer.isOpened():
                raise SystemExit(f'{self.a.out} を開けない (親ディレクトリはあるか)')
            self.get_logger().info(f'{self.a.topic} {w}×{h} @ {self.a.fps} fps → {self.a.out}')
            self.t_first = time.monotonic()
        self.writer.write(img)
        self.n += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
            dur = time.monotonic() - self.t_first
            print(f'{self.n} フレーム, 受信 {self.n / max(1e-6, dur):.1f} Hz, '
                  f'動画長 {self.n / self.a.fps:.1f} s (実時間 {dur:.1f} s) → {self.a.out}')
            if abs(self.n / self.a.fps - dur) > 0.1 * dur:
                print('★ 動画長と実時間がずれている。--fps を受信レートに合わせること')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('-o', '--out', default=os.path.expanduser('~/Videos/camera.mp4'))
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--fps', type=float, default=15.0, help='mp4 のフレームレート (受信レートに合わせる)')
    ap.add_argument('--scale', type=int, default=1, help='拡大倍率 (224×224 は小さいので 3 など)')
    ap.add_argument('--hud', action='store_true', help='sim 時刻・車速・横偏差を焼き込む')
    a = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    rclpy.init()
    n = Recorder(a)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < a.seconds:
            rclpy.spin_once(n, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
