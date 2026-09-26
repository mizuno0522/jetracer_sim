#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
policy_net: 画像 (3×224×224) ＋ IMU 直近窓 (50×6) → LookAhead (u, v, s)。

ONNX Runtime で推論する器。model_file が空か読めなければ LookAhead を出さない (cmd_shaper が IDLE)。
入出力の契約 (学習側と合わせる):
  入力  image: float32 [1,3,H,W] (BGR→RGB, /255, ImageNet 正規化)   imu: float32 [1,50,6] (ax ay az gx gy gz)
  出力  [1,3] = (u_norm, v_norm, s) ∈ [-1,1]²×[0,1]。u = (u_norm+1)/2 × W, v = (v_norm+1)/2 × H
camera_preproc (正規化と車体写り込みマスク) は sim と実機で同一にするためここに同居させる。
TensorRT に置き換えるときも、この入出力の形を変えない。
"""
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, Imu

from minicar_msgs.msg import LookAhead

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


class PolicyNet(Node):

    def __init__(self):
        super().__init__('policy_net')
        self.declare_parameter('model_file', '')
        self.declare_parameter('imu_window', 50)
        self.declare_parameter('mask_bottom_frac', 0.0)   # 車体写り込みの黒塗り (両系で同じ値にする)
        self.declare_parameter('providers', ['CUDAExecutionProvider', 'CPUExecutionProvider'])
        self.win = int(self.get_parameter('imu_window').value)
        self.imu = deque(maxlen=self.win)
        self.sess = None
        path = str(self.get_parameter('model_file').value).strip()
        if path:
            try:
                import onnxruntime as ort
                self.sess = ort.InferenceSession(path, providers=list(self.get_parameter('providers').value))
                self.in_names = [i.name for i in self.sess.get_inputs()]
                self.get_logger().info(f"policy_net: {path} 入力 {self.in_names} ({self.sess.get_providers()})")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"モデルを読めない ({path}): {e}。LookAhead を出さない")
        else:
            self.get_logger().warn("model_file が空。LookAhead を出さない (sim では gt_teacher を使う)")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(LookAhead, '/lookahead', 10)
        self.create_subscription(Imu, '/imu', self.cb_imu, qos)
        self.create_subscription(Image, '/camera/image_raw', self.cb_img, qos)

    def cb_imu(self, m):
        self.imu.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                         m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])

    def preprocess(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        if msg.encoding == 'bgr8':
            img = img[:, :, ::-1]
        mb = float(self.get_parameter('mask_bottom_frac').value)
        x = img.astype(np.float32) / 255.0
        if mb > 0:
            x = x.copy()
            x[int(msg.height * (1.0 - mb)):] = 0.0
        x = (x - MEAN) / STD
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None])

    def cb_img(self, msg):
        if self.sess is None:
            return
        if len(self.imu) < self.win:
            return
        feed = {self.in_names[0]: self.preprocess(msg)}
        if len(self.in_names) > 1:
            feed[self.in_names[1]] = np.asarray(self.imu, np.float32)[None]
        y = np.asarray(self.sess.run(None, feed)[0]).reshape(-1)
        m = LookAhead()
        m.header = msg.header
        m.u = float((np.clip(y[0], -1, 1) + 1.0) / 2.0 * msg.width)
        m.v = float((np.clip(y[1], -1, 1) + 1.0) / 2.0 * msg.height)
        m.s = float(np.clip(y[2], 0.0, 1.0))
        m.valid = bool(np.all(np.isfinite(y)))
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = PolicyNet()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
