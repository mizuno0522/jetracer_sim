#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
failsafe: 上流側の監視。画像・IMU・指令の途絶と NaN を見て /actuator_cmd を ESTOP で上書きする。

  画像 150 ms / IMU 50 ms / 指令 100 ms の途絶 → mode=ESTOP を 30 Hz で流す
  /actuator_cmd に NaN・範囲外 → ESTOP
ブリッジ (と vehicle_sim) は ESTOP を受けると estop_hold_s の間 RUN を無視する。
ブリッジ自身の 300 ms 失効とは別の層 (安全の層: ①RC 手動切替 → ②ブリッジ 300 ms → ③ここ)。
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

from minicar_msgs.msg import ActuatorCmd


class Failsafe(Node):

    def __init__(self):
        super().__init__('failsafe')
        self.declare_parameter('image_timeout_s', 0.15)
        self.declare_parameter('imu_timeout_s', 0.05)
        self.declare_parameter('cmd_timeout_s', 0.10)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('steer_max_rad', 0.6)
        self.declare_parameter('speed_max_mps', 4.0)
        self.declare_parameter('arm_grace_s', 3.0)   # 起動直後はセンサが揃うまで待つ
        # 途絶が連続 N ティック (30 Hz) 続いたときだけ ESTOP。1 サンプルの到着ジッタで止めない
        self.declare_parameter('confirm_ticks', 2)
        self._miss = {}
        self.t_img = self.t_imu = self.t_cmd = None
        self.tripped = None
        self._last_reason = None     # ログ用 (掛け金ではない)
        self._last_log_s = -1e9
        self.t0 = self.get_clock().now()
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        cmd_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, '/camera/image_raw', lambda m: self._seen('img'), qos)
        self.create_subscription(Imu, '/imu', lambda m: self._seen('imu'), qos)
        self.create_subscription(ActuatorCmd, '/actuator_cmd', self.cb_cmd, cmd_qos)
        self.pub = self.create_publisher(ActuatorCmd, '/actuator_cmd', cmd_qos)
        self.pub_state = self.create_publisher(String, '/failsafe/state', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().info("failsafe: 画像 150 ms / IMU 50 ms / 指令 100 ms の途絶で ESTOP")

    def _seen(self, k):
        now = self.get_clock().now()
        if k == 'img':
            self.t_img = now
        elif k == 'imu':
            self.t_imu = now

    def cb_cmd(self, m):
        if m.mode == ActuatorCmd.MODE_ESTOP:
            return           # 自分 (か他の監視) の上書き
        self.t_cmd = self.get_clock().now()
        bad = (not math.isfinite(m.steer_rad) or not math.isfinite(m.speed_mps)
               or abs(m.steer_rad) > float(self.get_parameter('steer_max_rad').value)
               or abs(m.speed_mps) > float(self.get_parameter('speed_max_mps').value))
        if bad and self.tripped is None:
            self.tripped = 'cmd_invalid'

    def _age(self, t):
        return None if t is None else (self.get_clock().now() - t).nanoseconds * 1e-9

    def tick(self):
        p = self.get_parameter
        since_start = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        reason = self.tripped
        if reason is None and since_start > float(p('arm_grace_s').value):
            n_confirm = int(p('confirm_ticks').value)
            for name, t, lim in (('image', self.t_img, 'image_timeout_s'),
                                 ('imu', self.t_imu, 'imu_timeout_s'),
                                 ('cmd', self.t_cmd, 'cmd_timeout_s')):
                a = self._age(t)
                if a is None or a > float(p(lim).value):
                    self._miss[name] = self._miss.get(name, 0) + 1
                else:
                    self._miss[name] = 0
                if self._miss[name] >= n_confirm and reason is None:
                    reason = f'{name}_timeout'
        if reason is not None:
            m = ActuatorCmd()
            m.header.stamp = self.get_clock().now().to_msg()
            m.mode = ActuatorCmd.MODE_ESTOP
            self.pub.publish(m)
            # ログは理由が変わったときと 1 s に 1 回だけ (途絶が続く間に毎ティック 30 Hz で出さない)。
            # self.tripped は cmd_invalid だけを掛け金にする (途絶は回復したら自動で解除)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if self._last_reason != reason or now_s - self._last_log_s >= 1.0:
                self.get_logger().error(f"ESTOP: {reason}")
                self._last_log_s = now_s
            self._last_reason = reason
            self.tripped = reason if reason == 'cmd_invalid' else None
        elif self._last_reason is not None:
            self.get_logger().info(f"ESTOP 解除 ({self._last_reason})")
            self._last_reason = None
        self.pub_state.publish(String(data=reason or 'ok'))


def main(args=None):
    rclpy.init(args=args)
    n = Failsafe()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                       # launch の SIGINT で ExternalShutdownException が出る (Traceback を出さない)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
