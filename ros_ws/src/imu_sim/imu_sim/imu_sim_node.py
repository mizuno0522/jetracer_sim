#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_sim: /sim/body_state (100 Hz・物理の剛体状態) → /imu (sensor_msgs/Imu・100 Hz)。

センサモデル本体は jetracer_common.imu_model (ROS 非依存・単体テスト済み)。ここは
  - config_file (imu_sim.yaml) を読んでモデルを作る
  - /sim/episode (seed) が来たらエピソードを引き直す (ターンオンバイアス・取付角誤差・スケール誤差)
  - sensor_msgs/Imu の埋め方: orientation は無効 (orientation_covariance[0] = -1)、
    linear_acceleration は重力を含む生の比力、covariance は雑音の分散 (Allan の実測で置換)
の 3 つだけ。時間軸は body_state の sim_time (lockstep でも同じコードが動く)。
出力の stamp は body_state の stamp ＋ (出力時刻 − 物理時刻)。
"""
import os

import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import UInt32, Bool

from minicar_sim_msgs.msg import BodyState
from jetracer_common.imu_model import ImuModel

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)


class ImuSimNode(Node):

    def __init__(self):
        super().__init__('imu_sim')
        self.declare_parameter('config_file', '')
        self.declare_parameter('seed', 0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_temperature', True)
        # 陽性対照 (較正タブ): true にすると /imu を定数ゼロにする
        self.declare_parameter('zero_output', False)
        path = os.path.expanduser(self.get_parameter('config_file').value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('minicar_sim'), 'config', 'imu_sim.yaml')
        with open(path, 'r') as f:
            self.cfg = yaml.safe_load(f)['imu_sim']
        self.frame_id = str(self.cfg.get('frame_id', self.get_parameter('frame_id').value))
        self.zero = bool(self.get_parameter('zero_output').value)
        self.model = ImuModel(self.cfg, seed=int(self.get_parameter('seed').value))
        va, vg = self.model.covariances()
        self.cov_a = [va, 0.0, 0.0, 0.0, va, 0.0, 0.0, 0.0, va]
        self.cov_g = [vg, 0.0, 0.0, 0.0, vg, 0.0, 0.0, 0.0, vg]

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Imu, str(self.cfg.get('imu_topic', '/imu')), qos)
        self.pub_temp = (self.create_publisher(Temperature, '/imu/temp', 10)
                         if bool(self.get_parameter('publish_temperature').value) else None)
        self.pub_sat = self.create_publisher(Bool, '/sim/imu_saturated', 10)
        self.create_subscription(BodyState, str(self.cfg.get('body_state_topic', '/sim/body_state')),
                                 self.cb_body, 50)
        self.create_subscription(UInt32, '/sim/episode', self.cb_episode, LATCHED)
        self._n = 0
        self._t_last_temp = -1.0
        self.get_logger().info(
            f"imu_sim: {path} 内部 {self.cfg['rate']['internal_hz']} Hz → 出力 {self.cfg['rate']['output_hz']} Hz, "
            f"取付 rpy={self.cfg['mount']['rpy_deg']}, 乱択化 scale={self.cfg.get('domain_rand', {}).get('scale', 1.0)}"
            + (" ★zero_output (陽性対照)" if self.zero else ""))

    def cb_episode(self, msg):
        self.model.new_episode(int(msg.data))
        self.get_logger().info(f"エピソード seed={msg.data}: ターンオンバイアス等を引き直した")

    def cb_body(self, m: BodyState):
        samples = self.model.push(m.sim_time, (m.ax, m.ay, m.az), (m.wx, m.wy, m.wz),
                                  m.roll, m.pitch, m.v, int(m.surface), m.rough_frac)
        if not samples:
            return
        base = Time.from_msg(m.header.stamp)
        for s in samples:
            out = Imu()
            dt = s.t - m.sim_time                 # 出力時刻は物理時刻より少し前 (≤ 0)
            out.header.stamp = (base + Duration(seconds=dt)).to_msg() if dt >= 0 else \
                (base - Duration(seconds=-dt)).to_msg()
            out.header.frame_id = self.frame_id
            # 6 軸なので姿勢は入れない (地磁気が無く絶対方位が出せない)
            out.orientation.w = 0.0
            out.orientation_covariance[0] = -1.0
            if self.zero:
                a = np.zeros(3)
                g = np.zeros(3)
            else:
                a, g = s.accel, s.gyro
            out.linear_acceleration.x = float(a[0])
            out.linear_acceleration.y = float(a[1])
            out.linear_acceleration.z = float(a[2])
            out.angular_velocity.x = float(g[0])
            out.angular_velocity.y = float(g[1])
            out.angular_velocity.z = float(g[2])
            out.linear_acceleration_covariance = self.cov_a
            out.angular_velocity_covariance = self.cov_g
            self.pub.publish(out)
            if s.saturated:
                self.pub_sat.publish(Bool(data=True))
            self._n += 1
            if self.pub_temp is not None and s.t - self._t_last_temp >= 1.0:
                self._t_last_temp = s.t
                t = Temperature()
                t.header = out.header
                t.temperature = float(s.temp_c)
                self.pub_temp.publish(t)


def main(args=None):
    rclpy.init(args=args)
    n = ImuSimNode()
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
