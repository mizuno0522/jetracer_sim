#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jetracer_bridge: 実機バックエンド。既存 sim の arduino_bridge が座っていた位置に置く。

  /actuator_cmd (minicar_msgs/ActuatorCmd 30 Hz) → PCA9685 ch0 (ステア) / ch1 (ESC)  パルス幅 [µs] で書く
  SY-151 (MPU-6500 互換) → /imu (sensor_msgs/Imu 100 Hz)   orientation は無効 (-1)
  /jetracer_bridge/state (String) に failsafe / ESC 状態

安全:
  - ★300 ms 失効監視 (cmd_timeout_ms): 上流が止まったらスロットル中立・舵は保持。NvidiaRacecar は
    値を書いたら次に書くまで保持するので、これが無いと全開のまま走り続ける
  - ESTOP を受けたら estop_hold_s の間 RUN を無視
  - 終了時は中立
ESC (TBLE-02S) の後退ロックは jetracer_common.EscModel (sim と同じ状態機械) を通す:
  前進 → 中立 120 ms → 後退。中立を飛ばすとブレーキになる。
"""
import math
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import String, Bool

from minicar_msgs.msg import ActuatorCmd
from jetracer_common.actuator_model import EscModel
from jetracer_common.profile import find_profile, load_profile


def _interp_table(table, x):
    """[[x, y], ...] の折れ線で補間 (端は飽和)。"""
    xs = [float(r[0]) for r in table]
    ys = [float(r[1]) for r in table]
    return float(np.interp(x, xs, ys))


class JetRacerBridge(Node):

    def __init__(self):
        super().__init__('jetracer_bridge')
        d = self.declare_parameter
        d('vehicle_profile_file', 'jetracer_tt02')
        d('i2c.bus', 1)
        d('i2c.pca9685_addr', 0x40)
        d('i2c.imu_addr', 0x68)
        d('i2c.pca_freq_hz', 50.0)
        d('steering.channel', 0)
        d('steering.pulse_us.min', 1000)       # 右いっぱい (δ = −δmax)
        d('steering.pulse_us.center', 1500)
        d('steering.pulse_us.max', 2000)       # 左いっぱい (δ = +δmax)
        d('steering.invert', False)
        d('steering.map', [0.0])              # [δ_rad, µs, δ_rad, µs, ...] 較正カーブ (空なら線形)
        d('throttle.channel', 1)
        d('throttle.pulse_us.min', 1000)
        d('throttle.pulse_us.neutral', 1500)
        d('throttle.pulse_us.max', 2000)
        d('throttle.invert', False)
        d('throttle.map_v_us', [0.0, 1500.0, 3.0, 1650.0])   # [v_mps, µs, ...] 前進の開ループ写像 ★要実測
        d('throttle.reverse_us', 1300)         # 後退の固定パルス
        d('throttle.brake_us', 1350)           # ブレーキ帯のパルス
        d('failsafe.cmd_timeout_ms', 300)
        d('failsafe.estop_hold_s', 1.0)
        d('publish.imu_hz', 100.0)
        d('publish.imu_frame', 'imu_link')
        d('imu.accel_g', 4)
        d('imu.gyro_dps', 500)
        d('imu.accel_bw_hz', 92)
        d('imu.gyro_bw_hz', 92)
        d('imu.cov_accel', 0.02)
        d('imu.cov_gyro', 0.0004)
        d('dry_run', False)                    # true なら I2C を開かない (ソフト試験)
        p = self.get_parameter
        prof = load_profile(find_profile(p('vehicle_profile_file').value))
        self.dmax = float(prof['delta_max_rad'])
        esc = prof.get('esc', {})
        self.esc = EscModel(float(esc.get('deadband_mps', 0.15)),
                            float(esc.get('reverse_via_neutral_ms', 120)) * 1e-3,
                            float(esc.get('brake_decel_mps2', 4.0)))
        self.dry = bool(p('dry_run').value)
        self.bus = None
        self.pca = None
        self.imu = None
        self.timeout = float(p('failsafe.cmd_timeout_ms').value) * 1e-3
        self.estop_hold = float(p('failsafe.estop_hold_s').value)
        self.cmd = None
        self.t_cmd = -1.0
        self.t_estop_until = -1.0
        self.v_est = 0.0             # 車輪速が無いので指令の一次遅れで代用 (ESC の状態機械用)
        self.failsafe = True
        self.state = 'INIT'
        self._open_hw()
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        cmd_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub_imu = self.create_publisher(Imu, '/imu', qos)
        self.pub_temp = self.create_publisher(Temperature, '/imu/temp', 10)
        self.pub_state = self.create_publisher(String, '/jetracer_bridge/state', 10)
        self.pub_fs = self.create_publisher(Bool, '/jetracer_bridge/failsafe', 10)
        self.create_subscription(ActuatorCmd, '/actuator_cmd', self.cb_cmd, cmd_qos)
        self.create_timer(1.0 / float(p('publish.imu_hz').value), self.tick_imu)
        self.create_timer(1.0 / 50.0, self.tick_cmd)         # PCA9685 の 50 Hz に合わせて書く
        self.create_timer(0.2, self.tick_state)
        self._n_imu = 0
        self._t_rate = time.monotonic()
        self.get_logger().info(
            f"jetracer_bridge: bus {p('i2c.bus').value} PCA9685 0x{int(p('i2c.pca9685_addr').value):02X} "
            f"IMU 0x{int(p('i2c.imu_addr').value):02X} ({self.imu.model if self.imu else 'dry_run'}) "
            f"δmax={self.dmax:.2f} 失効 {self.timeout * 1000:.0f} ms")

    # ------------------------------------------------------------------
    def _open_hw(self):
        if self.dry:
            return
        p = self.get_parameter
        try:
            from smbus2 import SMBus
            from .pca9685 import PCA9685
            from .mpu6500 import MPU6500
            self.bus = SMBus(int(p('i2c.bus').value))
            self.pca = PCA9685(self.bus, int(p('i2c.pca9685_addr').value), float(p('i2c.pca_freq_hz').value))
            self._write(self._steer_center_us(), int(p('throttle.pulse_us.neutral').value))
            self.imu = MPU6500(self.bus, int(p('i2c.imu_addr').value), int(p('imu.accel_g').value),
                               int(p('imu.gyro_dps').value), int(p('imu.accel_bw_hz').value),
                               int(p('imu.gyro_bw_hz').value), int(float(p('publish.imu_hz').value)))
            self.get_logger().info(f"IMU WHO_AM_I=0x{self.imu.who:02X} ({self.imu.model})")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"I2C を開けない: {e} (i2cdetect -y -r {p('i2c.bus').value} で 0x40 / 0x68 を確認)")
            self.bus = None
            self.pca = None
            self.imu = None

    def _steer_center_us(self):
        return int(self.get_parameter('steering.pulse_us.center').value)

    def steer_us(self, delta):
        p = self.get_parameter
        d = max(-self.dmax, min(self.dmax, float(delta)))
        if bool(p('steering.invert').value):
            d = -d
        table = list(p('steering.map').value)
        if len(table) >= 4:
            pairs = [(table[i], table[i + 1]) for i in range(0, len(table) - 1, 2)]
            return _interp_table(sorted(pairs), d)
        c = float(p('steering.pulse_us.center').value)
        if d >= 0:
            return c + d / self.dmax * (float(p('steering.pulse_us.max').value) - c)
        return c + d / self.dmax * (c - float(p('steering.pulse_us.min').value))

    def throttle_us(self, target_v, brake):
        p = self.get_parameter
        neutral = float(p('throttle.pulse_us.neutral').value)
        # map_v_us・brake_us・reverse_us は「中立より上 = 前進」の向きで書く。ESC の配線で逆 (中立より下で前進) なら
        # invert: true で中立を軸に折り返す。★これを無視すると前進のつもりで後退し、ブレーキが加速になる
        inv = bool(p('throttle.invert').value)
        flip = (lambda us: 2.0 * neutral - us) if inv else (lambda us: us)
        if brake:
            return flip(float(p('throttle.brake_us').value))
        if target_v is None:
            return neutral
        if target_v < 0:
            return flip(float(p('throttle.reverse_us').value))
        table = list(p('throttle.map_v_us').value)
        pairs = [(table[i], table[i + 1]) for i in range(0, len(table) - 1, 2)]
        us = flip(_interp_table(pairs, target_v))
        return max(float(p('throttle.pulse_us.min').value), min(float(p('throttle.pulse_us.max').value), us))

    def _write(self, steer_us, thr_us):
        self.cur_steer_us, self.cur_thr_us = int(round(steer_us)), int(round(thr_us))
        if self.pca is None:
            return
        try:
            self.pca.set_pulse_us(int(self.get_parameter('steering.channel').value), self.cur_steer_us)
            self.pca.set_pulse_us(int(self.get_parameter('throttle.channel').value), self.cur_thr_us)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"PCA9685 書き込み失敗: {e}", throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    def cb_cmd(self, m: ActuatorCmd):
        now = time.monotonic()
        if m.mode == ActuatorCmd.MODE_ESTOP:
            self.t_estop_until = now + self.estop_hold
        self.cmd = m
        self.t_cmd = now

    def tick_cmd(self):
        now = time.monotonic()
        dt = 0.02
        stale = self.cmd is None or (now - self.t_cmd) > self.timeout
        estop = now < self.t_estop_until
        run = (not stale) and (not estop) and self.cmd.mode == ActuatorCmd.MODE_RUN
        self.failsafe = stale
        if run:
            tgt, brake, st = self.esc.step(float(self.cmd.speed_mps), self.v_est, dt)
            steer = self.steer_us(self.cmd.steer_rad)
            self.state = st
        else:
            self.esc.reset()
            tgt, brake = None, False
            steer = getattr(self, 'cur_steer_us', self._steer_center_us())   # 舵は保持
            self.state = 'ESTOP' if estop else ('FAILSAFE' if stale else 'IDLE')
        # 車速の代用 (指令の一次遅れ)。実車速ではないので ESC の状態遷移の目安にだけ使う
        v_ref = 0.0 if tgt is None else tgt
        self.v_est += (v_ref - self.v_est) * min(1.0, dt / 0.4)
        self._write(steer, self.throttle_us(tgt, brake))

    def tick_imu(self):
        if self.imu is None:
            return
        try:
            acc, gyr, temp, sat = self.imu.read()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"IMU 読み出し失敗: {e}", throttle_duration_sec=2.0)
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = str(self.get_parameter('publish.imu_frame').value)
        m.orientation_covariance[0] = -1.0        # 6 軸: 姿勢は無効
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = acc
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyr
        ca = float(self.get_parameter('imu.cov_accel').value)
        cg = float(self.get_parameter('imu.cov_gyro').value)
        m.linear_acceleration_covariance = [ca, 0.0, 0.0, 0.0, ca, 0.0, 0.0, 0.0, ca]
        m.angular_velocity_covariance = [cg, 0.0, 0.0, 0.0, cg, 0.0, 0.0, 0.0, cg]
        self.pub_imu.publish(m)
        self._n_imu += 1
        if self._n_imu % 100 == 0:
            t = Temperature()
            t.header = m.header
            t.temperature = float(temp)
            self.pub_temp.publish(t)

    def tick_state(self):
        now = time.monotonic()
        hz = self._n_imu / max(1e-3, now - self._t_rate)
        self._n_imu, self._t_rate = 0, now
        self.pub_state.publish(String(
            data=f"state={self.state} failsafe={self.failsafe} steer_us={getattr(self, 'cur_steer_us', 0)} "
                 f"thr_us={getattr(self, 'cur_thr_us', 0)} imu_hz={hz:.0f}"))
        self.pub_fs.publish(Bool(data=bool(self.failsafe)))

    def destroy_node(self):
        try:
            self._write(self._steer_center_us(), int(self.get_parameter('throttle.pulse_us.neutral').value))
            if self.bus is not None:
                self.bus.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    n = JetRacerBridge()
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
