# -*- coding: utf-8 -*-
"""jetracer.nvidia_racecar.NvidiaRacecar と同じ使い方 (steering / throttle / steering_gain / steering_offset / throttle_gain)。
書いた値を実機と同じ変換 (mapping.py) で ActuatorCmd にして /actuator_cmd に 30 Hz で出し続ける
(実機の PCA9685 はパルスを保持し続けるので、sim の 300 ms 失効に掛からないよう再送する)。"""
import threading
import time

import rclpy
import traitlets

from minicar_msgs.msg import ActuatorCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy

from . import ros_io
from .mapping import CommandMapper, load_bridge_calib, servo_pulse


class Racecar(traitlets.HasTraits):
    steering = traitlets.Float()
    throttle = traitlets.Float()

    @traitlets.validate('steering')
    def _clip_steering(self, proposal):
        return max(-1.0, min(1.0, proposal['value']))

    @traitlets.validate('throttle')
    def _clip_throttle(self, proposal):
        return max(-1.0, min(1.0, proposal['value']))


class NvidiaRacecar(Racecar):
    i2c_address = traitlets.Integer(default_value=0x40)          # 互換のため (使わない)
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0)
    steering_channel = traitlets.Integer(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    throttle_channel = traitlets.Integer(default_value=1)

    def __init__(self, *args, rate_hz=30.0, bridge_config=None, vehicle_profile='jetracer_tt02', **kwargs):
        super().__init__(*args, **kwargs)
        from jetracer_common.profile import find_profile, load_profile
        prof = load_profile(find_profile(vehicle_profile))
        self.mapper = CommandMapper(load_bridge_calib(bridge_config), prof['delta_max_rad'])
        n = ros_io.node()
        self._pub = n.create_publisher(ActuatorCmd, '/actuator_cmd',
                                       QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._period = 1.0 / rate_hz
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def command(self):
        """今の steering / throttle が実機で作る (δ rad, v m/s)。"""
        us_s = servo_pulse(self.steering * self.steering_gain + self.steering_offset)
        us_t = servo_pulse(self.throttle * self.throttle_gain)
        return self.mapper.steer_rad(us_s), self.mapper.speed_mps(us_t)

    def _loop(self):
        node = ros_io.node()
        while not self._stop:
            if not rclpy.ok():
                break
            d, v = self.command()
            m = ActuatorCmd()
            m.header.stamp = node.get_clock().now().to_msg()
            m.steer_rad, m.speed_mps, m.mode = float(d), float(v), ActuatorCmd.MODE_RUN
            self._pub.publish(m)
            time.sleep(self._period)

    def stop(self):
        """送信を止める (sim 側は 300 ms 後に失効して止まる)。"""
        self._stop = True
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
