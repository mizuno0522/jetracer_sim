#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_shaper: LookAhead (u, v, s) → /actuator_cmd (δ rad・v m/s・mode) 30 Hz。

  画像座標 → 床面の車体座標 (カメラ幾何・vehicle_profile.camera)
  → Pure Pursuit: δ = atan(2 L sin(α) / ld)   α = atan2(y, x), ld = hypot(x, y)
  → ±δmax で飽和、レート制限
  v = s × v_max、加速度制限
ここに車両の数値が集まる (ネットワークは「点」しか出さない)。sim と実機で同一コード。
LookAhead が途絶 (max_age_s) したら減速停止 (IDLE)。停止の最終手段は failsafe と ブリッジの 300 ms。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from minicar_msgs.msg import ActuatorCmd, LookAhead
from jetracer_common.profile import find_profile, load_profile
from jetracer_common.cam_geom import CamGeom


class CmdShaper(Node):

    def __init__(self):
        super().__init__('cmd_shaper')
        self.declare_parameter('vehicle_profile_file', 'jetracer_tt02')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('max_age_s', 0.30)          # LookAhead の鮮度
        self.declare_parameter('steer_rate_limit_rad_s', 8.0)
        self.declare_parameter('accel_limit_mps2', 3.0)
        self.declare_parameter('decel_limit_mps2', 6.0)
        self.declare_parameter('min_lookahead_m', 0.25)
        self.declare_parameter('v_max_override_mps', 0.0)  # >0 なら profile の v_max を上書き
        self.declare_parameter('run_topic', '/run')         # Bool true で RUN、false で IDLE
        self.declare_parameter('auto_run', False)           # true なら /run を待たず RUN
        prof = load_profile(find_profile(self.get_parameter('vehicle_profile_file').value))
        self.L = float(prof['wheelbase_m'])
        self.dmax = float(prof['delta_max_rad'])
        vo = float(self.get_parameter('v_max_override_mps').value)
        self.vmax = vo if vo > 0 else float(prof['v_max_mps'])
        self.geom = CamGeom.from_profile(prof['camera'])
        self.rate = float(self.get_parameter('rate_hz').value)
        self.max_age = float(self.get_parameter('max_age_s').value)
        self.steer_rate = float(self.get_parameter('steer_rate_limit_rad_s').value)
        self.acc_lim = float(self.get_parameter('accel_limit_mps2').value)
        self.dec_lim = float(self.get_parameter('decel_limit_mps2').value)
        self.min_ld = float(self.get_parameter('min_lookahead_m').value)
        self.run = bool(self.get_parameter('auto_run').value)
        self.la = None
        self.la_time = None
        self.steer = 0.0
        self.v = 0.0
        cmd_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(ActuatorCmd, '/actuator_cmd', cmd_qos)
        self.create_subscription(LookAhead, '/lookahead', self.cb_la, 10)
        self.create_subscription(Bool, str(self.get_parameter('run_topic').value), self.cb_run, 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"cmd_shaper: L={self.L:.3f} δmax={self.dmax:.2f} v_max={self.vmax:.2f} "
                               f"cam f={self.geom.f:.1f}px run={'auto' if self.run else 'wait /run'}")

    def cb_run(self, msg):
        self.run = bool(msg.data)
        self.get_logger().info(f"RUN = {self.run}")

    def cb_la(self, msg):
        self.la = msg
        self.la_time = self.get_clock().now()

    def pure_pursuit(self, u, v):
        g = self.geom.ground_from_pixel(u, v)
        if g is None:
            return None
        x, y = g
        ld = max(self.min_ld, math.hypot(x, y))
        alpha = math.atan2(y, x)
        return math.atan(2.0 * self.L * math.sin(alpha) / ld)

    def tick(self):
        dt = 1.0 / self.rate
        now = self.get_clock().now()
        fresh = (self.la is not None and self.la_time is not None
                 and (now - self.la_time).nanoseconds * 1e-9 <= self.max_age)
        mode = ActuatorCmd.MODE_RUN if (self.run and fresh and self.la.valid) else ActuatorCmd.MODE_IDLE
        if mode == ActuatorCmd.MODE_RUN:
            d = self.pure_pursuit(self.la.u, self.la.v)
            steer_want = self.steer if d is None else max(-self.dmax, min(self.dmax, d))
            v_want = max(0.0, min(1.0, float(self.la.s))) * self.vmax
        else:
            steer_want = self.steer
            v_want = 0.0
        lim = self.steer_rate * dt
        self.steer += max(-lim, min(lim, steer_want - self.steer))
        dv = v_want - self.v
        self.v += max(-self.dec_lim * dt, min(self.acc_lim * dt, dv))
        if v_want == 0.0 and self.v < 0.05:
            self.v = 0.0
        m = ActuatorCmd()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'base_link'
        m.steer_rad = float(self.steer)
        m.speed_mps = float(self.v)
        m.mode = mode
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = CmdShaper()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
