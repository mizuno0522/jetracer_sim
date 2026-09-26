#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gt_teacher (sim 専用): /sim/ground_truth の先行注視点 (u, v) と曲率から LookAhead (u, v, s) を出す。

模倣学習のデータ生成 (学習ループ ①) の「教師」と、policy_net が無い状態で sim の閉ループを
回すための代役。★推論スタックではない (/sim/ を購読するので実機には存在しない)。
速度係数 s は曲率と区間から: s = clip(s_max − k_curv × |κ|, s_min, s_max)、でこぼこ・坂は下げる。
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from minicar_msgs.msg import LookAhead
from minicar_sim_msgs.msg import GroundTruth, BodyState


class GtTeacher(Node):

    def __init__(self):
        super().__init__('gt_teacher')
        self.declare_parameter('s_max', 0.6)
        self.declare_parameter('s_min', 0.25)
        self.declare_parameter('curv_gain', 0.18)          # s の減り / (1/m)
        self.declare_parameter('zone_scale_rough', 0.7)
        self.declare_parameter('zone_scale_slope', 0.8)
        self.declare_parameter('zone_scale_slip', 0.8)
        self.declare_parameter('image_stamp_delay_s', 0.0)  # ラベルを画像に合わせて遅らせるとき
        self.pub = self.create_publisher(LookAhead, '/lookahead', 10)
        self.create_subscription(GroundTruth, '/sim/ground_truth', self.cb, 10)
        self.get_logger().info("gt_teacher: /sim/ground_truth → /lookahead (sim 専用の教師)")

    def cb(self, g: GroundTruth):
        p = self.get_parameter
        s = float(p('s_max').value) - float(p('curv_gain').value) * abs(g.curvature)
        if g.zone == BodyState.SURFACE_ROUGH:
            s *= float(p('zone_scale_rough').value)
        elif g.zone == BodyState.SURFACE_SLOPE:
            s *= float(p('zone_scale_slope').value)
        elif g.zone == BodyState.SURFACE_SLIP:
            s *= float(p('zone_scale_slip').value)
        s = max(float(p('s_min').value), min(float(p('s_max').value), s))
        m = LookAhead()
        m.header = g.header
        m.u = float(g.u)
        m.v = float(g.v_px)
        m.s = float(s)
        m.valid = bool(g.la_visible)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = GtTeacher()
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
