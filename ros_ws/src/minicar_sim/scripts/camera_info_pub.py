#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
/camera/camera_info を /camera/image_raw と同じ stamp で出す (sim 側)。

Unity は CameraInfo を出さないので、vehicle_profile.camera (定義元) から K を組んで、
画像が届くたびに同じ header で配信する。実機では camera_node が同じ形で出す。
歪み係数 D は distortion: none のとき全ゼロ (plumb_bob)。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

from jetracer_common.profile import find_profile, load_profile
from jetracer_common.cam_geom import CamGeom


class CameraInfoPub(Node):

    def __init__(self):
        super().__init__('camera_info_pub')
        self.declare_parameter('vehicle_profile_file', 'jetracer_tt02')
        self.declare_parameter('frame_id', 'camera_link')
        prof = load_profile(find_profile(self.get_parameter('vehicle_profile_file').value))
        cam = prof['camera']
        g = CamGeom.from_profile(cam)
        K = g.K()
        self.info = CameraInfo()
        self.info.header.frame_id = self.get_parameter('frame_id').value
        self.info.width = g.w
        self.info.height = g.out_height
        self.info.distortion_model = 'plumb_bob'
        self.info.d = [float(v) for v in cam.get('D', [0.0, 0.0, 0.0, 0.0, 0.0])]
        self.info.k = [float(v) for v in K.flatten()]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0,
                       0.0, float(K[1, 1]), float(K[1, 2]), 0.0,
                       0.0, 0.0, 1.0, 0.0]
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos)
        self.create_subscription(Image, '/camera/image_raw', self.cb, qos)
        self.get_logger().info(f"camera_info: {g.w}x{g.out_height} f={g.f:.1f}px")

    def cb(self, msg):
        self.info.header.stamp = msg.header.stamp
        if msg.width != self.info.width or msg.height != self.info.height:
            self.get_logger().warn(
                f"画像 {msg.width}x{msg.height} と profile {self.info.width}x{self.info.height} が違う "
                f"(Unity の course.json を書き出し直してビルドすること)", throttle_duration_sec=10.0)
        self.pub.publish(self.info)


def main(args=None):
    rclpy.init(args=args)
    n = CameraInfoPub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
