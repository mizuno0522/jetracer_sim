#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CSI カメラ (nvarguscamerasrc) → /camera/image_raw (bgr8 224×224・15 Hz) と /camera/camera_info。

凍結値 224×224・15 Hz は vehicle_profile.camera から。センサ (4:3 / 16:9) から正方形にする方法
(切り出し / 縮小) で画角が変わるので、ここでは **中央を正方形に切り出してから縮小** する
(square_mode: crop)。実機の meta に合わせて変えたら、profile の hfov_deg も合わせて測り直すこと。
camera_info の K は profile のピンホール近似 (歪み係数 D はチェッカーボードで測って profile.camera.D へ)。
"""
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

from jetracer_common.profile import find_profile, load_profile
from jetracer_common.cam_geom import CamGeom


def gst_pipeline(sensor_id, mode, cap_w, cap_h, fps, out_w, out_h, flip, square_mode):
    if square_mode == 'crop':
        side = min(cap_w, cap_h)
        left = (cap_w - side) // 2
        top = (cap_h - side) // 2
        crop = f"left={left} right={cap_w - left - side} top={top} bottom={cap_h - top - side} "
    else:
        crop = ""
    return (f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={mode} ! "
            f"video/x-raw(memory:NVMM),width={cap_w},height={cap_h},framerate={fps}/1,format=NV12 ! "
            f"nvvidconv flip-method={flip} {crop}! "
            f"video/x-raw,width={out_w},height={out_h},format=BGRx ! videoconvert ! "
            f"video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false")


class CsiCameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        d = self.declare_parameter
        d('vehicle_profile_file', 'jetracer_tt02')
        d('sensor_id', 0)
        d('sensor_mode', 4)          # IMX219: mode4 = 1280x720@60
        d('capture_width', 1280)
        d('capture_height', 720)
        d('capture_fps', 30)
        d('flip_method', 0)
        d('square_mode', 'crop')     # crop / scale
        d('frame_id', 'camera_link')
        d('source', 'csi')           # csi / v4l2:/dev/video0 / file:<path>
        p = self.get_parameter
        prof = load_profile(find_profile(p('vehicle_profile_file').value))
        cam = prof['camera']
        self.geom = CamGeom.from_profile(cam)
        self.w, self.h = int(cam['width']), int(cam['height'])
        self.rate = float(cam.get('rate_hz', 15.0))
        self.frame_id = str(p('frame_id').value)
        import cv2
        self.cv2 = cv2
        src = str(p('source').value)
        if src == 'csi':
            pipe = gst_pipeline(int(p('sensor_id').value), int(p('sensor_mode').value),
                                int(p('capture_width').value), int(p('capture_height').value),
                                int(p('capture_fps').value), self.w, self.h, int(p('flip_method').value),
                                str(p('square_mode').value))
            self.get_logger().info(pipe)
            self.cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        elif src.startswith('v4l2:'):
            self.cap = cv2.VideoCapture(src[5:], cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(src.replace('file:', ''))
        if not self.cap.isOpened():
            self.get_logger().error("カメラを開けない (gst-launch-1.0 nvarguscamerasrc ... で単体確認すること)")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', qos)
        K = self.geom.K()
        self.info = CameraInfo()
        self.info.header.frame_id = self.frame_id
        self.info.width, self.info.height = self.w, self.h
        self.info.distortion_model = 'plumb_bob'
        self.info.d = [float(v) for v in cam.get('D', [0, 0, 0, 0, 0])]
        self.info.k = [float(v) for v in K.flatten()]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0, 0.0, float(K[1, 1]), float(K[1, 2]), 0.0,
                       0.0, 0.0, 1.0, 0.0]
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"camera_node: {self.w}x{self.h} @ {self.rate:.0f} Hz ({src})")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return
        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame = self.cv2.resize(frame, (self.w, self.h), interpolation=self.cv2.INTER_AREA)
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.height, m.width = self.h, self.w
        m.encoding = 'bgr8'
        m.is_bigendian = 0
        m.step = self.w * 3
        m.data = np.ascontiguousarray(frame).tobytes()
        self.pub.publish(m)
        self.info.header.stamp = m.header.stamp
        self.pub_info.publish(self.info)


def main(args=None):
    rclpy.init(args=args)
    n = CsiCameraNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
