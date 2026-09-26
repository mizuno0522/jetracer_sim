# -*- coding: utf-8 -*-
"""jetcam.csi_camera.CSICamera と同じ使い方 (value / read() / running / width / height)。
sim の /camera/image_raw (bgr8) を受ける。取り込み経路 (解像度・縦横比・歪み) は sim が実機に合わせて描いている。"""
import threading

import numpy as np
import traitlets

from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from . import ros_io


def _resize_nearest(img, w, h):
    if img.shape[1] == w and img.shape[0] == h:
        return img
    ys = (np.arange(h) * img.shape[0] / h).astype(int)
    xs = (np.arange(w) * img.shape[1] / w).astype(int)
    return img[ys][:, xs]


class Camera(traitlets.HasTraits):
    value = traitlets.Any()
    width = traitlets.Integer(default_value=224)
    height = traitlets.Integer(default_value=224)
    format = traitlets.Unicode(default_value='bgr8')
    running = traitlets.Bool(default_value=False)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self._running = False

    def _read(self):
        raise NotImplementedError

    def read(self):
        if self._running:
            raise RuntimeError('Cannot read directly while camera is running')
        self.value = self._read()
        return self.value

    def _capture_frames(self):
        while self._running:
            self.value = self._read()

    @traitlets.observe('running')
    def _on_running(self, change):
        if change['new'] and not change['old']:
            self._running = True
            self.thread = threading.Thread(target=self._capture_frames, daemon=True)
            self.thread.start()
        elif change['old'] and not change['new']:
            self._running = False
            self.thread.join()


class CSICamera(Camera):
    capture_device = traitlets.Integer(default_value=0)      # 互換のため (使わない)
    capture_fps = traitlets.Integer(default_value=30)        # sim は vehicle_profile の rate_hz (15 Hz) で出す
    capture_width = traitlets.Integer(default_value=640)
    capture_height = traitlets.Integer(default_value=480)

    def __init__(self, *args, topic='/camera/image_raw', timeout_s=5.0, **kwargs):
        super().__init__(*args, **kwargs)
        self._cond = threading.Condition()
        self._latest = None
        self._seq = 0
        self._timeout = timeout_s
        ros_io.node().create_subscription(Image, topic, self._cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.read()                                          # jetcam と同じく、最初の 1 枚が来るまで待つ

    def _cb(self, m):
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            img = img[:, :, ::-1]
        with self._cond:
            self._latest = _resize_nearest(img, self.width, self.height).copy()
            self._seq += 1
            self._cond.notify_all()

    def _read(self):
        with self._cond:
            seq = self._seq
            if not self._cond.wait_for(lambda: self._seq != seq, timeout=self._timeout):
                raise RuntimeError('sim の /camera/image_raw が来ない (sim_host は起動しているか、ROS_DOMAIN_ID は同じか)')
            return self._latest
