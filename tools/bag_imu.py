# -*- coding: utf-8 -*-
"""rosbag2 (sqlite3 / mcap) か CSV から /imu を numpy で取り出す共通部。"""
import csv
import os
import sys

import numpy as np


def load_imu(path, topic='/imu'):
    """→ dict(t, acc[N,3], gyr[N,3])。path は bag ディレクトリか CSV (t,ax,ay,az,gx,gy,gz)。"""
    if os.path.isfile(path) and path.endswith('.csv'):
        rows = []
        with open(path) as f:
            for r in csv.reader(f):
                try:
                    rows.append([float(v) for v in r[:7]])
                except ValueError:
                    continue
        a = np.asarray(rows)
        return dict(t=a[:, 0], acc=a[:, 1:4], gyr=a[:, 4:7])
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    reader = rosbag2_py.SequentialReader()
    storage_id = 'mcap' if any(fn.endswith('.mcap') for fn in os.listdir(path)) else 'sqlite3'
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    t, acc, gyr = [], [], []
    while reader.has_next():
        tp, data, ts = reader.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, Imu)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    if not t:
        sys.exit(f'{path}: {topic} が無い')
    return dict(t=np.asarray(t), acc=np.asarray(acc), gyr=np.asarray(gyr))
