#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosbag (record.sh の mcap) から /camera/image_raw を PNG に書き出す (sim→real 学習の A ドメイン)。
ファイル名に sim 時刻 stamp と、直近の /sim/ground_truth の x, y を入れる (実画像と場所で並べて比べるため)。

  python3 tools/sim2real/extract_frames.py bags/ep_101_* -o ~/jetracer/data/sim_frames [--every 2]
ROS の Python (source scripts/sim_env.sh 後の python3、または venv の python) で動く。
"""
import argparse
import glob
import os

import numpy as np
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--every', type=int, default=1, help='N 枚に 1 枚')
    ap.add_argument('--topic', default='/camera/image_raw')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    total = 0
    for bag in sorted(set(b for p in a.bags for b in glob.glob(p) if os.path.isdir(b))):
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        img_t = get_message(types[a.topic])
        gt_t = get_message(types['/sim/ground_truth']) if '/sim/ground_truth' in types else None
        xy = (float('nan'), float('nan'))
        n = k = 0
        tag = os.path.basename(bag.rstrip('/'))
        while r.has_next():
            topic, data, _ = r.read_next()
            if topic == '/sim/ground_truth' and gt_t is not None:
                g = deserialize_message(data, gt_t)
                xy = (g.x, g.y)
            elif topic == a.topic:
                k += 1
                if (k - 1) % a.every:
                    continue
                m = deserialize_message(data, img_t)
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
                rgb = img[:, :, ::-1] if m.encoding == 'bgr8' else img
                st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                Image.fromarray(rgb).save(os.path.join(a.out, f'{tag}_{st:.3f}_x{xy[0]:.2f}_y{xy[1]:.2f}.png'))
                n += 1
        print(f'{tag}: {n} 枚')
        total += n
    print(f'計 {total} 枚 → {a.out}')


if __name__ == '__main__':
    main()
