#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
記録済みの bag (record.sh) の /camera/image_raw を CUT の生成器 (G.onnx) で実画像風に変換し、新しい bag に書く。
stamp とほかのトピック (/imu・/sim/ground_truth・/actuator_cmd …) はそのまま複製する (オフライン変換)。

  source scripts/sim_env.sh
  ~/jetracer/venv_sim2real/bin/python tools/sim2real/convert_bag.py bags/ep_101_* --model ~/jetracer/runs/cut_001/G.onnx
  → bags/ep_101_..._s2r/ (と同名 .json に元の bag・モデル・変換時間)

画面下端の柱は変換後に入力 (Unity が描いた柱) の画素で上書きする (実機 camera_preproc のマスクと同値の領域)。
学習データの既定は「変換した bag と変換していない bag を半々」(9/12 の会場に寄せきらない。docs 参照)。
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from translator import Translator  # noqa: E402


def convert(bag, model, out, translator):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = r.get_all_topics_and_types()
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for t in topics:
        w.create_topic(rosbag2_py.TopicMetadata(name=t.name, type=t.type, serialization_format='cdr',
                                                offered_qos_profiles=t.offered_qos_profiles))
    img_t = get_message(next(t.type for t in topics if t.name == '/camera/image_raw'))
    n, dt = 0, 0.0
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic == '/camera/image_raw':
            m = deserialize_message(data, img_t)
            img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
            bgr = img if m.encoding == 'bgr8' else img[:, :, ::-1]
            t0 = time.perf_counter()
            o = translator(np.ascontiguousarray(bgr))
            dt += time.perf_counter() - t0
            if m.encoding != 'bgr8':
                o = o[:, :, ::-1]
            m.data = np.ascontiguousarray(o).tobytes()
            data = serialize_message(m)
            n += 1
        w.write(topic, data, ts)
    del w
    return n, dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--model', required=True)
    ap.add_argument('--suffix', default='_s2r')
    ap.add_argument('--threads', type=int, default=0)
    a = ap.parse_args()
    tr = Translator(os.path.expanduser(a.model), a.threads)
    rev = subprocess.run(['git', '-C', os.path.dirname(os.path.abspath(__file__)), 'rev-parse', '--short', 'HEAD'],
                         capture_output=True, text=True).stdout.strip()
    for bag in sorted(set(b.rstrip('/') for p in a.bags for b in glob.glob(p) if os.path.isdir(b))):
        if bag.endswith(a.suffix):
            continue
        out = bag + a.suffix
        if os.path.exists(out):
            print('skip (既にある)', out)
            continue
        n, dt = convert(bag, os.path.expanduser(a.model), out, tr)
        meta = dict(source=bag, model=os.path.abspath(os.path.expanduser(a.model)), frames=n,
                    ms_per_frame=round(1000 * dt / max(1, n), 1), git=rev, converted_at=time.strftime('%Y-%m-%dT%H:%M:%S'))
        src_json = bag + '.json'
        if os.path.exists(src_json):
            meta['source_meta'] = json.load(open(src_json))
        json.dump(meta, open(out + '.json', 'w'), ensure_ascii=False, indent=1)
        print(f'{os.path.basename(out)}: {n} 枚, {meta["ms_per_frame"]} ms/枚')


if __name__ == '__main__':
    main()
