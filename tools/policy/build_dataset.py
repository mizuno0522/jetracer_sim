#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
記録した bag から方策 (policy_net) の学習データを作る。画像 1 枚ごとに:
  入力  image  224×224 BGR uint8 (/camera/image_raw。*_s2r の bag なら実画像風に変換済み)
        imu    直近 50 サンプル × [ax ay az gx gy gz] (画像の stamp 以前の /imu。policy_net と同じ窓)
  正解  u, v   先行注視点の画像座標 [px] (/sim/ground_truth の真値)
        s      速度係数 [0,1] (gt_teacher と同じ式: 曲率と区間から)
        valid  注視点が画面内か
正解は教師の出力 (/lookahead) ではなく真値から作る (揺らぎを入れて走らせても正しいラベルになる)。

  source scripts/sim_env.sh
  ~/jetracer/venv_sim2real/bin/python tools/policy/build_dataset.py "bags/pol_4*" -o ~/jetracer/data/policy [--every 1]
出力: <out>/<bag 名>.npz (images は別の .npy で mmap 読み)。
"""
import argparse
import glob
import os

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
STACK_YAML = os.path.join(REPO, 'ros_ws', 'src', 'jetracer_stack', 'config', 'stack.yaml')


def teacher_params():
    with open(STACK_YAML) as f:
        p = yaml.safe_load(f)['gt_teacher']['ros__parameters']
    return dict(s_max=p.get('s_max', 0.6), s_min=p.get('s_min', 0.25), curv_gain=p.get('curv_gain', 0.18),
                rough=p.get('zone_scale_rough', 0.7), slope=p.get('zone_scale_slope', 0.8), slip=p.get('zone_scale_slip', 0.8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--every', type=int, default=1, help='画像 N 枚に 1 枚')
    ap.add_argument('--window', type=int, default=50)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    tp = teacher_params()
    BodyState = get_message('minicar_sim_msgs/msg/BodyState')
    bags = sorted(set(b.rstrip('/') for p in a.bags for b in glob.glob(p) if os.path.isdir(b)))
    total = 0
    for bag in bags:
        name = os.path.basename(bag)
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
        imgs, imu, gt = [], [], []
        k = 0
        while r.has_next():
            topic, data, _ = r.read_next()
            if topic == '/camera/image_raw':
                k += 1
                if (k - 1) % a.every:
                    continue
                m = deserialize_message(data, types[topic])
                st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                im = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == 'rgb8':
                    im = im[:, :, ::-1]
                imgs.append((st, im.copy()))
            elif topic == '/imu':
                m = deserialize_message(data, types[topic])
                st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                imu.append((st, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
            elif topic == '/sim/ground_truth':
                g = deserialize_message(data, types[topic])
                st = g.header.stamp.sec + g.header.stamp.nanosec * 1e-9
                s = tp['s_max'] - tp['curv_gain'] * abs(g.curvature)
                if g.zone == BodyState.SURFACE_ROUGH:
                    s *= tp['rough']
                elif g.zone == BodyState.SURFACE_SLOPE:
                    s *= tp['slope']
                elif g.zone == BodyState.SURFACE_SLIP:
                    s *= tp['slip']
                s = max(tp['s_min'], min(tp['s_max'], s))
                gt.append((st, g.u, g.v_px, s, float(g.la_visible), g.cte_m, float(g.collision), g.v))
        if not imgs or len(imu) < a.window or not gt:
            print(f'{name}: データ不足 (画像 {len(imgs)}, imu {len(imu)}, gt {len(gt)})')
            continue
        imu = np.array(sorted(imu), np.float64)
        gt = np.array(sorted(gt), np.float64)
        X, W, Y = [], [], []
        for st, im in imgs:
            j = np.searchsorted(imu[:, 0], st, side='right')
            if j < a.window:
                continue
            i = np.searchsorted(gt[:, 0], st, side='right') - 1
            if i < 0 or st - gt[i, 0] > 0.03:
                continue
            if gt[i, 6] > 0.5:           # 衝突中のフレームは使わない
                continue
            X.append(im)
            W.append(imu[j - a.window:j, 1:7].astype(np.float32))
            Y.append(gt[i, 1:5].astype(np.float32))      # u, v, s, valid
        if not X:
            print(f'{name}: 使える画像なし')
            continue
        np.save(os.path.join(a.out, name + '_img.npy'), np.stack(X))
        np.savez(os.path.join(a.out, name + '.npz'), imu=np.stack(W), y=np.stack(Y))
        total += len(X)
        print(f'{name}: {len(X)} 枚 (valid {int(np.stack(Y)[:, 3].sum())})')
    print(f'計 {total} 枚 → {a.out}')


if __name__ == '__main__':
    main()
