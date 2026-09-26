#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最小 URDF (base_link・imu_link・camera_link) を vehicle_profile と imu_sim.yaml から生成する。
数値を手で二重に書かないための生成器。robot_state_publisher に渡す (launch が呼ぶ)。
  python3 -m jetracer_stack.urdf_gen --profile jetracer_tt02 --imu imu_sim.yaml
"""
import argparse
import math
import os
import sys

import yaml

from jetracer_common.profile import find_profile, load_profile


def build_urdf(profile_name, imu_yaml_path):
    prof = load_profile(find_profile(profile_name))
    with open(imu_yaml_path) as f:
        imu = yaml.safe_load(f)['imu_sim']['mount']
    cam = prof['camera']
    ix, iy, iz = imu['xyz']
    ir, ip, iyaw = [math.radians(v) for v in imu['rpy_deg']]
    # カメラ: 光軸は前・下向き pitch。REP-103 では下向きピッチは +y 回りに正
    cx, cy, cz = float(cam.get('mount_x_m', 0.0)), float(cam.get('mount_y_m', 0.0)), float(cam['mount_height_m'])
    cp = math.radians(float(cam['pitch_deg']))
    L = float(prof['wheelbase_m'])
    W = float(prof.get('width_m', 0.19))
    return f"""<?xml version="1.0"?>
<robot name="{prof.get('name', 'jetracer')}">
  <link name="base_link">
    <visual><origin xyz="{L / 2:.3f} 0 0.03"/><geometry><box size="{float(prof.get('length_m', 0.43)):.3f} {W:.3f} 0.06"/></geometry></visual>
  </link>
  <link name="imu_link"/>
  <link name="camera_link"/>
  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/><child link="imu_link"/>
    <origin xyz="{ix:.4f} {iy:.4f} {iz:.4f}" rpy="{ir:.5f} {ip:.5f} {iyaw:.5f}"/>
  </joint>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
    <origin xyz="{cx:.4f} {cy:.4f} {cz:.4f}" rpy="0 {cp:.5f} 0"/>
  </joint>
</robot>
"""


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='jetracer_tt02')
    ap.add_argument('--imu', default='')
    a = ap.parse_args(argv)
    imu = a.imu
    if not imu:
        from ament_index_python.packages import get_package_share_directory
        imu = os.path.join(get_package_share_directory('minicar_sim'), 'config', 'imu_sim.yaml')
    sys.stdout.write(build_urdf(a.profile, imu))


if __name__ == '__main__':
    main()
