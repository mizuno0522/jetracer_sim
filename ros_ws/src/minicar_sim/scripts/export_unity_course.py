#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity 描画 (unity/MinicarSim) 用にコースとカメラ諸元を JSON へ書き出す。

  python3 export_unity_course.py            # 既定の出力先へ
  python3 export_unity_course.py -o out.json

定義元は course.py (壁・ギミック・駐車枠・矢印標識・ライト)、config/sim.yaml (壁の高さ・表示)、
config/vehicle_profile/<name>.yaml (カメラ幾何) のまま。Unity 側に数値を書かない
(sim の OpenCV 描画と Unity 描画が別のコースを見ることを防ぐ)。
course.py か sim.yaml を変えたら再実行して Unity をビルドし直すこと。

座標は course.py と同じ ROS 座標 (x 右/東, y 上/北, z 高さ, m)。
Unity 座標への変換は Unity 側 (RosFrame.ToUnity) でだけ行う。
"""

import argparse
import json
import os

import yaml

from course import (WALLS_M, WALL_COLORS, NARROW_DIVIDER_INDEX, GIMMICK_AREAS,
                    PARKING_SLOTS, ARROW_SIGN, LIGHT_POS, LIGHT_RIG, default_course)

HERE = os.path.dirname(os.path.realpath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..'))
# 既定はこのリポジトリの unity/course.json。Unity プロジェクト (minicarbattle2026/unity/MinicarSim) の
# Assets/StreamingAssets/course.json へは scripts/export_course.sh がコピーする。
DEFAULT_OUT = os.path.join(REPO, 'unity', 'course.json')
DEFAULT_PROFILE = 'jetracer_tt02'


def _sim_yaml(node='vehicle_sim'):
    # install 側ではなく src 側の sim.yaml を読む (編集した値をそのまま使う)
    path = os.path.join(HERE, '..', 'config', 'sim.yaml')
    with open(path, 'r') as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def _profile_camera(name):
    from jetracer_common.profile import find_profile, load_profile
    return load_profile(find_profile(name))['camera']


def build(profile=DEFAULT_PROFILE, route=None):
    p = _sim_yaml()
    v = _sim_yaml('sim_viz')
    cam = _profile_camera(profile)
    walls = [dict(x0=w[0], y0=w[1], x1=w[2], y1=w[3], color=c,
                  divider=(i == NARROW_DIVIDER_INDEX))
             for i, (w, c) in enumerate(zip(WALLS_M, WALL_COLORS))]
    areas = [dict(name=n, x0=x0, y0=y0, x1=x1, y1=y1)
             for n, x0, y0, x1, y1 in GIMMICK_AREAS]
    slots = [dict(name=n, color=c, x0=x0, y0=y0, x1=x1, y1=y1)
             for n, c, x0, y0, x1, y1 in PARKING_SLOTS]
    return dict(
        walls=walls,
        wall_height_m=float(p.get('wall_height_m', 0.089)),
        wall_base_m=float(p.get('wall_base_m', 0.030)),     # 規約 p.34: 床から 30 mm 浮く
        wall_thickness_m=0.019,          # SPF 1x4 材の厚み
        areas=areas,
        parking_slots=slots,
        parking_tape_m=0.05,             # レギュレーション: テープ幅 5cm
        arrow_sign=dict(x=ARROW_SIGN['x'],
                        post_y0=ARROW_SIGN['post_y'][0],
                        post_y1=ARROW_SIGN['post_y'][1],
                        board_w=ARROW_SIGN['board_w'],
                        board_h=ARROW_SIGN['board_h'],
                        board_z0=ARROW_SIGN['board_z0'],
                        post_h=ARROW_SIGN['post_h'],
                        post_r=ARROW_SIGN['post_r']),
        light=dict(x=LIGHT_POS[0], y=LIGHT_POS[1], z=LIGHT_POS[2],
                   bar_ew=LIGHT_RIG['bar_ew'], bar_ns=LIGHT_RIG['bar_ns'],
                   bar_z=LIGHT_RIG['bar_z'], leg_r=LIGHT_RIG['leg_r']),
        tunnel_height_m=1.33,            # レギュレーション: 高さ 133cm・上部 OPEN
        # ★ vehicle_profile.camera が定義元 (224×224・15 Hz の凍結値もここから)
        camera=dict(width=int(cam['width']),
                    height=int(cam['height']),
                    fov_deg=min(170.0, float(cam['hfov_deg'])),
                    mount_height_m=float(cam['mount_height_m']),
                    pitch_deg=float(cam['pitch_deg']),
                    crop_top_frac=float(cam.get('crop_top_frac', 0.0)),
                    rate_hz=float(cam.get('rate_hz', 15.0))),
        # 実カメラ風の後処理と会場の演出 (Unity のみ)。無ければ Unity 側の既定 (オフ)
        realism=cam.get('realism', {'enable': False}),
        # RViz (sim_viz) と同じ表示仕様: 走行軌跡の速度色と凡例バー
        viz=dict(speed_color_max_mps=float(v.get('speed_color_max_mps', 2.5)),
                 trail_max_points=int(v.get('trail_max_points', 700)),
                 trail_min_dist_m=float(v.get('trail_min_dist_m', 0.03)),
                 legend_x=-0.75, legend_y0=1.20, legend_y1=4.20),
        # コース中心線 (閉ループ, [x0, y0, x1, y1, ...])。Unity のミニマップと
        # 周回・ラップタイム・セクタ表示に使う。-route long で外周経路を選ぶ
        centerline_shortcut=_centerline(True),
        centerline_long=_centerline(False),
        # 参照線 (make_route.py の route.yaml)。Unity はミニマップに描くだけ (無ければ描かない)。
        # 周回・セクタの判定は従来どおり中心線で行う
        reference_line=_reference_line(route),
        reference_line_name=os.path.basename(route) if route else '',
    )


def _reference_line(route, name='shortcut'):
    if not route:
        return []
    from jetracer_common.reference_line import ReferenceLine
    with open(route) as f:
        d = yaml.safe_load(f)
    rl = ReferenceLine(d[name]['waypoints'], resample_m=0.06)
    return [round(float(v), 3) for xy in rl.pts for v in xy]


def _centerline(use_shortcut):
    c = default_course(use_shortcut=use_shortcut).center
    return [round(float(v), 3) for xy in c for v in xy]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', default=DEFAULT_OUT)
    ap.add_argument('-p', '--profile', default=DEFAULT_PROFILE,
                    help='vehicle_profile の名前かパス (カメラ幾何の定義元)')
    ap.add_argument('-r', '--route', default=None,
                    help='参照線 route.yaml (make_route.py の出力)。ミニマップ用に course.json の reference_line へ書き出す')
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        json.dump(build(a.profile, route=a.route), f, indent=1, ensure_ascii=False)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
