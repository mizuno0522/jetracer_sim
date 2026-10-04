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
                    PARKING_SLOTS, START_LINES, START_LINE_TAPE_M, ARROW_SIGN, LIGHT_POS, LIGHT_RIG, default_course)

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


def _profile(name):
    from jetracer_common.profile import find_profile, load_profile
    return load_profile(find_profile(name))


def _profile_camera(name):
    return _profile(name)['camera']


def _vehicle_block(prof):
    """Unity が車体の大きさ (見た目の縮尺)・エンジン音の回転数・スキールの閾値に使う車両諸元。物理には使わない。"""
    mu = float(prof.get('tire_mu', 0.45))          # TT-02 は実測のフルロック横加速度 0.45 g
    return dict(name=str(prof.get('name', '')), length_m=float(prof.get('length_m', 0.43)),
                width_m=float(prof.get('width_m', 0.19)), wheelbase_m=float(prof['wheelbase_m']),
                v_max_mps=float(prof['v_max_mps']), a_lat_max_mps2=round(mu * 9.81, 2))


def _camera_block(cam):
    """course.json の camera。★定義元は vehicle_profile.camera、式は jetracer_common.cam_geom (OpenCV plumb_bob)。
    Unity は次の手順で描く (vehicle_sim の OpenCV 描画と同じ):
      1. ピンホールで、正規化座標 x ∈ [render_tan_x0, render_tan_x1]、y ∈ [render_tan_y0, render_tan_y1] の範囲を描く
         (歪みがあると出力の端はピンホールでより外側を見ているので、出力より広く描く)
      2. 出力画素 (u, v) ごとに xd = (u + 0.5 − cx) / fx, yd = (v + 0.5 − cy) / fy を、
         x = xd / (1 + k1 r² + k2 r⁴) (r² = x² + y²) の不動点反復 (5〜10 回) で逆歪みして、ピンホール画像の (x, y) を引く
      fov_deg は旧形式 (歪み無し・正方画素) の互換用。fx/fy/k1/k2 がある Unity はそちらを使うこと。"""
    from jetracer_common.cam_geom import CamGeom
    g = CamGeom.from_profile(cam)
    x0, x1, y0, y1 = g.undistorted_extent()
    th, tv = g.true_fov_deg()
    return dict(width=int(cam['width']), height=int(cam['height']),
                fov_deg=min(170.0, float(cam['hfov_deg'])),
                fx=round(g.fx, 4), fy=round(g.fy, 4), cx=round(g.cx, 4), cy=round(g.cy, 4),
                k1=g.k1, k2=g.k2,
                render_tan_x0=round(x0, 5), render_tan_x1=round(x1, 5),
                render_tan_y0=round(y0, 5), render_tan_y1=round(y1, 5),
                true_hfov_deg=round(th, 2), true_vfov_deg=round(tv, 2),
                mount_height_m=float(cam['mount_height_m']),
                pitch_deg=float(cam['pitch_deg']),
                crop_top_frac=float(cam.get('crop_top_frac', 0.0)),
                rate_hz=float(cam.get('rate_hz', 15.0)))


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
        kind='minicar',
        vehicle=_vehicle_block(_profile(profile)),
        walls=walls,
        wall_height_m=float(p.get('wall_height_m', 0.089)),
        wall_base_m=float(p.get('wall_base_m', 0.030)),     # 規約 p.34: 床から 30 mm 浮く
        wall_thickness_m=0.019,          # SPF 1x4 材の厚み
        areas=areas,
        parking_slots=slots,
        parking_tape_m=0.05,             # レギュレーション: テープ幅 5cm
        # スタートライン 1/2/3 (常設の白テープ。規約 p.24)
        start_lines=[dict(name=n, x=x, y0=y0, y1=y1, width=START_LINE_TAPE_M) for n, x, y0, y1 in START_LINES],
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
        camera=_camera_block(cam),
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


def build_circuit(course_name, profile):
    """実車スケールのサーキット (course.py の circuit_course)。壁・ギミック・駐車枠は無く、Unity は circuit から
    舗装・白線・縁石・バリア・コントロールラインを組み立てる。"""
    from course import circuit_course
    c = circuit_course(course_name)
    prof = _profile(profile)
    pts = c.center[:-1]
    flat = [round(float(v), 2) for xy in c.center for v in xy]
    vmax = float(prof['v_max_mps'])
    return dict(
        kind='circuit',
        vehicle=_vehicle_block(prof),
        circuit=dict(name=c.name, title=c.title, width_m=c.width, runoff_m=c.runoff,
                     barrier_height_m=1.0, kerb_width_m=1.2, kerb_min_curvature=1.0 / 220.0,
                     corners=[dict(name=k['name'], x=float(k['x']), y=float(k['y'])) for k in c.corners],
                     bounds=[float(pts[:, 0].min()), float(pts[:, 1].min()), float(pts[:, 0].max()), float(pts[:, 1].max())]),
        walls=[], areas=[], parking_slots=[], start_lines=[],
        wall_height_m=1.0, wall_base_m=0.0, wall_thickness_m=0.3,
        camera=_camera_block(prof['camera']),
        realism={'enable': False},
        viz=dict(speed_color_max_mps=vmax, trail_max_points=3000, trail_min_dist_m=1.5,
                 legend_x=0.0, legend_y0=0.0, legend_y1=0.0),
        centerline_shortcut=flat, centerline_long=flat,
        reference_line=[], reference_line_name='',
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
    ap.add_argument('-c', '--course', default='minicar',
                    help='minicar (既定) | fuji など config/courses/ のサーキット')
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    data = build(a.profile, route=a.route) if a.course == 'minicar' else build_circuit(a.course, a.profile)
    with open(a.out, 'w') as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
