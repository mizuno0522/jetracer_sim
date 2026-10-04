#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
富士スピードウェイ (レーシングコース・全長 4,563 m・時計回り) の中心線を、公表されているコーナーの半径と
直線の長さから組み立てて ros_ws/src/minicar_sim/config/courses/fuji.yaml に書き出す。

★測量データではない。公開されている数値 (全長 4,563 m・メインストレート 1,475 m・各コーナーの R) に
  合わせた「トレース」で、曲がる角度と途中の直線の長さは推定。形が閉じる (始点に戻り、向きが 360° 回る) ように
  右コーナーの角度を一律に伸縮し、メインストレート以外の直線の長さを最小の変更で調整している。
  実際の線形 (GPX・OSM) が手に入ったら --gpx で差し替えられる。

  python3 tools/make_fuji_course.py [--plot /tmp/fuji.png] [--gpx track.gpx]

座標は course.py と同じ ROS 座標 (x 東・y 北・m)。原点はコースの外接矩形の左下。高低差は持たない (2D)。
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
OUT = os.path.normpath(os.path.join(HERE, '..', 'ros_ws', 'src', 'minicar_sim', 'config', 'courses', 'fuji.yaml'))

TOTAL_M = 4563.0          # 全長 (公表値)
MAIN_M = 1475.0           # メインストレート (公表値)
WIDTH_M = 15.0            # コース幅 (15〜25 m の狭い側)
RUNOFF_M = 8.0            # コース端からバリアまで (場所で大きく違う。sim では一律)
CONTROL_LINE_M = 900.0    # メインストレートの始まりからコントロールラインまで (推定)

# ('S', 長さ) / ('A', 半径 m, 角度 deg (+左 / -右), 名前)。半径は公表値、角度と直線は推定
SEQ = [
    ('S', MAIN_M, 'メインストレート'),
    ('A', 27, -150, 'TGRコーナー'),          # 1 コーナー 27R
    ('S', 70),
    ('A', 75, 25, None),                      # 2 コーナー 75R
    ('S', 90),
    ('A', 80, 70, 'コカ・コーラコーナー'),   # 80R
    ('S', 160),
    ('A', 105, -55, None),                    # 4〜6 コーナー 105R → 100R → 95R
    ('A', 100, -55, '100R'),
    ('A', 95, -55, None),
    ('S', 160),
    ('A', 30, 175, 'ADVANコーナー'),         # ヘアピン 30R
    ('S', 150),
    ('A', 120, -25, None),                    # 120R → 300R → 230R
    ('A', 300, -35, '300R'),
    ('A', 230, -25, None),
    ('S', 260),
    ('A', 20, -85, 'ダンロップコーナー'),    # シケイン
    ('A', 20, 70, None),
    ('S', 200),
    ('A', 60, 70, '13コーナー'),
    ('S', 150),
    ('A', 85, -45, None),                     # 85R → 25R
    ('A', 25, -60, 'GRスープラコーナー'),
    ('S', 140),
    ('A', 75, -45, None),                     # 75R → 33R
    ('A', 33, -60, 'パナソニックコーナー'),
    ('S', 60),
]
# 長さを調整しない直線 (メインストレート) と、ほとんど動かさない直線 (100R → ADVAN。伸びるとヘアピンがストレートに重なる)
FIXED = {0}
STIFF = {10: 0.03}


def walk(seq, ds=0.5):
    x = y = h = 0.0
    pts = [(0.0, 0.0)]
    marks = []
    for a in seq:
        if a[0] == 'S':
            n = max(1, int(a[1] / ds))
            st = a[1] / n
            for _ in range(n):
                x += st * math.cos(h)
                y += st * math.sin(h)
                pts.append((x, y))
        else:
            r, ang, name = a[1], math.radians(a[2]), a[3]
            arc = r * abs(ang)
            n = max(1, int(arc / ds))
            st, dh = arc / n, ang / n
            mid = None
            for i in range(n):
                h += dh / 2
                x += st * math.cos(h)
                y += st * math.sin(h)
                h += dh / 2
                pts.append((x, y))
                if i == n // 2:
                    mid = (x, y)
            if name:
                marks.append((name, mid))
    return np.array(pts), marks, h


def length(seq):
    return sum(a[1] if a[0] == 'S' else a[1] * abs(math.radians(a[2])) for a in seq)


def close_loop(seq):
    """右コーナーの角度を伸縮して向きを 360° に閉じ、直線の長さで位置と全長を閉じる"""
    L = sum(a[2] for a in seq if a[0] == 'A' and a[2] > 0)
    R = -sum(a[2] for a in seq if a[0] == 'A' and a[2] < 0)
    k = (360.0 + L) / R
    seq = [(a[0], a[1], a[2] * k, a[3]) if a[0] == 'A' and a[2] < 0 else a for a in seq]
    idx = [i for i, a in enumerate(seq) if a[0] == 'S' and i not in FIXED]
    for _ in range(4):
        pts, _, _ = walk(seq)
        end = pts[-1]
        h, dirs = 0.0, {}
        for i, a in enumerate(seq):
            if a[0] == 'S':
                dirs[i] = h
            else:
                h += math.radians(a[2])
        A = np.array([[math.cos(dirs[i]) for i in idx], [math.sin(dirs[i]) for i in idx], [1.0] * len(idx)])
        b = np.array([-end[0], -end[1], TOTAL_M - length(seq)])
        w = np.array([STIFF.get(i, 1.0) for i in idx])
        d = np.linalg.lstsq(A * np.sqrt(w), b, rcond=None)[0] * np.sqrt(w)
        seq = [(a[0], a[1] + d[idx.index(i)]) + tuple(a[2:]) if i in idx else a for i, a in enumerate(seq)]
    bad = [seq[i][1] for i in idx if seq[i][1] < 15.0]
    if bad:
        raise SystemExit(f'直線が短くなりすぎた {bad}。SEQ の角度か直線を見直す')
    return seq, k


def resample(pts, step):
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    n = int(round(cum[-1] / step))
    s = np.linspace(0.0, cum[-1], n, endpoint=False)
    return np.stack([np.interp(s, cum, pts[:, 0]), np.interp(s, cum, pts[:, 1])], 1), cum[-1]


def from_gpx(path):
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    pts = [(float(e.get('lat')), float(e.get('lon'))) for e in root.iter() if e.tag.endswith('trkpt') or e.tag.endswith('rtept')]
    if len(pts) < 10:
        raise SystemExit('GPX に点が少なすぎる')
    lat0 = pts[0][0]
    R = 6371000.0
    xy = np.array([((lo - pts[0][1]) * math.radians(1) * R * math.cos(math.radians(lat0)),
                    (la - pts[0][0]) * math.radians(1) * R) for la, lo in pts])
    return xy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--plot', default='')
    ap.add_argument('--gpx', default='', help='実際の線形 (時計回りの 1 周)。指定するとトレースの代わりに使う')
    ap.add_argument('--step', type=float, default=2.0)
    a = ap.parse_args()

    if a.gpx:
        pts = from_gpx(a.gpx)
        marks, k, source = [], 1.0, f'GPX {os.path.basename(a.gpx)}'
    else:
        seq, k = close_loop(SEQ)
        pts, marks, h = walk(seq)
        assert abs(math.degrees(h) + 360.0) < 1e-6 and np.hypot(*pts[-1]) < 1e-3
        source = 'trace from published corner radii (estimate)'
    P, total = resample(pts, a.step)
    # コントロールラインを index 0 に (メインストレートの始まりから CONTROL_LINE_M)
    if not a.gpx:
        i0 = int(round(CONTROL_LINE_M / (total / len(P))))
        P = np.roll(P, -i0, axis=0)
        marks.append(('コントロールライン', tuple(P[0])))
    lo = P.min(0) - (WIDTH_M / 2 + RUNOFF_M + 40.0)
    P = P - lo
    marks = [(n, (m[0] - lo[0], m[1] - lo[1])) for n, m in marks]

    # 自分自身に近づきすぎていないか (バリアどうしが重ならないこと)
    from itertools import combinations  # noqa: F401
    n = len(P)
    need = WIDTH_M + 2 * RUNOFF_M + 4.0
    d = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])
    gap = np.abs(np.arange(n)[:, None] - np.arange(n)[None, :])
    gap = np.minimum(gap, n - gap) * (total / n)
    close = (d < need) & (gap > 3 * need)
    if close.any():
        i, j = np.argwhere(close)[0]
        raise SystemExit(f'コースが自分に近づきすぎ: {P[i]} と {P[j]} が {d[i, j]:.1f} m (必要 {need:.0f} m)')

    out = {
        'course': {
            'name': 'fuji',
            'title': '富士スピードウェイ (レーシングコース)',
            'source': source,
            'note': '公表値 (全長 4,563 m・メインストレート 1,475 m・各コーナーの R) に合わせた推定の線形。高低差なし',
            'direction': 'clockwise',
            'total_length_m': round(float(total), 1),
            'width_m': WIDTH_M,
            'runoff_m': RUNOFF_M,
            'off_track_margin_m': 3.0,     # コース端からこれ以上出たらコース外 (タイヤが 2 本以上外れた状態)
            'collision_clear_m': 0.3,      # バリアまでの余裕がこれ未満で衝突
            'step_m': a.step,
            'corners': [{'name': nm, 'x': round(float(m[0]), 1), 'y': round(float(m[1]), 1)} for nm, m in marks],
            'centerline': [[round(float(x), 2), round(float(y), 2)] for x, y in P],
        }
    }
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write('# tools/make_fuji_course.py が書き出す。手で直さない\n')
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, default_flow_style=None, width=200)
    span = P.max(0) - P.min(0)
    print(f'wrote {a.out}: {len(P)} 点 / 全長 {total:.1f} m / 外接 {span[0]:.0f}×{span[1]:.0f} m '
          f'/ 右コーナー角度 ×{k:.3f}')
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 5))
        Q = np.vstack([P, P[:1]])
        plt.plot(Q[:, 0], Q[:, 1], lw=1)
        plt.plot(P[0, 0], P[0, 1], 'ro')
        for nm, m in marks:
            plt.text(m[0], m[1], nm, fontsize=7)
        plt.axis('equal')
        plt.savefig(a.plot, dpi=90)
        print(f'plot: {a.plot}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
