#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetRacer 実機のカメラ較正 (チェッカーボード)。sim の vehicle_profile.camera に貼る値を出す。

★ 実機と同じ経路 (jetracer_bridge の csi_camera_node → /camera/image_raw 224×224) の画像で測ること。
   取り込みモード・切り出しか縮小か・224 への縮め方が違う画像で測ると、fx・fy・歪みが別物になる。

1) 内部パラメータ (fx, fy, cx, cy, k1, k2)
     ros2 launch jetracer_bridge bridge.launch.py            # 車上で (カメラノードが上がれば良い)
     python3 tools/camera_calib.py collect --out ~/calib/intr --cols 9 --rows 6
       ボードを画面の中央・四隅・端・傾けた姿勢で 20〜40 枚。検出できて前の姿勢と十分違うときだけ自動で保存する
     python3 tools/camera_calib.py intrinsics ~/calib/intr --cols 9 --rows 6 --square 0.030
   → 再投影誤差 (0.3 px 未満が目安)・fx/fy・k1/k2・見込み角と、profile に貼る yaml

2) 取付 (高さ・ピッチ・前後位置)
     ボードを床に平らに置く。ボードの向き: 行 (cols 方向) を車の左右、列 (rows 方向) を前後に。
     最も車に近い列の、右端の内側コーナーの位置を後軸中心から巻尺で測る (前方 X0 [m]、左 Y0 [m]。右なら負)
     ★ カメラが低い (約 0.1 m) ので、床のボードは強く斜めに潰れる。**大きめのマス (40 mm 程度) で角の少ないボード
       (7×5 など) を、車のすぐ前 (x0 ≈ 0.10 m) に**置く。遠い (x0 ≈ 0.2 m・30 mm・9×6) と検出できなかった (合成画像で確認)
     python3 tools/camera_calib.py collect --out ~/calib/floor --cols 7 --rows 5 --max 3
     python3 tools/camera_calib.py mount ~/calib/floor/0000.png --cols 7 --rows 5 --square 0.040 \
             --x0 0.10 --y0 -0.14 --intr ~/calib/intr/calib.yaml
   → mount_height_m・pitch_deg・mount_x_m (と横ずれ・ロール・ヨーの確認値)

ボードは平らな板に貼る (紙のまま曲がると歪みを誤推定する)。cols/rows は内側コーナーの数。
合成画像 (fx 80・fy 100・k1 −0.05・k2 0.002、床置き 高さ 0.110 m・ピッチ 40°) で、intrinsics は fx 79.94・fy 99.91・
k1 −0.0497・k2 +0.0019 (RMS 0.10 px)、mount は 高さ 0.110 m・ピッチ 40.0°・前 0.020 m を再現した。
モデルは OpenCV plumb_bob のうち k1, k2 だけ (p1=p2=k3=0 固定) で、sim の CamGeom と同じ。
"""
import argparse
import glob
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros_ws', 'src', 'jetracer_common'))


def board_points(cols, rows, square):
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return obj


def find_corners(gray, cols, rows):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, c = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        # 224×224 は小さいので 2 倍に上げてもう一度
        big = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        ok, c = cv2.findChessboardCorners(big, (cols, rows), flags)
        if not ok:
            return None
        c = c / 2.0
    c = cv2.cornerSubPix(gray, c.astype(np.float32), (3, 3), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01))
    return c


# ----------------------------------------------------------------------------- collect
def cmd_collect(a):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    os.makedirs(os.path.expanduser(a.out), exist_ok=True)
    out = os.path.expanduser(a.out)
    rclpy.init()
    node = rclpy.create_node('camera_calib_collect')
    br = CvBridge()
    kept = []          # (中心, 大きさ, 傾き)
    state = dict(n=len(glob.glob(os.path.join(out, '*.png'))))

    def cb(msg):
        img = br.imgmsg_to_cv2(msg, 'bgr8')
        c = find_corners(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), a.cols, a.rows)
        if c is None:
            return
        pts = c.reshape(-1, 2)
        ctr = pts.mean(axis=0)
        size = math.sqrt(cv2.contourArea(cv2.convexHull(pts)))
        d = pts[a.cols - 1] - pts[0]
        ang = math.degrees(math.atan2(d[1], d[0]))
        for kc, ks, ka in kept:
            if np.hypot(*(ctr - kc)) < a.min_move and abs(size - ks) < 0.15 * ks and abs(ang - ka) < 8:
                return
        kept.append((ctr, size, ang))
        path = os.path.join(out, f"{state['n']:04d}.png")
        cv2.imwrite(path, img)
        state['n'] += 1
        node.get_logger().info(f"保存 {path} (中心 {ctr.round(0)}, 大きさ {size:.0f} px, 傾き {ang:.0f}°)")

    node.create_subscription(Image, a.topic, cb, qos_profile_sensor_data)
    node.get_logger().info(f"{a.topic} でボード ({a.cols}×{a.rows}) を待つ。{a.max} 枚で終了 (Ctrl-C でも可)")
    try:
        while rclpy.ok() and len(kept) < a.max:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(f'{len(kept)} 枚 → {out}')


# ----------------------------------------------------------------------------- intrinsics
def cmd_intrinsics(a):
    files = sorted(glob.glob(os.path.join(os.path.expanduser(a.dir), '*.png')) +
                   glob.glob(os.path.join(os.path.expanduser(a.dir), '*.jpg')))
    obj = board_points(a.cols, a.rows, a.square)
    objs, imgs, size = [], [], None
    for f in files:
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        size = g.shape[::-1]
        c = find_corners(g, a.cols, a.rows)
        if c is not None:
            objs.append(obj)
            imgs.append(c)
    if len(imgs) < 8:
        sys.exit(f'ボードが見えた画像が {len(imgs)} 枚しか無い (8 枚以上。できれば 20〜40 枚)')
    flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    if a.square_pixels:
        flags |= cv2.CALIB_FIX_ASPECT_RATIO
    K0 = np.array([[size[0] / 2.0, 0, size[0] / 2.0], [0, size[0] / 2.0, size[1] / 2.0], [0, 0, 1]])
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objs, imgs, size, K0, None, flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    k1, k2 = float(D.ravel()[0]), float(D.ravel()[1])
    per = []
    for o, c, r, t in zip(objs, imgs, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, D)
        per.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - c.reshape(-1, 2)) ** 2, axis=1)))))
    from jetracer_common.cam_geom import CamGeom
    g = CamGeom(size[0], size[1], 90.0, 0.1, 0.0, fx=fx, fy=fy, cx=cx, cy=cy, k1=k1, k2=k2)
    th, tv = g.true_fov_deg()
    hfov = math.degrees(2 * math.atan((size[0] / 2.0) / fx))
    vfov = math.degrees(2 * math.atan((size[1] / 2.0) / fy))
    print(f'{len(imgs)}/{len(files)} 枚、画像 {size[0]}×{size[1]}、再投影誤差 RMS {rms:.3f} px '
          f'(最悪の 1 枚 {max(per):.3f} px)')
    print(f'fx {fx:.2f}  fy {fy:.2f}  (fy/fx {fy / fx:.3f})  cx {cx:.2f}  cy {cy:.2f}  k1 {k1:+.4f}  k2 {k2:+.4f}')
    print(f'ピンホール等価 hfov {hfov:.1f}°・vfov {vfov:.1f}°、歪み込みの見込み角 水平 {th:.0f}°・垂直 {tv:.0f}°')
    print('fy/fx の読み: 1.0 = 正方画素 (正方形に切り出し)、1.33 = 4:3 を 224×224 に縮小、1.78 = 16:9 を縮小')
    if rms > 0.5:
        print('★ 再投影誤差が大きい。ボードが曲がっていないか、ブレた画像が混ざっていないか確認する')
    res = dict(image_width=size[0], image_height=size[1], rms_px=round(float(rms), 4), n_images=len(imgs),
               fx=round(float(fx), 3), fy=round(float(fy), 3), cx=round(float(cx), 3), cy=round(float(cy), 3),
               k1=round(k1, 5), k2=round(k2, 5), hfov_deg=round(hfov, 2), vfov_deg=round(vfov, 2),
               true_hfov_deg=round(th, 1), true_vfov_deg=round(tv, 1))
    out = os.path.join(os.path.expanduser(a.dir), 'calib.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(res, f, sort_keys=False)
    print(f'\n→ {out}\n\n# vehicle_profile.camera に貼る (チェッカーボード較正 {time.strftime("%Y-%m-%d")}、RMS {rms:.2f} px)')
    print(yaml.safe_dump(dict(hfov_deg=res['hfov_deg'], vfov_deg=res['vfov_deg'], cx=res['cx'], cy=res['cy'],
                              k1=res['k1'], k2=res['k2']), sort_keys=False).rstrip())


# ----------------------------------------------------------------------------- mount
def cmd_mount(a):
    c = yaml.safe_load(open(os.path.expanduser(a.intr)))
    K = np.array([[c['fx'], 0, c['cx']], [0, c['fy'], c['cy']], [0, 0, 1]], float)
    D = np.array([c['k1'], c['k2'], 0, 0, 0], float)
    g = cv2.imread(os.path.expanduser(a.image), cv2.IMREAD_GRAYSCALE)
    corners = find_corners(g, a.cols, a.rows)
    if corners is None:
        sys.exit('ボードが見つからない')
    # ボード上の点を車体座標で: cols 方向 = 車の左 (+y)、rows 方向 = 前 (+x)。原点 = 最も近い列の右端
    obj = np.zeros((a.rows * a.cols, 3), np.float64)
    ii, jj = np.mgrid[0:a.cols, 0:a.rows]
    obj[:, 0] = a.x0 + jj.T.reshape(-1) * a.square
    obj[:, 1] = a.y0 + ii.T.reshape(-1) * a.square
    # 検出されたコーナーの並びとボードの向きの対応は 4 通りあり得るので、全部試して誤差最小で高さが正のものを採る
    best = None
    cand = corners.reshape(a.rows, a.cols, 2)
    for flip_r in (False, True):
        for flip_c in (False, True):
            cc = cand[::-1 if flip_r else 1, ::-1 if flip_c else 1].reshape(-1, 1, 2)
            ok, rvec, tvec = cv2.solvePnP(obj, cc, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            p, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
            err = float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - cc.reshape(-1, 2)) ** 2, axis=1))))
            R, _ = cv2.Rodrigues(rvec)
            Cw = (-R.T @ tvec).ravel()                 # カメラ中心 (車体座標)
            if Cw[2] > 0 and (best is None or err < best[0]):
                best = (err, R, Cw)
    if best is None:
        sys.exit('姿勢が解けない (x0/y0 とボードの向きを確認)')
    err, R, Cw = best
    zc = R.T @ np.array([0, 0, 1.0])          # 光軸 (車体座標)
    xc = R.T @ np.array([1.0, 0, 0])          # 画像の右方向
    pitch = math.degrees(math.asin(-zc[2]))
    yaw = math.degrees(math.atan2(zc[1], zc[0]))
    roll = math.degrees(math.atan2(-xc[2], math.hypot(xc[0], xc[1])))
    print(f'再投影誤差 {err:.3f} px')
    print(f'カメラ位置 (後軸中心から): 前 {Cw[0]:+.3f} m・左 {Cw[1]:+.3f} m・高さ {Cw[2]:.3f} m')
    print(f'ピッチ (下向き) {pitch:.1f}°、ヨー {yaw:+.1f}° (左正)、ロール {roll:+.1f}°')
    if abs(yaw) > 2 or abs(roll) > 2:
        print('★ ヨーかロールが 2° を超える。sim は 0 を仮定しているので、取付を直すか、sim に項を足す')
    print('\n# vehicle_profile.camera に貼る')
    print(yaml.safe_dump(dict(mount_height_m=round(float(Cw[2]), 4), mount_x_m=round(float(Cw[0]), 4),
                              pitch_deg=round(pitch, 2)), sort_keys=False).rstrip())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)
    c = sp.add_parser('collect')
    c.add_argument('--out', required=True)
    c.add_argument('--topic', default='/camera/image_raw')
    c.add_argument('--cols', type=int, default=9)
    c.add_argument('--rows', type=int, default=6)
    c.add_argument('--max', type=int, default=40)
    c.add_argument('--min-move', type=float, default=18.0, help='前の保存からの中心の移動 [px]')
    i = sp.add_parser('intrinsics')
    i.add_argument('dir')
    i.add_argument('--cols', type=int, default=9)
    i.add_argument('--rows', type=int, default=6)
    i.add_argument('--square', type=float, required=True, help='マスの一辺 [m]')
    i.add_argument('--square-pixels', action='store_true', help='fy = fx に固定 (正方に切り出していると分かっているとき)')
    m = sp.add_parser('mount')
    m.add_argument('image')
    m.add_argument('--intr', required=True, help='intrinsics が書いた calib.yaml')
    m.add_argument('--cols', type=int, default=9)
    m.add_argument('--rows', type=int, default=6)
    m.add_argument('--square', type=float, required=True)
    m.add_argument('--x0', type=float, required=True, help='最も近い列の右端コーナー: 後軸中心から前へ [m]')
    m.add_argument('--y0', type=float, required=True, help='同: 左へ [m] (右なら負)')
    a = ap.parse_args()
    dict(collect=cmd_collect, intrinsics=cmd_intrinsics, mount=cmd_mount)[a.cmd](a)


if __name__ == '__main__':
    main()
