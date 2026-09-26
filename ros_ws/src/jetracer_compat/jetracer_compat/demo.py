# -*- coding: utf-8 -*-
"""動作確認: JetRacer 標準の書き方そのままで sim を動かす (一定の steering / throttle)。
  ros2 run jetracer_compat jetracer_compat_demo [--seconds 10] [--throttle -0.12] [--steering -0.22]
既定の gain / offset は 9/12 実走ロガーの値 (steering_gain −0.55・offset 0.12・throttle_gain 1.0)。
この車は ★スロットルがマイナスで前進 (ESC の向き)、steering −0.22 でほぼ直進 (サーボの機械中立がずれている)。"""
import argparse
import time


def main():
    import jetracer_compat
    jetracer_compat.install()
    from jetracer.nvidia_racecar import NvidiaRacecar     # ← 他チームのコードと同じ import
    from jetcam.csi_camera import CSICamera

    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--throttle', type=float, default=-0.12)
    ap.add_argument('--steering', type=float, default=-0.22)
    ap.add_argument('--steering-gain', type=float, default=-0.55)
    ap.add_argument('--steering-offset', type=float, default=0.12)
    ap.add_argument('--throttle-gain', type=float, default=1.0)
    a, _ = ap.parse_known_args()
    car = NvidiaRacecar()
    car.steering_gain, car.steering_offset, car.throttle_gain = a.steering_gain, a.steering_offset, a.throttle_gain
    camera = CSICamera(width=224, height=224, capture_fps=15)
    print('camera', camera.value.shape, camera.value.dtype)
    car.steering = a.steering
    car.throttle = a.throttle
    d, v = car.command()
    print(f'steering {a.steering:+.2f} throttle {a.throttle:+.2f} → δ {d:+.3f} rad, v {v:.2f} m/s')
    n = 0
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        camera.read()
        n += 1
    print(f'{n} frames in {a.seconds:.0f} s ({n / a.seconds:.1f} Hz)')
    car.throttle = 0.0
    time.sleep(0.5)
    car.stop()


if __name__ == "__main__":
    main()
