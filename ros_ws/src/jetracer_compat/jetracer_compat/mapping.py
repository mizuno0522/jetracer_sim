# -*- coding: utf-8 -*-
"""
JetRacer 標準の正規化指令 (NvidiaRacecar の steering / throttle ∈ [-1, 1]) → PWM パルス → sim の ActuatorCmd (δ rad, v m/s)。

実機では NvidiaRacecar が ServoKit.continuous_servo に u = 値 × gain + offset を書き、PCA9685 が
パルス = 1500 + 750·u µs (adafruit_motor ContinuousServo の既定 750〜2250 µs) を出す。
そのパルスが実機で起こす δ・v は jetracer_bridge の較正 (config/jetracer_bridge.yaml) の逆写像で求める。
→ 実機と sim で「同じ正規化値が同じ舵角・車速になる」ことを 1 つの較正ファイルで保証する (数値を二重に持たない)。
"""
import os

import yaml

PULSE_CENTER = 1500.0
PULSE_HALF = 750.0          # ContinuousServo 既定 (min 750 / max 2250 µs)


def load_bridge_calib(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('jetracer_bridge'), 'config', 'jetracer_bridge.yaml')
    with open(path) as f:
        d = yaml.safe_load(f)
    return d.get('jetracer_bridge', d).get('ros__parameters', d.get('jetracer_bridge', d))


def servo_pulse(u):
    u = max(-1.0, min(1.0, float(u)))
    return PULSE_CENTER + PULSE_HALF * u


def _interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / max(1e-9, xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


class CommandMapper:
    """パルス → (δ rad, v m/s)。bridge の steer_us / throttle_us の逆。"""

    def __init__(self, calib, delta_max_rad, v_reverse_max_mps=1.0):
        st, th = calib['steering'], calib['throttle']
        self.s_min, self.s_c, self.s_max = (float(st['pulse_us'][k]) for k in ('min', 'center', 'max'))
        self.s_inv = bool(st.get('invert', False))
        table = [float(v) for v in st.get('map', [])]
        self.s_map = sorted((table[i + 1], table[i]) for i in range(0, len(table) - 1, 2)) if len(table) >= 4 else None
        self.dmax = float(delta_max_rad)
        self.t_min, self.t_n, self.t_max = (float(th['pulse_us'][k]) for k in ('min', 'neutral', 'max'))
        self.t_inv = bool(th.get('invert', False))
        tv = [float(v) for v in th.get('map_v_us', [0.0, 1500.0, 3.0, 1660.0])]
        pairs = sorted((tv[i + 1], tv[i]) for i in range(0, len(tv) - 1, 2))      # (µs, v)
        self.t_us, self.t_v = [p[0] for p in pairs], [p[1] for p in pairs]
        self.v_rev = float(v_reverse_max_mps)

    def steer_rad(self, pulse):
        if self.s_inv:
            pulse = 2 * self.s_c - pulse
        if self.s_map:
            d = _interp(pulse, [p[0] for p in self.s_map], [p[1] for p in self.s_map])
        elif pulse >= self.s_c:
            d = (pulse - self.s_c) / max(1e-9, self.s_max - self.s_c) * self.dmax
        else:
            d = (pulse - self.s_c) / max(1e-9, self.s_c - self.s_min) * self.dmax
        return max(-self.dmax, min(self.dmax, d))

    def speed_mps(self, pulse):
        if self.t_inv:
            pulse = 2 * self.t_n - pulse
        if pulse >= self.t_n:
            return max(0.0, _interp(pulse, self.t_us, self.t_v))
        # 中立より下: ESC はブレーキ → (中立を経由して) 後退。sim の ESC 模型が経由を再現するので負の速度で渡す
        return -self.v_rev * min(1.0, (self.t_n - pulse) / max(1e-9, self.t_n - self.t_min))
