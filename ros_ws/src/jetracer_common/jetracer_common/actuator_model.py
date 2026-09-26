# -*- coding: utf-8 -*-
"""
アクチュエータ模型 (TT-02 ＋ JetRacer 標準 BOM)。sim (vehicle_sim) と実機ブリッジで共用。

  ServoModel  一次遅れ τ ＋ レート制限。IMU に「横加速度の立ち上がりが指令より遅れる」形で出る
  EscModel    TBLE-02S: 前進 / ブレーキ帯 / 後退は中立を 120 ms 経由。中立を飛ばすとブレーキになる
  MotorModel  DC モータの電流上限 → 駆動力の上限を車速依存に (高速域で加速度が落ちる)

IMU を「指令」からではなく「実際に起きたこと」から合成するための鎖の 2 番目。
数値はすべて vehicle_profile.yaml から入れる (ここには既定値だけ)。
"""

import math


class ServoModel:
    """操舵サーボ: 一次遅れ + レート制限 + 飽和。"""

    def __init__(self, tau_s=0.06, rate_limit_rad_s=6.0, delta_max_rad=0.47):
        self.tau = max(1e-3, float(tau_s))
        self.rate = max(1e-3, float(rate_limit_rad_s))
        self.dmax = float(delta_max_rad)
        self.delta = 0.0

    def reset(self, delta=0.0):
        self.delta = float(delta)

    def step(self, cmd_rad, dt):
        cmd = max(-self.dmax, min(self.dmax, float(cmd_rad)))
        want = (cmd - self.delta) * (dt / self.tau)
        lim = self.rate * dt
        self.delta += max(-lim, min(lim, want))
        return self.delta


class EscModel:
    """
    TBLE-02S (前進/ブレーキ/後退) の状態機械。

    入力は目標車速 [m/s] (符号つき) と実車速。出力は
      (drive_target_v, brake, state)
    drive_target_v が None のときは駆動力ゼロ (惰行)。brake=True のときは減速度上限つきで制動。

    - |cmd| < deadband_mps         → 惰行 (不感帯)
    - 前進中に負の指令            → まずブレーキ。ほぼ止まったら中立 reverse_via_neutral_s の後に後退
    - 後退中に正の指令            → 前進 (即時)
    """
    FWD, BRAKE, NEUTRAL_WAIT, REV = 'FWD', 'BRAKE', 'NEUTRAL_WAIT', 'REV'

    def __init__(self, deadband_mps=0.15, reverse_via_neutral_s=0.12,
                 brake_decel_max=4.0, stop_thresh_mps=0.08):
        self.deadband = float(deadband_mps)
        self.t_neutral = float(reverse_via_neutral_s)
        self.brake_decel = float(brake_decel_max)
        self.stop_th = float(stop_thresh_mps)
        self.state = self.FWD
        self._timer = 0.0

    def reset(self):
        self.state = self.FWD
        self._timer = 0.0

    def step(self, cmd_v, v, dt):
        if abs(cmd_v) < self.deadband:
            # 不感帯。後退中の中立は後退ロックを解く
            if self.state in (self.REV, self.BRAKE):
                self.state = self.FWD if abs(v) < self.stop_th else self.state
            if self.state == self.NEUTRAL_WAIT:
                self._timer += dt
            return None, False, self.state

        if cmd_v > 0.0:
            self.state = self.FWD
            return cmd_v, False, self.state

        # cmd_v < 0: 後退したい
        if self.state == self.FWD:
            self.state = self.BRAKE if v > self.stop_th else self.NEUTRAL_WAIT
            self._timer = 0.0
        if self.state == self.BRAKE:
            if v > self.stop_th:
                return 0.0, True, self.state
            self.state = self.NEUTRAL_WAIT
            self._timer = 0.0
        if self.state == self.NEUTRAL_WAIT:
            self._timer += dt
            if self._timer < self.t_neutral:
                return None, False, self.state       # ★ 空白 120 ms
            self.state = self.REV
        return cmd_v, False, self.state


class MotorModel:
    """駆動力の上限 F(v) = F0 × max(0, 1 − |v| / v_free)。"""

    def __init__(self, force_n=8.0, v_free_mps=4.5):
        self.f0 = float(force_n)
        self.v_free = max(0.1, float(v_free_mps))

    def force_cap(self, v):
        return self.f0 * max(0.0, 1.0 - abs(v) / self.v_free)
