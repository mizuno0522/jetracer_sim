# -*- coding: utf-8 -*-
"""
6 軸 IMU (SY-151 / MPU-6500 互換) のセンサモデル。ROS 非依存。

入力は vehicle_sim の剛体状態 (100 Hz)。内部は 1 kHz で回し、
  1. 重力の投影 (roll / pitch)           … 比力 f = a_kin − g_body
  2. 取付点のてこ腕  a = a_cg + ω×(ω×r) + α×r
  3. 振動 (モータ高調波 + 路面、車速比例)
  4. DLPF (2 次 Butterworth) と群遅延
  5. 100 Hz へ間引き (I2C 読み出しの遅延・ジッタ・取りこぼし)
  6. ターンオンバイアス + ランダムウォーク + 温度ドリフト
  7. スケール誤差・軸ずれ、白色雑音
  8. 飽和と 16 bit 量子化
の順に「実機の /imu に必ず含まれる汚れ」を足す。パラメータは imu_sim.yaml (要実測の欄あり)。

★ 折り返し: 出力 100 Hz のナイキストは 50 Hz。1 kHz で振動を足してから DLPF → 間引き
  するので、DLPF を広く設定すれば実機と同じ折り返しが出る (物理の 100 Hz に雑音を
  足す実装ではこの現象が出ない)。

座標: 車体 = x 前・y 左・z 上 (base_link)。センサ軸は mount.rpy_deg で回す
  (SY-151: 水平・部品面下・x が車の左 → rpy=[180,0,90] → センサ x=左, y=前, z=下)。
"""

import math

import numpy as np

try:
    from scipy.signal import butter
except Exception:  # scipy が無い環境 (実機側) では DLPF を 1 次で近似
    butter = None

G = 9.80665


def rpy_to_matrix(roll, pitch, yaw):
    """ZYX オイラー (rad) → 回転行列 R (body→world 相当)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class _Biquad:
    """2 次 IIR (軸ごとに状態を持つ、3 軸ベクトル用)。"""

    def __init__(self, b, a):
        self.b = np.asarray(b, float)
        self.a = np.asarray(a, float)
        self.x1 = np.zeros(3)
        self.x2 = np.zeros(3)
        self.y1 = np.zeros(3)
        self.y2 = np.zeros(3)

    def step(self, x):
        y = (self.b[0] * x + self.b[1] * self.x1 + self.b[2] * self.x2
             - self.a[1] * self.y1 - self.a[2] * self.y2) / self.a[0]
        self.x2, self.x1 = self.x1, np.array(x, float)
        self.y2, self.y1 = self.y1, y
        return y

    def reset(self, x0):
        x0 = np.asarray(x0, float)
        self.x1 = self.x2 = x0.copy()
        self.y1 = self.y2 = x0.copy()


def _lowpass(bw_hz, fs):
    if bw_hz <= 0 or bw_hz >= fs / 2:
        return _Biquad([1, 0, 0], [1, 0, 0])
    if butter is not None:
        b, a = butter(2, bw_hz / (fs / 2.0))
        return _Biquad(b, a)
    # 1 次の後方差分近似
    k = 1.0 - math.exp(-2 * math.pi * bw_hz / fs)
    return _Biquad([k, 0, 0], [1, -(1 - k), 0])


class ImuSample:
    __slots__ = ('t', 'accel', 'gyro', 'temp_c', 'saturated')

    def __init__(self, t, accel, gyro, temp_c, saturated):
        self.t = t
        self.accel = accel
        self.gyro = gyro
        self.temp_c = temp_c
        self.saturated = saturated


class ImuModel:

    SURFACE_NAMES = ('carpet', 'tunnel', 'light', 'turf', 'slip', 'rough', 'slope')

    def __init__(self, cfg, seed=0):
        """cfg は imu_sim.yaml の `imu_sim:` 直下の dict。"""
        self.cfg = cfg
        rate = cfg['rate']
        self.fs = float(rate['internal_hz'])
        self.fo = float(rate['output_hz'])
        self.dt = 1.0 / self.fs
        self.r = np.array(cfg['mount']['xyz'], float)
        self.rpy_nom = np.radians(np.array(cfg['mount']['rpy_deg'], float))
        rng_ = cfg['range']
        self.acc_range = float(rng_['accel_g']) * G
        self.gyr_range = math.radians(float(rng_['gyro_dps']))
        self.bits = int(rng_.get('bits', 16))
        self.saturate = bool(rng_.get('saturate', True))
        self.acc_lsb = self.acc_range / (2 ** (self.bits - 1))
        self.gyr_lsb = self.gyr_range / (2 ** (self.bits - 1))
        d = cfg['dlpf']
        self.acc_lpf = _lowpass(float(d['accel_bw_hz']), self.fs)
        self.gyr_lpf = _lowpass(float(d['gyro_bw_hz']), self.fs)
        # 角加速度 α は差分のあと 30 Hz 程度の一次遅れ (実機の α もその程度しか観測できない)
        self.alpha_lpf = _lowpass(float(cfg.get('alpha_bw_hz', 30.0)), self.fs)
        n = cfg['noise']
        self.acc_nd = float(n['accel_nd'])
        self.gyr_nd = float(n['gyro_nd'])
        self.acc_rw = float(n['accel_rw'])
        self.gyr_rw = float(n['gyro_rw'])
        self.tob = cfg['turn_on_bias']
        self.scale_cfg = cfg['scale']
        self.temp_cfg = cfg['temp']
        self.vib = cfg['vibration']
        lat = cfg['latency']
        self.lat_base = float(lat['base_ms']) * 1e-3
        self.lat_jit = float(lat['jitter_ms']) * 1e-3
        self.drop_p = float(lat.get('drop_prob', 0.0))
        dr = cfg.get('domain_rand', {})
        self.dr_enable = bool(dr.get('enable', True))
        self.dr_scale = float(dr.get('scale', 1.0)) if self.dr_enable else 0.0
        # 遅延バッファ (1 kHz の濾波後の値)。最大 50 ms
        self._buf_n = int(0.05 * self.fs) + 2
        self._buf_a = np.zeros((self._buf_n, 3))
        self._buf_g = np.zeros((self._buf_n, 3))
        self._buf_t = np.full(self._buf_n, -1.0)
        self._buf_i = 0
        self._prev = None
        self._w_prev = np.zeros(3)
        self._t_int = None
        self._t_next_out = None
        self._t0 = None
        self._phase = np.zeros(8)
        self._surf_state = np.zeros(3)
        self.new_episode(seed)

    # ------------------------------------------------------------------
    def new_episode(self, seed):
        """エピソード毎に引き直すもの: ターンオンバイアス・取付角誤差・スケール誤差・温度初期値。"""
        self.rng = np.random.default_rng(int(seed))
        k = self.dr_scale
        self.b_acc0 = self.rng.normal(0, np.array(self.tob['accel_sigma'], float)) * k
        self.b_gyr0 = self.rng.normal(0, np.array(self.tob['gyro_sigma'], float)) * k
        self.b_acc_rw = np.zeros(3)
        self.b_gyr_rw = np.zeros(3)
        rpy_err = np.radians(self.rng.normal(0, np.array(self.cfg['mount']['rpy_err_deg'], float))) * k
        self.R_bs = rpy_to_matrix(*(self.rpy_nom + rpy_err))     # sensor→body
        s_acc = 1.0 + self.rng.normal(0, float(self.scale_cfg['accel_err_sigma']), 3) * k
        s_gyr = 1.0 + self.rng.normal(0, float(self.scale_cfg['gyro_err_sigma']), 3) * k
        mis = np.radians(self.rng.normal(0, float(self.scale_cfg['misalign_deg_sigma']), 3)) * k
        Rm = rpy_to_matrix(*mis)
        self.M_acc = Rm @ np.diag(s_acc)
        self.M_gyr = Rm @ np.diag(s_gyr)
        self.temp_start = float(self.temp_cfg['start_c']) + self.rng.normal(0, 2.0) * k
        self._prev = None
        self._t_int = None
        self._t_next_out = None
        self._t0 = None
        self._buf_t[:] = -1.0
        self.acc_lpf.reset(np.zeros(3))
        self.gyr_lpf.reset(np.zeros(3))
        self.alpha_lpf.reset(np.zeros(3))
        self._surf_state[:] = 0.0

    # ------------------------------------------------------------------
    def temperature(self, t):
        tc = self.temp_cfg
        return self.temp_start + float(tc['rise_c']) * (1.0 - math.exp(-t / max(1e-3, float(tc['tau_s']))))

    def _vibration(self, v, surface, rough_frac, dt):
        """1 kHz 1 サンプルぶんの振動加速度 (車体座標) と角速度の揺れ。"""
        vib = self.vib
        a = np.zeros(3)
        speed = abs(v)
        # モータ高調波 (車速に比例した基本周波数)
        f0 = float(vib['motor_hz_per_mps']) * speed
        if f0 > 0.5:
            for i, (h, amp) in enumerate(zip(vib['harmonics'], vib['amp_ms2'])):
                self._phase[i] = (self._phase[i] + 2 * math.pi * f0 * h * dt) % (2 * math.pi)
                s = amp * math.sin(self._phase[i]) * min(1.0, speed / 1.0)
                a += s * np.array([0.3, 0.3, 1.0])
        gm = float(vib.get('gear_mesh_hz', 0.0))
        if gm > 0.0:
            self._phase[6] = (self._phase[6] + 2 * math.pi * gm * speed * dt) % (2 * math.pi)
            a[2] += 0.3 * math.sin(self._phase[6])
        # 路面 (有色雑音: 1 次 LPF を通した白色雑音を、rms が coef×速度 になるよう正規化)
        name = self.SURFACE_NAMES[surface] if 0 <= surface < len(self.SURFACE_NAMES) else 'carpet'
        surf = vib['surface'].get(name, vib['surface']['carpet'])
        if name in ('tunnel', 'light'):
            surf = vib['surface']['carpet']
        rms = float(surf['rms_per_mps']) * speed
        fc = float(surf['fc_hz'])
        if rms > 0.0:
            k = 1.0 - math.exp(-2 * math.pi * fc / self.fs)
            # 1 次 LPF 後の白色雑音の分散 = σ² k / (2 − k)
            sigma = rms / math.sqrt(k / (2.0 - k))
            w = self.rng.normal(0.0, sigma, 3)
            self._surf_state += k * (w - self._surf_state)
            a += self._surf_state * np.array([0.5, 0.5, 1.0])
        # 角速度の揺れ (路面振動に比例した小さなピッチ/ロールの揺れ)
        g_vib = a[[1, 0, 2]] * float(vib.get('gyro_per_ms2', 0.02)) * np.array([1.0, 1.0, 0.3])
        return a, g_vib

    # ------------------------------------------------------------------
    def push(self, t, a_kin, w, roll, pitch, v, surface=0, rough_frac=0.0):
        """
        物理 1 ステップ (100 Hz) の剛体状態を入れ、この間に生成された出力サンプルを返す。

        t       : sim 時刻 [s]
        a_kin   : 重心の運動学的加速度 (車体座標、重力を含まない) [m/s²]
        w       : 角速度 (車体座標) [rad/s]
        roll    : 右下がり正 [rad]、pitch: 鼻上げ正 [rad]
        """
        a_kin = np.asarray(a_kin, float)
        w = np.asarray(w, float)
        out = []
        if self._prev is None:
            self._prev = (t, a_kin, w, roll, pitch)
            self._t0 = t
            self._n_int = 0                 # 内部サンプルの通し番号 (t = t0 + n / fs)
            self._n_out = 1                 # 次に出す出力サンプルの番号 (t = t0 + n / fo)
            self._t_int = t
            self._w_prev = w.copy()
            return out
        t0, a0, w0, r0, p0 = self._prev
        span = t - t0
        if span <= 0.0:
            return out
        # 100 Hz の隣接 2 点を線形補間しながら 1 kHz で進める (時刻は整数インデックスから作る)
        while self._t0 + (self._n_int + 1) / self.fs <= t + 1e-9:
            self._n_int += 1
            self._t_int = self._t0 + self._n_int / self.fs
            f = min(1.0, max(0.0, (self._t_int - t0) / span))
            a_c = a0 + (a_kin - a0) * f
            w_c = w0 + (w - w0) * f
            roll_c = r0 + (roll - r0) * f
            pitch_c = p0 + (pitch - p0) * f
            # 角加速度: 差分 → 30 Hz の一次遅れ
            alpha_raw = (w_c - self._w_prev) / self.dt
            self._w_prev = w_c
            alpha = self.alpha_lpf.step(alpha_raw)
            # 1. 重力の投影 (pitch は鼻上げ正 → ROS 右手系の y 軸回りでは −pitch)
            R = rpy_to_matrix(roll_c, -pitch_c, 0.0)
            g_body = R.T @ np.array([0.0, 0.0, -G])
            f_cg = a_c - g_body
            # 2. てこ腕
            f_mount = f_cg + np.cross(w_c, np.cross(w_c, self.r)) + np.cross(alpha, self.r)
            # 3. 振動
            a_vib, g_vib = self._vibration(v, surface, rough_frac, self.dt)
            f_body = f_mount + a_vib
            w_body = w_c + g_vib
            # センサ軸へ
            f_s = self.R_bs.T @ f_body
            w_s = self.R_bs.T @ w_body
            # 4. DLPF
            f_lp = self.acc_lpf.step(f_s)
            w_lp = self.gyr_lpf.step(w_s)
            i = self._buf_i % self._buf_n
            self._buf_a[i] = f_lp
            self._buf_g[i] = w_lp
            self._buf_t[i] = self._t_int
            self._buf_i += 1
            # 5. 出力 (100 Hz)
            while self._t0 + self._n_out / self.fo <= self._t_int + 1e-9:
                t_out = self._t0 + self._n_out / self.fo
                self._n_out += 1
                if self.drop_p > 0.0 and self.rng.random() < self.drop_p:
                    continue
                delay = self.lat_base + abs(self.rng.normal(0.0, self.lat_jit))
                t_read = t_out - delay
                k = int(round((self._t_int - t_read) * self.fs))
                k = max(0, min(self._buf_n - 1, k))
                j = (self._buf_i - 1 - k) % self._buf_n
                if self._buf_t[j] < 0:
                    j = i
                out.append(self._finish(t_out, self._buf_a[j].copy(), self._buf_g[j].copy()))
        self._prev = (t, a_kin, w, roll, pitch)
        return out

    def _finish(self, t_out, f, w):
        """6〜8: バイアス・温度・スケール・雑音・飽和・量子化。"""
        dto = 1.0 / self.fo
        self.b_acc_rw += self.rng.normal(0.0, self.acc_rw * math.sqrt(dto), 3)
        self.b_gyr_rw += self.rng.normal(0.0, self.gyr_rw * math.sqrt(dto), 3)
        temp = self.temperature(t_out - self._t0)
        dT = temp - float(self.temp_cfg['start_c'])
        b_acc = self.b_acc0 + self.b_acc_rw + float(self.temp_cfg['accel_coef']) * dT
        b_gyr = self.b_gyr0 + self.b_gyr_rw + float(self.temp_cfg['gyro_coef']) * dT
        bw_a = float(self.cfg['dlpf']['accel_bw_hz'])
        bw_g = float(self.cfg['dlpf']['gyro_bw_hz'])
        f = self.M_acc @ f + b_acc + self.rng.normal(0.0, self.acc_nd * math.sqrt(bw_a), 3)
        w = self.M_gyr @ w + b_gyr + self.rng.normal(0.0, self.gyr_nd * math.sqrt(bw_g), 3)
        sat = False
        if self.saturate:
            if np.any(np.abs(f) > self.acc_range) or np.any(np.abs(w) > self.gyr_range):
                sat = True
            f = np.clip(f, -self.acc_range, self.acc_range)
            w = np.clip(w, -self.gyr_range, self.gyr_range)
        f = np.round(f / self.acc_lsb) * self.acc_lsb
        w = np.round(w / self.gyr_lsb) * self.gyr_lsb
        return ImuSample(t_out, f, w, temp, sat)

    # ------------------------------------------------------------------
    def covariances(self):
        """sensor_msgs/Imu の covariance 対角 (白色雑音の分散。Allan の実測値で置き換える)。"""
        bw_a = float(self.cfg['dlpf']['accel_bw_hz'])
        bw_g = float(self.cfg['dlpf']['gyro_bw_hz'])
        va = (self.acc_nd ** 2) * bw_a + float(np.mean(np.array(self.tob['accel_sigma']) ** 2))
        vg = (self.gyr_nd ** 2) * bw_g + float(np.mean(np.array(self.tob['gyro_sigma']) ** 2))
        return va, vg
