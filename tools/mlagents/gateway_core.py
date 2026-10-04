# -*- coding: utf-8 -*-
"""
mlagents_gateway の ROS に依存しない部分 (単体で試験できるように分けてある)。

- ActionShaper: ML-Agents の行動 [-1, 1]² → ActuatorCmd の (δ rad, v m/s)。
  cmd_shaper と同じ制限 (±δmax で飽和・舵のレート制限・加減速の制限) を 1/30 s ごとにかける。
  学習した方策が、実機の cmd_shaper を通したときと同じ指令の流れで学ぶため。
- Reward: StepInfo (報酬の材料) から 1 判断ぶんの報酬を組む。式は reward.yaml で変える。
  sim には報酬を持たせない (docs/lockstep.md の方針)。
"""
from dataclasses import dataclass, field


@dataclass
class ShaperLimits:
    delta_max_rad: float = 0.47
    v_max_mps: float = 3.0
    steer_rate_rad_s: float = 8.0      # cmd_shaper の steer_rate_limit_rad_s
    accel_mps2: float = 3.0            # cmd_shaper の accel_limit_mps2
    decel_mps2: float = 6.0            # cmd_shaper の decel_limit_mps2
    control_dt: float = 1.0 / 30.0     # /sim/step 1 回の長さ


class ActionShaper:
    def __init__(self, lim: ShaperLimits):
        self.lim = lim
        self.steer = 0.0
        self.v = 0.0

    def reset(self):
        self.steer = 0.0
        self.v = 0.0

    @staticmethod
    def _clip(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def target(self, a_steer, a_speed):
        """行動 → 目標の (δ, v)。速度は -1 = 停止 〜 +1 = v_max"""
        L = self.lim
        d = self._clip(float(a_steer), -1.0, 1.0) * L.delta_max_rad
        v = (self._clip(float(a_speed), -1.0, 1.0) + 1.0) * 0.5 * L.v_max_mps
        return d, v

    def step(self, a_steer, a_speed):
        """1/30 s ぶん制限をかけた (δ, v) を返す (cmd_shaper.tick と同じ順序)"""
        L = self.lim
        d_want, v_want = self.target(a_steer, a_speed)
        lim = L.steer_rate_rad_s * L.control_dt
        self.steer += self._clip(d_want - self.steer, -lim, lim)
        dv = v_want - self.v
        self.v += self._clip(dv, -L.decel_mps2 * L.control_dt, L.accel_mps2 * L.control_dt)
        if v_want == 0.0 and self.v < 0.05:
            self.v = 0.0
        return self.steer, self.v


@dataclass
class RewardWeights:
    progress: float = 1.0          # 参照線上で 1 m 進むごと (主項)
    cte_free_m: float = 0.10       # この横偏差までは罰しない
    cte: float = 5.0               # (|cte| - cte_free)² に掛ける
    wall_margin_m: float = 0.05    # 壁までの余裕がこれを割ったら罰
    wall: float = 2.0              # 割った量 [m] に掛ける
    steer_rate: float = 0.002      # 舵の変化率² × dt に掛ける (舵の振動を抑える。サーボを守る)
    time: float = 0.002            # 毎 step の小さな罰 (止まっていても得をしない)
    lap_bonus: float = 5.0         # 周回 1 回
    terminal: float = 5.0          # 衝突・コース外
    backward: float = 2.0          # 後ろに進んだ距離 [m] に追加で掛ける (逆走の抑止)

    @classmethod
    def from_dict(cls, d):
        w = cls()
        for k, v in (d or {}).items():
            if not hasattr(w, k):
                raise KeyError(f'reward.yaml: 不明な項目 {k}')
            setattr(w, k, float(v))
        return w


@dataclass
class StepSummary:
    """1 判断 (= 複数 step) を集計したもの"""
    reward: float = 0.0
    progress_m: float = 0.0
    cte_m: float = 0.0
    speed_mps: float = 0.0
    terminated: bool = False
    collision: bool = False
    off_track: bool = False
    lap_done: bool = False
    terms: dict = field(default_factory=dict)


def step_reward(info, w: RewardWeights, dt: float):
    """StepInfo 1 個 → (報酬, 内訳)。info は属性で値を持つもの (ROS の StepInfo か同じ名前の簡易オブジェクト)"""
    t = {}
    p = float(info.progress_m)
    t['progress'] = w.progress * p - (w.backward * -p if p < 0 else 0.0)
    over = max(0.0, abs(float(info.cte_m)) - w.cte_free_m)
    t['cte'] = -w.cte * over * over
    t['wall'] = -w.wall * max(0.0, w.wall_margin_m - float(info.min_wall_clear_m))
    t['steer_rate'] = -w.steer_rate * float(info.steer_rate_rad_s) ** 2 * dt
    t['time'] = -w.time
    t['lap'] = w.lap_bonus if info.lap_done else 0.0
    t['terminal'] = -w.terminal if info.terminated else 0.0
    return sum(t.values()), t


def summarize(infos, w: RewardWeights, dt: float):
    """1 判断ぶんの StepInfo の列を集計する。終端に達したらそこで止める"""
    s = StepSummary()
    for info in infos:
        r, terms = step_reward(info, w, dt)
        s.reward += r
        for k, v in terms.items():
            s.terms[k] = s.terms.get(k, 0.0) + v
        s.progress_m += float(info.progress_m)
        s.cte_m = float(info.cte_m)
        s.speed_mps = float(info.speed_mps)
        s.lap_done = s.lap_done or bool(info.lap_done)
        s.collision = s.collision or bool(info.collision)
        s.off_track = s.off_track or bool(info.off_track)
        if info.terminated:
            s.terminated = True
            break
    return s


def mean_imu(imus):
    """sensor_msgs/Imu の列 → [ax, ay, az, gx, gy, gz] の平均 (空なら None)"""
    if not imus:
        return None
    acc = [0.0] * 6
    for m in imus:
        la, av = m.linear_acceleration, m.angular_velocity
        for i, v in enumerate((la.x, la.y, la.z, av.x, av.y, av.z)):
            acc[i] += float(v)
    n = float(len(imus))
    return [v / n for v in acc]


# /mlagents/obs の並び (Unity の MinicarAgent.O と同じ)
OBS_FIELDS = ('seq', 'kind', 'reward', 'terminated', 'truncated', 'lap_done', 'progress_m', 'cte_m', 'speed_mps',
              'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'collision', 'off_track')
# /mlagents/action の並び
ACTION_FIELDS = ('seq', 'kind', 'steer', 'speed', 'seed', 'spawn')
