import math
import numpy as np
import yaml
import os

from jetracer_common.imu_model import ImuModel, G

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), 'imu_sim_test.yaml')))['imu_sim']


def run(model, n=300, a=(0, 0, 0), w=(0, 0, 0), roll=0.0, pitch=0.0, v=0.0, surface=0):
    out = []
    for i in range(n):
        out += model.push(i * 0.01, a, w, roll, pitch, v, surface)
    return out


def test_static_reads_gravity_in_sensor_frame():
    cfg = dict(CFG)
    cfg['domain_rand'] = {'enable': False}
    m = ImuModel(cfg, seed=1)
    s = run(m)
    assert len(s) > 200
    acc = np.mean([x.accel for x in s[-100:]], axis=0)
    # 取付 rpy=[180,0,90]: センサ z は下向き → 静止では −g
    assert abs(acc[2] + G) < 0.05, acc
    assert abs(acc[0]) < 0.05 and abs(acc[1]) < 0.05
    assert abs(len(s) / 3.0 - 100.0) < 3.0   # 100 Hz


def test_pitch_leaks_into_forward_axis():
    cfg = dict(CFG)
    cfg['domain_rand'] = {'enable': False}
    m = ImuModel(cfg, seed=2)
    s = run(m, pitch=math.radians(5.0))
    acc = np.mean([x.accel for x in s[-100:]], axis=0)
    # センサ y = 車の前。鼻上げ 5° で +g sin5° = 0.855 m/s² が前軸に出る
    assert abs(acc[1] - G * math.sin(math.radians(5.0))) < 0.05, acc


def test_yaw_rate_on_sensor_z_is_negative_for_left_turn():
    cfg = dict(CFG)
    cfg['domain_rand'] = {'enable': False}
    m = ImuModel(cfg, seed=3)
    s = run(m, w=(0, 0, 1.0), a=(0, 1.3, 0), v=1.3)
    gyr = np.mean([x.gyro for x in s[-100:]], axis=0)
    # センサ z は下向きなので左旋回 (+wz) は −1 rad/s に写る
    assert abs(gyr[2] + 1.0) < 0.02, gyr


def test_turn_on_bias_changes_per_episode():
    m = ImuModel(CFG, seed=10)
    b1 = m.b_acc0.copy()
    m.new_episode(11)
    assert np.linalg.norm(m.b_acc0 - b1) > 1e-4


def test_saturation_flag():
    cfg = dict(CFG)
    cfg['domain_rand'] = {'enable': False}
    m = ImuModel(cfg, seed=4)
    s = run(m, a=(60.0, 0, 0))
    assert any(x.saturated for x in s[-50:])
    assert max(abs(x.accel[1]) for x in s) <= m.acc_range + 1e-6
