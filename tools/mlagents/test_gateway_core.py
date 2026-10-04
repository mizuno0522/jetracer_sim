# -*- coding: utf-8 -*-
"""gateway_core と joy_teleop の割り当ての単体試験 (ROS 不要)。  python3 -m pytest tools/mlagents -q"""
import math
import os
import sys
import types
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
from gateway_core import ActionShaper, ShaperLimits, RewardWeights, step_reward, summarize, mean_imu  # noqa: E402


def info(**kw):
    d = dict(progress_m=0.0, cte_m=0.0, min_wall_clear_m=0.5, steer_rate_rad_s=0.0, lap_done=False,
             terminated=False, collision=False, off_track=False, speed_mps=0.0)
    d.update(kw)
    return NS(**d)


def test_shaper_limits_match_cmd_shaper():
    s = ActionShaper(ShaperLimits(delta_max_rad=0.47, v_max_mps=3.0))
    d, v = s.step(1.0, 1.0)
    assert math.isclose(d, 8.0 / 30.0, rel_tol=1e-6)        # 舵のレート制限 8 rad/s × 1/30 s
    assert math.isclose(v, 3.0 / 30.0, rel_tol=1e-6)        # 加速度 3 m/s² × 1/30 s
    for _ in range(100):
        d, v = s.step(1.0, 1.0)
    assert math.isclose(d, 0.47) and math.isclose(v, 3.0)   # 飽和
    for _ in range(100):
        d, v = s.step(0.0, -1.0)
    assert v == 0.0 and abs(d) < 1e-9                        # 停止・直進に戻る


def test_speed_action_maps_minus_one_to_stop():
    s = ActionShaper(ShaperLimits(v_max_mps=3.0))
    assert s.target(0.0, -1.0)[1] == 0.0
    assert math.isclose(s.target(0.0, 0.0)[1], 1.5)


def test_reward_terms():
    w = RewardWeights()
    r, t = step_reward(info(progress_m=0.1), w, 1 / 30)
    assert math.isclose(t['progress'], 0.1) and r < 0.1
    r_back, _ = step_reward(info(progress_m=-0.1), w, 1 / 30)
    assert r_back < -0.1 * (1 + w.backward) + 1e-9           # 逆走は余計に罰
    _, t = step_reward(info(cte_m=0.3), w, 1 / 30)
    assert math.isclose(t['cte'], -w.cte * 0.2 ** 2)
    _, t = step_reward(info(min_wall_clear_m=0.0), w, 1 / 30)
    assert math.isclose(t['wall'], -w.wall * w.wall_margin_m)
    r_term, _ = step_reward(info(terminated=True, collision=True), w, 1 / 30)
    assert r_term <= -w.terminal


def test_summarize_stops_at_terminal():
    w = RewardWeights()
    s = summarize([info(progress_m=0.1), info(progress_m=0.1, terminated=True, collision=True), info(progress_m=5.0)],
                  w, 1 / 30)
    assert s.terminated and s.collision
    assert math.isclose(s.progress_m, 0.2)                   # 終端の後ろは数えない


def test_reward_weights_reject_unknown_key():
    try:
        RewardWeights.from_dict({'progres': 1.0})
    except KeyError:
        return
    raise AssertionError('typo が通ってしまった')


def test_mean_imu():
    v3 = lambda x, y, z: NS(x=x, y=y, z=z)  # noqa: E731
    imus = [NS(linear_acceleration=v3(1, 0, 9.8), angular_velocity=v3(0, 0, 1)),
            NS(linear_acceleration=v3(3, 0, 9.8), angular_velocity=v3(0, 0, 3))]
    assert mean_imu(imus) == [2.0, 0.0, 9.8, 0.0, 0.0, 2.0]
    assert mean_imu([]) is None


def _import_joy_teleop():
    # rclpy などを空のモジュールで置き換えて read_input だけ試す
    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg'):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy.node'].Node = object
    q = sys.modules['rclpy.qos']
    q.QoSProfile = q.QoSReliabilityPolicy = q.QoSHistoryPolicy = object
    sys.modules['rclpy.time'].Time = object
    sys.modules['sensor_msgs.msg'].Joy = object
    sys.modules['std_msgs.msg'].Float64MultiArray = object
    sys.path.insert(0, os.path.join(HERE, '..', 'teleop'))
    import joy_teleop
    return joy_teleop


def test_joy_trigger_waits_for_rest():
    jt = _import_joy_teleop()
    jt._ARMED.clear()
    spec = {'type': 'trigger', 'index': 5, 'rest': 1.0, 'pressed': -1.0}
    assert jt.read_input(spec, NS(axes=[0.0] * 6, buttons=[])) == 0.0   # 触る前の 0.0 は「半分」ではない
    assert jt.read_input(spec, NS(axes=[1.0] * 6, buttons=[])) == 0.0   # 離した値を見た
    assert jt.read_input(spec, NS(axes=[0.0] * 6, buttons=[])) == 0.5
    assert jt.read_input(spec, NS(axes=[-1.0] * 6, buttons=[])) == 1.0


def test_joy_axis_deadzone_and_buttons():
    jt = _import_joy_teleop()
    spec = {'type': 'axis', 'index': 0, 'deadzone': 0.1}
    assert jt.read_input(spec, NS(axes=[0.05], buttons=[])) == 0.0
    assert math.isclose(jt.read_input(spec, NS(axes=[1.0], buttons=[])), 1.0)
    assert jt.read_any([{'type': 'button', 'index': 1}, spec], NS(axes=[0.0], buttons=[0, 1])) == 1.0


if __name__ == '__main__':
    n = 0
    for k, f in list(globals().items()):
        if k.startswith('test_') and callable(f):
            f(); n += 1
    print(f'{n} passed')
