from jetracer_common.actuator_model import ServoModel, EscModel, MotorModel


def test_servo_rate_limit_and_lag():
    s = ServoModel(tau_s=0.05, rate_limit_rad_s=6.0, delta_max_rad=0.47)
    d = s.step(1.0, 0.01)                 # 飽和 0.47、レート 6 rad/s → 0.06 が上限
    assert abs(d - 0.06) < 1e-9
    for _ in range(200):
        d = s.step(1.0, 0.01)
    assert abs(d - 0.47) < 1e-3


def test_esc_reverse_requires_neutral_gap():
    e = EscModel(deadband_mps=0.1, reverse_via_neutral_s=0.12, stop_thresh_mps=0.05)
    # 前進中に後退指令 → まずブレーキ
    tgt, brake, st = e.step(-0.5, v=1.0, dt=0.01)
    assert st == EscModel.BRAKE and brake and tgt == 0.0
    # 止まった → 中立待ち 120 ms (駆動ゼロ)
    n_wait = 0
    for _ in range(100):
        tgt, brake, st = e.step(-0.5, v=0.0, dt=0.01)
        if st == EscModel.REV:
            break
        assert tgt is None and not brake
        n_wait += 1
    assert 11 <= n_wait <= 13
    assert st == EscModel.REV and tgt == -0.5


def test_esc_deadband_coasts():
    e = EscModel(deadband_mps=0.15)
    tgt, brake, st = e.step(0.1, v=0.0, dt=0.01)
    assert tgt is None and not brake


def test_motor_force_falls_with_speed():
    m = MotorModel(force_n=8.0, v_free_mps=4.0)
    assert m.force_cap(0.0) == 8.0
    assert abs(m.force_cap(2.0) - 4.0) < 1e-9
    assert m.force_cap(5.0) == 0.0
