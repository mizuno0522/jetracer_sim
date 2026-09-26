# -*- coding: utf-8 -*-
"""
vehicle_profile.yaml の読み込み。車両の数値はここ (1 ファイル) にだけ書き、
vehicle_sim / cmd_shaper / jetracer_bridge / export_unity_course が同じものを読む。

  from jetracer_common.profile import load_profile, find_profile
  prof = load_profile(find_profile('jetracer_tt02'))
"""

import os

import yaml

REQUIRED = ('wheelbase_m', 'delta_max_rad', 'v_max_mps', 'drive', 'camera')


def find_profile(name_or_path):
    """名前 (jetracer_tt02) か絶対パスから yaml のパスを返す。"""
    if os.path.isfile(os.path.expanduser(name_or_path)):
        return os.path.expanduser(name_or_path)
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('minicar_sim'),
                            'config', 'vehicle_profile', f'{name_or_path}.yaml')
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    here = os.path.dirname(os.path.realpath(__file__))
    cand = os.path.normpath(os.path.join(here, '..', '..', 'minicar_sim', 'config',
                                         'vehicle_profile', f'{name_or_path}.yaml'))
    if os.path.isfile(cand):
        return cand
    raise FileNotFoundError(f'vehicle_profile が見つからない: {name_or_path}')


def load_profile(path):
    with open(path, 'r') as f:
        d = yaml.safe_load(f)
    prof = d['vehicle_profile'] if 'vehicle_profile' in d else d
    missing = [k for k in REQUIRED if k not in prof]
    if missing:
        raise KeyError(f'{path}: vehicle_profile に {missing} が無い')
    return prof


def vehicle_sim_overrides(prof):
    """vehicle_profile → vehicle_sim の ROS パラメータ名への写像。"""
    m = {
        'wheelbase_m': float(prof['wheelbase_m']),
        'max_steer_rad': float(prof['delta_max_rad']),
        'max_speed_mps': float(prof['v_max_mps']),
        'drivetrain': str(prof['drive']),
        'vehicle_profile_name': str(prof.get('name', 'unknown')),
    }
    if 'drive_split' in prof:
        m['drive_load_share'] = float(prof['drive_split'][0])
    if 'mass_kg' in prof:
        m['vehicle_mass_kg'] = float(prof['mass_kg'])
    if 'cg_height_m' in prof:
        m['cog_height_m'] = float(prof['cg_height_m'])
    if 'length_m' in prof:
        m['vehicle_length_m'] = float(prof['length_m'])
    if 'width_m' in prof:
        m['vehicle_half_width_m'] = float(prof['width_m']) / 2.0
    mo = prof.get('motor', {})
    if 'force_n' in mo:
        m['motor_force_n'] = float(mo['force_n'])
    if 'v_free_mps' in mo:
        m['motor_v_free_mps'] = float(mo['v_free_mps'])
    if 'accel_time_constant_s' in mo:
        m['accel_time_constant'] = float(mo['accel_time_constant_s'])
    sv = prof.get('servo', {})
    if 'tau_s' in sv:
        m['servo_tau_s'] = float(sv['tau_s'])
    if 'rate_limit_rad_s' in sv:
        m['servo_rate_limit_rad_s'] = float(sv['rate_limit_rad_s'])
    esc = prof.get('esc', {})
    if 'deadband_mps' in esc:
        m['esc_deadband_mps'] = float(esc['deadband_mps'])
    if 'reverse_via_neutral_ms' in esc:
        m['esc_reverse_via_neutral_s'] = float(esc['reverse_via_neutral_ms']) * 1e-3
    if 'brake_decel_mps2' in esc:
        m['esc_brake_decel_mps2'] = float(esc['brake_decel_mps2'])
    if 'coast_decel_mps2' in esc:
        m['coast_decel_mps2'] = float(esc['coast_decel_mps2'])
    lk = prof.get('drivetrain_lock', {})
    if 'windup_gain' in lk:
        m['windup_gain'] = float(lk['windup_gain'])
    if 'windup_drag' in lk:
        m['windup_drag'] = float(lk['windup_drag'])
    su = prof.get('suspension', {})
    if 'roll_per_ms2' in su:
        m['roll_per_ms2'] = float(su['roll_per_ms2'])
    if 'pitch_per_ms2' in su:
        m['pitch_per_ms2'] = float(su['pitch_per_ms2'])
    cam = prof['camera']
    m.update({
        'cam_width': int(cam['width']),
        'cam_height': int(cam['height']),
        'cam_fov_deg': float(cam['hfov_deg']),
        'cam_vfov_deg': float(cam.get('vfov_deg', 0.0) or 0.0),
        'cam_k1': float(cam.get('k1', 0.0) or 0.0),
        'cam_k2': float(cam.get('k2', 0.0) or 0.0),
        'cam_mount_height_m': float(cam['mount_height_m']),
        'cam_pitch_deg': float(cam['pitch_deg']),
        'cam_crop_top_frac': float(cam.get('crop_top_frac', 0.0)),
        'cam_rate_hz': float(cam.get('rate_hz', 15.0)),
    })
    return m
