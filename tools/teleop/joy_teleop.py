#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ゲームパッド (Nintendo Switch Pro Controller など) で運転する。/joy (ROS の joy パッケージ) を読み、
割り当て (joy_map.yaml) を解いて 2 つの形で出す。割り当てを書くのはここ 1 か所だけ。

  /teleop/cmd    std_msgs/Float64MultiArray [steer -1〜1 (左正), throttle -1〜1 (負はブレーキ), 記録ボタン回数, 非常停止]
                 → Unity の MinicarAgent (ML-Agents の人の運転・.demo 記録) が読む
  /actuator_cmd  minicar_msgs/ActuatorCmd 30 Hz (publish_actuator:=true のときだけ)
                 → realtime の sim (または実機ブリッジ) を直接運転する。cmd_shaper と同じ制限をかける。
                   ★vehicle_stack (cmd_shaper) と同時に上げない (/actuator_cmd を取り合う)

使い方:
  sudo apt install ros-humble-joy           # 初回だけ
  ros2 run joy game_controller_node         # プロコンを USB か Bluetooth でつないでから
  python3 tools/teleop/joy_teleop.py --probe          # どのスティック・ボタンが何番か調べる (動かしたものを表示)
  python3 tools/teleop/joy_teleop.py --ros-args -p map_file:=tools/teleop/joy_map.yaml [-p publish_actuator:=true]
"""
import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
_ARMED = set()   # 離した値を一度見たトリガの軸番号


def read_input(spec, joy):
    """割り当て 1 項目 → 0〜1 (trigger・button) または -1〜1 (axis)"""
    if spec is None:
        return 0.0
    kind = spec.get('type', 'axis')
    i = int(spec['index'])
    if kind == 'button':
        return 1.0 if i < len(joy.buttons) and joy.buttons[i] else 0.0
    if i >= len(joy.axes):
        return 0.0
    x = float(joy.axes[i])
    if kind == 'trigger':
        # 離した値 rest と押し切った値 pressed の間を 0〜1 に写す (ドライバで向きが違うので両方書く)
        rest, pressed = float(spec.get('rest', 1.0)), float(spec.get('pressed', -1.0))
        if abs(pressed - rest) < 1e-6:
            return 0.0
        # 一度も離した値を見ていないトリガは 0 とする。ドライバによっては触るまで 0.0 を出すので、
        # rest=1.0 の割り当てだと「半分踏んでいる」と読んで発進してしまう
        if i not in _ARMED:
            if abs(x - rest) > 0.1:
                return 0.0
            _ARMED.add(i)
        return max(0.0, min(1.0, (x - rest) / (pressed - rest)))
    dz = float(spec.get('deadzone', 0.0))
    if abs(x) < dz:
        return 0.0
    x = math.copysign((abs(x) - dz) / (1.0 - dz), x) if dz > 0 else x
    return max(-1.0, min(1.0, x * float(spec.get('scale', 1.0))))


def read_any(specs, joy):
    """同じ役割に複数を割り当てたとき (ZR とA の両方でアクセル等) は大きい方"""
    if specs is None:
        return 0.0
    if isinstance(specs, dict):
        specs = [specs]
    return max(read_input(s, joy) for s in specs)


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.declare_parameter('map_file', os.path.join(HERE, 'joy_map.yaml'))
        self.declare_parameter('publish_actuator', False)
        self.declare_parameter('vehicle_profile_file', 'jetracer_tt02')
        self.declare_parameter('v_max_override_mps', 0.0)
        self.declare_parameter('rate_hz', 30.0)
        with open(os.path.expanduser(self.get_parameter('map_file').value)) as f:
            self.map = yaml.safe_load(f)['joy_map']
        self.joy = None
        self.rec_count = 0
        self.prev_rec = False
        self.estop = False
        self.prev_estop_btn = False
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/teleop/cmd', 10)
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.actuator = bool(self.get_parameter('publish_actuator').value)
        if self.actuator:
            from minicar_msgs.msg import ActuatorCmd
            from jetracer_common.profile import find_profile, load_profile
            sys.path.insert(0, os.path.join(HERE, '..', 'mlagents'))
            from gateway_core import ActionShaper, ShaperLimits
            self.ActuatorCmd = ActuatorCmd
            prof = load_profile(find_profile(self.get_parameter('vehicle_profile_file').value))
            vo = float(self.get_parameter('v_max_override_mps').value)
            rate = float(self.get_parameter('rate_hz').value)
            self.shaper = ActionShaper(ShaperLimits(delta_max_rad=float(prof['delta_max_rad']),
                                                    v_max_mps=vo if vo > 0 else float(prof['v_max_mps']),
                                                    control_dt=1.0 / rate))
            qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.pub_act = self.create_publisher(ActuatorCmd, '/actuator_cmd', qos)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().info(f"joy_teleop: /joy → /teleop/cmd{' ＋ /actuator_cmd' if self.actuator else ''}")

    def cb_joy(self, msg):
        self.joy = msg
        rec = read_any(self.map.get('record'), msg) > 0.5
        if rec and not self.prev_rec:
            self.rec_count += 1
            self.get_logger().info(f'記録ボタン ({self.rec_count})')
        self.prev_rec = rec
        es = read_any(self.map.get('estop'), msg) > 0.5
        if es and not self.prev_estop_btn:
            self.estop = not self.estop
            self.get_logger().warn('非常停止 ON' if self.estop else '非常停止 OFF')
        self.prev_estop_btn = es

    def tick(self):
        if self.joy is None:
            return
        age = (self.get_clock().now() - Time.from_msg(self.joy.header.stamp)).nanoseconds * 1e-9 \
            if self.joy.header.stamp.sec else 0.0
        stale = age > 0.5
        steer = 0.0 if stale else read_any(self.map.get('steer'), self.joy)
        thr = 0.0 if stale else read_any(self.map.get('throttle'), self.joy)
        brk = 0.0 if stale else read_any(self.map.get('brake'), self.joy)
        throttle = max(-1.0, min(1.0, thr - brk))
        m = Float64MultiArray()
        m.data = [float(steer), float(throttle), float(self.rec_count), 1.0 if (self.estop or stale) else 0.0]
        self.pub_cmd.publish(m)
        if self.actuator:
            a_speed = (-1.0 if (self.estop or stale) else max(0.0, throttle) * 2.0 - 1.0)
            d, v = self.shaper.step(steer, a_speed)
            c = self.ActuatorCmd()
            c.header.stamp = self.get_clock().now().to_msg()
            c.header.frame_id = 'base_link'
            c.steer_rad, c.speed_mps = float(d), float(v)
            c.mode = self.ActuatorCmd.MODE_ESTOP if self.estop else self.ActuatorCmd.MODE_RUN
            self.pub_act.publish(c)


class JoyProbe(Node):
    """動かした軸・押したボタンの番号と値を表示する (joy_map.yaml を書くため)"""
    def __init__(self):
        super().__init__('joy_probe')
        self.prev = None
        self.create_subscription(Joy, '/joy', self.cb, 10)
        self.get_logger().info('/joy を待っている。スティック・トリガ・ボタンを 1 つずつ動かしてください (Ctrl-C で終了)')

    def cb(self, msg):
        if self.prev is None:
            self.get_logger().info(f'軸 {len(msg.axes)} 本・ボタン {len(msg.buttons)} 個。離した状態の軸: '
                                   + ', '.join(f'{i}:{v:+.2f}' for i, v in enumerate(msg.axes)))
        else:
            for i, v in enumerate(msg.axes):
                if i < len(self.prev.axes) and abs(v - self.prev.axes[i]) > 0.25:
                    self.get_logger().info(f'axis {i}: {v:+.2f}')
            for i, b in enumerate(msg.buttons):
                if i < len(self.prev.buttons) and b != self.prev.buttons[i] and b:
                    self.get_logger().info(f'button {i}')
        self.prev = msg


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument('--probe', action='store_true')
    known, rest = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + rest)
    node = JoyProbe() if known.probe else JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
