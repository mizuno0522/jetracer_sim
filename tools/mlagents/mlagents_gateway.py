#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ML-Agents (Unity) と vehicle_sim (lockstep) の間の中継。物理は vehicle_sim のまま (設計 P3)。

  Unity MinicarAgent ──/mlagents/action──▶ このノード ──/sim/reset, /sim/step──▶ vehicle_sim (sim_mode:=lockstep)
                     ◀──/mlagents/obs────                ◀── StepInfo・IMU ─────

- 行動 [舵, 速度] (-1〜1) を cmd_shaper と同じ制限で ActuatorCmd にし、1 判断につき /sim/step を
  decision_steps 回 (既定 2 = 1/15 s、カメラ 15 Hz に合わせる) 呼ぶ
- 報酬は StepInfo から組む (gateway_core.step_reward、重みは reward.yaml)。sim は報酬を持たない
- 画像は Unity が自分で持っている (配信中のセンサ画像と同じもの) ので、ここでは IMU と報酬だけ返す

起動 (sim PC。sim_host は lockstep・Unity は -mlagents 付き。docs/mlagents.md):
  source scripts/sim_env.sh
  python3 tools/mlagents/mlagents_gateway.py --ros-args -p reward_file:=tools/mlagents/config/reward.yaml
"""
import os
import queue
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    import jetracer_common.rclpy_lean  # noqa: F401  rclpy の CPU 対策 (あれば)
except ImportError:
    pass
import yaml

from minicar_msgs.msg import ActuatorCmd
from minicar_sim_msgs.srv import Reset, Step
from jetracer_common.profile import find_profile, load_profile

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from gateway_core import (ActionShaper, ShaperLimits, RewardWeights, summarize, mean_imu,  # noqa: E402
                          OBS_FIELDS)


class MlAgentsGateway(Node):
    def __init__(self):
        super().__init__('mlagents_gateway')
        self.declare_parameter('vehicle_profile_file', 'jetracer_tt02')
        self.declare_parameter('v_max_override_mps', 0.0)
        self.declare_parameter('decision_steps', 2)
        self.declare_parameter('reward_file', '')
        self.declare_parameter('seed_base', 1000)
        self.declare_parameter('service_timeout_s', 5.0)
        prof = load_profile(find_profile(self.get_parameter('vehicle_profile_file').value))
        vo = float(self.get_parameter('v_max_override_mps').value)
        cl = prof.get('cmd_limits', {})        # 実車プロファイルの指令制限 (無ければ cmd_shaper の既定)
        self.lim = ShaperLimits(delta_max_rad=float(prof['delta_max_rad']),
                                v_max_mps=vo if vo > 0 else float(prof['v_max_mps']),
                                steer_rate_rad_s=float(cl.get('steer_rate_rad_s', ShaperLimits.steer_rate_rad_s)),
                                accel_mps2=float(cl.get('accel_mps2', ShaperLimits.accel_mps2)),
                                decel_mps2=float(cl.get('decel_mps2', ShaperLimits.decel_mps2)))
        self.shaper = ActionShaper(self.lim)
        self.n_steps = max(1, int(self.get_parameter('decision_steps').value))
        self.seed_base = int(self.get_parameter('seed_base').value)
        self.timeout = float(self.get_parameter('service_timeout_s').value)
        rf = str(self.get_parameter('reward_file').value)
        wd = {}
        if rf:
            with open(os.path.expanduser(rf)) as f:
                wd = (yaml.safe_load(f) or {}).get('reward', {})
        self.w = RewardWeights.from_dict(wd)

        cg = MutuallyExclusiveCallbackGroup()
        self.cli_reset = self.create_client(Reset, '/sim/reset', callback_group=cg)
        self.cli_step = self.create_client(Step, '/sim/step', callback_group=cg)
        self.pub = self.create_publisher(Float64MultiArray, '/mlagents/obs', 10)
        self.create_subscription(Float64MultiArray, '/mlagents/action', self.cb_action, 10)
        self.q = queue.Queue()
        self.last_seq = None
        self.last_reply = None
        self.ep = {'ret': 0.0, 'prog': 0.0, 'laps': 0, 'n': 0, 't0': time.monotonic()}
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f"mlagents_gateway: δmax={self.lim.delta_max_rad:.2f} v_max={self.lim.v_max_mps:.2f} "
            f"1 判断 = /sim/step × {self.n_steps}  報酬={self.w}")

    def cb_action(self, msg):
        self.q.put(list(msg.data))

    # ------------------------------------------------------------------
    def run(self):
        for c, name in ((self.cli_reset, '/sim/reset'), (self.cli_step, '/sim/step')):
            while rclpy.ok() and not c.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'{name} を待っている (sim_host.launch.py sim_mode:=lockstep で起動しているか)')
        self.get_logger().info('vehicle_sim (lockstep) に接続した。Unity の要求を待つ')
        while rclpy.ok():
            try:
                d = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            if len(d) < 6:
                continue
            seq = d[0]
            if self.last_seq is not None and abs(seq - self.last_seq) < 0.5:
                # Unity の送り直し: 同じ要求をもう一度実行しない (sim が 2 回進んでしまう)。前の返事を出し直す
                if self.last_reply is not None:
                    self.pub.publish(self.last_reply)
                continue
            try:
                reply = self.do_reset(d) if d[1] > 0.5 else self.do_step(d)
            except Exception as e:  # noqa: BLE001  サービスの失敗で中継を止めない
                self.get_logger().error(f'要求の処理に失敗: {e}')
                continue
            self.last_seq, self.last_reply = seq, reply
            self.pub.publish(reply)

    def call(self, cli, req):
        fut = cli.call_async(req)
        t_end = time.monotonic() + self.timeout
        while rclpy.ok() and not fut.done() and time.monotonic() < t_end:
            time.sleep(0.0005)
        if not fut.done():
            raise TimeoutError('サービスの応答が無い')
        return fut.result()

    def make_obs(self, seq, kind, summary=None, imu=None):
        v = {k: 0.0 for k in OBS_FIELDS}
        v['seq'], v['kind'] = float(seq), float(kind)
        if summary is not None:
            v['reward'] = summary.reward
            v['terminated'] = float(summary.terminated)
            v['lap_done'] = float(summary.lap_done)
            v['progress_m'] = summary.progress_m
            v['cte_m'] = summary.cte_m
            v['speed_mps'] = summary.speed_mps
            v['collision'] = float(summary.collision)
            v['off_track'] = float(summary.off_track)
        if imu is not None:
            for k, x in zip(('ax', 'ay', 'az', 'gx', 'gy', 'gz'), imu):
                v[k] = x
        else:
            v['az'] = 9.81
        m = Float64MultiArray()
        m.data = [float(v[k]) for k in OBS_FIELDS]
        return m

    def do_reset(self, d):
        seed = self.seed_base + int(d[4])
        spawn = 'random' if d[5] > 0.5 else 'start'
        if self.ep['n'] > 0:
            dt = time.monotonic() - self.ep['t0']
            self.get_logger().info(
                f"episode done: return={self.ep['ret']:.1f} progress={self.ep['prog']:.1f} m laps={self.ep['laps']} "
                f"decisions={self.ep['n']} ({self.ep['n'] / max(dt, 1e-3):.1f} /s)")
        self.ep = {'ret': 0.0, 'prog': 0.0, 'laps': 0, 'n': 0, 't0': time.monotonic()}
        self.shaper.reset()
        res = self.call(self.cli_reset, Reset.Request(seed=seed, spawn=spawn))
        return self.make_obs(d[0], 1, imu=mean_imu(list(res.imu)[-4:]))

    def do_step(self, d):
        infos, imus = [], []
        for _ in range(self.n_steps):
            steer, v = self.shaper.step(d[2], d[3])
            a = ActuatorCmd()
            a.header.stamp = self.get_clock().now().to_msg()
            a.header.frame_id = 'base_link'
            a.steer_rad = float(steer)
            a.speed_mps = float(v)
            a.mode = ActuatorCmd.MODE_RUN
            res = self.call(self.cli_step, Step.Request(action=a))
            infos.append(res.info)
            imus.extend(res.imu)
            if res.info.terminated:
                break
        s = summarize(infos, self.w, self.lim.control_dt)
        self.ep['ret'] += s.reward
        self.ep['prog'] += s.progress_m
        self.ep['laps'] += int(s.lap_done)
        self.ep['n'] += 1
        return self.make_obs(d[0], 0, summary=s, imu=mean_imu(imus))


def main():
    rclpy.init()
    node = MlAgentsGateway()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
