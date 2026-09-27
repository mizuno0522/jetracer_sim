#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
記録した相手の走り (race2_pose_<domain>.csv の自車列 t,x,y,yaw,v) を /sim/rival_state として再生する「ゴースト」。
2 台レースの相手を、毎回同じ走りで出したいとき (MPPI の修正を同じ条件で比べる) に使う。相手の走行スタックは要らない。
  source scripts/sim_env.sh; python3 tools/race/ghost_replay.py tools/race/ghosts/m05_pp_safe_line3.csv
- スタート前: 記録の最初の姿勢 (スタートライン上) で止まった相手を流す
- 自車 (/sim/render_state の速度) が動き出したら、記録の「相手が動き出した時刻」に合わせて再生を始める (両車とも同じ /run で出るため)
- 再生の終わりの後は最後の姿勢で止まる
受け取る側 (vehicle_sim) は車どうしの衝突でこれを相手として扱い、Unity はカメラ画像に相手の車を描く。
記録は minicarbattle2026 の race2.sh / jetracer の vehicle_sim の RACE_POSE_LOG (同じ列) のどちらでもよい。
ゴーストは押されても動かない (記録どおり走る) ので、接触したときは自車だけが半分の衝撃を受ける。
"""
import csv, math, sys, time
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray


def main():
    rows = list(csv.DictReader(open(sys.argv[1])))
    t = np.array([float(r['t']) for r in rows]); t -= t[0]
    x = np.array([float(r['x']) for r in rows]); y = np.array([float(r['y']) for r in rows])
    yaw = np.unwrap(np.array([float(r['yaw']) for r in rows])); v = np.array([float(r['v']) for r in rows])
    moving = np.nonzero(v > 0.05)[0]
    t_move = t[moving[0]] if len(moving) else 0.0
    rclpy.init()
    n = rclpy.create_node('ghost_replay')
    pub = n.create_publisher(Float64MultiArray, '/sim/rival_state', 10)
    st = {'start': None}

    def on_own(m):
        if st['start'] is None and len(m.data) > 13 and abs(m.data[13]) > 0.05:
            st['start'] = time.time()
            n.get_logger().info(f'自車が動き出した → ゴーストの再生を開始 (記録 {t_move:.1f} s から)')
    n.create_subscription(Float64MultiArray, '/sim/render_state', on_own, 10)

    def tick():
        now = time.time()
        tr = t_move + (now - st['start']) if st['start'] is not None else t[0]
        tr = min(max(tr, t[0]), t[-1])
        px, py = float(np.interp(tr, t, x)), float(np.interp(tr, t, y))
        pyaw = float(math.atan2(math.sin(np.interp(tr, t, yaw)), math.cos(np.interp(tr, t, yaw))))
        pv = float(np.interp(tr, t, v)) if st['start'] is not None else 0.0
        m = Float64MultiArray()
        m.data = [float(int(now)), float((now % 1) * 1e9), px, py, pyaw, float(tr), 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, pv, 0.0, 0.0, 0.0]
        pub.publish(m)
    n.create_timer(0.02, tick)
    n.get_logger().info(f'ゴースト: {sys.argv[1]} ({t[-1]:.0f} s、動き出し {t_move:.1f} s)。スタート位置 ({x[0]:.2f}, {y[0]:.2f})')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
