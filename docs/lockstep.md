# lockstep — 強化学習 IF (sim_mode:=lockstep)

設計書「強化学習 IF」タブ。観測 (画像 ＋ IMU) と行動 (`/actuator_cmd`) は凍結済みでそのまま使う。足すのはエピソード制御・報酬の材料・終端判定だけで、すべて `/sim/` 名前空間に閉じる。推論スタックは何も知らない。

| | `sim_mode:=realtime` (既定) | `sim_mode:=lockstep` |
|---|---|---|
| 時計 | 壁時計。`use_sim_time: false` | sim 時計。`/clock` を出す。Jetson 側は `vehicle_stack.launch.py use_sim_time:=true` |
| 進み方 | 100 Hz で流れ続ける | `/sim/step` を呼ぶたびに 1/30 s 進む。呼ばなければ止まる |
| 物理ティック | 10 ms | 同じ 10 ms。1 step = 33.3 ms なので **3, 3, 4, 3, 3, 4 …** (累積器で端数を持ち越す。3 に丸めると制御周期が 30 ms になり実機と 10 % ずれる) |
| 画像 | 15 Hz で流れる | 2 step に 1 枚。来る予定の step だけ、新しい stamp の画像を `step_image_timeout_s` (0.5 s) まで待つ |
| IMU | `imu_sim` が 100 Hz | 同じ `imu_sim` (別プロセス) がティックごとに 1 サンプル。step はその 3〜4 個が届くまで待つ (上限 0.1 s) |
| 指令の失効 | 300 ms | 無し (step の指令は失効しない) |
| 用途 | bench・A/B・教師データ生成 | 強化学習・決定的な再生 (同じ seed で同じ軌跡) |
| 物理・センサモデル | **同一コード** (P13)。モードで分岐するのはスケジューラだけ | |

## 起動

```bash
# sim PC (Unity 無し。Unity ありなら camera_backend:=unity unity_player:=<Build>)
ros2 launch minicar_sim sim_host.launch.py sim_mode:=lockstep camera_backend:=opencv unity_player:=none
# Docker なら
SIM_MODE=lockstep CAMERA_BACKEND=opencv docker compose up -d sim
```

`vehicle_stack` (cmd_shaper 等) は lockstep では**上げない**。学習器が `cmd_shaper` と同じ計算 (`jetracer_common.actuator_model` / `vehicle_profile`) を通して `ActuatorCmd` を作り、`/sim/step` に直接渡す。

## サービスとメッセージ (`minicar_sim_msgs`)

```
# srv/Reset.srv
uint32 seed               # 物理・imu_sim・Unity の乱択化をすべてこの seed から (/sim/episode に配る)
string spawn              # "start" | "random" | "zone:<id>"
string[] override_keys    # imu_sim.yaml のパス上書き。★未実装 (警告を出す。yaml を直接変えること)
float64[] override_values
---
bool ok
sensor_msgs/Image image   # 初期観測 (来ていなければ空)
sensor_msgs/Imu[] imu     # 直近窓 (来ているぶん)
StepInfo info

# srv/Step.srv  ── lockstep のみ
minicar_msgs/ActuatorCmd action   # ★実機と同じ型。cmd_shaper を通した後の値 (δ rad・v m/s・mode)
---
sensor_msgs/Image image           # この step の後に描画された画像 (timeout 内に来なければ直前のもの)
sensor_msgs/Imu[] imu             # この step で生成された 3〜4 サンプル (窓は学習器側で保持)
StepInfo info
```

`StepInfo` は**報酬の材料**で、報酬そのものは入れない (式を変えるたびに sim を触ることになり、bench と同じバイナリでなくなる)。

| フィールド | 意味 | 使い方 | 穴 |
|---|---|---|---|
| `progress_m` | 参照線上の弧長の前進 (この step ぶん) | 主項 | これだけだと壁すれすれの最短を走る |
| `cte_m` | 横偏差 (左正) | 二乗で罰。0.2 m を効き始めの幅に | 強すぎると参照線をなぞるだけで速度を出さない |
| `heading_err_rad` | 方位誤差 | 補助 | |
| `speed_mps` | 真の車速 (実機では取れない) | 監視用。観測に入れない | |
| `min_wall_clear_m` | 壁までの最小余裕 (半車幅を引いた値) | 閾値を割ったら罰 | |
| `steer_rate_rad_s` | 舵の変化率 | 二乗で小さく罰 | 入れないと舵が振動する方策になる (実機のサーボが壊れる) |
| `zone` | 区間 ID (BodyState.SURFACE_*) | 区間別の統計 | |
| `collision` / `off_track` | 終端条件 | 終端 ＋ 大きな罰 | |
| `lap_done` | 周回完了 (その step だけ true) | ボーナス | 周回タイムを直接報酬にすると分散が大きい |
| `terminated` | collision or off_track | エピソード終了 | |
| `truncated` | 時間切れ。★sim は常に false (学習器側で step 数を数えて切る) | 終端と区別する (価値関数のブートストラップに影響) | |

## Python から (rclpy 直接)

```python
import rclpy
from rclpy.node import Node
from minicar_sim_msgs.srv import Reset, Step
from minicar_msgs.msg import ActuatorCmd

rclpy.init(); n = Node('learner')
reset = n.create_client(Reset, '/sim/reset'); step = n.create_client(Step, '/sim/step')
reset.wait_for_service(); step.wait_for_service()

r = Reset.Request(seed=1, spawn='start')
res = reset.call(r)                      # 同期呼び出しで十分 (1 プロセスで直列)
for k in range(900):                     # 30 s
    a = ActuatorCmd(); a.steer_rad = 0.0; a.speed_mps = 1.0; a.mode = ActuatorCmd.MODE_RUN
    out = step.call(Step.Request(action=a))
    img, imu, info = out.image, out.imu, out.info
    if info.terminated: break
```

この Jetson (Orin Nano) での実測: 90 step (3.0 s sim) が 3.6〜6.2 s wall (OpenCV 描画・物理と imu_sim を同居)。ticks/step は `[3, 3, 4, 2, 2, 4, ...]` になることがあった → `imu_model` の 1 kHz/100 Hz の時刻を整数インデックスにして修正済み (浮動小数の累積で step ごとのサンプル数が揺れないように)。Unity ありの速さは未実測。

## Gymnasium ラッパ (仕様。実装は未着手)

```python
observation_space = Dict({
    "image": Box(0, 255, shape=(224, 224, 3), dtype=uint8),   # camera_preproc 前の生画像
    "imu":   Box(-inf, inf, shape=(50, 6), dtype=float32),    # 0.5 s 窓 × [ax ay az gx gy gz]
})
action_space = Box(-1.0, 1.0, shape=(3,), dtype=float32)      # (u, v, s) 正規化。policy_net と同じ形
# step の内側: action → cmd_shaper 相当 (vehicle_profile) → ActuatorCmd → /sim/step → obs, info
#   reward = f(info)   ← ここだけが学習側の自由。sim は触らない
#   terminated = info.terminated, truncated = (step 数 >= 上限)
```

行動空間を模倣学習と同じ (u, v, s) にするのは、模倣学習の `policy_net` を初期方策にして改善するため。操舵角を直接出す行動空間にするとこの接続が切れる。

## 決定性

同じ `seed` で同じ軌跡になること。`Reset` の seed は `vehicle_sim` の rng と `/sim/episode` (LATCHED) 経由で `imu_sim` のターンオンバイアス・取付角誤差・スケール誤差を引き直す。Unity の乱択化 (照明・材質) は未接続。確認は 2 回 Reset→Step を同じ行動列で回し、`GroundTruth` の (x, y, yaw) が一致すること。imu_sim の `latency.drop_prob` は乱数で落とすので imu 配列の長さは揺れうる。

## N 台並列 (未実装)

名前空間 `/env_k/…` で 1 プロセスの `vehicle_sim` が N 台を持ち、`imu_sim` は seed + k で N インスタンス、Unity は 1 シーンに N 台 ＋ N カメラ、`/sim/step` はバッチ。`/sim/render_state` の `step_id`・`car_id` はこのために予約済み。模倣学習の方策が実機で走ってから着手する。
