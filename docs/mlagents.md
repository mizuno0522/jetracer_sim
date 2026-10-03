# ML-Agents — プロコンで運転して模倣学習、PPO で最適化

物理は `vehicle_sim` のまま (設計 P3)。Unity の `MinicarAgent` は「観測を集めて行動を出す」だけで、車を動かすのは
lockstep の `/sim/step` (`docs/lockstep.md`)。報酬は sim ではなく中継ノードで組む。

```
mlagents-learn (PyTorch) ◀──gRPC 5004──▶ Unity MinicarSim (MinicarAgent)
                                             │ /mlagents/action [seq, kind, 舵, 速度, seed, spawn]
                                             ▼                      ▲ /mlagents/obs [報酬・終端・IMU …]
                                  tools/mlagents/mlagents_gateway.py ──/sim/reset, /sim/step ×2──▶ vehicle_sim (lockstep)
プロコン ─▶ joy (game_controller_node) ─/joy─▶ tools/teleop/joy_teleop.py ─/teleop/cmd─▶ MinicarAgent.Heuristic
```

| | 中身 |
|---|---|
| 1 判断 | `/sim/step` 2 回 = 1/15 s (カメラ 15 Hz に合わせる。`decision_steps`) |
| 観測 | 車載カメラ画像 (配信している 224×224 を縮小。`-mlres` 既定 96) ＋ IMU 6 値 (2 step の平均) ＋ 直前の行動 2 値。3 判断ぶん積む。**真の車速・位置は入れない** (実機で取れない) |
| 行動 | 連続 2 値。舵 -1〜1 (左正) → ±δmax、速度 -1〜1 → 0〜v_max。gateway が **cmd_shaper と同じ制限** (舵 8 rad/s・加速 3・減速 6 m/s²) をかけて `ActuatorCmd` にする |
| 報酬 | `tools/mlagents/config/reward.yaml`。前進距離が主項、横偏差・壁への接近・舵の振動・逆走に罰、周回にボーナス、衝突・コース外で終端 |
| エピソード | 衝突・コース外で終わり、`-maxsteps` (既定 1800 判断 = 2 分) で打ち切り。開始位置は学習 `random`・人の運転 `start` |

## 準備 (sim PC・初回だけ)

```bash
# Unity: Packages/manifest.json に com.unity.ml-agents 4.0.0 を足してある (Unity 6000.0 以上)。
#        初回にエディタで開くと取り込まれる。プレイヤーを作り直す (docs/unity.md の手順 2)
# 学習器: Python 3.10.12 の仮想環境 (ROS の Python と分ける)
conda create -n mlagents python=3.10.12 && conda activate mlagents
pip install torch~=2.2.1 --index-url https://download.pytorch.org/whl/cu121
pip install mlagents==1.1.0
# ゲームパッド
sudo apt install ros-humble-joy
```

## 1. プロコンをつないで割り当てを確かめる

プロコンは USB か Bluetooth でつなぐ (Ubuntu 22.04 は `hid-nintendo` で認識する)。

```bash
source scripts/sim_env.sh
ros2 run joy game_controller_node &                 # /joy
python3 tools/teleop/joy_teleop.py --probe          # 動かした軸・ボタンの番号と値が出る
```

既定の割り当て (`tools/teleop/joy_map.yaml`): **左スティック = ハンドル、ZR = アクセル、ZL = ブレーキ、＋ = 記録の区切り、− = 非常停止**。
probe の結果と違えば yaml を直す (向きが逆なら `scale: -1`、トリガの離した値・押し切った値は `rest` / `pressed`)。
トリガは「離した値」を一度見るまで 0 として扱う (ドライバによっては触るまで 0.0 を出し、半分踏んだと誤読して発進するため)。

## 2. 人の運転を記録する (.demo)

```bash
# 端末 1: sim (lockstep) ＋ Unity。Unity に -mlagents -demo を渡す
ros2 launch minicar_sim sim_host.launch.py sim_mode:=lockstep unity_player:=<Build/MinicarSim.x86_64> \
     mlagents:=true demo:=pro_run01 car:=rx7
# 端末 2: 中継
python3 tools/mlagents/mlagents_gateway.py --ros-args -p reward_file:=tools/mlagents/config/reward.yaml
# 端末 3: プロコン
ros2 run joy game_controller_node & python3 tools/teleop/joy_teleop.py
```

- 実時間 15 Hz で進む。画面下に `ML-Agents: human driving ● REC` と周回・報酬が出る
- 壁に当たるかコースを出るとエピソードが切れて、スタート位置に戻る。**＋** でも区切れる
- Unity を閉じると `demos/pro_run01.demo` に書き出される (`demo_dir`、既定は launch を起動したフォルダの `demos/`)。**閉じずに落とすと書き出されない**
- 5〜10 周を目安に。うまく走れた周だけを残したいときは、周回ごとに別の名前で記録して選ぶ
- キーボードでも記録できる (←→ ハンドル、↑ アクセル、↓ ブレーキ)。プロコンが来ていればそちらを使う

## 3. 学習する (模倣 → PPO)

先に学習器を上げ、そのあと Unity を起動する (Unity は起動時に 5004 番の学習器を探す)。

```bash
# 端末 0: 学習器 (リポジトリ直下で。demo_path: demos はここからの相対)
conda activate mlagents
mlagents-learn tools/mlagents/config/minicar_bc_ppo.yaml --run-id=minicar_bc_ppo_01 \
    --time-scale=1 --capture-frame-rate=0 --target-frame-rate=60
# 端末 1・2: 上と同じだが demo は付けない
ros2 launch minicar_sim sim_host.launch.py sim_mode:=lockstep unity_player:=<Build/MinicarSim.x86_64> mlagents:=true sound:=off
python3 tools/mlagents/mlagents_gateway.py --ros-args -p reward_file:=tools/mlagents/config/reward.yaml
# 経過
tensorboard --logdir results        # Environment/Cumulative Reward、minicar/progress_m・laps・crash
```

- `behavioral_cloning` が最初の 15 万判断で人の運転に寄せ、以降は PPO が報酬で伸ばす。デモが数周しか無いときは `minicar_gail_ppo.yaml` (GAIL を弱く足す)
- `--time-scale=1 --capture-frame-rate=0` は必須。ML-Agents の既定 (時間 20 倍・キャプチャ 60 fps) は Unity の時計を変え、配信画像の周期と自動露出がずれる。sim の時間は lockstep が決めるので、時間倍率では速くならない
- 速さの目安: 1 判断ごとに画像の描画を待つので **毎秒 10〜15 判断** (gateway のログ `decisions=… (… /s)`)。100 万判断で 20 時間前後。止めて再開は `--resume`
- 学習した方策で走らせるだけなら `mlagents-learn <同じ yaml> --run-id=<同じ> --resume --inference`
- 結果の方策は `results/<run-id>/MinicarDriver.onnx`

## 報酬を変える

`tools/mlagents/config/reward.yaml` を直して gateway を上げ直すだけ (sim と Unity は触らない)。項目名の打ち間違いは起動時に止まる。
ROS 無しで式を試すなら `python3 tools/mlagents/test_gateway_core.py`。

## 注意

- **実機の推論スタック (policy_net) とは出力が違う。** policy_net は先行注視点 (u, v, s) を出し cmd_shaper が舵にする。ここの方策は舵と速度を直接出す (人の運転と同じ形で記録できるように)。実機に載せるには、ONNX の入出力に合わせた小さな推論ノードが別に要る
- 観測の IMU は `imu_sim` の汚れ (ゼロ点・雑音・振動) 込み。画像も配信と同じ後処理 (歪み・露出・ノイズ) 込み
- lockstep では `vehicle_stack` を上げない (`/actuator_cmd` を出すものが 2 つになる)。realtime で人が運転するだけなら、`joy_teleop.py --ros-args -p publish_actuator:=true` が `/actuator_cmd` を直接出す (このときも `vehicle_stack` は上げない)
