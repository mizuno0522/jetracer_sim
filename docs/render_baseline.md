# 描画の基準 (評価用 PC)

合格条件 (「学習の速さは基準の +10 % 以内」「medium 60 fps・high 30 fps 以上」) を比べるための数値。
見た目やシェーダを変えたら同じ手順で計り直し、ここの値と比べる。

| 項目 | 値 |
|---|---|
| 計った日 | 2026-10-05 (ブランチ feat/cars-teleop-mlagents-sound、コミット 746ca5b) |
| PC | MSI Bravo 15 (Ryzen 5 4600H ×12・16 GB)、電源接続 |
| GPU | Radeon RX 5300M 3 GB (Mesa の名前は `AMD Radeon Graphics (navi14, …)`、OpenGLCore)。`DRI_PRIME=1` で選ぶ (launch・shots.sh の既定) |
| Unity | 6000.0.83f1、Built-in の描画 |
| 条件 | ほかに重い処理が動いていない状態 (負荷平均 1〜4)。画面 1850×1016 |

## 描画 (`./scripts/shots.sh`、600 フレーム、車速 50 m/s)

| コース | 画質 | レイアウト | 平均 | 99 % | 最悪 | コースの組み立て |
|---|---|---|---|---|---|---|
| 富士 (RX-7 + 787B + ND) | low | aic (3 分割) | 150.5 fps | 9.50 ms | 11.12 ms | 3.4 s |
| 富士 | medium | aic | 144.3 fps | 8.86 ms | 10.83 ms | 3.3 s |
| 富士 | high | aic | 128.4 fps | 10.18 ms | 12.58 ms | — |
| 富士 | medium | chase (追従視点) | 273.5 fps | 6.54 ms | 7.44 ms | — |
| 富士 | high | chase | 251.8 fps | 6.58 ms | 10.61 ms | — |
| ミニカーの会場 | low | aic | 224.3 fps | 6.36 ms | 7.31 ms | 0.2 s |

センサ配信はどれも 15.0 Hz (設計値どおり)。

### HDRP 版 (`~/jetracer/unity/player_hdrp`、Vulkan・RADV)

| コース | 画質 | レイアウト | 平均 | 99 % | 最悪 |
|---|---|---|---|---|---|
| 富士 | medium | chase | 47.7 fps | 31.96 ms | 41.35 ms |
| 富士 | high | chase | 36.0 fps | — | — |

Built-in 版の同じ条件 (medium 273.5 fps・high 251.8 fps) の 1/6〜1/7。合格条件は high の 30 fps は満たすが、medium の 60 fps に届かない。

```bash
JETRACER_UNITY_PLAYER=$HOME/jetracer/unity/player_hdrp QUALITY=medium LAYOUT=chase ./scripts/shots.sh fuji rx7
```

```bash
QUALITY=medium LAYOUT=chase SHOTS=0 SHOTVIEWS= ./scripts/shots.sh fuji rx7     # → shots/…/bench.md
```

## 学習モード (lockstep + mlagents_gateway、画質は常に low)

人の運転 (`demo:=…`、`/teleop/cmd` でアクセル 0.3 を入れ続ける) で 3 分。gateway のログ `decisions=… (… /s)`。

| コース | 1 秒あたりの判断 | 1 判断あたり |
|---|---|---|
| ミニカーの会場 (RX-7 の車体) | 6.6 /s | 152 ms |
| 富士 (real_rx7、ライバル 787B・ND) | 7.3 /s | 137 ms |

`docs/mlagents.md` の目安 (毎秒 10〜15) より遅い。学習器 (`mlagents-learn`) をつないだ状態ではまだ計っていない。

```bash
ros2 launch minicar_sim sim_host.launch.py sim_mode:=lockstep mlagents:=true demo:=bench sound:=off \
     tcp_port:=10001 unity_player:=$HOME/jetracer/unity/player/MinicarSim.x86_64 ml_maxsteps:=600
python3 tools/mlagents/mlagents_gateway.py --ros-args -p reward_file:=tools/mlagents/config/reward.yaml
ros2 topic pub -r 30 /teleop/cmd std_msgs/msg/Float64MultiArray "{data: [0.0, 0.3, 0.0, 0.0]}"
```

## 注意

- 何も指定しないと内蔵 GPU (Renoir) で動き、富士の low で 27 fps・センサ配信 8.8 Hz まで落ちる。`DRI_PRIME=1` が効いているかは
  起動ログ `[RenderQuality] … GPU=… (navi14 …)` で確かめる
- ほかの検証 (例: minicarbattle2026 の 3 台戦) が動いていると数値は大きく下がる (富士 low で 49.5 fps、学習 4.2 /s だった)。計るときは止める
- `.demo` のファイル名は ML-Agents が英数字だけにする (`demo:=bench_fuji` → `demos/benchfuji.demo`)
- 学習用の Python は `~/jetracer/venv_mlagents` (Python 3.10・torch 2.2.2 CPU 版・mlagents 1.1.0)。この PC の GPU は AMD なので CUDA 版は使えない

## プロコンの確認 (人が行う)

```bash
source scripts/sim_env.sh
ros2 run joy game_controller_node &
python3 tools/teleop/joy_teleop.py --probe      # 動かした軸・ボタンの番号が出る。tools/teleop/joy_map.yaml と違えば直す
# 富士を走る (realtime)
ros2 launch minicar_sim sim_host.launch.py course:=fuji vehicle_profile:=real_rx7 car:=rx7 rival_car:=b787 rival2_car:=nd \
     tcp_port:=10001 unity_player:=$HOME/jetracer/unity/player/MinicarSim.x86_64
python3 tools/teleop/joy_teleop.py --ros-args -p publish_actuator:=true -p vehicle_profile_file:=real_rx7
```
