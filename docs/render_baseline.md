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

## 3 版を同じ仕様にしたあと (2026-10-06、ブランチ feat/unify-render-tiers)

3 版とも Linear・同じ後処理・ミニカーの会場は部屋。`QUALITY=… SHOTS=0 SHOTVIEWS= ./scripts/shots.sh <fuji|minicar> rx7`、600 フレーム、aic (3 分割)。
裏で minicarbattle2026 の検証が動いている状態 (負荷平均 2〜4) で計ったので、上の表より数 % 低く出ている可能性がある。

| コース | 画質 | Built-in | URP | HDRP |
|---|---|---|---|---|
| 富士 | medium | 134.2 fps | 123.8 fps | 39.2 fps |
| 富士 | low | 147.3 fps | — | — |
| ミニカーの会場 (部屋) | medium | 123.1 fps | 227.4 fps | 114.3 fps |
| ミニカーの会場 (部屋) | low | 125.3 fps | — | — |

- ミニカーの会場の Built-in low は 224 → 125 fps。会場が部屋になり描く物が増えたため (後処理は low では掛からない)
- 学習モード (lockstep) の 1 判断あたりの時間は、部屋の会場ではまだ計り直していない (描画は 1 コマ 4.5 → 8.0 ms)

### 静かな状態での計り直し (2026-10-06 朝、main 947079a + 壁板の足をまとめて描く変更)

ほかの検証を止めてもらって計った (負荷平均 1〜4)。`QUALITY=… SHOTS=0 SHOTVIEWS= ./scripts/shots.sh <minicar|fuji> rx7`、600 フレーム、aic (3 分割)。

| コース | 画質 | Built-in | URP | HDRP |
|---|---|---|---|---|
| ミニカーの会場 | low | 196.9 fps | — | — |
| ミニカーの会場 | medium | 191.8 fps | 224.3 fps | 114.3 fps |
| 富士 | low | 148.6 fps | — | — |
| 富士 | medium | 134.3 fps | 123.5 fps | 39.1 fps |

Built-in のミニカーの会場の移り変わり (low / medium):

| 状態 | low | medium |
|---|---|---|
| 以前の灰色の床の会場 (plain) | 224.3 fps | — |
| 部屋 | 125.3 fps | 123.1 fps |
| 地面と空だけ | 143.8 fps | 135.4 fps |
| 地面と空だけ + 壁板の足をまとめて描く | 196.9 fps | 191.8 fps |

- 部屋をやめただけでは 144 fps までしか戻らなかった。重かったのは壁板の足 (鉄板・ボルト・塩ビ管が約 100 か所 × 4 部品、影つき)。
  動かない足を 1 つにまとめて描く (`StaticBatchingUtility.Combine`) と 197 fps。絵は変わらない (前後の画像の差 0.1〜0.6/255)
- URP・HDRP の値は、足をまとめる変更を入れる前のプレイヤーで計ったもの

学習モード (lockstep、Built-in、画質 low、手順は上の「学習モード」と同じで 3 分):

| コース | 1 秒あたりの判断 | 1 判断あたり | 基準との差 |
|---|---|---|---|
| ミニカーの会場 (地面と空だけ) | 5.6 /s (10 エピソードの平均。5.0〜6.0) | 179 ms | +18 % |
| ミニカーの会場 (+ 足をまとめて描く) | 5.8 /s (5.2〜6.8) | 171 ms | +13 % |
| 富士 (real_rx7) | 7.0 /s (2 エピソード) | 143 ms | +4 % |

★ミニカーの会場は「基準の +10 % 以内」を越えている。描画を 1 コマ 2 ms 軽くしても 8 ms しか縮まらなかったので、描画が律速ではなさそう。
1 エピソードが 97 判断 (約 17 秒) で終わっていて、リセットの時間が効いている可能性がある (基準を計ったときのエピソードの長さは記録が無い)。
原因はまだ調べていない。
