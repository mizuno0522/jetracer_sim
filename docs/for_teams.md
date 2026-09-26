# 他チーム向け: JetRacer のソフトをこの sim で走らせる

NVIDIA JetRacer (https://github.com/NVIDIA-AI-IOT/jetracer) ベースの車両ソフトを、**コードをほぼ変えずに**
自動運転ミニカーバトル 2026 のコースの sim で走らせるための手順です。版は **v0.2.0 (試用版)**。

## まず知っておいてほしいこと (v0.2.0 の信頼度)

| 項目 | 状態 | 影響 |
|---|---|---|
| コース形状・壁の配置・寸法 | レギュレーション p.24 どおり | 信頼してよい |
| カメラの画角・取付角度 | 9/12 の実走画像からの**推定値** (水平 144°・垂直 120°・下向き 51°・高さ 0.148 m)。チェッカーボード較正は未 | 実機と数度ずれている可能性 |
| 舵角 (steering → 前輪の角度) | 9/12 の実走ログから推定: 中立 1681 µs (**steering −0.22 でほぼ直進**)・端点 1178 / 2002 µs = ±0.47 rad | 左右の効きは概ね合う。舵の実現率 (指令どおり切れるか) は未測定 |
| 速度 (throttle → 車速) | 9/12 の実走ログの周回時間から推定: **マイナスで前進** (ESC の向き)。throttle_gain 1.0 で −0.106 → 1.26 / −0.12 → 2.04 / −0.13 → 2.45 m/s | 周回の平均 (カーブ込み) なので直線の最高速はもっと速い。不感帯の直上でわずかな差が大きな速度差になる。★速さの比較はまだしないこと |
| 見た目 | Unity の CG に実画像の色・ぼけ・周辺減光を足したもの。**実画像そっくりではない** | 画像で学習したモデルは実機でそのまま通用するとは限らない |
| 床の白テープ・照明・観客 | 走るたびに変わる乱択化 (seed) | 特定の会場の見た目を覚えないため |

「コースを 1 周できるか」「曲がり角で壁に当たらないか」「画像が途切れたとき止まるか」の確認には使えます。
**タイム・速度の比較や、ここで学習したモデルの実機での性能保証には、まだ使えません。**

## 必要なもの

- **sim を動かす PC**: Ubuntu 22.04 (x86_64)・ROS 2 Humble・GPU (内蔵 GPU で可。OpenGL/Vulkan)
- **車両ソフトを動かすマシン**: 同じ PC でも、Jetson (JetPack 6・ROS 2 Humble) を LAN でつないでも可
- Unity Editor は**不要** (ビルド済みプレイヤーを [Release v0.2.0](https://github.com/mizuno0522/jetracer_sim/releases/tag/v0.2.0) から取る。`scripts/get_unity_player.sh` が自動で取る)

## 手順 (sim PC)

```bash
git clone https://github.com/mizuno0522/jetracer_sim.git && cd jetracer_sim
git checkout v0.2.0
sudo ./scripts/setup_host.sh pc --no-net     # apt (ROS 2 Humble・cyclonedds 等) と受信バッファ。固定 IP を触らない
./scripts/setup_ws.sh                        # Unity との中継 (ros_tcp_endpoint) を取り込む
./scripts/build.sh
./scripts/get_unity_player.sh v0.2.0         # ビルド済み Unity プレイヤー (約 28 MB)
source scripts/sim_env.sh                    # 端末ごとに毎回
ros2 launch minicar_sim sim_host.launch.py unity_player:=$HOME/jetracer/unity/player/MinicarSim.x86_64
```
Unity の窓が開き、コースと車が見えれば OK。車はまだ止まっています。

## 手順 (車両ソフト側)

JetRacer のコード (ノートブックや .py) の**先頭に 2 行**足すだけです。以降の `NvidiaRacecar` と `CSICamera` が sim につながります。

```python
import jetracer_compat
jetracer_compat.install()

from jetracer.nvidia_racecar import NvidiaRacecar    # ← ここから下は元のコードのまま
from jetcam.csi_camera import CSICamera
car = NvidiaRacecar()
camera = CSICamera(width=224, height=224, capture_fps=65)
car.steering_gain = -0.55; car.steering_offset = 0.12; car.throttle_gain = 1.0   # 例: 9/12 実走ロガーの値
car.throttle = -0.12       # ★この車はマイナスで前進
```

- ROS の環境を読み込んだ端末から起動してください: `source scripts/sim_env.sh` → `jupyter lab` (または `python3 your_code.py`)
- 別マシン (Jetson 等) で動かすときも同じリポジトリを clone・build し、`ROS_DOMAIN_ID=42` を sim PC と揃えます (`scripts/sim_env.sh` が設定)
- 動作確認: `ros2 run jetracer_compat jetracer_compat_demo` (throttle −0.12・steering −0.22 で前進してほぼ直進すれば OK。`--steering 0.5` で右へ)

### 変換のしかた

`car.steering` / `car.throttle` (−1〜1) は、実機と同じく `値 × gain + offset` → PWM パルス → 舵角・車速に変換されます。
`steering_gain` / `steering_offset` / `throttle_gain` はあなたのコードで設定した値がそのまま効きます。
`camera.value` / `camera.read()` は 224×224 の BGR 画像 (15 Hz) です。カメラの取り込みは実機の jetcam 既定
(1280×720 を 224×224 に縮めるので横が潰れる) に合わせてあります。

## 見た目を自分の会場の実画像に寄せたいとき

sim の画像を、自分の車で撮った実画像の見た目に変換する道具があります (学習データの一部に混ぜる用)。手順は [sim2real.md](sim2real.md)。
Release v0.2.0 に、9/12 の会場の実画像で学習した変換器 `sim2real_cut_003.onnx` を添付しています (その会場の見た目に寄るので、自分の会場の画像で作り直すのがおすすめ)。

## sim で方策 (画像 → 操舵) を学習したいとき

教師に揺らぎを足して走らせて記録し、画像 ＋ IMU から注視点と速度係数を出す ResNet18 を模倣学習する道具があります
([tools/policy/README.md](../tools/policy/README.md))。CPU だけで学習できます (1 エポック約 13 分)。
Release v0.2.0 には、この PC で学習した見本のモデル `policy_003.onnx` (1 ファイル・opset 18) を添付しています。
sim では学習に使っていない 7 通りの照明・床で完走・衝突 0 でしたが、**実機では未確認**です。
入出力の約束 (入力 `image` [1,3,224,224] RGB・ImageNet 正規化、`imu` [1,50,6]、出力 [1,3]) は `jetracer_stack/policy_net.py` のもので、
JetRacer 標準のコードからそのまま使う形ではありません。

## うまくいかないとき

| 症状 | 確認 |
|---|---|
| `RuntimeError: sim の /camera/image_raw が来ない` | sim が起動しているか。車両ソフト側の端末で `source scripts/sim_env.sh` したか。`ros2 topic hz /camera/image_raw` が 15 Hz か |
| 車が動かない・後退する | この車はマイナスで前進。throttle_gain 1.0 で −0.10 付近が不感帯の境目。sim 側の端末に失効 (failsafe) のログが出ていないか |
| `No module named traitlets` | `sudo apt install python3-traitlets` |
| Unity の窓が真っ黒 | GPU ドライバ。`~/.ros/log/jetracer_unity_player.log` を見る |

問題や「実機と違う」という気づきは GitHub の Issues へ。実機の較正値 (サーボ端点・スロットルと速度の対応) を
提供してもらえると、sim の精度が上がります。
