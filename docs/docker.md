# Docker — sim PC の ROS 側だけを入れる

設計書「環境と配置」タブの「ホスト側を Docker にするか」。役割で分ける:

| ホスト | 判断 | 理由 |
|---|---|---|
| sim PC の ROS 側 (物理・imu_sim・camera_info・endpoint・学習器) | **Docker** | 既存記録の empy 4.x / numpy 2.x が `~/.local` から混ざる問題を壁で消す。Humble を 22.04 イメージに固定。lockstep の「同じ seed で同じ結果」にも効く |
| sim PC の Unity プレイヤー | 素 | Linux ヘッドレス描画の設定が Unity 6 では面倒で得るものが少ない。ビルド済みプレイヤーは依存が自己完結。同じマシンの Docker 内 endpoint に 127.0.0.1:10000 で届く (`network_mode: host`) |
| Jetson 走行スタック | 素 | I2C・CSI・SCHED_FIFO・mlockall を使う。Docker でもできるが `--privileged --ulimit rtprio` が増えるだけ |

## ファイル

| | |
|---|---|
| `docker/sim.Dockerfile` | `ros:humble-ros-base` ＋ cyclonedds・cv_bridge・rosbag2 (mcap)・numpy/scipy/opencv ＋ ROS-TCP-Endpoint (main-ros2) ＋ `ros_ws/src` を colcon build |
| `docker/learner.Dockerfile` | `pytorch/pytorch:2.4.0-cuda12.1-cudnn9-runtime` に Humble を足し、`minicar_msgs`・`minicar_sim_msgs`・`jetracer_common` だけビルド。onnx・onnxruntime-gpu 入り |
| `docker/docker-compose.yml` | `sim` (常用) と `learner` (`--profile learn`)。両方 `network_mode: host` |

## 使い方 (sim PC)

```bash
cd docker
docker compose build                       # 初回は数分
docker compose up -d sim                   # sim_host.launch.py unity_player:=none を起動 (Unity は素で別に上げる)
docker compose logs -f sim                 # vehicle_sim・imu_sim・camera_info_pub・endpoint の起動ログ
SIM_MODE=lockstep docker compose up -d sim # 強化学習モード。CAMERA_BACKEND=opencv で Unity 無し
docker compose --profile learn run --rm learner   # 学習器のシェル (GPU)
```

- `sim.yaml`・`imu_sim.yaml`・`vehicle_profile/*.yaml` は `ros_ws/src/minicar_sim/config` を読み取り専用でマウントしているので、**編集したら `docker compose restart sim` だけ** (再ビルド不要)。
- コード (`*.py`) を変えたら `docker compose build sim`。
- rosbag は `../bags` (ホスト) に書く。イメージの中に書くと消える上に肥大する。
- `ROS_DOMAIN_ID` はホストの環境変数を引き継ぐ (`scripts/sim_env.sh` を source してから compose を打つ)。既定 42。

### 確認

```bash
docker compose exec sim bash -lc "ros2 topic list"   # /imu /sim/body_state /sim/ground_truth /camera/camera_info
ros2 topic hz /imu        # ★ホスト側 (Docker の外) から 100 Hz が見える = network_mode: host が効いている
ros2 topic hz /imu        # Jetson からも同じ値が見える = P9 (2 ホスト接続) が通った
docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi   # learner 用に GPU が見える
```

## 踏みやすい 4 つ

1. **`network_mode: host` 以外では動かない。** DDS は探索にマルチキャスト、データにエフェメラルポートを使う。bridge ネットワークでは別ホストからも別コンテナからも見えない。
2. **`net.core.rmem_max` はホストの sysctl。** コンテナの中で打っても効かない。`/etc/sysctl.d/60-cyclonedds.conf` に置く (`docs/setup.md` Step 0)。
3. **chrony はホストで動かす。** コンテナはホストの時計を見る。中に入れると二重に動いて壊れる。
4. **RMW を混ぜない。** 全ホスト `rmw_cyclonedds_cpp`。Fast DDS に戻すなら `ipc: host` と `pid: host` が要る (共有メモリ転送)。

## 1 台にまとめたときの奪い合い

CPU 側は既存の 1 台構成で実績がある (物理 100 Hz ＋ 描画同居で 3 周 42.99 s)。新しく増えるのは learner が Unity と同じ GPU を使うこと。模倣学習のデータ生成中は learner を止めておく。lockstep で両方が同時に走るときは Unity の描画レート (step の速さの上限) が落ちる。`nvidia-smi dmon` で取り合いを見て、足りなければ learner を別 PC に出す (DDS なので `ROS_DOMAIN_ID` を揃えるだけ)。

## Jetson で Docker を使いたい場合

開発用なら `nvcr.io/nvidia/l4t-base` ＋ Humble でも動く。ただし `--privileged` (I2C・CSI)・`--ulimit rtprio=99`・`--network host` が要り、`jetracer_bridge` の優先度設計は素の前提で書いてある。本番は素で動かす。
