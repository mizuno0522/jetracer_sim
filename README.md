# JetRacer sim

> **他チームの方へ**: JetRacer ベースのソフトをこの sim で走らせる手順は [docs/for_teams.md](docs/for_teams.md) (v0.1.1・試用版。信頼度の表を先に読んでください)。ビルド済み Unity プレイヤーは [Release v0.1.1](https://github.com/mizuno0522/jetracer_sim/releases/tag/v0.1.1)。

自動運転ミニカーバトル 2026 のコースを **JetRacer ベースの車両 (Tamiya TT-02 4WD ＋ Jetson Orin Nano ＋ CSI カメラ ＋ 6 軸 IMU)** で走らせるための、ROS 2 ＋ Unity シミュレータ。
[minicarbattle2026](https://github.com/mizuno0522/minicarbattle2026) の既存 sim (物理 `vehicle_sim`・Unity 描画) を転用し、**差し替えたのはブリッジ 1 枚とセンサ合成 (`imu_sim`) だけ**。物理・描画・`/actuator_cmd` の境界は動かしていない。

- 設計書 (詳細版・全タブ): **https://claude.ai/artifact/NBkzJSj8s61NLonsNQwBUG** ← まずこれを読む。この README は実装側の案内
- 判断は CNN (画像 ＋ IMU)。出力は **先行注視点 (u, v) と速度係数 s**。操舵角は `cmd_shaper` が車両諸元から作る
- **IMU は指令からではなく `vehicle_sim` の物理状態から合成する** (設計 P8)。指令から作ると、学習したネットワークが実機で外れる

```
推論スタック (Jetson)  camera_preproc/policy_net → cmd_shaper → failsafe        ★sim と実機で同一
      ▲ /camera/image_raw 224×224 15 Hz   ▲ /imu 100 Hz          ▼ /actuator_cmd δ[rad] v[m/s] 30 Hz
──────────────── ROS 2 トピック境界 (型・単位・周期・QoS を凍結) ──────────────────────────────────
A. sim (PC)     vehicle_sim 100 Hz → imu_sim ★新規 / ros_tcp_endpoint ↔ Unity MinicarSim
B. 実機 (車上)  jetracer_bridge (PCA9685 servo/ESC・SY-151 IMU) / csi_camera_node
```

## 構成

```
jetracer_sim/
├── ros_ws/src/
│   ├── minicar_msgs/        共通メッセージ。★ActuatorCmd (凍結する契約)・LookAhead (方策の出力)
│   ├── minicar_sim_msgs/    sim 専用 (/sim/…): BodyState・GroundTruth・StepInfo・srv Reset/Step。推論は購読禁止
│   ├── jetracer_common/     ROS 非依存の共有モジュール: カメラ幾何・アクチュエータ模型・IMU モデル・参照線・profile (単体テスト付き)
│   ├── minicar_sim/         物理 vehicle_sim (転用＋改修)・course.py・camera_info_pub・Unity 書き出し・config/
│   │   └── config/vehicle_profile/{jetracer_tt02,m05}.yaml   ★車両の数値の定義元 (1 か所)
│   ├── imu_sim/             ★新規。/sim/body_state → /imu (1 kHz 内部・100 Hz 出力・9 種の汚れ)
│   ├── jetracer_stack/      Jetson に載る器: cmd_shaper・failsafe・policy_net (ONNX)・gt_teacher (sim 専用の教師)・TF
│   └── jetracer_bridge/     実機: PCA9685 ＋ MPU-6500 互換 ＋ CSI カメラ (arduino_bridge の位置に置く)
├── unity/course.json        コース＋カメラ幾何の書き出し (Unity 側は数値を持たない)。Unity プロジェクトは minicarbattle2026/unity
├── docker/                  sim PC の ROS 側 (sim.Dockerfile / docker-compose.yml) と学習器 (learner.Dockerfile)
├── config/cyclonedds.xml    2 ホスト用 DDS 設定
├── tools/                   make_route.py・corridor_check.py (参照線)・lap_eval.py (周回評価)・camera_calib.py (チェッカーボード較正)・fit_camera_from_images.py (実走画像からカメラ推定)・record_video.py・imu_allan.py・imu_spectrum.py (IMU 5 測定)・sim2real/ (画像変換)
├── scripts/                 setup_host.sh (★環境を一気に作る: apt・sysctl・固定 IP・chrony)・p9_check.sh (2 ホスト直結の起動前チェック)・build.sh・test.sh・run_sim_local.sh・smoke_test.sh・setup_ws.sh・setup_unity_player.sh・export_course.sh・check_no_sim_topics.sh・sim_env.sh
└── docs/                    architecture (境界・トピック)・setup (2 ホスト構築)・unity・docker・calibration・lockstep (強化学習 IF)
```

## 動かす (最短: 1 台・Unity 無し)

Jetson でも PC でも同じ。Unity の代わりに `vehicle_sim` の OpenCV 描画を使い、ground truth の教師で閉ループを回す。

```bash
git clone https://github.com/mizuno0522/jetracer_sim.git && cd jetracer_sim
./scripts/build.sh                  # colcon build (PYTHONNOUSERSITE=1 込み)
./scripts/test.sh                   # 走行ゼロで落とせる枝: 単体テスト 14 件
./scripts/run_sim_local.sh          # sim_host (opencv) ＋ vehicle_stack (teacher) を起動。Ctrl-C で止める
```

別端末で:

```bash
source ros_ws/install/setup.bash
ros2 topic hz /imu                  # 100 Hz
ros2 topic hz /camera/image_raw     # 15 Hz (224×224)
ros2 topic hz /actuator_cmd         # 30 Hz
ros2 topic echo /sim/ground_truth --once   # lap・cte_m・u/v_px (先行注視点の画像座標) が動く
```

本番の 2 ホスト構成 (sim PC: Unity ＋ Docker / Jetson: 推論) は [docs/setup.md](docs/setup.md)。

| 起動 | コマンド |
|---|---|
| sim PC (Unity 描画) | `ros2 launch minicar_sim sim_host.launch.py` (`unity_player:=<Build/MinicarSim.x86_64>`。既存 sim と同居なら `tcp_port:=10001`・[docs/unity.md](docs/unity.md)) |
| sim PC (Unity 無し) | `ros2 launch minicar_sim sim_host.launch.py camera_backend:=opencv unity_player:=none` |
| 参照線を変える | `python3 tools/make_route.py --delta-max <rad> --plot /tmp/route.png` → `tools/corridor_check.py --route ...` → `route_file:=` |
| 録画 | Unity の表示: `sim_host.launch.py ... record:=~/Videos/run.mp4` / 車が見ている画: `python3 tools/record_video.py --scale 3 --hud` ([docs/unity.md](docs/unity.md)) |
| sim PC (強化学習) | `ros2 launch minicar_sim sim_host.launch.py sim_mode:=lockstep` → `/sim/reset`・`/sim/step` |
| Jetson (sim 接続・教師) | `ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true` |
| Jetson (推論) | `ros2 launch jetracer_stack vehicle_stack.launch.py model_file:=policy.onnx` |
| Jetson (実機) | `ros2 launch jetracer_bridge bridge.launch.py` ＋ 上の推論 |
| スタート | `ros2 topic pub --once /run std_msgs/Bool "data: true"` |

## 何を流用し、何を足したか

| 層 | 既存 sim (M-05) | JetRacer 機 | 扱い |
|---|---|---|---|
| 指令の境界 | `/actuator_cmd` (DriveCommand 正規化値) | `/actuator_cmd` = **`minicar_msgs/ActuatorCmd` (δ rad・v m/s・mode)** | ★凍結 (`cmd_msg:=drive` で旧型も受ける) |
| 物理 | `vehicle_sim` 100 Hz 摩擦限界つき自転車モデル | 同一 ＋ `vehicle_profile` 切替 ＋ **`drive: 4wd_locked`** ＋ サーボ/ESC/モータ模型 | 流用＋改修 |
| 描画 | Unity ＋ ROS-TCP-Connector | 同一 (`course.json` を 224×224・15 Hz で書き出し) | 流用 |
| IMU | MPU-9250 (9 軸・指令由来の簡易値) | **`imu_sim` (物理状態から合成・6 軸)** | ★新規 |
| ブリッジ | `arduino_bridge` | **`jetracer_bridge`** (PCA9685・MPU-6500・300 ms 失効) | 差し替え |
| 学習ラベル | 無し | **`/sim/ground_truth`** (先行注視点の画像座標・横偏差・方位誤差・曲率・区間) | ★新規 |
| 外界センサ・オドメトリ | LiDAR・ToF・車輪速 | 無し (`use_lidar:=false`。車輪速は無い = 積分で距離は出せない) | 削除 |

## 現状 (2026-09-26)

**sim 側は一通り動く。実機側 (較正・学習) がこれから。**

| | |
|---|---|
| ✅ ビルド・単体テスト 17 件 | Jetson (Orin Nano・JetPack 6.2.1・Humble) と sim PC (Ubuntu 22.04.5・AMD GPU) の両方 |
| ✅ 閉ループ (1 台・Unity 無し) | realtime / lockstep とも。`scripts/run_sim_local.sh`・`scripts/smoke_test.sh` |
| ✅ **2 ホスト接続 (P9)** | 1000BASE-T 直結で通った。画像 15 Hz・遅延 64 ms (Unity 描画) / 5 ms (OpenCV)、IMU 100 Hz・8 ms、往路 0.8 ms、教師で 24.0 s/周・衝突 0。[docs/setup.md](docs/setup.md) |
| ✅ **参照線を TT-02 用に引き直した** | 中心線は坂道出口の右ヘアピンで R 0.42 m と δmax 27° の R_min 0.506 m を割っていた。最小曲率で引き直して最小 R 0.556 m・壁余裕 ≥ 0.18 m (`tools/make_route.py` → `config/route_jetracer_tt02.yaml`、launch の既定) |
| ✅ Unity 描画 | JetRacer 用の複製プロジェクト (`unity/MinicarSim`)。224×224・15 Hz、カメラの幾何は profile の 1 か所から (fx≠fy・樽型歪み)、実カメラ寄せの後処理 (周辺減光・ブラー・自動露出・柱)、駐車枠の P1/P2/P3、エピソード乱択化 (照明・床・観戦者・床の白テープ)、参照線のミニマップ表示 |
| ✅ 記録と録画 | `scripts/record.sh` (seed を変えて rosbag を N 本)・`record:=` (Unity 表示を mp4)・`tools/record_video.py` (車が見ている画を mp4) |
| 🟡 sim→real 画像変換 | CUT (FastCUT) を PC の CPU で学習中。記録済みの bag を後から変換する方式で、変換した bag は学習データの一部だけに混ぜる ([tools/sim2real/README.md](tools/sim2real/README.md)) |
| ✅ Jetson の画面をネット越しに | `scripts/setup_vnc.sh` (仮想ディスプレイ) / `--mirror` (普段の画面)。Mac からは SSH トンネル ([docs/setup.md](docs/setup.md)) |
| ✅ **他チームの JetRacer 標準コードを sim で走らせる** | `jetracer_compat` (NvidiaRacecar / jetcam CSICamera の互換クラス)。先頭に 2 行足すだけで、元のコードは無改造。**v0.1.1** として公開 ([docs/for_teams.md](docs/for_teams.md)) |
| 🟡 舵とスロットル | 9/12 の実走ログから推定: **スロットルはマイナスで前進** (中立より下のパルス)、steering −0.22 で直進 (中立 1681 µs)、平均車速 −0.106 → 1.26 〜 −0.130 → 2.45 m/s。車体を持ち上げた実測で確定させる |
| 🟡 カメラの幾何 | 実走画像から推定して反映 (高さ 0.148 m・ピッチ 50.9°・水平 144°/垂直 120°・fy/fx 1.78)。旧値 (0.12 m・12°・120° 正方) は実機と大きくずれていた。**チェッカーボード較正 (`tools/camera_calib.py`) で確定させる** |
| ⬜ **実機の較正** | δmax・舵の端点と実現率・ESC の写像・IMU 5 測定・カメラ。今の値はすべて実走ログや画像からの推定か仮値。**ここが全部の前提** ([docs/calibration.md](docs/calibration.md)) |
| ⬜ policy_net の学習 | 記録 → 学習 → ONNX → TensorRT → sim で陽性対照 → 実機 A/B |
| ⬜ 強化学習 | lockstep の IF はある。Gymnasium ラッパと N 台並列は未着手 ([docs/lockstep.md](docs/lockstep.md)) |

- 数値のうち ★要実測 は暫定値 (`vehicle_profile`・`imu_sim.yaml`・`jetracer_bridge.yaml` の各コメント)。**δmax を実測したら `make_route.py --delta-max <rad>` で参照線を引き直す**
- 4WD 拘束 (windup) は飽和形に直したが係数は ★要較正 (フルロック旋回の減速量を実測して合わせる)

## 開発の進め方 (このリポジトリを直す人向け)

- Jetson (車両側・較正) と sim PC (Unity・学習) の 2 か所から同じ `main` に push する。**push の前に必ず `git pull --rebase`**。同じファイルを同時に直さないよう、担当を分ける (Jetson: `jetracer_common`・`minicar_sim`・`jetracer_bridge`・`jetracer_stack`・tools の較正系 / PC: `unity/`・`jetracer_compat`・`tools/sim2real`・Unity 系の scripts)
- 他チームに渡す版はタグ (`v0.1.1` など) で固定し、Unity プレイヤーは同じ名前の GitHub Release に添付する。`main` を進めても他チームの手元は変わらない。版を上げるときはタグと Release を両方作る
- 数値の定義元は 1 か所: 車両・カメラは `vehicle_profile/jetracer_tt02.yaml`、舵とスロットルの較正は `jetracer_bridge/config/jetracer_bridge.yaml`。値を変えたら `./scripts/test.sh` と `tools/lap_eval.py` で閉ループを確かめる

## ドキュメント

| | |
|---|---|
| [docs/for_teams.md](docs/for_teams.md) | **他チーム向け**: JetRacer 標準のコードをこの sim で走らせる手順と、各数値の信頼度 |
| [docs/architecture.md](docs/architecture.md) | 境界と凍結するトピック・sim を動かす 2 通りの車両ソフト・カメラの幾何・3 つの時計・ノードの契約 |
| [docs/setup.md](docs/setup.md) | 2 ホスト (sim PC ＋ Jetson) の環境構築。`sudo ./scripts/setup_host.sh {jetson\|pc}` で一気に作る → `p9_check.sh` → P9 接続手順 |
| [docs/unity.md](docs/unity.md) | Unity 環境 (プロジェクトの場所・course.json・ビルド・接続・未実装の描画) |
| [docs/docker.md](docs/docker.md) | sim PC の Docker (sim / learner) と踏みやすい 4 つ |
| [docs/calibration.md](docs/calibration.md) | 走行ゼロで取る IMU 5 測定・サーボ/ESC とカメラの較正・陽性対照 |
| [ros_ws/src/jetracer_compat/README.md](ros_ws/src/jetracer_compat/README.md) | 互換クラスの変換 (値 → パルス → δ・v) |
| [tools/sim2real/README.md](tools/sim2real/README.md) | sim→real 画像変換 (CUT) の学習と変換 |
| [docs/lockstep.md](docs/lockstep.md) | 強化学習 IF (sim_mode:=lockstep・Reset/Step・StepInfo) |
