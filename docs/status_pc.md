# sim PC 側の状況 (2026-09-26)

PC 側セッションが main=3cb8872 を clone して確認した結果と、Jetson 側への提案。環境の事実は docs/setup.md・docs/unity.md に反映済み。

## 確認結果

| 項目 | 結果 |
|---|---|
| `scripts/build.sh` | 7 パッケージ 28 s で成功 (Humble desktop・PYTHONNOUSERSITE=1) |
| `scripts/test.sh` | 14 passed |
| 1 台閉ループ (opencv・teacher・auto_run) | /imu 98.7 Hz・/camera/image_raw 15.0 Hz・/actuator_cmd 30.0 Hz・/sim/ground_truth 99.9 Hz。教師で周回 (cte ≈ 0.004 m)。`check_no_sim_topics.sh`: cmd_shaper/failsafe ok |
| Unity (プレイヤーコピー方式・`tcp_port:=10001`) | /camera/image_raw 224×224 bgr8 15.0 Hz・/camera/camera_info 15.0 Hz・/imu 99.9 Hz・/actuator_cmd 30.0 Hz。endpoint に `Connection from 127.0.0.1`。画像の frame_id は `camera` (Unity 側の値。実機 `camera_link` と揃えるなら SimBridge.cs の 1 行) |
| `rmw_cyclonedds_cpp` | **未導入** (sudo が要るのでユーザーに依頼中)。上の確認は `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` で実施。`scripts/sim_env.sh` は cyclonedds を強制するので、導入までは source 後に上書きが要る |
| `ros_tcp_endpoint` | このリポジトリの ros_ws には無い。素で動かすには `source ~/ros2_unity_ws/install/setup.bash` を追加で source する (docs/unity.md の記載どおり)。`sim_host.launch.py camera_backend:=unity` は endpoint が無いと launch 自体が例外で止まる |
| 終了時の traceback | SIGINT で imu_sim_node / camera_info_pub / vehicle_sim が exit code 1 (rclpy の ExternalShutdownException を except していない)。動作には影響しないが、launch ログが赤くなる。`except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException)` で消える |

## 追加したスクリプト (提案・このブランチ)

| ファイル | 内容 |
|---|---|
| `scripts/stop_sim.sh` | 起動物を全部止める。`pkill -f` のパターンは実行ファイルのパスで先頭固定 (`^`)。緩い文字列だと同じ文字列を含む自分の端末まで殺す (実際に踏んだ) |
| `scripts/run_sim_local.sh` | `set -m` を追加。非対話シェルのバックグラウンド job は SIGINT 無視を継承するので、`kill -INT` が launch に届かず `wait` が永久に止まる (実際に踏んだ)。trap で stop_sim.sh も呼ぶ |
| `scripts/setup_unity_player.sh` | ビルド済みプレイヤーを `~/jetracer/unity/player` にコピーし、`export_course.sh` の course.json を差し替える (再ビルド不要の手順の自動化) |
| `scripts/smoke_test.sh` | 単体テスト → 閉ループ起動 → /imu・/camera・/actuator_cmd のレートと教師の走行 (10 s で Δs > 3 m, |cte| < 0.3) → /sim/ 購読禁止 |

## Unity (JetRacer 用の複製プロジェクト)

- 複製: `~/jetracer/unity/MinicarSim` (Assets/Packages/ProjectSettings のみ。Library は初回ビルドで生成)。ビルド済み: `~/jetracer/unity/player`。
- 追加: 駐車枠の P1/P2/P3 マーク (`LabelTexture.cs` 5×7 ドット文字・テープ色・走路側から読める向き。`course.json` の `parking_slots[].name` を描く)。
- 進行中 (ユーザー要望): 実カメラ画像 (2026-09-12 手動走行ログ 11,271 枚) に描画を寄せる。実画像の統計: 全体平均 BGR (110,112,116)・床 (110,113,115)・白壁 (195,193,190)・赤帯 (67,70,141)・周辺減光 中心/端 = 119/102・画面下端に黒い柱 2 本 (列 54–57・121–125、行 209–223)。sim 現状は床が暗く (92,95,95)・背景が一様灰色・壁が細く暗い。後処理 (樽型歪み・周辺減光・ブラー・柱) と材質/照明の調整を Unity 側で入れる。

## PC 側で分かったこと (Jetson 側の設計へのフィードバック)

- imu_model: PC 草案との差分で「こちらに無い汚れ」は見当たらない (重力投影・てこ腕・DLPF→間引き・量子化・飽和・遅延/ジッタ/取りこぼし・温度・乱択化は両方にある)。確認したい点だけ: (1) 停車中のモータ高調波の振幅 (v→0 で位相が止まると定数 sin が偽のバイアスになる。草案は min(1, |v|/0.2) の包絡を掛けた)、(2) 高調波の位相を push を跨いで連続にしているか、(3) 白色雑音 σ を √bw で取っている (草案は √fs_out)。Allan 分散の実測が入れば消える差。
- 既存 sim の sweep (DOMAIN 0) と同居して 42 で問題なく分離できた。
