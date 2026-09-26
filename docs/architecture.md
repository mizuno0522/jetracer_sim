# アーキテクチャ — 境界を動かさない

設計書 (詳細版): https://claude.ai/artifact/NBkzJSj8s61NLonsNQwBUG 。ここは実装との対応表。

## 凍結するインターフェース (sim と実機の契約)

左の 5 トピックは両系で完全に同一でなければならない。下の `/sim/…` は sim にしか存在しない。

| トピック | 型 | 周期 | 単位・frame | QoS | sim の出し手 | 実機の出し手 |
|---|---|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` bgr8 | 15 Hz | 224×224、`camera_link` | best_effort d1 | Unity (または vehicle_sim の OpenCV 描画) | `csi_camera_node` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | K・D (plumb_bob) | best_effort d1 | `camera_info_pub` (profile 由来) | `csi_camera_node` |
| `/imu` | `sensor_msgs/Imu` | 100 Hz | m/s²・rad/s、`imu_link`。orientation 無効 (`orientation_covariance[0] = -1`)、加速度は重力込みの生値 | best_effort d1 | `imu_sim` | `jetracer_bridge` |
| `/actuator_cmd` | `minicar_msgs/ActuatorCmd` | 30 Hz | `steer_rad` (左正・±δmax)・`speed_mps` (前進正)・`mode` 0=IDLE 1=RUN 2=ESTOP | reliable d1 | `cmd_shaper` (＋ `failsafe` が ESTOP で上書き) | 同一 |
| `/tf_static` | `base_link → imu_link / camera_link` | — | `jetracer_stack/urdf_gen` が profile と imu_sim.yaml から生成 | 既定 | `robot_state_publisher` | 同一 |

| sim 専用 | 型 | 周期 | 中身 |
|---|---|---|---|
| `/sim/body_state` | `minicar_sim_msgs/BodyState` | 100 Hz | 剛体状態 (運動学的加速度・角速度・roll/pitch・車速・舵角・路面)。`imu_sim` の入力 |
| `/sim/ground_truth` | `minicar_sim_msgs/GroundTruth` | 100 Hz | 学習ラベル: 先行注視点の画像座標 (u, v_px)・横偏差・方位誤差・曲率・区間・真の v・周回・壁余裕 |
| `/sim/render_state` | `Float64MultiArray` (17＋2 ch) | 50 Hz | Unity への姿勢。既存の 17 ch ＋ 予約 `step_id`・`car_id` |
| `/sim/episode` | `UInt32` (latched) | — | エピソードの seed。`imu_sim` がターンオンバイアス等を引き直す |
| `/sim/reset`・`/sim/step` | srv | — | lockstep (docs/lockstep.md) |
| `/odom`・`/tf` (odom→base_link)・`/imu/*`・`/sensors/*` | 旧 sim の互換出力 | — | 推論は購読しない。`odom → base_link` は sim の真値であり実機には無い |

**不変量**: 推論スタックのノードは `/sim/` を購読しない。`scripts/check_no_sim_topics.sh` で機械的に確かめる (`gt_teacher` は sim 専用の例外)。

## 推論スタックの 4 ノード (jetracer_stack)

| ノード | 入力 | 出力 | やること |
|---|---|---|---|
| `policy_net` | `/camera/image_raw`・`/imu` 直近窓 (50) | `/lookahead` (`minicar_msgs/LookAhead` u, v, s) | ONNX Runtime。camera_preproc (正規化・車体写り込みマスク) を同居。model_file が無ければ出さない |
| `gt_teacher` (sim 専用) | `/sim/ground_truth` | `/lookahead` | 教師。模倣学習のデータ生成と、policy_net が無いときの閉ループ用 |
| `cmd_shaper` | `/lookahead` ＋ vehicle_profile | `/actuator_cmd` 30 Hz | 画像座標 → 床面 (カメラ幾何) → Pure Pursuit で δ。v = s × v_max。レート制限。**車両の数値はここに集まる** |
| `failsafe` | 画像・IMU・指令の到着時刻 | `/actuator_cmd` (mode=ESTOP) | 画像 150 ms・IMU 50 ms・指令 100 ms の途絶 (連続 2 ティック)、NaN で ESTOP。ブリッジの 300 ms とは別の層 |

安全の層 (上から): ① RC マルチプレクサの手動切替 (試走のみ・本番は受信機を外す) → ② `jetracer_bridge` の 300 ms 失効 (スロットル中立・舵は保持) → ③ `failsafe` の途絶検出 (ESTOP を 1 s 保持)。`vehicle_sim` も同じ 300 ms 失効と ESTOP 保持を実装している。

## カメラの幾何は 1 か所で定義する

`vehicle_profile.camera` (fx・fy・cx・cy・k1・k2・取付の高さ・ピッチ) が唯一の定義元で、式は `jetracer_common/cam_geom.py` (OpenCV plumb_bob)。これを読むのは:

| 使う側 | 用途 |
|---|---|
| `vehicle_sim` | `/sim/ground_truth` の先行注視点 (u, v) の計算と OpenCV 描画 |
| Unity (course.json 経由) | センサーカメラの投影と歪み |
| `camera_info_pub` / `csi_camera_node` | `/camera/camera_info` の K・D |
| `cmd_shaper` | 画像座標 (u, v) → 床面 → Pure Pursuit |

**ラベルを作る式と、推論時に画像座標を床に戻す式が同じ**であることが、「ラベルの点が走路の上に来る」条件。どれか 1 つだけ値を変えると sim-to-real の穴になるので、値は profile にだけ書き、`tools/camera_calib.py` (チェッカーボード) の出力をそのまま貼る。物理的にあり得ない歪みと画角の組み合わせは `CamGeom` が起動時に拒否する。

## 3 つの時計

| 時計 | レート | 役割 |
|---|---|---|
| `vehicle_sim` 物理 | 100 Hz | 姿勢の唯一の定義元。既存のまま (10 ms ティック) |
| `imu_sim` 内部 | 1 kHz | 剛体状態を補間し、てこ腕・振動を足し、DLPF を通す。**50 Hz 超の振動と折り返しを出すため** |
| `/imu` 出力 | 100 Hz | 実機の I2C 読み出し周期。遅延 4 ms ＋ ジッタ 2 ms・取りこぼし |
| `/camera/image_raw` | 15 Hz | 実機の取り込みに合わせる。画像と IMU の相対遅れが学習の当たり外れを決める |

## IMU は物理状態から作る (P8)

```
/actuator_cmd → サーボ模型 (τ・レート制限) / ESC 模型 (不感帯・ブレーキ帯・後退は中立 120 ms 経由) / モータ上限 F(v)
             → vehicle_sim 摩擦円 (4 輪 μ・荷重移動・4WD 拘束の突っ張り・横滑り・でこぼこ・坂)
             → /sim/body_state (a_kin・ω・roll・pitch・路面)
             → imu_sim: 重力投影 → てこ腕 → 振動 (モータ高調波＋路面 rms×速度) → DLPF → 100 Hz 間引き (遅延・ジッタ・欠落)
                        → ターンオンバイアス＋ランダムウォーク＋温度 → スケール・軸ずれ → 白色雑音 → 飽和・16 bit 量子化
             → /imu (センサ軸: x=左 y=前 z=下。取付 rpy=[180,0,90])
```

車両の数値は `ros_ws/src/minicar_sim/config/vehicle_profile/jetracer_tt02.yaml` にだけ書く (`jetracer_common.profile` が各ノードへ配る)。IMU の数値は `config/imu_sim.yaml`。★要実測 の欄は docs/calibration.md の測定で埋める。

## 4WD 拘束 (`drive: 4wd_locked`)

シャフト 4WD・センターデフ無しでは前後輪の回転が拘束される。旋回中は前輪の経路が後輪より 1/cosδ 長いので前輪が引きずられ後輪が押す。`vehicle_sim._integrate_motion` では
巻き込み量 (1/cosδ − 1) を `slip_elastic` で正規化した割合 `windup` だけ前後力を使ったとみなし、摩擦円の横方向の余力を √(1−windup²) に減らし (小回りで曲がりにくい)、`windup_drag × μg × windup` を抵抗として捨てる (突っ張りの減速)。係数 `drivetrain_lock.{windup_gain, windup_drag}` は ★要較正 (小回りでの減速量を実測して合わせる)。

## 参照線と R_min

R_min = L / tan δmax。TT-02 (WB 0.257・δmax 27° 暫定) で 0.506 m。`tools/corridor_check.py` の机上判定では、コース中心線の最小曲率半径 0.42 m (坂道の右 180°・(7.3, 3.8) 付近) と、既存 sim の参照線 route.yaml の 0.39 m がこれを割る。**δmax を実測してから参照線を引き直す** (`vehicle_sim` の `route_file:=` で差し替え可)。
