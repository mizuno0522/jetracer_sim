# 較正 — 走行ゼロで取れるものから取る

設計書「較正と検証」「パラメータ」タブ。★要実測 の暫定値は `config/vehicle_profile/jetracer_tt02.yaml`・`config/imu_sim.yaml`・`jetracer_bridge/config/jetracer_bridge.yaml` の各コメントにある。**「寸法は図面から、特性は実測から。推測で埋めない」**。

順番: 走らせずに測れるもの (IMU 5 測定・6 面静置・サーボ端点) → 持ち上げて測れるもの (振動・舵の実現率) → 走って測るもの (モータ力・δmax・車速写像)。

## 1. IMU の 5 測定 (実機・車を走らせない)

実機の `/imu` を rosbag に取る (`ros2 launch jetracer_bridge bridge.launch.py` を上げた状態で `ros2 bag record -s mcap /imu`)。CSV (`t,ax,ay,az,gx,gy,gz`) でもよい。

| # | 測定 | やり方 | 出るパラメータ (`imu_sim.yaml`) | 所要 | ツール |
|---|---|---|---|---|---|
| 1 | 静止ログ | 電源を入れて動かさず **10 分以上** | `noise.accel_nd / gyro_nd / accel_rw / gyro_rw` | 10 分 | `tools/imu_allan.py <bag>` → yaml を印字するので貼る |
| 2 | 持ち上げ回転 | タイヤを浮かせ、スロットル数段階で回す。**ファンだけ** (モータ停止) も 1 本 | `vibration.motor_hz_per_mps / harmonics / amp_ms2 / gear_mesh_hz` | 15 分 | `tools/imu_spectrum.py <bag> --speed <指令車速>` を段階ごとに |
| 3 | 6 面静置 | 各面を下にして静置し重力ベクトルの読みを取る | `scale.accel_err_sigma / misalign_deg_sigma`・ゼロ点 (mount.rpy_deg の確認) | 10 分 | 手計算 (‖a‖ と 9.81 の差がスケール、直交からのずれが軸ずれ) |
| 4 | ターンオンばらつき | 電源の入れ直し **10 回以上**、起動直後 10 s の平均の分布 | `turn_on_bias.accel_sigma / gyro_sigma` | 20 分 | 静止ログの先頭を切って標準偏差 |
| 5 | 昇温 | 起動から 30 分、温度レジスタと出力を同時記録 | `temp.rise_c / tau_s / accel_coef / gyro_coef` | 30 分 | 温度と出力の直線あてはめ |

`imu_allan.py` は τ=1 s の Allan 偏差を雑音密度 N、最小点 / 0.664 をバイアス不安定性、τ=3 s の傾き +1/2 からランダムウォーク K として出す。**サンプルは 100 Hz で独立に取る** (2026-09-12 のログは画像に従属した 15 Hz で、折り返しを含む。`vibration.surface` の値はその 15 Hz ログ由来なので 100 Hz で取り直すと帯域の内訳が出る)。

先に確定させておくこと:
- **WHO_AM_I**: 0x68 なら MPU-6050、0x70 なら MPU-6500。雑音特性が違う。`jetracer_bridge` が起動ログに出す。
- **レンジ**: 実データは ±2 g で `az` が飽和した (5 セッションで 130 フレーム)。`accel_g: 4` にし、`imu_sim.yaml` の `range` と `jetracer_bridge.yaml` の `imu` を**必ず一致**させる。
- **取付**: 基板は水平・部品面が下・x 軸が車の左 (確認済み)。センサ軸 x=左・y=前・z=下 → `mount.rpy_deg: [180, 0, 90]`。位置 (`mount.xyz`) は重心から前 5〜10 cm・上 5〜8 cm の推定で、てこ腕の項が横加速度の 15 % 程度になる。**実測して固定する**。
- 配線はジャンパ線のピン差し。振動で I2C が瞬断する典型なので短く結束 (tegra-i2c タイムアウトの候補・設計 未決①)。

## 2. サーボと ESC (`jetracer_bridge.yaml`)

車体を持ち上げ、`dry_run: false` で `ros2 topic pub /actuator_cmd minicar_msgs/ActuatorCmd ...` を手で打って目で確かめる。

| 項目 | やり方 | 入れる先 |
|---|---|---|
| 舵の端点 | µs を 1000 → 2000 まで刻んで、リンケージが当たる直前を min/max、直進を center に | `steering.pulse_us` |
| 舵の実現率 | δ を数段階で与え、前輪の切れ角を分度器 (または上から写真) で測る。左右別々に | `steering.map: [δ_rad, µs, ...]`。空なら線形 |
| **δmax** | 上の表の端。左右の小さいほう | `vehicle_profile.delta_max_rad` (暫定 0.47 = 27°)。決まったら `python3 tools/make_route.py --delta-max <rad>` で参照線を引き直し、`tools/corridor_check.py --route` と `tools/lap_eval.py` で確認 (設計 未決⑦) |
| 中立 | 2026-09-12 のログでは直進時に運転者が左へ 0.22 当てていた = 機械中立がずれている | リンケージで合わせるか `center` を動かす。**直したら学習データは取り直し** |
| ESC の中立と不感帯 | TBLE-02S の設定手順で neutral を合わせ、動き出す µs を前後で記録 | `throttle.pulse_us`・`vehicle_profile.esc.deadband_mps` |
| 後退ロック | 前進中に後退を入れると中立を経由しないとブレーキになる。中立を挟む時間を測る | `esc.reverse_via_neutral_ms` (暫定 120) |

## 3. 走って測るもの (`vehicle_profile`)

| 項目 | やり方 | 入れる先 |
|---|---|---|
| 車速写像 | µs を数段階で固定して直線を走り、区間タイムから v。実走ログでは **スロットル 0.02 の差で速度 2 倍** (不感帯の直上) だったので細かく刻む | `throttle.map_v_us` |
| モータ力 | 直線で 0 → 2.0 m/s の到達時間 t。F = m·2.0/t | `motor.force_n`・`accel_time_constant_s` |
| 4WD の突っ張り | フルロックで定速旋回し、直線との速度差 | `drivetrain_lock.windup_*` |
| ロール/ピッチ | 旋回中の重力漏れ (2026-09-12 のログで加速度計側が 8〜20 % 高かった分) | `suspension.roll_per_ms2 / pitch_per_ms2` |
| 質量・タイヤ径・トレッド | 秤・ノギス | `mass_kg`・`tire_dia_m`・`track_m` |

## 4. カメラ

**まず `tools/camera_calib.py`** (チェッカーボード)。内部パラメータ (fx・fy・cx・cy・k1・k2) と取付 (高さ・ピッチ・前後位置) が出て、`vehicle_profile.camera` にそのまま貼れる。使い方は先頭のコメント。**実機と同じ経路 (`csi_camera_node` → `/camera/image_raw` 224×224) の画像で測る**こと。

実機が無いときの第一近似は `tools/fit_camera_from_images.py <走行ログ>` (実画像の壁と course.py の壁の配置から同時推定)。2026-09-26 に 9/12 のログで推定した値が今の profile に入っている。縦 (高さ・ピッチ・fy) は安定して決まるが、横 (fx) と縦横比は画像の壁だけでは決まらないので、縦横比は取り込み経路から固定した (jetcam 既定 640×480 要求 → Argus は IMX219 の mode 4 = 1280×720 を選ぶ → 224×224 に潰す → fy/fx = 1.78)。


| 項目 | やり方 | 入れる先 |
|---|---|---|
| 切り出しか縮小か | 実機の nvarguscamerasrc パイプライン (センサ解像度→224×224 の方法) | `camera_node.square_mode`・`vehicle_profile.camera.hfov_deg` |
| 画角・歪み | チェッカーボード (OpenCV `calibrateCamera`)。実データは樽型で魚眼ではない | `camera.hfov_deg`・`camera.distortion` (Unity は未実装) |
| 取付 | 高さ・後軸からの前後位置・ピッチ | `camera.mount_height_m / mount_x_m / pitch_deg` |
| 露光時刻 | GPIO で LED を点灯した時刻と、点灯が写った最初のフレームの PTS の差 | `csi_camera_node` の stamp 補正 |
| カメラ↔IMU の時刻ずれ | ジャイロのヨーレートと画像のオプティカルフローの相互相関。ピーク位置がずれそのもの (Kalibr と同じ原理)。走行ログのたびに取れる | `imu_sim.latency.base_ms` の妥当性確認 |

## 5. 陽性対照 (学習後・sim で)

「入れたつもり」を見つける。`docs/lockstep.md` か realtime のどちらでも。

| 対照 | やり方 | 期待 | 外れたら |
|---|---|---|---|
| IMU ゼロ | `/imu` を全部 0 にして走らせる | 成績が落ちる | IMU は最初から使われていない (画像だけで走っている) |
| 乱択化 0 / 3 倍 | `domain_rand.scale: 0.0` と `3.0` | 0 で少し良く、3 で崩れる | 3 で崩れないなら幅が狭すぎる、0 で崩れるなら過学習 |
| 描画バックエンド | `camera_backend:=opencv` と `unity` | 同じ参照線で同程度 | 見た目に過学習 |
| 遅延 | `latency.base_ms` を 2 倍 | 少し悪化するが走る | 時刻の整合に依存しすぎ |

`tools/corridor_check.py --profile jetracer_tt02 --route ros_ws/src/minicar_sim/config/route_jetracer_tt02.yaml` は走行ゼロで参照線の通過可否を出す机上判定。δmax を実測したら `vehicle_profile.delta_max_rad` を直し、`tools/make_route.py` で参照線を引き直してから再実行する。
