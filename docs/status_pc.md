# sim PC 側の状況 (2026-09-28 朝 時点)

sim PC (Ubuntu 22.04.5・Humble desktop・AMD 内蔵 GPU = CUDA 無し・Unity 6000.0.83f1) で確かめたことと、PC 側の担当
(`unity/`・`jetracer_compat`・`tools/sim2real`・Unity 系の scripts) の現状。環境構築の手順そのものは [setup.md](setup.md)・[unity.md](unity.md)。結果の画像と動画は [results/2026-09-26](results/2026-09-26/README.md)・[results/2026-09-27](results/2026-09-27/README.md) (方策)・[results/2026-09-27/real_log.md](results/2026-09-27/real_log.md) (実機ログ)。

## いまの状態

| 項目 | 状態 |
|---|---|
| ビルド・単体テスト | 新規 clone から `colcon build` 8 パッケージ・`scripts/test.sh` とも通過 (v0.1.1 相当) |
| 1 台閉ループ (`scripts/smoke_test.sh`) | PASS。教師で 50 s に 2 周・\|cte\| p95 0.10 m・衝突 0、/imu 100 Hz・画像 15 Hz・/actuator_cmd 30 Hz |
| 2 ホスト直結 (P9) | 完了 (下の節)。PC の enp5s0 は NetworkManager の `jetracer-link` (192.168.10.2)。既存 sim 用の `sim_link` (192.168.0.20) とは排他で、戻すときは `sudo nmcli con up sim_link` |
| DDS | `rmw_cyclonedds_cpp` 導入済み。`scripts/sim_env.sh` が有線のリンク状態で `~/cyclonedds-wired.xml` / `~/cyclonedds-local.xml` を自動で選ぶ |
| Unity 中継 | `scripts/setup_ws.sh` が `ros_ws/src/ros_tcp_endpoint` に取り込む (この PC は `~/ros2_unity_ws` の checkout を symlink) |
| Unity プレイヤー | ソースはリポジトリの `unity/MinicarSim`、ビルドは `scripts/build_unity.sh` → `~/jetracer/unity/player`。他チーム向けはビルド済みを [Release v0.2.0](https://github.com/mizuno0522/jetracer_sim/releases/tag/v0.2.0) に置き `scripts/get_unity_player.sh` で取る |
| カメラ幾何 (Unity) | `course.json` の `fx, fy, cx, cy, k1, k2`・`render_tan_*` で描く (下の節)。推定値 (高さ 0.148 m・下向き 50.9°・水平 144°/垂直 120°・fy/fx 1.78) で OpenCV 描画と一致 |
| 他チームの JetRacer 標準コード | `jetracer_compat` で無改造のまま sim で走る (下の節)。v0.1.1 として公開 |
| sim→real 画像変換 | 3 回目 (標準 CUT + 形を保つ損失) で合格。F 値 0.832 (下の節・[sim2real.md](sim2real.md))。9/27 の実画像を足した追加学習 (cut_004・005) は sim に無い赤い壁を描いて不合格。A 側に壁に寄った場面が無いのが原因で、カメラ較正のあと撮り直す。当面は cut_003 |
| 実機ログ 9/27 | 8 本・約 7 分 (画像 60 fps・IMU 120 Hz 独立・ジャイロ付き)。IMU の静止雑音・ゼロ点・駆動系の振動振幅が実測に。2 本で末尾欠け (1 本は電源断型)、±2 g・±250 dps で飽和あり ([results/2026-09-27/real_log.md](results/2026-09-27/real_log.md)) |
| sim の物理 (壁・車どうし) | **壁は剛体** (押し戻し・法線衝撃・摩擦、通り抜けの修正込み)。2 台走行の相手 (`/sim/rival_state`) とは車体どうしの剛体衝突 (下の節) |
| 2 台走行・ゴースト | 相手の走りを記録して再生する `tools/race/ghost_replay.py`。2 台の位置の CSV (`RACE_POSE_LOG`)。M-05 の 2 台レースは minicarbattle2026 の `unity/race2.sh`・`race_ghost.sh` |
| policy_net の学習 | **sim の閉ループで合格**。画像 (＋IMU) だけで、学習に使っていない 7 seed を全部完走・衝突 0 (下の節・[tools/policy](../tools/policy/README.md))。実機は未確認 |

## カメラ幾何の Unity 実装と確認

- `SimBridge.ApplyIntrinsics`: `render_tan_x0/x1/y0/y1` の範囲を off-axis 透視 (`Matrix4x4.Frustum`) で描く。fy ≠ fx もこれで扱う。
- `SensorPost.shader`: 出力画素 → 正規化座標 → 不動点反復 8 回で歪みを解く → ピンホールで描いた RT をサンプル。式は `jetracer_common/cam_geom.py` と同じ。
  旧 `realism.k1/k2/zoom` は向きが逆 (糸巻き型) だったので廃止。
- 確認: start 姿勢で `camera_backend:=opencv` と `unity` を撮り比べ、Sobel エッジの一致 (F 値)。

| カメラの値 | F 値 (許容 2 px) | F 値 (許容 4 px) |
|---|---|---|
| 仮の歪みあり (vfov 90・k1 −0.02・pitch 30) | 0.69 | 0.78 |
| 実走画像からの推定値 (ca98075) | 0.76 | 0.84 |

  残りの差は床の雑音粒・観戦者など描画内容の違いで、地平線と壁の下端は重なる。
- 実画像との並べ比べで、構図 (地平線が上から約 14 %・床が大半) が近くなったことをユーザーに確認してもらった。
- AIC 表示の下段「CAMERA」は、表示用の横長カメラではなく実際に配信している 224×224 (後処理後) をそのまま出す。
  以前は別の横長カメラを映していて、「画角が違う」と見える一因だった。

## jetracer_compat (他チームの JetRacer 標準コードを sim で)

- `import jetracer_compat; jetracer_compat.install()` で `jetracer.nvidia_racecar` / `jetcam.csi_camera` が sim 版になる。
- 変換: 値 × gain + offset → ServoKit のパルス 1500 + 750·u µs → `jetracer_bridge.yaml` の較正の逆写像 → ActuatorCmd (30 Hz 再送)。
- 確認 (9/12 ロガーの gain −0.55・offset 0.12・throttle_gain 1.0): throttle −0.12 → 2.04 m/s 前進、steering −0.22 で直進 (ヨーレート 0.00)、±0.3 で左右に曲がる、画像 15 Hz。
  Release から取ったプレイヤーでも同じ結果。
- 終了時のクラッシュ (`terminate called without an active exception`) は、spin を `spin_once` のループにして終了時に止めてから rclpy を閉じる形で解消。

## sim→real 画像変換 (CUT、PC の CPU)

- 道具: `tools/sim2real/` ([README](../tools/sim2real/README.md))。venv は `scripts/setup_sim2real.sh` (`python3-venv` が無いこの PC では virtualenv で作る)。
- 学習データ: A = Unity 描画 (推定カメラ値・床テープの乱択化入り) 8 本 × 60 s から 3,700 枚、B = 9/12 の実画像 11,271 枚。
- 1 回目 `cut_001` (FastCUT・128 切り出し・0.21 s/反復) は**失敗**。19,000 反復で床に緑・水色の斑点が出た。色を保つ恒等 NCE を省く FastCUT は崩れやすく、
  128 の切り出しも推論時の 224 全体と構図が合わなかった。
- 2 回目 `cut_002` (標準 CUT・176 切り出し・0.56 s/反復・16,000 反復) は**不合格**。色と質感は実画像に近づいたが、床の奥に実物に無い明るい塊を描き、
  水色の滑り板や壁の縁が消えることがある。変換前後のエッジ F 値 0.559 (目安 0.6、未学習モデル 0.856)。
- 3 回目 `cut_003` は**合格**: cut_002 の重みから、形を保つ損失 (1/4 に縮めた輝度の勾配を揃える `--lambda-struct 10`) と λ_NCE 2 で 8,000 反復 (約 1.2 時間)。
  エッジ F 値 0.832 (下半分 0.676)。滑り板の色・壁と床の位置が保たれた。学習データ 8 本を変換して `bags/ep_30*_s2r` に。
- 自分の実画像で作り直す手順は [sim2real.md](sim2real.md)。
- 推論は ONNX で約 57 ms/枚 (CPU)。走行中に挟むと画像の遅れが増えるので、記録済み bag を後から変換する (`convert_bag.py`)。
- 変換した bag は学習データの一部だけに混ぜる (9/12 の会場に寄せきらない)。

## policy_net の模倣学習 (PC の CPU) — 2026-09-27

- 道具: `tools/policy/` ([README](../tools/policy/README.md))。教師 (gt_teacher) の注視点に揺らぎを足して 30 本 × 60 s 記録し、
  正解は真値 (/sim/ground_truth) から作る。半分 (奇数 seed) は cut_003 で実画像風に変換。ResNet18 ＋ IMU の 1 次元畳み込み、CPU で 1 エポック約 13 分。
- **policy_001** (8 エポック): 学習に使っていない seed 900〜906 の 7 本すべてで完走・衝突 0。ラップ 24 s は教師と同じで、横偏差 (\|cte\| rms 0.025〜0.034 m) は教師 (0.050 m) より小さい。
- 目隠しテスト: 画像を 0 にすると走れない (衝突 36 回)、IMU を 0 にしてもほぼ同じ → 方策は画像で走っていて、IMU はほとんど使っていない。
- 実画像 (9/12 の実走ログ、人の操縦) で予測した注視点と人の舵の相関は +0.21〜0.23。曲がる向きは概ね揃うが、左の曲がりの読みが弱い。
- 改善 1 回目 **policy_002** (強い拡張: ぼけ・白飛び・遮り): 実画像の相関が +0.11 に下がった → 不採用。
- 改善 2 回目 **policy_003** (学習 bag を全部変換して追加学習 3 エポック): 実画像の相関 +0.29〜0.30、sim 閉ループも 7 本完走 (ラップ 23.5 s)。
  ただし変換器が同じ 9/12 の会場の画像で学習しているので、その会場に寄ったぶんを含む。
- モデル: `~/jetracer/runs/policy_00{1,3}/policy.onnx` ＋ `policy.onnx.data` (2 つで 1 組、opset 18)。git には入れていない。
- 前夜の評価が 0 % だった原因: 評価スクリプトが launch に `model_file:=` (空) を渡し、launch 全体が引数エラーで起動していなかった。引数を外して直した。

## sim の修正 (2026-09-27 夜、M-05 の 2 台レースと並行)

M-05 (minicarbattle2026) の MPPI 対 Pure Pursuit の 2 台レースを PC ⇔ Jetson で回しながら見つけた sim の問題を、
両方の sim に入れた。M-05 側の詳細は minicarbattle2026 の `docs/unity_sim.md`「2 台レース」。

| 修正 | JetRacer (このリポジトリ) | M-05 (minicarbattle2026) |
|---|---|---|
| 壁を剛体に (以前は余裕 < 0.10 m で衝突の印を立てるだけで、壁を突き抜けた) | 8473c5b。TT-02 の車体 3 円 (後軸から 0 / 0.13 / 0.26 m、半径 0.11)。壁に触れている間も collision | 9/27 昼に Jetson 側で導入済み |
| 壁の通り抜け (角に斜めに速く当たる・薄い仕切りで円の中心が壁の線を越えると、押し戻す向きが逆になり向こうへ抜けた) | c712d47 | 7c33cb2 (Jetson 側) |
| スタートライン 1/2/3 を常設 (p.21 の図の位置 2.56 / 4.36 / 6.16 m) | ef68f0f | 38fe480 |
| 車体の見た目のロール・ピッチを実物並みに (横 1 g で約 1.5°) | ef68f0f | 38fe480 |
| 車どうしの衝突 (車体に収まる円どうしの剛体衝突、同じ質量として半分ずつ) | 8384f20 (相手の姿勢が届いたときだけ) | 38fe480 (`race_physics:=true`) |
| 相手を測距センサに写す (LiDAR・超音波・ToF の仕様の視野・遮蔽) と LiDAR 検出からの `/opponent_info` | 対象外 (カメラと IMU だけ。相手は Unity がカメラに描く) | 38fe480。`/opponent_info` は 54534ac から既定で出さない (実機は Jetson の検出ノードが `/scan` から作る。`RIVAL_OPPONENT_INFO=1` で出す) |
| ゴーストの再生開始の判定 | 自車の `/sim/render_state` の速度 | 115f5a3 から同じ (`/odom` は車両側の推定なので読まない) |
| ゴースト (記録した相手の走りを再生) | `tools/race/ghost_replay.py` | `unity/race_ghost.sh` |
| レース表示では乱択化の白テープを出さない | 対象外 (学習データ用に残す) | 04f8156 |
| **制動中の摩擦円** (横が限界に張り付くとブレーキが 0 になり、一定速度のまま外へ膨らんで壁に当たった) | 2026-09-28。制動要求のときは前後＋横の要求を同じ比で縮める (楕円の飽和) | 同じ修正 (M-05 の真値なし検証、決勝モードの坂道ヘアピン出口の正面衝突で発見) |
| 制動中の車輪速の向き (`brake_slip_underspeed`。従来は制動中も車輪が速く回る扱いで、最大 10 倍) | 対象外 (車輪速センサ無し) | 既定を true に |

- **制動中の摩擦円** (2026-09-28): 旧モデルは横方向を優先し、横の要求が限界を超えると `util_y=1` → 前後に使える上限 `a_x_cap = a_x_pure·√(1−util_y²) = 0` で、
  ブレーキ指令を出しても全く減速しなかった (M-05 で速度 2.98 m/s のまま 1.4 s 惰性で走り、ヘアピン出口の壁に正面衝突)。
  実車では滑っているタイヤの摩擦は滑る向きに働くので、ブレーキを掛ければ減速し、そのぶん曲がらなくなる。
  制動要求 (`a_cmd < 0`) で前後＋横が摩擦円を超えるときだけ、両方を同じ比で縮める。加速側は従来どおり (FF のアンダー・4WD の突っ張りの再現を変えない)。
- **推論スタックが読んでよい入力** (2026-09-28): `scripts/check_no_sim_topics.sh` を「`/sim/` を読まない」から「許可一覧 (`/camera/image_raw`・`/imu`・`/lookahead`・`/run`・`/actuator_cmd`) だけを読む」に変えた。
  sim は M-05 と共通の vehicle_sim なので `/odom`・`/lane_info`・`/imu/yaw`・`/perception/ground_speed` などにも真値を出しており、`/sim/` を避けるだけでは検出できない
  (M-05 で、スタックがこれらの真値で走っていたのが見つかった)。JetRacer のスタックはいまもカメラと IMU だけを読んでいる。
- 乱択化の白テープは走路を横切る向きで 0〜6 本置く (実会場の床のテープをまねた学習用)。スタートラインと見分けがつかないので、レース表示 (M-05) では 0 本にした。
- 確認: smoke test PASS (2 周・衝突 0)。画像を消した方策で壁に当て続けても、コース外 2 回 → 0 回 (以前は実際に壁を抜けていた)。
  教師で走る JetRacer に M-05 の記録のゴーストが後ろから追いつき、接触して押された (v 1.80 → 2.50)。

## 踏んだ落とし穴 (PC)

| 症状 | 原因 | 対処 |
|---|---|---|
| 自分の端末が落ちる | `pkill -f "<文字列>"` や `kill $(pgrep -f "<文字列>")` がその文字列を含む自分のシェルにも一致 | `scripts/stop_sim.sh` は実行ファイルのパスで先頭固定 (`^`)。手で止めるときは `ps` で PID を確かめてから |
| スクリプトから上げた launch が Ctrl-C で止まらない | 非対話シェルの `&` job は SIGINT 無視を継承 | `set -m` を付けてから起動 (`run_sim_local.sh`・`smoke_test.sh`・`record.sh`) |
| `git pull --rebase` が「unstaged changes」で止まる | Unity のビルドが `Assets/Scenes/Minicar.unity` を毎回書き換える | ビルド後は `git checkout -- unity/MinicarSim/Assets/Scenes/Minicar.unity` |
| venv が作れない | `python3.10-venv` が未導入 (sudo が要る) | `setup_sim2real.sh` が virtualenv に切り替える |
| Unity の白い板が灰色に沈む | 影側の壁には環境光しか当たらない | `realism.ambient_gain` 1.8 |
| Unity がちらつく・白線が増えて見える | 起動に失敗した sim と Unity が残り、同じドメインで 2 つ動いていた | 起動前に `ps` で残りを確かめる。レーススクリプトは後始末で名前で止める |

## P9 (2 ホスト直結) — 2026-09-26 完了

PC (192.168.10.2, enp5s0) ↔ Jetson (192.168.10.3)。`setup_host.sh pc --nic enp5s0` → `p9_check.sh` 9/9 → ufw で 192.168.10.0/24 を許可 (NTP が落ちていた) → chrony 同期 0.1 ms。
Jetson の教師スタック → PC の物理で 3 周 (24.0 s/周・衝突 0・cte p95 0.10 m)。遅延の実測は docs/setup.md の表 (画像 OpenCV 5 ms / Unity 64 ms、/imu 8〜12 ms、/actuator_cmd 往路 0.8 ms)。

## データ記録

`scripts/record.sh [本数] [秒/本] [--unity]`: seed を変えながら教師で走らせ、7 トピック (画像・camera_info・/imu・/actuator_cmd・/lookahead・/sim/ground_truth・/sim/episode) を mcap で `bags/ep_<seed>_<日時>/` に。同名 .json に seed・秒数・git rev。
Unity は `/sim/episode` で照明・床の色味・観戦者を引き直す (`camera.realism.episode_*`)。

## Unity 画面の録画 (`record:=`) — PC で確認 2026-09-26

`ros2 launch minicar_sim sim_host.launch.py camera_backend:=unity unity_player:=... tcp_port:=10001 record:=~/Videos/run.mp4`
で、Jetson の教師で走らせながら録画。mp4 は再生可 (断片化 mp4 なので `stop_sim.sh` の SIGINT で止めても壊れない)。
`record:=` 無し (空文字) のときは録画しない (ffmpeg も起動しない) ことも確認。

録画中の `/camera/image_raw` (PC ローカル 30 s、`tools/rate_probe.py`):

| 条件 | 画像 | 間隔 最大 | 遅延 (sim 時刻 → 受信) 平均 / 最大 | /imu |
|---|---|---|---|---|
| 録画なし | 15.0 Hz | 71 ms | 59.7 / 90.0 ms | 99.6 Hz |
| record_width 1280 | 15.0 Hz | 72 ms | 59.8 / 83.3 ms | 99.6 Hz |
| record_width 1920 | 15.0 Hz | 72 ms | 61.0 / 72.7 ms | 99.6 Hz |

| window 1920×1080 + record_width 1920 | 15.0 Hz | 99 ms (p99 93) | 72.5 / 105.0 ms | 99.6 Hz |

最後の行は `window_width:=1920 window_height:=1080 record_width:=1920`。mp4 は **1850×1016** になった (画面が 1920×1080 で、
パネルとウィンドウ枠のぶん Wayland がウィンドウを縮める)。画像は 15 Hz を保つが、描画が重くなるぶん遅延が平均 +13 ms・最大 105 ms に増える
(failsafe の 150 ms には余裕あり)。学習データを取るときや遅延を測るときは既定の 1024×768 で、見せる動画だけ大きく録るのがよい。

224×224 配信では録画による劣化は無い。**record_width の既定 1280 のままでよい**。
注意: 録画サイズは `min(record_width, ウィンドウ幅)` なので、既定のウィンドウ (1024×768) では 1280 でも 1920 でも 1024×768 になる。
大きく録るならプレイヤーに `-screen-width 1920 -screen-height 1080` を足す。約 90 s で 30 MB。

## 床のカーペット (白い破線の修正)

`tools/make_real_textures.py` が作るタイルに、白いテープ・壁の下端・反射を含むパッチが混ざっていて、
床に敷き詰めると規則的な白い破線に見えていた。パッチ内に平均 +45 を超える画素が 0.2 % 以上あるものを捨てるようにした。
