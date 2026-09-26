# sim PC 側の状況 (2026-09-26 時点)

sim PC (Ubuntu 22.04.5・Humble desktop・AMD 内蔵 GPU = CUDA 無し・Unity 6000.0.83f1) で確かめたことと、PC 側の担当
(`unity/`・`jetracer_compat`・`tools/sim2real`・Unity 系の scripts) の現状。環境構築の手順そのものは [setup.md](setup.md)・[unity.md](unity.md)。結果の画像と動画は [results/2026-09-26](results/2026-09-26/README.md)。

## いまの状態

| 項目 | 状態 |
|---|---|
| ビルド・単体テスト | 新規 clone から `colcon build` 8 パッケージ・`scripts/test.sh` とも通過 (v0.1.1 相当) |
| 1 台閉ループ (`scripts/smoke_test.sh`) | PASS。教師で 50 s に 2 周・\|cte\| p95 0.10 m・衝突 0、/imu 100 Hz・画像 15 Hz・/actuator_cmd 30 Hz |
| 2 ホスト直結 (P9) | 完了 (下の節)。PC の enp5s0 は NetworkManager の `jetracer-link` (192.168.10.2)。既存 sim 用の `sim_link` (192.168.0.20) とは排他で、戻すときは `sudo nmcli con up sim_link` |
| DDS | `rmw_cyclonedds_cpp` 導入済み。`scripts/sim_env.sh` が有線のリンク状態で `~/cyclonedds-wired.xml` / `~/cyclonedds-local.xml` を自動で選ぶ |
| Unity 中継 | `scripts/setup_ws.sh` が `ros_ws/src/ros_tcp_endpoint` に取り込む (この PC は `~/ros2_unity_ws` の checkout を symlink) |
| Unity プレイヤー | ソースはリポジトリの `unity/MinicarSim`、ビルドは `scripts/build_unity.sh` → `~/jetracer/unity/player`。他チーム向けはビルド済みを [Release v0.1.1](https://github.com/mizuno0522/jetracer_sim/releases/tag/v0.1.1) に置き `scripts/get_unity_player.sh` で取る |
| カメラ幾何 (Unity) | `course.json` の `fx, fy, cx, cy, k1, k2`・`render_tan_*` で描く (下の節)。推定値 (高さ 0.148 m・下向き 50.9°・水平 144°/垂直 120°・fy/fx 1.78) で OpenCV 描画と一致 |
| 他チームの JetRacer 標準コード | `jetracer_compat` で無改造のまま sim で走る (下の節)。v0.1.1 として公開 |
| sim→real 画像変換 | 1 回目 (FastCUT)・2 回目 (標準 CUT) は不合格、形を保つ損失を足して 3 回目を学習中 (下の節) |

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
- 3 回目 `cut_003` を学習中: cut_002 の重みから、形を保つ損失 (1/4 に縮めた輝度の勾配を揃える `--lambda-struct 10`) と λ_NCE 2 で 8,000 反復 (約 1.2 時間)。
- 推論は ONNX で約 57 ms/枚 (CPU)。走行中に挟むと画像の遅れが増えるので、記録済み bag を後から変換する (`convert_bag.py`)。
- 変換した bag は学習データの一部だけに混ぜる (9/12 の会場に寄せきらない)。

## 踏んだ落とし穴 (PC)

| 症状 | 原因 | 対処 |
|---|---|---|
| 自分の端末が落ちる | `pkill -f "<文字列>"` や `kill $(pgrep -f "<文字列>")` がその文字列を含む自分のシェルにも一致 | `scripts/stop_sim.sh` は実行ファイルのパスで先頭固定 (`^`)。手で止めるときは `ps` で PID を確かめてから |
| スクリプトから上げた launch が Ctrl-C で止まらない | 非対話シェルの `&` job は SIGINT 無視を継承 | `set -m` を付けてから起動 (`run_sim_local.sh`・`smoke_test.sh`・`record.sh`) |
| `git pull --rebase` が「unstaged changes」で止まる | Unity のビルドが `Assets/Scenes/Minicar.unity` を毎回書き換える | ビルド後は `git checkout -- unity/MinicarSim/Assets/Scenes/Minicar.unity` |
| venv が作れない | `python3.10-venv` が未導入 (sudo が要る) | `setup_sim2real.sh` が virtualenv に切り替える |
| Unity の白い板が灰色に沈む | 影側の壁には環境光しか当たらない | `realism.ambient_gain` 1.8 |

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
