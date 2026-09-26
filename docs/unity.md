# Unity 描画 — カメラ画像だけを Unity で描く

設計書 (詳細版): https://claude.ai/artifact/NBkzJSj8s61NLonsNQwBUG の「環境と配置」タブ。
既存 sim の説明は [minicarbattle2026/docs/unity_sim.md](https://github.com/mizuno0522/minicarbattle2026/blob/main/docs/unity_sim.md)。ここは JetRacer sim で変わった点だけ。

Unity は **カメラの代わり**。物理は `vehicle_sim` にしかなく、Unity は `/sim/render_state` の姿勢どおりにカメラを置いて描き、`/camera/image_raw` を返すだけ (P2・P3)。実機では存在しないので車には載せない。

```
vehicle_sim ──/sim/render_state (sim 時刻 stamp, step_id, car_id)──▶ ros_tcp_endpoint ──TCP 10000──▶ Unity MinicarSim
     ▲                                                                                                    │ 224×224 で描く
     └── /camera/image_raw (bgr8 224×224 15 Hz) ◀── ros_tcp_endpoint ◀─────────────────────────────────────┘
```

## 場所と流れ

| もの | 場所 | 備考 |
|---|---|---|
| Unity プロジェクト | **`unity/MinicarSim`** (このリポジトリ。`minicarbattle2026/unity/MinicarSim` の複製 + JetRacer 向けの変更。Unity 6000.0.83f1・ROS-TCP-Connector 0.7.0) | Assets / Packages / ProjectSettings だけを管理。Library は初回ビルドで生成 (数分)。複製元との差分: `Scripts/LabelTexture.cs` (P1/P2/P3 のドット文字)、`Shaders/SensorPost.shader` (実カメラ風の後処理)、`CourseBuilder.cs` (駐車枠ラベル・実画像カーペット・観戦者・realism の色)、`CourseData.cs` (`RealismData`)、`SimBridge.cs` (後処理・自動露出・frame_id `camera_link`)、`Editor/MinicarBuild.cs` (Mat_SensorPost)。ビルドは `scripts/build_unity.sh` |
| コースとカメラ幾何の定義元 | `ros_ws/src/minicar_sim/scripts/course.py`・`config/sim.yaml`・`config/vehicle_profile/jetracer_tt02.yaml` | Unity 側に数値を書かない (P11) |
| 書き出し | `./scripts/export_course.sh` → `unity/course.json` | `UNITY_PROJ=<path>` を付けたときだけ `Assets/StreamingAssets/course.json` へコピー |
| プレイヤーのビルド | `minicarbattle2026/unity/build_player.sh` → `Build/MinicarSim.x86_64` | ★既存スクリプトは `jetson/ros_ws/.../export_unity_course.py` (M-05 用) を呼ぶ。JetRacer 用は先に `export_course.sh` でコピーしてから Unity 部分だけ実行する (下) |
| ROS 側の中継 | `ros_tcp_endpoint` (sim コンテナに同梱・`docs/docker.md`)。素で動かすなら `~/ros2_unity_ws` | Unity → endpoint は localhost。LAN に乗るのは DDS だけ |

`course.json` に入るもの: `walls`・`areas` (ギミック区間)・`parking_slots`・`arrow_sign`・`light`・`camera` (224×224・15 Hz・hfov・取付高さ・ピッチ・クロップ)・`viz`・`centerline_shortcut`/`centerline_long`。JetRacer 用に変わるのは `camera` だけで、コースは同一。

### 手順 (sim PC)

```bash
# 1. JetRacer 用のカメラ幾何で course.json を書き出し、Unity プロジェクトへコピー
UNITY_PROJ=~/minicarbattle2026/unity/MinicarSim ./scripts/export_course.sh jetracer_tt02
# 2. Unity をビルド (export の行は飛ばす。既存 build_player.sh の Unity 呼び出し部分と同じ)
UNITY=~/Unity/Hub/Editor/6000.0.83f1/Editor/Unity
"$UNITY" -batchmode -nographics -projectPath ~/minicarbattle2026/unity/MinicarSim \
   -executeMethod Minicar.EditorTools.MinicarBuild.SetupAndBuild -logFile /tmp/build_player.log
# 3. 起動 (endpoint は sim_host.launch.py が上げる)
ros2 launch minicar_sim sim_host.launch.py unity_player:=~/minicarbattle2026/unity/MinicarSim/Build/MinicarSim.x86_64
```

★ `~/minicarbattle2026` は既存車両 (M-05) の作業コピー。`course.json` を JetRacer 用で上書きすると M-05 の sim が 224×224 になる。M-05 に戻すときは `minicarbattle2026/unity/build_player.sh` を再実行する (M-05 用の export が走る)。両方を並行して使うなら Unity プロジェクトを複製すること。

エディタの Play で繋ぐとき (スクリプトを直しながら見る): `unity_player:=none` で起動し、Unity Hub からプロジェクトを開いて `Assets/Scenes/Minicar.unity` を Play。プレイヤーのログは `~/.ros/log/minicar_unity_player.log`。

## 同じ PC で既存 sim (M-05) と並行して動かす

sim PC に既存の ROS-Unity 版 (`~/minicarbattle2026/jetson/ros_ws`) と JetRacer sim (`~/jetracer/jetracer_sim`) が同居する前提。既存 sim のベンチ sweep が `ROS_DOMAIN_ID` 未設定 (=0) で動いていることがあり、42 で分離できるが CPU 負荷は共有する。**4 つを分ける**。

| 衝突するもの | 既存 sim | JetRacer sim | 分け方 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | 0 (未設定)・race2.sh は 81/82 | **42** (`scripts/sim_env.sh`) | 同じ ID だと `/actuator_cmd` の型が違う (DriveCommand vs ActuatorCmd) ので型不一致エラーが出る |
| `minicar_msgs` パッケージ | `~/minicarbattle2026/jetson/ros_ws/install` | `~/jetracer/jetracer_sim/ros_ws/install` | **同じ端末で両方を source しない**。片方の端末は片方だけ |
| `ros_tcp_endpoint` の TCP ポート | 10000 | `tcp_port:=10001` | 両方 Unity を同時に上げるときだけ。片方ずつなら 10000 のまま |
| Unity プロジェクト / `course.json` | 既存の `MinicarSim` (320×216・30 Hz) | **複製** して 224×224 用にする | `export_course.sh` を既存プロジェクトに向けると M-05 の描画が 224×224 になる |

**再ビルドは要らない (sim PC で確認済み)**: ビルド済みプレイヤーは `Build/MinicarSim_Data/StreamingAssets/course.json` を実行時に読む。プレイヤーのフォルダをコピーして course.json だけ差し替えれば JetRacer 用になる。Unity プロジェクトの複製とビルドは、描画そのもの (駐車枠の P1/P2/P3 マーク・レンズ歪みなど) を変えるときだけ。

```bash
# 最短: ビルド済みプレイヤーをコピーして course.json を差し替える (Unity Editor 不要)
cp -a ~/minicarbattle2026/unity/MinicarSim/Build ~/jetracer/unity/player
./scripts/export_course.sh jetracer_tt02
cp unity/course.json ~/jetracer/unity/player/MinicarSim_Data/StreamingAssets/course.json
ros2 launch minicar_sim sim_host.launch.py unity_player:=~/jetracer/unity/player/MinicarSim.x86_64 tcp_port:=10001

# 描画を変えるとき: JetRacer 用に Unity プロジェクトを複製 (初回のみ。Build/ と Logs/ は除く)
rsync -a --exclude Build --exclude Logs --exclude Library ~/minicarbattle2026/unity/MinicarSim/ ~/jetracer/unity/MinicarSim/
UNITY_PROJ=~/jetracer/unity/MinicarSim ./scripts/export_course.sh jetracer_tt02
"$UNITY" -batchmode -nographics -projectPath ~/jetracer/unity/MinicarSim -executeMethod Minicar.EditorTools.MinicarBuild.SetupAndBuild -logFile /tmp/build_jetracer.log
# 起動 (既存 sim も同時に上げるなら tcp_port を変える)
source scripts/sim_env.sh
ros2 launch minicar_sim sim_host.launch.py unity_player:=~/jetracer/unity/MinicarSim/Build/MinicarSim.x86_64 tcp_port:=10001
```

`Library/` を除くと初回の Unity 起動でインポートが走る (数分)。GPU は 1 枚を 2 つの Unity が分け合うので、両方同時はフレームレートが落ちる。

## 確認

```bash
ros2 topic hz /camera/image_raw            # 15 Hz
ros2 topic echo /camera/image_raw --once --no-arr | grep -E "width|height|encoding"   # 224 / 224 / bgr8
ros2 topic echo /camera/camera_info --once  # camera_info_pub が vehicle_profile から出す K と一致していること
```

`camera_backend:=opencv` (Unity 無し・`vehicle_sim` の射影描画) と同じ `camera_info` を使うので、先行注視点の画像座標 (`/sim/ground_truth` の u・v_px) は描画バックエンドに依らず同じになる。Unity と OpenCV で壁の位置がずれて見えたら `course.json` の書き出し忘れ。

## 録画

**Unity の表示 (追従視点・HUD・ミニマップ・センサ画像)** は Unity が描いた画面をそのまま ffmpeg (libx264 ultrafast) へ流して録る。デスクトップ録画 (x11grab) と違い Wayland でも黒画面にならず、描画も止まらない。既存 sim の `ScreenRecorder.cs` をそのまま使う (`sudo apt install ffmpeg` が要る)。

```bash
ros2 launch minicar_sim sim_host.launch.py camera_backend:=unity      unity_player:=~/jetracer/unity/player/MinicarSim.x86_64 tcp_port:=10001      record:=~/Videos/run.mp4                      # record_fps:=30 record_width:=1280 も指定できる
```

- 録画は起動直後から始まり、**Unity を SIGTERM で閉じたときに mp4 を閉じる** (Ctrl-C で launch を止めれば良い)。SIGKILL で殺すと再生できないファイルになる。
- 断片化 mp4 なので、途中で止めてもそこまでは再生できる。
- **録画サイズは min(`record_width`, ウィンドウ幅)**。既定ウィンドウは 1024×768 なので、1280 で録るなら `window_width:=1280 window_height:=720`、フル HD なら `window_width:=1920 window_height:=1080 record_width:=1920`。
- sim PC での実測 (224×224 配信): ウィンドウ 1024×768 のままなら録画なし / 1280 / 1920 のいずれでも `/camera/image_raw` 15.0 Hz・遅延 60 ms で差なし、`/imu` 99.6 Hz。
- **ウィンドウを 1920×1080 にすると画像の遅延が 60 → 72.5 ms (最大 105 ms) に増える** (15 Hz は維持)。描画が重くなるため。画像と IMU の相対遅れが変わるので、**学習データの記録と遅延の計測は既定の 1024×768 で**。大きいウィンドウは見せる動画を撮るときだけ。なお画面が 1920×1080 だと Wayland がパネルと枠のぶん縮め、mp4 は 1850×1016 になる。
- `record_width` を上げるほど符号化が重くなり、`/camera/image_raw` の配信レートが落ちる (既存 sim で 30 → 21 Hz になった実測)。既定の 1280 から上げるときはレートを確認する。
- 録画できるのは **Unity を動かしているホスト (sim PC)** の画面。Jetson からは録れない。

**車が実際に見ている画** (`/camera/image_raw`) を録るなら `tools/record_video.py`。Unity の HUD は入らないが、Unity 無し (`camera_backend:=opencv`) でも 2 ホストでも録れ、受信したフレームそのものなので経路の取りこぼしもそのまま映る。

```bash
python3 tools/record_video.py --seconds 60 --scale 3 --hud -o ~/Videos/camera.mp4
```

`--scale 3` は 224×224 を 3 倍に、`--hud` は sim 時刻・車速・横偏差・周回を下端に焼き込む。`--fps` は受信レート (既定 15) に合わせること。ずれていると終了時に警告が出る。

## lockstep (`sim_mode:=lockstep`) での描画

`/sim/render_state` の stamp は sim 時刻。`/sim/step` は、この step で画像が来る予定 (15 Hz なので 2 step に 1 枚) のときだけ、直前より新しい stamp の画像が返るまで `step_image_timeout_s` (既定 0.5 s) 待つ。Unity が遅ければ step がその分遅くなるだけで、物理は進まない。`step_id`・`car_id` は `/sim/render_state` に予約済みで、Unity 側は当面無視してよい (N 台並列のときに使う)。

## カメラの幾何 (画角・縦横比・歪み)

**定義元は `vehicle_profile.camera` の 1 か所**、式は `jetracer_common/cam_geom.py` (OpenCV plumb_bob の k1, k2)。ラベル (`/sim/ground_truth` の u, v)・`cmd_shaper`・`camera_info`・OpenCV 描画・Unity がすべて同じ値と式を使う。

- `export_course.sh` が `course.json` の `camera` に `fx, fy, cx, cy, k1, k2` と、ピンホールで描くべき範囲 `render_tan_x0/x1/y0/y1` を書き出す
- Unity (`SimBridge.ApplyIntrinsics`) はその範囲を off-axis 透視 (`Matrix4x4.Frustum`) で描き、`SensorPost` が出力画素ごとに逆歪み (不動点反復 8 回) してサンプルする。fy ≠ fx もこれで扱える
- OpenCV 描画 (`vehicle_sim`) も同じ手順 (広いキャンバスに描いて `cv2.remap`)。同じ姿勢で撮り比べてエッジの F 値 0.76 @2px / 0.84 @4px
- 今の値 (2026-09-26) は実走画像からの推定: 高さ 0.148 m・下向き 50.9°・水平 144°/垂直 120° (歪み込み)・fy/fx 1.78 (jetcam 既定 640×480 → Argus mode 4 = 1280×720 → 224×224 に潰す)。**チェッカーボード較正 (`tools/camera_calib.py`) で確定させる** ([calibration.md](calibration.md))
- 値を変えたら `./scripts/export_course.sh` → `./scripts/setup_unity_player.sh` (course.json の差し替え) だけで Unity に反映される。再ビルドは要らない
- ★歪みは今 `SensorPost` の中で掛けているので、`realism.enable: false` にすると歪まない (幾何は realism と切り離すべきなので、将来は歪みだけ常に掛ける形に分ける)

## 実カメラの見た目に寄せる (JetRacer 用複製プロジェクト・`vehicle_profile.camera.realism`)

複製プロジェクト `unity/MinicarSim` にだけ入っている。数値の定義元は
`ros_ws/src/minicar_sim/config/vehicle_profile/jetracer_tt02.yaml` の `camera.realism` で、
`export_course.sh` が `course.json` の `realism` に書き出す (無ければ Unity 側の既定 = オフ)。
根拠は 2026-09-12 の手動走行ログ (`tools/make_real_textures.py` が統計と床タイルを作る)。

| 項目 | 実装 | 実画像との比較 (`tools/compare_images.py`) |
|---|---|---|
| 床のカーペット | 実画像の近景から作ったタイル `StreamingAssets/textures/carpet.png` | 床 BGR (120,117,113) vs 実 (116,123,118) |
| 壁の色 | 白 (215,213,211)・赤 (196,38,36、規約の色)。影側で白が沈むので環境光を `ambient_gain` 1.8 倍 | p95 210 vs 220 |
| 樽型歪み・画角 | 上の「カメラの幾何」(profile の `k1, k2, fx, fy`)。旧 `realism.k1/k2/zoom` は廃止 (向きが逆で糸巻き型になっていた) | — |
| 周辺減光・ぼけ・ノイズ | 同シェーダ (vignette 0.16・blur 0.8 px・noise 0.02)。2 倍で描いて縮小 | 周辺減光 1.1 vs 1.2、鮮鋭度 (Laplacian 分散) 371 vs 345 |
| 自動露出 | 平均輝度を目標 (110) に一次遅れ (τ 0.4 s) で寄せる | 中央値 104 vs 105 |
| モーションブラー | ヨーレート比例の横ブラー (2 px per rad/s) | — |
| 車体の柱 | 画面下の 2 本を実機と同じ列 (54–57・121–124、行 209〜) に描く (`posts`) | ★**両系同値**: 実機側 `camera_preproc` (stack.yaml `policy_net.mask_bottom_frac`、現在 0.0) でマスクするなら、その領域と `posts` を同じ値にすること。片方だけだと sim-to-real の穴 |
| 観戦者 | 壁の外に 24 人 (seed 固定)。壁の上に見える雑音として | — |
| エピソード乱択化 | `/sim/episode` (vehicle_sim が LATCHED で出す seed) か起動引数 `-seed` を受けて、天井光の強さ・色味、環境光、床の色味、観戦者の配置を引き直す (`episode_*`。幅は暫定) | 1 本の bag = 1 seed (`scripts/record.sh`)。物理・IMU も同じ seed |
| 床の白テープ | **固定では描かず乱択化**: 走路を横切るテープを 0〜6 本 (位置・角度 ±10°・長さ・幅・明るさを振る)、規約のスタートライン 1/2/3 (x ≈ 2.7/4.6/6.5 m、下段レーン) はそれぞれ確率 0.5 (`episode_tapes_max`・`tape_*`・`start_line_*`) | 実画像の会場 (9/12 MEC) は本番どおりのコースではなく白線の有無が違う。特定の配置を覚えさせないため |
| 背景 | Unity 既定のスカイボックス (`background_gray: -1`)。実画像を並べた背景円筒は平面的で不採用 | 上半分が実画像より 10〜15 暗い (未調整) |

描画そのもの (C#・シェーダ) を変えたときだけ `scripts/build_unity.sh`。数値だけなら `setup_unity_player.sh` (course.json 差し替え) で足りる。

## 未実装 (設計 未決タブ「決める順番」)

実データ (2026-09-12 の手動走行ログ) で必要と分かったが、まだ済んでいないもの。レンズ歪み・自動露出・モーションブラーは実装済み (上の表)。

| 対象 | 実データ | 入れ方 |
|---|---|---|
| カメラの確定値 | 今は実走画像からの推定 | 実機でチェッカーボード較正 (`tools/camera_calib.py`)。横の画角 (fx) と縦横比は画像の壁だけでは決まらなかった |
| 背景の明るさ | 上半分が実画像より 10〜15 暗い | 未調整。会場ごとに違うので乱択化の幅で吸収する方向 |
| 車体の写り込み | 画面下に黒い柱 (実画像では 2 本: 列 54–57・121–124、行 209〜223) | Unity では `realism.posts` で描いた (上の表)。`camera_preproc` でマスクするなら**両系とも同じ領域** (片方だけだと sim-to-real の穴) |
| 壁の浮き | 規約では床から 30 mm 浮いて上端 120 mm | `course.json` の `wall_base_m` は書き出し済み (0.03)。Unity 側の反映は未確認 |

既存 sim の RViz 風・AI チャレンジ風レイアウト、2 台レース (`race2.sh`) はそのまま使える。JetRacer 機は LiDAR が無いので占有格子は表示されない。
