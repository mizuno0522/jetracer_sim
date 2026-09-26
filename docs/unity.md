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
| Unity プロジェクト | `minicarbattle2026/unity/MinicarSim` (Unity 6000.0.83f1・ROS-TCP-Connector 0.7.0) | **このリポジトリには Unity プロジェクトを持たない**。既存のものをそのまま使う |
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

## lockstep (`sim_mode:=lockstep`) での描画

`/sim/render_state` の stamp は sim 時刻。`/sim/step` は、この step で画像が来る予定 (15 Hz なので 2 step に 1 枚) のときだけ、直前より新しい stamp の画像が返るまで `step_image_timeout_s` (既定 0.5 s) 待つ。Unity が遅ければ step がその分遅くなるだけで、物理は進まない。`step_id`・`car_id` は `/sim/render_state` に予約済みで、Unity 側は当面無視してよい (N 台並列のときに使う)。

## 未実装 (設計 未決タブ「決める順番」)

実データ (2026-09-12 の手動走行ログ) で必要と分かったが、Unity に入れていないもの。

| 対象 | 実データ | 入れ方 |
|---|---|---|
| レンズ歪み | 樽型 (魚眼ではない・120〜160° 級) | ピンホール描画 ＋ 後処理の Brown モデル。係数はチェッカーボードで測ってから `vehicle_profile.camera.distortion` に |
| 露出変動 | 暗幕付近は全体が灰色に潰れる | 自動露出の模擬 (平均輝度への一次遅れ) |
| モーションブラー | 旋回中のフレームに強いブレ | ヨーレート比例のブラー |
| 車体の写り込み | 画面下に黒い 4 本の柱が全フレーム同位置 | Unity で描くか、`camera_preproc` で**両系とも同じ領域をマスク** (片方だけだと sim-to-real の穴) |
| 壁の浮き | 規約では床から 30 mm 浮いて上端 120 mm | `course.json` の `wall_base_m` は書き出し済み (0.03)。Unity 側の反映は未確認 |

既存 sim の RViz 風・AI チャレンジ風レイアウト、2 台レース (`race2.sh`) はそのまま使える。JetRacer 機は LiDAR が無いので占有格子は表示されない。
