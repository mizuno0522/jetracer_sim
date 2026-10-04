# 富士スピードウェイ (実車スケール)

ミニカーのコースと同じ仕組み (物理 `vehicle_sim`・描画 Unity・`/actuator_cmd` の契約) のまま、コースと車両の数値だけを
実車にしたもの。787B・ロードスター ND・RX-7 の 3 台で走れる。プロコンでの運転と ML-Agents の学習もそのまま使える。

| | ミニカー (`course:=minicar`) | 富士 (`course:=fuji`) |
|---|---|---|
| コース | 規約 p.24 の 10.3 × 6.4 m・1 周 約 29 m | 全長 **4,563 m**・幅 15 m・ランオフ 8 m・時計回り (`config/courses/fuji.yaml`) |
| 車両 | `jetracer_tt02` | `real_nd` / `real_rx7` / `real_b787` (`config/vehicle_profile/`) |
| 壁 | 板 (高さ 9 cm) | コース端の 8 m 外のバリア (高さ 1 m)。コース外はコース端 + 3 m |
| 路面 | カーペット・芝・滑り板… | 舗装 1 種。μ は車両の `tire_mu` |
| IMU | `imu_sim.yaml` | `imu_sim_real.yaml` (重心付近・±8 g・舗装の振動) |
| カメラ | 224×224・15 Hz (TT-02 の実測幾何) | 224×224・15 Hz (ボンネット上・水平画角 100°) |

## コースの線形について (★推定)

測量データではない。公表値 (全長 4,563 m・メインストレート 1,475 m・各コーナーの半径 TGR 27R / コカ・コーラ 80R /
100R / ADVAN 30R / 300R / GR スープラ 85R→25R / パナソニック 75R→33R) から `tools/make_fuji_course.py` が組み立てる。
曲がる角度と途中の直線の長さは推定で、形が閉じるように最小の変更で合わせている。高低差は持たない (2D)。
実際の線形 (GPX) が手に入ったら差し替えられる:

```bash
python3 tools/make_fuji_course.py --gpx fuji.gpx --plot /tmp/fuji.png    # config/courses/fuji.yaml を書き直す
for p in real_nd real_rx7 real_b787; do COURSE=fuji ./scripts/export_course.sh $p; done
```

## 車両 (★公表値からの概算)

| | ND | RX-7 | 787B |
|---|---|---|---|
| 全長 × 全幅・WB | 3.92 × 1.74 m・2.31 m | 4.29 × 1.76 m・2.43 m | 4.78 × 1.99 m・2.66 m |
| 重さ (乗員込み) | 1,100 kg | 1,350 kg | 900 kg |
| 最高速 (`v_max_mps`) | 200 km/h | 250 km/h | 330 km/h |
| `tire_mu` | 1.00 | 1.05 | 1.90 (スリック＋ダウンフォースを一定の μ で近似) |

駆動力は `F(v) = force_n × (1 − v / v_free)` の直線 (TT-02 の DC モータと同じ式) で、最高速と 0-100 km/h に合うように置いた。
制動を全輪で受けるため `drive: all` にしている (vehicle_sim の制動上限が「駆動輪の荷重 × μ」のため)。

**ROS 無しの検証 (物理そのまま、中心線を追う簡単なドライバ、横 0.85 μg まで):**
ND 2:36.7 (最高 198 km/h・横 0.89 g)、RX-7 2:25.8 (236 km/h・0.94 g)、787B 1:48.0 (311 km/h・1.73 g)。3 台とも衝突・コース外 0。
ミニカーのコースは変更前と同じ結果 (11.80 s)。

## 動かす

```bash
# 0. Unity 用のコース (3 台ぶん。カメラの取付が車ごとに違う)。StreamingAssets にも入れてある
for p in real_nd real_rx7 real_b787; do COURSE=fuji UNITY_PROJ=unity/MinicarSim ./scripts/export_course.sh $p; done
#    ビルド済みのプレイヤーなら Build/MinicarSim_Data/StreamingAssets/ に course_fuji_*.json をコピーするだけでよい

# 1. sim (Unity・エンジン音つき)
ros2 launch minicar_sim sim_host.launch.py course:=fuji vehicle_profile:=real_rx7 car:=rx7 \
     rival_car:=b787 rival2_car:=nd unity_player:=<Build/MinicarSim.x86_64>

# 2. プロコンで運転 (realtime。vehicle_stack は上げない)
ros2 run joy game_controller_node &
python3 tools/teleop/joy_teleop.py --ros-args -p publish_actuator:=true -p vehicle_profile_file:=real_rx7
```

`vehicle_profile:=` と `car:=` は別 (物理と見た目)。組み合わせは自由だが、そろえる (`real_rx7` と `rx7`) のが自然。

## ML-Agents (docs/mlagents.md の手順のまま、次だけ変える)

```bash
ros2 launch minicar_sim sim_host.launch.py course:=fuji vehicle_profile:=real_b787 car:=b787 \
     sim_mode:=lockstep mlagents:=true ml_maxsteps:=3600 demo:=fuji_b787_01 unity_player:=<...>
python3 tools/mlagents/mlagents_gateway.py --ros-args -p vehicle_profile_file:=real_b787 \
     -p reward_file:=tools/mlagents/config/reward_fuji.yaml
```

- 報酬は `reward_fuji.yaml` (距離をコース幅の比 25 倍で伸ばした重み)。ミニカーの `reward.yaml` のままだと横偏差の罰が桁違いに大きくなる
- 1 判断 = 1/15 s のまま。1 周 ND で約 2,300 判断なので `ml_maxsteps:=3600` (4 分)
- 舵・速度の制限は車両の `cmd_limits` (加速 3.5〜9 m/s²・減速 9〜17 m/s²・舵 1.2 rad/s) を gateway とプロコンが読む
- 開始位置 `random` はコース上の任意の位置・横 ±2.25 m

## Unity の描画

`course_fuji_<profile>.json` (`kind: circuit`) を読むと、ミニカーの会場の代わりに次を組み立てる (`CourseBuilder.Circuit.cs`・`Landscape.cs`):
舗装のランオフ・コース (アスファルト)・白線・縁石 (半径 220 m より急なところに紅白 3 m ごと)・バリア・コントロールライン (市松)・
ブレーキングの目安の距離板 (300 m 以上の直線の先のコーナー手前に 300 / 200 / 100)・観客席とピット棟、そのまわりの景色:

- **地形**: コースの外 25 m までは平らな芝生。そこから丘が立ち上がり、遠くほど起伏が大きい。北西 9 km に高さ 1.8 km の山 (上ほど急な斜面・谷筋・雪)
- **森**: コースの周り 1.5 km に約 2.6 万本。広葉樹・針葉樹 各 3 種の絵 (枝と数千の葉の固まりを 3D に置いて陰影をつけたもの) を十字の板にして立てる。
  林の縁は入り組み、草地にも所々一本木。地面は森の下が暗い樹冠の色、草地は黄緑のむら
- **空**: 積雲の帯 (半径 10 km のドーム)。遠くは指数のかすみ (2 km で 16 %・9 km で 55 %)

テクスチャはすべて実行時に式とシードで作る (画像ファイルなし。起動時に数秒かかる)。車体は `vehicle` の全長に合わせて拡大し (センサマストは隠す)、
追従視点の距離も同じ倍率にする。Unity を使わずに見た目を確かめるには:

```bash
python3 tools/preview_circuit.py --course unity/course_fuji_real_rx7.json --s 1250 --out /tmp/chase.png           # 追従視点
python3 tools/preview_circuit.py --course unity/course_fuji_real_rx7.json --s 2600 --view scenic --out /tmp/view.png  # 景色
```

Unity の実際の画面で確かめるには、プレイヤーを撮影・計測モードで起動する (ROS 不要・画面が要る)。決めた位置 (既定 s = 0・1250・2600・3300 m) に
自車とライバル 2 台を置いて、追従視点・車載カメラ (配信している画像そのもの)・俯瞰を撮り、そのあと車を走らせて描画の重さを計って終了する:

```bash
./scripts/shots.sh fuji rx7      # → shots/fuji_rx7_<日時>/ に 12 枚・sheet.png (一覧)・bench.md (GPU・組み立て時間・fps)
./scripts/shots.sh minicar       # ミニカーの会場 (s = 0・8・15・22 m)
python3 tools/shot_sheet.py shots/<変更前> shots/<変更後> --out /tmp/ab.png   # 左右に並べて比べる
```

プレイヤーの引数で直接使うなら `-shots "0,1250" -shotdir <dir> -shotsize 1920x1080 -bench 600` (`SimBridge.Shots.cs`)。

## まだ無いもの

- 高低差 (坂の加減速・ピッチ)。`vehicle_sim` の坂は②坂道の区間だけ
- `vehicle_stack` の教師 (`gt_teacher`) での自動運転 (先行注視点の距離などがミニカーの値)。走らせるのはプロコンか ML-Agents
- OpenCV 描画 (`camera_backend:=opencv`) はミニカーの板の見た目になる。Unity を使う
- 空気抵抗とダウンフォースの速度依存 (惰行の減速と一定の μ で近似)
