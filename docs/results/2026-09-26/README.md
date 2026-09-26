# 結果 2026-09-26 (sim PC)

sim PC 側で撮った結果の画像と動画。数値の詳細は [../../status_pc.md](../../status_pc.md)。

## 動画

- [`sim_lap_unity.mp4`](sim_lap_unity.mp4) (84 s・640 幅): 教師 (参照線の Pure Pursuit) で周回。上 = 追従視点とミニマップ (橙 = TT-02 用の参照線)、
  下 = 実際に配信している 224×224 のカメラ画像 (推定カメラ・横の潰れ・周辺減光・車体の柱・床テープの乱択化)。
  録り方: `ros2 launch minicar_sim sim_host.launch.py ... record:=~/Videos/run.mp4` ([../../unity.md](../../unity.md) の録画)。

## カメラ幾何 — 実画像との比較

| 画像 | 内容 |
|---|---|
| [`camera_before_sim_vs_real.png`](camera_before_sim_vs_real.png) | 修正前。上 = sim (仮値: 下向き 12°・水平 120°・正方画素)、下 = 9/12 の実画像。sim は地平線が画面の中央で空が半分、実画像は床が大半 |
| [`camera_after_sim_vs_real.png`](camera_after_sim_vs_real.png) | 修正後。実走画像からの推定値 (高さ 0.148 m・下向き 50.9°・水平 144°/垂直 120°・fy/fx 1.78) で、構図が実画像に近づいた |
| [`geometry_opencv_vs_unity.png`](geometry_opencv_vs_unity.png) | 同じ姿勢の OpenCV 描画 (左)・Unity 描画 (中)・エッジの重ね (右: 白 = 一致、紫 = OpenCV のみ、緑 = Unity のみ)。F 値 0.76 @2px / 0.84 @4px |

## Unity の表示

| 画像 | 内容 |
|---|---|
| [`minimap_reference_line.png`](minimap_reference_line.png) | ミニマップ。参照線 (橙) があるときは白い中心線を描かない。白・赤の細線は壁、黄はスタートライン |
| [`floor_tapes_seed7.png`](floor_tapes_seed7.png) | seed 7 の 1 周から 8 枚。床の白テープは走るたびに位置・本数が変わる (会場ごとに白線が違うので固定では描かない) |

## sim→real 画像変換 (CUT)

| 画像 | 内容 |
|---|---|
| [`sim2real_cut001_failed_019000.png`](sim2real_cut001_failed_019000.png) | **失敗例**。FastCUT・128 切り出しの 19,000 反復。上段 = sim、下段 = 変換後、右端 = 実画像。床に緑・水色の斑点が出た。標準 CUT で学び直し中 (`runs/cut_002`) |
