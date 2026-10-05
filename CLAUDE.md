# jetracer_sim の約束

## 設計
- 物理は vehicle_sim だけ (P3)。Unity は描く・鳴らすだけ
- Unity は数値を持たない (P11)。コース・車両寸法・カメラは course_*.json (scripts/export_course.sh) から
- 景色・質感・車体・エンジン音の式は Python に写しがある。変えたら必ず合わせる:
  Landscape.cs / ProcTex.cs / CourseBuilder.Circuit.cs / CarModel.cs ↔ tools/preview_circuit.py
  EngineAudio.cs ↔ tools/engine_sound.py

## 守るもの
- 3 つの版 (Built-in・URP・HDRP) は同じ仕様の絵を出す。違うのは PC への負荷だけ (2026-10-05 水野)。HDRP 版の絵が正で、
  URP 版・Built-in 版をそれに合わせる。例外は 1 つ: Built-in 版のミニカーの会場は部屋を描かず、地面と空だけ (2026-10-06 水野。コースそのものは同じ)。絵作りを変えたら 3 版を同じ位置で撮って並べる。1 つの版だけに効果を足さない。
  見た目の数値 (光・露出・後処理) は RenderCompat.cs の 1 か所に置く (docs/hdrp.md)
- 車載カメラ (配信するセンサ画像) に掛ける後処理は実カメラ風の SensorPost だけ (表示用の後処理は掛けない)。
  ミニカーの会場は URP 版・HDRP 版が部屋、Built-in 版 (学習もこの版) が地面と空だけ (以前の `-venue plain` は無くした)
- 画質 low (学習・-mlagents) の速さ: lockstep の 1 判断あたりの時間を基準 (docs/render_baseline.md) の +10 % 以内
- 周回タイム: ミニカー 11.80 s、ND 2:36.7、RX-7 2:25.8、787B 1:48.0 (描画だけの変更なら vehicle_sim に差分がないこと)
- python3 tools/mlagents/test_gateway_core.py が全件合格
- 見せる用の起動は GPU を見て 3 つの版から選ぶ (scripts/pick_unity_player.sh。docs/hdrp.md)。学習 (-mlagents) は軽い Built-in 版で行う

## しないこと
- 車体のロゴ・車名・配色は、非営利なので厳しく避けなくてよい (2026-10-04 水野)。本物らしさを優先してよいが、
  ロゴの画像ファイルは外部素材と同じ扱い (下の出典・ライセンスの決まり)
- 他社のゲーム (GT7 など) の画面・ロゴ・素材を使わない。参考にするのは光や空気感の方向だけ
- 外部素材は CC0 か商用可だけ。入れたら docs/assets_license.md に名前・URL・ライセンス。
  公開リポジトリなので大きな素材はリポジトリに入れず、取得スクリプト方式 (Git LFS の無料枠はすぐ尽きる)
- main に直接コミットしない

## 進め方
- 変更の前に計画を見せる。1 段階ごとにコミット
- 見た目を変えたら ./scripts/shots.sh で撮り、自分で画像を開いて確かめてから報告。前の結果と左右に並べる
- 音を変えたら tools/engine_sound.py で WAV を作り、人に聴いてもらう (Claude は音を聴けない)
- docs/fuji.md・docs/hdrp.md を変更に追いつかせる
