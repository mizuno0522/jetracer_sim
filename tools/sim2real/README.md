# sim→real 画像変換 (CUT) — PC の CPU で学習・変換

Unity の `/camera/image_raw` を、実カメラ (2026-09-12 MEC の手動走行ログ) の見た目に寄せる。形 (壁・走路の縁) は保ち、
質感・色・ぼけを実画像に近づける unpaired 変換 (CUT / FastCUT)。**オフライン変換** (記録済みの bag を後から変換) が既定。
走行中に挟むと画像の遅れに推論時間 (CPU で約 57 ms/枚) が乗り、画像と IMU の相対遅れが実機と変わるため。

**自分の実画像で作り直す手順 (データの集め方・推奨設定・検証・うまくいかないとき) は [docs/sim2real.md](../../docs/sim2real.md)**。このページは道具の要約と、この PC での学習の記録。

```bash
./scripts/setup_sim2real.sh                          # 初回のみ: ~/jetracer/venv_sim2real (CPU 版 PyTorch・onnxruntime)
PY=~/jetracer/venv_sim2real/bin/python

# 1. 学習データ (A = sim)。Unity 描画で教師を走らせて記録し、画像を抜く
./scripts/record.sh 8 60 --unity --seed0 301
source scripts/sim_env.sh
$PY tools/sim2real/extract_frames.py "bags/ep_30*" -o ~/jetracer/data/sim_frames --every 2
# B = 実画像: ~/jetracer/data/real_260912 (zip を展開したもの)

# 2. 学習 (CPU。標準 CUT・176 切り出し・形を保つ損失つきで約 0.57 s/反復 → 16,000 反復で約 2.5 時間。--resume で続きから)
$PY tools/sim2real/train_cut.py --sim ~/jetracer/data/sim_frames --real ~/jetracer/data/real_260912 \
    --out ~/jetracer/runs/cut_003 --iters 16000 --crop 176 --lambda-nce 2 --lambda-struct 10
#   → runs/cut_003/samples/*.png で途中経過、終了時に G.onnx

# 3. 幾何が保たれているか (変換前後のエッジの一致。中央値 ≥ 0.6 で合格)
$PY tools/sim2real/eval_geometry.py --model ~/jetracer/runs/cut_003/G.onnx --sim ~/jetracer/data/sim_frames --grid /tmp/s2r.png

# 4. bag を変換 (画像だけ差し替え、stamp とほかのトピックはそのまま) → bags/<元>_s2r/
$PY tools/sim2real/convert_bag.py "bags/ep_30*" --model ~/jetracer/runs/cut_003/G.onnx
```

- 画面下端の柱は学習させない: 入力の柱を両ドメインとも同じ色で塗り、変換後は入力の画素で上書き (`common.py`)。
  柱の位置は `vehicle_profile.camera.realism.posts` = 実機 `camera_preproc` のマスクと同値。
- 学習データの既定は **変換した bag と変換していない bag を半々**。9/12 の MEC 会場に寄せきると本番会場で外れる。
- 実画像は 1 会場・1 日分。本番会場の画像が撮れたら B に足して学習し直す。
- カメラ幾何 (vehicle_profile.camera) が変わったら A を撮り直し、既存の重みから `--resume` で追加学習 (短時間で追従する)。

## 現状 (2026-09-26)

| 学習 | 設定 | 結果 |
|---|---|---|
| `runs/cut_001` | FastCUT (`--fast`)・128 切り出し・0.21 s/反復 | **失敗**。19,000 反復で床に緑・水色の斑点が出て、実画像とかけ離れた (色を保つ恒等 NCE が無く崩れやすい。128 の切り出しと推論時の 224 全体の構図も合わない)。止めた |
| `runs/cut_002` | 標準 CUT (恒等 NCE あり・λ_NCE 1)・176 切り出し・0.56 s/反復・16,000 反復 (約 2.5 時間) | **不合格**。色と質感は実画像に近づいた (空 → 会場の室内、床 → カーペット、壁 → 実物の赤白) が、床の奥に実物に無い明るい塊を描き、水色の滑り板や壁の縁が消えることがある。`eval_geometry.py` のエッジ F 値 0.559 (目安 0.6、未学習モデル 0.856) |
| `runs/cut_003` | cut_002 の重みから (`--init`)・λ_NCE 2・**形を保つ損失 `--lambda-struct 10`**・8,000 反復 (約 1.2 時間) | **合格**。エッジ F 値 0.832 (下半分 0.676)。空 → 会場の照明と骨組み、壁 → 実物の赤白、床 → カーペット。滑り板の水色・壁と床の位置も保たれた。学習データ 8 本を変換 (`bags/ep_30*_s2r`) |

- A: 推定カメラ値 (高さ 0.148 m・下向き 50.9°・fy/fx 1.78) と床テープの乱択化入りの Unity 描画 8 本 (3,700 枚)。B: 9/12 MEC の実画像 11,271 枚。
- 途中経過は `runs/<名前>/samples/NNNNNN.png` (上段 = sim、下段 = 変換後、右端の列 = 実画像)。
- **CPU で急ぐときも `--fast` は使わない**。色が崩れた。切り出しは推論の 224 に近い大きさ (176 以上) にする。
- 形を保つ損失 `--lambda-struct`: 入力と出力を 1/4 (`--struct-pool`) に縮めた輝度の勾配の大きさを L1 で揃える。縮めることでカーペットの粒などの質感は消え、
  壁の縁・床の境目・滑り板の縁だけが残るので、「質感は実画像へ、縁の位置は sim のまま」を狙える。実画像に多い「床の奥が照明で明るい」見た目を
  形を無視してまねる崩れ (cut_002) への対策。
- `--init <ckpt.pt>`: 別の学習の重みから始める (反復数は 0 から)。追加学習で損失の設定を変えるときに使う。
- 終わったら `eval_geometry.py` で幾何が保たれているかを確かめてから `convert_bag.py` を使う。
- カメラを実機のチェッカーボード較正で確定させたら、A を撮り直して `--resume` で追加学習する。
