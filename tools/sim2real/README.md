# sim→real 画像変換 (CUT) — PC の CPU で学習・変換

Unity の `/camera/image_raw` を、実カメラ (2026-09-12 MEC の手動走行ログ) の見た目に寄せる。形 (壁・走路の縁) は保ち、
質感・色・ぼけを実画像に近づける unpaired 変換 (CUT / FastCUT)。**オフライン変換** (記録済みの bag を後から変換) が既定。
走行中に挟むと画像の遅れに推論時間 (CPU で約 57 ms/枚) が乗り、画像と IMU の相対遅れが実機と変わるため。

```bash
./scripts/setup_sim2real.sh                          # 初回のみ: ~/jetracer/venv_sim2real (CPU 版 PyTorch・onnxruntime)
PY=~/jetracer/venv_sim2real/bin/python

# 1. 学習データ (A = sim)。Unity 描画で教師を走らせて記録し、画像を抜く
./scripts/record.sh 8 60 --unity --seed0 201
source scripts/sim_env.sh
$PY tools/sim2real/extract_frames.py "bags/ep_20*" -o ~/jetracer/data/sim_frames --every 2
# B = 実画像: ~/jetracer/data/real_260912 (zip を展開したもの)

# 2. 学習 (CPU。FastCUT・128 切り出しで約 0.21 s/反復 → 3 万反復で約 1.8 時間。--resume で続きから)
$PY tools/sim2real/train_cut.py --sim ~/jetracer/data/sim_frames --real ~/jetracer/data/real_260912 \
    --out ~/jetracer/runs/cut_001 --iters 30000 --crop 128 --fast
#   → runs/cut_001/samples/*.png で途中経過、終了時に G.onnx

# 3. 幾何が保たれているか (変換前後のエッジの一致。中央値 ≥ 0.6 で合格)
$PY tools/sim2real/eval_geometry.py --model ~/jetracer/runs/cut_001/G.onnx --sim ~/jetracer/data/sim_frames --grid /tmp/s2r.png

# 4. bag を変換 (画像だけ差し替え、stamp とほかのトピックはそのまま) → bags/<元>_s2r/
$PY tools/sim2real/convert_bag.py "bags/ep_20*" --model ~/jetracer/runs/cut_001/G.onnx
```

- 画面下端の柱は学習させない: 入力の柱を両ドメインとも同じ色で塗り、変換後は入力の画素で上書き (`common.py`)。
  柱の位置は `vehicle_profile.camera.realism.posts` = 実機 `camera_preproc` のマスクと同値。
- 学習データの既定は **変換した bag と変換していない bag を半々**。9/12 の MEC 会場に寄せきると本番会場で外れる。
- 実画像は 1 会場・1 日分。本番会場の画像が撮れたら B に足して学習し直す。
- カメラ幾何 (vehicle_profile.camera) が変わったら A を撮り直し、既存の重みから `--resume` で追加学習 (短時間で追従する)。

## 現状 (2026-09-26)

- `~/jetracer/runs/cut_001` を学習中。A は推定カメラ値 (高さ 0.148 m・下向き 50.9°・fy/fx 1.78) と床テープの乱択化入りの Unity 描画 8 本 (3,700 枚)、
  B は 9/12 MEC の実画像 11,271 枚。最初の 5,000 反復は仮のカメラ値の A で、そこから `--resume` で 3 万反復まで。
- 途中経過は `runs/cut_001/samples/NNNNNN.png` (上段 = sim、下段 = 変換後、右端 = 実画像)。
- 終わったら `eval_geometry.py` で幾何が保たれているかを確かめてから `convert_bag.py` を使う。
- カメラを実機のチェッカーボード較正で確定させたら、A を撮り直して `--resume` で追加学習する。
