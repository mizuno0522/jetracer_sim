# sim→real 画像変換 — 自分の実画像で変換器を作る

sim (Unity) のカメラ画像を、**自分の車のカメラで撮った実画像の見た目**に寄せる変換器の作り方。
形 (壁・走路の縁) は sim のまま保ち、質感・色・ぼけ・会場の雰囲気だけを実画像に近づける。
使う方式は CUT (Contrastive Unpaired Translation)。**sim と実画像を 1 枚ずつ対応づける必要はない** (別々に撮った画像の山が 2 つあればよい)。

道具は `tools/sim2real/`、この PC での学習の記録は [tools/sim2real/README.md](../tools/sim2real/README.md)。

## 全体の流れ

```
 ① 実画像を集める (B)        自分の車で走って撮った 224×224 の画像。数千枚〜
 ② sim の画像を撮る (A)       自分の車のカメラ設定にした sim で、教師に走らせて記録 → 画像を抜く
 ③ 学習                      A → B の変換器 (CPU で約 2.5 時間 / NVIDIA GPU なら数十分)
 ④ 検証                      形が保たれているか (数値) と、見た目 (目視)
 ⑤ 学習データの bag を変換    記録済みの bag の画像だけを差し替えた bag を作る
 ⑥ 方策の学習に混ぜる         変換した bag は一部だけ (目安 半分)。残りは乱択化した sim のまま
```

変換は**走行中ではなく、記録済みのデータに後から掛ける** (オフライン変換)。CPU で 1 枚約 57 ms かかり、
走行中に挟むと画像の遅れが増えて、画像と IMU の相対遅れが実機と変わるため。

## 0. 準備 (初回のみ)

```bash
cd jetracer_sim
./scripts/setup_sim2real.sh                      # ~/jetracer/venv_sim2real に CPU 版 PyTorch・onnxruntime
# NVIDIA GPU の PC なら:  TORCH_INDEX=https://download.pytorch.org/whl/cu128 ./scripts/setup_sim2real.sh
PY=~/jetracer/venv_sim2real/bin/python
```
ROS を使う手順 (②・⑤) は、先に `source scripts/sim_env.sh` した端末で行う。

## ① 実画像を集める (B)

| 条件 | 理由 |
|---|---|
| **自分の車のカメラで、実際の取り込み経路のまま** (JetRacer 標準なら jetcam の 224×224) | 変換器は画角・縦横比・ぼけ・色味まで覚える。別のカメラや別の縮め方の画像を混ぜると、その差まで sim に足してしまう |
| **数千枚以上** (この PC では 11,271 枚) | 少ないと特定の場面をそのまま写す |
| **走路の上から、いろいろな場面を** (直線・カーブ・トンネル・芝・滑り板・壁に寄った位置) | 変換器は「よく見る場面」に寄せる。直線ばかりだと、カーブでも直線の見た目を描く |
| **会場を複数入れられるなら入れる** | 1 会場だけだと、その会場の暗幕・窓・照明を覚えて、本番会場で外れる |
| ラベルは要らない | 画像だけでよい |

置き方は自由 (フォルダの下を再帰的に探し、`.jpg`・`.png` を全部使う。224×224 でなければ縮めて使う)。

```
~/jetracer/data/real_myteam/
  session1/*.jpg
  session2/*.jpg
```

**画面下の車体の写り込み (柱など) は学習させない**: 実画像に写る自車の部品の位置を、自分の車の profile の
`camera.realism.posts` (画像の正規化座標で `[x0, x1, y0]` を部品ごとに並べる。無ければ `[]`) に書き、
`SIM2REAL_PROFILE=<profile 名>` を付けて学習・検証・変換を実行する。その領域は学習の入力で塗りつぶし、変換後は sim の画素に戻す。
この位置は、実機の `camera_preproc` でマスクする領域と**同じ値**にすること。

## ② sim の画像を撮る (A)

sim のカメラを**自分の車に合わせてから**撮る。カメラの高さ・下向き角度・画角・縦横比は `vehicle_profile.camera` の 1 か所
([docs/calibration.md](calibration.md))。ここが実機と違うと、変換器は形の違いまで埋めようとして壁の位置を動かす (失敗の原因になる)。

```bash
source scripts/sim_env.sh
./scripts/record.sh 8 60 --unity --seed0 301 [--profile <自分の profile>]   # 8 本 × 60 秒、seed ごとに照明・床テープ等が変わる
$PY tools/sim2real/extract_frames.py "bags/ep_30*" -o ~/jetracer/data/sim_frames --every 2   # 2 枚に 1 枚 → 約 3,700 枚
```

A の場面も B と同じくらい散らしたい。教師は参照線をなぞるので、同じ場面が多くなる。本数 (seed) を増やすと照明・床テープ・観戦者が変わる。

## ③ 学習

```bash
$PY tools/sim2real/train_cut.py \
    --sim ~/jetracer/data/sim_frames --real ~/jetracer/data/real_myteam \
    --out ~/jetracer/runs/myteam_001 --iters 16000 --crop 176 --lambda-nce 2 --lambda-struct 10
```

| 引数 | 推奨 | 意味 |
|---|---|---|
| `--iters` | 16,000 (白紙から) / 8,000 (`--init` で追加学習) | 反復数。CPU (12 スレッドの Ryzen) で 0.57 s/反復 → 16,000 で約 2.5 時間 |
| `--crop` | 176 | 学習で使う切り出し [px]。推論は 224 全体で行うので、**小さすぎると構図が合わない** (128 で失敗) |
| `--lambda-nce` | 2 | 変換前後で同じ場所のパッチを対応させる強さ (形を保つ) |
| `--lambda-struct` | 10 | **形を保つ損失**。入力と出力を 1/4 に縮めた輝度の縁の強さを揃える。質感は変えてよく、壁の縁・床の境目の位置を守る |
| `--fast` | **使わない** | 軽量版 (FastCUT)。約 2.7 倍速いが、色を保つ仕組みを省くので崩れた (床に緑・水色の斑点) |
| `--resume` | — | 中断したところから続ける (`--out` の `ckpt.pt`) |
| `--init <ckpt.pt>` | — | 別の学習の重みから始める。カメラ設定や実画像を足したときの追加学習に (最初から学ぶより速い) |

途中経過は 1,000 反復ごとに `runs/<名前>/samples/NNNNNN.png` に出る (上段 = sim、下段 = 変換後、右端の列 = 実画像)。
終了時に `G.onnx` (+ `G.onnx.data`) が書き出される。

## ④ 検証

```bash
SIM2REAL_PROFILE=<自分の profile> $PY tools/sim2real/eval_geometry.py \
    --model ~/jetracer/runs/myteam_001/G.onnx --sim ~/jetracer/data/sim_frames --grid /tmp/s2r_check.png
```

- **数値**: 変換前後の縁の一致 (Sobel エッジの F 値、許容 2 px) の中央値。**全体 0.6 以上**を合格の目安にしている
  (学習していないモデルで 0.86。質感が付くぶん少し下がるのは正常。この PC の合格例 cut_003 は 0.83、不合格の cut_002 は 0.56)。下半分 (床) は質感で縁が増えるので低く出やすい。
- **目視**: `/tmp/s2r_check.png` (上段 = sim、下段 = 変換後) で次を確かめる。
  - 壁・走路の縁・滑り板・芝の**位置と形**が変わっていないか (変わると、学習の正解ラベル = 注視点の位置と画像がずれる)
  - 実物に無いもの (明るい塊・斑点・存在しない壁) を描いていないか
  - 芝の緑・滑り板の色のような**意味のある色**が消えていないか

## ⑤ 学習データの bag を変換

```bash
source scripts/sim_env.sh
SIM2REAL_PROFILE=<自分の profile> $PY tools/sim2real/convert_bag.py "bags/ep_30*" --model ~/jetracer/runs/myteam_001/G.onnx
# → bags/ep_301_..._s2r/ (画像だけ差し替え。stamp・/imu・/sim/ground_truth などはそのまま) と同名 .json (元の bag・モデル・変換時間)
```

## ⑥ 方策の学習に混ぜる

変換した bag (`*_s2r`) は**学習データの一部だけ** (目安 半分) に混ぜ、残りは変換していない (乱択化した) sim の bag にする。
変換器は実画像を撮った会場の見た目を覚えているので、それだけで学習すると**その会場に寄りすぎて、本番会場で外れる**。

## うまくいかないとき

| 症状 (samples / 検証画像) | 原因の見込み | 対処 |
|---|---|---|
| 床に緑・水色などの斑点、色が全体に崩れる | `--fast` や `--lambda-nce` が弱い | `--fast` を外す。`--lambda-nce 2` |
| 床に実物に無い明るい塊・影を描く、壁の縁が消える | 実画像に多い見た目 (照明の反射など) を形を無視してまねている | `--lambda-struct` を上げる (10 → 20)。A と B の場面の偏りを減らす |
| 壁・走路の位置がずれる、形が歪む | sim のカメラ設定が実機と違う | `vehicle_profile.camera` を較正し直して A を撮り直す |
| 芝の緑や滑り板の色が消える | B にその場面が少ない | その場面の実画像を B に足す |
| 画面下に変な模様が出る | 柱の位置が合っていない | `realism.posts` を自分の車に合わせ、`SIM2REAL_PROFILE` を付ける |
| 変換が遅い | CPU 推論 (約 57 ms/枚) | オフライン変換なので問題ない。走行中に使う計画なら GPU で測り直す |

## この PC での学習の記録

| 学習 | 設定 | 結果 |
|---|---|---|
| cut_001 | FastCUT・128 切り出し | 失敗: 床に緑・水色の斑点 |
| cut_002 | 標準 CUT・176 切り出し・λ_NCE 1・16,000 反復 | 不合格: 床の奥に明るい塊、滑り板・壁の縁が消える (F 値 0.559) |
| **cut_003** | cut_002 の重みから (`--init`)・λ_NCE 2・形を保つ損失 10・8,000 反復 (約 1.2 時間) | **合格**: F 値 0.832 (下半分 0.676)。空 → 会場の照明と骨組み、壁 → 実物の赤白、床 → 実物のカーペット。滑り板の水色・壁と床の位置も保たれた。モデルは `~/jetracer/runs/cut_003/G.onnx` |

詳細と途中の画像は [tools/sim2real/README.md](../tools/sim2real/README.md) と [results/2026-09-26](results/2026-09-26/README.md)。
