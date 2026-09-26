# 方策 (policy_net) の模倣学習 — sim で学習して sim の閉ループで評価する

`jetracer_stack/policy_net.py` の約束 (画像 224×224 ＋ IMU 直近 50 サンプル → 注視点 u, v ∈ [-1,1] と速度係数 s ∈ [0,1]) に合わせた ONNX を作る。

| 手順 | コマンド | 内容 |
|---|---|---|
| 1. 記録 | `tools/policy/record_policy_data.sh 30 60 401` | 教師 (gt_teacher) の出力に揺らぎ (`noisy_lookahead.py`: 注視点の横位置に OU 雑音と押し出し) を足して走らせ、コースから外れかけて戻る場面を含める。正解は真値から作るので揺らいでも正しい |
| 2. 変換 | `tools/sim2real/convert_bag.py` | 半分 (奇数 seed) を実画像風に変換 ([docs/sim2real.md](../../docs/sim2real.md)) |
| 3. データ化 | `tools/policy/build_dataset.py` | 画像ごとに IMU 窓と正解 (u, v, s, valid) を並べる。s は gt_teacher と同じ式 (曲率と区間) |
| 4. 学習 | `tools/policy/train_policy.py` | ResNet18 (ImageNet 事前学習) ＋ IMU の 1 次元畳み込み。CPU で約 1.2〜1.7 s/反復 (バッチ 32)。`--export --zero-imu / --zero-image` で目隠しテスト用の ONNX |
| 5. 評価 | `tools/policy/eval_closed_loop.sh <onnx> <ラベル> [秒] [seed] [--video]` | sim (Unity 描画) の閉ループで `tools/lap_eval.py`。policy_net は venv の Python で動かす (システムの Python に onnxruntime が無いため)。`<onnx>` に `teacher` と書くと比較用に教師で走る |
| 5'. 複数 seed | `tools/policy/eval_sweep.sh <onnx> <ラベル> 60 901 902 ...` | seed ごとに照明・床・床テープ・観戦者が変わる。1 行 1 seed の表 (`sweep_<ラベル>.tsv`) |
| 6. 実画像 | `tools/policy/offline_check.py` / `real_lag_check.py` | 走らせずに確かめる。検証 bag の誤差 (sim / 変換後) と、実走ログの画像で予測した注視点と人の舵の相関 |
| 一括 | `tools/policy/night_pipeline.sh <学習の上限時間>` | 2〜5 を順に。ログは `log/night_pipeline.log` |

必要なもの: `scripts/setup_sim2real.sh` の venv ＋ `pip install torchvision matplotlib` (CPU 版)。

追加学習: `train_policy.py --init <ckpt.pt> --epochs 3 --lr 1e-4 [--strong-aug]` (`--strong-aug` = 動きのぼけ・白飛び・ガンマ・遮り)。

書き出しの注意: ONNX は opset 18 で出る (17 への変換が通らず、そのまま 18 で保存される。ログに Traceback が出るが書き出しは成功)。
重みは `policy.onnx.data` に分かれるので、**2 つのファイルを同じフォルダに置く**。

## 結果 (2026-09-27、この PC の CPU)

学習データ: 揺らぎ入り教師で 30 本 × 60 s (seed 401〜430)。奇数 seed は cut_003 で実画像風に変換。検証 = seed 428〜430。
閉ループ評価は学習に使っていない seed (900〜907)。

| モデル | 学習 | 検証誤差 u / v / s | sim 閉ループ (seed 900〜906) | 実画像: 予測 u と人の舵の相関 |
|---|---|---|---|---|
| 教師 (gt_teacher、比較用) | — | — | 全 seed で完走、ラップ 24.0 s、\|cte\| rms 0.050 m、衝突 0 | — |
| **policy_001** | ImageNet から 8 エポック (約 1.8 時間) | 3.28 px / 1.60 px / 0.020 | **全 seed で完走**、ラップ 23.9〜24.2 s、\|cte\| rms 0.025〜0.034 m、衝突 0 | +0.21 / +0.23 |
| policy_001 の IMU を 0 に | (目隠しテスト) | — | seed 900 で完走 (ほぼ変わらない) | — |
| policy_001 の画像を 0 に | (目隠しテスト) | — | **走れない** (衝突 36 回) | — |
| policy_002 | 001 から + 強い拡張 3 エポック | 3.34 / 1.61 / 0.020 | (未評価) | +0.11 / +0.11 (悪化) |
| policy_003 | 001 から + 学習 bag を**全部**変換して 3 エポック | 3.11 / 1.58 / 0.020 | **全 seed で完走**、ラップ 23.4〜23.6 s、\|cte\| rms 0.030〜0.035 m、衝突 0 | **+0.29 / +0.30** |

- 相関は `real_lag_check.py` の実走ログ 2 本 (9/12 の log_2・log_5、人が PS5 のコントローラで操縦) で、ずれ 0 の値。人の舵は見てから遅れて切る・
  切り方が人によって違う (-1 に張り付く) ので、1 にはならない。**目安の比較用**。
- 目隠しテスト: 画像を消すと走れず、IMU を消してもほぼ変わらない → **方策は画像で走っている** (IMU はほとんど使っていない)。
- policy_003 の相関の上がり方には注意: 変換器 cut_003 は同じ 9/12 の会場の実画像で学習しているので、**その会場に寄った**ぶんを含む。
  別の会場では確かめていない。docs/sim2real.md の「変換 bag は半分」は、この寄りすぎを避けるための目安。

