# 方策 (policy_net) の模倣学習 — sim で学習して sim の閉ループで評価する

`jetracer_stack/policy_net.py` の約束 (画像 224×224 ＋ IMU 直近 50 サンプル → 注視点 u, v ∈ [-1,1] と速度係数 s ∈ [0,1]) に合わせた ONNX を作る。

| 手順 | コマンド | 内容 |
|---|---|---|
| 1. 記録 | `tools/policy/record_policy_data.sh 30 60 401` | 教師 (gt_teacher) の出力に揺らぎ (`noisy_lookahead.py`: 注視点の横位置に OU 雑音と押し出し) を足して走らせ、コースから外れかけて戻る場面を含める。正解は真値から作るので揺らいでも正しい |
| 2. 変換 | `tools/sim2real/convert_bag.py` | 半分 (奇数 seed) を実画像風に変換 ([docs/sim2real.md](../../docs/sim2real.md)) |
| 3. データ化 | `tools/policy/build_dataset.py` | 画像ごとに IMU 窓と正解 (u, v, s, valid) を並べる。s は gt_teacher と同じ式 (曲率と区間) |
| 4. 学習 | `tools/policy/train_policy.py` | ResNet18 (ImageNet 事前学習) ＋ IMU の 1 次元畳み込み。CPU で約 1.2〜1.7 s/反復 (バッチ 32)。`--export --zero-imu / --zero-image` で目隠しテスト用の ONNX |
| 5. 評価 | `tools/policy/eval_closed_loop.sh <onnx> <ラベル>` | sim (Unity 描画) の閉ループで `tools/lap_eval.py`。policy_net は venv の Python で動かす (システムの Python に onnxruntime が無いため) |
| 一括 | `tools/policy/night_pipeline.sh <学習の上限時間>` | 2〜5 を順に。ログは `log/night_pipeline.log` |

必要なもの: `scripts/setup_sim2real.sh` の venv ＋ `pip install torchvision` (CPU 版)。
