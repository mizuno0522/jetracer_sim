# 結果 2026-09-27 (sim PC) — 方策 (policy_net) の模倣学習

sim で学習した policy_net (画像 224×224 ＋ IMU → 注視点と速度係数) を、sim の閉ループと実画像で確かめた。
手順と表は [../../../tools/policy/README.md](../../../tools/policy/README.md)。

## 動画

- [`policy_001_closed_loop.mp4`](policy_001_closed_loop.mp4) (45 s・960 幅): **学習した方策が画像で走っている様子** (seed 907、学習に使っていない照明・床)。
  上 = 追従視点とミニマップ、下 = 方策に入っている 224×224 のカメラ画像。参照線も真値も使っていない (`/sim/*` を購読しない)。
  60 s で 2 周、ラップ 24.1 s、衝突 0。

## 実画像での予測

| 画像 | 内容 |
|---|---|
| [`policy_001_real_aimpoints.png`](policy_001_real_aimpoints.png) | 9/12 の実画像 24 枚に policy_001 の注視点 (緑の丸と線) を描いた。左上は人の舵 st と予測した速度係数 s |
| [`policy_003_real_aimpoints.png`](policy_003_real_aimpoints.png) | 同じ画像に policy_003 (学習 bag を全部実画像風に変換して追加学習) |
| [`policy_real_steering_log2.png`](policy_real_steering_log2.png) | 実走ログ log_2 の最初の 60 s。黒 = 人の舵、青 = policy_001 の注視点 u、橙 = policy_002。曲がる向きは概ね揃う |

## 数値 (要点)

| | sim 閉ループ (学習に使っていない seed) | 実画像: 予測 u と人の舵の相関 |
|---|---|---|
| 教師 (真値で走る) | 7 seed 完走、\|cte\| rms 0.050 m | — |
| policy_001 | 7 seed 完走、\|cte\| rms 0.025〜0.034 m、衝突 0 | +0.21 / +0.23 |
| policy_003 | 7 seed 完走、ラップ 23.4〜23.6 s、\|cte\| rms 0.030〜0.035 m、衝突 0 | +0.29 / +0.30 |
