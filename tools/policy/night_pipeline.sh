#!/bin/bash
# 夜間の一括処理: 録画の完了待ち → 半分を sim→real 変換 → データ化 → 方策の学習 → 陽性対照のモデル → 閉ループ評価
#   tools/policy/night_pipeline.sh <学習の上限時間 (h)>
# (set -u は ROS の setup.bash が未定義変数を参照するので使わない)
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
HOURS="${1:-3.5}"
PY=~/jetracer/venv_sim2real/bin/python
RUN=~/jetracer/runs/policy_001
DATA=~/jetracer/data/policy
LOG="$ROOT/log/night_pipeline.log"
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
cd "$ROOT"
source scripts/sim_env.sh >/dev/null
export PYTHONNOUSERSITE=1

say "録画の完了待ち"
while pgrep -f "^/bin/bash tools/policy/record_policy_data.sh" >/dev/null; do sleep 20; done
say "録画完了: $(find bags -maxdepth 1 -type d -name 'pol_4*' ! -name '*_s2r' | wc -l) 本"

say "奇数 seed の bag を sim→real 変換 (cut_003)"
ODD=$(find bags -maxdepth 1 -type d -name 'pol_4*' ! -name '*_s2r' | sort | awk -F_ '$2 % 2 == 1')
$PY tools/sim2real/convert_bag.py $ODD --model ~/jetracer/runs/cut_003/G.onnx >> "$LOG" 2>&1
say "変換完了: $(find bags -maxdepth 1 -type d -name 'pol_4*_s2r' | wc -l) 本"

say "データ化 (偶数 seed = 変換なし、奇数 seed = 変換あり)"
rm -rf "$DATA"; mkdir -p "$DATA"
EVEN=$(find bags -maxdepth 1 -type d -name 'pol_4*' ! -name '*_s2r' | sort | awk -F_ '$2 % 2 == 0')
S2R=$(find bags -maxdepth 1 -type d -name 'pol_4*_s2r' | sort)
$PY tools/policy/build_dataset.py $EVEN $S2R -o "$DATA" >> "$LOG" 2>&1
say "データ: $(tail -1 "$LOG")"

say "学習 (上限 ${HOURS} h)"
$PY tools/policy/train_policy.py --data "$DATA" --val pol_428,pol_429,pol_430 --out "$RUN" --epochs 8 --hours "$HOURS" >> "$RUN.log" 2>&1
say "学習完了: $(grep EPOCH "$RUN.log" | tail -1)"

say "陽性対照のモデル (IMU ゼロ・画像ゼロ)"
$PY tools/policy/train_policy.py --out "$RUN" --export --zero-imu >> "$LOG" 2>&1
$PY tools/policy/train_policy.py --out "$RUN" --export --zero-image >> "$LOG" 2>&1

say "閉ループ評価"
for m in policy policy_zero_imu policy_zero_image; do
  tools/policy/eval_closed_loop.sh "$RUN/$m.onnx" "$m" 90 900 >> "$LOG" 2>&1
  say "  $m: $(grep -E '^周回|衝突' "$RUN/eval_$m.txt" | tr '\n' ' ')"
done
tools/policy/eval_closed_loop.sh "$RUN/policy.onnx" "policy_video" 90 900 --video >> "$LOG" 2>&1
say "完了"
