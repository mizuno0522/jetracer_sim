#!/bin/bash
# 複数 seed (照明・床・床テープ・観戦者が変わる) で方策と教師を同じ条件で評価し、1 行ずつ表にする。
#   ./tools/policy/eval_sweep.sh MODEL LABEL SEC SEED...
# 出力: <MODEL のフォルダ>/sweep_<LABEL>.tsv  (teacher のときは ~/jetracer/runs/teacher/)
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
MODEL="$1"; LABEL="$2"; SEC="$3"; shift 3
if [ "$MODEL" = teacher ]; then OUT="$HOME/jetracer/runs/teacher"; else OUT="$(dirname "$(realpath "$MODEL")")"; fi
mkdir -p "$OUT"; TSV="$OUT/sweep_$LABEL.tsv"
echo -e "seed\t周回\tラップ\tcte_rms\tcte_p95\t壁余裕最小\t衝突\tコース外" > "$TSV"
for s in "$@"; do
  "$ROOT/tools/policy/eval_closed_loop.sh" "$MODEL" "${LABEL}_s$s" "$SEC" "$s" > /dev/null 2>&1
  f="$OUT/eval_${LABEL}_s$s.txt"
  python3 - "$f" "$s" >> "$TSV" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
g = lambda p, d='-': (re.search(p, t) or [None, d])[1]
laps = re.search(r'ラップ ([\d./ ]+)', t)
print('\t'.join([sys.argv[2], g(r'周回 (\d+)'), laps[1].strip().split(' / ')[-1] if laps else '-',
                 g(r'rms ([\d.]+)'), g(r'p95 ([\d.]+)'), g(r'壁余裕 最小 ([\d.]+)'), g(r'衝突 (\d+)'), g(r'コース外 (\d+)')]))
PY
done
cat "$TSV"
