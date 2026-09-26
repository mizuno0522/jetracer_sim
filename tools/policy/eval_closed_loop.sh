#!/bin/bash
# 学習した policy_net を sim の閉ループで走らせ、周回・横偏差・衝突を測る (tools/lap_eval.py)。
#   tools/policy/eval_closed_loop.sh <policy.onnx> <ラベル> [秒 (既定 90)] [seed (既定 900)] [--video]
# 結果: <onnx のフォルダ>/eval_<ラベル>.txt (と --video なら eval_<ラベル>.mp4)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
if [ "$1" = teacher ]; then MODEL=teacher; else MODEL="$(realpath "$1")"; fi; LABEL="$2"; SEC="${3:-90}"; SEED="${4:-900}"; VIDEO="${5:-}"
OUTDIR="$(dirname "$MODEL")"; [ "$MODEL" = teacher ] && OUTDIR="${OUTDIR_TEACHER:-$HOME/jetracer/runs/teacher}"; mkdir -p "$OUTDIR"
source "$ROOT/scripts/sim_env.sh" >/dev/null
export PYTHONNOUSERSITE=1
PLAYER="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}/MinicarSim.x86_64"
REC=()
[ "$VIDEO" = "--video" ] && REC=(record:="$OUTDIR/eval_${LABEL}.mp4")
cleanup() { "$ROOT/scripts/stop_sim.sh" >/dev/null 2>&1 || true; pkill -INT -f "^$HOME/jetracer/venv_sim2real/bin/python -c from jetracer_stack.policy_net" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
ros2 daemon stop >/dev/null 2>&1 || true
set -m
DISPLAY="${DISPLAY:-:0}" ros2 launch minicar_sim sim_host.launch.py seed:=$SEED camera_backend:=unity \
  unity_player:="$PLAYER" tcp_port:=10001 rviz:=false "${REC[@]}" > "$ROOT/log/evalpol_sim_$LABEL.log" 2>&1 &
sleep 12
# policy_net は ONNX Runtime が要る。システムの Python (PYTHONNOUSERSITE=1) には無いので、
# launch の policy_net は model_file 空で待機させ、同じコードを venv の Python で別名のノードとして動かす
PY=~/jetracer/venv_sim2real/bin/python
if [ "$MODEL" = teacher ]; then   # 比較用: sim の真値で走る教師 (gt_teacher)
ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true auto_run:=true \
  > "$ROOT/log/evalpol_stack_$LABEL.log" 2>&1 &
else
ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=false auto_run:=true \
  > "$ROOT/log/evalpol_stack_$LABEL.log" 2>&1 &
"$PY" -c 'from jetracer_stack.policy_net import main; main()' --ros-args -r __node:=policy_net_onnx \
  -p model_file:="$MODEL" -p imu_window:=50 -p mask_bottom_frac:=0.0 > "$ROOT/log/evalpol_net_$LABEL.log" 2>&1 &
fi
set +m
sleep 6
{
  echo "# $LABEL  model=$MODEL  seed=$SEED  ${SEC}s  $(date -Iseconds)"
  python3 "$ROOT/tools/lap_eval.py" --seconds "$SEC" 2>&1 || true
  echo "--- policy_net の状態"; grep -iE "error|モデル|model|Traceback" "$ROOT/log/evalpol_net_$LABEL.log" | head -5
} | tee "$OUTDIR/eval_${LABEL}.txt"
