#!/bin/bash
# 方策 (policy_net) の学習データを記録する。教師 (gt_teacher) の走りに揺らぎを足し、戻りの場面を含める。
# 正解ラベルは /sim/ground_truth (真値) から付けるので、走りが揺らいでもラベルは正しい。
#   tools/policy/record_policy_data.sh <本数> <秒/本> [seed0 (既定 401)]
# 出力: bags/pol_<seed>_<日時>/ (mcap) と .json。Unity 描画 (実カメラ寄せ・推定カメラ)。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
N="${1:-30}"; SEC="${2:-60}"; SEED0="${3:-401}"
source "$ROOT/scripts/sim_env.sh" >/dev/null
export PYTHONNOUSERSITE=1
PLAYER="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}/MinicarSim.x86_64"
PARAMS="$ROOT/ros_ws/install/jetracer_stack/share/jetracer_stack/config/stack.yaml"
TOPICS=(/camera/image_raw /camera/camera_info /imu /actuator_cmd /lookahead /lookahead_clean /sim/ground_truth /sim/episode)
REV=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)
mkdir -p "$ROOT/bags" "$ROOT/log"
cleanup() { "$ROOT/scripts/stop_sim.sh" >/dev/null 2>&1 || true; pkill -INT -f "^python3 $HERE/noisy_lookahead.py" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
for ((i = 0; i < N; i++)); do
  SEED=$((SEED0 + i))
  # 揺らぎの強さを本ごとに変える (弱い / 中 / 強い の繰り返し)
  case $((i % 3)) in 0) SIG=8; KR=0.05 ;; 1) SIG=18; KR=0.15 ;; *) SIG=28; KR=0.25 ;; esac
  OUT="$ROOT/bags/pol_${SEED}_$(date +%Y%m%d_%H%M%S)"
  echo "=== [$((i + 1))/$N] seed=$SEED sigma=${SIG}px kick=${KR}/s → $(basename "$OUT") ==="
  set -m
  DISPLAY="${DISPLAY:-:0}" ros2 launch minicar_sim sim_host.launch.py seed:=$SEED camera_backend:=unity \
    unity_player:="$PLAYER" tcp_port:=10001 rviz:=false > "$ROOT/log/pol_sim_$SEED.log" 2>&1 &
  sleep 12
  ros2 run jetracer_stack gt_teacher --ros-args --params-file "$PARAMS" -r /lookahead:=/lookahead_clean > "$ROOT/log/pol_teacher_$SEED.log" 2>&1 &
  python3 "$HERE/noisy_lookahead.py" --sigma $SIG --kick-rate $KR --seed $SEED > "$ROOT/log/pol_noise_$SEED.log" 2>&1 &
  ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=false auto_run:=true > "$ROOT/log/pol_stack_$SEED.log" 2>&1 &
  set +m
  sleep 4
  timeout -s INT $((SEC + 2)) ros2 bag record -s mcap -o "$OUT" "${TOPICS[@]}" > "$ROOT/log/pol_bag_$SEED.log" 2>&1 || true
  cat > "$OUT.json" <<JSON
{"seed": $SEED, "seconds": $SEC, "unity": 1, "noise_sigma_px": $SIG, "kick_rate": $KR, "profile": "jetracer_tt02", "git": "$REV", "recorded_at": "$(date -Iseconds)"}
JSON
  cleanup
  sleep 2
  ros2 bag info "$OUT" 2>/dev/null | grep -E "image_raw|Duration" | sed 's/^/  /' | head -2
done
echo "完了: $(find "$ROOT/bags" -maxdepth 1 -type d -name 'pol_*' | wc -l) 本"
