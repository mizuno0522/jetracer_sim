#!/bin/bash
# 学習データの記録: エピソード seed を変えながら N 本、教師で走らせて rosbag (mcap) に取る。
#   ./scripts/record.sh [本数 (既定 3)] [秒/本 (既定 60)] [--unity] [--seed0 <最初の seed (既定 1)>]
# 記録するトピック: /camera/image_raw /camera/camera_info /imu /actuator_cmd /lookahead /sim/ground_truth /sim/episode
# 出力: bags/ep_<seed>_<日時>/ (mcap) と同名の .json (seed・profile・秒数・git rev)。
# 乱択化 (imu_sim のターンオンバイアス等・Unity の照明) は seed 毎に引き直される (/sim/episode)。
# 既定は Unity 無し (OpenCV 描画)。実カメラ寄せの画像で取るなら --unity (setup_unity_player.sh 済みであること)。
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
N=3; SEC=60; UNITY=0; SEED0=1
while [ $# -gt 0 ]; do
  case "$1" in
    --unity) UNITY=1 ;;
    --seed0) SEED0="$2"; shift ;;
    -*) echo "不明な引数: $1" >&2; exit 1 ;;
    *) if [ "$N" = 3 ] && [ -z "${_n:-}" ]; then N="$1"; _n=1; else SEC="$1"; fi ;;
  esac
  shift
done
source "$HERE/sim_env.sh" >/dev/null
export PYTHONNOUSERSITE=1
mkdir -p "$ROOT/bags" "$ROOT/log"
PLAYER="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}/MinicarSim.x86_64"
if [ $UNITY -eq 1 ] && [ ! -x "$PLAYER" ]; then echo "Unity プレイヤーが無い: $PLAYER (scripts/setup_unity_player.sh)" >&2; exit 1; fi
TOPICS=(/camera/image_raw /camera/camera_info /imu /actuator_cmd /lookahead /sim/ground_truth /sim/episode)
REV=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)

cleanup() { "$HERE/stop_sim.sh" >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

for ((i = 0; i < N; i++)); do
  SEED=$((SEED0 + i))
  TAG="ep_${SEED}_$(date +%Y%m%d_%H%M%S)"
  OUT="$ROOT/bags/$TAG"
  echo "=== [$((i + 1))/$N] seed=$SEED ${SEC}s → $OUT ==="
  set -m
  if [ $UNITY -eq 1 ]; then
    DISPLAY="${DISPLAY:-:0}" ros2 launch minicar_sim sim_host.launch.py seed:=$SEED camera_backend:=unity \
      unity_player:="$PLAYER" tcp_port:="${JETRACER_TCP_PORT:-10001}" rviz:=false > "$ROOT/log/record_sim_$SEED.log" 2>&1 &
  else
    ros2 launch minicar_sim sim_host.launch.py seed:=$SEED camera_backend:=opencv unity_player:=none rviz:=false \
      > "$ROOT/log/record_sim_$SEED.log" 2>&1 &
  fi
  SIM=$!
  sleep $([ $UNITY -eq 1 ] && echo 12 || echo 5)
  ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true auto_run:=true > "$ROOT/log/record_stack_$SEED.log" 2>&1 &
  STK=$!
  set +m
  sleep 3
  timeout -s INT $((SEC + 2)) ros2 bag record -s mcap -o "$OUT" "${TOPICS[@]}" > "$ROOT/log/record_bag_$SEED.log" 2>&1 || true
  cat > "$OUT.json" <<JSON
{"seed": $SEED, "seconds": $SEC, "unity": $UNITY, "profile": "jetracer_tt02", "git": "$REV", "topics": $(printf '%s\n' "${TOPICS[@]}" | python3 -c 'import json,sys;print(json.dumps(sys.stdin.read().split()))'), "recorded_at": "$(date -Iseconds)"}
JSON
  kill -INT $STK $SIM 2>/dev/null || true
  for _ in $(seq 1 15); do kill -0 $SIM 2>/dev/null || kill -0 $STK 2>/dev/null || break; sleep 1; done
  "$HERE/stop_sim.sh" >/dev/null 2>&1 || true
  ros2 bag info "$OUT" 2>/dev/null | grep -E "Duration|Messages|image_raw|/imu " | sed 's/^/  /'
done
echo "完了: $(find "$ROOT/bags" -maxdepth 1 -type d -name "ep_*" | wc -l) 本 in $ROOT/bags"
