#!/bin/bash
# 走行ゼロ・Unity 無しの動作確認: 単体テスト → 閉ループ起動 → レート → 教師で 1 周 (lap_eval) → /sim/ 購読禁止。
#   ./scripts/smoke_test.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$HERE/../ros_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
LOG="$HERE/../log"; mkdir -p "$LOG"

echo "=== 1/3 単体テスト ==="
"$HERE/test.sh"

echo "=== 2/3 閉ループ (opencv・teacher) ==="
set -m
ros2 launch minicar_sim sim_host.launch.py camera_backend:=opencv unity_player:=none rviz:=false > "$LOG/smoke_sim.log" 2>&1 &
SIM=$!
sleep 5
ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true auto_run:=true > "$LOG/smoke_stack.log" 2>&1 &
STK=$!
set +m
cleanup() {
  kill -INT $STK $SIM 2>/dev/null
  for _ in $(seq 1 15); do kill -0 $SIM 2>/dev/null || kill -0 $STK 2>/dev/null || break; sleep 1; done
  "$HERE/stop_sim.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT
sleep 15
fail=0
rate() {  # topic 期待Hz 許容%
  local hz; hz=$(timeout 6 ros2 topic hz "$1" 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
  if [ -z "$hz" ]; then echo "NG  $1: 受信なし"; fail=1; return; fi
  awk -v h="$hz" -v e="$2" -v p="$3" 'BEGIN{ok=(h>e*(1-p/100)&&h<e*(1+p/100)); printf("%s  %s: %.1f Hz (期待 %s)\n", ok?"OK ":"NG ", "'"$1"'", h, e); exit ok?0:1}' || fail=1
}
rate /imu 100 10
rate /camera/image_raw 15 10
rate /actuator_cmd 30 10
# 教師で 1 周以上走らせて、周回・|cte| p95・衝突を tools/lap_eval.py で判定 (1 周 ≈ 23 s + スタート)
python3 "$HERE/../tools/lap_eval.py" --seconds 50 --min-laps 1 --max-cte-p95 0.30 --no-collision || fail=1
if grep -E "process has died" "$LOG/smoke_sim.log" "$LOG/smoke_stack.log" | grep -qv "exit code 0"; then
  grep -E "process has died" "$LOG/smoke_sim.log" "$LOG/smoke_stack.log" | grep -v "exit code 0" | head -3
  echo "NG  起動中に死んだノードがある (上)"; fail=1
fi

echo "=== 3/3 実機に無い入力の購読禁止 ==="
"$HERE/check_no_sim_topics.sh" /cmd_shaper /failsafe || fail=1
[ $fail -eq 0 ] && echo "smoke test PASS" || { echo "smoke test FAIL (ログ: $LOG)"; exit 1; }
