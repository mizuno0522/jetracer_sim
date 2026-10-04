#!/bin/bash
# 富士スピードウェイで 3 台を走らせる (見せる用): P1 787B・P2 ロードスター ND・P3 RX-7。
#
#   ./tools/race/race3_fuji.sh                         # 150 秒走って終わる
#   RECORD=~/Videos/fuji3.mp4 MAX_S=240 ./tools/race/race3_fuji.sh
#
# tools/race/race3.sh と同じ仕組み: 3 台はそれぞれ sim_host.launch.py を別の ROS_DOMAIN_ID で動かし (物理は車ごとの
# vehicle_profile)、tools/race/race_relay.py が互いの姿勢を /sim/rival_state・/sim/rival2_state として届ける。
# Unity は P1 のドメインにだけ繋ぎ、3 台とも描く。運転は tools/demo/centerline_driver.py (中心線を横へずらした線を追う。
# 相手は避けない。線を分けてあるので並んでも当たらない)。
# 環境変数: D1/D2/D3 (既定 42/43/44)、TCP_PORT (10001)、PLAYER、QUALITY (medium)、RECORD、MAX_S (150)、WIDTH/HEIGHT。
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
D1="${D1:-42}"; D2="${D2:-43}"; D3="${D3:-44}"
TCP_PORT="${TCP_PORT:-10001}"
LOG="$REPO/log"; mkdir -p "$LOG"
source "$REPO/scripts/sim_env.sh" > /dev/null
unset ROS_DOMAIN_ID
cd "$REPO"
PIDS=()
kill_tree() { local c; for c in $(pgrep -P "$1" 2>/dev/null); do kill_tree "$c"; done; kill -9 "$1" 2>/dev/null; }
cleanup() {
  trap - INT TERM
  [ -n "${UNITY_PID:-}" ] && kill -INT "$UNITY_PID" 2>/dev/null      # 録画中なら mp4 を閉じてから終わる
  for _ in $(seq 20); do kill -0 "${UNITY_PID:-0}" 2>/dev/null || break; sleep 0.5; done
  kill -INT "${PIDS[@]}" 2>/dev/null
  sleep 3
  for p in "${PIDS[@]}"; do kill_tree "$p"; done
}
trap 'cleanup; exit 0' INT TERM
set -m
# 車: 名前・ドメイン・vehicle_profile・スタート位置 [m]・横の位置 [m, 左が正]・運転の上限 (最高速 m/s・横 G・減速)
car() {   # car <名前> <ドメイン> <profile> <s> <横> <vmax> <alat> <decel> <カメラ>
  ROS_DOMAIN_ID=$2 ros2 launch minicar_sim sim_host.launch.py course:=fuji vehicle_profile:=$3 camera_backend:=unity \
    unity_player:=none rviz:=false viz:=false use_camera:=$9 tcp_port:=$TCP_PORT start_offset_m:=$4 start_lateral_m:=$5 \
    > "$LOG/fuji3_$1_sim.log" 2>&1 &
  PIDS+=($!)
}
car p1 $D1 real_b787 40 0.0 85 15 12 true
car p2 $D2 real_nd   25 -3.5 50 8 6 false
car p3 $D3 real_rx7  10 3.5 62 8.5 7 false
relay() { local s=$1 d=$2; shift 2; python3 "$HERE/race_relay.py" --src-domain "$s" --dst-domain "$d" "$@" > /dev/null 2>&1 & PIDS+=($!); }
relay $D2 $D1; relay $D3 $D1 --dst-topic /sim/rival2_state
relay $D1 $D2; relay $D3 $D2 --dst-topic /sim/rival2_state
relay $D1 $D3; relay $D2 $D3 --dst-topic /sim/rival2_state
REC=()
[ -n "${RECORD:-}" ] && REC=(-record "$RECORD" -recordfps 30 -recordwidth 1280)
DRI_PRIME="${DRI_PRIME:-1}" "${PLAYER:-$HOME/jetracer/unity/player/MinicarSim.x86_64}" -rosip 127.0.0.1 -rosport "$TCP_PORT" -layout aic -laps 0 \
  -fps 60 -course course_fuji_real_b787.json -owncar b787 -rivalcar nd -rival2car rx7 -quality "${QUALITY:-medium}" -sound "${SOUND:-on}" \
  -ownlabel "P1 787B" -rivallabel "P2 ROADSTER" -rival2label "P3 RX-7" -screen-width "${WIDTH:-1600}" -screen-height "${HEIGHT:-900}" \
  "${REC[@]}" -logFile "$LOG/fuji3_unity.log" &
UNITY_PID=$!
set +m
echo "起動待ち..."
T0=$SECONDS
for c in p1 p2 p3; do
  until grep -aq "VehicleSim started" "$LOG/fuji3_${c}_sim.log" 2>/dev/null; do
    [ $((SECONDS - T0)) -gt 90 ] && { echo "★$c が起動しない ($LOG/fuji3_${c}_sim.log)"; break; }
    sleep 0.5
  done
done
sleep 8
echo "スタート"
drive() { ROS_DOMAIN_ID=$1 python3 "$REPO/tools/demo/centerline_driver.py" --course unity/course_fuji_$2.json --lateral $3 --vmax $4 --alat $5 --decel $6 > "$LOG/fuji3_drv_$2.log" 2>&1 & PIDS+=($!); }
drive $D1 real_b787 0.0 85 15 12
drive $D2 real_nd -3.5 50 8 6
drive $D3 real_rx7 3.5 62 8.5 7
T0=$SECONDS
while [ $((SECONDS - T0)) -lt ${MAX_S:-150} ]; do sleep 1; done
for c in p1 p2 p3; do echo "$c: 壁接触 $(grep -c '壁接触 #' "$LOG/fuji3_${c}_sim.log") 回、車両接触 $(grep -c '車両接触 #' "$LOG/fuji3_${c}_sim.log") 回"; done
cleanup
