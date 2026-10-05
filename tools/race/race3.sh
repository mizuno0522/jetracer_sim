#!/bin/bash
# Unity で 3 台レース (青・黄・緑) を PC 1 台で動かす。JetRacer (TT-02) 版 (minicarbattle2026 の unity/race3.sh を移植、2026-09-29)。
#
#   ./tools/race/race3.sh                                   # 3 台とも教師 (gt_teacher)・矢印は周回ごとに左右を入れ替え
#   BLUE_MODEL=~/jetracer/runs/policy_003/policy.onnx ./tools/race/race3.sh   # 青だけ方策 (画像＋IMU) で走らせる
#   RECORD=~/Videos/race3.mp4 ./tools/race/race3.sh         # Unity の画面を録画 (カウントダウンから)
#
# 3 台はそれぞれ sim_host.launch.py + jetracer_stack を別の ROS_DOMAIN_ID で動かす (トピック名が絶対パスなので):
#   青 (42): カメラあり (Unity が描く)。教師、または BLUE_MODEL の policy_net
#   黄 (43) / 緑 (44): 教師・カメラなし (use_camera:=false。画像が無いので failsafe は切る)
# スタートは規約のスタートライン 1 / 2 / 3 (x = 2.56 / 4.36 / 6.16 m)。車体の前端を線に合わせる。
# tools/race/race_relay.py が各車の姿勢を他の 2 台へ /sim/rival_state・/sim/rival2_state として届け、
# 車どうしの衝突を各車の sim が判定する。Unity は青のドメインにだけ繋ぎ、3 台とも描く (ros_tcp_endpoint も青だけ)。
# 信号は 1 つ: 青の sim が周回ごとに切り替え (arrow_dir)、黄・緑の sim は中継で届く青の向きに従う (arrow_follow)。
# ★ 教師は同じ参照線をなぞるだけで、相手を避けない (追いつけば接触する)。
#
# 環境変数: D1/D2/D3 (ドメイン、既定 42/43/44)、TCP_PORT (既定 10001。M-05 の sim の 10000 と分ける)、
#   ARROW (alternate|random|left|right|center)、BLUE_MODEL、LOOKAHEAD_M (教師の注視距離)、
#   PLAYER (Unity プレイヤー。既定は scripts/pick_unity_player.sh が GPU を見て Built-in / URP / HDRP から選ぶ。
#   BLUE_MODEL を使うときは画像で走るので Built-in に固定。JETRACER_PIPELINE=builtin|urp|hdrp で指定もできる)、UNITY_FPS (既定 60)、
#   RECORD (mp4)・RECORD_FPS・RECORD_WIDTH・RECORD_FROM、UNITY_ARGS、MAX_S (走らせる秒数、既定 180)。
# Ctrl-C で全部止まる。ログは log/race3_*.log、2 台以上の位置の CSV は log/race3_pose_<ドメイン>.csv。
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
D1="${D1:-42}"; D2="${D2:-43}"; D3="${D3:-44}"
TCP_PORT="${TCP_PORT:-10001}"
ARROW="${ARROW:-alternate}"
LOG="$REPO/log"
mkdir -p "$LOG"
source "$REPO/scripts/sim_env.sh" > /dev/null
unset ROS_DOMAIN_ID

# スタート位置 (コース中心線の弧長。ライン 1/2/3 の弧長 0.56 / 2.36 / 4.16 m から、後軸 → 前端 0.344 m を引く)
S1=0.216; S2=2.016; S3=3.816
export RACE_POSE_LOG="${RACE_POSE_LOG:-$LOG/race3_pose}"
LA="lookahead_m:=${LOOKAHEAD_M:-0.5}"
COMMON="rviz:=false viz:=false arrow_dir:=$ARROW camera_backend:=unity unity_player:=none $LA"
PIDS=()
kill_tree() { local c; for c in $(pgrep -P "$1" 2>/dev/null); do kill_tree "$c"; done; kill -9 "$1" 2>/dev/null; }
cleanup() {
  trap - INT TERM
  echo "停止中..."
  local stop=()
  for d in $D1 $D2 $D3; do
    ROS_DOMAIN_ID=$d timeout 5 ros2 topic pub -1 /run std_msgs/Bool "data: false" >/dev/null 2>&1 &
    stop+=($!)
  done
  wait "${stop[@]}" 2>/dev/null
  # Unity は TERM で正常終了させる (録画中なら mp4 を閉じてから終わる)
  [ -n "${UNITY_PID:-}" ] && kill -TERM "$UNITY_PID" 2>/dev/null
  for _ in $(seq 20); do kill -0 "${UNITY_PID:-0}" 2>/dev/null || break; sleep 0.5; done
  kill -INT "${PIDS[@]}" 2>/dev/null
  for _ in $(seq 20); do kill -0 "${PIDS[@]}" 2>/dev/null || break; sleep 0.5; done
  # 自分が起動したプロセスの子孫だけを止める (名前で pkill すると別のレースや M-05 の sim を巻き込む)
  for p in "${PIDS[@]}"; do kill_tree "$p"; done
  for c in $(pgrep -P $$ 2>/dev/null); do kill_tree "$c"; done
}
trap 'cleanup; exit 0' INT TERM

# ★ set -m: 非対話シェルのバックグラウンド job は SIGINT 無視を継承するので、ジョブ制御を有効にしてから起動する
set -m
echo "青 を起動: ドメイン $D1、ライン 1 (カメラあり・Unity は TCP $TCP_PORT)"
ROS_DOMAIN_ID=$D1 ros2 launch minicar_sim sim_host.launch.py $COMMON tcp_port:=$TCP_PORT \
  use_camera:=true start_offset_m:=$S1 > "$LOG/race3_blue_sim.log" 2>&1 &
PIDS+=($!)
echo "黄 を起動: ドメイン $D2、ライン 2 (カメラなし)"
ROS_DOMAIN_ID=$D2 ros2 launch minicar_sim sim_host.launch.py $COMMON use_camera:=false arrow_follow:=true \
  start_offset_m:=$S2 > "$LOG/race3_yellow_sim.log" 2>&1 &
PIDS+=($!)
echo "緑 を起動: ドメイン $D3、ライン 3 (カメラなし)"
ROS_DOMAIN_ID=$D3 ros2 launch minicar_sim sim_host.launch.py $COMMON use_camera:=false arrow_follow:=true \
  start_offset_m:=$S3 > "$LOG/race3_green_sim.log" 2>&1 &
PIDS+=($!)

# 車両スタック (auto_run なし: race_start.py の /run で一斉に出る)
if [ -n "${BLUE_MODEL:-}" ]; then
  BLUE_STACK="model_file:=$BLUE_MODEL"; BLUE_NAME="BLUE (policy)"
else
  BLUE_STACK="teacher:=true"; BLUE_NAME="BLUE (teacher)"
fi
ROS_DOMAIN_ID=$D1 ros2 launch jetracer_stack vehicle_stack.launch.py $BLUE_STACK > "$LOG/race3_blue_stack.log" 2>&1 &
PIDS+=($!)
ROS_DOMAIN_ID=$D2 ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true failsafe:=false > "$LOG/race3_yellow_stack.log" 2>&1 &
PIDS+=($!)
ROS_DOMAIN_ID=$D3 ros2 launch jetracer_stack vehicle_stack.launch.py teacher:=true failsafe:=false > "$LOG/race3_green_stack.log" 2>&1 &
PIDS+=($!)

RELAY="$HERE/race_relay.py"
relay() {   # relay <src> <dst> <ログ名> [追加の引数]
  local s=$1 d=$2 n=$3; shift 3
  python3 "$RELAY" --src-domain "$s" --dst-domain "$d" "$@" > "$LOG/race3_relay_$n.log" 2>&1 &
  PIDS+=($!)
}
# 姿勢: 各車へ他の 2 台を /sim/rival_state・/sim/rival2_state として (車どうしの衝突・Unity の描画)
relay $D2 $D1 y2b; relay $D3 $D1 g2b --dst-topic /sim/rival2_state
relay $D1 $D2 b2y; relay $D3 $D2 g2y --dst-topic /sim/rival2_state
relay $D1 $D3 b2g; relay $D2 $D3 y2g --dst-topic /sim/rival2_state
# 信号を 1 つに: 青の sim の矢印を黄・緑の sim へ
relay $D1 $D2 arrow_y --msg int32 --src-topic /sim/arrow_dir --dst-topic /sim/arrow_master
relay $D1 $D3 arrow_g --msg int32 --src-topic /sim/arrow_dir --dst-topic /sim/arrow_master

REC_ARGS=()
[ -n "${RECORD:-}" ] && REC_ARGS=(-record "$RECORD" -recordfps "${RECORD_FPS:-30}" -recordwidth "${RECORD_WIDTH:-1280}" -recordfrom "${RECORD_FROM:-countdown}")
# プレイヤー: 指定が無ければ GPU を見て選ぶ。方策 (BLUE_MODEL) は画像で走るので、学習したのと同じ Built-in 版に固定する
if [ -z "${PLAYER:-}" ]; then
  if [ -n "${BLUE_MODEL:-}" ]; then PLAYER="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}/MinicarSim.x86_64"
  else PLAYER="$("$REPO/scripts/pick_unity_player.sh")"; fi
fi
"$PLAYER" -rosip 127.0.0.1 -rosport "$TCP_PORT" -layout aic -laps 0 \
  -fps "${UNITY_FPS:-60}" -ownlabel "$BLUE_NAME" -rivallabel "YELLOW (teacher)" -rival2label "GREEN (teacher)" \
  "${REC_ARGS[@]}" ${UNITY_ARGS:-} -logFile "$LOG/race3_unity.log" &
UNITY_PID=$!
PIDS+=($UNITY_PID)
set +m

echo "起動待ち..."
# 準備ができたかは各車のログで見る (ros2 topic で順に問い合わせると、負荷が高いとき 1 回ごとに DDS に
# 参加し直して遅い。M-05 で 183 s → 11 s)。/run の購読者がそろうのは race_start.py が待つ (最長 30 s)
ready() {   # ready <ログ> <文字列>...: すべての行が出たら 0
  local f=$1; shift
  for p in "$@"; do grep -aq "$p" "$f" 2>/dev/null || return 1; done
}
T_WAIT=$SECONDS
for c in blue yellow green; do
  until ready "$LOG/race3_${c}_sim.log" "VehicleSim started" && ready "$LOG/race3_${c}_stack.log" "cmd_shaper: L="; do
    [ $((SECONDS - T_WAIT)) -gt 180 ] && { echo "★$c が起動しない (log/race3_${c}_*.log)"; break; }
    sleep 0.5
  done
done
# 青は Unity のカメラ画像が流れ始めてから (方策が画像を受け取れる状態で) スタート
until ready "$LOG/race3_blue_sim.log" "RegisterPublisher(/camera/image_raw"; do
  [ $((SECONDS - T_WAIT)) -gt 180 ] && { echo "★Unity のカメラ画像が来ない (log/race3_unity.log)"; break; }
  sleep 0.5
done
echo "起動まで $((SECONDS - T_WAIT)) s"
sleep 2

echo "スタート! (カウントダウン 5 秒)"
python3 "$HERE/race_start.py" --min-subs 1 $D1 $D2 $D3
echo "走行中 (${MAX_S:-180} s)。Unity の [L] で表示切替。Ctrl-C で終了"
T0=$SECONDS
while [ $((SECONDS - T0)) -lt ${MAX_S:-180} ]; do sleep 1; done

echo ""
echo "=== 結果 (壁接触・車両接触の回数) ==="
for c in blue:青 yellow:黄 green:緑; do
  f="$LOG/race3_${c%%:*}_sim.log"
  echo "${c#*:}: 壁接触 $(grep -c '壁接触 #' "$f") 回、車両接触 $(grep -c '車両接触 #' "$f") 回"
done
sleep 1
cleanup
