#!/bin/bash
# 不変量: 推論スタック (Jetson に載るノード) は、実機にもある入力しか購読しない。
# sim は /odom・/lane_info・/imu/yaw・/perception/ground_speed・/opponent_info などにも真値を出している
# (M-05 と共通の vehicle_sim)。/sim/ 以外の名前でも真値は流れるので、「/sim/ を読まない」ではなく
# 「許可した入力だけを読む」で確かめる。これを破ると「sim では通ったが実機で動かない」になる。
# vehicle_stack.launch.py を teacher:=false で起動した状態で実行する (gt_teacher は sim 専用の例外)。
#   ./scripts/check_no_sim_topics.sh            # 既定のノード一覧
#   ./scripts/check_no_sim_topics.sh /policy_net /cmd_shaper
#   ALLOW_EXTRA="/foo /bar" ./scripts/check_no_sim_topics.sh   # 実機にもある入力を足すとき
set -u
NODES=("$@")
if [ ${#NODES[@]} -eq 0 ]; then NODES=(/cmd_shaper /failsafe /policy_net /camera_node /jetracer_bridge); fi
# 実機にもある入力: カメラ・IMU・スタック内の注視点・走行開始・指令。/parameter_events は ROS の標準
ALLOW=(/camera/image_raw /imu /lookahead /run /actuator_cmd /parameter_events ${ALLOW_EXTRA:-})
bad=0
for n in "${NODES[@]}"; do
  info=$(ros2 node info "$n" 2>/dev/null) || { echo "skip $n (起動していない)"; continue; }
  subs=$(echo "$info" | awk '/Subscribers:/{f=1;next}/Publishers:/{f=0}f' | sed -n 's/^\s*\(\/[^:]*\):.*/\1/p')
  ng=""
  for t in $subs; do
    ok=0; for a in "${ALLOW[@]}"; do [ "$t" = "$a" ] && ok=1 && break; done
    [ $ok -eq 0 ] && ng="$ng $t"
  done
  if [ -n "$ng" ]; then echo "★ $n が実機に無い入力を購読している:$ng"; bad=1; else echo "ok  $n"; fi
done
exit $bad
