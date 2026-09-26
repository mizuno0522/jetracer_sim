#!/bin/bash
# 不変量: 推論スタック (Jetson に載るノード) が /sim/ 名前空間を購読していないことを機械的に確かめる。
# 「sim では通ったが実機で動かない」の典型的な原因。vehicle_stack.launch.py を teacher:=false で起動した状態で実行する。
#   ./scripts/check_no_sim_topics.sh            # 既定のノード一覧
#   ./scripts/check_no_sim_topics.sh /policy_net /cmd_shaper
set -u
NODES=("$@")
if [ ${#NODES[@]} -eq 0 ]; then NODES=(/cmd_shaper /failsafe /policy_net /camera_node /jetracer_bridge); fi
bad=0
for n in "${NODES[@]}"; do
  info=$(ros2 node info "$n" 2>/dev/null) || { echo "skip $n (起動していない)"; continue; }
  subs=$(echo "$info" | awk '/Subscribers:/{f=1;next}/Publishers:/{f=0}f' | grep -E '^\s+/sim/' || true)
  if [ -n "$subs" ]; then echo "★ $n が /sim/ を購読している:"; echo "$subs"; bad=1; else echo "ok  $n"; fi
done
exit $bad
