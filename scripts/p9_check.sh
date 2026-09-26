#!/bin/bash
# P9 (sim PC ↔ Jetson の 1 GbE 直結) の起動前チェック。どちらのホストでも同じ。
#   ./scripts/p9_check.sh            # 相手 = 既定 (Jetson なら 192.168.10.2、PC なら 192.168.10.3)
#   ./scripts/p9_check.sh 192.168.10.2
# 通らない項目が 1 つでもあれば 2 ホストでの sim は始めない (docs/setup.md Step 5)。
set -u
ME=$(hostname)
PEER="${1:-}"
if [ -z "$PEER" ]; then
  if ip -br addr | grep -q "192.168.10.3/"; then PEER=192.168.10.2; else PEER=192.168.10.3; fi
fi
ok=0; ng=0
pass() { echo "ok   $1"; ok=$((ok+1)); }
fail() { echo "★NG  $1"; ng=$((ng+1)); }

# 1. 有線 IP
IF=$(ip -br addr | awk '/192\.168\.10\./{print $1}' | head -1)
[ -n "$IF" ] && pass "有線 $IF に 192.168.10.x がある" || fail "192.168.10.x の IP が無い (Step 1)"

# 2. 相手に届く
ping -c 2 -W 1 "$PEER" >/dev/null 2>&1 && pass "ping $PEER" || fail "ping $PEER が通らない (ケーブル・相手の IP)"

# 3. RMW と DDS 設定
[ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ] && pass "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" || fail "RMW_IMPLEMENTATION が cyclonedds でない (source scripts/sim_env.sh)"
[ "${ROS_LOCALHOST_ONLY:-0}" = "0" ] && pass "ROS_LOCALHOST_ONLY=0" || fail "ROS_LOCALHOST_ONLY=1 だと別ホストが見えない"
if ros2 pkg list 2>/dev/null | grep -q rmw_cyclonedds_cpp; then pass "rmw_cyclonedds_cpp 導入済み"; else fail "ros-humble-rmw-cyclonedds-cpp が無い (apt)"; fi
XML="${CYCLONEDDS_URI#file://}"
if [ -f "$XML" ]; then
  NIC=$(grep -o 'NetworkInterface name="[^"]*"' "$XML" | cut -d'"' -f2)
  [ "$NIC" = "$IF" ] && pass "$(basename "$XML") の NIC = $NIC" || fail "$(basename "$XML") の NIC が '${NIC:-指定なし}' (有線は $IF)。source scripts/sim_env.sh で有線版に切り替わる (リンク UP のとき)"
else
  fail "CYCLONEDDS_URI のファイルが無い ($XML)"
fi

# 4. 受信バッファ
R=$(sysctl -n net.core.rmem_max)
[ "$R" -ge 16777216 ] && pass "net.core.rmem_max=$R" || fail "net.core.rmem_max=$R (< 16 MB。画像を取りこぼす。Step 0 の sysctl)"

# 5. 時刻同期 (両ホストとも chrony。ずれ 1 ms 未満)
if command -v chronyc >/dev/null; then
  OFF=$(chronyc tracking 2>/dev/null | awk '/System time/{print $4}')
  REF=$(chronyc tracking 2>/dev/null | awk '/Reference ID/{print $NF}')
  LEAP=$(chronyc tracking 2>/dev/null | awk -F': ' '/Leap status/{print $2}')
  if [ "$LEAP" != "Normal" ] || [ -z "$REF" ] || [ "$REF" = "()" ]; then
    fail "chrony 未同期 (Leap status: ${LEAP:-?}, 参照 ${REF:-無し})。相手の chrony server が上がってから数分待つ (chronyc sources -v)"
  elif awk -v o="$OFF" 'BEGIN{exit !(o < 0.001)}'; then pass "chrony ずれ ${OFF}s (参照 $REF)"
  else fail "chrony ずれ ${OFF}s (参照 $REF)。1 ms 未満になるまで待つ"; fi
else
  fail "chrony が無い (apt install chrony)"
fi

# 6. ROS_DOMAIN_ID
[ "${ROS_DOMAIN_ID:-0}" = "42" ] && pass "ROS_DOMAIN_ID=42" || fail "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未設定} (両ホスト 42)"

echo "---- $ME: ok $ok / NG $ng"
[ $ng -eq 0 ] && echo "次: 相手で sim を上げて  ros2 topic hz /imu  と  ros2 topic hz /camera/image_raw  をこちらで取る (100 Hz / 15 Hz)"
exit $ng
