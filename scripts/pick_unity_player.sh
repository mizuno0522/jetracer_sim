#!/bin/bash
# この PC の GPU を見て、使う Unity プレイヤー (Built-in / URP / HDRP) を選び、実行ファイルのパスを標準出力に 1 行で出す。
# 選んだ理由は標準エラーに出す。見せる用の起動 (tools/race/race3.sh・race3_fuji.sh、launch の unity_player:=auto) が使う。
#
#   PLAYER=$(./scripts/pick_unity_player.sh)            # GPU から選ぶ
#   JETRACER_PIPELINE=hdrp ./scripts/pick_unity_player.sh   # 指定する (builtin | urp | hdrp | auto)
#
# 選び方 (ビデオメモリは /sys/class/drm の値と nvidia-smi。いちばん大きい GPU で決める。RenderQuality.cs の low / medium / high と同じ境目):
#   2 GB 未満・Intel の内蔵だけ・GPU が分からない → Built-in (軽い。学習・検出器の評価もこの版)
#   2 GB 以上 6 GB 未満 (例: Radeon RX 5300M 3 GB = 評価用の MSI Bravo 15) → URP
#   6 GB 以上 → HDRP
# 選んだ版のプレイヤーがまだ無ければ、1 つ軽い版へ下げる (HDRP → URP → Built-in)。
# ★URP 版・HDRP 版は配信するセンサ画像の見え方が Built-in と違う。画像で走る方策・検出器の評価・学習には Built-in を使う。
# 環境: JETRACER_UNITY_PLAYER (Built-in。既定 ~/jetracer/unity/player)、JETRACER_UNITY_PLAYER_URP (player_urp)、JETRACER_UNITY_PLAYER_HDRP (player_hdrp)
B="${JETRACER_UNITY_PLAYER:-$HOME/jetracer/unity/player}/MinicarSim.x86_64"
U="${JETRACER_UNITY_PLAYER_URP:-$HOME/jetracer/unity/player_urp}/MinicarSim.x86_64"
H="${JETRACER_UNITY_PLAYER_HDRP:-$HOME/jetracer/unity/player_hdrp}/MinicarSim.x86_64"
want="${JETRACER_PIPELINE:-auto}"
why="JETRACER_PIPELINE=$want"
if [ "$want" = "auto" ]; then
  best=0; name="不明"
  for d in /sys/class/drm/card[0-9]*/device; do
    [ -r "$d/vendor" ] || continue
    v=$(cat "$d/vendor"); mb=0
    case "$v" in
      0x1002) [ -r "$d/mem_info_vram_total" ] && mb=$(( $(cat "$d/mem_info_vram_total") / 1048576 )); n="AMD" ;;
      0x10de) n="NVIDIA"; mb=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | sort -n | tail -1); mb=${mb:-0} ;;
      0x8086) n="Intel (内蔵)"; mb=0 ;;
      *) n="$v" ;;
    esac
    if [ "$mb" -ge "$best" ]; then best=$mb; name=$n; fi
  done
  if [ "$best" -ge 6144 ]; then want=hdrp; elif [ "$best" -ge 2048 ]; then want=urp; else want=builtin; fi
  why="auto: $name ビデオメモリ ${best} MB"
fi
case "$want" in
  hdrp) p="$H"; [ -x "$p" ] || { why="$why (HDRP 版が無いので URP 版へ)"; want=urp; } ;;
esac
case "$want" in
  urp) p="$U"; [ -x "$p" ] || { why="$why (URP 版が無いので Built-in 版へ)"; want=builtin; } ;;
esac
case "$want" in
  builtin) p="$B" ;;
  urp|hdrp) ;;
  *) echo "★JETRACER_PIPELINE は builtin | urp | hdrp | auto: $want" >&2; exit 1 ;;
esac
echo "[pick_unity_player] $want ($why) → $p" >&2
echo "$p"
