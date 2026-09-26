# コース上のギミック区間推定
# レギュレーション「コース概要」の①〜⑧に対応
std_msgs/Header header

uint8 ZONE_UNKNOWN        = 0
uint8 ZONE_NORMAL         = 1
uint8 ZONE_TUNNEL         = 2   # ①トンネル(漆黒シート・上部OPEN)
uint8 ZONE_SLOPE          = 3   # ②坂道ショートカット
uint8 ZONE_LIGHT_DISTURB  = 4   # ③ライトかく乱ゾーン
uint8 ZONE_MU_CHANGE      = 5   # ④低μ/高μ路
uint8 ZONE_NARROW         = 6   # ⑤狭い道
uint8 ZONE_ARROW_GATE     = 7   # ⑥矢印信号の手前(進路選択区間)
uint8 ZONE_ROUGH          = 8   # ⑦でこぼこ道
uint8 ZONE_PARKING        = 9   # ⑧ゴール後駐車エリア

uint8 zone
uint8 previous_zone
float32 confidence
float32 distance_in_zone_m      # 区間に入ってからの走行距離
float32 distance_to_next_m      # 次の区間までの推定距離 (負値=不明)

# 区間判定の根拠 (デバッグ用)
bool cue_dark                   # 画面が暗い→トンネル
bool cue_color_unstable         # 色相が不安定→ライトかく乱
bool cue_narrow                 # 左右ToFが近い→狭い道
bool cue_pitch                  # ピッチ変化→坂道
bool cue_vibration              # 振動→でこぼこ道
bool cue_odom                   # オドメトリのコース位置一致
