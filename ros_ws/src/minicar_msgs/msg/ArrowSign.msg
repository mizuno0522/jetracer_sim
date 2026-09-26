# 矢印信号(LED電光掲示板 93cm x 19cm)の認識結果
# 決勝ラウンドでは指示された側の狭い道を通過すると +0.5周/回
std_msgs/Header header

uint8 DIR_UNKNOWN = 0
uint8 DIR_LEFT    = 1
uint8 DIR_RIGHT   = 2

uint8 direction
bool detected
float32 confidence          # 0.0-1.0
float32 distance_m          # 標識までの推定距離
float32 bearing_rad         # 車両正面からの方位 (+左)

# 直近 N フレームの多数決後の安定判定結果
uint8 stable_direction
uint32 stable_count
