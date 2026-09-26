# 他車（決勝ラウンドの同時走行相手）の状態
# opponent_detector が魚眼カメラ主軸 + 測距の裏取りで推定する。
# 予選ラウンド(1台走行)では detected は常に false。
std_msgs/Header header

bool detected
float32 confidence            # 0.0-1.0

# コリドー内での相対位置
uint8 RELPOS_AHEAD = 0        # 正面前方
uint8 RELPOS_LEFT  = 1        # 左寄り
uint8 RELPOS_RIGHT = 2        # 右寄り
uint8 rel_position

# base_link 基準
float32 distance_m            # 他車までの前方距離 (測距で確定)
float32 bearing_rad           # 車両正面からの方位 (+左)
float32 lateral_offset_m      # コリドー中心から見た他車の横位置 (+左)

# 接近速度 [m/s]。+ で接近。自車速 - 他車の対地速度推定。
# 静止した壁は closing ≒ 自車速、追走中の他車は closing ≒ 0 になる。
float32 closing_speed_mps
float32 rel_speed_mps         # 他車の推定対地速度 (自車速 - closing)

# 追い越し可否の判断材料。他車の左右にどれだけ空きがあるか。
float32 gap_left_m
float32 gap_right_m
float32 opponent_width_m      # 他車の見かけの横幅

# 直近フレームの多数決後の安定検出フラグ (1フレームの誤検出で動かないため)
bool stable_detected
uint32 stable_count
