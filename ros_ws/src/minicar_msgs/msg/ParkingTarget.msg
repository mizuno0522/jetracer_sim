# 駐車枠(P1緑 / P2赤 / P3青, 幅50cm x 長さ124cm, テープ幅5cm)の検出結果
std_msgs/Header header

uint8 SLOT_NONE  = 0
uint8 SLOT_P1    = 1   # 緑テープ
uint8 SLOT_P2    = 2   # 赤テープ
uint8 SLOT_P3    = 3   # 青テープ

uint8 target_slot        # 狙う枠 (スタートレーンと同番号)
uint8 detected_slot      # 実際に見えている枠
bool detected

# base_link 基準の枠中心位置と向き
float32 x_m
float32 y_m
float32 yaw_rad

float32 confidence
bool inside_slot         # 車体が枠内(線上含む)に収まっているか
float32 alignment_error_m
