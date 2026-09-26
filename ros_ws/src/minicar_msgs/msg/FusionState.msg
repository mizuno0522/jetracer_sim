# センサフュージョンが出す統合状態。
# 走行制御と BT が「いま何を信用してよいか」を 1 つの窓口で判断できるようにする。
std_msgs/Header header

# --- 統合された自己状態 ---
float32 yaw_rad             # 融合ヨー角 (ジャイロ積分 + 地磁気補正)
float32 yaw_rate_radps      # バイアス補正後のヨーレート
float32 gyro_bias_radps     # 推定したジャイロバイアス
float32 speed_mps           # 融合速度 (車輪速 + 対地速度)
float32 slip_ratio          # 1 - 対地速度/車輪速 (低μ路④の指標)
float32 yaw_sigma_rad       # ヨー角の推定標準偏差 (大きいほど信用できない)

# --- 前方の余裕 (測距の統合結果) ---
float32 free_ahead_m        # 進行方向の最短障害物距離
float32 clearance_left_m
float32 clearance_right_m
float32 corridor_width_m

# --- 各センサの健全性 (受信が新しく、値が妥当か) ---
bool ok_wheel
bool ok_imu
bool ok_mag                 # 地磁気が乱れていないか (乱れたら false → 不採用)
bool ok_tof
bool ok_sonar
bool ok_lidar
bool ok_camera

uint8 range_sources         # 測距に寄与したセンサ数 (0 なら前方情報なし)

# --- データ鮮度 (QoS) ---
float32 worst_age_s         # 採用中の測距データのうち最も古いものの経過時間
float32 data_health         # 0.0-1.0。測距の本数と鮮度から出す総合指標。
                            # 下流はこれを速度に掛けて、情報が薄いときは
                            # 自動的に慎重に走る (見えていないのに飛ばさない)。
uint32 n_dropped            # タイムスタンプが古くて捨てたメッセージ数 (累積)
