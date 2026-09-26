# 路面状態推定 (低μ(PTFEシート) / 高μ(人工芝) / でこぼこ道 / 坂道)
std_msgs/Header header

uint8 SURF_NORMAL = 0     # パンチカーペット
uint8 SURF_LOW_MU = 1     # PTFEシート(スリップ板)
uint8 SURF_HIGH_MU = 2    # 人工芝
uint8 SURF_ROUGH = 3      # でこぼこ道(滑り止めマット)
uint8 surface

float32 slip_ratio        # (車輪速 - 対地速度推定) / 車輪速   0=グリップ
float32 mu_estimate       # 推定摩擦係数 (相対値 0.0-1.0)
float32 vibration_rms     # IMU z軸加速度の高周波RMS [m/s^2]
float32 pitch_rad         # 車体ピッチ角 (+登り)
bool on_slope             # 坂道ショートカット上にいるか
bool wheel_slip           # スリップ検出フラグ
float32 confidence
