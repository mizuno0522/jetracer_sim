# /sim/ground_truth ── 学習ラベル。記録のみで購読者を作らない (推論スタックは知らない)。
std_msgs/Header header
float64 sim_time
uint32 step_id
# 姿勢の真値 (odom 座標、m / rad)
float64 x
float64 y
float64 yaw
float32 v                 # 真の車速 [m/s] (実機では取れない量)
float32 vy                # 横滑り [m/s]
float32 yaw_rate
float32 steer_rad         # 実現された舵角
# 参照線に対する量
float32 s_m               # 参照線上の弧長 [m]
float32 progress          # s / 全長 [0,1)
float32 cte_m             # 横偏差 [m] 左正
float32 heading_err_rad   # 方位誤差 [rad] 左正
float32 curvature         # 先行注視点付近の参照線の曲率 [1/m] 左正
uint8 zone                # 区間 ID (BodyState.SURFACE_* と同じ並び)
uint32 lap
# 先行注視点 (参照線上、lookahead_m 先) の画像座標。画像はクロップ後の /camera/image_raw と同じ座標系
float32 lookahead_m
float32 la_x              # 先行注視点 (odom)
float32 la_y
float32 u                 # [px]
float32 v_px              # [px]
bool la_visible           # 画像内に入っているか
float32 min_wall_clear_m
bool off_track
bool collision
