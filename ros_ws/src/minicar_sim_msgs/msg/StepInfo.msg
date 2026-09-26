# 強化学習の報酬の「材料」。報酬そのものは入れない (学習器側で組む)。
std_msgs/Header header
uint32 step_id
float32 progress_m        # 参照線上の弧長の前進 (この step ぶん)
float32 cte_m             # 横偏差 (符号つき)
float32 heading_err_rad
float32 speed_mps         # 真の車速
float32 min_wall_clear_m  # 壁までの最小余裕
float32 steer_rate_rad_s  # 舵の変化率 (滑らかさの罰に)
uint8 zone                # 区間 ID
bool collision
bool off_track
bool lap_done
bool terminated           # 終端: 衝突・コース外
bool truncated            # 打ち切り: 時間切れ (終端と区別する)
