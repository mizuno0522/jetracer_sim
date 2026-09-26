# Arduino(Nano V3) から取得した車両ハードウェア状態
std_msgs/Header header

bool armed                # ESC がアーム済みか
bool failsafe             # 指令途絶によるフェイルセーフ発動中
bool tof_left_ok
bool tof_right_ok
bool sonar_ok
bool rpm_ok
# 追加の測距センサ (前方45度x2 / 後方超音波)
# FW v3.1 で後方45度 ToF x2 を廃止した。STATUS の bit9/bit10 は欠番のまま。
bool tof_fl_ok
bool tof_fr_ok
bool sonar_rear_ok

uint16 steer_pwm_us
uint16 throttle_pwm_us

float32 wheel_speed_mps   # RPMセンサから算出した車輪速
float32 wheel_distance_m  # 直近周期の走行距離増分

uint32 frame_errors
uint32 checksum_errors
float32 link_hz           # Arduino からのフレーム受信レート
