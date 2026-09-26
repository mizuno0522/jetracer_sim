# /actuator_cmd ── sim と実機で同一。この定義が契約そのもの (設計 P4)。
# 型・単位・周期 (30 Hz)・QoS (reliable, depth 1) を凍結する。
std_msgs/Header header    # stamp は指令を作った時刻

float32 steer_rad         # 操舵角 [rad]。左が正。±delta_max_rad で飽和
float32 speed_mps         # 目標車速 [m/s]。前進が正

uint8 mode                # 0=IDLE 1=RUN 2=ESTOP
uint8 MODE_IDLE=0
uint8 MODE_RUN=1
uint8 MODE_ESTOP=2
