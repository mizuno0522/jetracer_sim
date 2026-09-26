# BehaviorTree / 各コントローラ → cmd_arbiter への走行指令
std_msgs/Header header

# 指令元の識別名 ("lane_follow" / "wall_follow" / "parking" / "recovery" / "teleop")
string source

# 優先度 (大きいほど優先。同時発行時は最大優先度が採用される)
uint8 priority

float32 speed_mps         # 目標速度 [m/s] (負値=後退)
float32 steer_norm        # 正規化舵角 -1.0(右) 〜 +1.0(左)

# 速度指令のかわりに正規化スロットルを直接使う場合
bool use_raw_throttle
float32 throttle_norm     # -1.0 〜 +1.0

bool brake                # アクティブブレーキ要求
bool disarm               # ESC を中立ロック (駐車完了時の自律停止)

float32 timeout_s         # この指令の有効期限。0以下ならデフォルト値
