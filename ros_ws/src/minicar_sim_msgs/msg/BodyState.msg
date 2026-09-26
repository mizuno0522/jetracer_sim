# vehicle_sim → imu_sim。物理 100 Hz の剛体状態 (指令ではなく「実際に起きたこと」)。
# 座標は base_link (x 前・y 左・z 上)。加速度は運動学的な値 (重力を含まない)。
# 比力 (加速度計の読み) は imu_sim が roll/pitch から重力を投影して作る。
std_msgs/Header header
float64 sim_time          # [s] sim 時計 (lockstep ではこれが唯一の時間軸)
uint32 step_id            # lockstep の step 番号 (realtime では 0)
float32 ax                # [m/s^2] 重心の前後加速度 (運動学)
float32 ay                # [m/s^2] 重心の横加速度 (運動学、左正)
float32 az                # [m/s^2] 重心の上下加速度 (運動学、上正。突き上げ)
float32 wx                # [rad/s] ロール角速度
float32 wy                # [rad/s] ピッチ角速度
float32 wz                # [rad/s] ヨー角速度
float32 roll              # [rad] 右が下がると正 (x 軸まわり右手系)
float32 pitch             # [rad] 鼻上げが正 ... ROS 右手系では鼻下げ正なので注意: ここは「鼻上げ正」
float32 v                 # [m/s] 前後車速
float32 vy                # [m/s] 横滑り速度 (左正)
float32 steer_rad         # [rad] 実現された舵角 (サーボ模型の出力)
uint8 surface             # 路面 ID (SURFACE_*)
float32 rough_frac        # でこぼこに乗っている車輪の割合 [0,1]
float32 slip_ratio        # 駆動輪スリップ率
bool stuck
uint8 SURFACE_CARPET=0
uint8 SURFACE_TUNNEL=1
uint8 SURFACE_LIGHT=2
uint8 SURFACE_TURF=3
uint8 SURFACE_SLIP=4
uint8 SURFACE_ROUGH=5
uint8 SURFACE_SLOPE=6
