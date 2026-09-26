# レース進行状態 (周回・フェーズ管理)
std_msgs/Header header

uint8 MODE_IDLE      = 0
uint8 MODE_QUALIFY   = 1   # 予選ラウンド: 3周タイムアタック
uint8 MODE_FINAL     = 2   # 決勝ラウンド: 3分間の周回数 + 駐車チャレンジ
uint8 mode

uint8 PHASE_IDLE     = 0
uint8 PHASE_RACING   = 1
uint8 PHASE_PARKING  = 2   # 決勝: 3分経過後の駐車チャレンジ(1分)
uint8 PHASE_FINISHED = 3
uint8 PHASE_RECOVERY = 4   # スタック自動脱出中
uint8 phase

uint32 lap_count
float32 lap_progress          # 現在周の進捗 0.0-1.0
float32 elapsed_s             # スタートからの経過時間
float32 remaining_s           # 決勝: 残り時間 (予選は負値)
float32 total_distance_m
float32 last_lap_time_s
float32 best_lap_time_s

uint8 start_lane              # 1/2/3 → 駐車枠 P1/P2/P3 に対応
uint32 arrow_bonus_count      # 矢印指示側の狭い道を通過した回数
uint32 recovery_count         # 自動復帰(スタック脱出)を行った回数
bool parking_completed
