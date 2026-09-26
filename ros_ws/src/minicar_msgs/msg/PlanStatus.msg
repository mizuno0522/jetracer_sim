# ローカルパスプランナの出力状態。BT がこれを見て path_follow を選ぶ。
std_msgs/Header header

bool valid                  # 使える経路が見つかったか
float32 curvature_1pm       # 選んだ経路の曲率 [1/m] (+左曲がり)
float32 target_speed_mps    # 経路の余裕と曲率から決めた推奨速度
float32 cost                # 採用経路のコスト (小さいほど良い)
float32 clearance_m         # 経路上の最小クリアランス
float32 length_m            # 経路長 (前方に伸ばせた距離)
float32 lateral_offset_m    # コリドー中心からの目標横位置 (+左)
uint8 n_candidates          # 生成した候補数
uint8 n_feasible            # 衝突しなかった候補数
bool blocked                # 全候補が塞がれた (減速/停止して回避へ)
string reason               # 無効時の理由 (ログ用)
