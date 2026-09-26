# policy_net (または gt_teacher) → cmd_shaper。「先行注視点」の画像座標と速度係数。
# 操舵角そのものを出さない (車両諸元は cmd_shaper の yaml に集める)。
std_msgs/Header header    # stamp は元になった画像の stamp

float32 u                 # 画像座標 [px] (クロップ後の画像。左が 0)
float32 v                 # 画像座標 [px] (上が 0)
float32 s                 # 速度係数 [0,1]。v_cmd = s × v_max
bool valid                # false なら cmd_shaper は減速停止する
