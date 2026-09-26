# カメラ(IMX219 魚眼)から推定した走行可能コリドー情報
std_msgs/Header header

# 検出ソース
uint8 SOURCE_NONE   = 0
uint8 SOURCE_COLOR  = 1   # 赤/白ウォールの色分割 (通常路)
uint8 SOURCE_EDGE   = 2   # 輝度エッジ (ライトかく乱/低コントラスト時)
uint8 SOURCE_RANGE  = 3   # ToF/超音波のみ (トンネル内)
uint8 source

bool valid                # 推定が有効か
float32 confidence        # 0.0-1.0

# 車両基準(base_link, x前方 / y左方)のコリドー中心線
# 前方 lookahead_m 地点での横偏差と方位偏差
float32 cross_track_error_m   # +左ずれ / -右ずれ
float32 heading_error_rad     # +左向き / -右向き
float32 curvature_1pm         # 中心線の曲率 [1/m] (+左曲がり)
float32 lookahead_m

# コリドー幅と左右余裕
float32 corridor_width_m
float32 left_margin_m
float32 right_margin_m

# 中心線サンプル点 (base_link座標系, 近い順)
geometry_msgs/Point[] centerline

# 前方の見通し距離 [m] (この先で壁に当たるまで)
float32 free_distance_m
