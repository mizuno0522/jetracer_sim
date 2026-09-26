// course.json (jetson/ros_ws/src/minicar_sim/scripts/export_unity_course.py が生成) の型。
// 数値の定義元は course.py / sim.yaml。ここに数値を書かないこと。
using System;

namespace Minicar
{
    [Serializable]
    public class WallData
    {
        public float x0, y0, x1, y1;
        public string color;     // "red" / "white"
        public bool divider;     // ⑤狭い道の中央仕切り (予選では無い)
    }

    [Serializable]
    public class AreaData
    {
        public string name;      // MU_HIGH / MU_LOW / ROUGH / SLOPE / TUNNEL / LIGHT
        public float x0, y0, x1, y1;
    }

    [Serializable]
    public class SlotData
    {
        public string name, color;
        public float x0, y0, x1, y1;
    }

    [Serializable]
    public class ArrowSignData
    {
        public float x, post_y0, post_y1, board_w, board_h, board_z0, post_h, post_r;
    }

    [Serializable]
    public class LightData
    {
        public float x, y, z, bar_ew, bar_ns, bar_z, leg_r;
    }

    [Serializable]
    public class CameraData
    {
        public int width, height;
        public float fov_deg, mount_height_m, pitch_deg, crop_top_frac, rate_hz;
        // カメラ幾何 (OpenCV plumb_bob。定義元は vehicle_profile.camera、式は jetracer_common/cam_geom.py)。
        // fx <= 0 (古い course.json) なら fov_deg の正方ピンホールで描く
        public float fx, fy, cx, cy, k1, k2;
        // ピンホールで描く正規化座標の範囲 (x = X/Z 右、y = Y/Z 下)。歪みの逆写像がこの範囲を参照する
        public float render_tan_x0, render_tan_x1, render_tan_y0, render_tan_y1;
        public float true_hfov_deg, true_vfov_deg;
        public bool HasIntrinsics => fx > 0f && fy > 0f && render_tan_x1 > render_tan_x0 && render_tan_y1 > render_tan_y0;
    }

    /// <summary>実カメラの見た目に寄せる後処理と会場の演出。定義元は vehicle_profile.camera.realism。
    /// 古い course.json (項目なし) は null → 既定値 (後処理オフ) で読む。</summary>
    [Serializable]
    public class RealismData
    {
        public bool enable = false;
        public int supersample = 2;              // センサカメラをこの倍率で描いてから縮小 (ぼけと AA)
        // (旧) k1/k2/zoom は使わない。歪みは camera の fx, fy, cx, cy, k1, k2 から描く
        public float k1 = 0f, k2 = 0f, zoom = 1f;
        public float vignette = 0.15f;
        public float blur_px = 1.0f;             // 描画解像度の画素
        public float motion_px_per_rad_s = 2.0f; // ヨーレート比例の横ブラー
        public float exposure_target = 110f;     // 自動露出の目標平均輝度 [0-255]
        public float exposure_tau_s = 0.4f;
        public float gamma = 1.0f;
        public float noise = 0.012f;
        public float post_dark = 0.15f;
        public float[] posts = { 0.24f, 0.26f, 0.93f, 0.54f, 0.56f, 0.93f };   // 柱 (x0, x1, y0) × n、正規化
        public string carpet_tex = "textures/carpet.png";
        public string backdrop_tex = "textures/backdrop.png";
        public float backdrop_radius_m = 9f, backdrop_height_m = 4f, backdrop_z0_m = -0.3f;
        public float background_gray = 110f;
        // 環境光の倍率。影側を向いた壁の板 (白・赤) も明るく見えるように (実画像の白壁 ≈ 195、床 ≈ 110)
        public float ambient_gain = 1f;     // 背景 (背景円筒より上・外) の明るさ [0-255]
        public int spectators = 24;
        public int spectator_seed = 1;
        // エピソード乱択化 (/sim/episode の seed で引き直す)。0 で固定
        public float episode_light_range = 0.15f;   // 天井光の強さ ×(1 ± r)
        public float episode_tint_range = 0.04f;    // 床・照明の色味 ± r (RGB 各)
        public float episode_ambient_range = 0.15f; // 環境光 ×(1 ± r)
        public bool episode_spectators = true;      // 観戦者の配置も引き直す
        public float wall_white_r = 193f, wall_white_g = 192f, wall_white_b = 191f;
        public float wall_red_r = 141f, wall_red_g = 70f, wall_red_b = 67f;
        public float carpet_r = 117f, carpet_g = 115f, carpet_b = 112f;
    }

    [Serializable]
    public class VizData
    {
        public float speed_color_max_mps;
        public int trail_max_points;
        public float trail_min_dist_m;
        public float legend_x, legend_y0, legend_y1;
    }

    [Serializable]
    public class CourseData
    {
        public WallData[] walls;
        public float wall_height_m, wall_thickness_m;
        /// <summary>板の下端の床からの高さ。規約 p.34: 板は床から 30 mm 浮かせ、上端 120 mm。
        /// 古い course.json (項目なし) は 0 として読まれる。</summary>
        public float wall_base_m;
        public AreaData[] areas;
        public SlotData[] parking_slots;
        public float parking_tape_m;
        public ArrowSignData arrow_sign;
        public LightData light;
        public float tunnel_height_m;
        public CameraData camera;
        public RealismData realism;
        public VizData viz;
        // コース中心線 (閉ループ, [x0, y0, x1, y1, ...])。ミニマップと周回・セクタ表示用
        public float[] centerline_shortcut, centerline_long;
        // 参照線 (make_route.py の route.yaml、[x0, y0, x1, y1, ...])。ミニマップに描くだけ。無ければ空
        public float[] reference_line;
        public string reference_line_name;
    }
}
