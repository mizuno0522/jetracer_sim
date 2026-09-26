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
    }

    /// <summary>実カメラの見た目に寄せる後処理と会場の演出。定義元は vehicle_profile.camera.realism。
    /// 古い course.json (項目なし) は null → 既定値 (後処理オフ) で読む。</summary>
    [Serializable]
    public class RealismData
    {
        public bool enable = false;
        public int supersample = 2;              // センサカメラをこの倍率で描いてから縮小 (ぼけと AA)
        public float k1 = -0.08f, k2 = 0f, zoom = 1f;   // Brown 樽型歪み (正規化半径)
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
        public float background_gray = 110f;     // 背景 (背景円筒より上・外) の明るさ [0-255]
        public int spectators = 24;
        public int spectator_seed = 1;
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
    }
}
