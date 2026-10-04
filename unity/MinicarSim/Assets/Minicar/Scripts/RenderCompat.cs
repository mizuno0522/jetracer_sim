// 描画の仕組み (Built-in / HDRP) の違いを吸収する。コースと車は今まで通り Standard シェーダの材質で組み、
// HDRP で動いているときだけ組み上がったあとで HDRP の材質・光・空・霞・露出に置き換える (RenderCompat.Hdrp.cs)。
//
// HDRP 版は別の Unity プロジェクト unity/MinicarSimHDRP (scripts/migrate_hdrp.sh が unity/MinicarSim から作る) で、
// スクリプト定義 MINICAR_HDRP が付いたときだけ RenderCompat.Hdrp.cs が有効になる。
// Built-in のプロジェクト (学習・ミニカーの会場) では、ここの関数はすべて何もしない。
using UnityEngine;

namespace Minicar
{
    public static partial class RenderCompat
    {
        public static string PipelineName
        {
            get
            {
                var rp = UnityEngine.Rendering.GraphicsSettings.currentRenderPipeline;
                return rp == null ? "Built-in" : rp.GetType().Name;
            }
        }

        /// コース・車・カメラを作り終えたあとに 1 回。sensors = 配信用のセンサカメラ (後処理を掛けない)
        public static void AfterBuild(CourseBuilder course, Camera[] sensors, bool circuit) { AfterBuildImpl(course, sensors, circuit); }

        /// 置き換えたあとに元の材質・光・環境光を変えたとき (エピソードの乱択化) に呼ぶ
        public static void Refresh() { RefreshImpl(); }

        /// 毎フレーム (元のコードが光の強さを書き換えたら HDRP の単位に換算し直す)
        public static void Tick() { TickImpl(); }

        static partial void TickImpl();
        static partial void AfterBuildImpl(CourseBuilder course, Camera[] sensors, bool circuit);
        static partial void RefreshImpl();
    }
}
