// Built-in の描画で動いているとき (スクリプト定義なし = unity/MinicarSim)。いちばん軽い版。docs/hdrp.md
// URP 版・HDRP 版と同じ仕様の絵を出す (数値は RenderCompat.GetLook・CircuitSun など 3 版で共通):
//   材質: Standard のまま
//   光  : 色空間は Linear (MinicarBuild)。富士の太陽・環境光は HDRP の絵に合わせた強さに置き直す
//   映り込み: ミニカーの会場は環境光と同じ 3 色の空を映す (HDRP の空と同じ)。既定のスカイボックス (屋外の青空) は映さない
//   後処理 (medium 以上・表示用のカメラだけ): ViewPost (際の陰り・ぼけ・にじみ・周辺減光・コントラスト・ACES)
//   センサカメラ: 後処理を掛けない
#if !MINICAR_HDRP && !MINICAR_URP
using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace Minicar
{
    public static partial class RenderCompat
    {
        static bool s_Circuit;

        static partial void AfterBuildImpl(CourseBuilder course, Camera[] sensors, bool circuit)
        {
            s_Circuit = circuit;
            var look = GetLook(circuit);
            QualitySettings.shadowDistance = look.shadowDistance;
            if (!circuit) ApplyRoomReflection();
            int n = 0;
            foreach (var cam in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (Array.IndexOf(sensors, cam) >= 0) continue;
                var k = look;
                if (cam.orthographic) k.dof = false;       // 真上からの全景 (RViz 風) は図として読むものなので、ぼかさない
                if (cam.orthographic) k.ao = 0f;           // 際の陰りは透視のカメラの深度から作る
                ViewPost.Attach(cam, k);
                if (k.post) n++;
            }
            Debug.Log($"[RenderCompat] Built-in: {QualitySettings.activeColorSpace}, post on {n} cameras, shadow {look.shadowDistance:F0} m, quality {RenderQuality.Current}");
        }

        static partial void RefreshImpl() { if (!s_Circuit) ApplyRoomReflection(); }
    }
}
#endif
