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
        static Cubemap s_Env;
        static bool s_Circuit;

        static partial void AfterBuildImpl(CourseBuilder course, Camera[] sensors, bool circuit)
        {
            s_Circuit = circuit;
            var look = GetLook(circuit, course.Room);
            QualitySettings.shadowDistance = look.shadowDistance;
            ApplyAmbient();
            int n = 0;
            foreach (var cam in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (Array.IndexOf(sensors, cam) >= 0) continue;
                var k = look;
                if (cam.orthographic) k.dof = false;       // 真上からの全景 (RViz 風) は図として読むものなので、ぼかさない
                ViewPost.Attach(cam, k);
                if (k.post) n++;
            }
            Debug.Log($"[RenderCompat] Built-in: {QualitySettings.activeColorSpace}, post on {n} cameras, shadow {look.shadowDistance:F0} m, quality {RenderQuality.Current}");
        }

        static partial void RefreshImpl() { ApplyAmbient(); }

        /// ミニカーの会場: 映り込みに環境光と同じ 3 色 (上・横・下) の空を使う
        static void ApplyAmbient()
        {
            if (s_Circuit) return;
            if (s_Env == null) s_Env = new Cubemap(16, TextureFormat.RGBAHalf, true) { name = "RoomEnv" };
            Color top = RenderSettings.ambientSkyColor.linear, mid = RenderSettings.ambientEquatorColor.linear, bot = RenderSettings.ambientGroundColor.linear;
            var px = new Color[16 * 16];
            for (int f = 0; f < 6; f++)
            {
                for (int y = 0; y < 16; y++)
                    for (int x = 0; x < 16; x++)
                    {
                        float u = (x + 0.5f) / 8f - 1f, v = (y + 0.5f) / 8f - 1f;
                        Vector3 d;
                        switch ((CubemapFace)f)
                        {
                            case CubemapFace.PositiveX: d = new Vector3(1, -v, -u); break;
                            case CubemapFace.NegativeX: d = new Vector3(-1, -v, u); break;
                            case CubemapFace.PositiveY: d = new Vector3(u, 1, v); break;
                            case CubemapFace.NegativeY: d = new Vector3(u, -1, -v); break;
                            case CubemapFace.PositiveZ: d = new Vector3(u, -v, 1); break;
                            default: d = new Vector3(-u, -v, -1); break;
                        }
                        float s = d.normalized.y;
                        px[y * 16 + x] = s >= 0f ? Color.Lerp(mid, top, s) : Color.Lerp(mid, bot, -s);
                    }
                s_Env.SetPixels(px, (CubemapFace)f);
            }
            s_Env.Apply(true);
            RenderSettings.defaultReflectionMode = DefaultReflectionMode.Custom;
            RenderSettings.customReflectionTexture = s_Env;
        }
    }
}
#endif
