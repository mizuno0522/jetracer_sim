// HDRP で動いているときの置き換え (スクリプト定義 MINICAR_HDRP のときだけコンパイルされる。docs/hdrp.md)。
//
//   材質: Standard → HDRP/Lit (色・テクスチャ・タイル・法線・滑らかさ・金属・切り抜き・半透明・detail を写す)。車体の塗装 (CarPaint) は
//         クリアコート付きにし、Built-in 用の 2 枚目のクリア層 (ClearCoat) は外す。
//         Unlit/Texture・Sprites/Default (矢印板・文字・雲のドーム・軌跡) は HDRP でもそのまま描ける (光の影響を受けない) ので触らない
//   光と露出: サーキット = 太陽 100,000 lux・物理的な空・露出 EV100 14.4 (今の Built-in の明るさに合わせた値)。
//             ミニカーの会場 = 今の光の強さのまま、露出 EV100 −1.9 (= log2(1/(1.2π))) で Built-in と同じ明るさになるように換算
//   霞: サーキットだけ。今の指数の霞 (密度 0.000055/m) と同じ減衰 (平均自由行程 18 km)・高さ 2.5 km で薄れる
//   段階 (RenderQuality): low = 後処理なし (HDRP 既定の AO・ブルーム・ブラーも 0)・影 150 m / medium = AO・ブルーム・影 300 m / high = 影 600 m・立体の霧・立体の雲
//   センサカメラ: 後処理 (トーンマップ・ブルーム) と AO を掛けない。★それでも Built-in と画像は変わるので、学習は Built-in のプロジェクトで行う
//
// HDRP の API は Unity 6 (HDRP 17) を前提に書いている。コンパイルエラーが出たらこのファイルだけ直せばよい。
#if MINICAR_HDRP
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Minicar
{
    public static partial class RenderCompat
    {
        const float kSunLux = 100000f;
        const float kCircuitEV = 14.4f;            // log2(kSunLux / (1.2 · π · 1.2)) ≈ 14.43 (Built-in の太陽 1.2 と同じ明るさ)
        const float kLegacyEV = -1.915f;           // log2(1 / (1.2 · π)): 光の強さを Built-in の値のまま使うときの露出
        static float Multiplier(float ev) => 1f / (1.2f * Mathf.Pow(2f, ev));

        static readonly Dictionary<Material, Material> s_Map = new Dictionary<Material, Material>();
        static readonly Dictionary<Texture, Texture2D> s_Detail = new Dictionary<Texture, Texture2D>();
        static readonly List<(Light light, float scale, float last)> s_Lights = new List<(Light, float, float)>();
        static Material s_LitTemplate;
        static GradientSky s_Gradient;
        static bool s_Circuit;
        static float s_EV;

        static partial void AfterBuildImpl(CourseBuilder course, Camera[] sensors, bool circuit)
        {
            if (!(GraphicsSettings.currentRenderPipeline is HDRenderPipelineAsset))
            {
                Debug.LogWarning("[RenderCompat] MINICAR_HDRP だが HDRP が有効でない (scripts/migrate_hdrp.sh の phase 2)");
                return;
            }
            s_Circuit = circuit;
            s_EV = circuit ? kCircuitEV : kLegacyEV;
            // 材質の変形 (切り抜き・半透明・法線・detail) ごとのシェーダはビルドに残すため phase 2 が Resources/HDVariants に置いている
            s_LitTemplate = Resources.Load<Material>("Mat_HDLit");
            if (s_LitTemplate == null)
            {
                var sh = Shader.Find("HDRP/Lit");
                if (sh == null) { Debug.LogError("[RenderCompat] HDRP/Lit が無い (Mat_HDLit を phase 2 で作る)"); return; }
                s_LitTemplate = new Material(sh);
            }

            // ---- 材質
            int n = ConvertAll();

            // ---- 光
            foreach (var l in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (!l.TryGetComponent(out HDAdditionalLightData _)) l.gameObject.AddComponent<HDAdditionalLightData>();
                if (l.type == LightType.Directional)
                {
                    SetUnit(l, "Lux");
                    float scale = circuit ? kSunLux / Mathf.Max(1e-3f, l.intensity) : 1f;
                    l.intensity *= scale;
                    s_Lights.Add((l, scale, l.intensity));
                }
                else
                {
                    // 点光源・スポット: Built-in の強さ ≒ 1 m での明るさ → 光度 [cd] として同じ値を置く (露出 kLegacyEV と組)
                    SetUnit(l, "Candela");
                    s_Lights.Add((l, 1f, l.intensity));
                }
            }

            // ---- カメラ
            foreach (var cam in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (!cam.TryGetComponent(out HDAdditionalCameraData hd)) hd = cam.gameObject.AddComponent<HDAdditionalCameraData>();
                if (cam.clearFlags == CameraClearFlags.SolidColor)
                {
                    hd.clearColorMode = HDAdditionalCameraData.ClearColorMode.Color;
                    hd.backgroundColorHDR = cam.backgroundColor.linear;            // 背景色は露出を掛けずにそのまま書かれる
                }
                else hd.clearColorMode = HDAdditionalCameraData.ClearColorMode.Sky;
                if (Array.IndexOf(sensors, cam) >= 0)
                {
                    // 配信するセンサ画像には後処理を掛けない (実機のカメラに無い)
                    hd.customRenderingSettings = true;
                    hd.renderingPathCustomFrameSettingsOverrideMask.mask[(uint)FrameSettingsField.Postprocess] = true;
                    hd.renderingPathCustomFrameSettings.SetEnabled(FrameSettingsField.Postprocess, false);
                    hd.renderingPathCustomFrameSettingsOverrideMask.mask[(uint)FrameSettingsField.SSAO] = true;     // AO は光の計算側なので別に切る
                    hd.renderingPathCustomFrameSettings.SetEnabled(FrameSettingsField.SSAO, false);
                    hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.None;
                }
                else
                {
                    hd.antialiasing = RenderQuality.Current == QualityTier.Low ? HDAdditionalCameraData.AntialiasingMode.FastApproximateAntialiasing
                                                                             : HDAdditionalCameraData.AntialiasingMode.TemporalAntialiasing;
                }
            }

            // ---- 空・霞・露出・後処理 (全体に効く Volume)
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            var env = profile.Add<VisualEnvironment>(true);
            env.skyAmbientMode.value = SkyAmbientMode.Dynamic;
            if (circuit)
            {
                env.skyType.value = (int)SkyType.PhysicallyBased;
                profile.Add<PhysicallyBasedSky>(true);
                var fog = profile.Add<Fog>(true);
                fog.enabled.value = true;
                fog.meanFreePath.value = 18000f;
                fog.baseHeight.value = 0f;
                fog.maximumHeight.value = 2500f;
                fog.maxFogDistance.value = 50000f;
                fog.enableVolumetricFog.value = RenderQuality.Current == QualityTier.High;
            }
            else
            {
                // 屋内の会場: 今の Trilight の環境光を、上・横・下の 3 色の空として置く (/π で Built-in と同じ明るさ)
                env.skyType.value = (int)SkyType.Gradient;
                s_Gradient = profile.Add<GradientSky>(true);
                ApplyAmbient();
            }
            var ex = profile.Add<Exposure>(true);
            ex.mode.value = ExposureMode.Fixed;
            ex.fixedExposure.value = s_EV;
            var tm = profile.Add<Tonemapping>(true);
            tm.mode.value = RenderQuality.Current == QualityTier.Low ? TonemappingMode.None : TonemappingMode.ACES;
            var sh2 = profile.Add<HDShadowSettings>(true);
            sh2.maxShadowDistance.value = RenderQuality.Current == QualityTier.High ? 600f : RenderQuality.Current == QualityTier.Medium ? 300f : 150f;
            // HDRP の既定の Volume は ブルーム 0.2・AO 0.5・モーションブラー 0.5 を入れているので、全部ここで上書きする
            bool low = RenderQuality.Current == QualityTier.Low;
            profile.Add<ScreenSpaceAmbientOcclusion>(true).intensity.value = low ? 0f : 0.6f;
            profile.Add<Bloom>(true).intensity.value = low ? 0f : 0.12f;
            profile.Add<MotionBlur>(true).intensity.value = 0f;
            if (circuit && RenderQuality.Current == QualityTier.High)
            {
                var vc = profile.Add<VolumetricClouds>(true);
                vc.enable.value = true;
                var dome = GameObject.Find("SkyDome");                     // 立体の雲を使うときは雲の絵のドームを消す
                if (dome != null) dome.SetActive(false);
            }
            var go = new GameObject("HDRPVolume");
            var vol = go.AddComponent<Volume>();
            vol.isGlobal = true;
            vol.priority = 10f;
            vol.sharedProfile = profile;
            Debug.Log($"[RenderCompat] HDRP: materials on {n} renderers, {s_Lights.Count} lights, EV100 {s_EV:F2}, quality {RenderQuality.Current}");
        }

        /// 場面のすべての Renderer の Standard 材質を HDRP/Lit に差し替える (あとから作られたもの = エピソードごとの床テープ・観戦者も)
        static int ConvertAll()
        {
            if (s_LitTemplate == null) return 0;
            int n = 0;
            foreach (var r in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                var src = r.sharedMaterials;
                var mats = new List<Material>(src.Length);
                bool changed = false;
                foreach (var sm in src)
                {
                    // 車のクリア層 (2 枚目) は外す。HDRP では下地の材質のクリアコートで描く
                    if (sm != null && sm.name == "ClearCoat") { changed = true; continue; }
                    var m = Convert(sm);
                    if (m != sm) changed = true;
                    mats.Add(m);
                }
                if (changed) { r.sharedMaterials = mats.ToArray(); n++; }
            }
            return n;
        }

        static partial void RefreshImpl()
        {
            ConvertAll();
            foreach (var kv in s_Map) CopyLit(kv.Key, kv.Value);
            ApplyAmbient();
            TickImpl();
        }

        /// 元のコードが毎フレーム光の強さを書き換える (③ライトかく乱・エピソードの乱択化) ので、書き換わったら換算し直す
        static partial void TickImpl()
        {
            for (int i = 0; i < s_Lights.Count; i++)
            {
                var (l, scale, last) = s_Lights[i];
                if (l == null || Mathf.Approximately(l.intensity, last)) continue;
                l.intensity *= scale;
                s_Lights[i] = (l, scale, l.intensity);
            }
        }

        static void ApplyAmbient()
        {
            if (s_Gradient == null) return;
            float k = 1f / Multiplier(s_EV);     // 空の輝度 L → 拡散 = albedo·L·倍率。Built-in の albedo·色 に合わせる
            s_Gradient.top.value = RenderSettings.ambientSkyColor.linear * k;
            s_Gradient.middle.value = RenderSettings.ambientEquatorColor.linear * k;
            s_Gradient.bottom.value = RenderSettings.ambientGroundColor.linear * k;
        }

        static void SetUnit(Light l, string unit)
        {
            // Unity 6 は Light.lightUnit、それより前は HDAdditionalLightData.lightUnit。どちらも反射で探す (型の違いでコンパイルを止めない)
            foreach (var target in new object[] { l, l.GetComponent<HDAdditionalLightData>() })
            {
                if (target == null) continue;
                var p = target.GetType().GetProperty("lightUnit");
                if (p == null || !p.CanWrite) continue;
                try { p.SetValue(target, Enum.Parse(p.PropertyType, unit)); return; }
                catch (Exception e) { Debug.LogWarning($"[RenderCompat] lightUnit {unit}: {e.Message}"); }
            }
        }

        // ------------------------------------------------------------ 材質
        static Material Convert(Material s)
        {
            if (s == null || s.shader == null) return s;
            if (s_Map.TryGetValue(s, out var done)) return done;
            if (!s.shader.name.StartsWith("Standard")) return s;           // Unlit/Sprites/SensorPost などはそのまま
            var m = new Material(s_LitTemplate) { name = s.name + "_HD" };
            CopyLit(s, m);
            s_Map[s] = m;
            return m;
        }

        static void CopyLit(Material s, Material m)
        {
            m.SetColor("_BaseColor", s.HasProperty("_Color") ? s.GetColor("_Color") : Color.white);
            var tex = s.HasProperty("_MainTex") ? s.GetTexture("_MainTex") : null;
            m.SetTexture("_BaseColorMap", tex);
            if (tex != null)
            {
                m.SetTextureScale("_BaseColorMap", s.GetTextureScale("_MainTex"));
                m.SetTextureOffset("_BaseColorMap", s.GetTextureOffset("_MainTex"));
            }
            var nrm = s.HasProperty("_BumpMap") && s.IsKeywordEnabled("_NORMALMAP") ? s.GetTexture("_BumpMap") : null;
            m.SetTexture("_NormalMap", nrm);            // 法線の詰め方 (x = A・y = G) は HDRP も同じく読める
            if (nrm != null) m.SetFloat("_NormalScale", s.GetFloat("_BumpScale"));
            m.SetFloat("_Smoothness", s.HasProperty("_Glossiness") ? s.GetFloat("_Glossiness") : 0.5f);
            m.SetFloat("_Metallic", s.HasProperty("_Metallic") ? s.GetFloat("_Metallic") : 0f);

            float mode = s.HasProperty("_Mode") ? s.GetFloat("_Mode") : 0f;
            bool cutout = Mathf.Approximately(mode, 1f), transparent = mode >= 1.5f;
            m.SetFloat("_AlphaCutoffEnable", cutout ? 1f : 0f);
            if (cutout) m.SetFloat("_AlphaCutoff", s.GetFloat("_Cutoff"));
            m.SetFloat("_SurfaceType", transparent ? 1f : 0f);
            if (transparent) m.SetFloat("_BlendMode", 0f);               // 0 = Alpha
            m.SetFloat("_DoubleSidedEnable", cutout ? 1f : 0f);           // 木の板

            if (s.IsKeywordEnabled("_DETAIL_MULX2") && s.GetTexture("_DetailAlbedoMap") != null)
            {
                m.SetTexture("_DetailMap", PackDetail(s.GetTexture("_DetailAlbedoMap"), s.GetTexture("_DetailNormalMap")));
                m.SetTextureScale("_DetailMap", s.GetTextureScale("_DetailAlbedoMap"));
                bool uv1 = s.HasProperty("_UVSec") && s.GetFloat("_UVSec") > 0.5f;
                m.SetFloat("_UVDetail", uv1 ? 1f : 0f);
                m.SetVector("_UVDetailsMappingMask", uv1 ? new Vector4(0, 1, 0, 0) : new Vector4(1, 0, 0, 0));
                m.SetFloat("_DetailAlbedoScale", 1f);
                m.SetFloat("_DetailNormalScale", s.HasProperty("_DetailNormalMapScale") ? s.GetFloat("_DetailNormalMapScale") : 1f);
                m.SetFloat("_DetailSmoothnessScale", 0f);
                m.SetFloat("_LinkDetailsWithBase", 0f);     // Built-in と同じく detail のタイルを独立に
            }
            m.SetFloat("_CoatMask", s.name == "CarPaint" ? 1f : 0f);   // 車体の塗装はクリアコート付き
            HDMaterial.ValidateMaterial(m);             // 上の値からキーワードと描画パスを決め直す
        }

        /// HDRP の detail は 1 枚に詰める: R = 明るさ (0.5 で変化なし)・G = 法線 y・B = 滑らかさ (0.5)・A = 法線 x
        static Texture2D PackDetail(Texture albedo, Texture normal)
        {
            if (s_Detail.TryGetValue(albedo, out var cached)) return cached;
            var a = albedo as Texture2D;
            var nm = normal as Texture2D;
            if (a == null || !a.isReadable) return null;
            int w = a.width, h = a.height;
            var pa = a.GetPixels32();
            Color32[] pn = nm != null && nm.isReadable && nm.width == w && nm.height == h ? nm.GetPixels32() : null;
            var o = new Color32[w * h];
            for (int i = 0; i < o.Length; i++)
                o[i] = new Color32(pa[i].r, pn != null ? pn[i].g : (byte)128, 128, pn != null ? pn[i].a : (byte)128);
            var t = new Texture2D(w, h, TextureFormat.RGBA32, true, true) { name = "DetailPacked", wrapMode = TextureWrapMode.Repeat, anisoLevel = 8 };
            t.SetPixels32(o);
            t.Apply(true);
            s_Detail[albedo] = t;
            return t;
        }
    }
}
#endif
