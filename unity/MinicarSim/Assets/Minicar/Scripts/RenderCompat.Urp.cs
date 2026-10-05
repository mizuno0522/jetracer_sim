// URP で動いているときの置き換え (スクリプト定義 MINICAR_URP のときだけコンパイルされる。docs/hdrp.md)。
// Built-in (軽い・学習用) と HDRP (重い・きれい) の間の段。scripts/pick_unity_player.sh が GPU を見て 3 つから選ぶ。
//
//   材質: Standard → Universal Render Pipeline/Lit (色・テクスチャ・タイル・法線・滑らかさ・金属・切り抜き・半透明・detail を写す)。
//         Unlit/Texture・Sprites/Default (矢印板・文字・雲のドーム・軌跡) は URP でもそのまま描けるので触らない
//   光  : URP 版は Linear 色空間。Built-in (Gamma) と同じ明るさに見えるよう、光の強さを 2.2 乗して入れる
//         (強さ 0.55 の光は Gamma では 0.55 倍に見えるが、Linear でそのまま使うと約 0.76 倍に見える)。環境光は色なので自動で換算される
//   影  : URP は QualitySettings ではなく設定アセットの値を使うので、コードが決めた影の距離・分割・アンチエイリアスを写す
//   後処理 (medium 以上・表示用のカメラだけ): トーンマップ・ブルーム。部屋の会場 (-venue room) は遠くのぼけ・周辺減光も
//   センサカメラ: 後処理を掛けない。★それでも Built-in と画像は変わるので、学習は Built-in のプロジェクトで行う
//
// URP の API は Unity 6 (URP 17) を前提に書いている。コンパイルエラーが出たらこのファイルだけ直せばよい。
#if MINICAR_URP
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace Minicar
{
    public static partial class RenderCompat
    {
        const float kGamma = 2.2f;

        static readonly Dictionary<Material, Material> s_Map = new Dictionary<Material, Material>();
        static readonly List<(Light light, float last)> s_Lights = new List<(Light, float)>();
        static Material s_LitTemplate;

        static partial void AfterBuildImpl(CourseBuilder course, Camera[] sensors, bool circuit)
        {
            var urp = GraphicsSettings.currentRenderPipeline as UniversalRenderPipelineAsset;
            if (urp == null)
            {
                Debug.LogWarning("[RenderCompat] MINICAR_URP だが URP が有効でない (scripts/migrate_urp.sh の phase 2)");
                return;
            }
            // 材質の変形 (切り抜き・半透明・法線・detail・金属感の絵) ごとのシェーダはビルドに残すため phase 2 が Resources/URPVariants に置いている
            s_LitTemplate = Resources.Load<Material>("Mat_URPLit");
            if (s_LitTemplate == null)
            {
                var sh = Shader.Find("Universal Render Pipeline/Lit");
                if (sh == null) { Debug.LogError("[RenderCompat] Universal Render Pipeline/Lit が無い (Mat_URPLit を phase 2 で作る)"); return; }
                s_LitTemplate = new Material(sh);
            }
            bool low = RenderQuality.Current == QualityTier.Low;

            // ---- 材質
            int n = ConvertAll();

            // ---- 光 (Gamma での見え方に合わせる)
            foreach (var l in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                l.intensity = Mathf.Pow(Mathf.Max(0f, l.intensity), kGamma);
                s_Lights.Add((l, l.intensity));
            }

            // ---- 影・アンチエイリアス: コース側と RenderQuality.ApplyBuiltin が QualitySettings に入れた値を設定アセットへ写す
            urp.shadowDistance = QualitySettings.shadowDistance;
            urp.shadowCascadeCount = Mathf.Clamp(QualitySettings.shadowCascades, 1, 4);
            urp.msaaSampleCount = Mathf.Max(1, QualitySettings.antiAliasing);

            // ---- カメラ
            foreach (var cam in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                var d = cam.GetUniversalAdditionalCameraData();
                bool sensor = Array.IndexOf(sensors, cam) >= 0;
                // 配信するセンサ画像には後処理を掛けない (実機のカメラに無い)。真上からの全景 (RViz 風) は図として読むものなので掛けない
                d.renderPostProcessing = !sensor && !low && !cam.orthographic;
                d.antialiasing = AntialiasingMode.None;
                d.renderShadows = true;
            }

            // ---- 後処理 (全体に効く Volume)
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            var tm = profile.Add<Tonemapping>(true);
            tm.mode.value = low ? TonemappingMode.None : TonemappingMode.Neutral;
            var bloom = profile.Add<Bloom>(true);
            bloom.intensity.value = low ? 0f : 0.15f;
            bloom.threshold.value = 1.0f;
            if (!circuit && course.Room && !low)
            {
                // 部屋の会場 (-venue room): 小さな車を近くから撮った写真らしく (HDRP 版と同じねらい。映り込みと際の陰りは URP 版には無い)
                bloom.intensity.value = 0.25f;
                var dof = profile.Add<DepthOfField>(true);
                dof.mode.value = DepthOfFieldMode.Gaussian;
                dof.gaussianStart.value = 4.5f;
                dof.gaussianEnd.value = 22f;
                dof.gaussianMaxRadius.value = 1.0f;
                var vg = profile.Add<Vignette>(true);
                vg.intensity.value = 0.22f; vg.smoothness.value = 0.45f;
                var ca = profile.Add<ColorAdjustments>(true);
                ca.contrast.value = 10f; ca.saturation.value = 6f;
            }
            var go = new GameObject("URPVolume");
            var vol = go.AddComponent<Volume>();
            vol.isGlobal = true;
            vol.priority = 10f;
            vol.sharedProfile = profile;
            Debug.Log($"[RenderCompat] URP: materials on {n} renderers, {s_Lights.Count} lights, shadow {urp.shadowDistance:F0} m, quality {RenderQuality.Current}");
        }

        /// 場面のすべての Renderer の Standard 材質を URP/Lit に差し替える (あとから作られたもの = エピソードごとの床テープ・観戦者も)
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
                    // 車のクリア層 (2 枚目。Built-in では乗算済みアルファで反射だけを重ねる) は外す。URP/Lit に写すと車体を白く塗りつぶした
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
            TickImpl();
        }

        /// 元のコードが毎フレーム光の強さを書き換える (③ライトかく乱・エピソードの乱択化) ので、書き換わったら換算し直す
        static partial void TickImpl()
        {
            for (int i = 0; i < s_Lights.Count; i++)
            {
                var (l, last) = s_Lights[i];
                if (l == null || Mathf.Approximately(l.intensity, last)) continue;
                l.intensity = Mathf.Pow(Mathf.Max(0f, l.intensity), kGamma);
                s_Lights[i] = (l, l.intensity);
            }
        }

        // ------------------------------------------------------------ 材質
        static Material Convert(Material s)
        {
            if (s == null || s.shader == null) return s;
            if (s_Map.TryGetValue(s, out var done)) return done;
            if (!s.shader.name.StartsWith("Standard")) return s;           // Unlit/Sprites/SensorPost などはそのまま
            var m = new Material(s_LitTemplate) { name = s.name + "_URP" };
            CopyLit(s, m);
            s_Map[s] = m;
            return m;
        }

        static void Keyword(Material m, string kw, bool on) { if (on) m.EnableKeyword(kw); else m.DisableKeyword(kw); }

        static void CopyLit(Material s, Material m)
        {
            m.SetColor("_BaseColor", s.HasProperty("_Color") ? s.GetColor("_Color") : Color.white);
            var tex = s.HasProperty("_MainTex") ? s.GetTexture("_MainTex") : null;
            m.SetTexture("_BaseMap", tex);
            if (tex != null)
            {
                m.SetTextureScale("_BaseMap", s.GetTextureScale("_MainTex"));
                m.SetTextureOffset("_BaseMap", s.GetTextureOffset("_MainTex"));
            }
            var nrm = s.HasProperty("_BumpMap") && s.IsKeywordEnabled("_NORMALMAP") ? s.GetTexture("_BumpMap") : null;
            m.SetTexture("_BumpMap", nrm);              // 法線の詰め方 (x = A・y = G) は URP も同じく読める
            if (nrm != null) m.SetFloat("_BumpScale", s.GetFloat("_BumpScale"));
            Keyword(m, "_NORMALMAP", nrm != null);
            m.SetFloat("_Smoothness", s.HasProperty("_Glossiness") ? s.GetFloat("_Glossiness") : 0.5f);
            m.SetFloat("_Metallic", s.HasProperty("_Metallic") ? s.GetFloat("_Metallic") : 0f);
            // 金属感・滑らかさの絵 (車体のシェル): R = 金属感・A = 滑らかさ。URP/Lit も同じ並びで読む
            var mg = s.IsKeywordEnabled("_METALLICGLOSSMAP") && s.HasProperty("_MetallicGlossMap") ? s.GetTexture("_MetallicGlossMap") : null;
            m.SetTexture("_MetallicGlossMap", mg);
            if (mg != null) m.SetFloat("_Smoothness", s.HasProperty("_GlossMapScale") ? s.GetFloat("_GlossMapScale") : 1f);
            Keyword(m, "_METALLICSPECGLOSSMAP", mg != null);

            // 描き方: 0 = 不透明・1 = 切り抜き・2 = 半透明 (Fade)・3 = 半透明 (Transparent。乗算済みアルファ = 車のクリア層)
            float mode = s.HasProperty("_Mode") ? s.GetFloat("_Mode") : 0f;
            bool cutout = Mathf.Approximately(mode, 1f), transparent = mode >= 1.5f, premul = mode >= 2.5f;
            m.SetFloat("_AlphaClip", cutout ? 1f : 0f);
            if (cutout) m.SetFloat("_Cutoff", s.GetFloat("_Cutoff"));
            Keyword(m, "_ALPHATEST_ON", cutout);
            m.SetFloat("_Cull", cutout ? 0f : 2f);                       // 木の板は両面
            m.SetFloat("_Surface", transparent ? 1f : 0f);
            m.SetFloat("_Blend", premul ? 1f : 0f);
            m.SetFloat("_SrcBlend", (float)(transparent && !premul ? BlendMode.SrcAlpha : BlendMode.One));
            m.SetFloat("_DstBlend", (float)(transparent ? BlendMode.OneMinusSrcAlpha : BlendMode.Zero));
            m.SetFloat("_SrcBlendAlpha", 1f);
            m.SetFloat("_DstBlendAlpha", transparent ? (float)BlendMode.OneMinusSrcAlpha : 0f);
            m.SetFloat("_ZWrite", transparent ? 0f : 1f);
            Keyword(m, "_SURFACE_TYPE_TRANSPARENT", transparent);
            Keyword(m, "_ALPHAPREMULTIPLY_ON", premul);
            m.SetOverrideTag("RenderType", transparent ? "Transparent" : cutout ? "TransparentCutout" : "Opaque");
            m.renderQueue = transparent ? (s.renderQueue >= 3000 ? s.renderQueue : 3000) : cutout ? 2450 : 2000;
            m.SetShaderPassEnabled("ShadowCaster", !transparent);
            m.SetShaderPassEnabled("DepthOnly", !transparent);

            bool detail = s.IsKeywordEnabled("_DETAIL_MULX2") && s.GetTexture("_DetailAlbedoMap") != null;
            if (detail)
            {
                // ★URP/Lit の detail は 1 番目の UV にだけ乗る (Built-in の地面は 2 番目の UV を使う)。タイルは元の値のまま写す
                m.SetTexture("_DetailAlbedoMap", s.GetTexture("_DetailAlbedoMap"));
                m.SetTextureScale("_DetailAlbedoMap", s.GetTextureScale("_DetailAlbedoMap"));
                m.SetTexture("_DetailNormalMap", s.GetTexture("_DetailNormalMap"));
                m.SetFloat("_DetailAlbedoMapScale", 1f);
                m.SetFloat("_DetailNormalMapScale", s.HasProperty("_DetailNormalMapScale") ? s.GetFloat("_DetailNormalMapScale") : 1f);
            }
            Keyword(m, "_DETAIL_MULX2", detail);
            // 車体の塗装: クリア層を外したぶん、下地のつやを上げる
            if (s.name == "CarPaint") m.SetFloat("_Smoothness", Mathf.Max(m.GetFloat("_Smoothness"), 0.86f));
        }
    }
}
#endif
