// URP で動いているときの置き換え (スクリプト定義 MINICAR_URP のときだけコンパイルされる。docs/hdrp.md)。
// Built-in (軽い・学習用) と HDRP (重い・きれい) の間の段。scripts/pick_unity_player.sh が GPU を見て 3 つから選ぶ。
//
//   材質: Standard → Universal Render Pipeline/Lit (色・テクスチャ・タイル・法線・滑らかさ・金属・切り抜き・半透明・detail を写す)。
//         Unlit/Texture・Sprites/Default (矢印板・文字・雲のドーム・軌跡) は URP でもそのまま描けるので触らない
//   光  : 3 版とも Linear 色空間。光の強さは Built-in 版と同じ明るさになるよう 2.2 乗して入れる (Built-in は強さを sRGB の値として読むため)
//   影  : URP は QualitySettings ではなく設定アセットの値を使うので、コードが決めた影の距離・分割・アンチエイリアスを写す
//   後処理 (medium 以上・表示用のカメラだけ): 数値は RenderCompat.GetLook (3 版で共通)。ACES・にじみ・際の陰り (SSAO)・遠くのぼけ・周辺減光・コントラスト
//   センサカメラ: 後処理と際の陰りを掛けない (際の陰りの無い 2 番目の描画器で描く)
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
        // Built-in は光の「色 × 強さ」を sRGB の値として読んでから Linear に直す (強さ 1.8 は 1.8^2.2 = 3.6 倍に効く)。
        // URP は強さを Linear の倍率としてそのまま使うので、同じ明るさになるよう 2.2 乗して入れる
        const float kGamma = 2.2f;

        static readonly Dictionary<Material, Material> s_Map = new Dictionary<Material, Material>();
        static readonly List<(Light light, float last)> s_Lights = new List<(Light, float)>();
        static Material s_LitTemplate, s_CoatTemplate, s_CoatMapTemplate;

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
            // 車体の塗装用のクリアコートつきの材質 (Complex Lit)。無ければ普通の Lit で描く
            s_CoatTemplate = Resources.Load<Material>("Mat_URPCoat");
            s_CoatMapTemplate = Resources.Load<Material>("Mat_URPCoatMap");
            var look = GetLook(circuit);
            bool low = !look.post;

            s_Circuit = circuit;
            if (!circuit) ApplyRoomReflection();

            // ---- 材質
            int n = ConvertAll();

            // ---- 光 (Gamma での見え方に合わせる)
            foreach (var l in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                l.intensity = Mathf.Pow(Mathf.Max(0f, l.intensity), kGamma);
                s_Lights.Add((l, l.intensity));
            }

            // ---- 影・アンチエイリアス: コース側と RenderQuality.ApplyBuiltin が QualitySettings に入れた値を設定アセットへ写す
            urp.shadowDistance = look.shadowDistance;
            urp.shadowCascadeCount = Mathf.Clamp(QualitySettings.shadowCascades, 1, 4);
            urp.msaaSampleCount = Mathf.Max(1, QualitySettings.antiAliasing);

            // ---- カメラ
            foreach (var cam in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                var d = cam.GetUniversalAdditionalCameraData();
                bool sensor = Array.IndexOf(sensors, cam) >= 0;
                // 配信するセンサ画像には後処理を掛けない (実機のカメラに無い)。真上からの全景 (RViz 風) は図として読むものなので掛けない
                d.renderPostProcessing = !sensor && !low;
                d.antialiasing = AntialiasingMode.None;
                d.renderShadows = true;
                if (sensor) d.SetRenderer(1);              // 際の陰り (SSAO) の無い描画器 (UrpSetup が 2 番目に置く)
                // 真上からの全景 (RViz 風) は図として読むものなので、ぼかさない: ぼけの無い Volume だけを見せる
                d.volumeLayerMask = cam.orthographic ? 1 << kOrthoVolumeLayer : 1 << 0;
            }

            // ---- 後処理 (全体に効く Volume)。真上からの全景用に、ぼけだけ外した同じものをもう 1 つ置く
            SetAO(urp, look);
            MakeVolume("URPVolume", 0, look);
            var flat = look; flat.dof = false;
            MakeVolume("URPVolumeOrtho", kOrthoVolumeLayer, flat);
            Debug.Log($"[RenderCompat] URP: materials on {n} renderers, {s_Lights.Count} lights, shadow {urp.shadowDistance:F0} m, quality {RenderQuality.Current}");
        }

        const int kOrthoVolumeLayer = 30;

        static void MakeVolume(string name, int layer, Look look)
        {
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            var tm = profile.Add<Tonemapping>(true);
            tm.mode.value = look.post ? TonemappingMode.ACES : TonemappingMode.None;
            var bloom = profile.Add<Bloom>(true);
            bloom.intensity.value = look.bloom;
            bloom.threshold.value = 0f;
            bloom.scatter.value = 0.7f;
            var ca = profile.Add<ColorAdjustments>(true);
            ca.postExposure.value = Mathf.Log(look.exposure * Tune("urpexp", 1f) / (1f + look.bloom * Tune("urpbloomcomp", 0.75f)), 2f);
            ca.contrast.value = look.contrast; ca.saturation.value = look.saturation;
            if (look.dof)
            {
                var dof = profile.Add<DepthOfField>(true);
                dof.mode.value = DepthOfFieldMode.Gaussian;
                dof.gaussianStart.value = look.dofStart;
                dof.gaussianEnd.value = look.dofEnd;
                dof.gaussianMaxRadius.value = look.dofBlur / 3.5f * Tune("urpdof", 1.5f);
            }
            if (look.vignette > 0f)
            {
                var vg = profile.Add<Vignette>(true);
                vg.intensity.value = look.vignette; vg.smoothness.value = look.vignetteSmooth;
            }
            var go = new GameObject(name) { layer = layer };
            var vol = go.AddComponent<Volume>();
            vol.isGlobal = true;
            vol.priority = 10f;
            vol.sharedProfile = profile;
        }

        /// 際の陰り (SSAO): UrpSetup が 1 番目の描画器に付けた機能の強さ・半径を入れる。設定の型は非公開なので名前で探す
        static void SetAO(UniversalRenderPipelineAsset urp, Look look)
        {
            try
            {
                const System.Reflection.BindingFlags bf = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public;
                var list = typeof(UniversalRenderPipelineAsset).GetField("m_RendererDataList", bf)?.GetValue(urp) as ScriptableRendererData[];
                if (list == null || list.Length == 0 || list[0] == null) { Debug.LogWarning("[RenderCompat] URP の描画器が見つからない (際の陰りなし)"); return; }
                foreach (var f in list[0].rendererFeatures)
                {
                    if (f == null || f.GetType().Name != "ScreenSpaceAmbientOcclusion") continue;
                    f.SetActive(look.ao > 0f);
                    var st = f.GetType().GetField("m_Settings", bf)?.GetValue(f);
                    if (st == null) continue;
                    void Set(string n, object v) { var fi = st.GetType().GetField(n, bf); if (fi != null) fi.SetValue(st, System.Convert.ChangeType(v, fi.FieldType.IsEnum ? typeof(int) : fi.FieldType)); }
                    Set("Intensity", look.ao * Tune("urpao", 0.5f));
                    Set("Radius", look.aoRadius);
                    Set("DirectLightingStrength", 0.25f);
                    list[0].SetDirty();
                    return;
                }
                Debug.LogWarning("[RenderCompat] URP の描画器に SSAO が無い (scripts/migrate_urp.sh の phase 2 をやり直す)");
            }
            catch (Exception e) { Debug.LogWarning($"[RenderCompat] SSAO: {e.Message}"); }
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

        static bool s_Circuit;

        static partial void RefreshImpl()
        {
            if (!s_Circuit) ApplyRoomReflection();
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
            // 車体の塗装 (CarPaint): Built-in の 2 枚目のクリア層の代わりに、クリアコートつきの材質で描く
            bool map = s.IsKeywordEnabled("_METALLICGLOSSMAP");
            var coat = s.name == "CarPaint" ? (map ? s_CoatMapTemplate : s_CoatTemplate) : null;
            var m = new Material(coat != null ? coat : s_LitTemplate) { name = s.name + "_URP" };
            CopyLit(s, m);
            if (coat != null) { m.EnableKeyword("_CLEARCOAT"); m.SetFloat("_ClearCoat", 1f); m.SetFloat("_ClearCoatMask", Tune("urpcoat", 0.4f)); m.SetFloat("_ClearCoatSmoothness", 0.95f); }
            s_Map[s] = m;
            return m;
        }

        // URP/Lit の detail は「0.5 で変化なし」を Linear の値で読む (Standard は sRGB の 0.5 を色空間の係数で 1 倍に直す)。
        // sRGB のままだと 0.5 が 0.21 と読まれて地面・カーペットが半分以下の暗さになるので、同じ画素を Linear の絵として持ち直す
        static readonly Dictionary<Texture, Texture2D> s_Detail = new Dictionary<Texture, Texture2D>();
        static Texture LinearCopy(Texture t)
        {
            if (t == null) return null;
            if (s_Detail.TryGetValue(t, out var done)) return done;
            var a = t as Texture2D;
            if (a == null || !a.isReadable) return t;
            var c = new Texture2D(a.width, a.height, TextureFormat.RGBA32, true, true) { name = a.name + "_Lin", wrapMode = a.wrapMode, anisoLevel = a.anisoLevel, filterMode = a.filterMode };
            c.SetPixels32(a.GetPixels32());
            c.Apply(true);
            s_Detail[t] = c;
            return c;
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
                // ★URP/Lit の detail は 1 番目の UV にだけ乗る (Built-in の地面は 2 番目の UV = m 単位を使う)。
                //   2 番目の UV を使う材質は、1 番目の UV で同じ大きさになるようタイルを換算する
                m.SetTexture("_DetailAlbedoMap", LinearCopy(s.GetTexture("_DetailAlbedoMap")));
                var tile = s.GetTextureScale("_DetailAlbedoMap");
                if (s.HasProperty("_UVSec") && s.GetFloat("_UVSec") > 0.5f && DetailUv0Scale.TryGetValue(s, out var k)) tile = Vector2.Scale(tile, k);
                m.SetTextureScale("_DetailAlbedoMap", tile);
                m.SetTexture("_DetailNormalMap", s.GetTexture("_DetailNormalMap"));
                m.SetFloat("_DetailAlbedoMapScale", 1f);
                m.SetFloat("_DetailNormalMapScale", s.HasProperty("_DetailNormalMapScale") ? s.GetFloat("_DetailNormalMapScale") : 1f);
            }
            Keyword(m, "_DETAIL_MULX2", detail);
        }
    }
}
#endif
