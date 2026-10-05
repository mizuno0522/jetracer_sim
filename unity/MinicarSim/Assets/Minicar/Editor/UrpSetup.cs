// URP 版のプロジェクトを作る手順の 2 段目 (MINICAR_URP のときだけコンパイルされる。1 段目は UrpMigration.Phase1):
//   URP の設定アセット (Assets/MinicarURP/MinicarURP.asset と描画器 MinicarURP_Renderer.asset) を作って全画質に割り当て、
//   色空間を Linear にし、URP の全体設定 (Global Settings) を用意し、実行時に使う URP/Lit の材質を Resources に置く。
//   Unity -batchmode -nographics -projectPath unity/MinicarSimURP -executeMethod Minicar.EditorTools.UrpSetup.Phase2
// 何度実行してもよい (あるものは作り直さない)。描画 API は Built-in 版と同じ (Linux の既定) のままにする。
#if MINICAR_URP
using System;
using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace Minicar.EditorTools
{
    public static class UrpSetup
    {
        const string kDir = "Assets/MinicarURP";
        const string kAsset = kDir + "/MinicarURP.asset";
        const string kRenderer = kDir + "/MinicarURP_Renderer.asset";

        [MenuItem("Minicar/URP Setup (phase 2)")]
        public static void Phase2()
        {
            bool ok = true;
            Directory.CreateDirectory(kDir + "/Resources");
            AssetDatabase.Refresh();

            // ---- 描画器 (前方描画) と設定アセット
            var data = AssetDatabase.LoadAssetAtPath<UniversalRendererData>(kRenderer);
            if (data == null)
            {
                data = ScriptableObject.CreateInstance<UniversalRendererData>();
                AssetDatabase.CreateAsset(data, kRenderer);
                // 後処理に使うシェーダ・テクスチャの束。空のままだと後処理が動かない
                ResourceReloader.ReloadAllNullIn(data, UniversalRenderPipelineAsset.packagePath);
                var pp = AssetDatabase.LoadAssetAtPath<PostProcessData>(UniversalRenderPipelineAsset.packagePath + "/Runtime/Data/PostProcessData.asset");
                if (pp == null) { Debug.LogError("[UrpSetup] PostProcessData.asset が見つからない"); ok = false; }
                data.postProcessData = pp;
                EditorUtility.SetDirty(data);
                Debug.Log($"[UrpSetup] created {kRenderer}");
            }
            var asset = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>(kAsset);
            if (asset == null)
            {
                asset = UniversalRenderPipelineAsset.Create(data);
                AssetDatabase.CreateAsset(asset, kAsset);
                Debug.Log($"[UrpSetup] created {kAsset}");
            }
            asset.supportsHDR = true;                       // ブルーム・トーンマップ用
            asset.supportsCameraDepthTexture = true;        // 遠くのぼけ (被写界深度) 用
            asset.msaaSampleCount = 2;
            asset.shadowDistance = 50f;                     // 実行時に RenderCompat がコースに合わせて入れ直す
            asset.shadowCascadeCount = 4;
            // 公開の設定口が無いもの (影の解像度・やわらかい影・点光源を画素ごとに) はフィールド名で入れる。名前が変わっていたら警告だけ
            var so = new SerializedObject(asset);
            void Set(string name, int v)
            {
                var p = so.FindProperty(name);
                if (p == null) { Debug.LogWarning($"[UrpSetup] {name} が見つからない (URP の版の違い)"); return; }
                if (p.propertyType == SerializedPropertyType.Boolean) p.boolValue = v != 0; else p.intValue = v;
            }
            Set("m_MainLightShadowmapResolution", 4096);
            Set("m_SoftShadowsSupported", 1);
            Set("m_AdditionalLightsRenderingMode", 1);      // 1 = 画素ごと (ライトかく乱の点光源)
            Set("m_AdditionalLightsPerObjectLimit", 4);
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(asset);

            GraphicsSettings.defaultRenderPipeline = asset;
            int cur = QualitySettings.GetQualityLevel();
            for (int i = 0; i < QualitySettings.names.Length; i++)
            {
                QualitySettings.SetQualityLevel(i, false);
                QualitySettings.renderPipeline = asset;
            }
            QualitySettings.SetQualityLevel(cur, false);

            // ---- プレイヤーの設定: 後処理を正しく掛けるため Linear 色空間 (光の強さは RenderCompat.Urp が換算する)
            PlayerSettings.colorSpace = ColorSpace.Linear;

            // ---- 全体設定 (Global Settings)。公開 API が版で違うので反射で用意を試みる
            EnsureGlobalSettings();

            // ---- 実行時に複製する URP/Lit (シェーダをビルドに確実に含める)
            var lit = Shader.Find("Universal Render Pipeline/Lit");
            if (lit == null) { Debug.LogError("[UrpSetup] Universal Render Pipeline/Lit shader not found"); ok = false; }
            else
            {
                string mp = kDir + "/Resources/Mat_URPLit.mat";
                if (AssetDatabase.LoadAssetAtPath<Material>(mp) == null) AssetDatabase.CreateAsset(new Material(lit), mp);
                MakeVariants(lit);
            }
            // 車体の塗装用: クリアコートつきの Complex Lit (金属的な下地の上に透明な層。ソウルレッドの深さはこれで出る)
            var coatSh = Shader.Find("Universal Render Pipeline/Complex Lit");
            if (coatSh == null) Debug.LogWarning("[UrpSetup] Universal Render Pipeline/Complex Lit が無い (車体のクリアコートは付かない)");
            else
            {
                foreach (bool map in new[] { false, true })
                {
                    var m = new Material(coatSh) { name = map ? "Mat_URPCoatMap" : "Mat_URPCoat" };
                    m.SetFloat("_ClearCoat", 1f);
                    m.SetFloat("_ClearCoatMask", 1f);
                    m.SetFloat("_ClearCoatSmoothness", 0.95f);
                    m.EnableKeyword("_CLEARCOAT");
                    if (map)
                    {
                        m.SetTexture("_MetallicGlossMap", AssetDatabase.LoadAssetAtPath<Texture2D>(kDir + "/Resources/URPVariants/White.asset"));
                        m.EnableKeyword("_METALLICSPECGLOSSMAP");
                    }
                    string cp = $"{kDir}/Resources/{m.name}.mat";
                    if (AssetDatabase.LoadAssetAtPath<Material>(cp) != null) AssetDatabase.DeleteAsset(cp);
                    AssetDatabase.CreateAsset(m, cp);
                }
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[UrpSetup] done ok={ok} pipeline={GraphicsSettings.defaultRenderPipeline?.name} colorSpace={PlayerSettings.colorSpace}");
            if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
        }

        // URP/Lit の法線・detail・切り抜き・半透明・金属感の絵は shader_feature なので、ビルドに入る材質が使っていない組み合わせは
        // プレイヤーから削られる。RenderCompat.Urp が実行時に作る組み合わせを材質として置いておく
        static void MakeVariants(Shader lit)
        {
            string dir = kDir + "/Resources/URPVariants";
            Directory.CreateDirectory(dir);
            AssetDatabase.Refresh();
            var white = TexAsset(dir + "/White.asset", new Color32(255, 255, 255, 255), false);
            var grey = TexAsset(dir + "/Grey.asset", new Color32(128, 128, 128, 255), true);
            var flatN = TexAsset(dir + "/FlatNormal.asset", new Color32(255, 128, 128, 128), true);
            // 0 = 不透明・1 = 切り抜き・2 = 半透明 (アルファ)・3 = 半透明 (乗算済み)
            string[][] modes = { new string[0], new[] { "_ALPHATEST_ON" }, new[] { "_SURFACE_TYPE_TRANSPARENT" }, new[] { "_SURFACE_TYPE_TRANSPARENT", "_ALPHAPREMULTIPLY_ON" } };
            for (int i = 0; i < modes.Length; i++)
                for (int k = 0; k < 8; k++)
                {
                    if (i == 0 && k == 0) continue;            // Mat_URPLit と同じ
                    var m = new Material(lit) { name = $"URP_{i}{k}" };
                    // ★キーワードだけ立てても、URP が材質を取り込むときに「設定値から決め直す」ので消える (木の切り抜きが効かず黒い板になった)。
                    //   キーワードの元になる設定値とテクスチャも入れておく
                    if (i == 1) m.SetFloat("_AlphaClip", 1f);
                    if (i >= 2)
                    {
                        m.SetFloat("_Surface", 1f);
                        m.SetFloat("_Blend", i == 3 ? 1f : 0f);
                        m.SetFloat("_SrcBlend", (float)(i == 3 ? BlendMode.One : BlendMode.SrcAlpha));
                        m.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
                        m.SetFloat("_ZWrite", 0f);
                        m.SetOverrideTag("RenderType", "Transparent");
                        m.renderQueue = 3000;
                    }
                    if ((k & 1) != 0) m.SetTexture("_BumpMap", flatN);
                    if ((k & 2) != 0) { m.SetTexture("_DetailAlbedoMap", grey); m.SetTexture("_DetailNormalMap", flatN); }
                    if ((k & 4) != 0) m.SetTexture("_MetallicGlossMap", white);
                    foreach (var kw in modes[i]) m.EnableKeyword(kw);
                    if ((k & 1) != 0) m.EnableKeyword("_NORMALMAP");
                    if ((k & 2) != 0) m.EnableKeyword("_DETAIL_MULX2");
                    if ((k & 4) != 0) m.EnableKeyword("_METALLICSPECGLOSSMAP");
                    string path = $"{dir}/{m.name}.mat";
                    if (AssetDatabase.LoadAssetAtPath<Material>(path) != null) AssetDatabase.DeleteAsset(path);
                    AssetDatabase.CreateAsset(m, path);
                }
            Debug.Log($"[UrpSetup] shader variant materials → {dir}");
        }

        static Texture2D TexAsset(string path, Color32 c, bool linear)
        {
            var old = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
            if (old != null) return old;
            var t = new Texture2D(4, 4, TextureFormat.RGBA32, false, linear);
            var px = new Color32[16];
            for (int i = 0; i < px.Length; i++) px[i] = c;
            t.SetPixels32(px);
            t.Apply();
            AssetDatabase.CreateAsset(t, path);
            return t;
        }

        static void EnsureGlobalSettings()
        {
            var t = typeof(UniversalRenderPipelineAsset).Assembly.GetType("UnityEngine.Rendering.Universal.UniversalRenderPipelineGlobalSettings");
            if (t == null) { Debug.LogWarning("[UrpSetup] UniversalRenderPipelineGlobalSettings が見つからない"); return; }
            foreach (var m in t.GetMethods(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic))
            {
                if (m.Name != "Ensure" || m.ContainsGenericParameters) continue;
                try
                {
                    var ps = m.GetParameters();
                    var args = new object[ps.Length];
                    for (int i = 0; i < ps.Length; i++)
                        args[i] = ps[i].HasDefaultValue ? ps[i].DefaultValue : (ps[i].ParameterType == typeof(bool) ? (object)true : null);
                    var r = m.Invoke(null, args);
                    Debug.Log($"[UrpSetup] UniversalRenderPipelineGlobalSettings.Ensure → {(r as UnityEngine.Object)?.name ?? r?.ToString() ?? "(void)"}");
                    return;
                }
                catch (Exception e) { Debug.LogWarning($"[UrpSetup] Ensure failed: {e.InnerException?.Message ?? e.Message}"); }
            }
            // 見つからなければ、URP を初めて描くときに自動で作られる
            Debug.LogWarning("[UrpSetup] Global Settings を明示的に作れなかった (ビルドのログで警告を確かめる)");
        }
    }
}
#endif
