// HDRP 版のプロジェクトを作る手順の 2 段目 (MINICAR_HDRP のときだけコンパイルされる。1 段目は HdrpMigration.Phase1):
//   HDRP の設定アセット (Assets/MinicarHDRP/MinicarHDRP.asset) を作って全画質に割り当て、色空間を Linear、
//   Linux の描画 API を Vulkan にし、HDRP の全体設定 (Global Settings) を用意し、実行時に使う HDRP/Lit の材質を Resources に置く。
//   Unity -batchmode -nographics -projectPath unity/MinicarSimHDRP -executeMethod Minicar.EditorTools.HdrpSetup.Phase2
// 何度実行してもよい (あるものは作り直さない)。うまくいかないときはエディタで開き、Window > Rendering > HDRP Wizard の Fix All。
#if MINICAR_HDRP
using System;
using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Minicar.EditorTools
{
    public static class HdrpSetup
    {
        const string kDir = "Assets/MinicarHDRP";
        const string kAsset = kDir + "/MinicarHDRP.asset";

        [MenuItem("Minicar/HDRP Setup (phase 2)")]
        public static void Phase2()
        {
            bool ok = true;
            Directory.CreateDirectory(kDir + "/Resources");
            AssetDatabase.Refresh();

            // ---- 設定アセット
            var asset = AssetDatabase.LoadAssetAtPath<HDRenderPipelineAsset>(kAsset);
            if (asset == null)
            {
                asset = ScriptableObject.CreateInstance<HDRenderPipelineAsset>();
                AssetDatabase.CreateAsset(asset, kAsset);
                Debug.Log($"[HdrpSetup] created {kAsset}");
            }
            // 使う機能 (立体の雲・霧・AO)。フィールド名が変わっていたら警告だけ出して進む
            var so = new SerializedObject(asset);
            foreach (var name in new[] { "supportVolumetricClouds", "supportVolumetrics", "supportSSAO", "supportMotionVectors" })
            {
                var p = so.FindProperty("m_RenderPipelineSettings." + name);
                if (p != null && p.propertyType == SerializedPropertyType.Boolean) p.boolValue = true;
                else Debug.LogWarning($"[HdrpSetup] m_RenderPipelineSettings.{name} が見つからない (HDRP の版の違い)");
            }
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

            // ---- プレイヤーの設定: HDRP は Linear 色空間と、Linux では Vulkan が要る
            PlayerSettings.colorSpace = ColorSpace.Linear;
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.StandaloneLinux64, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.StandaloneLinux64, new[] { GraphicsDeviceType.Vulkan });

            // ---- 全体設定 (Global Settings)。公開 API が版で違うので反射で用意を試みる
            ok &= EnsureGlobalSettings();

            // ---- 実行時に複製する HDRP/Lit (シェーダをビルドに確実に含める)
            var lit = Shader.Find("HDRP/Lit");
            if (lit == null) { Debug.LogError("[HdrpSetup] HDRP/Lit shader not found"); ok = false; }
            else
            {
                string mp = kDir + "/Resources/Mat_HDLit.mat";
                if (AssetDatabase.LoadAssetAtPath<Material>(mp) == null) AssetDatabase.CreateAsset(new Material(lit), mp);
                MakeVariants(lit);
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[HdrpSetup] done ok={ok} pipeline={GraphicsSettings.defaultRenderPipeline?.name} colorSpace={PlayerSettings.colorSpace}");
            if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
        }

        // HDRP/Lit の切り抜き・法線・detail などは shader_feature なので、ビルドに入る材質が使っていない組み合わせは
        // プレイヤーから削られる (エディタでは見えるのにプレイヤーで木が四角くなる)。実行時に作る組み合わせを材質として置いておく
        static void MakeVariants(Shader lit)
        {
            string dir = kDir + "/Resources/HDVariants";
            Directory.CreateDirectory(dir);
            AssetDatabase.Refresh();
            var white = TexAsset(dir + "/White.asset", new Color32(255, 255, 255, 255), false);
            var flatN = TexAsset(dir + "/FlatNormal.asset", new Color32(255, 128, 128, 128), true);
            var det = TexAsset(dir + "/Detail.asset", new Color32(128, 128, 128, 128), true);
            void V(string name, bool normal, bool detail, bool cutout, bool transparent, bool coat = false)
            {
                var m = new Material(lit) { name = name };
                m.SetTexture("_BaseColorMap", white);
                if (normal) m.SetTexture("_NormalMap", flatN);
                if (detail)
                {
                    m.SetTexture("_DetailMap", det);
                    m.SetFloat("_UVDetail", 1f);
                    m.SetVector("_UVDetailsMappingMask", new Vector4(0, 1, 0, 0));
                    m.SetFloat("_LinkDetailsWithBase", 0f);
                }
                m.SetFloat("_AlphaCutoffEnable", cutout ? 1f : 0f);
                m.SetFloat("_DoubleSidedEnable", cutout ? 1f : 0f);
                m.SetFloat("_SurfaceType", transparent ? 1f : 0f);
                if (transparent) m.SetFloat("_BlendMode", 0f);
                m.SetFloat("_CoatMask", coat ? 1f : 0f);
                HDMaterial.ValidateMaterial(m);
                string path = $"{dir}/{name}.mat";
                if (AssetDatabase.LoadAssetAtPath<Material>(path) != null) AssetDatabase.DeleteAsset(path);
                AssetDatabase.CreateAsset(m, path);
            }
            // RenderCompat.CopyLit が作る組み合わせ: 車の塗装 (Coat)・壁 (Plain)・路面 (Normal)・地面 (Normal + Detail UV1)・木 (Cutout)・タイヤ痕 (Transparent)
            V("Plain", false, false, false, false);
            V("Normal", true, false, false, false);
            V("NormalDetail", true, true, false, false);
            V("Cutout", false, false, true, false);
            V("CutoutNormal", true, false, true, false);
            V("Transparent", false, false, false, true);
            V("TransparentNormal", true, false, false, true);
            V("Coat", false, false, false, false, true);           // 車体の塗装 (クリアコート)
            Debug.Log($"[HdrpSetup] shader variant materials → {dir}");
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

        static bool EnsureGlobalSettings()
        {
            var t = typeof(HDRenderPipelineGlobalSettings);
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
                    Debug.Log($"[HdrpSetup] HDRenderPipelineGlobalSettings.Ensure → {(r as UnityEngine.Object)?.name ?? r?.ToString() ?? "(void)"}");
                    return true;
                }
                catch (Exception e) { Debug.LogWarning($"[HdrpSetup] Ensure failed: {e.InnerException?.Message ?? e.Message}"); }
            }
            // 見つからなければ、HDRP を初めて描くときに自動で作られる版もある。ビルドのログで "Global Settings" の警告を確かめる
            Debug.LogWarning("[HdrpSetup] Global Settings を明示的に作れなかった。ビルドで警告が出たらエディタで HDRP Wizard の Fix All を 1 回");
            return true;
        }
    }
}
#endif
