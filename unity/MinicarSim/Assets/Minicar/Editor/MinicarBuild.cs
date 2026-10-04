// バッチモードでシーン・マテリアル・Linux プレイヤーを作る。
//
//   Unity -batchmode -nographics -projectPath unity/MinicarSim \
//         -executeMethod Minicar.EditorTools.MinicarBuild.SetupAndBuild -quit
//
// 出力: unity/MinicarSim/Build/MinicarSim.x86_64
// (unity/build_player.sh がコースの書き出しからここまでをまとめて行う)
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Minicar.EditorTools
{
    public static class MinicarBuild
    {
        const string kScene = "Assets/Scenes/Minicar.unity";
        const string kResDir = "Assets/Minicar/Resources";
        const string kOut = "Build/MinicarSim.x86_64";

        [MenuItem("Minicar/Setup Scene")]
        public static void Setup()
        {
            // ROS-TCP-Connector を ROS 2 の配線 (Header に seq が無い等) でコンパイルする
            // HDRP 版 (scripts/migrate_hdrp.sh) が足す MINICAR_HDRP などは残す
            var defs = new System.Collections.Generic.List<string>(
                PlayerSettings.GetScriptingDefineSymbols(NamedBuildTarget.Standalone).Split(new[] { ';' }, System.StringSplitOptions.RemoveEmptyEntries));
            if (!defs.Contains("ROS2")) defs.Insert(0, "ROS2");
            PlayerSettings.SetScriptingDefineSymbols(NamedBuildTarget.Standalone, string.Join(";", defs));

            PlayerSettings.productName = "MinicarSim";
            PlayerSettings.companyName = "minicarbattle2026";
            PlayerSettings.runInBackground = true;       // 端末にフォーカスがあっても描き続ける
            PlayerSettings.visibleInBackground = true;
            PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
            PlayerSettings.defaultScreenWidth = 1280;
            PlayerSettings.defaultScreenHeight = 720;
            PlayerSettings.resizableWindow = true;
            PlayerSettings.usePlayerLog = true;

            MakeMaterials();
            MakeScene();
        }

        static void MakeMaterials()
        {
            Directory.CreateDirectory(kResDir);
            // 実行時に Resources.Load で複製して色を付ける。シェーダを
            // ビルドに確実に含めるため、アセットとして持っておく。
            Save(new Material(Shader.Find("Standard")), "Mat_Lit");
            Save(new Material(Shader.Find("Unlit/Texture")), "Mat_Unlit");
            // 頂点色で塗る (走行軌跡・凡例バー)。Sprites/Default は両面・ライティングなし
            Save(new Material(Shader.Find("Sprites/Default")), "Mat_VertexColor");
            // 車載カメラの後処理 (Assets/Minicar/Shaders/SensorPost.shader)。course.json の realism.enable で使う
            var post = Shader.Find("Minicar/SensorPost");
            if (post == null) Debug.LogError("[MinicarBuild] Minicar/SensorPost shader not found");
            else Save(new Material(post), "Mat_SensorPost");
            MakeStandardVariants();
        }

        // Standard の法線・detail・切り抜き・半透明は shader_feature なので、ビルドに入る材質が使っていない組み合わせは
        // プレイヤーから削られる (木が四角い板になり、車のクリア層が白く塗りつぶす)。実行時に作る組み合わせを材質として置いておく
        static void MakeStandardVariants()
        {
            Directory.CreateDirectory(kResDir + "/StdVariants");
            string[] modes = { null, "_ALPHATEST_ON", "_ALPHABLEND_ON", "_ALPHAPREMULTIPLY_ON" };
            for (int i = 0; i < modes.Length; i++)
                for (int k = 0; k < 4; k++)
                {
                    if (i == 0 && k == 0) continue;            // Mat_Lit と同じ
                    var m = new Material(Shader.Find("Standard"));
                    if (modes[i] != null) m.EnableKeyword(modes[i]);
                    if ((k & 1) != 0) m.EnableKeyword("_NORMALMAP");
                    if ((k & 2) != 0) m.EnableKeyword("_DETAIL_MULX2");
                    Save(m, $"StdVariants/Std_{i}{k}");
                }
            // 車体のシェル: 金属感・滑らかさの絵 (不透明)
            var mg = new Material(Shader.Find("Standard"));
            mg.EnableKeyword("_METALLICGLOSSMAP");
            Save(mg, "StdVariants/Std_MetalMap");
        }

        static void Save(Material m, string name)
        {
            string path = $"{kResDir}/{name}.mat";
            if (AssetDatabase.LoadAssetAtPath<Material>(path) != null)
                AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(m, path);
        }

        static void MakeScene()
        {
            Directory.CreateDirectory(Path.GetDirectoryName(kScene));
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var go = new GameObject("MinicarSim");
            go.AddComponent<CourseBuilder>();
            go.AddComponent<SimBridge>();
            EditorSceneManager.SaveScene(scene, kScene);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(kScene, true) };
            AssetDatabase.SaveAssets();
        }

        public static void SetupAndBuild()
        {
            Setup();
            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = new[] { kScene },
                locationPathName = kOut,
                target = BuildTarget.StandaloneLinux64,
                options = BuildOptions.None,
            });
            Debug.Log($"[MinicarBuild] result={report.summary.result} errors={report.summary.totalErrors} out={kOut}");
            if (Application.isBatchMode)
                EditorApplication.Exit(report.summary.result == UnityEditor.Build.Reporting.BuildResult.Succeeded ? 0 : 1);
        }
    }
}
