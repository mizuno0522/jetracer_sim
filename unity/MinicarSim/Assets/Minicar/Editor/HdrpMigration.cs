// HDRP 版のプロジェクト (unity/MinicarSimHDRP) を作る手順の 1 段目: HDRP のパッケージを入れ、スクリプト定義 MINICAR_HDRP を足す。
// HDRP の型を使わないので、HDRP が入っていないプロジェクトでもコンパイルできる。scripts/migrate_hdrp.sh から呼ぶ:
//   Unity -batchmode -nographics -projectPath unity/MinicarSimHDRP -executeMethod Minicar.EditorTools.HdrpMigration.Phase1
// (-quit は付けない。パッケージの取得が終わったら自分で終了する)。2 段目は HdrpSetup.Phase2 (MINICAR_HDRP のときだけある)
using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.PackageManager;
using UnityEditor.PackageManager.Requests;
using UnityEngine;

namespace Minicar.EditorTools
{
    [InitializeOnLoad]
    public static class HdrpMigration
    {
        // パッケージを入れたあとの再読み込み (domain reload) で Poll が消えた場合: 起動引数が Phase1 で、
        // manifest にもう HDRP が入っていれば、定義を足して終了する (ここで止まり続けないように)
        static HdrpMigration()
        {
            if (!Application.isBatchMode) return;
            if (Array.IndexOf(Environment.GetCommandLineArgs(), "Minicar.EditorTools.HdrpMigration.Phase1") < 0) return;
            EditorApplication.delayCall += () =>
            {
                if (s_Request != null) return;                     // まだ Phase1 の最中 (再読み込み前)
                string manifest = System.IO.File.ReadAllText("Packages/manifest.json");
                if (!manifest.Contains(kPackage)) return;
                Debug.Log("[HdrpMigration] package present after reload");
                SetDefine(kDefine, true);
                AssetDatabase.SaveAssets();
                EditorApplication.Exit(0);
            };
        }

        const string kPackage = "com.unity.render-pipelines.high-definition";
        const string kDefine = "MINICAR_HDRP";
        static AddRequest s_Request;
        static double s_T0;

        public static void Phase1()
        {
            // 版を指定しない = この Unity に合う版 (Unity 6 では Editor に付属の版) を入れる
            Debug.Log($"[HdrpMigration] adding {kPackage} …");
            s_T0 = EditorApplication.timeSinceStartup;
            s_Request = Client.Add(kPackage);
            EditorApplication.update += Poll;
        }

        static void Poll()
        {
            if (!s_Request.IsCompleted)
            {
                if (EditorApplication.timeSinceStartup - s_T0 > 1800) { Debug.LogError("[HdrpMigration] timeout"); EditorApplication.Exit(2); }
                return;
            }
            EditorApplication.update -= Poll;
            if (s_Request.Status != StatusCode.Success)
            {
                Debug.LogError($"[HdrpMigration] package add failed: {s_Request.Error?.message}");
                EditorApplication.Exit(1);
                return;
            }
            Debug.Log($"[HdrpMigration] installed {s_Request.Result.name} {s_Request.Result.version}");
            SetDefine(kDefine, true);
            AssetDatabase.SaveAssets();
            EditorApplication.Exit(0);
        }

        public static void SetDefine(string define, bool on)
        {
            var t = NamedBuildTarget.Standalone;
            var defs = new List<string>(PlayerSettings.GetScriptingDefineSymbols(t).Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries));
            if (on && !defs.Contains(define)) defs.Add(define);
            if (!on) defs.Remove(define);
            if (!defs.Contains("ROS2")) defs.Insert(0, "ROS2");
            PlayerSettings.SetScriptingDefineSymbols(t, string.Join(";", defs));
            Debug.Log($"[HdrpMigration] defines: {string.Join(";", defs)}");
        }
    }
}
