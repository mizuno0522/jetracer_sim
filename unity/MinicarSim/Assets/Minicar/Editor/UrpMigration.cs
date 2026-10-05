// URP 版のプロジェクト (unity/MinicarSimURP) を作る手順の 1 段目: URP のパッケージを入れ、スクリプト定義 MINICAR_URP を足す。
// HdrpMigration (HDRP 版の 1 段目) と同じ流れ。URP の型を使わないので、URP が入っていないプロジェクトでもコンパイルできる。
// scripts/migrate_urp.sh から呼ぶ:
//   Unity -batchmode -nographics -projectPath unity/MinicarSimURP -executeMethod Minicar.EditorTools.UrpMigration.Phase1
// (-quit は付けない。パッケージの取得が終わったら自分で終了する)。2 段目は UrpSetup.Phase2 (MINICAR_URP のときだけある)
using System;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.PackageManager.Requests;
using UnityEngine;

namespace Minicar.EditorTools
{
    [InitializeOnLoad]
    public static class UrpMigration
    {
        // パッケージを入れたあとの再読み込み (domain reload) で Poll が消えた場合: 起動引数が Phase1 で、
        // manifest にもう URP が入っていれば、定義を足して終了する (ここで止まり続けないように)
        static UrpMigration()
        {
            if (!Application.isBatchMode) return;
            if (Array.IndexOf(Environment.GetCommandLineArgs(), "Minicar.EditorTools.UrpMigration.Phase1") < 0) return;
            EditorApplication.delayCall += () =>
            {
                if (s_Request != null) return;                     // まだ Phase1 の最中 (再読み込み前)
                string manifest = System.IO.File.ReadAllText("Packages/manifest.json");
                if (!manifest.Contains(kPackage)) return;
                Debug.Log("[UrpMigration] package present after reload");
                HdrpMigration.SetDefine(kDefine, true);
                AssetDatabase.SaveAssets();
                EditorApplication.Exit(0);
            };
        }

        const string kPackage = "com.unity.render-pipelines.universal";
        const string kDefine = "MINICAR_URP";
        static AddRequest s_Request;
        static double s_T0;

        public static void Phase1()
        {
            // 版を指定しない = この Unity に合う版 (Unity 6 では Editor に付属の版) を入れる
            Debug.Log($"[UrpMigration] adding {kPackage} …");
            s_T0 = EditorApplication.timeSinceStartup;
            s_Request = Client.Add(kPackage);
            EditorApplication.update += Poll;
        }

        static void Poll()
        {
            if (!s_Request.IsCompleted)
            {
                if (EditorApplication.timeSinceStartup - s_T0 > 1800) { Debug.LogError("[UrpMigration] timeout"); EditorApplication.Exit(2); }
                return;
            }
            EditorApplication.update -= Poll;
            if (s_Request.Status != StatusCode.Success)
            {
                Debug.LogError($"[UrpMigration] package add failed: {s_Request.Error?.message}");
                EditorApplication.Exit(1);
                return;
            }
            Debug.Log($"[UrpMigration] installed {s_Request.Result.name} {s_Request.Result.version}");
            HdrpMigration.SetDefine(kDefine, true);
            AssetDatabase.SaveAssets();
            EditorApplication.Exit(0);
        }
    }
}
