// 画質の段階 (-quality low | medium | high | auto)。既定 auto = GPU を見て選ぶ。
//
//   low    : ML-Agents の学習・弱い GPU 用。今までと同じ描画 (森の影だけ切る)。-mlagents のときの既定
//   medium : 表示の既定。影を遠くまで、画面のアンチエイリアス 2x
//   high   : 影をさらに遠く・細かく、アンチエイリアス 4x、遠くの細部 (LOD) を高く
//
// auto の決め方 (SystemInfo。起動ログと bench.md に理由を出す):
//   ソフトウェア描画 (llvmpipe など)・内蔵 GPU (Intel・AMD APU)・ビデオメモリ 2 GB 未満 → low
//   ビデオメモリ 6 GB 未満 (例: Radeon RX 5300M 3 GB・GTX 1650 4 GB) → medium、それ以上 → high
// 配信するセンサ画像 (車載カメラ) は段階によらず同じ設定で描く (RenderTexture のアンチエイリアスは常に 1)。
// HDRP で動かすとき (unity/MinicarSimHDRP、docs/hdrp.md) は RenderCompat が同じ段階で空・霞・後処理を決める。
using System;
using UnityEngine;

namespace Minicar
{
    public enum QualityTier { Low, Medium, High }

    public static class RenderQuality
    {
        public static QualityTier Current { get; private set; } = QualityTier.Low;
        public static string Reason { get; private set; } = "";
        public static bool Initialized { get; private set; }

        /// 起動時に 1 回 (コースを組む前)。-quality を読み、auto なら GPU から決める
        public static void Init()
        {
            if (Initialized) return;
            Initialized = true;
            string q = SimBridge.Arg("-quality", "auto").ToLowerInvariant();
            bool ml = Array.IndexOf(Environment.GetCommandLineArgs(), "-mlagents") >= 0;
            if (q == "low" || q == "medium" || q == "high")
            {
                Current = q == "low" ? QualityTier.Low : q == "medium" ? QualityTier.Medium : QualityTier.High;
                Reason = "-quality " + q;
            }
            else if (ml)
            {
                Current = QualityTier.Low;
                Reason = "auto: -mlagents (学習は low)";
            }
            else { Current = Auto(out string why); Reason = why; }
            Debug.Log($"[RenderQuality] {Current} ({Reason}) GPU={SystemInfo.graphicsDeviceName} {SystemInfo.graphicsMemorySize} MB {SystemInfo.graphicsDeviceType}");
        }

        static QualityTier Auto(out string why)
        {
            string name = (SystemInfo.graphicsDeviceName ?? "").ToLowerInvariant();
            int mb = SystemInfo.graphicsMemorySize;
            if (name.Contains("llvmpipe") || name.Contains("swiftshader") || name.Contains("software") || name.Contains("basic render"))
            { why = "auto: ソフトウェア描画"; return QualityTier.Low; }
            if (IsIntegrated(name))
            { why = $"auto: 内蔵 GPU ({SystemInfo.graphicsDeviceName})"; return QualityTier.Low; }
            if (mb > 0 && mb < 2048) { why = $"auto: ビデオメモリ {mb} MB < 2 GB"; return QualityTier.Low; }
            if (mb > 0 && mb < 6144) { why = $"auto: ビデオメモリ {mb} MB < 6 GB"; return QualityTier.Medium; }
            why = $"auto: ビデオメモリ {mb} MB";
            return QualityTier.High;
        }

        static bool IsIntegrated(string n)
        {
            // Intel の内蔵 (UHD / Iris / HD Graphics)。Arc は単体 GPU
            if (n.Contains("intel") && !n.Contains("arc")) return true;
            // AMD の APU: "AMD Radeon(TM) Graphics"・"Radeon Vega 8 Graphics"・"Radeon 780M" 等 (RX / Pro の型番が無い)
            if (n.Contains("radeon") && !n.Contains(" rx") && !n.Contains("pro ") && !n.Contains("firepro")
                && (n.Contains("(tm) graphics") || n.Contains("vega") || n.Contains("radeon graphics") || n.EndsWith("m graphics") || n.Contains("780m") || n.Contains("760m") || n.Contains("680m") || n.Contains("660m")))
                return true;
            return false;
        }

        /// 画面表示の設定 (Built-in の描画)。low は今までの設定をそのまま使う
        public static void ApplyBuiltin(bool circuit)
        {
            switch (Current)
            {
                case QualityTier.Low:
                    break;
                case QualityTier.Medium:
                    QualitySettings.antiAliasing = 2;
                    if (circuit)
                    {
                        QualitySettings.shadowDistance = 300f;
                        QualitySettings.shadowCascades = 4;
                        QualitySettings.shadowResolution = ShadowResolution.High;
                    }
                    break;
                case QualityTier.High:
                    QualitySettings.antiAliasing = 4;
                    QualitySettings.lodBias = 2f;
                    QualitySettings.shadowResolution = ShadowResolution.VeryHigh;
                    if (circuit)
                    {
                        QualitySettings.shadowDistance = 600f;
                        QualitySettings.shadowCascades = 4;
                    }
                    break;
            }
        }

        /// 森が影を落とすか (木 2.6 万本の影は弱い GPU で重い)
        public static bool TreeShadows => Current != QualityTier.Low;
    }
}
