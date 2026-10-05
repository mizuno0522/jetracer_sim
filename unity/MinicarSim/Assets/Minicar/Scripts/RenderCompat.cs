// 描画の仕組み (Built-in / URP / HDRP) の違いを吸収する。3 つの版は同じ仕様の絵を出し、違うのは PC への負荷だけにする
// (2026-10-05 水野。HDRP 版の絵が正で、URP 版・Built-in 版をそれに合わせる。docs/hdrp.md)。
//
// コースと車はどの版でも Standard シェーダの材質で組み、組み上がったあとで版ごとの材質・光・空・霞・露出・後処理に置き換える:
//   RenderCompat.Builtin.cs (定義なし)  RenderCompat.Urp.cs (MINICAR_URP)  RenderCompat.Hdrp.cs (MINICAR_HDRP)
// 見た目の数値 (光の強さ・露出・後処理の強さ) はここの Look にまとめ、3 つの版が同じ値を読む。版ごとのファイルに数値を書かない。
//
// 色空間は 3 版とも Linear。センサカメラ (配信する画像) には後処理を掛けない (どの版・どの段階でも)。
using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Rendering;

namespace Minicar
{
    /// 画面表示の後処理の仕様 (3 版で同じ値)。low では post = false (後処理なし)
    public struct Look
    {
        public bool post;               // 後処理をするか (medium 以上)
        public float exposure;          // トーンカーブの前に掛ける明るさ (倍率)
        public float bloom, bloomThreshold;
        public float ao, aoRadius;      // 物の際の陰り (強さ・半径 [m])
        public bool dof;                // 遠くのぼけ (部屋の会場の追従視点)
        public float dofStart, dofEnd, dofBlur;
        public float vignette, vignetteSmooth;
        public float contrast, saturation;      // HDRP の ColorAdjustments と同じ単位 (−100〜100)
        public bool ssr;                // 床や板への映り込み
        public float shadowDistance;
    }

    public static partial class RenderCompat
    {
        public static string PipelineName
        {
            get
            {
                var rp = UnityEngine.Rendering.GraphicsSettings.currentRenderPipeline;
                return rp == null ? "Built-in" : rp.GetType().Name;
            }
        }

        // ---------------------------------------------------------------- 見た目の数値 (3 版で共通)
        /// 富士の太陽: HDRP の 100,000 lux・露出 kCircuitEV の絵と同じ明るさになる、Built-in / URP の光の強さ
        public static float CircuitSun => Tune("sun", 1.8f);
        // 太陽の色。HDRP は物理的な空が大気で日の光を赤く寄せるので、もとの色を渡す。Built-in・URP にはその結果の色を渡す
#if MINICAR_HDRP
        public static Color CircuitSunColor => new Color(1f, 0.95f, 0.86f);
#else
        public static Color CircuitSunColor => new Color(Tune("sunr", 1f), Tune("sung", 0.92f), Tune("sunb", 0.72f));
#endif
        public static Color CircuitAmbientTint => new Color(Tune("ambr", 0.95f), Tune("ambg", 1f), Tune("ambb", 1.05f));

        /// 富士の空 (Built-in・URP): HDRP の物理的な空に色と明るさを合わせた、大気の散乱の空。HDRP 版は自分の空を使うので何もしない
        public static void ApplyCircuitSky()
        {
#if !MINICAR_HDRP
            var src = Resources.Load<Material>("Mat_Sky");
            if (src == null) { Debug.LogWarning("[RenderCompat] Mat_Sky が無い (MinicarBuild.MakeMaterials)"); return; }
            var m = new Material(src);
            m.SetFloat("_Exposure", Tune("skyexp", 0.8f));
            m.SetFloat("_AtmosphereThickness", Tune("skythick", 1f));
            m.SetColor("_SkyTint", new Color(Tune("skyr", 0.5f), Tune("skyg", 0.56f), Tune("skyb", 0.5f)));
            m.SetColor("_GroundColor", new Color(0.369f, 0.349f, 0.341f));
            m.SetFloat("_SunSize", 0.04f);
            m.SetFloat("_SunSizeConvergence", 5f);
            RenderSettings.skybox = m;
#endif
        }

        /// 富士の環境光 (空・横・地面の 3 色) に掛ける倍率
        public static float CircuitAmbient => Tune("amb", 1.4f);
        /// ミニカーの会場のライトかく乱 (点光源) に掛ける倍率
        public static float PointLightGain => Tune("point", 1f);

        public static Look GetLook(bool circuit)
        {
            var q = RenderQuality.Current;
            var k = new Look
            {
                post = q != QualityTier.Low,
                exposure = Tune("exposure", 1f),
                bloom = 0.12f, bloomThreshold = 1f,
                ao = 0.6f, aoRadius = circuit ? 1.5f : 0.12f,
                vignette = 0f, vignetteSmooth = 0.45f,
                shadowDistance = circuit ? (q == QualityTier.High ? 600f : q == QualityTier.Medium ? 300f : 150f) : 8f,
            };
            if (!circuit)
            {
                // ミニカーの会場 (部屋): 小さな車を近くから撮った写真らしく。床や板への映り込み、物の際の陰り、遠くのぼけ、周辺の落ち込み
                k.ao = 1.0f; k.bloom = 0.22f;
                k.dof = true; k.dofStart = 4.5f; k.dofEnd = 22f; k.dofBlur = 3.5f;
                k.vignette = 0.22f;
                k.contrast = 10f; k.saturation = 6f;
                k.ssr = true;
            }
            k.bloom = Tune("bloom", k.bloom);
            k.ao = Tune("ao", k.ao);
            k.aoRadius = Tune("aoradius", k.aoRadius);
            k.dofBlur = Tune("dofblur", k.dofBlur);
            k.vignette = Tune("vignette", k.vignette);
            k.contrast = Tune("contrast", k.contrast);
            k.saturation = Tune("saturation", k.saturation);
            if (!k.post) { k.bloom = 0f; k.ao = 0f; k.dof = false; k.vignette = 0f; k.contrast = 0f; k.saturation = 0f; k.ssr = false; }
            return k;
        }

        // ---------------------------------------------------------------- 合わせ込み用の上書き
        // -tune "sun=1.2,amb=0.9": 見た目の数値を起動引数で上書きする (3 版の絵を合わせるとき、ビルドし直さずに試すため)。
        // ふだんは使わない。決まった値はコードの既定値に入れる
        static Dictionary<string, float> s_Tune;

        public static float Tune(string key, float def)
        {
            if (s_Tune == null)
            {
                s_Tune = new Dictionary<string, float>();
                foreach (var kv in SimBridge.Arg("-tune", "").Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries))
                {
                    int p = kv.IndexOf('=');
                    if (p > 0 && float.TryParse(kv.Substring(p + 1), NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
                        s_Tune[kv.Substring(0, p).Trim()] = v;
                }
                if (s_Tune.Count > 0) Debug.Log($"[RenderCompat] -tune: {string.Join(", ", s_Tune.Keys)}");
            }
            return s_Tune.TryGetValue(key, out float t) ? t : def;
        }

#if !MINICAR_HDRP
        static Cubemap s_Env;

        /// ミニカーの会場 (Built-in・URP): 映り込みに環境光と同じ 3 色 (上・横・下) の空を使う (HDRP の空と同じ)。
        /// これが無いと既定のスカイボックスが映り、暗い色の床や板ほど版ごとに明るさが違って見える
        public static void ApplyRoomReflection()
        {
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
#endif

        // 地面の細かい模様 (detail) を 2 番目の UV (m 単位) に乗せている材質。URP/Lit は 1 番目の UV にしか乗せられないので、
        // 1 番目の UV で同じ大きさになる倍率を覚えておく (URP 版だけが使う)
        public static readonly Dictionary<Material, Vector2> DetailUv0Scale = new Dictionary<Material, Vector2>();

        public static void RegisterDetailUv1(Material m, Mesh mesh)
        {
            var a = mesh.uv; var b = mesh.uv2;
            if (a == null || b == null || a.Length == 0 || a.Length != b.Length) return;
            Vector2 a0 = a[0], a1 = a[0], b0 = b[0], b1 = b[0];
            for (int i = 1; i < a.Length; i++)
            {
                a0 = Vector2.Min(a0, a[i]); a1 = Vector2.Max(a1, a[i]);
                b0 = Vector2.Min(b0, b[i]); b1 = Vector2.Max(b1, b[i]);
            }
            Vector2 da = a1 - a0, db = b1 - b0;
            if (da.x > 1e-6f && da.y > 1e-6f) DetailUv0Scale[m] = new Vector2(db.x / da.x, db.y / da.y);
        }

        /// コース・車・カメラを作り終えたあとに 1 回。sensors = 配信用のセンサカメラ (後処理を掛けない)
        public static void AfterBuild(CourseBuilder course, Camera[] sensors, bool circuit) { AfterBuildImpl(course, sensors, circuit); }

        /// 置き換えたあとに元の材質・光・環境光を変えたとき (エピソードの乱択化) に呼ぶ
        public static void Refresh() { RefreshImpl(); }

        /// 毎フレーム (元のコードが光の強さを書き換えたら版ごとの単位に換算し直す)
        public static void Tick() { TickImpl(); }

        static partial void TickImpl();
        static partial void AfterBuildImpl(CourseBuilder course, Camera[] sensors, bool circuit);
        static partial void RefreshImpl();
    }
}
