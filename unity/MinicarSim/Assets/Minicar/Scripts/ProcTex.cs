// 実行時に作る路面などのテクスチャ (画像ファイルを持たない。シードで決まる)。
// どれも上下左右がつながる (タイル貼りで継ぎ目が出ない)。値ノイズの fBm と画素ごとのハッシュで作る。
// tools/preview_circuit.py が同じ式で絵を描くので、式を変えたらそちらも合わせる。
using UnityEngine;

namespace Minicar
{
    public static class ProcTex
    {
        // ------------------------------------------------------------ ノイズ
        public static float Hash(int x, int y, int s)
        {
            unchecked
            {
                uint h = (uint)x * 374761393u + (uint)y * 668265263u + (uint)s * 2246822519u;
                h = (h ^ (h >> 13)) * 1274126177u;
                h ^= h >> 16;
                return (h & 0xFFFFFFu) / 16777215f;
            }
        }

        static int Wrap(int i, int p) { int r = i % p; return r < 0 ? r + p : r; }

        /// 周期 period の値ノイズ (u, v は 0〜1 で 1 周期 = period 格子)
        public static float VNoise(float u, float v, int period, int seed)
        {
            float x = u * period, y = v * period;
            int x0 = Mathf.FloorToInt(x), y0 = Mathf.FloorToInt(y);
            float fx = x - x0, fy = y - y0;
            fx = fx * fx * (3f - 2f * fx);
            fy = fy * fy * (3f - 2f * fy);
            int xa = Wrap(x0, period), xb = Wrap(x0 + 1, period), ya = Wrap(y0, period), yb = Wrap(y0 + 1, period);
            float a = Hash(xa, ya, seed), b = Hash(xb, ya, seed), c = Hash(xa, yb, seed), d = Hash(xb, yb, seed);
            return Mathf.Lerp(Mathf.Lerp(a, b, fx), Mathf.Lerp(c, d, fx), fy);
        }

        /// fBm (オクターブごとに周期 2 倍・振幅 1/2)。0〜1 付近
        public static float Fbm(float u, float v, int basePeriod, int octaves, int seed)
        {
            float sum = 0f, amp = 0.5f, norm = 0f;
            int p = basePeriod;
            for (int o = 0; o < octaves; o++)
            {
                sum += amp * VNoise(u, v, p, seed + o * 101);
                norm += amp;
                amp *= 0.5f;
                p *= 2;
            }
            return sum / norm;
        }

        // ------------------------------------------------------------ 組み立て
        delegate void PixelFn(float u, float v, int x, int y, out Color albedo, out float height);

        /// albedo とその高さから作った法線マップ (Standard の _BumpMap 用。DXT5nm と同じく x = A・y = G)
        static void Make(int w, int h, PixelFn fn, float bump, out Texture2D albedo, out Texture2D normal, string name)
        {
            var px = new Color32[w * h];
            var hh = new float[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    fn((x + 0.5f) / w, (y + 0.5f) / h, x, y, out Color c, out float ht);
                    px[y * w + x] = c;
                    hh[y * w + x] = ht;
                }
            albedo = new Texture2D(w, h, TextureFormat.RGBA32, true) { name = name, wrapMode = TextureWrapMode.Repeat, anisoLevel = 8 };
            albedo.SetPixels32(px);
            albedo.Apply(true);
            normal = null;
            if (bump <= 0f) return;
            var np = new Color32[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    float dx = hh[y * w + (x + 1) % w] - hh[y * w + (x - 1 + w) % w];
                    float dy = hh[((y + 1) % h) * w + x] - hh[((y - 1 + h) % h) * w + x];
                    Vector3 n = new Vector3(-dx * bump, -dy * bump, 1f).normalized;
                    byte nx = (byte)Mathf.Clamp((n.x * 0.5f + 0.5f) * 255f, 0, 255);
                    byte ny = (byte)Mathf.Clamp((n.y * 0.5f + 0.5f) * 255f, 0, 255);
                    np[y * w + x] = new Color32(255, ny, ny, nx);
                }
            normal = new Texture2D(w, h, TextureFormat.RGBA32, true, true) { name = name + "_N", wrapMode = TextureWrapMode.Repeat, anisoLevel = 8 };
            normal.SetPixels32(np);
            normal.Apply(true);
        }

        /// 材質を作る (Standard)。tile は 1 枚が何 m か (メッシュの uv は m 単位で書く)
        public static Material Material(Material lit, Texture2D albedo, Texture2D normal, float smooth, float bumpScale, Vector2 tile)
        {
            var m = new Material(lit) { color = Color.white, mainTexture = albedo };
            m.mainTextureScale = new Vector2(1f / tile.x, 1f / tile.y);
            m.SetFloat("_Glossiness", smooth);
            m.SetFloat("_Metallic", 0f);
            if (normal != null)
            {
                m.SetTexture("_BumpMap", normal);
                m.SetFloat("_BumpScale", bumpScale);
                m.EnableKeyword("_NORMALMAP");
            }
            return m;
        }

        /// 半透明 (Fade) にする。タイヤ痕など路面に重ねる帯用
        public static void MakeFade(Material m)
        {
            m.SetFloat("_Mode", 2f);
            m.SetOverrideTag("RenderType", "Transparent");
            m.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            m.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            m.SetInt("_ZWrite", 0);
            m.DisableKeyword("_ALPHATEST_ON");
            m.EnableKeyword("_ALPHABLEND_ON");
            m.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            m.renderQueue = 3000;
        }

        // ------------------------------------------------------------ 各材質
        /// アスファルト: 大きなむら (補修跡) ＋ 骨材の粒 (明るい石・暗い穴)。1 枚 6 m を想定
        public static void Asphalt(int size, int seed, float lightness, out Texture2D a, out Texture2D n)
        {
            Make(size, size, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float big = Fbm(u, v, 3, 4, seed);
                float mid = Fbm(u, v, 24, 3, seed + 7);
                float r = Hash(x, y, seed + 13);
                float stone = r > 0.90f ? (r - 0.90f) / 0.10f : 0f;
                float pore = r < 0.06f ? 1f - r / 0.06f : 0f;
                float g = lightness + 0.07f * (big - 0.5f) + 0.05f * (mid - 0.5f) + 0.20f * stone - 0.10f * pore;
                c = new Color(g * 0.98f, g * 0.99f, g * 1.03f, 1f);
                h = 0.6f * mid + 0.8f * stone - 0.8f * pore;
            }, 2.2f, out a, out n, "Asphalt");
        }

        /// 芝: 緑の濃淡 (大小 2 段) ＋ 葉の粒 ＋ 刈り込みの縞 (1 枚に 2 本)。1 枚 24 m を想定
        public static void Grass(int size, int seed, out Texture2D a, out Texture2D n)
        {
            Make(size, size, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float big = Fbm(u, v, 4, 4, seed);
                float mid = Fbm(u, v, 32, 3, seed + 5);
                float blade = Hash(x, y, seed + 9);
                float stripe = (Mathf.FloorToInt(v * 2f) & 1) == 0 ? 1.06f : 0.94f;
                float k = (0.85f + 0.3f * big) * stripe * (0.9f + 0.2f * blade);
                Color dry = new Color(0.42f, 0.44f, 0.22f), lush = new Color(0.20f, 0.36f, 0.14f);
                c = Color.Lerp(lush, dry, Mathf.Clamp01(0.25f + 0.6f * (mid - 0.5f) + 0.4f * (big - 0.5f))) * k;
                c.a = 1f;
                h = 0.5f * blade + 0.5f * mid;
            }, 1.5f, out a, out n, "Grass");
        }

        /// 砂利 (グラベル): 小石の粒と影。1 枚 4 m
        public static void Gravel(int size, int seed, out Texture2D a, out Texture2D n)
        {
            Make(size, size, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float cell = VNoise(u, v, 96, seed);
                float r = Hash(x, y, seed + 3);
                float k = 0.75f + 0.35f * cell + 0.15f * (r - 0.5f);
                c = new Color(0.66f * k, 0.60f * k, 0.50f * k, 1f);
                h = cell + 0.3f * r;
            }, 3.0f, out a, out n, "Gravel");
        }

        /// 縁石: 長さ方向 (v) に赤 3 m・白 3 m (1 枚 6 m)。段差の凹凸と塗装の剥げ
        public static void Kerb(int w, int h, int seed, out Texture2D a, out Texture2D n)
        {
            Make(w, h, (float u, float v, int x, int y, out Color c, out float ht) =>
            {
                bool red = v < 0.5f;
                float wear = Fbm(u, v, 8, 4, seed);
                float grit = Hash(x, y, seed + 1);
                Color baseCol = red ? new Color(0.78f, 0.12f, 0.10f) : new Color(0.92f, 0.91f, 0.88f);
                Color dirt = new Color(0.35f, 0.34f, 0.33f);
                float worn = Mathf.Clamp01((wear - 0.62f) * 4f) * (0.4f + 0.6f * u);
                c = Color.Lerp(baseCol, dirt, worn) * (0.92f + 0.12f * grit);
                c.a = 1f;
                ht = 0.5f + 0.5f * Mathf.Sin(v * Mathf.PI * 2f * 12f) * (1f - u) + 0.2f * grit;
            }, 1.2f, out a, out n, "Kerb");
        }

        /// 白線 (塗装): 少しの汚れと剥げ。1 枚 幅 × 8 m
        public static Texture2D Line(int seed)
        {
            Make(32, 256, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float wear = Fbm(u, v, 16, 3, seed);
                float k = 0.80f + 0.18f * wear + 0.04f * Hash(x, y, seed);
                c = new Color(0.93f * k, 0.93f * k, 0.90f * k, 1f);
                h = 0f;
            }, 0f, out Texture2D a, out Texture2D _, "Line");
            return a;
        }

        /// ガードレール (波形鋼板 2 段・支柱は 4 m ごと)。u = 長さ (1 枚 4 m)、v = 高さ 0〜1
        public static void Armco(int seed, out Texture2D a, out Texture2D n)
        {
            Make(256, 128, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                bool post = u < 0.04f;
                bool rail = v > 0.35f && v < 0.92f;
                float wave = Mathf.Cos((v - 0.35f) / 0.57f * Mathf.PI * 4f);
                float rust = Mathf.Clamp01((Fbm(u, v, 8, 4, seed) - 0.66f) * 5f);
                float g = rail ? 0.62f + 0.18f * wave : 0.32f;
                if (post) g = 0.28f;
                Color steel = new Color(g, g * 1.01f, g * 1.04f);
                c = Color.Lerp(steel, new Color(0.45f, 0.30f, 0.20f), rust * 0.5f);
                c.a = 1f;
                h = rail ? wave : (post ? 1f : 0f);
            }, 2.0f, out a, out n, "Armco");
        }

        /// タイヤ痕 (半透明): 長さ方向に伸びた筋。u = 幅、v = 長さ
        public static Texture2D Rubber(int seed)
        {
            Make(128, 512, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float streak = Fbm(u, v * 0.125f, 32, 3, seed);
                float across = Mathf.Sin(u * Mathf.PI);
                float alpha = Mathf.Clamp01((streak - 0.35f) * 2.2f) * across * across;
                c = new Color(0.05f, 0.05f, 0.05f, alpha * 0.55f);
                h = 0f;
            }, 0f, out Texture2D a, out Texture2D _, "Rubber");
            return a;
        }

        /// 観客席の人 (色の粒)。1 枚 8 m × 8 m
        public static Texture2D Crowd(int seed)
        {
            Make(128, 128, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float r = Hash(x / 2, y / 3, seed), s = Hash(x / 2, y / 3, seed + 1);
                bool empty = s < 0.25f;
                Color shirt = Color.HSVToRGB(r, 0.45f + 0.4f * s, 0.55f + 0.4f * s);
                c = empty ? new Color(0.55f, 0.56f, 0.58f) : shirt;
                c.a = 1f;
                h = 0f;
            }, 0f, out Texture2D a, out Texture2D _, "Crowd");
            a.filterMode = FilterMode.Point;
            return a;
        }
    }
}
