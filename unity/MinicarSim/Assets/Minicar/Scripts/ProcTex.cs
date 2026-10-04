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

        /// 世界座標のノイズ (scale [m] が 1 格子。周期 256 格子なので繰り返しは見えない)
        public static float FbmW(float x, float y, float scale, int octaves, int seed)
        {
            const int P = 256;
            float u = x / (scale * P), v = y / (scale * P);
            u -= Mathf.Floor(u); v -= Mathf.Floor(v);
            return Fbm(u, v, P, octaves, seed);
        }

        public static float Smooth(float a, float b, float x)
        {
            float t = Mathf.Clamp01((x - a) / (b - a));
            return t * t * (3f - 2f * t);
        }

        /// 木の絵 (透過つき・256×512、画素単位で 1 本の高さ = 512)。kind 0 = 広葉樹、1 = 針葉樹。
        /// 枝と数千の小さな葉の固まりを 3D に置き、奥から順に描く。葉は向きと樹冠の奥行で陰影 (左上の光・内側と下ほど暗い)。
        /// 広葉樹: 太い幹 → 枝 → 大小 7 つの膨らみ (楕円体) の表面近くに葉。針葉樹: 細い幹から段ごとに 5〜7 本の
        /// 垂れた枝、枝に沿って針葉の房 (上ほど枝が短い円錐)。乱数はハッシュ (preview_circuit.py と同じ並び)
        public static Texture2D Tree(int kind, int seed)
        {
            const int W = 256, H = 512;
            int ri = 0;
            float R() => Hash(ri++, kind, seed);
            Vector3 L = new Vector3(-0.55f, 0.62f, 0.56f).normalized;
            Color leafA = kind == 0 ? new Color(0.20f, 0.30f, 0.10f) : new Color(0.10f, 0.20f, 0.12f);
            Color leafB = kind == 0 ? new Color(0.44f, 0.52f, 0.20f) : new Color(0.20f, 0.32f, 0.17f);
            var px = new Color[W * H];
            Color clear = new Color(leafA.r * 0.6f, leafA.g * 0.6f, leafA.b * 0.6f, 0f);  // 縮小で縁が黒くにじまないように
            for (int i = 0; i < px.Length; i++) px[i] = clear;
            float hue = (R() - 0.5f) * 0.25f;
            var items = new System.Collections.Generic.List<(float z, int type, float a, float b, float c, float d, float e, float f, Color col)>();
            Color Bark(float k) => new Color(0.30f * k, 0.24f * k, 0.18f * k, 1f);
            float top = kind == 0 ? 0.45f * H : 0.95f * H;
            float w0 = kind == 0 ? 15f : 9f, w1 = kind == 0 ? 7f : 1.5f;
            items.Add((0f, 1, 128f, 0f, 128f + (R() - 0.5f) * 8f, top, w0, w1, Bark(0.9f)));
            if (kind == 0)
            {
                const int nl = 6;
                var lobes = new System.Collections.Generic.List<float[]> { new[] { 128f, 0.58f * H, 0f, 100f, 125f, 100f } };
                for (int i = 0; i < nl; i++)
                {
                    float a = R() * Mathf.PI * 2f;
                    float cx = 128f + Mathf.Cos(a) * (40f + 30f * R());
                    float cy = 0.56f * H + (R() - 0.45f) * 170f;
                    float cz = Mathf.Sin(a) * 45f;
                    float rx = 45f + 25f * R(), ry = 45f + 25f * R(), rz = 45f + 25f * R();
                    lobes.Add(new[] { cx, cy, cz, rx, ry, rz });
                }
                for (int i = 1; i < lobes.Count; i++)          // 枝
                {
                    float y0 = 0.28f * H + R() * 0.14f * H;
                    items.Add((lobes[i][2] * 0.3f, 1, 128f, y0, lobes[i][0], lobes[i][1], 6f, 1.5f, Bark(0.7f)));
                }
                for (int i = 0; i < 5200; i++)
                {
                    int j = (int)(R() * (nl + 4));
                    if (j > nl) j = 0;
                    var lb = lobes[j];
                    float th = R() * Mathf.PI * 2f, ph = Mathf.Acos(2f * R() - 1f);
                    float dx = Mathf.Sin(ph) * Mathf.Cos(th), dy = Mathf.Cos(ph), dz = Mathf.Sin(ph) * Mathf.Sin(th);
                    float rr = 0.55f + 0.45f * Mathf.Sqrt(R());
                    float x = lb[0] + dx * lb[3] * rr, y = lb[1] + dy * lb[4] * rr, z = lb[2] + dz * lb[5] * rr;
                    float nx = dx + (R() - 0.5f) * 0.8f, ny = dy + (R() - 0.5f) * 0.8f, nz = dz + (R() - 0.5f) * 0.8f;
                    float nn = Mathf.Sqrt(nx * nx + ny * ny + nz * nz);
                    float dif = Mathf.Max(0f, (nx * L.x + ny * L.y + nz * L.z) / nn);
                    float ao = (0.45f + 0.55f * rr * rr) * (0.72f + 0.28f * Mathf.Clamp01((y - 0.30f * H) / (0.6f * H)));
                    float k = (0.36f + 0.85f * dif) * ao * (0.85f + 0.3f * R());
                    Color c = Color.Lerp(leafA, leafB, Mathf.Clamp01(R() * 0.6f + dif * 0.5f + hue)) * k;
                    items.Add((z, 0, x, y, 2.2f + 2.6f * R(), 0f, 0f, 0f, c));
                }
            }
            else
            {
                float y0 = 0.10f * H, yt = 0.97f * H, y = y0;
                while (y < yt)
                {
                    float f = (yt - y) / (yt - y0);
                    float Lb = Mathf.Pow(f, 0.9f) * 96f + 6f;
                    int nb = 5 + (int)(R() * 3f);
                    float a0 = R() * Mathf.PI * 2f;
                    for (int b = 0; b < nb; b++)
                    {
                        float a = a0 + b * 2f * Mathf.PI / nb + (R() - 0.5f) * 0.6f;
                        float lb = Lb * (0.8f + 0.35f * R());
                        float droop = 0.30f * lb;
                        float ex = Mathf.Cos(a) * lb, ez = Mathf.Sin(a) * lb, ey = y - droop;
                        items.Add((ez * 0.5f, 1, 128f, y, 128f + ex, ey, 2.2f, 0.8f, Bark(0.55f)));
                        int m = (int)(lb / 2.2f) + 3;
                        for (int j = 0; j < m; j++)
                        {
                            float t = 0.15f + 0.85f * R();
                            float sx = (R() - 0.5f) * (4f + 9f * t);
                            float px0 = 128f + ex * t - Mathf.Sin(a) * sx;
                            float pz = ez * t + Mathf.Cos(a) * sx;
                            float py = y - droop * t * t + (R() - 0.3f) * 6f;
                            float nx = Mathf.Cos(a), ny = 0.45f, nz = Mathf.Sin(a);
                            float nn = Mathf.Sqrt(nx * nx + ny * ny + nz * nz);
                            float dif = Mathf.Max(0f, (nx * L.x + ny * L.y + nz * L.z) / nn);
                            float ao = (0.35f + 0.65f * t) * (0.75f + 0.25f * (1f - f));
                            float k = (0.30f + 0.90f * dif) * ao * (0.85f + 0.3f * R());
                            Color c = Color.Lerp(leafA, leafB, Mathf.Clamp01(R() * 0.5f + dif * 0.4f + hue)) * k;
                            items.Add((pz, 0, px0, py, 2.0f + 2.2f * R(), 0f, 0f, 0f, c));
                        }
                    }
                    y += 9f + 5f * R();
                }
            }
            items.Sort((p, q) => p.z.CompareTo(q.z));          // 奥から
            foreach (var it in items)
            {
                if (it.type == 0)
                {
                    float x = it.a, y = it.b, r = it.c;
                    int x0 = Mathf.Max(0, (int)(x - r)), x1 = Mathf.Min(W, (int)(x + r) + 1);
                    int ya = Mathf.Max(0, (int)(y - r)), yb = Mathf.Min(H, (int)(y + r) + 1);
                    Color c = it.col; c.a = 1f;
                    for (int yy = ya; yy < yb; yy++)
                        for (int xx = x0; xx < x1; xx++)
                        {
                            float ddx = xx + 0.5f - x, ddy = yy + 0.5f - y;
                            if (ddx * ddx + ddy * ddy <= r * r) px[yy * W + xx] = c;
                        }
                }
                else
                {
                    float ax = it.a, ay = it.b, bx = it.c, by = it.d, wa = it.e, wb = it.f;
                    int x0 = Mathf.Max(0, (int)(Mathf.Min(ax, bx) - wa)), x1 = Mathf.Min(W, (int)(Mathf.Max(ax, bx) + wa) + 1);
                    int ya = Mathf.Max(0, (int)(Mathf.Min(ay, by) - wa)), yb = Mathf.Min(H, (int)(Mathf.Max(ay, by) + wa) + 1);
                    float vx = bx - ax, vy = by - ay, ll = vx * vx + vy * vy;
                    for (int yy = ya; yy < yb; yy++)
                        for (int xx = x0; xx < x1; xx++)
                        {
                            float X = xx + 0.5f, Y = yy + 0.5f;
                            float t = Mathf.Clamp01(((X - ax) * vx + (Y - ay) * vy) / ll);
                            float ox = X - ax - t * vx, oy = Y - ay - t * vy;
                            if (Mathf.Sqrt(ox * ox + oy * oy) > (wa + (wb - wa) * t) * 0.5f) continue;
                            float sh = Mathf.Clamp(0.8f + 0.4f * ox / Mathf.Max(1f, wa), 0.5f, 1.3f);   // 幹の丸み
                            px[yy * W + xx] = new Color(it.col.r * sh, it.col.g * sh, it.col.b * sh, 1f);
                        }
                }
            }
            var t2 = new Texture2D(W, H, TextureFormat.RGBA32, true) { name = "Tree" + kind, wrapMode = TextureWrapMode.Clamp, anisoLevel = 4 };
            t2.SetPixels(px);
            t2.Apply(true);
            return t2;
        }

        /// 透明 (Transparent・乗算済みアルファ) にする。色は透けて、映り込みとハイライトだけが残る (車のクリア層用)
        public static void MakeTransparent(Material m)
        {
            m.SetFloat("_Mode", 3f);
            m.SetOverrideTag("RenderType", "Transparent");
            m.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.One);
            m.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            m.SetInt("_ZWrite", 0);
            m.DisableKeyword("_ALPHATEST_ON");
            m.DisableKeyword("_ALPHABLEND_ON");
            m.EnableKeyword("_ALPHAPREMULTIPLY_ON");
            m.renderQueue = 3000;
        }

        /// 切り抜き (Cutout) にする。木の絵のように透明の部分を抜き、影も絵の形で落とす
        public static void MakeCutout(Material m, float cutoff)
        {
            m.SetFloat("_Mode", 1f);
            m.SetOverrideTag("RenderType", "TransparentCutout");
            m.SetFloat("_Cutoff", cutoff);
            m.EnableKeyword("_ALPHATEST_ON");
            m.DisableKeyword("_ALPHABLEND_ON");
            m.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            m.renderQueue = 2450;
        }

        /// 雲の帯 (空のドームに巻く 1024×256。u = 方位、v = 仰角 0〜1)。白〜灰の積雲、上ほど薄い
        public static Texture2D Clouds(int seed)
        {
            const int W = 1024, H = 256;
            var px = new Color32[W * H];
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++)
                {
                    float u = (x + 0.5f) / W, v = (y + 0.5f) / H;
                    float f = Fbm(u, v * 0.35f, 12, 6, seed);
                    float cover = Mathf.Clamp01((f - 0.50f) * 4.5f) * Smooth(0f, 0.12f, v) * (1f - Smooth(0.55f, 1f, v));
                    // 雲の底は暗く、上の縁は明るい (下の方を少しずらしたノイズで測る)
                    float below = Fbm(u, v * 0.35f - 0.012f, 12, 6, seed);
                    float shade = 0.74f + 0.30f * Mathf.Clamp01((f - below) * 6f + 0.5f) + 0.06f * Fbm(u, v, 64, 2, seed + 3);
                    px[y * W + x] = new Color(shade, shade * 1.0f, shade * 1.02f, cover * 0.9f);
                }
            var t = new Texture2D(W, H, TextureFormat.RGBA32, true) { name = "Clouds", wrapMode = TextureWrapMode.Repeat };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }

        /// 芝の細部 (灰色。地形の色に ×2 で掛ける Standard の detail 用。平均 ≈ 0.5)。1 枚 12 m
        public static void GrassDetail(int size, int seed, out Texture2D a, out Texture2D n)
        {
            Make(size, size, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float big = Fbm(u, v, 4, 4, seed);
                float mid = Fbm(u, v, 32, 3, seed + 5);
                float blade = Hash(x, y, seed + 9);
                float g = 0.5f * (0.70f + 0.30f * big + 0.24f * mid + 0.20f * blade);
                c = new Color(g, g, g, 1f);
                h = 0.5f * blade + 0.5f * mid;
            }, 1.5f, out a, out n, "GrassDetail");
        }

        /// 平らな法線マップ (detail の法線だけを効かせたいときの主の法線)
        public static Texture2D FlatNormal()
        {
            var t = new Texture2D(4, 4, TextureFormat.RGBA32, false, true) { name = "FlatN" };
            var px = new Color32[16];
            for (int i = 0; i < 16; i++) px[i] = new Color32(255, 128, 128, 128);
            t.SetPixels32(px);
            t.Apply();
            return t;
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
