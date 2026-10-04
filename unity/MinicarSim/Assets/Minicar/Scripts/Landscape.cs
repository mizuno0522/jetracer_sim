// 実車スケールのサーキットの周りの景色: 起伏のある地形・森 (絵の木を十字に組んだ板)・遠景の山・雲。
// CourseBuilder.Circuit から呼ぶ。形と色はすべて式とシードで決まり (画像ファイルなし)、tools/preview_circuit.py が
// 同じ式で絵を描く。式を変えたらそちらも合わせる。座標は ROS (x 前・y 左・z 上、m)。
//
//   地形: コースの外 25 m までは平ら (芝の高さ −0.4 m)。そこから 350 m かけて丘 (高さ ±20 m 前後) が立ち上がり、
//         遠く (1.5〜7 km) ほど起伏が大きくなる。富士スピードウェイは富士山の東の裾野にあるので、18 km 先に富士山
//         (コースから 3,150 m 上・裾の半径 22 km・上ほど急な凹んだ斜面・平らな山頂・谷筋に残る雪の筋) を置き、
//         その長い裾野の上りがコースのまわりまで続く。向きは「最終のパナソニックコーナーの向こう」「パドックから
//         100R・ADVAN・300R 越し」に富士山が見える側 (コースの線形が推定なので方位は景色に合わせて決めている)。
//         格子はコースの周り 1.5 km まで 25 m、外へ行くほど粗く (10 km まで最大 150 m、その先 400 m) して ±30 km。
//   地肌: 地形全体に 1 枚の色の地図 (芝生・草地・森の樹冠・山の森→低木→岩→雪) を貼り、芝の細部を 12 m ごとに重ねる。
//   森:   コースの周り 1.5 km の中に、18 m 格子から森の濃さに応じて木を立てる (林の縁は入り組み、草地にも所々一本木)。
//         広葉樹・針葉樹 各 3 種の絵。樹種は場所でまとまる。
//   雲:   半径 40 km の空のドーム (雲の帯の絵)。かすみの影響を受けない材質で描く。
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace Minicar
{
    public class Landscape
    {
        // ---- 定数 (preview_circuit.py の Landscape と同じ) ----
        public const float Inner = 1500f;           // コースの範囲からこの距離までが細かい格子・森
        public const float Outer = 30000f;          // 地形の端 (コースの中心から)
        // 富士山: 山頂まで 18 km・裾野からの高さ 3,200 m。断面は指数の裾 (山頂から 5 km で半分・10 km で 1/4) で、上ほど急 (25°前後)
        public const float MountainDist = 18000f, MountainH = 3200f, MountainR = 30000f, MountainL = 6500f;
        public static readonly Vector2 FujiDir = new Vector2(-0.25f, -0.97f).normalized;   // コースの中心 → 山頂
        public const float TreeGrid = 18f;
        public const float SkyR = 40000f;           // 雲のドーム (富士山 18 km より遠く)

        public readonly float[] Xs, Ys;             // 格子線 (ROS x, y)
        public readonly float[] H, D;               // 高さ・コースの中心線からの距離 [j * nx + i]
        public readonly Vector2 Mid, Peak, Hoei, HoeiCrater;
        public readonly float Bar;
        readonly float[] m_B;                       // コースの範囲
        readonly Vector2 m_Start;                   // コントロールライン
        readonly float m_MidMountain;               // コースの中心での山の高さ (コースは平らに置くので差し引く)

        public Landscape(Vector2[] center, float bar, float[] bounds)
        {
            Bar = bar;
            m_B = bounds;
            m_Start = center[0];
            Mid = new Vector2((bounds[0] + bounds[2]) * 0.5f, (bounds[1] + bounds[3]) * 0.5f);
            Peak = Mid + FujiDir * MountainDist;
            // 宝永山 (南東の中腹の火口と盛り上がり)。東の御殿場・小山から見ると左 (南) の肩のこぶ。見る向きの左へ、少し手前に
            Vector2 left = new Vector2(-FujiDir.y, FujiDir.x);      // 富士山を見る向きの左 (東から見て南)
            Hoei = Peak + left * 3300f;                                 // 真横 = 稜線に出る
            HoeiCrater = Hoei + (Hoei - Peak).normalized * 450f;
            m_MidMountain = Mountain(Mid.x, Mid.y, out _);
            Xs = Axis(bounds[0], bounds[2], Mid.x);
            Ys = Axis(bounds[1], bounds[3], Mid.y);
            int nx = Xs.Length, ny = Ys.Length;
            H = new float[nx * ny];
            D = new float[nx * ny];
            // 中心線は 2 点おき (4 m) で距離を測る。コースの範囲から 400 m 以上離れた所は範囲までの距離で足りる
            var pts = new List<Vector2>();
            for (int k = 0; k < center.Length; k += 2) pts.Add(center[k]);
            var P = pts.ToArray();
            Parallel.For(0, ny, j =>
            {
                for (int i = 0; i < nx; i++)
                {
                    float x = Xs[i], y = Ys[j];
                    float bx = Mathf.Max(0f, Mathf.Max(bounds[0] - x, x - bounds[2]));
                    float by = Mathf.Max(0f, Mathf.Max(bounds[1] - y, y - bounds[3]));
                    float d = Mathf.Sqrt(bx * bx + by * by);
                    if (d < 400f)
                    {
                        float best = float.MaxValue;
                        for (int k = 0; k < P.Length; k++)
                        {
                            float dx = P[k].x - x, dy = P[k].y - y;
                            float q = dx * dx + dy * dy;
                            if (q < best) best = q;
                        }
                        d = Mathf.Sqrt(best);
                    }
                    D[j * nx + i] = d;
                    H[j * nx + i] = Height(x, y, d);
                }
            });
        }

        /// 格子線: コースの範囲 ± Inner は 25 m、その外は 1.08 倍ずつ粗く (中心から 10 km まで最大 150 m、その先 400 m) して中心 ± Outer まで
        static float[] Axis(float lo, float hi, float mid)
        {
            var a = new List<float>();
            float x0 = lo - Inner, x1 = hi + Inner;
            int n = Mathf.CeilToInt((x1 - x0) / 25f);
            for (int k = 0; k <= n; k++) a.Add(x0 + k * 25f);
            float last = a[a.Count - 1], s = 25f;
            while (last < mid + Outer) { s = Mathf.Min(last - mid < 10000f ? 150f : 400f, s * 1.08f); last += s; a.Add(last); }
            float first = a[0];
            s = 25f;
            var lead = new List<float>();
            while (first > mid - Outer) { s = Mathf.Min(mid - first < 10000f ? 150f : 400f, s * 1.08f); first -= s; lead.Add(first); }
            lead.Reverse();
            lead.AddRange(a);
            return lead.ToArray();
        }

        // ------------------------------------------------------------ 形
        /// 富士山の高さ (裾野 = 0) と谷筋のノイズ g (0〜1)
        public float Mountain(float x, float y, out float g)
        {
            float dx = x - Peak.x, dy = y - Peak.y;
            float r = Mathf.Sqrt(dx * dx + dy * dy);
            g = 0.5f;
            if (r >= MountainR) return 0f;
            float u = Mathf.Atan2(dy, dx) / (2f * Mathf.PI) + 0.5f;
            g = ProcTex.Fbm(u, r / MountainR * 0.5f, 90, 3, 311);              // 放射状の谷筋 (雪の筋になる)
            float re = Mathf.Max(r, 350f);                                     // 山頂は半径 350 m ほど平ら (火口の縁)
            float e0 = Mathf.Exp(-MountainR / MountainL);
            float skirt = (Mathf.Exp(-re / MountainL) - e0) / (Mathf.Exp(-350f / MountainL) - e0);
            float h = MountainH * skirt * (1f + 0.18f * (g - 0.5f) * Mathf.Min(1f, r / 2500f));   // 谷筋は浅く (富士山の肌はなめらか)
            // 宝永山: 盛り上がり (+330 m・半径 1.1 km) と、その下側の火口 (−380 m・半径 600 m)
            float hx = x - Hoei.x, hy = y - Hoei.y, cx = x - HoeiCrater.x, cy = y - HoeiCrater.y;
            h += 330f * Mathf.Exp(-(hx * hx + hy * hy) / (1100f * 1100f)) - 380f * Mathf.Exp(-(cx * cx + cy * cy) / (600f * 600f));
            return Mathf.Max(0f, h);
        }

        public float Height(float x, float y, float d)
        {
            float ramp = ProcTex.Smooth(Bar + 25f, Bar + 350f, d);
            float mx = x - Mid.x, my = y - Mid.y;
            float dc = Mathf.Sqrt(mx * mx + my * my);
            float amp = 1f + 0.8f * ProcTex.Smooth(1500f, 7000f, dc);   // 御殿場・小山の裾野はなだらか
            float hill = 34f * (ProcTex.FbmW(x, y, 110f, 4, 301) - 0.45f) + 10f * (ProcTex.FbmW(x, y, 30f, 3, 307) - 0.5f);
            // 富士山の裾野の上り (コースの中心で 0)。コースのそば (ramp = 0) は平ら
            float mtn = Mountain(x, y, out _);
            float foot = mtn - m_MidMountain;
            hill *= 1f - ProcTex.Smooth(300f, 1200f, mtn);                  // 富士山の斜面はなめらか (丘の起伏を消す)
            return -0.4f + ramp * (hill * amp + foot);
        }

        /// 森の濃さ 0〜1 (コースのそばは 0)
        public float Forest(float x, float y, float d)
        {
            float f = ProcTex.Smooth(0.40f, 0.56f, ProcTex.FbmW(x, y, 400f, 4, 321));
            return f * ProcTex.Smooth(Bar + 18f, Bar + 34f, d);
        }

        /// 格子の値をバイリニアで (nonuniform 格子)
        public float Sample(float[] f, float x, float y)
        {
            int i = Find(Xs, x), j = Find(Ys, y), nx = Xs.Length;
            float fx = Mathf.Clamp01((x - Xs[i]) / (Xs[i + 1] - Xs[i])), fy = Mathf.Clamp01((y - Ys[j]) / (Ys[j + 1] - Ys[j]));
            float a = f[j * nx + i], b = f[j * nx + i + 1], c = f[(j + 1) * nx + i], e = f[(j + 1) * nx + i + 1];
            return Mathf.Lerp(Mathf.Lerp(a, b, fx), Mathf.Lerp(c, e, fx), fy);
        }

        static int Find(float[] a, float x)
        {
            int k = Array.BinarySearch(a, x);
            if (k < 0) k = ~k - 1;
            return Mathf.Clamp(k, 0, a.Length - 2);
        }

        // ------------------------------------------------------------ 色
        static readonly Color Lawn = new Color(0.27f, 0.38f, 0.17f), MeadowA = new Color(0.30f, 0.41f, 0.18f), MeadowB = new Color(0.46f, 0.46f, 0.26f);
        static readonly Color Canopy = new Color(0.12f, 0.19f, 0.11f), Scrub = new Color(0.34f, 0.31f, 0.22f);
        static readonly Color Rock = new Color(0.30f, 0.24f, 0.23f), Snow = new Color(0.94f, 0.95f, 0.98f);   // 赤みがかった黒い火山の岩

        /// 地図の 1 点の色。h = 高さ、d = コースからの距離
        public Color Ground(float x, float y, float h, float d)
        {
            float m = Mountain(x, y, out float g);
            float tone = ProcTex.FbmW(x, y, 60f, 3, 331);
            Color meadow = Color.Lerp(MeadowA, MeadowB, ProcTex.Smooth(0.35f, 0.75f, tone));
            Color canopy = Canopy * (0.75f + 0.5f * ProcTex.FbmW(x, y, 14f, 3, 333));
            float forest = Forest(x, y, d);
            // 富士山: 中腹は森 (樹林帯)、森林限界 (山の高さ 1,900 m ≒ 標高 2,500 m) を越えると低木 → 黒っぽい火山の岩 → 雪。雪の境目は谷筋で下がる
            forest = Mathf.Max(forest, ProcTex.Smooth(150f, 500f, m) * 0.9f) * (1f - ProcTex.Smooth(1700f, 1900f, m));   // 裾野から樹林帯
            Color c = Color.Lerp(meadow, canopy, forest);
            c = Color.Lerp(Lawn, c, ProcTex.Smooth(Bar + 10f, Bar + 30f, d));
            float n = ProcTex.FbmW(x, y, 80f, 3, 337);
            c = Color.Lerp(c, Scrub, ProcTex.Smooth(1700f, 1900f, m + 120f * (n - 0.5f)));
            c = Color.Lerp(c, Rock * (0.8f + 0.4f * g), ProcTex.Smooth(1900f, 2100f, m + 120f * (n - 0.5f)));
            // 雪: 標高 2,200 m 前後 (裾野から 1,650 m) まで。谷筋に沿って下へ伸び、尾根は岩が出る (春の富士山の筋)
            float snowLine = 1650f - 1300f * (g - 0.5f) + 120f * (n - 0.5f);
            c = Color.Lerp(c, Snow, ProcTex.Smooth(snowLine, snowLine + 40f, m));
            c *= Mathf.Lerp(1f, 0.80f + 0.4f * g, ProcTex.Smooth(400f, 900f, m));     // 谷筋は暗く、尾根は明るく
            c.a = 1f;
            return c;
        }

        /// 地形全体の色の地図 (size × size。u = x、v = y の範囲を 1 枚)
        public Texture2D ColorMap(int size)
        {
            var px = new Color32[size * size];
            float x0 = Xs[0], x1 = Xs[Xs.Length - 1], y0 = Ys[0], y1 = Ys[Ys.Length - 1];
            Parallel.For(0, size, j =>
            {
                float y = y0 + (j + 0.5f) / size * (y1 - y0);
                for (int i = 0; i < size; i++)
                {
                    float x = x0 + (i + 0.5f) / size * (x1 - x0);
                    px[j * size + i] = Ground(x, y, Sample(H, x, y), Sample(D, x, y));
                }
            });
            var t = new Texture2D(size, size, TextureFormat.RGBA32, true) { name = "LandMap", wrapMode = TextureWrapMode.Clamp, anisoLevel = 8 };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }

        // ------------------------------------------------------------ メッシュ
        /// 地形のメッシュ。uv0 = 色の地図 (0〜1)、uv1 = m 単位 (芝の細部)
        public Mesh TerrainMesh()
        {
            int nx = Xs.Length, ny = Ys.Length;
            var v = new Vector3[nx * ny];
            var uv0 = new Vector2[nx * ny];
            var uv1 = new Vector2[nx * ny];
            float x0 = Xs[0], x1 = Xs[nx - 1], y0 = Ys[0], y1 = Ys[ny - 1];
            for (int j = 0; j < ny; j++)
                for (int i = 0; i < nx; i++)
                {
                    int k = j * nx + i;
                    v[k] = RosFrame.ToUnity(Xs[i], Ys[j], H[k]);
                    uv0[k] = new Vector2((Xs[i] - x0) / (x1 - x0), (Ys[j] - y0) / (y1 - y0));
                    uv1[k] = new Vector2(Xs[i], Ys[j]);
                }
            // 表が上を向く巻き順を最初の三角形で決める (Unity は cross(b − a, c − a) が表)
            bool flip = Vector3.Cross(v[1] - v[0], v[nx] - v[0]).y < 0f;
            var tri = new int[(nx - 1) * (ny - 1) * 6];
            int t = 0;
            for (int j = 0; j < ny - 1; j++)
                for (int i = 0; i < nx - 1; i++)
                {
                    int a = j * nx + i, b = a + 1, c = a + nx, e = c + 1;
                    if (!flip) { tri[t++] = a; tri[t++] = b; tri[t++] = c; tri[t++] = b; tri[t++] = e; tri[t++] = c; }
                    else { tri[t++] = a; tri[t++] = c; tri[t++] = b; tri[t++] = b; tri[t++] = c; tri[t++] = e; }
                }
            var m = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32, name = "Terrain" };
            m.vertices = v;
            m.uv = uv0;
            m.uv2 = uv1;
            m.triangles = tri;
            m.RecalculateNormals();
            m.RecalculateTangents();
            m.RecalculateBounds();
            return m;
        }

        public struct TreeSpot { public Vector2 P; public float Z, Height, Width, Yaw; public int Sprite; }

        /// 木の位置。sprite 0〜2 = 広葉樹、3〜5 = 針葉樹
        public List<TreeSpot> Trees()
        {
            var list = new List<TreeSpot>();
            float x0 = m_B[0] - Inner, y0 = m_B[1] - Inner, x1 = m_B[2] + Inner, y1 = m_B[3] + Inner;
            int nx = Mathf.CeilToInt((x1 - x0) / TreeGrid), ny = Mathf.CeilToInt((y1 - y0) / TreeGrid);
            for (int j = 0; j < ny; j++)
                for (int i = 0; i < nx; i++)
                {
                    float x = x0 + (i + 0.5f + 0.8f * (ProcTex.Hash(i, j, 401) - 0.5f)) * TreeGrid;
                    float y = y0 + (j + 0.5f + 0.8f * (ProcTex.Hash(i, j, 402) - 0.5f)) * TreeGrid;
                    float d = Sample(D, x, y);
                    if (d < Bar + 22f + 25f * ProcTex.Hash(i, j, 403)) continue;          // コースのそばは芝生 (縁は不揃い)
                    if ((new Vector2(x, y) - m_Start).sqrMagnitude < 380f * 380f) continue;   // 観客席・ピットのまわり
                    float f = Forest(x, y, d);
                    float r = ProcTex.Hash(i, j, 404);
                    if (r >= Mathf.Max(f * 0.92f, 0.015f)) continue;                     // 森の濃さ + 草地の一本木
                    float conif = ProcTex.Smooth(0.38f, 0.62f, ProcTex.FbmW(x, y, 600f, 2, 341));
                    bool c = ProcTex.Hash(i, j, 405) < conif;
                    float s = ProcTex.Hash(i, j, 406);
                    float h = c ? 12f + 10f * s : 9f + 8f * s;
                    if (f < 0.3f) h *= 1.15f;                                              // 一本木は枝を広げて大きめ
                    list.Add(new TreeSpot
                    {
                        P = new Vector2(x, y),
                        Z = Sample(H, x, y) - 0.5f,
                        Height = h,
                        Width = h * 0.5f * (0.9f + 0.2f * ProcTex.Hash(i, j, 407)),
                        Yaw = ProcTex.Hash(i, j, 408) * Mathf.PI,
                        Sprite = (c ? 3 : 0) + Mathf.Min(2, (int)(ProcTex.Hash(i, j, 409) * 3f)),
                    });
                }
            return list;
        }

        /// 木を 500 m 四方・絵ごとのメッシュにまとめる (見えない区画は描かない)。十字の板 2 枚、両面
        public static Dictionary<(int, int, int), Mesh> TreeMeshes(List<TreeSpot> trees)
        {
            var v = new Dictionary<(int, int, int), (List<Vector3> v, List<Vector2> uv, List<int> t)>();
            foreach (var s in trees)
            {
                var key = (Mathf.FloorToInt(s.P.x / 500f), Mathf.FloorToInt(s.P.y / 500f), s.Sprite);
                if (!v.TryGetValue(key, out var b)) { b = (new List<Vector3>(), new List<Vector2>(), new List<int>()); v[key] = b; }
                for (int q = 0; q < 2; q++)
                {
                    float a = s.Yaw + q * Mathf.PI * 0.5f;
                    float cx = Mathf.Cos(a) * s.Width * 0.5f, cy = Mathf.Sin(a) * s.Width * 0.5f;
                    int k = b.v.Count;
                    b.v.Add(RosFrame.ToUnity(s.P.x - cx, s.P.y - cy, s.Z));
                    b.v.Add(RosFrame.ToUnity(s.P.x + cx, s.P.y + cy, s.Z));
                    b.v.Add(RosFrame.ToUnity(s.P.x + cx, s.P.y + cy, s.Z + s.Height));
                    b.v.Add(RosFrame.ToUnity(s.P.x - cx, s.P.y - cy, s.Z + s.Height));
                    b.uv.Add(new Vector2(0f, 0f)); b.uv.Add(new Vector2(1f, 0f)); b.uv.Add(new Vector2(1f, 1f)); b.uv.Add(new Vector2(0f, 1f));
                    b.t.AddRange(new[] { k, k + 1, k + 2, k, k + 2, k + 3, k, k + 2, k + 1, k, k + 3, k + 2 });
                }
            }
            var o = new Dictionary<(int, int, int), Mesh>();
            foreach (var kv in v)
            {
                var m = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32, name = "Trees" };
                m.SetVertices(kv.Value.v);
                m.SetUVs(0, kv.Value.uv);
                m.SetTriangles(kv.Value.t, 0);
                var nr = new Vector3[kv.Value.v.Count];
                for (int i = 0; i < nr.Length; i++) nr[i] = Vector3.up;      // 陰影は絵に描いてあるので、光は一様に受ける
                m.normals = nr;
                m.RecalculateBounds();
                o[kv.Key] = m;
            }
            return o;
        }

        /// 空のドーム (内側が表)。u = 方位、v = 仰角 / 90°
        public Mesh SkyDome()
        {
            const int na = 64, ne = 16;
            var v = new List<Vector3>();
            var uv = new List<Vector2>();
            var col = new List<Color32>();
            var t = new List<int>();
            for (int e = 0; e <= ne; e++)
                for (int a = 0; a <= na; a++)
                {
                    float el = e / (float)ne * Mathf.PI * 0.5f * 0.98f, az = a / (float)na * Mathf.PI * 2f;
                    v.Add(RosFrame.ToUnity(Mid.x + Mathf.Cos(az) * Mathf.Cos(el) * SkyR, Mid.y + Mathf.Sin(az) * Mathf.Cos(el) * SkyR, Mathf.Sin(el) * SkyR - 200f));
                    uv.Add(new Vector2(a / (float)na, el / (Mathf.PI * 0.5f)));
                    col.Add(new Color32(255, 255, 255, 255));
                }
            for (int e = 0; e < ne; e++)
                for (int a = 0; a < na; a++)
                {
                    int p = e * (na + 1) + a, q = p + na + 1;
                    t.AddRange(new[] { p, q, p + 1, p + 1, q, q + 1 });
                }
            var m = new Mesh { name = "SkyDome" };
            m.SetVertices(v);
            m.SetUVs(0, uv);
            m.SetColors(col);
            m.SetTriangles(t, 0);
            m.RecalculateBounds();
            return m;
        }
    }
}
