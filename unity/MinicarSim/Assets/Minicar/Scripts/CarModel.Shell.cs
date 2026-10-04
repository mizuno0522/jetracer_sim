// 実車の車体を 1 枚の曲面 (シェル) で作る。RX-7 (FD3S) から使う。
//
// 作り方:
//   - 断面 (Sec) を前後に並べる。数値は実車の m (z = 後軸から前へ、y = 地面から、幅は半幅)。模型の大きさへは Scale で縮める
//   - 半断面は制御点 9 個の 2 次 B スプライン (折れ線の角を丸めた曲線。制御点を近づけると角が立つ):
//       c0 底の中央 → c1 底 → c2 底の外 → c3 下の側面 → c4 側面の最大幅 → c5 肩 → c6 窓の付け根 → c7 屋根の縁 → c8 屋根の中央
//     車軸の前後 (ArchR 以内) では c1〜c4 をホイールハウス (内壁・天井・アーチの縁) に置き換える
//   - 前後方向は断面の値を単調な 3 次補間でつなぎ、端は断面を中心へすぼめて丸く閉じる
//   - 窓・パネルの合わせ目・ランプ・黒い樹脂は、曲面に貼る 1 枚の絵 (色 + 金属感・滑らかさ) に描く (Paint)。
//     絵は実車の座標 (z, x, y) と断面上の位置 u (0〜7) で塗り分けるので、形を変えても場所がずれない
// tools/preview_circuit.py が同じ式で描く (断面の表は正規表現で読む。St(…) の書き方を変えたら合わせる)
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public partial class CarModel
    {
        /// 断面 1 つ (実車の m)
        static float[] St(float z, float yb, float wb, float w2, float y2, float w3, float y3, float w4, float y4,
                          float w5, float y5, float w6, float y6, float y7) =>
            new[] { z, yb, wb, w2, y2, w3, y3, w4, y4, w5, y5, w6, y6, y7 };

        delegate void ShellPaint(float z, float x, float y, float u, out Color32 albedo, out float metal, out float smooth);

        class Shell
        {
            public const int K = 6;                 // 1 区間の分割
            public const int Segs = 7;              // 2 次ベジェの本数 (制御点 9 個)
            public const int NU = Segs * K + 1;     // 半断面の点の数
            public float Scale;                     // 実車 m → 模型
            public float[] Axles;                   // 車軸の z [m]
            public float ArchR, ArchY, WellX;       // アーチの半径・中心の高さ (= タイヤの半径)・ホイールハウスの内壁
            readonly float[][] m_S;                 // 断面
            readonly float[][] m_T;                 // 補間の傾き
            // 輪 (断面を置く位置): 評価する z・実際の z・中心へのすぼめ (1 = そのまま)・絵の横位置 (0〜1)
            public readonly List<float> RingZ = new List<float>(), RingPos = new List<float>(), RingF = new List<float>(), RingU = new List<float>();

            public Shell(float[][] secs, float scale, float[] axles, float archR, float archY, float wellX)
            {
                m_S = secs; Scale = scale; Axles = axles; ArchR = archR; ArchY = archY; WellX = wellX;
                int n = secs.Length, np = secs[0].Length;
                m_T = new float[n][];
                for (int k = 0; k < n; k++) m_T[k] = new float[np];
                // 単調な 3 次補間 (Fritsch–Carlson): 行き過ぎない
                for (int i = 1; i < np; i++)
                {
                    var d = new float[n - 1];
                    for (int k = 0; k < n - 1; k++) d[k] = (secs[k + 1][i] - secs[k][i]) / (secs[k + 1][0] - secs[k][0]);
                    m_T[0][i] = d[0]; m_T[n - 1][i] = d[n - 2];
                    for (int k = 1; k < n - 1; k++)
                        m_T[k][i] = d[k - 1] * d[k] <= 0f ? 0f : 2f * d[k - 1] * d[k] / (d[k - 1] + d[k]);
                }
                BuildRings();
            }

            float Val(int k, float t, float h, int i)
            {
                float t2 = t * t, t3 = t2 * t;
                return (2 * t3 - 3 * t2 + 1) * m_S[k][i] + (t3 - 2 * t2 + t) * h * m_T[k][i]
                     + (-2 * t3 + 3 * t2) * m_S[k + 1][i] + (t3 - t2) * h * m_T[k + 1][i];
            }

            /// z での制御点 9 個 (x = 半幅, y = 高さ)。f < 1 なら断面を中心へすぼめる (前後の端を丸く閉じる)
            public void Controls(float z, float f, Vector2[] c)
            {
                int n = m_S.Length, k = 0;
                z = Mathf.Clamp(z, m_S[0][0], m_S[n - 1][0]);
                while (k < n - 2 && z > m_S[k + 1][0]) k++;
                float h = m_S[k + 1][0] - m_S[k][0], t = (z - m_S[k][0]) / h;
                float yb = Val(k, t, h, 1), wb = Val(k, t, h, 2), w2 = Val(k, t, h, 3), y2 = Val(k, t, h, 4), w3 = Val(k, t, h, 5), y3 = Val(k, t, h, 6);
                c[0] = new Vector2(0f, yb);
                c[1] = new Vector2(wb * 0.6f, yb);
                c[2] = new Vector2(wb, yb);
                c[3] = new Vector2(w2, y2);
                c[4] = new Vector2(w3, y3);
                c[5] = new Vector2(Val(k, t, h, 7), Val(k, t, h, 8));
                c[6] = new Vector2(Val(k, t, h, 9), Val(k, t, h, 10));
                c[7] = new Vector2(Val(k, t, h, 11), Val(k, t, h, 12));
                c[8] = new Vector2(0f, Val(k, t, h, 13));
                if (f < 1f)
                {
                    var mid = new Vector2(0f, (c[0].y + c[8].y) * 0.5f);
                    for (int i = 0; i < 9; i++) c[i] = mid + (c[i] - mid) * f;
                }
            }

            /// z でのアーチの縁の高さ (車軸から ArchR 以内。それ以外は負)
            public float ArchTop(float z)
            {
                foreach (float za in Axles)
                {
                    float dz = Mathf.Abs(z - za);
                    if (dz < ArchR) return ArchY + Mathf.Sqrt(ArchR * ArchR - dz * dz);
                }
                return -1f;
            }

            /// 半断面の点 (NU 個)。lip = ホイールハウスの中の点の数 (0 = アーチの外)。
            /// アーチの中では、縁より下の側面をホイールハウス (内壁 → 天井 → 縁) に置き換える。縁より上の面は変えない
            public void Section(float z, float f, Vector2[] c, Vector2[] p, out int lip)
            {
                Controls(z, f, c);
                Profile(c, p);
                lip = 0;
                float ya = f < 1f ? -1f : ArchTop(z);
                if (ya < 0f) return;
                ya = Mathf.Min(ya, c[5].y - 0.06f);
                int i1 = 2 * K;
                while (i1 < 5 * K && p[i1].y < ya) i1++;
                if (i1 <= 2 * K) return;
                Vector2 a = p[i1 - 1], b = p[i1];
                Vector2 L = Vector2.Lerp(a, b, Mathf.Clamp01((ya - a.y) / Mathf.Max(1e-5f, b.y - a.y)));
                // 縁の点の番号はアーチの中で一定 (Lip) にする。輪ごとに変わると縁がぎざぎざになる
                // 縁から肩まで: 元の曲線を長さで取り直す
                int nUp = 0;
                m_Up[nUp++] = L;
                for (int i = i1; i <= 5 * K; i++) m_Up[nUp++] = p[i];
                Resample(m_Up, nUp, p, Lip, 5 * K);
                // 底の中央 → 内壁 → 天井 → 縁
                m_Well[0] = p[0];
                m_Well[1] = new Vector2(WellX - 0.03f, p[0].y);
                m_Well[2] = new Vector2(WellX, ya + 0.03f);
                m_Well[3] = new Vector2(L.x - 0.03f, ya + 0.03f);
                m_Well[4] = L;
                Resample(m_Well, 5, p, 0, Lip);
                lip = Lip;
            }
            public const int Lip = 3 * K;

            /// 折れ線 src[0..n-1] を長さで等分して dst[i0..i1] に置く
            static void Resample(Vector2[] src, int n, Vector2[] dst, int i0, int i1)
            {
                float total = 0f;
                for (int k = 0; k < n - 1; k++) total += (src[k + 1] - src[k]).magnitude;
                int seg = 0; float acc = 0f;
                for (int i = i0; i <= i1; i++)
                {
                    float d = total * (i - i0) / (i1 - i0);
                    while (seg < n - 2 && acc + (src[seg + 1] - src[seg]).magnitude < d) { acc += (src[seg + 1] - src[seg]).magnitude; seg++; }
                    float len = (src[seg + 1] - src[seg]).magnitude;
                    dst[i] = Vector2.Lerp(src[seg], src[seg + 1], len < 1e-6f ? 0f : Mathf.Clamp01((d - acc) / len));
                }
            }
            readonly Vector2[] m_Up = new Vector2[NU];
            readonly Vector2[] m_Well = new Vector2[5];

            /// 制御点 → 半断面の点 (NU 個)
            public static void Profile(Vector2[] c, Vector2[] p)
            {
                for (int s = 0; s < Segs; s++)
                {
                    Vector2 a = s == 0 ? c[0] : (c[s] + c[s + 1]) * 0.5f;
                    Vector2 b = c[s + 1];
                    Vector2 e = s == Segs - 1 ? c[8] : (c[s + 1] + c[s + 2]) * 0.5f;
                    for (int i = 0; i < K; i++)
                    {
                        float t = i / (float)K, q = 1f - t;
                        p[s * K + i] = q * q * a + 2f * q * t * b + t * t * e;
                    }
                }
                p[NU - 1] = c[8];
            }

            void BuildRings()
            {
                float z0 = m_S[0][0], z1 = m_S[m_S.Length - 1][0];
                var zs = new List<float>();
                for (float z = z0; z < z1; z += 0.03f) zs.Add(z);
                zs.Add(z1);
                foreach (var s in m_S) zs.Add(s[0]);
                foreach (float za in Axles)
                {
                    for (int i = 0; i <= 20; i++) zs.Add(za + ArchR * 0.999f * Mathf.Cos(Mathf.PI * i / 20f));
                    zs.Add(za - ArchR * 1.001f); zs.Add(za + ArchR * 1.001f);
                }
                zs.Sort();
                var ring = new List<Vector3>();     // (評価 z, 実際の z, すぼめ)
                float[] closeF = { 0.93f, 0.78f, 0.52f, 0.0f }, closeD = { 0.008f, 0.016f, 0.021f, 0.022f };
                for (int i = closeF.Length - 1; i >= 0; i--) ring.Add(new Vector3(z0, z0 - closeD[i], closeF[i]));
                float prev = float.NegativeInfinity;
                foreach (float z in zs)
                {
                    if (z < z0 || z > z1 || z - prev < 0.0005f) continue;
                    ring.Add(new Vector3(z, z, 1f));
                    prev = z;
                }
                for (int i = 0; i < closeF.Length; i++) ring.Add(new Vector3(z1, z1 + closeD[i], closeF[i]));
                // 絵の横位置: 輪と輪の間の実際の長さ (肩と屋根の中央で測る) に比例させる。前後の面にも画素が行き渡る
                var ca = new Vector2[9]; var cb = new Vector2[9];
                float acc = 0f;
                for (int i = 0; i < ring.Count; i++)
                {
                    Controls(ring[i].x, ring[i].z, cb);
                    if (i > 0)
                    {
                        float dz = ring[i].y - ring[i - 1].y;
                        float d5 = new Vector3(cb[5].x - ca[5].x, cb[5].y - ca[5].y, dz).magnitude;
                        float d8 = new Vector2(cb[8].y - ca[8].y, dz).magnitude;
                        acc += Mathf.Max(Mathf.Abs(dz), Mathf.Max(d5, d8), 0.002f);
                    }
                    RingZ.Add(ring[i].x); RingPos.Add(ring[i].y); RingF.Add(ring[i].z); RingU.Add(acc);
                    var tmp = ca; ca = cb; cb = tmp;
                }
                for (int i = 0; i < RingU.Count; i++) RingU[i] /= acc;
            }

            public Mesh BuildMesh()
            {
                int nr = RingZ.Count, ringN = 2 * NU - 2;
                var verts = new List<Vector3>(nr * ringN);
                var uvs = new List<Vector2>(nr * ringN);
                var c = new Vector2[9]; var p = new Vector2[NU];
                for (int j = 0; j < nr; j++)
                {
                    Section(RingZ[j], RingF[j], c, p, out _);
                    float z = RingPos[j] * Scale;
                    // 右半分 (底 → 屋根) のあと、左半分 (屋根の手前 → 底の手前)。左右で同じ絵を使う
                    for (int i = 0; i < NU; i++)
                    {
                        verts.Add(new Vector3(p[i].x * Scale, p[i].y * Scale, z));
                        uvs.Add(new Vector2(RingU[j], i / (float)(NU - 1)));
                    }
                    for (int i = NU - 2; i >= 1; i--)
                    {
                        verts.Add(new Vector3(-p[i].x * Scale, p[i].y * Scale, z));
                        uvs.Add(new Vector2(RingU[j], i / (float)(NU - 1)));
                    }
                }
                var tris = new List<int>((nr - 1) * ringN * 6);
                for (int j = 0; j < nr - 1; j++)
                    for (int i = 0; i < ringN; i++)
                    {
                        int a = j * ringN + i, b = j * ringN + (i + 1) % ringN, cc = a + ringN, d = b + ringN;
                        tris.Add(a); tris.Add(cc); tris.Add(b); tris.Add(b); tris.Add(cc); tris.Add(d);
                    }
                return Finish(verts, uvs, tris);
            }

            /// 曲面に貼る絵 2 枚 (色、金属感 R + 滑らかさ A)
            public void BuildTextures(ShellPaint paint, int w, int h, out Texture2D albedo, out Texture2D gloss)
            {
                var ca = new Color32[w * h]; var cg = new Color32[w * h];
                var c = new Vector2[9]; var p = new Vector2[NU];
                int j = 0, nr = RingZ.Count;
                for (int ix = 0; ix < w; ix++)
                {
                    float U = (ix + 0.5f) / w;
                    while (j < nr - 2 && U > RingU[j + 1]) j++;
                    float t = Mathf.Clamp01((U - RingU[j]) / Mathf.Max(1e-6f, RingU[j + 1] - RingU[j]));
                    float f = Mathf.Lerp(RingF[j], RingF[j + 1], t), pos = Mathf.Lerp(RingPos[j], RingPos[j + 1], t);
                    // アーチの端 (断面が不連続) をまたぐ所は近い方の輪で評価する
                    float ze = Mathf.Abs(RingZ[j + 1] - RingZ[j]) < 0.003f ? (t < 0.5f ? RingZ[j] : RingZ[j + 1]) : Mathf.Lerp(RingZ[j], RingZ[j + 1], t);
                    Section(ze, f, c, p, out int lip);
                    for (int iy = 0; iy < h; iy++)
                    {
                        float v = iy / (float)(h - 1) * (NU - 1);
                        int i0 = Mathf.Min(NU - 2, (int)v);
                        Vector2 q = Vector2.Lerp(p[i0], p[i0 + 1], v - i0);
                        Color32 col; float metal, smooth;
                        if (v < lip - 0.6f) { col = new Color32(9, 9, 9, 255); metal = 0f; smooth = 0.1f; }     // ホイールハウスの中
                        else paint(pos, q.x, q.y, v / K, out col, out metal, out smooth);
                        ca[iy * w + ix] = col;
                        cg[iy * w + ix] = new Color32((byte)(metal * 255f), 0, 0, (byte)(smooth * 255f));
                    }
                }
                albedo = new Texture2D(w, h, TextureFormat.RGBA32, true) { wrapMode = TextureWrapMode.Clamp, anisoLevel = 8 };
                albedo.SetPixels32(ca); albedo.Apply(true);
                gloss = new Texture2D(w, h, TextureFormat.RGBA32, true, true) { wrapMode = TextureWrapMode.Clamp, anisoLevel = 8 };
                gloss.SetPixels32(cg); gloss.Apply(true);
            }
        }

        /// 面を外向きにそろえて (符号つき体積で判定) メッシュにする
        static Mesh Finish(List<Vector3> verts, List<Vector2> uvs, List<int> tris)
        {
            Vector3 mid = Vector3.zero;
            foreach (var v in verts) mid += v;
            mid /= Mathf.Max(1, verts.Count);
            double vol = 0;
            for (int i = 0; i + 2 < tris.Count; i += 3)
                vol += Vector3.Dot(verts[tris[i]] - mid, Vector3.Cross(verts[tris[i + 1]] - mid, verts[tris[i + 2]] - mid));
            if (vol < 0)
                for (int i = 0; i + 2 < tris.Count; i += 3) { int t = tris[i + 1]; tris[i + 1] = tris[i + 2]; tris[i + 2] = t; }
            var mesh = new Mesh();
            if (verts.Count > 65000) mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh.SetVertices(verts);
            if (uvs != null) mesh.SetUVs(0, uvs);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        /// 回転体 (x 軸まわり)。prof = (半径, x)。タイヤ・リムに使う
        static Mesh Lathe(Vector2[] prof, int seg)
        {
            var verts = new List<Vector3>(); var tris = new List<int>();
            for (int j = 0; j < prof.Length; j++)
                for (int i = 0; i < seg; i++)
                {
                    float a = i * Mathf.PI * 2f / seg;
                    verts.Add(new Vector3(prof[j].y, prof[j].x * Mathf.Cos(a), prof[j].x * Mathf.Sin(a)));
                }
            for (int j = 0; j < prof.Length - 1; j++)
                for (int i = 0; i < seg; i++)
                {
                    int a = j * seg + i, b = j * seg + (i + 1) % seg, c = a + seg, d = b + seg;
                    tris.Add(a); tris.Add(c); tris.Add(b); tris.Add(b); tris.Add(c); tris.Add(d);
                }
            return Finish(verts, null, tris);
        }

        static readonly Dictionary<CarStyle, (Mesh mesh, Texture2D albedo, Texture2D gloss)> s_ShellCache =
            new Dictionary<CarStyle, (Mesh, Texture2D, Texture2D)>();

        /// シェルの車体を hull に付ける (同じ車種は形と絵を使い回す)
        void AddShell(Transform hull, Shell shell, ShellPaint paint, Material lit)
        {
            if (!s_ShellCache.TryGetValue(Style, out var c))
            {
                c.mesh = shell.BuildMesh();
                shell.BuildTextures(paint, 2048, 1024, out c.albedo, out c.gloss);
                s_ShellCache[Style] = c;
            }
            var m = new Material(lit) { color = Color.white, mainTexture = c.albedo };
            m.SetTexture("_MetallicGlossMap", c.gloss);
            m.SetFloat("_GlossMapScale", 1f);
            m.EnableKeyword("_METALLICGLOSSMAP");
            Part("Shell", hull, c.mesh, Paint(m));
        }

        // ------------------------------------------------------------------ 車輪 (実車の 3 台)
        /// 丸い肩のタイヤ・リム・5 本スポーク・ブレーキディスク。sx = 外側の向き (±1)
        void BuildWheel(Transform spin, float sx, Material tire, Material rim, Material dark, Material alu)
        {
            float R = TireRadius, W = TireWidth * 0.5f, rr = R * 0.66f;       // rr = リムの半径 (16〜17 インチ)
            Part("Tire", spin, Lathe(new[]
            {
                new Vector2(rr, -W), new Vector2(R * 0.86f, -W), new Vector2(R * 0.97f, -W * 0.86f), new Vector2(R, -W * 0.62f),
                new Vector2(R, W * 0.62f), new Vector2(R * 0.97f, W * 0.86f), new Vector2(R * 0.86f, W), new Vector2(rr, W),
            }, 36), tire);
            // リム: 外周のふち → 内側へ落ちる樽
            float o = sx * W;
            Part("Rim", spin, Lathe(new[]
            {
                new Vector2(rr * 1.01f, o), new Vector2(rr * 0.96f, o + sx * 0.0006f), new Vector2(rr * 0.90f, o - sx * 0.002f),
                new Vector2(rr * 0.86f, o - sx * W * 1.2f),
            }, 36), rim);
            var disc = Prim(PrimitiveType.Cylinder, "Brake", spin, dark);
            disc.localRotation = Quaternion.Euler(0f, 0f, 90f);
            disc.localPosition = new Vector3(o - sx * W * 0.9f, 0f, 0f);
            disc.localScale = new Vector3(rr * 1.7f, 0.0008f, rr * 1.7f);
            var hub = Prim(PrimitiveType.Cylinder, "Hub", spin, alu);
            hub.localRotation = Quaternion.Euler(0f, 0f, 90f);
            hub.localPosition = new Vector3(o - sx * 0.0035f, 0f, 0f);
            hub.localScale = new Vector3(rr * 0.42f, 0.0012f, rr * 0.42f);
            for (int s = 0; s < 5; s++)
            {
                var pivot = new GameObject("Spoke").transform;
                pivot.SetParent(spin, false);
                pivot.localRotation = Quaternion.Euler(s * 72f, 0f, 0f);
                var sp = Cube("SpokeBar", pivot, new Vector3(o - sx * 0.0030f, rr * 0.50f, 0f), new Vector3(0.0022f, rr * 0.92f, rr * 0.30f), rim);
                sp.localRotation = Quaternion.Euler(0f, 0f, sx * 6f);      // 外へ向かって少し奥へ倒す
            }
        }

        // ------------------------------------------------------------------ RX-7 (FD3S)
        // 全長 4,285 × 全幅 1,760 × 全高 1,230 mm・WB 2,425 mm。曲面だけの低いクーペ。低いボンネットの両脇に前フェンダーの峰、
        // ぐっと絞った客室、盛り上がった後フェンダー、なだらかに落ちるハッチ、丸 3 灯のテール、尾端の羽根。色はイエロー
        const float kRx7Scale = 0.257f / 2.425f;        // TT-02 のホイールベースに合わせた縮尺

        static readonly float[][] kRx7Sections =
        {
            //   z      yb     wb     w2     y2     w3     y3     w4     y4     w5     y5     w6     y6     y7
            St(-0.925f, 0.340f, 0.440f, 0.540f, 0.400f, 0.600f, 0.580f, 0.570f, 0.740f, 0.480f, 0.800f, 0.260f, 0.820f, 0.825f),   // 尾端
            St(-0.870f, 0.290f, 0.620f, 0.760f, 0.360f, 0.800f, 0.580f, 0.760f, 0.780f, 0.640f, 0.850f, 0.340f, 0.870f, 0.875f),
            St(-0.700f, 0.230f, 0.700f, 0.840f, 0.310f, 0.865f, 0.560f, 0.830f, 0.830f, 0.690f, 0.890f, 0.400f, 0.900f, 0.900f),
            St(-0.400f, 0.160f, 0.740f, 0.860f, 0.260f, 0.880f, 0.540f, 0.850f, 0.865f, 0.700f, 0.915f, 0.420f, 0.920f, 0.920f),   // ハッチのガラスの下端
            St( 0.000f, 0.15f, 0.74f, 0.87f, 0.25f, 0.885f, 0.54f, 0.85f, 0.880f, 0.69f, 0.935f, 0.50f, 1.070f, 1.085f),  // 後軸 (後フェンダーの張り)
            St( 0.350f, 0.14f, 0.74f, 0.86f, 0.24f, 0.87f, 0.52f, 0.825f, 0.875f, 0.70f, 0.915f, 0.54f, 1.175f, 1.195f),  // 屋根の後端
            St( 0.750f, 0.14f, 0.74f, 0.845f, 0.24f, 0.85f, 0.50f, 0.80f, 0.860f, 0.70f, 0.895f, 0.55f, 1.205f, 1.230f),  // 屋根の頂点
            St( 1.050f, 0.14f, 0.74f, 0.84f, 0.24f, 0.845f, 0.50f, 0.795f, 0.850f, 0.705f, 0.885f, 0.54f, 1.185f, 1.205f), // 屋根の前端
            St( 1.400f, 0.14f, 0.74f, 0.84f, 0.24f, 0.845f, 0.50f, 0.80f, 0.845f, 0.71f, 0.875f, 0.60f, 1.030f, 1.050f),  // 前窓の中ほど
            St( 1.750f, 0.14f, 0.74f, 0.845f, 0.24f, 0.85f, 0.50f, 0.81f, 0.840f, 0.72f, 0.860f, 0.45f, 0.850f, 0.850f),  // カウル
            St( 2.100f, 0.14f, 0.74f, 0.855f, 0.24f, 0.865f, 0.50f, 0.83f, 0.820f, 0.72f, 0.815f, 0.42f, 0.780f, 0.780f),
            St( 2.425f, 0.14f, 0.74f, 0.86f, 0.24f, 0.875f, 0.50f, 0.835f, 0.775f, 0.70f, 0.765f, 0.40f, 0.715f, 0.715f), // 前軸 (フェンダーの峰がボンネットより高い)
            St(2.850f, 0.150f, 0.700f, 0.830f, 0.250f, 0.845f, 0.460f, 0.790f, 0.660f, 0.650f, 0.655f, 0.380f, 0.620f, 0.620f),
            St(3.130f, 0.170f, 0.580f, 0.700f, 0.260f, 0.720f, 0.390f, 0.660f, 0.520f, 0.540f, 0.535f, 0.300f, 0.525f, 0.525f),
            St(3.280f, 0.200f, 0.400f, 0.480f, 0.270f, 0.490f, 0.350f, 0.450f, 0.430f, 0.380f, 0.450f, 0.200f, 0.455f, 0.455f),
            St(3.320f, 0.230f, 0.280f, 0.340f, 0.290f, 0.350f, 0.340f, 0.320f, 0.400f, 0.270f, 0.415f, 0.140f, 0.420f, 0.420f),   // ノーズ
        };

        static bool In(float v, float a, float b) => v >= a && v <= b;

        /// RX-7 の塗り分け (実車の m。x は半幅 ≥ 0、u は断面上の位置 0〜7: 5.5〜6.4 が側面の窓、6.6〜7 が上面)
        static void PaintRx7(float z, float x, float y, float u, out Color32 col, out float metal, out float smooth)
        {
            var yellow = new Color32(247, 190, 14, 255);
            var black = new Color32(12, 12, 13, 255);
            var glass = new Color32(10, 13, 18, 255);
            var seam = new Color32(70, 52, 6, 255);
            col = yellow; metal = 0.25f; smooth = 0.80f;
            bool top = u > 6.55f, side = In(u, 2.6f, 5.4f);

            // ---- 窓 (ガラスと黒い縁)
            float sideF = 1.63f - (u - 5.5f) / 0.9f * 0.55f, sideR = -0.06f + (u - 5.5f) / 0.9f * 0.42f;   // 傾いた A ピラーと C ピラー
            bool gSide = In(u, 5.55f, 6.35f) && In(z, sideR, sideF);
            bool gFront = u > 6.62f && In(z, 1.07f, 1.74f);
            bool gRear = u > 6.62f && In(z, -0.40f, 0.33f);
            if (gSide || gFront || gRear)
            {
                bool edge = gSide ? (u < 5.60f || u > 6.30f || z < sideR + 0.025f || z > sideF - 0.025f || Mathf.Abs(z - (0.50f + (u - 5.5f) * 0.08f)) < 0.018f)
                          : gFront ? (u < 6.66f || z < 1.095f || z > 1.715f)
                          : (u < 6.66f || z < -0.375f || z > 0.305f);
                col = edge ? black : glass; metal = 0f; smooth = edge ? 0.45f : 0.96f;
                return;
            }

            // ---- 黒い樹脂: 前のリップ・後ろの下まわり
            if ((z > 2.75f && y < 0.185f) || (z < -0.55f && y < 0.27f && x < 0.62f))
            { col = black; metal = 0f; smooth = 0.35f; return; }

            // ---- ノーズ: 中央の開口・両脇のダクト・小さなランプ
            if (z > 3.20f)
            {
                if ((x < 0.27f && In(y, 0.225f, 0.33f)) || (In(x, 0.36f, 0.52f) && In(y, 0.235f, 0.315f) && z < 3.33f))
                { col = black; metal = 0f; smooth = 0.25f; return; }
            }
            if (z > 3.02f && In(x, 0.44f, 0.62f) && In(y, 0.375f, 0.41f))
            { col = new Color32(240, 170, 60, 255); metal = 0.1f; smooth = 0.95f; return; }

            // ---- 尾端: スモークの帯と丸 3 灯 (外 2 つが赤、内がバック灯)
            if (z < -0.86f && In(y, 0.625f, 0.765f) && x < 0.72f)
            {
                col = new Color32(22, 8, 8, 255); metal = 0f; smooth = 0.92f;
                for (int k = 0; k < 3; k++)
                {
                    float dx = x - (0.27f + 0.17f * k), dy = y - 0.695f, r = Mathf.Sqrt(dx * dx + dy * dy);
                    if (r < 0.062f) col = k == 0 ? new Color32(225, 220, 210, 255) : (r < 0.030f ? new Color32(120, 6, 8, 255) : new Color32(205, 12, 14, 255));
                }
                return;
            }

            // ---- パネルの合わせ目 (幅 8 mm の濃い線)
            const float lw = 0.004f;
            bool s = false;
            if (side && In(y, 0.17f, 0.90f) && (Mathf.Abs(z - 1.63f) < lw || Mathf.Abs(z - 0.50f) < lw)) s = true;       // ドアの前後
            if (side && In(z, 0.50f, 1.63f) && Mathf.Abs(y - 0.185f) < lw) s = true;                                       // ドアの下
            if (side && In(y, 0.17f, 0.50f) && (Mathf.Abs(z - 2.92f) < lw || Mathf.Abs(z + 0.52f) < lw)) s = true;       // 前後バンパーの継ぎ目
            if (u > 5.0f && z > 1.80f)
            {
                if (In(z, 1.80f, 3.06f) && Mathf.Abs(x - 0.645f) < lw) s = true;                                           // ボンネットとフェンダー
                if (x < 0.645f && (Mathf.Abs(z - 3.06f) < lw || Mathf.Abs(z - 1.80f) < lw)) s = true;                     // ボンネットの前後
                if (In(x, 0.33f, 0.60f) && In(z, 2.80f, 3.02f) && (Mathf.Abs(x - 0.33f) < lw || Mathf.Abs(x - 0.60f) < lw || Mathf.Abs(z - 2.80f) < lw || Mathf.Abs(z - 3.02f) < lw)) s = true;   // リトラクタブルライトのふた
            }
            if (top && z < -0.42f && (Mathf.Abs(z + 0.62f) < lw && x < 0.70f)) s = true;                                   // ハッチの下端
            if (side && In(z, 0.60f, 0.74f) && In(y, 0.795f, 0.825f)) { col = black; metal = 0f; smooth = 0.5f; return; } // ドアハンドル
            if (side && In(z, 2.02f, 2.10f) && In(y, 0.60f, 0.635f)) { col = new Color32(240, 140, 20, 255); metal = 0f; smooth = 0.9f; return; }   // サイドマーカー
            if (s) { col = seam; metal = 0f; smooth = 0.4f; }
        }

        void BuildRx7(Transform hull, bool withMast, MatFn Mat, Material black, Material glass,
                      Material lamp, Material tail, Material alu, Material lit)
        {
            float k = kRx7Scale;
            var shell = new Shell(kRx7Sections, k, new[] { 0f, 2.425f }, 0.365f, TireRadius / k, 0.60f);
            AddShell(hull, shell, PaintRx7, lit);
            var yellow = Paint(Mat(new Color(0.97f, 0.745f, 0.055f), 0.80f, 0.25f));
            // 尾端の羽根 (後期型の純正: 両端の脚から立つ 1 枚の板)
            Cube("WingPlate", hull, new Vector3(0f, 1.035f * k, -0.80f * k), new Vector3(1.50f * k, 0.028f * k, 0.21f * k), yellow, -7f);
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("WingStay", hull, new Vector3(sx * 0.70f * k, 0.975f * k, -0.79f * k), new Vector3(0.05f * k, 0.13f * k, 0.20f * k), yellow, -12f);
                // ドアミラー
                var mir = Prim(PrimitiveType.Sphere, "Mirror", hull, yellow);
                mir.localPosition = new Vector3(sx * 0.895f * k, 0.965f * k, 1.56f * k);
                mir.localScale = new Vector3(0.17f * k, 0.115f * k, 0.10f * k);
                Cube("MirrorStay", hull, new Vector3(sx * 0.83f * k, 0.925f * k, 1.57f * k), new Vector3(0.10f * k, 0.03f * k, 0.05f * k), black);
            }
            var pipe = Prim(PrimitiveType.Cylinder, "Exhaust", hull, alu);
            pipe.localPosition = new Vector3(0.42f * k, 0.265f * k, -0.90f * k);
            pipe.localRotation = Quaternion.Euler(90f, 0f, 0f);
            pipe.localScale = new Vector3(0.10f * k, 0.05f * k, 0.10f * k);
            if (withMast) Mast(hull, black, alu, 0.060f, 0.131f);
        }
    }
}
