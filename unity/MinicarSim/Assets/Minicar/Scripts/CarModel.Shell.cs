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
            public float WidthGain = 1f;            // 幅の合わせ (曲線は制御点の内側を通るので、実車の全幅になるよう少し広げる)
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
                for (int i = 0; i < 9; i++) c[i].x *= WidthGain;
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
                // 端は半球に近い丸みで閉じる (長さ 40 mm)
                float[] closeF = { 0.95f, 0.84f, 0.66f, 0.40f, 0.0f }, closeD = { 0.010f, 0.021f, 0.031f, 0.038f, 0.040f };
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
                int nr = RingZ.Count, ringN = 2 * NU - 1;
                var verts = new List<Vector3>(nr * ringN);
                var uvs = new List<Vector2>(nr * ringN);
                var c = new Vector2[9]; var p = new Vector2[NU];
                for (int j = 0; j < nr; j++)
                {
                    Section(RingZ[j], RingF[j], c, p, out _);
                    float z = RingPos[j] * Scale;
                    // 右半分 (底 → 屋根) のあと、左半分 (屋根の手前 → 底)。絵は上半分が左・下半分が右 (左右で違う塗り分けができる)
                    for (int i = 0; i < NU; i++)
                    {
                        verts.Add(new Vector3(p[i].x * Scale, p[i].y * Scale, z));
                        uvs.Add(new Vector2(RingU[j], 0.5f * i / (NU - 1)));
                    }
                    for (int i = NU - 2; i >= 0; i--)
                    {
                        verts.Add(new Vector3(-p[i].x * Scale, p[i].y * Scale, z));
                        uvs.Add(new Vector2(RingU[j], 1f - 0.5f * i / (NU - 1)));
                    }
                }
                var tris = new List<int>((nr - 1) * ringN * 6);
                for (int j = 0; j < nr - 1; j++)
                    for (int i = 0; i < ringN - 1; i++)
                    {
                        int a = j * ringN + i, b = a + 1, cc = a + ringN, d = b + ringN;
                        tris.Add(a); tris.Add(cc); tris.Add(b); tris.Add(b); tris.Add(cc); tris.Add(d);
                    }
                return Finish(verts, uvs, tris);
            }

            /// 曲面に貼る絵 2 枚 (色、金属感 R + 滑らかさ A。HDRP ではそのままマスクの絵として使う)
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
                        float vv = iy / (float)(h - 1), sgn = vv <= 0.5f ? 1f : -1f;       // 絵の下半分 = 右 (x > 0)、上半分 = 左
                        float v = (vv <= 0.5f ? vv : 1f - vv) * 2f * (NU - 1);
                        int i0 = Mathf.Min(NU - 2, (int)v);
                        Vector2 q = Vector2.Lerp(p[i0], p[i0 + 1], v - i0);
                        Color32 col; float metal, smooth;
                        if (v < lip - 0.6f) { col = new Color32(9, 9, 9, 255); metal = 0f; smooth = 0.1f; }     // ホイールハウスの中
                        else paint(pos, sgn * q.x, q.y, v / K, out col, out metal, out smooth);
                        ca[iy * w + ix] = col;
                        cg[iy * w + ix] = new Color32((byte)(metal * 255f), 255, 0, (byte)(smooth * 255f));    // G = 255: HDRP のマスクでは AO (遮りなし)
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
                var bs = c.mesh.bounds.size / shell.Scale;
                Debug.Log($"[CarModel] {Style}: 全長 {bs.z:F3} × 全幅 {bs.x:F3} × 全高 {c.mesh.bounds.max.y / shell.Scale:F3} m (車体の曲面。羽根・鏡を除く)");
                shell.BuildTextures(paint, 2048, 2048, out c.albedo, out c.gloss);
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
        void BuildWheel(Transform spin, float sx, Material tire, Material rim, Material dark, Material alu, int spokes = 5)
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
            for (int s = 0; s < spokes; s++)
            {
                var pivot = new GameObject("Spoke").transform;
                pivot.SetParent(spin, false);
                pivot.localRotation = Quaternion.Euler(s * 360f / spokes, 0f, 0f);
                var sp = Cube("SpokeBar", pivot, new Vector3(o - sx * 0.0030f, rr * 0.50f, 0f), new Vector3(0.0022f, rr * 0.92f, rr * (spokes > 5 ? 0.22f : 0.30f)), rim);
                sp.localRotation = Quaternion.Euler(0f, 0f, sx * 6f);      // 外へ向かって少し奥へ倒す
            }
        }

        // ------------------------------------------------------------------ RX-7 (FD3S)
        // 全長 4,285 × 全幅 1,760 × 全高 1,230 mm・WB 2,425 mm (シェルは鼻先 +3.225 〜 尾端 −1.000 m。尾端の先に羽根が出る)。曲面だけの低いクーペ。低いボンネットの両脇に前フェンダーの峰、
        // ぐっと絞った客室、盛り上がった後フェンダー、なだらかに落ちるハッチ、丸 3 灯のテール、尾端の羽根。色はイエロー
        const float kRx7Scale = 0.257f / 2.425f;        // TT-02 のホイールベースに合わせた縮尺
        const float kNdScale = 0.257f / 2.310f;
        const float kB787Scale = 0.257f / 2.662f;

        /// 実車の寸法: 縮尺、タイヤの半径・幅 [m]、車軸の位置での車体の半幅 [m]
        static void RealDims(CarStyle style, out float k, out float tireR, out float tireW, out float side)
        {
            switch (style)
            {
                case CarStyle.Roadster: k = kNdScale; tireR = 0.300f; tireW = 0.205f; side = 0.865f; break;     // 195/50R16
                case CarStyle.B787: k = kB787Scale; tireR = 0.335f; tireW = 0.320f; side = 0.990f; break;       // 前後の平均
                default: k = kRx7Scale; tireR = 0.315f; tireW = 0.245f; side = 0.875f; break;                    // 225/50R16
            }
        }

        // 断面は 4 面図 (側面・上面) の輪郭に合わせてある (2026-10-05 水野。tools の下見で図と重ねて確かめた):
        //   前の張り出し 0.80 m・後ろ 1.00 m (前軸から鼻先まで / 後軸から尾端まで)、上から見ると後ろへ向かってすぼまり、
        //   客室 (屋根の前端・前窓・カウル) は以前より 12 cm 前。幅は全幅 1,760 mm、高さは全高 1,230 mm に合わせた
        static readonly float[][] kRx7Sections =
        {
            //   z      yb     wb     w2     y2     w3     y3     w4     y4     w5     y5     w6     y6     y7
            St(-1.000f,  0.360f,  0.360f,  0.440f,  0.420f,  0.480f,  0.580f,  0.460f,  0.730f,  0.400f,  0.785f,  0.220f,  0.800f,  0.805f),   // 尾端
            St(-0.940f,  0.292f,  0.467f,  0.581f,  0.361f,  0.614f,  0.580f,  0.590f,  0.769f,  0.493f,  0.834f,  0.266f,  0.854f,  0.859f),
            St(-0.770f,  0.230f,  0.617f,  0.727f,  0.310f,  0.749f,  0.560f,  0.719f,  0.830f,  0.608f,  0.890f,  0.353f,  0.900f,  0.900f),
            St(-0.440f,  0.160f,  0.697f,  0.810f,  0.260f,  0.829f,  0.540f,  0.801f,  0.865f,  0.659f,  0.915f,  0.408f,  0.923f,  0.923f),   // ハッチのガラスの下端
            St( 0.000f,  0.150f,  0.736f,  0.865f,  0.250f,  0.880f,  0.540f,  0.845f,  0.880f,  0.686f,  0.935f,  0.499f,  1.089f,  1.107f),   // 後軸 (後フェンダーの張り)
            St( 0.350f,  0.140f,  0.736f,  0.855f,  0.240f,  0.865f,  0.520f,  0.820f,  0.875f,  0.696f,  0.915f,  0.538f,  1.178f,  1.198f),   // 屋根の後端
            St( 0.750f,  0.140f,  0.756f,  0.832f,  0.240f,  0.838f,  0.500f,  0.792f,  0.860f,  0.715f,  0.895f,  0.556f,  1.214f,  1.239f),   // 屋根の頂点
            St( 1.150f,  0.140f,  0.760f,  0.832f,  0.240f,  0.838f,  0.500f,  0.791f,  0.850f,  0.724f,  0.886f,  0.548f,  1.202f,  1.224f),   // 屋根の前端
            St( 1.350f,  0.140f,  0.769f,  0.842f,  0.240f,  0.847f,  0.500f,  0.802f,  0.847f,  0.735f,  0.880f,  0.582f,  1.124f,  1.145f),
            St( 1.550f,  0.140f,  0.777f,  0.851f,  0.240f,  0.856f,  0.500f,  0.815f,  0.845f,  0.747f,  0.874f,  0.612f,  1.018f,  1.037f),   // 前窓の中ほど
            St( 1.750f,  0.140f,  0.782f,  0.861f,  0.240f,  0.866f,  0.500f,  0.828f,  0.842f,  0.758f,  0.867f,  0.512f,  0.912f,  0.920f),
            St( 1.900f,  0.140f,  0.778f,  0.864f,  0.240f,  0.869f,  0.500f,  0.833f,  0.839f,  0.756f,  0.857f,  0.457f,  0.847f,  0.847f),   // カウル
            St( 2.150f,  0.140f,  0.752f,  0.869f,  0.240f,  0.878f,  0.499f,  0.843f,  0.818f,  0.673f,  0.800f,  0.428f,  0.766f,  0.761f),
            St( 2.425f,  0.140f,  0.746f,  0.866f,  0.240f,  0.882f,  0.512f,  0.841f,  0.810f,  0.645f,  0.775f,  0.403f,  0.730f,  0.725f),   // 前軸 (フェンダーの峰がボンネットより高い)
            St( 2.800f,  0.159f,  0.709f,  0.828f,  0.259f,  0.842f,  0.478f,  0.798f,  0.725f,  0.630f,  0.704f,  0.375f,  0.678f,  0.678f),
            St( 2.980f,  0.169f,  0.675f,  0.776f,  0.269f,  0.786f,  0.455f,  0.742f,  0.648f,  0.579f,  0.642f,  0.330f,  0.631f,  0.631f),
            St( 3.120f,  0.187f,  0.568f,  0.659f,  0.279f,  0.673f,  0.423f,  0.628f,  0.560f,  0.487f,  0.559f,  0.274f,  0.554f,  0.554f),
            St( 3.225f,  0.230f,  0.380f,  0.450f,  0.300f,  0.470f,  0.400f,  0.430f,  0.485f,  0.330f,  0.490f,  0.180f,  0.490f,  0.490f),   // ノーズ
        };

        static bool In(float v, float a, float b) => v >= a && v <= b;

        /// RX-7 の塗り分け (実車の m。x は右が正、u は断面上の位置 0〜7: 5.5〜6.4 が側面の窓、6.6〜7 が上面)
        // 断面を 4 面図に合わせて前後へ動かしたので、塗り分けの z は「動かす前の位置」に直してから判定する
        // (窓・ライト・合わせ目の数値は動かす前のまま使える)。CarModel の部品の位置と tools/preview_circuit.py も同じ写し替え
        static readonly float[] kRx7ZNew = { -1.000f, 0f, 0.75f, 1.15f, 1.90f, 2.25f, 2.425f, 3.225f };
        static readonly float[] kRx7ZOld = { -0.907f, 0f, 0.75f, 1.03f, 1.78f, 2.20f, 2.425f, 3.302f };
        static float Rx7PaintZ(float z)
        {
            int n = kRx7ZNew.Length;
            if (z <= kRx7ZNew[0]) return kRx7ZOld[0] + (z - kRx7ZNew[0]);
            for (int i = 1; i < n; i++)
                if (z <= kRx7ZNew[i])
                    return Mathf.Lerp(kRx7ZOld[i - 1], kRx7ZOld[i], (z - kRx7ZNew[i - 1]) / (kRx7ZNew[i] - kRx7ZNew[i - 1]));
            return kRx7ZOld[n - 1] + (z - kRx7ZNew[n - 1]);
        }

        static void PaintRx7(float z, float x, float y, float u, out Color32 col, out float metal, out float smooth)
        {
            float zn = z;            // いまの位置 (横の窓とピラーは 4 面図の側面の形をそのまま使う)
            z = Rx7PaintZ(z);
            x = Mathf.Abs(x);        // 左右対称
            var yellow = new Color32(250, 196, 10, 255);
            var black = new Color32(12, 12, 13, 255);
            var glass = new Color32(10, 13, 18, 255);
            var seam = new Color32(70, 52, 6, 255);
            col = yellow; metal = 0.25f; smooth = 0.80f;
            bool top = u > 6.55f, side = In(u, 2.6f, 5.4f);

            // ---- 窓 (ガラスと黒い縁)
            // 横の窓とピラー: 4 面図の側面を目盛りつきで読んだ形を、横から見た位置 (zn = 前後、y = 高さ) でそのまま塗る。
            //   ドアの窓: 下の縁はベルトライン (後ろ 0.868 → 前 0.843)、上の縁は 1.14 (その上に車体色の屋根の縁が残る)。
            //             前の縁は A ピラーと平行に後ろへ倒れ (下の角 zn 1.313 → 上の角 1.06)、前窓との間は車体色の A ピラー。
            //             後ろの縁は下 0.42 → 上 0.585 へ前に流れる
            //   B ピラー: 窓の後ろ下の角にはまる黒い三角の飾り板 (zn 0.52 より後ろ・高さ 1.05 まで)
            //   その後ろは車体色の太い C ピラー。ハッチのガラスは屋根から横へ回り込み、前の縁は (0.30, 1.17) → (0.02, 0.935) の斜めの線
            bool flank = In(u, 5.2f, 6.50f);
            float belt = 0.868f - (zn - 0.42f) * 0.028f;
            float rearEdge = y < 1.064f ? 0.42f + (y - 0.882f) * 0.516f : 0.514f + (y - 1.064f) * 0.934f;
            float frontEdge = 1.313f - (y - 0.843f) * 0.891f;
            bool gSide = flank && In(y, belt, 1.14f) && In(zn, rearEdge, frontEdge);
            bool bPillar = gSide && zn < 0.52f && y < 1.05f;
            float sideIn = gSide ? Mathf.Min(Mathf.Min(y - belt, 1.14f - y), Mathf.Min(zn - rearEdge, frontEdge - zn)) : 0f;     // 縁からの距離
            bool gHatchSide = In(u, 5.6f, 6.50f) && In(zn, -0.47f, 0.02f + (y - 0.935f) * 1.19f) && y > 0.925f + (zn + 0.47f) * 0.02f;
            // 前窓: 下の縁 (カウル) は中央が前へ張り出す弧。ハッチのガラス: 角の丸い大きな 1 枚が、後ろ寄りでは横 (屋根の縁の下) まで回り込む
            float cowl = 1.75f - 0.09f * (x / 0.60f) * (x / 0.60f);
            bool gFront = u > 6.50f && In(z, 1.06f, cowl);
            float hz = (z + 0.04f) / 0.38f, hx = x / 0.62f;
            bool gRear = (u > 6.50f && hz * hz * hz * hz + hx * hx * hx * hx < 1f) || gHatchSide;
            if (gSide || gFront || gRear)
            {
                float he = Mathf.Pow(hz * hz * hz * hz + hx * hx * hx * hx, 0.25f);
                bool edge = gSide ? (bPillar || sideIn < 0.014f)
                          : gFront ? (u < 6.515f || z < 1.075f || z > cowl - 0.015f)
                          : (u > 6.50f ? he > 0.94f : false);
                // ガラスは少し青みのある濃い色 (真っ黒にすると、黒い B ピラーや窓の縁と見分けがつかない)
                col = edge ? black : new Color32(30, 40, 52, 255); metal = 0f; smooth = edge ? (bPillar ? 0.20f : 0.45f) : 0.96f;
                // B ピラーの飾り板とドアの窓の間に、窓の縁の細い線 (ゴム) が 1 本見える
                if (gSide && !bPillar && zn < 0.532f && y < 1.05f) { col = new Color32(70, 72, 76, 255); smooth = 0.5f; }
                return;
            }

            // ---- 黒い樹脂: 前のリップ・後ろの下まわり
            if ((z > 2.60f && y < 0.20f) || (z < -0.55f && y < 0.27f && x < 0.66f) || (In(z, 0.40f, 2.05f) && y < 0.165f))
            { col = black; metal = 0f; smooth = 0.35f; return; }

            // ---- ボンネット: 黒いカーボン (排熱口つき)。前の角に固定式の細いライト (スモークのカバー)
            float bz = (z - 2.435f) / 0.645f, bx = x / 0.665f;
            if (u > 5.0f && bz * bz * bz * bz + bx * bx * bx * bx < 1f)
            {
                bool lamp = In(z, 2.86f, 3.07f) && In(x, 0.36f, 0.645f);
                bool vent = (In(z, 2.20f, 2.42f) && In(x, 0.14f, 0.44f)) || (In(z, 2.52f, 2.66f) && In(x, 0.20f, 0.46f));
                col = lamp ? new Color32(40, 43, 46, 255) : vent ? new Color32(4, 4, 5, 255) : new Color32(13, 13, 14, 255);
                metal = lamp ? 0.2f : 0f; smooth = lamp ? 0.96f : vent ? 0.15f : 0.55f;      // カーボンは空を強く映さない程度のつや
                if (lamp && In(z, 2.93f, 3.01f) && (In(x, 0.42f, 0.49f) || In(x, 0.52f, 0.59f))) col = new Color32(200, 204, 205, 255);     // 中の丸いランプ 2 つ
                return;
            }
            // ---- バンパー: 中央の大きな開口・両脇のダクト・オレンジのウインカー
            if (z > 3.15f)
            {
                if ((x < 0.36f && In(y, 0.215f, 0.335f)) || (In(x, 0.44f, 0.56f) && In(y, 0.235f, 0.32f)))
                { col = black; metal = 0f; smooth = 0.25f; return; }
            }
            if (z > 3.15f && In(x, 0.36f, 0.52f) && In(y, 0.385f, 0.42f)) { col = new Color32(240, 120, 20, 255); metal = 0f; smooth = 0.9f; return; }
            // ---- 尾端: スモークの帯と丸 3 灯 (外 2 つが赤、内がバック灯)
            if (z < -0.84f && In(y, 0.625f, 0.765f) && x < 0.70f)
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
            var shell = new Shell(kRx7Sections, k, new[] { 0f, 2.425f }, 0.365f, TireRadius / k, 0.60f) { WidthGain = 1.760f / 1.752f };   // 全幅 1,760 mm
            AddShell(hull, shell, PaintRx7, lit);
            var yellow = Paint(Mat(new Color(0.97f, 0.745f, 0.055f), 0.80f, 0.25f));
            // 前のリップ (黒い板が前へ張り出す)
            Cube("FrontLip", hull, new Vector3(0f, 0.185f * k, 3.06f * k), new Vector3(1.30f * k, 0.03f * k, 0.32f * k), black);
            // GT ウイング: 高い位置の黄色い翼と翼端板、黒い脚 2 本
            Cube("WingPlate", hull, new Vector3(0f, 1.20f * k, -0.88f * k), new Vector3(1.50f * k, 0.03f * k, 0.25f * k), yellow, -6f);
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("WingEnd", hull, new Vector3(sx * 0.75f * k, 1.17f * k, -0.88f * k), new Vector3(0.02f * k, 0.16f * k, 0.30f * k), yellow);
                Cube("WingStay", hull, new Vector3(sx * 0.44f * k, 1.04f * k, -0.82f * k), new Vector3(0.025f * k, 0.32f * k, 0.09f * k), black, -18f);
                // ドアミラー
                var mir = Prim(PrimitiveType.Sphere, "Mirror", hull, yellow);
                // 4 面図: ドアの前の端から後ろへ流れる、車体と同じ色の平たいミラー
                mir.localPosition = new Vector3(sx * 0.930f * k, 0.975f * k, 1.64f * k);
                mir.localRotation = Quaternion.Euler(0f, sx * 18f, 0f);
                mir.localScale = new Vector3(0.19f * k, 0.105f * k, 0.13f * k);
                Cube("MirrorStay", hull, new Vector3(sx * 0.845f * k, 0.925f * k, 1.68f * k), new Vector3(0.10f * k, 0.03f * k, 0.06f * k), yellow);
            }
            var pipe = Prim(PrimitiveType.Cylinder, "Exhaust", hull, alu);
            pipe.localPosition = new Vector3(0.42f * k, 0.265f * k, -0.99f * k);
            pipe.localRotation = Quaternion.Euler(90f, 0f, 0f);
            pipe.localScale = new Vector3(0.10f * k, 0.05f * k, 0.10f * k);
            if (withMast) Mast(hull, black, alu, 0.060f, 0.131f);
        }

        // ------------------------------------------------------------------ ロードスター (ND)
        // 全長 3,915 × 全幅 1,735 × 全高 1,235 mm・WB 2,310 mm。短く低いノーズ、張りのある前フェンダー、短いデッキ。
        // 幌を開けたオープン: 室内は車体の曲面をくぼませて黒く塗り、前窓・座席・ロールバーは別の部品。
        // 色はソウルレッド (金属的な下地 + クリア層)。右ハンドル。ミラー・前窓の枠・ホイールは黒
        // 形 (参考: 実車を斜め上と斜め後ろから撮った写真、2026-10-05 水野)。初代 (NA) のような丸い石けん形にしないこと:
        //   上から見ると前後が強くすぼまり、前後のフェンダーが張り出してドアのところで絞られる。
        //   前はフェンダーの峰が立ち、ボンネットはその間の低い谷 (中央がわずかに盛り上がる)。鼻先は低くとがる。
        //   後ろは張り出したフェンダーの峰の間でトランクが一段低く、尾端で跳ね上がる縁 (小さな羽根) になる。
        //   肩 (c5) と窓の付け根 (c6) を近づけて、峰と縁の角を立ててある
        static readonly float[][] kNdSections =
        {
            //   z      yb     wb     w2     y2     w3     y3     w4     y4     w5     y5     w6     y6     y7
            St(-0.755f, 0.340f, 0.360f, 0.460f, 0.420f, 0.540f, 0.600f, 0.535f, 0.800f, 0.500f, 0.868f, 0.300f, 0.880f, 0.880f),   // 尾端 (跳ね上がった縁)
            St(-0.700f, 0.28f, 0.56f, 0.66f, 0.36f, 0.700f, 0.60f, 0.685f, 0.830f, 0.62f, 0.888f, 0.34f, 0.886f, 0.884f),
            St(-0.500f, 0.20f, 0.67f, 0.77f, 0.30f, 0.795f, 0.58f, 0.775f, 0.860f, 0.69f, 0.912f, 0.40f, 0.880f, 0.872f),  // トランク (フェンダーより低い)
            St(-0.250f, 0.16f, 0.72f, 0.845f, 0.26f, 0.860f, 0.56f, 0.835f, 0.885f, 0.735f, 0.935f, 0.44f, 0.900f, 0.888f),
            St( 0.000f, 0.150f, 0.720f, 0.855f, 0.250f, 0.8675f, 0.560f, 0.840f, 0.895f, 0.735f, 0.945f, 0.500f, 0.915f, 0.905f), // 後軸 (後フェンダーの峰)
            St( 0.350f, 0.140f, 0.720f, 0.835f, 0.240f, 0.845f, 0.540f, 0.800f, 0.880f, 0.700f, 0.915f, 0.560f, 0.860f, 0.600f),
            St( 0.750f, 0.140f, 0.720f, 0.820f, 0.240f, 0.825f, 0.520f, 0.775f, 0.860f, 0.685f, 0.890f, 0.560f, 0.850f, 0.580f),  // ドア (いちばん絞られる所)
            St( 1.000f, 0.140f, 0.720f, 0.817f, 0.240f, 0.822f, 0.520f, 0.772f, 0.850f, 0.685f, 0.880f, 0.560f, 0.840f, 0.580f), // 前窓の上端
            St( 1.300f, 0.140f, 0.720f, 0.821f, 0.240f, 0.826f, 0.520f, 0.780f, 0.842f, 0.690f, 0.872f, 0.540f, 0.855f, 0.800f),
            St( 1.600f, 0.14f, 0.72f, 0.83f, 0.24f, 0.838f, 0.52f, 0.800f, 0.830f, 0.715f, 0.862f, 0.44f, 0.848f, 0.850f),  // カウル
            St( 1.950f, 0.14f, 0.72f, 0.845f, 0.24f, 0.857f, 0.52f, 0.825f, 0.795f, 0.745f, 0.838f, 0.40f, 0.772f, 0.786f),
            St( 2.310f, 0.14f, 0.72f, 0.85f, 0.24f, 0.8675f, 0.52f, 0.835f, 0.745f, 0.745f, 0.790f, 0.36f, 0.700f, 0.712f), // 前軸 (前フェンダーの峰)
            St( 2.700f, 0.15f, 0.64f, 0.76f, 0.25f, 0.775f, 0.46f, 0.735f, 0.650f, 0.64f, 0.690f, 0.32f, 0.612f, 0.626f),
            St( 2.950f, 0.17f, 0.47f, 0.56f, 0.26f, 0.575f, 0.40f, 0.535f, 0.530f, 0.45f, 0.558f, 0.24f, 0.518f, 0.528f),
            St( 3.070f, 0.20f, 0.30f, 0.36f, 0.28f, 0.375f, 0.37f, 0.345f, 0.452f, 0.29f, 0.466f, 0.16f, 0.464f, 0.466f),
            St( 3.080f, 0.230f, 0.200f, 0.240f, 0.300f, 0.250f, 0.350f, 0.230f, 0.412f, 0.190f, 0.425f, 0.110f, 0.430f, 0.430f),   // ノーズ
        };

        static void PaintNd(float z, float x, float y, float u, out Color32 col, out float metal, out float smooth)
        {
            bool left = x < 0f;      // 給油口は左だけ
            x = Mathf.Abs(x);        // ほかは左右対称
            var black = new Color32(12, 12, 13, 255);
            // ソウルレッドの下地: 金属感の強い深い赤 (陰は暗く沈み、光の当たる所だけ鮮やかに光る)。明るい普通の赤にしないこと (2026-10-05 水野)
            col = new Color32(168, 4, 10, 255); metal = 0.80f; smooth = 0.62f;
            bool side = In(u, 2.6f, 5.4f);
            // 室内 (幌を開けた開口の内側) とダッシュボードの上は黒
            if (u > 6.02f && In(z, 0.12f, 1.64f)) { col = new Color32(16, 16, 17, 255); metal = 0f; smooth = 0.15f; return; }
            // 座席の後ろ: たたんだ幌のカバー (黒い布)
            if (u > 5.80f && In(z, -0.10f, 0.12f)) { col = new Color32(20, 20, 22, 255); metal = 0f; smooth = 0.10f; return; }
            if (z > 2.60f && y < 0.185f) { col = black; metal = 0f; smooth = 0.35f; return; }
            // 後ろの下回り: 黒いディフューザー (中央が高い台形)。両端に縦長の赤い反射板
            if (z < -0.45f && x < 0.56f && y < 0.40f - x * 0.18f) { col = black; metal = 0f; smooth = 0.30f; return; }
            if (z < -0.66f && In(x, 0.585f, 0.625f) && In(y, 0.37f, 0.50f)) { col = new Color32(170, 10, 12, 255); metal = 0f; smooth = 0.9f; return; }
            // ナンバープレート (白地) と、その上の銀のエンブレム
            if (z < -0.74f && x < 0.165f && In(y, 0.475f, 0.640f)) { col = new Color32(236, 236, 230, 255); metal = 0f; smooth = 0.5f; return; }
            if (z < -0.735f && (x * x + (y - 0.795f) * (y - 0.795f)) < 0.036f * 0.036f) { col = new Color32(200, 202, 206, 255); metal = 0.9f; smooth = 0.9f; return; }
            if (z > 2.96f && x < 0.44f - (0.42f - y) * 0.9f && In(y, 0.235f, 0.42f)) { col = black; metal = 0f; smooth = 0.25f; return; }      // グリルの大きな口 (上が広い台形)
            // ヘッドライト: フェンダーの先の上面に、後ろへ流れる細い目。黒いレンズの中に白い帯
            if (In(z, 2.64f, 2.93f) && In(u, 4.55f, 5.55f) && x > 0.40f)
            {
                float t = (z - 2.64f) / 0.29f;                    // 0 = 後ろの端、1 = 前の端
                float mid = Mathf.Lerp(5.25f, 4.95f, t), half = Mathf.Lerp(0.10f, 0.32f, t);   // 前ほど太い
                if (Mathf.Abs(u - mid) < half)
                {
                    bool led = Mathf.Abs(u - mid) < half * 0.38f && t > 0.25f;
                    col = led ? new Color32(238, 238, 230, 255) : new Color32(18, 20, 24, 255); metal = 0.1f; smooth = 0.95f; return;
                }
            }
            // 尾灯: 角に寄った丸い灯 (車体の横まで回り込む) と、そこから内側へ伸びる細いくさび形
            if (z < -0.56f && y > 0.62f && u < 5.9f)
            {
                // 尾端の面では x、横へ回り込んだ所では z で測る (角をまたいで 1 つの丸に見えるように)
                float along = z < -0.735f ? x - 0.515f : 0.02f + (-0.735f - z) * 0.9f;
                float dy = y - 0.745f, r = Mathf.Sqrt(along * along + dy * dy);
                if (r < 0.080f) { col = r < 0.040f ? new Color32(130, 8, 10, 255) : r < 0.068f ? new Color32(220, 16, 18, 255) : new Color32(40, 6, 8, 255); metal = 0f; smooth = 0.92f; return; }
                if (z < -0.735f && In(x, 0.24f, 0.44f) && Mathf.Abs(dy) < 0.006f + (x - 0.24f) * 0.11f) { col = new Color32(228, 222, 216, 255); metal = 0.1f; smooth = 0.95f; return; }
            }
            // 前フェンダーの横の小さな方向指示灯 (橙)
            if (side && In(z, 1.74f, 1.82f) && In(y, 0.565f, 0.590f)) { col = new Color32(236, 150, 30, 255); metal = 0f; smooth = 0.9f; return; }
            const float lw = 0.004f;
            bool s = false;
            if (side && In(y, 0.17f, 0.88f) && (Mathf.Abs(z - 1.50f) < lw || Mathf.Abs(z - 0.42f) < lw)) s = true;
            if (side && In(z, 0.42f, 1.50f) && Mathf.Abs(y - 0.185f) < lw) s = true;
            if (u > 5.0f && z > 1.66f && ((In(z, 1.66f, 2.86f) && Mathf.Abs(x - 0.60f) < lw) || (x < 0.60f && (Mathf.Abs(z - 2.86f) < lw || Mathf.Abs(z - 1.66f) < lw)))) s = true;
            // トランクの合わせ目 (デッキの上)
            if (u > 5.6f && z < -0.12f && ((In(z, -0.66f, -0.12f) && Mathf.Abs(x - 0.56f) < lw) || (x < 0.56f && (Mathf.Abs(z + 0.66f) < lw || Mathf.Abs(z + 0.12f) < lw)))) s = true;
            // 給油口の丸いふた (左の後フェンダー)
            if (left && side)
            {
                float fz = z + 0.34f, fy = y - 0.74f, fr = Mathf.Sqrt(fz * fz + fy * fy);
                if (Mathf.Abs(fr - 0.068f) < lw) s = true;
            }
            // ドアの取っ手 (車体と同じ色の出っ張りなので、下に影の線だけ)
            if (side && In(z, 0.50f, 0.66f) && Mathf.Abs(y - 0.795f) < lw * 1.5f) s = true;
            if (s) { col = new Color32(66, 4, 8, 255); metal = 0f; smooth = 0.4f; }
        }

        void BuildRoadster(Transform hull, bool withMast, MatFn Mat, Material black, Material glass,
                           Material lamp, Material tail, Material alu, Material lit)
        {
            float k = kNdScale;
            var shell = new Shell(kNdSections, k, new[] { 0f, 2.310f }, 0.350f, TireRadius / k, 0.60f) { WidthGain = 1.735f / 1.718f };     // 全幅 1,735 mm
            AddShell(hull, shell, PaintNd, lit);
            // ドアミラー: つやのある黒。ドアの前の端 (A ピラーの付け根) から短い足で出る
            var gloss = Mat(new Color(0.03f, 0.03f, 0.035f), 0.85f);
            foreach (float sx in new[] { -1f, 1f })
            {
                var mir = Prim(PrimitiveType.Sphere, "Mirror", hull, gloss);
                mir.localPosition = new Vector3(sx * 0.900f * k, 0.965f * k, 1.36f * k);
                mir.localRotation = Quaternion.Euler(0f, sx * 12f, 0f);
                mir.localScale = new Vector3(0.20f * k, 0.12f * k, 0.085f * k);
                Cube("MirrorStay", hull, new Vector3(sx * 0.805f * k, 0.905f * k, 1.37f * k), new Vector3(0.09f * k, 0.035f * k, 0.06f * k), gloss);
            }
            foreach (float sx in new[] { -1f, 1f })
            {
                var pipe = Prim(PrimitiveType.Cylinder, "Exhaust", hull, alu);
                pipe.localPosition = new Vector3(sx * 0.12f * k + 0.40f * k, 0.255f * k, -0.76f * k);
                pipe.localRotation = Quaternion.Euler(90f, 0f, 0f);
                pipe.localScale = new Vector3(0.07f * k, 0.05f * k, 0.07f * k);
            }
            // 前窓 (後ろへ 59° 倒した板) と黒い枠
            var clear = Mat(new Color(0.55f, 0.66f, 0.70f, 0.28f), 0.96f);       // 透けて見える前窓
            ProcTex.MakeFade(clear);
            Cube("Windshield", hull, new Vector3(0f, 1.03f * k, 1.30f * k), new Vector3(1.16f * k, 0.655f * k, 0.008f * k), clear, -58.7f);
            // 前窓の枠は太い黒 (上の横桟と左右の A ピラー)。付け根に黒いカウル
            Cube("ShieldTop", hull, new Vector3(0f, 1.208f * k, 1.012f * k), new Vector3(1.26f * k, 0.045f * k, 0.050f * k), black);
            Cube("Cowl", hull, new Vector3(0f, 0.858f * k, 1.60f * k), new Vector3(1.20f * k, 0.03f * k, 0.10f * k), black);
            var seat = Mat(new Color(0.07f, 0.065f, 0.065f), 0.38f);
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("ShieldSide", hull, new Vector3(sx * 0.605f * k, 1.03f * k, 1.30f * k), new Vector3(0.060f * k, 0.69f * k, 0.055f * k), black, -58.7f);
                Cube("SeatBase", hull, new Vector3(sx * 0.36f * k, 0.52f * k, 0.58f * k), new Vector3(0.46f * k, 0.12f * k, 0.50f * k), seat);
                // 背もたれは肩から上が細くなり、頭当てと一体 (ハイバックのシート)
                Cube("SeatBack", hull, new Vector3(sx * 0.36f * k, 0.76f * k, 0.31f * k), new Vector3(0.46f * k, 0.46f * k, 0.12f * k), seat, -14f);
                Cube("SeatShoulder", hull, new Vector3(sx * 0.36f * k, 1.00f * k, 0.245f * k), new Vector3(0.36f * k, 0.12f * k, 0.11f * k), seat, -14f);
                Cube("HeadRest", hull, new Vector3(sx * 0.36f * k, 1.11f * k, 0.215f * k), new Vector3(0.24f * k, 0.14f * k, 0.10f * k), seat, -14f);
                // 座席の後ろの輪 (黒い覆い)
                Cube("RollHoop", hull, new Vector3(sx * 0.36f * k, 1.03f * k, 0.085f * k), new Vector3(0.34f * k, 0.22f * k, 0.06f * k), black, -10f);
            }
            // 座席の間の物入れと、後ろの風よけの板 (黒)
            Cube("Console", hull, new Vector3(0f, 0.66f * k, 0.62f * k), new Vector3(0.20f * k, 0.16f * k, 0.80f * k), seat);
            Cube("WindBlocker", hull, new Vector3(0f, 0.99f * k, 0.085f * k), new Vector3(0.36f * k, 0.14f * k, 0.02f * k), black, -10f);
            // アンテナ (右の後フェンダーの上、後ろへ傾く)
            var ant = Prim(PrimitiveType.Cylinder, "Antenna", hull, black);
            ant.localPosition = new Vector3(0.60f * k, 1.06f * k, -0.36f * k);
            ant.localRotation = Quaternion.Euler(-28f, 0f, 0f);
            ant.localScale = new Vector3(0.012f * k, 0.15f * k, 0.012f * k);
            var wheel = Prim(PrimitiveType.Cylinder, "SteeringWheel", hull, black);      // 右ハンドル (+x)
            wheel.localPosition = new Vector3(0.36f * k, 0.86f * k, 1.10f * k);
            wheel.localRotation = Quaternion.Euler(-65f, 0f, 0f);
            wheel.localScale = new Vector3(0.36f * k, 0.012f * k, 0.36f * k);
            if (withMast) Mast(hull, black, alu, 0.010f, 0.105f);
        }

        // ------------------------------------------------------------------ 787B
        // 全長 4,782 × 全幅 1,994 × 全高 1,003 mm・WB 2,662 mm。低く平たいグループ C のプロトタイプ。
        // 中央の細い操縦席 (キャノピー)、その両脇に盛り上がる前後のフェンダー、低いノーズ、高い位置の大きなリアウイング。
        // 塗り分けは明るいオレンジと緑、境目に白い線
        static readonly float[][] kB787Sections =
        {
            //   z      yb     wb     w2     y2     w3     y3     w4     y4     w5     y5     w6     y6     y7
            St(-1.132f, 0.200f, 0.800f, 0.930f, 0.260f, 0.960f, 0.400f, 0.950f, 0.560f, 0.860f, 0.600f, 0.400f, 0.600f, 0.600f),   // 尾端
            St(-1.000f, 0.10f, 0.86f, 0.96f, 0.16f, 0.985f, 0.38f, 0.975f, 0.600f, 0.88f, 0.660f, 0.40f, 0.660f, 0.660f),
            St(-0.500f, 0.08f, 0.88f, 0.97f, 0.14f, 0.995f, 0.38f, 0.985f, 0.680f, 0.86f, 0.730f, 0.38f, 0.700f, 0.700f),
            St( 0.000f, 0.07f, 0.88f, 0.975f, 0.13f, 0.997f, 0.38f, 0.985f, 0.720f, 0.82f, 0.760f, 0.36f, 0.720f, 0.740f), // 後軸 (後フェンダーの頂点)
            St( 0.600f, 0.07f, 0.88f, 0.97f, 0.13f, 0.99f, 0.36f, 0.97f, 0.620f, 0.78f, 0.660f, 0.40f, 0.800f, 0.860f),   // エンジンカウル
            St( 1.100f, 0.07f, 0.88f, 0.965f, 0.13f, 0.985f, 0.36f, 0.96f, 0.560f, 0.74f, 0.600f, 0.44f, 0.930f, 1.000f), // 操縦席の後ろ
            St( 1.550f, 0.07f, 0.88f, 0.96f, 0.13f, 0.98f, 0.36f, 0.955f, 0.540f, 0.72f, 0.580f, 0.45f, 0.960f, 1.003f),  // 屋根の頂点
            St( 1.950f, 0.07f, 0.88f, 0.96f, 0.13f, 0.98f, 0.36f, 0.955f, 0.540f, 0.72f, 0.580f, 0.46f, 0.800f, 0.840f),  // 前窓の中ほど
            St( 2.300f, 0.07f, 0.88f, 0.965f, 0.13f, 0.985f, 0.36f, 0.965f, 0.580f, 0.80f, 0.620f, 0.40f, 0.600f, 0.600f), // カウル
            St( 2.662f, 0.07f, 0.88f, 0.97f, 0.13f, 0.99f, 0.38f, 0.975f, 0.660f, 0.84f, 0.680f, 0.44f, 0.540f, 0.520f),  // 前軸 (前フェンダーがノーズより高い)
            St( 3.100f, 0.07f, 0.86f, 0.95f, 0.13f, 0.97f, 0.32f, 0.95f, 0.500f, 0.82f, 0.520f, 0.44f, 0.420f, 0.400f),
            St( 3.450f, 0.07f, 0.80f, 0.88f, 0.12f, 0.90f, 0.22f, 0.87f, 0.300f, 0.76f, 0.320f, 0.40f, 0.300f, 0.290f),
            St( 3.572f, 0.080f, 0.700f, 0.760f, 0.110f, 0.770f, 0.160f, 0.740f, 0.200f, 0.640f, 0.210f, 0.340f, 0.210f, 0.210f),   // ノーズ
        };

        static bool Text(string t, float a, float b, float h) => DotFont.At(t, a, b, h);

        /// 787B (1991 年ル・マン優勝車・ゼッケン 55) の塗り分け。x は右が正。
        /// 上から見て X 字に色が入れ替わる: 左前と右後ろが緑、右前と左後ろがオレンジ。境目は白い破線 (縫い目)。
        /// 屋根の中央は銀白、側面に白い帯と青い文字、ドアと鼻先に白い丸のゼッケン、床の縁は黄色
        static void PaintB787(float z, float x, float y, float u, out Color32 col, out float metal, out float smooth)
        {
            var orange = new Color32(240, 92, 22, 255);
            var green = new Color32(14, 122, 62, 255);
            var white = new Color32(240, 240, 235, 255);
            var blue = new Color32(26, 52, 150, 255);
            var black = new Color32(12, 12, 13, 255);
            var glass = new Color32(10, 13, 18, 255);
            float ax = Mathf.Abs(x), sgn = x >= 0f ? 1f : -1f;
            metal = 0.10f; smooth = 0.82f;
            // キャノピー (前窓と横の窓)
            // 前窓は横の窓まで回り込む。A ピラーは細い黒い枠だけ (u 6.44〜6.47)
            bool gSide = In(u, 5.68f, 6.44f) && In(z, 1.26f + (u - 5.68f) * 0.2f, 2.24f - (u - 5.68f) * 0.30f);
            bool gFront = u > 6.47f && In(z, 1.72f, 2.26f);
            if (In(u, 6.44f, 6.47f) && In(z, 1.72f, 2.03f)) { col = black; metal = 0f; smooth = 0.4f; return; }
            if (gFront && z < 1.86f) { col = orange; return; }                                       // 前窓の上のオレンジの帯
            if (gSide || gFront) { col = glass; metal = 0f; smooth = 0.96f; return; }
            if (y < 0.10f) { col = black; metal = 0f; smooth = 0.3f; return; }
            if (y < 0.16f && In(u, 2.0f, 5.0f)) { col = new Color32(245, 205, 20, 255); return; }                       // 床の縁の黄色
            // ヘッドライト: 左右のフェンダーの前に、透明なカバーの中の丸 2 灯
            if (z > 3.02f && In(ax, 0.50f, 0.88f) && In(y, 0.15f, 0.36f) && u < 5.6f)
            {
                col = new Color32(26, 30, 34, 255); metal = 0.2f; smooth = 0.95f;
                for (int k = 0; k < 2; k++)
                {
                    float lx = ax - (0.60f + 0.17f * k), ly = y - 0.25f;
                    if (lx * lx + ly * ly < 0.07f * 0.07f) col = new Color32(250, 246, 225, 255);
                }
                return;
            }
            if (z < -1.02f && In(ax, 0.55f, 0.92f) && In(y, 0.42f, 0.50f)) { col = new Color32(210, 14, 16, 255); metal = 0f; smooth = 0.9f; return; }
            if (z < -1.05f && ax < 0.50f && In(y, 0.24f, 0.52f)) { col = black; metal = 0f; smooth = 0.2f; return; }
            if (In(u, 2.6f, 5.0f) && In(z, 1.95f, 2.25f) && In(y, 0.20f, 0.50f)) { col = black; metal = 0f; smooth = 0.15f; return; }   // 前輪の後ろの排熱口
            // 上から見た模様 (実車を真上から撮った写真に合わせた): 操縦席を囲む大きな緑のひし形、四隅はオレンジ、
            // 後端の中央に緑の三角、鼻先は緑の帯。境目は白い破線
            const float zc = 1.45f;
            float d1 = Mathf.Abs(z - zc) / (z > zc ? 1.35f : 1.60f) + ax / 1.70f;          // 中央のひし形 (1 未満が中)
            float d2 = Mathf.Abs(z + 1.17f) / 1.05f + ax / 1.05f;                            // 後端の三角
            bool grn = d1 < 1f || d2 < 1f || z > 3.30f;
            col = grn ? green : orange;
            float e1 = Mathf.Abs(d1 - 1f) * 1.05f, e2 = Mathf.Abs(d2 - 1f) * 0.74f, e3 = Mathf.Abs(z - 3.30f);
            float dash = Mathf.Repeat(z * 0.8f + ax * 0.6f, 0.16f);
            if ((e1 < 0.022f || e2 < 0.022f) && dash < 0.10f) col = white;
            if (e3 < 0.018f && Mathf.Repeat(ax, 0.16f) < 0.10f) col = white;
            bool side = In(u, 2.6f, 5.2f), topS = u > 5.3f;
            // 操縦席の後ろ (エンジンカウルの中央) は白いパネル。前窓の上にオレンジの帯
            if (topS && In(z, 0.25f, 1.26f) && ax < 0.16f + (z - 0.25f) * 0.26f) { col = white; metal = 0.05f; }
            if (u > 6.40f && In(z, 1.26f, 1.72f)) col = new Color32(22, 24, 27, 255);       // 屋根 (黒)
            // 鼻先: 白い帯に青い RENOWN (前から読む向き)
            if (topS && In(z, 2.96f, 3.24f) && In(x, -0.20f, 0.84f))
            {
                col = white;
                if (Text("RENOWN", 0.80f - x, 3.22f - z, 0.20f)) col = blue;
            }
            // ゼッケン: 左前の角 (上面) と、左右の後輪の前 (側面) に白い四角と黒い 55
            if (topS && In(z, 2.50f, 2.86f) && In(x, -0.88f, -0.40f))
            {
                col = white;
                if (Text("55", -0.42f - x, 2.83f - z, 0.26f)) col = black;
            }
            if (side && In(z, 0.52f, 1.00f) && In(y, 0.20f, 0.50f))
            {
                col = white;
                float a = sgn > 0f ? z - 0.57f : 0.95f - z;
                if (Text("55", a, y - 0.24f, 0.22f)) col = black;
            }
            // 後フェンダーの上の空気取り入れ口 (黒い切り欠き)
            if (topS && In(z, 0.10f, 0.42f) && In(ax, 0.58f, 0.70f)) { col = black; metal = 0f; smooth = 0.15f; }
        }

        static Material s_WingLabel;
        static Material WingLabel(Material lit)
        {
            if (s_WingLabel != null) return s_WingLabel;
            const int w = 512, h = 128;
            var px = new Color32[w * h];
            var orange = new Color32(240, 92, 22, 255); var navy = new Color32(20, 26, 70, 255); var white = new Color32(245, 245, 240, 255);
            float th = 0.62f, tw = 6f * th * 6f / 7f;                 // 文字の高さと全体の幅 (絵の高さを 1 として)
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    float a = x / (float)h - (w / (float)h - tw) * 0.5f, b = y / (float)h - (1f - th) * 0.5f;
                    bool ink = Text("CHARGE", a, b, th), near = false;
                    for (int k = 0; k < 8 && !ink && !near; k++)
                    {
                        float ang = k * Mathf.PI / 4f;
                        near = Text("CHARGE", a + 0.035f * Mathf.Cos(ang), b + 0.035f * Mathf.Sin(ang), th);
                    }
                    px[y * w + x] = ink ? navy : near ? white : orange;
                }
            var t = new Texture2D(w, h, TextureFormat.RGBA32, true) { wrapMode = TextureWrapMode.Clamp, anisoLevel = 8 };
            t.SetPixels32(px); t.Apply(true);
            s_WingLabel = new Material(lit) { name = "WingLabel", color = Color.white, mainTexture = t };
            s_WingLabel.SetFloat("_Glossiness", 0.7f);
            return s_WingLabel;
        }

        void BuildB787(Transform hull, bool withMast, MatFn Mat, Material black, Material glass,
                       Material lamp, Material tail, Material alu, Material lit)
        {
            float k = kB787Scale;
            var shell = new Shell(kB787Sections, k, new[] { 0f, 2.662f }, 0.400f, TireRadius / k, 0.62f) { WidthGain = 1.994f / 1.984f };  // 全幅 1,994 mm
            AddShell(hull, shell, PaintB787, lit);
            var green = Paint(Mat(new Color(0.94f, 0.36f, 0.09f), 0.82f, 0.10f));      // 翼・翼端板・鏡はオレンジ
            // リアウイング: 後端の低い位置の大きな翼 (オレンジ)。左右の翼端板で車体につながる。鏡はフェンダーの上の高い柱
            Cube("WingPlate", hull, new Vector3(0f, 0.86f * k, -1.22f * k), new Vector3(1.98f * k, 0.035f * k, 0.50f * k), green, -5f);
            // 翼の上面の文字 (オレンジ地に、白で縁取った紺の CHARGE)。前から読む向き・C が車の右
            var label = Prim(PrimitiveType.Quad, "WingLabel", hull, WingLabel(lit));
            label.localPosition = new Vector3(0f, 0.882f * k, -1.22f * k);
            label.localRotation = Quaternion.Euler(90f - 5f, 180f, 0f);
            label.localScale = new Vector3(1.90f * k, 0.44f * k, 1f);
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("WingEnd", hull, new Vector3(sx * 0.995f * k, 0.74f * k, -1.20f * k), new Vector3(0.02f * k, 0.36f * k, 0.58f * k), green);
                Cube("WingStay", hull, new Vector3(sx * 0.25f * k, 0.74f * k, -1.12f * k), new Vector3(0.03f * k, 0.22f * k, 0.20f * k), black);
                var mir = Prim(PrimitiveType.Sphere, "Mirror", hull, green);
                mir.localPosition = new Vector3(sx * 0.80f * k, 0.86f * k, 2.42f * k);
                mir.localScale = new Vector3(0.16f * k, 0.11f * k, 0.10f * k);
                Cube("MirrorStay", hull, new Vector3(sx * 0.80f * k, 0.76f * k, 2.42f * k), new Vector3(0.03f * k, 0.18f * k, 0.04f * k), green);
            }
            if (withMast) Mast(hull, black, alu, 0.150f, 0.100f);
        }
    }
}
