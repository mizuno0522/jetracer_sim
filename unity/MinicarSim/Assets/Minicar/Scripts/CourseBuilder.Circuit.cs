// 実車スケールのサーキット (course.json の kind = "circuit") を組み立てる。形状の定義元は course.py の circuit_course
// (中心線 = centerline_shortcut、幅・ランオフ = circuit)。ここは「どう見えるか」だけ。
//
//   舗装のランオフ → グラベル (コーナー外側) → アスファルト (骨材・補修跡・法線マップ)
//   → 走行ラインのタイヤ痕・ブレーキングの黒い筋 → 白線 → 縁石 (紅白・凹凸・剥げ) → ガードレール (波形鋼板と支柱)
//   → コントロールライン (市松) → 距離板 (300/200/100) → 観客席・ピット棟 → 地形・森・山・雲 (Landscape.cs) → 屋外光とかすみ
// テクスチャは ProcTex が実行時に作る (画像ファイルなし)。メッシュの uv は m 単位で、材質の tile で 1 枚の大きさを決める。
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public partial class CourseBuilder
    {
        Vector2[] m_C;          // 中心線 (閉ループ・始点を末尾に重ねない)
        Vector2[] m_N;          // 左向きの法線
        float[] m_K;            // 符号つき曲率 (+左)
        float[] m_S;            // 弧長 (n + 1 個)

        /// <summary>サーキットの範囲 [xmin, ymin, xmax, ymax] (SimBridge の俯瞰・RViz 表示用)</summary>
        public float[] CircuitBounds => Data.circuit != null && Data.circuit.bounds != null && Data.circuit.bounds.Length == 4
            ? Data.circuit.bounds : new[] { 0f, 0f, 100f, 100f };

        void BuildCircuit()
        {
            var c = Data.circuit ?? new CircuitData { width_m = 15f, runoff_m = 8f };
            if (c.barrier_height_m <= 0f) c.barrier_height_m = 1f;
            if (c.kerb_width_m <= 0f) c.kerb_width_m = 1.2f;
            if (c.kerb_min_curvature <= 0f) c.kerb_min_curvature = 1f / 220f;
            PrepareCenterline(Data.centerline_shortcut);
            float half = c.width_m * 0.5f, bar = half + c.runoff_m, kw = c.kerb_width_m;
            var b = CircuitBounds;
            int n = m_C.Length;

            // ---- 地面 (地形・森・山・雲は Landscape) ----
            var land = new Landscape(m_C, bar, b);
            BuildLandscape(land);

            ProcTex.Gravel(256, 41, out var grvA, out var grvN);
            var gravel = ProcTex.Material(m_Lit, grvA, grvN, 0.02f, 0.9f, new Vector2(4f, 4f));
            ProcTex.Asphalt(512, 31, 0.30f, out var runA, out var runN);
            var runoff = ProcTex.Material(m_Lit, runA, runN, 0.12f, 0.5f, new Vector2(6f, 6f));
            Func<int, float> kmax = i => { float k = 0f; for (int d = -8; d <= 8; d++) k = Mathf.Max(k, Mathf.Abs(m_K[(i + d + n) % n])); return k; };
            Func<int, float> ksign = i => { float s = 0f; for (int d = -8; d <= 8; d++) s += m_K[(i + d + n) % n]; return Mathf.Sign(s); };
            Func<int, bool> kerbAt = i => KAround(i, 3) >= c.kerb_min_curvature;
            // 片側ずつ: [コース端 | 縁石 | ランオフ | グラベル (曲率の大きいコーナーの外側) | ランオフ 1 m | バリア]
            foreach (float side in new[] { 1f, -1f })
            {
                Func<int, bool> grv = i => kmax(i) > 1f / 140f && ksign(i) == -side;
                Func<int, float> inner = i => half + (kerbAt(i) ? kw : 0f);
                Func<int, float> gIn = i => half + kw + 1f, gOut = i => bar - 1f;
                MeshObject("Runoff", SideStrip(side, inner, i => grv(i) ? gIn(i) : bar, 0.02f, null), runoff);
                MeshObject("Gravel", SideStrip(side, gIn, gOut, 0.02f, grv), gravel);
                MeshObject("Runoff", SideStrip(side, gOut, i => bar, 0.02f, grv), runoff);
            }

            // ---- コース ----
            ProcTex.Asphalt(512, 11, 0.22f, out var asA, out var asN);
            var asphalt = ProcTex.Material(m_Lit, asA, asN, 0.22f, 0.8f, new Vector2(6f, 6f));
            MeshObject("Asphalt", Strip(i => -half + 0.30f, i => half - 0.30f, 0.02f, null), asphalt);
            MeshObject("AsphaltEdgeL", Strip(i => half - 0.05f, i => half, 0.02f, null), asphalt);
            MeshObject("AsphaltEdgeR", Strip(i => -half, i => -half + 0.05f, 0.02f, null), asphalt);

            // 走行ラインのタイヤ痕: コーナーのイン側に寄る線 (曲率を前後 60 m でならして横にずらす)
            var line = RacingLine(half);
            var rubberTex = ProcTex.Rubber(51);
            var rubber = ProcTex.Material(m_Lit, rubberTex, null, 0.30f, 0f, new Vector2(2.6f, 60f));
            ProcTex.MakeFade(rubber);
            var rubberLight = new Material(rubber) { color = new Color(1f, 1f, 1f, 0.45f) };   // 普段の走行ラインは薄く
            // 半透明の重ね貼りは深度を書かないので、少し浮かせても見た目は変わらない (遠くでのちらつき防止)
            MeshObject("RubberLine", Strip(i => line[i] - 1.3f, i => line[i] + 1.3f, 0.06f, null, i => line[i] - 1.3f), rubberLight);
            // ブレーキングゾーン: 長い直線の先のコーナー手前 150 m に濃い筋をもう 1 枚
            var brake = BrakeZones(150f);
            MeshObject("BrakeMarks", Strip(i => line[i] - 1.1f, i => line[i] + 1.1f, 0.07f, i => brake[i], i => line[i] - 1.1f), rubber);

            var paint = ProcTex.Material(m_Lit, ProcTex.Line(61), null, 0.35f, 0f, new Vector2(0.25f, 8f));
            MeshObject("EdgeL", Strip(i => half - 0.30f, i => half - 0.05f, 0.02f, null), paint);
            MeshObject("EdgeR", Strip(i => -half + 0.05f, i => -half + 0.30f, 0.02f, null), paint);

            ProcTex.Kerb(128, 256, 71, out var kbA, out var kbN);
            var kerb = ProcTex.Material(m_Lit, kbA, kbN, 0.35f, 1.0f, new Vector2(kw, 6f));
            MeshObject("KerbL", Strip(i => half, i => half + kw, 0.02f, kerbAt), kerb);
            MeshObject("KerbR", Strip(i => -half - kw, i => -half, 0.02f, kerbAt), kerb);

            // ---- バリア ----
            ProcTex.Armco(81, out var arA, out var arN);
            var armco = ProcTex.Material(m_Lit, arA, arN, 0.55f, 0.8f, new Vector2(4f, 1f));
            armco.SetFloat("_Metallic", 0.35f);
            MeshObject("BarrierL", Fence(bar, c.barrier_height_m), armco);
            MeshObject("BarrierR", Fence(-bar, c.barrier_height_m), armco);
            BuildTrackside(bar);

            BuildControlLine(half);
            BuildBrakeBoards(half + 2.5f);
            BuildPaddock(bar);
            BuildOutdoorLighting();
        }

        // ------------------------------------------------------------------
        void PrepareCenterline(float[] flat)
        {
            int n = flat != null ? flat.Length / 2 : 0;
            if (n < 4) { flat = new float[] { 0, 0, 100, 0, 100, 100, 0, 100 }; n = 4; }
            var pts = new List<Vector2>(n);
            for (int i = 0; i < n; i++) pts.Add(new Vector2(flat[2 * i], flat[2 * i + 1]));
            if ((pts[0] - pts[n - 1]).sqrMagnitude < 1e-6f) pts.RemoveAt(n - 1);
            n = pts.Count;
            m_C = pts.ToArray();
            m_N = new Vector2[n];
            m_K = new float[n];
            m_S = new float[n + 1];
            for (int i = 0; i < n; i++)
            {
                Vector2 t = (m_C[(i + 1) % n] - m_C[(i - 1 + n) % n]).normalized;
                m_N[i] = new Vector2(-t.y, t.x);
                m_S[i + 1] = m_S[i] + (m_C[(i + 1) % n] - m_C[i]).magnitude;
            }
            // 曲率: 前後 3 点 (中心線の刻み 2 m なら 12 m の弦) の外接円
            const int h = 3;
            for (int i = 0; i < n; i++)
            {
                Vector2 a = m_C[(i - h + n) % n], p = m_C[i], q = m_C[(i + h) % n];
                float ab = (p - a).magnitude, bc = (q - p).magnitude, ca = (a - q).magnitude;
                float cr = (p.x - a.x) * (q.y - a.y) - (q.x - a.x) * (p.y - a.y);
                m_K[i] = ab * bc * ca < 1e-6f ? 0f : 2f * cr / (ab * bc * ca);
            }
        }

        float KAround(int i, int w)
        {
            int n = m_C.Length;
            float k = 0f;
            for (int d = -w; d <= w; d++) k = Mathf.Max(k, Mathf.Abs(m_K[(i + d + n) % n]));
            return k;
        }

        Vector3 At(int i, float off, float z)
        {
            Vector2 p = m_C[i] + m_N[i] * off;
            return RosFrame.ToUnity(p.x, p.y, z);
        }

        /// 中心線から左 a(i)〜b(i) [m] の帯 (b > a)。include が false の区間は作らない。
        /// uv は m 単位 (u = 横位置 − uBase(i)、v = 弧長)。uBase を渡すと帯の端から測る (走行ラインのように横に動く帯)
        Mesh Strip(Func<int, float> a, Func<int, float> b, float z, Func<int, bool> include, Func<int, float> uBase = null)
        {
            int n = m_C.Length;
            var v = new List<Vector3>();
            var uv = new List<Vector2>();
            var tri = new List<int>();
            for (int i = 0; i < n; i++)
            {
                if (include != null && !include(i)) continue;
                int j = (i + 1) % n;
                int b0 = v.Count;
                float a0 = a(i), b0f = b(i), a1 = a(j), b1 = b(j);
                v.Add(At(i, a0, z)); v.Add(At(i, b0f, z)); v.Add(At(j, a1, z)); v.Add(At(j, b1, z));
                float ui = uBase != null ? uBase(i) : 0f, uj = uBase != null ? uBase(j) : 0f;
                uv.Add(new Vector2(a0 - ui, m_S[i])); uv.Add(new Vector2(b0f - ui, m_S[i]));
                uv.Add(new Vector2(a1 - uj, m_S[i + 1])); uv.Add(new Vector2(b1 - uj, m_S[i + 1]));
                // b が左 (大きい) なので、この順で上向き (Unity は cross(b − a, c − a) が表)
                tri.Add(b0); tri.Add(b0 + 1); tri.Add(b0 + 2);
                tri.Add(b0 + 1); tri.Add(b0 + 3); tri.Add(b0 + 2);
            }
            var m = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            m.SetVertices(v);
            m.SetUVs(0, uv);
            m.SetTriangles(tri, 0);
            var nr = new Vector3[v.Count];
            for (int i = 0; i < nr.Length; i++) nr[i] = Vector3.up;
            m.normals = nr;
            m.RecalculateTangents();
            m.RecalculateBounds();
            return m;
        }

        /// 片側の帯: side = +1 (左) なら中心線から inner〜outer、-1 (右) なら −outer〜−inner
        Mesh SideStrip(float side, Func<int, float> inner, Func<int, float> outer, float z, Func<int, bool> include)
        {
            return side > 0f ? Strip(inner, outer, z, include) : Strip(i => -outer(i), i => -inner(i), z, include);
        }

        /// 中心線から左 off [m] に立つ高さ h の塀。表はコース側 (カメラは常にコースの内側にある)
        Mesh Fence(float off, float h)
        {
            int n = m_C.Length;
            var v = new Vector3[(n + 1) * 2];
            var uv = new Vector2[(n + 1) * 2];
            var tri = new int[n * 6];
            for (int k = 0; k <= n; k++)
            {
                int i = k % n;
                v[2 * k] = At(i, off, 0f);
                v[2 * k + 1] = At(i, off, h);
                uv[2 * k] = new Vector2(m_S[k], 0f);
                uv[2 * k + 1] = new Vector2(m_S[k], 1f);
            }
            for (int k = 0; k < n; k++)
            {
                int a = 2 * k, t = k * 6;
                if (off > 0f)   // 左のバリア: 表は右 (コース側)
                {
                    tri[t] = a; tri[t + 1] = a + 1; tri[t + 2] = a + 2;
                    tri[t + 3] = a + 1; tri[t + 4] = a + 3; tri[t + 5] = a + 2;
                }
                else
                {
                    tri[t] = a; tri[t + 1] = a + 2; tri[t + 2] = a + 1;
                    tri[t + 3] = a + 1; tri[t + 4] = a + 2; tri[t + 5] = a + 3;
                }
            }
            var m = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            m.vertices = v;
            m.uv = uv;
            m.triangles = tri;
            m.RecalculateNormals();
            m.RecalculateTangents();
            m.RecalculateBounds();
            return m;
        }

        /// 走行ラインの横位置 (左正)。曲率を前後 60 m でならし、イン側へ寄せる (幅いっぱい − 2.5 m まで)
        float[] RacingLine(float half)
        {
            int n = m_C.Length;
            float step = m_S[n] / n;
            int w = Mathf.Max(1, Mathf.RoundToInt(60f / step));
            var o = new float[n];
            for (int i = 0; i < n; i++)
            {
                float s = 0f;
                for (int d = -w; d <= w; d++) s += m_K[(i + d + n) % n];
                s /= 2 * w + 1;
                o[i] = Mathf.Clamp(s * 450f, -(half - 2.5f), half - 2.5f);
            }
            // さらに軽くならして、ラインが折れないように
            var o2 = new float[n];
            for (int i = 0; i < n; i++)
            {
                float s = 0f;
                for (int d = -10; d <= 10; d++) s += o[(i + d + n) % n];
                o2[i] = s / 21f;
            }
            return o2;
        }

        /// 300 m 以上ほぼまっすぐな区間の終わり (コーナーの入口) の手前 len [m] を true にする
        bool[] BrakeZones(float len)
        {
            int n = m_C.Length;
            var z = new bool[n];
            foreach (int e in CornerEntries())
                for (int k = 0; k < n; k++)
                {
                    float ds = m_S[e] - m_S[k];
                    if (ds < 0f) ds += m_S[n];
                    if (ds <= len) z[k] = true;
                }
            return z;
        }

        List<int> CornerEntries()
        {
            int n = m_C.Length;
            const float kStraight = 1f / 600f;
            float run = 0f;
            var entries = new List<int>();
            for (int k = 0; k < 2 * n; k++)
            {
                int i = k % n, j = (i + 1) % n;
                float ds = (m_C[j] - m_C[i]).magnitude;
                if (Mathf.Abs(m_K[i]) < kStraight) run += ds;
                else
                {
                    if (run >= 300f && k >= n && !entries.Contains(i)) entries.Add(i);
                    run = 0f;
                }
            }
            return entries;
        }

        // コントロールライン: 中心線の始点を横切る市松 (幅 1.2 m)
        void BuildControlLine(float half)
        {
            Vector2 t = (m_C[1] - m_C[0]).normalized, nrm = m_N[0];
            float yaw = Mathf.Atan2(t.y, t.x);
            var white = Lit(new Color32(232, 232, 228, 255), null, 0.4f);
            var black = Lit(new Color32(20, 20, 22, 255), null, 0.4f);
            int cells = Mathf.Max(2, Mathf.RoundToInt(half * 2f / 0.6f));
            float cw = half * 2f / cells;
            for (int r = 0; r < 2; r++)
                for (int k = 0; k < cells; k++)
                {
                    float off = -half + (k + 0.5f) * cw;
                    Vector2 p = m_C[0] + nrm * off + t * ((r - 0.5f) * 0.6f);
                    Box("ControlLine", RosFrame.ToUnity(p.x, p.y, 0.03f), new Vector3(cw, 0.01f, 0.6f),
                        RosFrame.Yaw(yaw), ((k + r) & 1) == 0 ? white : black);
                }
        }

        // 減速の目安の距離板 (300 / 200 / 100 m)。コーナーの入口の手前、外側に立てる
        void BuildBrakeBoards(float off)
        {
            int n = m_C.Length;
            float total = m_S[n];
            var post = Lit(new Color32(60, 60, 62, 255), null, 0.3f);
            foreach (int e in CornerEntries())
            {
                float side = m_K[(e + 3) % n] > 0f ? -1f : 1f;      // 外側 = 曲がる向きの反対
                foreach (int d in new[] { 300, 200, 100 })
                {
                    float sTarget = m_S[e] - d;
                    if (sTarget < 0f) sTarget += total;
                    int i = Array.BinarySearch(m_S, sTarget);
                    if (i < 0) i = ~i;
                    i = Mathf.Clamp(i, 0, n - 1);
                    Vector2 p = m_C[i] + m_N[i] * side * off;
                    Vector2 t = (m_C[(i + 1) % n] - m_C[i]).normalized;
                    var tex = LabelTexture.Make(d.ToString(), new Color32(20, 20, 20, 255), new Color32(240, 240, 236, 255));
                    var face = new Material(m_Unlit) { mainTexture = tex };
                    Vector3 c = RosFrame.ToUnity(p.x, p.y, 1.6f);
                    Vector3 fwd = RosFrame.ToUnity(-t.x, -t.y, 0f).normalized;      // 走ってくる車の方を向く
                    Vector3 right = Vector3.Cross(Vector3.up, fwd).normalized;
                    float w = 1.6f, hgt = 1.0f;
                    var mesh = QuadMesh(c - right * w * 0.5f + Vector3.up * hgt * 0.5f, c + right * w * 0.5f + Vector3.up * hgt * 0.5f,
                                        c + right * w * 0.5f - Vector3.up * hgt * 0.5f, c - right * w * 0.5f - Vector3.up * hgt * 0.5f, fwd);
                    MeshObject($"Board{d}", mesh, face);
                    Box($"BoardPost{d}", RosFrame.ToUnity(p.x, p.y, 0.55f), new Vector3(0.12f, 1.1f, 0.12f), Quaternion.identity, post);
                }
            }
        }

        // 観客席 (コントロールライン付近の外側) とピット棟 (内側)。メインストレートの向きに沿って置く
        void BuildPaddock(float bar)
        {
            int n = m_C.Length;
            Vector2 t = (m_C[1] - m_C[0]).normalized, nrm = m_N[0];
            float yaw = Mathf.Atan2(t.y, t.x);
            // コースの重心の側が内側
            Vector2 mid = Vector2.zero;
            foreach (var p in m_C) mid += p;
            mid /= n;
            float inSide = Vector2.Dot(mid - m_C[0], nrm) > 0f ? 1f : -1f;
            Quaternion rot = RosFrame.Yaw(yaw);

            // 観客席: 階段状の 14 段、長さ 260 m。段の上面に観客、上に屋根
            var crowd = ProcTex.Material(m_Lit, ProcTex.Crowd(91), null, 0.05f, 0f, new Vector2(8f, 8f));
            var concrete = Lit(new Color32(150, 150, 146, 255), Speckle(128, 0.25f, 92), 0.1f, new Vector2(20f, 2f));
            var roof = Lit(new Color32(205, 208, 212, 255), null, 0.6f);
            float side = -inSide, len = 260f;
            for (int r = 0; r < 14; r++)
            {
                float d = bar + 6f + r * 0.9f, z = 0.4f + r * 0.45f;
                Vector2 p = m_C[0] + nrm * side * d;
                Box("StandStep", RosFrame.ToUnity(p.x, p.y, z * 0.5f), new Vector3(0.9f, z, len), rot, concrete);
                var top = Box("StandCrowd", RosFrame.ToUnity(p.x, p.y, z + 0.02f), new Vector3(0.86f, 0.04f, len), rot, crowd);
                top.GetComponent<Renderer>().sharedMaterial.mainTextureScale = new Vector2(1f, len / 8f);
            }
            Vector2 rp = m_C[0] + nrm * side * (bar + 12f);
            Box("StandRoof", RosFrame.ToUnity(rp.x, rp.y, 10.5f), new Vector3(16f, 0.3f, len + 6f), rot, roof);
            for (int k = -4; k <= 4; k++)
            {
                Vector2 pp = m_C[0] + nrm * side * (bar + 19f) + t * (k * len / 8.5f);
                Box("StandPillar", RosFrame.ToUnity(pp.x, pp.y, 5.3f), new Vector3(0.4f, 10.6f, 0.4f), rot, roof);
            }

            // ピット棟: 長さ 300 m・奥行 14 m・高さ 9 m。ガレージの扉の帯
            var wall = Lit(new Color32(196, 198, 202, 255), Speckle(128, 0.15f, 93), 0.3f, new Vector2(30f, 1f));
            var door = Lit(new Color32(48, 52, 60, 255), null, 0.5f);
            Vector2 pb = m_C[0] + nrm * inSide * (bar + 22f);
            Box("PitBuilding", RosFrame.ToUnity(pb.x, pb.y, 4.5f), new Vector3(14f, 9f, 300f), rot, wall);
            Vector2 pd = m_C[0] + nrm * inSide * (bar + 14.9f);
            Box("PitDoors", RosFrame.ToUnity(pd.x, pd.y, 2.2f), new Vector3(0.2f, 4.2f, 296f), rot, door);
        }

        // 地形 (色の地図 × 芝の細部)・森 (十字の板の木)・空の雲。中身は Landscape.cs
        void BuildLandscape(Landscape land)
        {
            ProcTex.GrassDetail(512, 21, out var detA, out var detN);
            var ground = new Material(m_Lit) { color = new Color(0.80f, 0.78f, 0.74f), mainTexture = land.ColorMap(2048) };   // 芝が明るく鮮やかに出すぎるのを抑える
            ground.SetFloat("_Glossiness", 0.06f);
            ground.SetFloat("_Metallic", 0f);
            ground.SetTexture("_BumpMap", ProcTex.FlatNormal());
            ground.EnableKeyword("_NORMALMAP");
            ground.SetTexture("_DetailAlbedoMap", detA);
            ground.SetTexture("_DetailNormalMap", detN);
            ground.SetFloat("_DetailNormalMapScale", 0.6f);
            ground.SetTextureScale("_DetailAlbedoMap", new Vector2(1f / 12f, 1f / 12f));
            ground.SetFloat("_UVSec", 1f);                 // 細部は uv1 (m 単位)
            ground.EnableKeyword("_DETAIL_MULX2");
            MeshObject("Terrain", land.TerrainMesh(), ground);

            var mats = new Material[6];
            for (int k = 0; k < 6; k++)
            {
                mats[k] = new Material(m_Lit) { color = new Color(0.70f, 0.76f, 0.68f), mainTexture = ProcTex.Tree(k < 3 ? 0 : 1, 500 + k) };
                mats[k].SetFloat("_Glossiness", 0.02f);
                mats[k].SetFloat("_Metallic", 0f);
                ProcTex.MakeCutout(mats[k], 0.45f);
            }
            var trees = land.Trees();
            foreach (var kv in Landscape.TreeMeshes(trees))
            {
                var tg = MeshObject("Trees", kv.Value, mats[kv.Key.Item3]);
                // 森の影は重い (2.6 万本)。-quality low では落とさない (RenderQuality)
                if (!RenderQuality.TreeShadows) tg.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            }
            Debug.Log($"[CourseBuilder] landscape: grid {land.Xs.Length}x{land.Ys.Length}, trees {trees.Count}");

            var vc = Resources.Load<Material>("Mat_VertexColor");
            if (vc != null)
            {
                var sky = new Material(vc) { mainTexture = ProcTex.Clouds(77), renderQueue = 2900 };
                var go = MeshObject("SkyDome", land.SkyDome(), sky);
                var r = go.GetComponent<Renderer>();
                r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                r.receiveShadows = false;
            }
        }

        /// 太陽の向き (ROS 座標・太陽へ向かう単位ベクトル)
        public static readonly Vector3 SunDir = new Vector3(0.42f, 0.64f, 0.643f).normalized;

        void BuildOutdoorLighting()
        {
            var go = new GameObject("Sun");
            var sun = go.AddComponent<Light>();
            m_Ceiling = sun;
            sun.type = LightType.Directional;
            sun.intensity = 1.35f;
            sun.color = new Color(1f, 0.95f, 0.86f);
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 0.8f;
            sun.shadowBias = 0.05f;
            sun.shadowNormalBias = 0.4f;
            // 太陽は富士山の反対側の空 (仰角 40°) から: 富士山の正面に日が当たる朝の景色 (preview_circuit.py の SUN と同じ)
            go.transform.rotation = Quaternion.LookRotation(-RosFrame.ToUnity(SunDir.x, SunDir.y, SunDir.z));
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.50f, 0.58f, 0.72f);
            RenderSettings.ambientEquatorColor = new Color(0.40f, 0.43f, 0.44f);
            RenderSettings.ambientGroundColor = new Color(0.18f, 0.19f, 0.16f);
            RenderSettings.fog = true;           // 遠くを霞ませる (遠景の山とコースのつながり)
            RenderSettings.fogMode = FogMode.Exponential;   // 2 km で 10 %・18 km (富士山) で 63 %
            RenderSettings.fogColor = new Color(0.72f, 0.79f, 0.86f);
            RenderSettings.fogDensity = 0.000055f;
            QualitySettings.shadowDistance = 180f;
            QualitySettings.shadowCascades = 4;
            QualitySettings.anisotropicFiltering = AnisotropicFiltering.ForceEnable;
            // 映り込み: コースの内側から周りの景色 (空・地平線・森) を 1 回だけ写し取り、全体の映り込みに使う。
            // これが無いと車の塗装とガラスに空の一色しか映らず、平板に見える。low (学習) では作らない
            if (RenderQuality.Current != QualityTier.Low)
            {
                // 置き場所は路面の上 (下半分に舗装が映る。芝の上に置くと全部が緑がかる)。観客席から離れた 1/4 周の地点
                var pc = m_C[m_C.Length * 28 / 100];
                var pg = new GameObject("EnvProbe");
                pg.transform.position = RosFrame.ToUnity(pc.x, pc.y, 1.2f);
                var probe = pg.AddComponent<ReflectionProbe>();
                probe.mode = UnityEngine.Rendering.ReflectionProbeMode.Realtime;
                probe.refreshMode = UnityEngine.Rendering.ReflectionProbeRefreshMode.ViaScripting;
                probe.timeSlicingMode = UnityEngine.Rendering.ReflectionProbeTimeSlicingMode.NoTimeSlicing;
                probe.resolution = 256;
                probe.hdr = true;
                probe.size = Vector3.one * 100000f;
                probe.nearClipPlane = 1f;
                probe.farClipPlane = 50000f;
                StartCoroutine(RenderProbeLater(probe));
            }
        }

        System.Collections.IEnumerator RenderProbeLater(ReflectionProbe probe)
        {
            yield return null;          // 地形・森・雲を組み終えてから
            yield return null;
            probe.RenderProbe();
        }
    }
}
