// 実車スケールのサーキット (course.json の kind = "circuit") を組み立てる。形状の定義元は course.py の circuit_course
// (中心線 = centerline_shortcut、幅・ランオフ = circuit)。ここは「どう見えるか」だけ。
//
//   芝の地面 → ランオフ (舗装・明るめ) → コース (アスファルト) → 白線 → 縁石 (曲率の大きいところ・紅白)
//   → バリア (ランオフの外側・高さ 1 m) → コントロールライン (中心線の始点・市松) → 減速の目安の距離板 (300/200/100)
//   → 遠景の山 (形は一般的な成層火山。地名・ロゴは描かない)
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public partial class CourseBuilder
    {
        static readonly Color32 kAsphalt = new Color32(62, 64, 68, 255);
        static readonly Color32 kRunoff = new Color32(96, 98, 100, 255);
        static readonly Color32 kGrass = new Color32(74, 104, 56, 255);
        static readonly Color32 kLine = new Color32(232, 232, 228, 255);
        static readonly Color32 kKerbRed = new Color32(200, 36, 32, 255);
        static readonly Color32 kBarrier = new Color32(176, 178, 182, 255);
        static readonly Color32 kMountain = new Color32(70, 82, 104, 255);
        static readonly Color32 kSnow = new Color32(236, 240, 246, 255);

        Vector2[] m_C;          // 中心線 (閉ループ・始点を末尾に重ねない)
        Vector2[] m_N;          // 左向きの法線
        float[] m_K;            // 符号つき曲率 (+左)
        float[] m_S;            // 弧長

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
            float half = c.width_m * 0.5f;
            float bar = half + c.runoff_m;
            var b = CircuitBounds;

            // 地面 (芝)。外接矩形 + 1 km
            var grassTex = Speckle(256, 0.30f, 41);
            var grass = Lit(kGrass, grassTex, 0.05f, new Vector2((b[2] - b[0] + 2000f) / 8f, (b[3] - b[1] + 2000f) / 8f));
            FloorRect("Grass", b[0] - 1000f, b[1] - 1000f, b[2] + 1000f, b[3] + 1000f, -0.05f, grass);

            var asphTex = Speckle(256, 0.22f, 42, true);
            MeshObject("Runoff", Ribbon(-bar, bar, 0.005f, 6f), Lit(kRunoff, asphTex, 0.15f));
            MeshObject("Asphalt", Ribbon(-half, half, 0.02f, 6f), Lit(kAsphalt, asphTex, 0.25f));
            var line = Lit(kLine, null, 0.4f);
            MeshObject("EdgeL", Ribbon(half - 0.30f, half - 0.05f, 0.03f, 6f), line);
            MeshObject("EdgeR", Ribbon(-half + 0.05f, -half + 0.30f, 0.03f, 6f), line);
            BuildKerbs(half, c.kerb_width_m, c.kerb_min_curvature);
            var barrier = Lit(kBarrier, null, 0.5f);
            MeshObject("BarrierL", Fence(bar, c.barrier_height_m), barrier);
            MeshObject("BarrierR", Fence(-bar, c.barrier_height_m), barrier);
            BuildControlLine(half);
            BuildBrakeBoards(half + 2.5f);
            BuildMountain(b);
            BuildOutdoorLighting(b);
        }

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

        Vector3 At(int i, float off, float z)
        {
            Vector2 p = m_C[i] + m_N[i] * off;
            return RosFrame.ToUnity(p.x, p.y, z);
        }

        // 中心線から左 off0〜off1 [m] の帯。v は弧長 / vTile で繰り返す
        Mesh Ribbon(float off0, float off1, float z, float vTile)
        {
            int n = m_C.Length;
            var v = new Vector3[(n + 1) * 2];
            var uv = new Vector2[(n + 1) * 2];
            var tri = new int[n * 6];
            float w = Mathf.Abs(off1 - off0);
            for (int k = 0; k <= n; k++)
            {
                int i = k % n;
                v[2 * k] = At(i, off0, z);
                v[2 * k + 1] = At(i, off1, z);
                uv[2 * k] = new Vector2(0f, m_S[k] / vTile);
                uv[2 * k + 1] = new Vector2(w / vTile, m_S[k] / vTile);
            }
            for (int k = 0; k < n; k++)
            {
                int a = 2 * k, t = k * 6;
                // off1 > off0 (off1 が左) のとき上向き。cross(b − a, c − a) が表
                tri[t] = a; tri[t + 1] = a + 1; tri[t + 2] = a + 2;
                tri[t + 3] = a + 1; tri[t + 4] = a + 3; tri[t + 5] = a + 2;
            }
            return Finish(v, uv, tri, Vector3.up);
        }

        // 中心線から左 off [m] に立つ高さ h の塀。表はコース側 (カメラは常にコースの内側にある)
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
                uv[2 * k] = new Vector2(m_S[k] / 4f, 0f);
                uv[2 * k + 1] = new Vector2(m_S[k] / 4f, 1f);
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
            return Finish(v, uv, tri, Vector3.zero);
        }

        static Mesh Finish(Vector3[] v, Vector2[] uv, int[] tri, Vector3 up)
        {
            var m = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            m.vertices = v;
            m.uv = uv;
            m.triangles = tri;
            if (up != Vector3.zero)
            {
                var nr = new Vector3[v.Length];
                for (int i = 0; i < nr.Length; i++) nr[i] = up;
                m.normals = nr;
            }
            else m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }

        // 縁石: 曲率が kMin を超える区間の両側に、紅白 (3 m ごと) の帯
        void BuildKerbs(float half, float width, float kMin)
        {
            int n = m_C.Length;
            var red = new List<int>[2] { new List<int>(), new List<int>() };
            var white = new List<int>[2] { new List<int>(), new List<int>() };
            var vRed = new List<Vector3>[] { new List<Vector3>(), new List<Vector3>() };
            var vWhite = new List<Vector3>[] { new List<Vector3>(), new List<Vector3>() };
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                // 曲がり始め・終わりの少し外側まで伸ばす (前後 3 点の最大で判定)
                float kk = 0f;
                for (int d = -3; d <= 3; d++) kk = Mathf.Max(kk, Mathf.Abs(m_K[(i + d + n) % n]));
                if (kk < kMin) continue;
                bool isRed = ((int)(m_S[i] / 3f) & 1) == 0;
                for (int side = 0; side < 2; side++)
                {
                    float s = side == 0 ? 1f : -1f;
                    var vl = isRed ? vRed[side] : vWhite[side];
                    var tl = isRed ? red[side] : white[side];
                    int b0 = vl.Count;
                    vl.Add(At(i, s * half, 0.035f)); vl.Add(At(i, s * (half + width), 0.035f));
                    vl.Add(At(j, s * half, 0.035f)); vl.Add(At(j, s * (half + width), 0.035f));
                    // 左 (外側の頂点が左) は Ribbon と同じ向き、右は逆向きで上を表にする
                    if (side == 0) { tl.Add(b0); tl.Add(b0 + 1); tl.Add(b0 + 2); tl.Add(b0 + 1); tl.Add(b0 + 3); tl.Add(b0 + 2); }
                    else { tl.Add(b0); tl.Add(b0 + 2); tl.Add(b0 + 1); tl.Add(b0 + 1); tl.Add(b0 + 2); tl.Add(b0 + 3); }
                }
            }
            var mr = Lit(kKerbRed, null, 0.35f);
            var mw = Lit(kLine, null, 0.35f);
            for (int side = 0; side < 2; side++)
            {
                if (vRed[side].Count > 0) MeshObject("KerbRed", Finish(vRed[side].ToArray(), new Vector2[vRed[side].Count], red[side].ToArray(), Vector3.up), mr);
                if (vWhite[side].Count > 0) MeshObject("KerbWhite", Finish(vWhite[side].ToArray(), new Vector2[vWhite[side].Count], white[side].ToArray(), Vector3.up), mw);
            }
        }

        // コントロールライン: 中心線の始点を横切る市松 (幅 1.2 m)
        void BuildControlLine(float half)
        {
            Vector2 t = (m_C[1] - m_C[0]).normalized, nrm = m_N[0];
            float yaw = Mathf.Atan2(t.y, t.x);
            var white = Lit(kLine, null, 0.4f);
            var black = Lit(new Color32(20, 20, 22, 255), null, 0.4f);
            int cells = Mathf.Max(2, Mathf.RoundToInt(half * 2f / 0.6f));
            float cw = half * 2f / cells;
            for (int r = 0; r < 2; r++)
                for (int k = 0; k < cells; k++)
                {
                    float off = -half + (k + 0.5f) * cw;
                    Vector2 p = m_C[0] + nrm * off + t * ((r - 0.5f) * 0.6f);
                    Box("ControlLine", RosFrame.ToUnity(p.x, p.y, 0.025f), new Vector3(cw, 0.01f, 0.6f),
                        RosFrame.Yaw(yaw), ((k + r) & 1) == 0 ? white : black);
                }
        }

        // 減速の目安の距離板 (300 / 200 / 100 m)。300 m 以上ほぼまっすぐな区間の終わり (コーナーの入口) の手前、外側に立てる
        void BuildBrakeBoards(float off)
        {
            int n = m_C.Length;
            float total = m_S[n];
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
            var post = Lit(new Color32(60, 60, 62, 255), null, 0.3f);
            foreach (int e in entries)
            {
                // 外側 = 曲がる向きの反対
                float side = m_K[(e + 3) % n] > 0f ? -1f : 1f;
                foreach (int d in new[] { 300, 200, 100 })
                {
                    float sTarget = m_S[e] - d;
                    if (sTarget < 0f) sTarget += total;
                    int i = System.Array.BinarySearch(m_S, sTarget);
                    if (i < 0) i = ~i;
                    i = Mathf.Clamp(i, 0, n - 1);
                    Vector2 p = m_C[i] + m_N[i] * side * off;
                    Vector2 t = (m_C[(i + 1) % n] - m_C[i]).normalized;
                    var tex = LabelTexture.Make(d.ToString(), new Color32(20, 20, 20, 255), new Color32(240, 240, 236, 255));
                    var face = new Material(m_Unlit) { mainTexture = tex };
                    // 板は走ってくる車の方を向く (進行方向の逆を法線に)
                    Vector3 c = RosFrame.ToUnity(p.x, p.y, 1.6f);
                    Vector3 fwd = RosFrame.ToUnity(-t.x, -t.y, 0f).normalized;
                    Vector3 right = Vector3.Cross(Vector3.up, fwd).normalized;
                    float w = 1.6f, hgt = 1.0f;
                    var mesh = QuadMesh(c - right * w * 0.5f + Vector3.up * hgt * 0.5f, c + right * w * 0.5f + Vector3.up * hgt * 0.5f,
                                        c + right * w * 0.5f - Vector3.up * hgt * 0.5f, c - right * w * 0.5f - Vector3.up * hgt * 0.5f, fwd);
                    MeshObject($"Board{d}", mesh, face);
                    Box($"BoardPost{d}", RosFrame.ToUnity(p.x, p.y, 0.55f), new Vector3(0.12f, 1.1f, 0.12f), Quaternion.identity, post);
                }
            }
        }

        // 遠景の山: コースの北西 6 km に高さ 1.6 km の円錐 (見かけの大きさが分かる程度)。雪の帽子つき
        void BuildMountain(float[] b)
        {
            Vector2 mid = new Vector2((b[0] + b[2]) * 0.5f, (b[1] + b[3]) * 0.5f);
            Vector2 p = mid + new Vector2(-4200f, 4300f);
            MeshObject("Mountain", Cone(p, 0f, 1600f, 5200f, 900f), Lit(kMountain, null, 0.05f));
            MeshObject("MountainSnow", Cone(p, 1150f, 1600f, 5200f * 450f / 1600f, 900f * 450f / 1600f), Lit(kSnow, null, 0.2f));
        }

        // 裾広がりの円錐 (頂上は平ら気味)。z0〜z1 の部分だけ。rBase は z=0 の半径、rTop は z=z1 での半径
        static Mesh Cone(Vector2 c, float z0, float z1, float rBase, float rTop)
        {
            const int seg = 48;
            float r0 = z0 <= 0f ? rBase : Mathf.Lerp(rBase, rTop, z0 / z1);
            var v = new List<Vector3>();
            var tri = new List<int>();
            for (int i = 0; i <= seg; i++)
            {
                float a = i * Mathf.PI * 2f / seg;
                // 稜線を少し波打たせる (単調な円錐に見えないように)
                float wob = 1f + 0.04f * Mathf.Sin(a * 5f) + 0.03f * Mathf.Sin(a * 11f + 1f);
                v.Add(RosFrame.ToUnity(c.x + Mathf.Cos(a) * r0 * wob, c.y + Mathf.Sin(a) * r0 * wob, z0));
                v.Add(RosFrame.ToUnity(c.x + Mathf.Cos(a) * rTop * wob, c.y + Mathf.Sin(a) * rTop * wob, z1));
            }
            for (int i = 0; i < seg; i++)
            {
                int a = 2 * i;
                tri.AddRange(new[] { a, a + 1, a + 2, a + 1, a + 3, a + 2 });
            }
            int top = v.Count;
            v.Add(RosFrame.ToUnity(c.x, c.y, z1 + 20f));
            for (int i = 0; i < seg; i++) tri.AddRange(new[] { 2 * i + 1, top, 2 * i + 3 });
            var m = new Mesh();
            m.SetVertices(v);
            m.SetTriangles(tri, 0);
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }

        void BuildOutdoorLighting(float[] b)
        {
            var go = new GameObject("Sun");
            var sun = go.AddComponent<Light>();
            m_Ceiling = sun;
            sun.type = LightType.Directional;
            sun.intensity = 1.15f;
            sun.color = new Color(1f, 0.96f, 0.88f);
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 0.75f;
            sun.shadowBias = 0.05f;
            sun.shadowNormalBias = 0.4f;
            go.transform.rotation = Quaternion.Euler(48f, -35f, 0f);
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.62f, 0.70f, 0.82f);
            RenderSettings.ambientEquatorColor = new Color(0.52f, 0.55f, 0.55f);
            RenderSettings.ambientGroundColor = new Color(0.26f, 0.28f, 0.22f);
            RenderSettings.fog = true;           // 遠くを霞ませる (遠景の山とコースのつながり)
            RenderSettings.fogMode = FogMode.Linear;
            RenderSettings.fogColor = new Color(0.72f, 0.79f, 0.86f);
            RenderSettings.fogStartDistance = 600f;
            RenderSettings.fogEndDistance = 9000f;
            QualitySettings.shadowDistance = 150f;
            QualitySettings.shadowCascades = 4;
        }
    }
}
