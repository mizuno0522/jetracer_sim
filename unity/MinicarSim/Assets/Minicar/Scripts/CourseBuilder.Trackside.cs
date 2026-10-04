// サーキットのコース脇: コーナーの外側の金網 (デブリフェンス) と支柱、バリアの後ろの看板。
// 見た目だけで、物理 (vehicle_sim の壁) には関係しない。看板は色の板だけ (文字・ロゴは入れていない)。
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public partial class CourseBuilder
    {
        const float kFenceOffset = 1.6f, kFenceHeight = 3.6f;      // バリアの外 1.6 m に高さ 3.6 m
        const float kBoardOffset = 0.45f, kBoardLo = 0.10f, kBoardHi = 1.00f;

        /// i のあたりがコーナーで、side (+1 = 左) がその外側か
        bool CornerOutside(int i, float side)
        {
            int n = m_C.Length;
            float kmax = 0f, sum = 0f;
            for (int d = -14; d <= 14; d++)
            {
                float k = m_K[((i + d) % n + n) % n];
                kmax = Mathf.Max(kmax, Mathf.Abs(k));
                sum += k;
            }
            return kmax > 1f / 330f && Mathf.Sign(sum) == -side;
        }

        void BuildTrackside(float bar)
        {
            int n = m_C.Length;
            var wire = new Material(m_Lit) { color = new Color(0.78f, 0.80f, 0.83f, 1f), mainTexture = ProcTex.WireMesh(128) };
            wire.SetFloat("_Glossiness", 0.25f);
            wire.SetFloat("_Metallic", 0.3f);
            ProcTex.MakeFade(wire);
            var steel = new Material(m_Lit) { color = new Color(0.52f, 0.54f, 0.57f) };
            steel.SetFloat("_Glossiness", 0.45f);
            steel.SetFloat("_Metallic", 0.6f);
            Color[] pal = { new Color(0.92f, 0.92f, 0.90f), new Color(0.80f, 0.10f, 0.10f), new Color(0.10f, 0.25f, 0.62f),
                            new Color(0.95f, 0.78f, 0.10f), new Color(0.08f, 0.08f, 0.09f) };
            var fv = new List<Vector3>(); var fu = new List<Vector2>(); var ft = new List<int>();
            var pv = new List<Vector3>(); var pt = new List<int>();
            var bv = new List<Vector3>[pal.Length]; var bt = new List<int>[pal.Length];
            for (int k = 0; k < pal.Length; k++) { bv[k] = new List<Vector3>(); bt[k] = new List<int>(); }

            foreach (float side in new[] { 1f, -1f })
                for (int i = 0; i < n; i += 2)
                {
                    if (!CornerOutside(i, side)) continue;
                    int j = (i + 2) % n;
                    Vector2 a = m_C[i] + m_N[i] * side * (bar + kFenceOffset), b = m_C[j] + m_N[j] * side * (bar + kFenceOffset);
                    float u0 = m_S[i] / 0.6f, u1 = u0 + (b - a).magnitude / 0.6f;
                    Quad(fv, ft, fu, RosFrame.ToUnity(a.x, a.y, 0f), RosFrame.ToUnity(b.x, b.y, 0f),
                         RosFrame.ToUnity(b.x, b.y, kFenceHeight), RosFrame.ToUnity(a.x, a.y, kFenceHeight), u0, u1, kFenceHeight / 0.6f);
                    Post(pv, pt, a, 0.06f, kFenceHeight + 0.15f);
                    // 看板: 4 枚並べて 1 枚あける。色は 8 枚ごとに変える
                    if ((i / 2) % 5 == 4) continue;
                    Vector2 c0 = m_C[i] + m_N[i] * side * (bar + kBoardOffset), c1 = m_C[j] + m_N[j] * side * (bar + kBoardOffset);
                    int col = (i / 20) % pal.Length;
                    Quad(bv[col], bt[col], null, RosFrame.ToUnity(c0.x, c0.y, kBoardLo), RosFrame.ToUnity(c1.x, c1.y, kBoardLo),
                         RosFrame.ToUnity(c1.x, c1.y, kBoardHi), RosFrame.ToUnity(c0.x, c0.y, kBoardHi), 0f, 1f, 1f);
                }
            if (fv.Count == 0) return;
            MeshObject("CatchFence", ToMesh(fv, fu, ft), wire).GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            MeshObject("FencePosts", ToMesh(pv, null, pt), steel);
            for (int k = 0; k < pal.Length; k++)
            {
                if (bv[k].Count == 0) continue;
                var m = new Material(m_Lit) { color = pal[k] };
                m.SetFloat("_Glossiness", 0.35f);
                MeshObject("Boards", ToMesh(bv[k], null, bt[k]), m);
            }
        }

        /// 両面の四角
        static void Quad(List<Vector3> v, List<int> t, List<Vector2> uv, Vector3 a, Vector3 b, Vector3 c, Vector3 d, float u0, float u1, float vTop)
        {
            for (int f = 0; f < 2; f++)
            {
                int o = v.Count;
                v.Add(a); v.Add(b); v.Add(c); v.Add(d);
                if (uv != null) { uv.Add(new Vector2(u0, 0f)); uv.Add(new Vector2(u1, 0f)); uv.Add(new Vector2(u1, vTop)); uv.Add(new Vector2(u0, vTop)); }
                if (f == 0) { t.Add(o); t.Add(o + 1); t.Add(o + 2); t.Add(o); t.Add(o + 2); t.Add(o + 3); }
                else { t.Add(o); t.Add(o + 2); t.Add(o + 1); t.Add(o); t.Add(o + 3); t.Add(o + 2); }
            }
        }

        /// 四角い支柱 (ROS の位置 p、太さ w、高さ h)
        static void Post(List<Vector3> v, List<int> t, Vector2 p, float w, float h)
        {
            float r = w * 0.5f;
            Vector2[] q = { new Vector2(-r, -r), new Vector2(r, -r), new Vector2(r, r), new Vector2(-r, r) };
            for (int k = 0; k < 4; k++)
            {
                Vector2 a = p + q[k], b = p + q[(k + 1) % 4];
                Quad(v, t, null, RosFrame.ToUnity(a.x, a.y, 0f), RosFrame.ToUnity(b.x, b.y, 0f), RosFrame.ToUnity(b.x, b.y, h), RosFrame.ToUnity(a.x, a.y, h), 0f, 1f, 1f);
            }
        }

        static Mesh ToMesh(List<Vector3> v, List<Vector2> uv, List<int> t)
        {
            var m = new Mesh();
            if (v.Count > 65000) m.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            m.SetVertices(v);
            if (uv != null) m.SetUVs(0, uv);
            m.SetTriangles(t, 0);
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }
    }
}
