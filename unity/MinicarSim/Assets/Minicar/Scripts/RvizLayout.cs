// RViz (minicar_sim/rviz/sim_view.rviz) と同じ表示仕様の画面レイアウト。
//   左  = 車載カメラ画像 (/camera/image_raw と同じセンサカメラ)
//   右  = コース全景 (真上から) ＋ 走行軌跡の速度色 ＋ 速度の凡例バー
// 速度色・軌跡の点間隔・点数・凡例の位置は sim_viz と同じ値を course.json (= sim.yaml) から取る。
using System.Collections.Generic;
using RosMessageTypes.Nav;
using UnityEngine;

namespace Minicar
{
    public class RvizLayout
    {
        public const int OverheadLayer = 10;   // トンネル天井・ライト架台など。全景カメラには写さない
        public const int OverviewOnlyLayer = 11; // 凡例バー。車載カメラには写さない
        public const int SensorOnlyLayer = 12;   // 会場の床。全景では RViz と同じ暗い背景にする

        const float LeftFrac = 0.36f;          // 左パネルの幅 (RViz のカメラパネル相当)
        static readonly Color32 kPanelBg = new Color32(40, 40, 42, 255);   // sim.rviz の Background Color

        readonly CourseData m_Data;
        readonly Camera m_Cam;
        readonly Material m_VertexMat;
        readonly List<Trail> m_Trails = new List<Trail>();
        readonly Mesh[] m_Grids = new Mesh[2];   // 0 = 自車, 1 = 2 台レースの相手
        Texture2D m_White;
        GUIStyle m_Label, m_Title;

        class Trail
        {
            public readonly List<Vector3> Pts = new List<Vector3>();
            public readonly List<Color> Cols = new List<Color>();
            public Mesh Mesh;
            public double LastDistance = -1;
            public bool Dirty;
        }

        public RvizLayout(CourseData data)
        {
            m_Data = data;
            m_VertexMat = Resources.Load<Material>("Mat_VertexColor");

            var go = new GameObject("OverviewCamera");
            m_Cam = go.AddComponent<Camera>();
            m_Cam.orthographic = true;
            m_Cam.clearFlags = CameraClearFlags.SolidColor;
            m_Cam.backgroundColor = kPanelBg;
            m_Cam.nearClipPlane = 0.1f;
            m_Cam.farClipPlane = 40f;
            m_Cam.cullingMask = ~((1 << OverheadLayer) | (1 << SensorOnlyLayer));
            // 画面右 = ROS +x、画面上 = ROS +y (RViz の TopDownOrtho と同じ向き)
            m_Cam.transform.rotation = Quaternion.LookRotation(Vector3.down, new Vector3(-1f, 0f, 0f));
            m_Cam.depth = 1;

            BuildLegend();
        }

        public bool Enabled
        {
            get => m_Cam.enabled;
            set => m_Cam.enabled = value;
        }

        // RViz sim_viz.speed_color と同じ: 青(遅い) → 緑(中) → 赤(速い)
        public static Color SpeedColor(float speed, float vmax)
        {
            float t = Mathf.Clamp01(speed / Mathf.Max(0.1f, vmax));
            if (t < 0.5f)
            {
                float a = t / 0.5f;
                return new Color(0.20f + 0.10f * a, 0.40f + 0.40f * a, 0.90f - 0.60f * a);
            }
            float b = (t - 0.5f) / 0.5f;
            return new Color(0.30f + 0.60f * b, 0.80f - 0.50f * b, 0.30f - 0.10f * b);
        }

        // ------------------------------------------------------------------
        // index 0 = 自車、1 = 2 台レースの相手
        public void AddSample(int index, double x, double y, double speed, double distance)
        {
            while (m_Trails.Count <= index) m_Trails.Add(new Trail());
            var tr = m_Trails[index];
            // /odom/reset (レース開始) で走行距離が巻き戻ったら軌跡も消す
            if (distance < tr.LastDistance - 0.5) { tr.Pts.Clear(); tr.Cols.Clear(); tr.Dirty = true; }
            tr.LastDistance = distance;

            Vector3 p = RosFrame.ToUnity(x, y, 0.03);
            if (tr.Pts.Count > 0 && Vector3.Distance(tr.Pts[tr.Pts.Count - 1], p) < m_Data.viz.trail_min_dist_m) return;
            tr.Pts.Add(p);
            tr.Cols.Add(SpeedColor(Mathf.Abs((float)speed), m_Data.viz.speed_color_max_mps));
            int max = Mathf.Max(2, m_Data.viz.trail_max_points);
            if (tr.Pts.Count > max) { tr.Pts.RemoveAt(0); tr.Cols.RemoveAt(0); }
            tr.Dirty = true;
        }

        public void Update()
        {
            if (!m_Cam.enabled) return;
            FitCamera();
            foreach (var tr in m_Trails)
            {
                if (tr.Dirty) { RebuildTrail(tr); tr.Dirty = false; }
                if (tr.Mesh != null && m_VertexMat != null)
                    Graphics.DrawMesh(tr.Mesh, Matrix4x4.identity, m_VertexMat, 0, m_Cam);
            }
            foreach (var grid in m_Grids)
                if (grid != null && m_VertexMat != null)
                    Graphics.DrawMesh(grid, Matrix4x4.identity, m_VertexMat, 0, m_Cam);
        }

        // 占有格子 (/fusion/local_map, base_link 基準)。自車は sim_viz と同じ色:
        // 占有 (≥50) = 赤紫 0.85 / 空き (0〜49) = 薄い水色 0.18 / 未知 (-1) = 描かない。
        // 2 台レースの相手 (index 1) は重なっても見分けられるよう、占有を黄・空きを薄い黄にする。
        // 格子を作った時刻の車体姿勢 (x, y, yaw) で世界座標へ置く。
        public void SetGrid(int index, OccupancyGridMsg g, double px, double py, double yaw)
        {
            int w = (int)g.info.width, h = (int)g.info.height;
            float res = g.info.resolution;
            if (g.data == null || g.data.Length < w * h || res <= 0f) return;
            double ox = g.info.origin.position.x, oy = g.info.origin.position.y;
            double c = System.Math.Cos(yaw), s = System.Math.Sin(yaw);
            var occ = index == 0 ? new Color(0.95f, 0.15f, 0.75f, 0.85f) : new Color(0.98f, 0.78f, 0.10f, 0.85f);
            var free = index == 0 ? new Color(0.30f, 0.85f, 0.95f, 0.18f) : new Color(0.98f, 0.90f, 0.40f, 0.14f);
            var v = new List<Vector3>(w * h);
            var col = new List<Color>(w * h);
            var idx = new List<int>(w * h);
            float half = res * 0.48f;
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++)
                {
                    sbyte d = g.data[j * w + i];
                    if (d < 0) continue;
                    Color cc = d >= 50 ? occ : free;
                    double bx = ox + (i + 0.5) * res, by = oy + (j + 0.5) * res;
                    int b = v.Count;
                    // セルは車体座標で軸平行 → 4 隅を回して置く
                    foreach (var (dx, dy) in new[] { (-half, -half), (half, -half), (-half, half), (half, half) })
                    {
                        double lx = bx + dx, ly = by + dy;
                        v.Add(RosFrame.ToUnity(px + c * lx - s * ly, py + s * lx + c * ly, 0.025));
                        col.Add(cc);
                    }
                    idx.Add(b); idx.Add(b + 1); idx.Add(b + 2);
                    idx.Add(b + 1); idx.Add(b + 3); idx.Add(b + 2);
                }
            if (index < 0 || index >= m_Grids.Length) return;
            var mesh = m_Grids[index];
            if (mesh == null) mesh = m_Grids[index] = new Mesh { name = "FusionGrid" + index, indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            mesh.Clear();
            mesh.SetVertices(v);
            mesh.SetColors(col);
            mesh.SetTriangles(idx, 0);
            mesh.RecalculateBounds();
        }

        // 凡例バー (x=-0.75) からコース右端 (x=10.4)、駐車枠 (y≈0) から上辺 (y≈6.4) までを右パネルに収める
        void FitCamera()
        {
            m_Cam.rect = new Rect(LeftFrac, 0f, 1f - LeftFrac, 1f);
            const float x0 = -1.45f, x1 = 10.55f, y0 = -0.35f, y1 = 6.75f;
            float w = x1 - x0, h = y1 - y0;
            float aspect = (Screen.width * (1f - LeftFrac)) / Mathf.Max(1f, Screen.height);
            m_Cam.orthographicSize = Mathf.Max(h * 0.5f, w * 0.5f / aspect) * 1.02f;
            m_Cam.transform.position = RosFrame.ToUnity((x0 + x1) * 0.5f, (y0 + y1) * 0.5f, 20f);
        }

        void RebuildTrail(Trail tr)
        {
            int n = tr.Pts.Count;
            if (tr.Mesh == null) tr.Mesh = new Mesh { name = "Trail" };
            tr.Mesh.Clear();
            if (n < 2) return;
            const float halfW = 0.02f;           // RViz の LINE_STRIP 幅 0.04 m 相当
            var v = new Vector3[n * 2];
            var c = new Color[n * 2];
            var idx = new int[(n - 1) * 6];
            for (int i = 0; i < n; i++)
            {
                Vector3 dir = (i < n - 1 ? tr.Pts[i + 1] - tr.Pts[i] : tr.Pts[i] - tr.Pts[i - 1]);
                dir.y = 0f;
                Vector3 side = Vector3.Cross(Vector3.up, dir.sqrMagnitude > 1e-8f ? dir.normalized : Vector3.forward) * halfW;
                v[i * 2] = tr.Pts[i] - side;
                v[i * 2 + 1] = tr.Pts[i] + side;
                c[i * 2] = c[i * 2 + 1] = tr.Cols[i];
            }
            for (int i = 0; i < n - 1; i++)
            {
                int k = i * 6, a = i * 2;
                // 上から見て表になる向き (両面描画のシェーダなので向きは問わないが揃えておく)
                idx[k] = a; idx[k + 1] = a + 1; idx[k + 2] = a + 2;
                idx[k + 3] = a + 1; idx[k + 4] = a + 3; idx[k + 5] = a + 2;
            }
            tr.Mesh.vertices = v;
            tr.Mesh.colors = c;
            tr.Mesh.triangles = idx;
            tr.Mesh.RecalculateBounds();
        }

        // 凡例バー: sim_viz と同じ位置 (x=-0.75, y 1.20〜4.20) に 24 段の色帯
        void BuildLegend()
        {
            var z = m_Data.viz;
            const int seg = 24;
            const float halfW = 0.08f;           // sim_viz の LINE_LIST 幅 0.16 m
            var v = new Vector3[seg * 4];
            var c = new Color[seg * 4];
            var idx = new int[seg * 6];
            for (int k = 0; k < seg; k++)
            {
                float ya = Mathf.Lerp(z.legend_y0, z.legend_y1, (float)k / seg);
                float yb = Mathf.Lerp(z.legend_y0, z.legend_y1, (float)(k + 1) / seg);
                Color col = SpeedColor(z.speed_color_max_mps * (k + 0.5f) / seg, z.speed_color_max_mps);
                int b = k * 4;
                v[b] = RosFrame.ToUnity(z.legend_x - halfW, ya, 0.03f);
                v[b + 1] = RosFrame.ToUnity(z.legend_x + halfW, ya, 0.03f);
                v[b + 2] = RosFrame.ToUnity(z.legend_x - halfW, yb, 0.03f);
                v[b + 3] = RosFrame.ToUnity(z.legend_x + halfW, yb, 0.03f);
                c[b] = c[b + 1] = c[b + 2] = c[b + 3] = col;
                int i = k * 6;
                idx[i] = b; idx[i + 1] = b + 1; idx[i + 2] = b + 2;
                idx[i + 3] = b + 1; idx[i + 4] = b + 3; idx[i + 5] = b + 2;
            }
            var mesh = new Mesh { name = "SpeedLegend", vertices = v, colors = c, triangles = idx };
            mesh.RecalculateBounds();
            var go = new GameObject("SpeedLegend");
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = m_VertexMat;
            go.layer = OverviewOnlyLayer;
        }

        // ------------------------------------------------------------------
        // cams: 左パネルに縦に並べるカメラ画像と見出し (1 台なら中央に 1 枚、2 台レースなら上下 2 枚)
        public void OnGUI(IList<(Texture tex, string title)> cams, float cropTopFrac)
        {
            if (!m_Cam.enabled) return;
            if (m_White == null) { m_White = new Texture2D(1, 1); m_White.SetPixel(0, 0, Color.white); m_White.Apply(); }
            if (m_Label == null)
            {
                m_Label = new GUIStyle(GUI.skin.label) { fontSize = 13, alignment = TextAnchor.MiddleRight };
                m_Label.normal.textColor = new Color(0.92f, 0.92f, 0.92f);
                m_Title = new GUIStyle(GUI.skin.label) { fontSize = 14, fontStyle = FontStyle.Bold };
                m_Title.normal.textColor = new Color(0.85f, 0.87f, 0.9f);
            }

            // 左パネル: 背景 + カメラ画像 (配信と同じく上部クロップ後の範囲を、縦横比を保って表示)
            float lw = Screen.width * LeftFrac;
            GUI.color = new Color32(28, 29, 31, 255);
            GUI.DrawTexture(new Rect(0, 0, lw, Screen.height), m_White);
            GUI.color = Color.white;
            int n = 0;
            foreach (var cam in cams) if (cam.tex != null) n++;
            if (n > 0)
            {
                const float titleH = 24f, gap = 18f;
                var first = cams[0].tex;
                float srcAspect = first.width / (first.height * (1f - cropTopFrac));
                // 幅いっぱいで入らなければ高さで合わせる
                float dw = lw - 24f;
                float dh = dw / srcAspect;
                float maxH = (Screen.height - 24f - n * titleH - (n - 1) * gap) / n;
                if (dh > maxH) { dh = maxH; dw = dh * srcAspect; }
                float total = n * (titleH + dh) + (n - 1) * gap;
                float y = Mathf.Max(8f, (Screen.height - total) * 0.5f);   // RViz の画像パネルと同じく縦中央
                foreach (var cam in cams)
                {
                    if (cam.tex == null) continue;
                    GUI.Label(new Rect(12, y, lw - 24, titleH), cam.title, m_Title);
                    // テクスチャは下が v=0。上部クロップぶんを除いた下側 (1-crop) を切り出す
                    GUI.DrawTextureWithTexCoords(new Rect(12 + (lw - 24 - dw) * 0.5f, y + titleH, dw, dh),
                                                 cam.tex, new Rect(0, 0, 1, 1f - cropTopFrac));
                    y += titleH + dh + gap;
                }
            }

            // 右パネル: 凡例の目盛 (sim_viz と同じ 0 / 1/3 / 2/3 / max と単位)
            var z = m_Data.viz;
            for (int k = 0; k < 4; k++)
            {
                float yv = Mathf.Lerp(z.legend_y0, z.legend_y1, k / 3f);
                Vector3 sp = m_Cam.WorldToScreenPoint(RosFrame.ToUnity(z.legend_x - 0.14f, yv, 0.03f));
                GUI.Label(new Rect(sp.x - 64, Screen.height - sp.y - 10, 60, 20),
                          (z.speed_color_max_mps * k / 3f).ToString("0.0"), m_Label);
            }
            Vector3 tp = m_Cam.WorldToScreenPoint(RosFrame.ToUnity(z.legend_x, z.legend_y1 + 0.35f, 0.03f));
            var center = new GUIStyle(m_Label) { alignment = TextAnchor.MiddleCenter };
            GUI.Label(new Rect(tp.x - 90, Screen.height - tp.y - 10, 180, 20), "trail color = speed", center);
            Vector3 up = m_Cam.WorldToScreenPoint(RosFrame.ToUnity(z.legend_x, z.legend_y0 - 0.35f, 0.03f));
            GUI.Label(new Rect(up.x - 40, Screen.height - up.y - 10, 80, 20), "m/s", center);
        }
    }
}
