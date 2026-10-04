// course.json からコースの 3D を実行時に組み立てる。
// 形状の定義元は course.py。ここでは「どう見えるか」(材質・色) だけを決める。
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Minicar
{
    public partial class CourseBuilder : MonoBehaviour
    {
        public CourseData Data { get; private set; }
        public GameObject Divider { get; private set; }
        public Material ArrowFrontMaterial { get; private set; }
        public Light DisturbLight { get; private set; }
        public RealismData Realism { get; private set; }
        Light m_Ceiling;
        Material m_CarpetMat;
        Color m_CarpetBase = Color.white;
        Transform m_Spectators;
        Transform m_Tapes;

        Material m_Lit, m_Unlit;
        Transform m_Root;

        // vehicle_sim の合成カメラと同じ色 (BGR → RGB に並べ替え済み)
        static Color32 kCarpet = new Color32(92, 95, 95, 255);
        static readonly Color32 kVenue = new Color32(122, 120, 116, 255);
        static Color32 kWallWhite = new Color32(226, 222, 212, 255);
        static Color32 kWallRed = new Color32(196, 38, 36, 255);
        static readonly Color32 kVelvet = new Color32(6, 6, 7, 255);
        static readonly Color32 kSus = new Color32(170, 172, 175, 255);
        static readonly Dictionary<string, Color32> kArea = new Dictionary<string, Color32>
        {
            { "MU_HIGH", new Color32(60, 120, 60, 255) },    // 人工芝
            { "MU_LOW", new Color32(228, 224, 222, 255) },   // 滑り板 (PTFE)
            { "ROUGH", new Color32(76, 192, 237, 255) },     // 水色の風呂マット
            { "SLOPE", new Color32(200, 198, 196, 255) },    // 坂道の白ボード
        };
        static readonly Dictionary<string, Color32> kSlot = new Dictionary<string, Color32>
        {
            { "green", new Color32(70, 160, 70, 255) },
            { "red", new Color32(200, 60, 60, 255) },
            { "blue", new Color32(60, 90, 200, 255) },
        };

        public void Build()
        {
            // -course <file>: StreamingAssets からの相対か絶対パス (既定 course.json。富士は course_fuji_<profile>.json)
            string file = SimBridge.Arg("-course", "course.json");
            string path = Path.IsPathRooted(file) ? file : Path.Combine(Application.streamingAssetsPath, file);
            if (!File.Exists(path))
            {
                Debug.LogError($"[CourseBuilder] {path} が無い。COURSE=<名前> ./scripts/export_course.sh <profile> で作って StreamingAssets へ置く");
                path = Path.Combine(Application.streamingAssetsPath, "course.json");
            }
            Data = JsonUtility.FromJson<CourseData>(File.ReadAllText(path));
            Debug.Log($"[CourseBuilder] {path} kind={(Data.IsCircuit ? "circuit" : "minicar")}");
            m_Lit = Resources.Load<Material>("Mat_Lit");
            m_Unlit = Resources.Load<Material>("Mat_Unlit");
            m_Root = new GameObject("Course").transform;
            Realism = Data.realism ?? new RealismData();
            if (Data.IsCircuit)
            {
                Realism.enable = false;         // 屋内会場の演出 (カーペット・観戦者・天井光) は使わない
                BuildCircuit();
                return;
            }
            if (Realism.enable)
            {
                kCarpet = new Color32((byte)Realism.carpet_r, (byte)Realism.carpet_g, (byte)Realism.carpet_b, 255);
                kWallWhite = new Color32((byte)Realism.wall_white_r, (byte)Realism.wall_white_g, (byte)Realism.wall_white_b, 255);
                kWallRed = new Color32((byte)Realism.wall_red_r, (byte)Realism.wall_red_g, (byte)Realism.wall_red_b, 255);
            }

            BuildFloor();
            if (Realism.enable)
            {
                BuildBackdrop();
                BuildSpectators();
            }
            BuildWalls();
            BuildAreas();
            BuildParking();
            BuildStartLines();
            BuildArrowSign();
            BuildLightRig();
            BuildVenueLighting();
            MarkOverhead();
        }

        // 真上からの全景 (RvizLayout) で床と走行軌跡を隠すものは別レイヤにする
        static readonly HashSet<string> kOverhead = new HashSet<string>
        {
            "CurtainRoof", "CurtainWDrop", "TunnelBarS", "TunnelBarN", "TunnelBarW", "TunnelBarE", "TunnelPost",
            "LightBarEW", "LightBarNS", "StageLight", "ArrowTopBar", "ArrowHanger",
        };

        void MarkOverhead()
        {
            foreach (Transform t in m_Root)
                if (kOverhead.Contains(t.name)) t.gameObject.layer = RvizLayout.OverheadLayer;
        }

        // ------------------------------------------------------------------
        Material Lit(Color32 c, Texture2D tex = null, float smooth = 0.1f, Vector2? tiling = null)
        {
            var m = new Material(m_Lit) { color = c };
            if (tex != null)
            {
                m.mainTexture = tex;
                m.mainTextureScale = tiling ?? Vector2.one;
            }
            m.SetFloat("_Glossiness", smooth);
            m.SetFloat("_Metallic", 0f);
            return m;
        }

        // 粒状のノイズ (カーペット・芝・木目の「一様でない」見え)。
        // 検出器が一様色に頼らないように、OpenCV 側の斑点描画と同じ役割。
        static Texture2D Speckle(int size, float amp, int seed, bool streak = false)
        {
            var rng = new System.Random(seed);
            var t = new Texture2D(size, size, TextureFormat.RGB24, true) { wrapMode = TextureWrapMode.Repeat };
            var px = new Color32[size * size];
            for (int y = 0; y < size; y++)
            {
                float row = streak ? (float)(rng.NextDouble() - 0.5) * amp * 0.6f : 0f;
                for (int x = 0; x < size; x++)
                {
                    float v = 1f + (float)(rng.NextDouble() - 0.5) * amp + row;
                    byte b = (byte)Mathf.Clamp(v * 200f, 0, 255);
                    px[y * size + x] = new Color32(b, b, b, 255);
                }
            }
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }

        GameObject Box(string name, Vector3 center, Vector3 size, Quaternion rot, Material mat)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(m_Root, false);
            go.transform.SetPositionAndRotation(center, rot);
            go.transform.localScale = size;
            go.GetComponent<Renderer>().sharedMaterial = mat;
            return go;
        }

        // 床に貼る矩形 (ROS 座標の軸平行矩形)。z は重なり順のための浮かせ量。
        GameObject FloorRect(string name, float x0, float y0, float x1, float y1, float z, Material mat)
        {
            Vector3 c = RosFrame.ToUnity((x0 + x1) * 0.5f, (y0 + y1) * 0.5f, z);
            var size = new Vector3(Mathf.Abs(y1 - y0), 0.002f, Mathf.Abs(x1 - x0));
            return Box(name, c, size, Quaternion.identity, mat);
        }

        // 4 隅 (ROS 座標) の板。normal を向く面だけを表にする。
        public static Mesh QuadMesh(Vector3 tl, Vector3 tr, Vector3 br, Vector3 bl, Vector3 normal)
        {
            var m = new Mesh();
            m.vertices = new[] { tl, tr, br, bl };
            m.uv = new[] { new Vector2(0, 1), new Vector2(1, 1), new Vector2(1, 0), new Vector2(0, 0) };
            int[] tri = { 0, 1, 2, 0, 2, 3 };
            if (Vector3.Dot(Vector3.Cross(tr - tl, br - tl), normal) < 0f)
                tri = new[] { 0, 2, 1, 0, 3, 2 };
            m.triangles = tri;
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }

        GameObject MeshObject(string name, Mesh mesh, Material mat)
        {
            var go = new GameObject(name);
            go.transform.SetParent(m_Root, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            return go;
        }

        // ------------------------------------------------------------------
        void BuildFloor()
        {
            // 会場の床 (コースの外) とパンチカーペット (コース内)
            FloorRect("VenueFloor", -6f, -6f, 16f, 12f, -0.004f,
                      Lit(kVenue, Speckle(128, 0.10f, 1), 0.2f, new Vector2(40, 40)))
                .layer = RvizLayout.SensorOnlyLayer;
            // 実画像から作ったカーペットのタイル (StreamingAssets/textures/carpet.png) があればそれを貼る
            var real = Realism.enable ? LoadTexture(Realism.carpet_tex) : null;
            m_CarpetMat = real != null
                ? Lit(new Color32(255, 255, 255, 255), real, 0.0f, new Vector2(30, 19))    // 1 タイル ≈ 0.35 m
                : Lit(kCarpet, Speckle(256, 0.28f, 2), 0.0f, new Vector2(60, 40));
            m_CarpetBase = m_CarpetMat.color;
            FloorRect("Carpet", -0.05f, -0.1f, 10.4f, 6.45f, 0f, m_CarpetMat);
        }

        Texture2D LoadTexture(string rel)
        {
            string path = Path.Combine(Application.streamingAssetsPath, rel);
            if (!File.Exists(path)) { Debug.LogWarning($"[CourseBuilder] texture not found: {path}"); return null; }
            var t = new Texture2D(2, 2, TextureFormat.RGB24, true) { wrapMode = TextureWrapMode.Repeat, filterMode = FilterMode.Trilinear };
            t.LoadImage(File.ReadAllBytes(path));
            return t;
        }

        // 壁の向こうの会場: 実画像の上部を横に並べた帯を、コースを囲む円筒の内側に貼る。
        // 幾何的には正しくないが、CNN が見る「壁の上の雑然とした明るさ」の分布を実画像から持ってくる。
        void BuildBackdrop()
        {
            if (string.IsNullOrEmpty(Realism.backdrop_tex)) return;
            var tex = LoadTexture(Realism.backdrop_tex);
            if (tex == null) return;
            tex.wrapMode = TextureWrapMode.Repeat;
            const int N = 96;
            float cx = 5.2f, cy = 3.2f, R = Realism.backdrop_radius_m, H = Realism.backdrop_height_m, z0 = Realism.backdrop_z0_m;
            var v = new Vector3[(N + 1) * 2];
            var uv = new Vector2[(N + 1) * 2];
            var tri = new int[N * 12];               // 両面 (内側から見るので裏面も張る)
            for (int i = 0; i <= N; i++)
            {
                float a = 2f * Mathf.PI * i / N;
                float x = cx + R * Mathf.Cos(a), y = cy + R * Mathf.Sin(a);
                v[i * 2] = RosFrame.ToUnity(x, y, z0);
                v[i * 2 + 1] = RosFrame.ToUnity(x, y, z0 + H);
                uv[i * 2] = new Vector2((float)i / N, 0f);
                uv[i * 2 + 1] = new Vector2((float)i / N, 1f);
            }
            for (int i = 0; i < N; i++)
            {
                int b = i * 2, t = i * 12;
                tri[t] = b; tri[t + 1] = b + 1; tri[t + 2] = b + 2;
                tri[t + 3] = b + 1; tri[t + 4] = b + 3; tri[t + 5] = b + 2;
                tri[t + 6] = b; tri[t + 7] = b + 2; tri[t + 8] = b + 1;
                tri[t + 9] = b + 1; tri[t + 10] = b + 2; tri[t + 11] = b + 3;
            }
            var m = new Mesh { vertices = v, uv = uv, triangles = tri };
            m.RecalculateNormals();
            m.RecalculateBounds();
            var mat = new Material(m_Unlit) { mainTexture = tex };
            var go = MeshObject("Backdrop", m, mat);
            go.layer = RvizLayout.SensorOnlyLayer;
        }

        // 観戦者: 壁の外側に立つ人。カプセル (胴) + 球 (頭)、服の色はランダム。
        // 実会場では壁の上に人が並んで見えるので、その「雑音」を入れる (seed で固定)。
        void BuildSpectators() => BuildSpectators(Realism.spectator_seed);

        void BuildSpectators(int seed)
        {
            int n = Realism.spectators;
            if (n <= 0) return;
            var rng = new System.Random(seed);
            float x0 = -0.05f, y0 = -0.1f, x1 = 10.4f, y1 = 6.45f;      // カーペットの外周
            if (m_Spectators != null) Destroy(m_Spectators.gameObject);
            var parent = new GameObject("Spectators").transform;
            parent.SetParent(m_Root, false);
            m_Spectators = parent;
            for (int i = 0; i < n; i++)
            {
                // 外周のどこかに、壁から 0.3〜2.5 m 離して立つ
                float d = 0.3f + 2.2f * (float)rng.NextDouble();
                float t = (float)rng.NextDouble();
                float x, y;
                switch (i % 4)
                {
                    case 0: x = x0 + t * (x1 - x0); y = y0 - d; break;
                    case 1: x = x0 + t * (x1 - x0); y = y1 + d; break;
                    case 2: x = x0 - d; y = y0 + t * (y1 - y0); break;
                    default: x = x1 + d; y = y0 + t * (y1 - y0); break;
                }
                float h = 1.5f + 0.3f * (float)rng.NextDouble();
                var shirt = new Color32((byte)rng.Next(20, 230), (byte)rng.Next(20, 230), (byte)rng.Next(20, 230), 255);
                var pants = new Color32((byte)rng.Next(20, 90), (byte)rng.Next(20, 90), (byte)rng.Next(30, 110), 255);
                var skin = new Color32((byte)rng.Next(150, 230), (byte)rng.Next(110, 180), (byte)rng.Next(90, 150), 255);
                float legs = h * 0.45f, torso = h * 0.4f, head = h * 0.15f;
                Spect(parent, PrimitiveType.Capsule, RosFrame.ToUnity(x, y, legs * 0.5f), new Vector3(0.28f, legs * 0.5f, 0.2f), Lit(pants));
                Spect(parent, PrimitiveType.Capsule, RosFrame.ToUnity(x, y, legs + torso * 0.5f), new Vector3(0.42f, torso * 0.5f, 0.24f), Lit(shirt));
                Spect(parent, PrimitiveType.Sphere, RosFrame.ToUnity(x, y, legs + torso + head * 0.55f), Vector3.one * head, Lit(skin));
            }
        }

        void Spect(Transform parent, PrimitiveType type, Vector3 pos, Vector3 scale, Material mat)
        {
            var go = GameObject.CreatePrimitive(type);
            Destroy(go.GetComponent<Collider>());
            go.name = "Spectator";
            go.transform.SetParent(parent, false);
            go.transform.position = pos;
            go.transform.localScale = scale;
            go.GetComponent<Renderer>().sharedMaterial = mat;
            go.layer = RvizLayout.SensorOnlyLayer;
        }

        void BuildWalls()
        {
            var d = Data;
            var white = Lit(kWallWhite, Speckle(128, 0.08f, 3, true), 0.15f, new Vector2(1, 8));
            var red = Lit(kWallRed, Speckle(128, 0.08f, 4, true), 0.25f, new Vector2(1, 8));
            var velvet = Lit(kVelvet, null, 0.0f);
            AreaData tunnel = System.Array.Find(d.areas, a => a.name == "TUNNEL");
            // 暗幕の中の壁は天井光も環境光も届かないので、材質ごと暗くする
            // (ビルトイン RP の環境光は遮蔽されないため、影だけでは暗くならない)
            var whiteDark = Lit(Dim(kWallWhite, kTunnelDim), null, 0.1f);
            var redDark = Lit(Dim(kWallRed, kTunnelDim), null, 0.1f);

            for (int i = 0; i < d.walls.Length; i++)
            {
                var w = d.walls[i];
                Vector3 a = RosFrame.ToUnity(w.x0, w.y0, 0f), b = RosFrame.ToUnity(w.x1, w.y1, 0f);
                float len = Vector3.Distance(a, b);
                if (len < 1e-3f) continue;
                Quaternion rot = Quaternion.LookRotation((b - a) / len, Vector3.up);
                Vector3 mid = (a + b) * 0.5f;

                float mx = (w.x0 + w.x1) * 0.5f, my = (w.y0 + w.y1) * 0.5f;
                bool inTunnel = tunnel != null && Inside(tunnel, mx, my, 0f);
                // 板は床から wall_base_m (30 mm) 浮いている (規約 p.34)。上端 = base + height。
                float wallTop = d.wall_base_m + d.wall_height_m;
                var go = Box($"Wall{i:00}_{w.color}", mid + Vector3.up * (d.wall_base_m + d.wall_height_m * 0.5f),
                             new Vector3(d.wall_thickness_m, d.wall_height_m, len), rot,
                             inTunnel ? (w.color == "red" ? redDark : whiteDark) : (w.color == "red" ? red : white));
                if (w.divider) Divider = go;

                // 内周の暗幕 (区域の内側に完全に入っている壁 = 八角の内コーナー)
                bool innerWall = tunnel != null && Inside(tunnel, w.x0, w.y0, -0.05f) && Inside(tunnel, w.x1, w.y1, -0.05f);
                if (innerWall)
                    Box($"TunnelInnerSheet{i:00}", mid + Vector3.up * (wallTop + (d.tunnel_height_m - wallTop) * 0.5f),
                        new Vector3(0.004f, d.tunnel_height_m - wallTop, len), rot, velvet);
            }

            if (tunnel != null) BuildTunnel(tunnel, velvet);
        }

        const float kTunnelDim = 0.22f;          // 暗幕の中の見かけの明るさ (外に対する比)
        const float kTunnelEntryOpen = 0.38f;    // 入口の開口高さ [m] (マスト込みで車が通れる高さ)

        static Color32 Dim(Color32 c, float k) =>
            new Color32((byte)(c.r * k), (byte)(c.g * k), (byte)(c.b * k), 255);

        // ①トンネル (暗幕ゾーン)。
        // △3 p.25-26: SUS 骨組み 195x195cm・高さ 133cm に漆黒シート (3 枚外周・1 枚内周)、
        // 西側がレーンの出入口。★資料は「上部 OPEN」だが、ここでは暗幕の中を暗くするため
        // 天井も暗幕で覆い、入口の上にも垂れ幕を下げている (見た目優先の近似)。
        void BuildTunnel(AreaData t, Material velvet)
        {
            float h = Data.tunnel_height_m, th = 0.006f;
            float cx = (t.x0 + t.x1) * 0.5f, cy = (t.y0 + t.y1) * 0.5f;
            float lx = t.x1 - t.x0, ly = t.y1 - t.y0;

            // 外周の暗幕: 南・北・東 (床から天井まで)
            Box("CurtainS", RosFrame.ToUnity(cx, t.y0, h * 0.5f), new Vector3(th, h, lx), Quaternion.identity, velvet);
            Box("CurtainN", RosFrame.ToUnity(cx, t.y1, h * 0.5f), new Vector3(th, h, lx), Quaternion.identity, velvet);
            Box("CurtainE", RosFrame.ToUnity(t.x1, cy, h * 0.5f), new Vector3(ly, h, th), Quaternion.identity, velvet);
            // 西 (出入口): 開口の上に垂れ幕
            float drop = h - kTunnelEntryOpen;
            Box("CurtainWDrop", RosFrame.ToUnity(t.x0, cy, kTunnelEntryOpen + drop * 0.5f), new Vector3(ly, drop, th), Quaternion.identity, velvet);
            // 天井の暗幕
            Box("CurtainRoof", RosFrame.ToUnity(cx, cy, h), new Vector3(ly, th, lx), Quaternion.identity, velvet);

            // 中の床 (暗いカーペット)。カーペットの上に重ねる
            FloorRect("TunnelFloor", t.x0, t.y0, t.x1, t.y1, 0.0035f,
                      Lit(Dim(kCarpet, kTunnelDim), Speckle(256, 0.28f, 9), 0.0f, new Vector2(12, 18)));

            // SUS 骨組み: 四隅の柱と上端の枠
            var sus = Lit(kSus, null, 0.7f);
            float r = 0.02f;
            foreach (var (px, py) in new[] { (t.x0, t.y0), (t.x0, t.y1), (t.x1, t.y0), (t.x1, t.y1) })
                Box("TunnelPost", RosFrame.ToUnity(px, py, h * 0.5f), new Vector3(r, h, r), Quaternion.identity, sus);
            Box("TunnelBarS", RosFrame.ToUnity(cx, t.y0, h), new Vector3(r, r, lx), Quaternion.identity, sus);
            Box("TunnelBarN", RosFrame.ToUnity(cx, t.y1, h), new Vector3(r, r, lx), Quaternion.identity, sus);
            Box("TunnelBarW", RosFrame.ToUnity(t.x0, cy, h), new Vector3(ly, r, r), Quaternion.identity, sus);
            Box("TunnelBarE", RosFrame.ToUnity(t.x1, cy, h), new Vector3(ly, r, r), Quaternion.identity, sus);
        }

        static bool Inside(AreaData a, float x, float y, float margin) =>
            x >= a.x0 - margin && x <= a.x1 + margin && y >= a.y0 - margin && y <= a.y1 + margin;

        void BuildAreas()
        {
            // 重なり順: 芝の上に滑り板 (vehicle_sim の surface_at と同じ優先)
            var order = new[] { "MU_HIGH", "SLOPE", "ROUGH", "MU_LOW" };
            float z = 0.001f;
            foreach (var name in order)
            {
                foreach (var a in Data.areas)
                {
                    if (a.name != name) continue;
                    Material mat = name switch
                    {
                        "MU_HIGH" => Lit(kArea[name], Speckle(256, 0.55f, 5), 0.0f, new Vector2(20, 30)),
                        "ROUGH" => Lit(kArea[name], Speckle(64, 0.25f, 6), 0.35f, new Vector2(6, 12)),
                        "MU_LOW" => Lit(kArea[name], Speckle(64, 0.04f, 7), 0.75f),
                        _ => Lit(kArea[name], Speckle(64, 0.06f, 8), 0.3f),
                    };
                    FloorRect(name, a.x0, a.y0, a.x1, a.y1, z, mat);
                    z += 0.001f;
                }
            }
        }

        void BuildParking()
        {
            // ⑧駐車枠は色テープの「枠線」(幅 5cm)。面で塗らない。
            float t = Data.parking_tape_m, z = 0.006f;
            foreach (var s in Data.parking_slots)
            {
                var mat = Lit(kSlot.TryGetValue(s.color, out var c) ? c : new Color32(200, 200, 200, 255), null, 0.4f);
                FloorRect($"{s.name}_W", s.x0, s.y0, s.x0 + t, s.y1, z, mat);
                FloorRect($"{s.name}_E", s.x1 - t, s.y0, s.x1, s.y1, z, mat);
                FloorRect($"{s.name}_S", s.x0, s.y0, s.x1, s.y0 + t, z, mat);
                FloorRect($"{s.name}_N", s.x0, s.y1 - t, s.x1, s.y1, z, mat);
                FloorLabel(s, kSlot.TryGetValue(s.color, out var lc) ? lc : new Color32(230, 230, 230, 255));
            }
        }

        // スタートライン 1/2/3 (規約 p.21・p.24)。下段レーンを横切る常設の白テープ。乱択化の床テープとは別
        void BuildStartLines()
        {
            if (Data.start_lines == null) return;
            var mat = Lit(new Color32(236, 236, 230, 255), null, 0.35f);
            foreach (var l in Data.start_lines)
            {
                float hw = 0.5f * (l.width > 0f ? l.width : 0.05f);
                FloorRect(l.name, l.x - hw, l.y0, l.x + hw, l.y1, 0.006f, mat);
            }
        }

        // 駐車枠の番号 (P1 / P2 / P3) を枠内の床に描く。テープと同じ色のドット文字。
        // 走路 (+y 側) から読める向き: 文字の右 = -x、文字の上 = -y。
        void FloorLabel(SlotData s, Color32 col)
        {
            const float w = 0.36f;                       // ラベル幅 [m]
            var tex = LabelTexture.Make(s.name, col, kCarpet);
            float h = w * tex.height / tex.width;
            float cx = (s.x0 + s.x1) * 0.5f, cy = (s.y0 + s.y1) * 0.5f, z = 0.0075f;
            float hw = w * 0.5f, hh = h * 0.5f;
            Vector3 tl = RosFrame.ToUnity(cx + hw, cy - hh, z), tr = RosFrame.ToUnity(cx - hw, cy - hh, z);
            Vector3 br = RosFrame.ToUnity(cx - hw, cy + hh, z), bl = RosFrame.ToUnity(cx + hw, cy + hh, z);
            var mat = new Material(m_Unlit) { mainTexture = tex };
            MeshObject($"{s.name}_label", QuadMesh(tl, tr, br, bl, RosFrame.ToUnity(0f, 0f, 1f)), mat);
        }

        void BuildArrowSign()
        {
            var s = Data.arrow_sign;
            var sus = Lit(kSus, null, 0.7f);
            // 門型フレーム: 左右の支柱 (壁の上) と上の横棒
            foreach (float py in new[] { s.post_y0, s.post_y1 })
            {
                var post = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                Destroy(post.GetComponent<Collider>());
                post.transform.SetParent(m_Root, false);
                post.transform.position = RosFrame.ToUnity(s.x, py, s.post_h * 0.5f);
                post.transform.localScale = new Vector3(s.post_r * 2f, s.post_h * 0.5f, s.post_r * 2f);
                post.GetComponent<Renderer>().sharedMaterial = sus;
            }
            float span = s.post_y1 - s.post_y0, midY = (s.post_y0 + s.post_y1) * 0.5f;
            Box("ArrowTopBar", RosFrame.ToUnity(s.x, midY, s.post_h), new Vector3(span, 0.02f, 0.02f), Quaternion.identity, sus);
            // 吊りワイヤ
            float boardTop = s.board_z0 + s.board_h;
            Box("ArrowHanger", RosFrame.ToUnity(s.x, midY, (boardTop + s.post_h) * 0.5f),
                new Vector3(0.004f, s.post_h - boardTop, 0.004f), Quaternion.identity, sus);

            // 掲示板: 表は +x 側 (狭い道の入口) を向く。車は -x へ走るので
            // 車から見た左 = -y 側がテクスチャの左 (vehicle_sim と同じ)。
            float hw = s.board_w * 0.5f, zt = boardTop, zb = s.board_z0, eps = 0.006f;
            Vector3 tl = RosFrame.ToUnity(s.x + eps, midY - hw, zt), tr = RosFrame.ToUnity(s.x + eps, midY + hw, zt);
            Vector3 br = RosFrame.ToUnity(s.x + eps, midY + hw, zb), bl = RosFrame.ToUnity(s.x + eps, midY - hw, zb);
            ArrowFrontMaterial = new Material(m_Unlit);
            MeshObject("ArrowBoardFront", QuadMesh(tl, tr, br, bl, RosFrame.ToUnity(1f, 0f, 0f)), ArrowFrontMaterial);
            // 筐体 (背面は黒いプラダン)
            Box("ArrowBoardBody", RosFrame.ToUnity(s.x - 0.01f, midY, (zt + zb) * 0.5f),
                new Vector3(s.board_w + 0.02f, s.board_h + 0.02f, 0.02f), Quaternion.identity, Lit(new Color32(24, 24, 26, 255)));
        }

        void BuildLightRig()
        {
            // ③ライトかく乱: 十字の SUS 材の交点にステージライトを吊る
            var l = Data.light;
            var sus = Lit(kSus, null, 0.7f);
            Box("LightBarEW", RosFrame.ToUnity(l.x, l.y, l.bar_z), new Vector3(0.02f, 0.02f, l.bar_ew), Quaternion.identity, sus);
            Box("LightBarNS", RosFrame.ToUnity(l.x, l.y, l.bar_z), new Vector3(l.bar_ns, 0.02f, 0.02f), Quaternion.identity, sus);
            var ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            Destroy(ball.GetComponent<Collider>());
            ball.name = "StageLight";
            ball.transform.SetParent(m_Root, false);
            ball.transform.position = RosFrame.ToUnity(l.x, l.y, l.z - 0.06f);
            ball.transform.localScale = Vector3.one * 0.12f;
            ball.GetComponent<Renderer>().sharedMaterial = Lit(new Color32(220, 220, 230, 255), null, 0.95f);

            var go = new GameObject("DisturbLight");
            go.transform.SetParent(m_Root, false);
            go.transform.position = RosFrame.ToUnity(l.x, l.y, l.z - 0.14f);
            DisturbLight = go.AddComponent<Light>();
            DisturbLight.type = LightType.Point;
            DisturbLight.range = 2.2f;
            DisturbLight.intensity = 2.5f;
            DisturbLight.shadows = LightShadows.None;
        }

        // /sim/episode の seed で照明・床の色味・観戦者を引き直す (乱択化。幅は realism.episode_*)。
        // 物理 (vehicle_sim) と IMU (imu_sim) も同じ seed で引き直すので、1 本の bag = 1 エピソード。
        public void ApplyEpisode(uint seed)
        {
            if (Realism == null || !Realism.enable) return;
            var rng = new System.Random(unchecked((int)seed));
            float U(float r) => 1f + r * (2f * (float)rng.NextDouble() - 1f);
            if (m_Ceiling != null)
            {
                m_Ceiling.intensity = 0.55f * U(Realism.episode_light_range);
                float t = Realism.episode_tint_range;
                m_Ceiling.color = new Color(U(t), 0.97f * U(t), 0.92f * U(t));
            }
            float a = U(Realism.episode_ambient_range) * Realism.ambient_gain;
            RenderSettings.ambientSkyColor = new Color(0.40f, 0.40f, 0.42f) * a;
            RenderSettings.ambientEquatorColor = new Color(0.30f, 0.30f, 0.31f) * a;
            RenderSettings.ambientGroundColor = new Color(0.15f, 0.15f, 0.15f) * a;
            if (m_CarpetMat != null)
            {
                float t = Realism.episode_tint_range;
                m_CarpetMat.color = new Color(m_CarpetBase.r * U(t), m_CarpetBase.g * U(t), m_CarpetBase.b * U(t), 1f);
            }
            if (Realism.episode_spectators && Realism.spectators > 0)
                BuildSpectators(Realism.spectator_seed + (int)(seed % 100000));
            BuildTapes(new System.Random(unchecked((int)(seed * 2654435761u))));
            Debug.Log($"[CourseBuilder] episode seed={seed}: light {m_Ceiling?.intensity:F2} ambient x{a:F2}");
        }

        // 床の白テープ (幅 5 cm)。走路の中心線上のランダムな位置に、走路にほぼ直交して 0〜N 本。
        // (旧) 規約のスタートラインを確率 start_line_prob で描く。常設は BuildStartLines (course.json の start_lines) なので既定 0。
        void BuildTapes(System.Random rng)
        {
            if (m_Tapes != null) Destroy(m_Tapes.gameObject);
            m_Tapes = new GameObject("Tapes").transform;
            m_Tapes.SetParent(m_Root, false);
            var r = Realism;
            float[] c = (Data.reference_line != null && Data.reference_line.Length >= 6) ? Data.reference_line : Data.centerline_shortcut;
            float R() => (float)rng.NextDouble();
            Material Tape()
            {
                byte g = (byte)(205 + rng.Next(0, 45));
                return Lit(new Color32(g, g, (byte)Mathf.Max(0, g - rng.Next(0, 12)), 255), null, 0.35f);
            }
            void Strip(Vector2 mid, Vector2 along, float len, float w, Material mat)
            {
                // along = テープの長手方向 (単位)、幅方向はそれに直交
                Vector2 n = new Vector2(-along.y, along.x) * (w * 0.5f), a = along * (len * 0.5f);
                const float z = 0.0065f;
                Vector3 P(Vector2 p) => RosFrame.ToUnity(p.x, p.y, z);
                var go = MeshObject("Tape", QuadMesh(P(mid - a + n), P(mid + a + n), P(mid + a - n), P(mid - a - n), RosFrame.ToUnity(0f, 0f, 1f)), mat);
                go.transform.SetParent(m_Tapes, true);
            }
            int m = c != null ? c.Length / 2 : 0;
            int nTapes = m >= 3 ? rng.Next(0, Mathf.Max(0, r.episode_tapes_max) + 1) : 0;
            for (int k = 0; k < nTapes; k++)
            {
                int i = rng.Next(0, m);
                int j = (i + 1) % m;
                var p0 = new Vector2(c[i * 2], c[i * 2 + 1]);
                var t = (new Vector2(c[j * 2], c[j * 2 + 1]) - p0).normalized;
                float ang = (R() * 2f - 1f) * r.tape_angle_deg * Mathf.Deg2Rad;
                var across = new Vector2(-t.y, t.x);
                across = new Vector2(across.x * Mathf.Cos(ang) - across.y * Mathf.Sin(ang), across.x * Mathf.Sin(ang) + across.y * Mathf.Cos(ang));
                float len = Mathf.Lerp(r.tape_len_min_m, r.tape_len_max_m, R());
                float w = r.tape_width_m * (0.8f + 0.4f * R());
                Strip(p0, across, len, w, Tape());
            }
            if (r.start_line_x != null)
                foreach (float x in r.start_line_x)
                    if (R() < r.start_line_prob)
                        Strip(new Vector2(x, 0.5f * (r.start_line_y0 + r.start_line_y1)), new Vector2(0f, 1f),
                              r.start_line_y1 - r.start_line_y0, r.tape_width_m, Tape());
        }

        void BuildVenueLighting()
        {
            // 屋内会場: 天井照明を模した拡散光 + 弱い影
            var go = new GameObject("CeilingLight");
            var sun = go.AddComponent<Light>();
            m_Ceiling = sun;
            sun.type = LightType.Directional;
            sun.intensity = 0.55f;
            sun.color = new Color(1f, 0.97f, 0.92f);
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 0.6f;
            sun.shadowBias = 0.005f;          // 既定値だと影が車から離れて「浮いて」見える
            sun.shadowNormalBias = 0.05f;
            sun.shadowResolution = UnityEngine.Rendering.LightShadowResolution.VeryHigh;
            go.transform.rotation = Quaternion.Euler(70f, 30f, 0f);
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
            // 明るさは OpenCV 描画 (カーペット平均 ≈ 95) に合わせて控えめにする。
            // /camera/brightness を使う zone_estimator のしきい値が両描画で通用するように。
            float ag = (Realism != null && Realism.enable) ? Realism.ambient_gain : 1f;
            RenderSettings.ambientSkyColor = new Color(0.40f, 0.40f, 0.42f) * ag;
            RenderSettings.ambientEquatorColor = new Color(0.30f, 0.30f, 0.31f) * ag;
            RenderSettings.ambientGroundColor = new Color(0.15f, 0.15f, 0.15f) * ag;
            QualitySettings.shadowDistance = 8f;
            QualitySettings.shadowCascades = 4;
            QualitySettings.shadowResolution = ShadowResolution.VeryHigh;
        }
    }
}
