// 車両の見た目 (タミヤ M-05 + FD 系クーペボディ + センサマスト)。
// -owncar 等で別のボディを選べる (CarStyle)。787B・ロードスター ND・RX-7 は TT-02 (WB 0.257) の
// 寸法に合わせた 1/10 相当の見た目で、ロゴ・文字は入れない。物理には一切使わない。
// 参考: 試走会場の走行動画 (濃紺メタリックのボディ・シルバーホイール・リアウイング・
// 屋根のセンサマスト、コーナーで外側へロール)。
//
// 原点は base_link = 後軸中心の真下 (vehicle_sim / sim_viz と同じ)。前軸は +z に wheelbase。
// 寸法は M-05 と sim.yaml / hw_params.yaml に合わせた見た目用の値で、物理には使わない。
using UnityEngine;

namespace Minicar
{
    /// ボディの種類。Default は従来の濃紺 FD 系 (M-05 寸法)
    public enum CarStyle { Default, B787, Roadster, Rx7 }

    public partial class CarModel
    {
        // 寸法はボディごと (Default は M-05、それ以外は TT-02)
        readonly float Wheelbase;
        readonly float HalfTrack;
        readonly float TireRadius;
        readonly float TireWidth;
        const float TireSink = 0.0015f;       // 接地面のつぶれ (わずかに床へ沈めて接地感を出す)
        // 見た目のロール量 (実測ではない)。1/10 ツーリングは重心が低くサスが硬く、横 1 g でも 1〜2° 程度。
        // 旧値 0.55°/(m/s²)・上限 6° はコーナーで 4° 以上傾き、大きすぎた (2026-09-27)
        const float RollDegPerMs2 = 0.15f;
        const float RollMaxDeg = 2f;
        const float PitchDegPerMs2 = 0.15f;   // 旧 0.45・上限 4°
        const float PitchMaxDeg = 1.5f;

        public Transform Root { get; }
        readonly Transform m_Body;
        readonly Transform[] m_SteerPivots = new Transform[2];
        readonly Transform[] m_Spinners = new Transform[4];
        float m_SpinDeg, m_Roll, m_Pitch, m_PrevV;
        bool m_HasPrev;

        public CarStyle Style { get; }

        /// 見た目の全長 [m] (縮尺 1 のとき)。実車スケールのコースでは vehicle_profile の全長に合わせて SetScale する
        public float ModelLength
        {
            get
            {
                switch (Style)
                {
                    case CarStyle.B787: return 4.784f * kB787Scale;
                    case CarStyle.Roadster: return 3.915f * kNdScale;
                    case CarStyle.Rx7: return 4.289f * kRx7Scale;      // 実車の全長 (丸めた端まで) × 縮尺
                    default: return 0.401f;
                }
            }
        }
        public float Scale { get; private set; } = 1f;

        /// 実車の 3 台: 模型の単位 → 実車の m の倍率 (これを SetScale すると実車の寸法になる)。従来のボディは 0
        public float RealScale
        {
            get
            {
                if (Style == CarStyle.Default) return 0f;
                RealDims(Style, out float k, out _, out _, out _);
                return 1f / k;
            }
        }

        /// 車体全体の縮尺 (サーキットでは約 10 倍)。タイヤの回転と姿勢の量もこれに合わせる
        public void SetScale(float s)
        {
            Scale = Mathf.Max(0.01f, s);
            Root.localScale = Vector3.one * Scale;
        }

        /// "b787" | "nd" (roadster) | "rx7" | それ以外 = Default
        public static CarStyle ParseStyle(string s)
        {
            switch ((s ?? "").Trim().ToLowerInvariant())
            {
                case "b787": case "787b": case "787": return CarStyle.B787;
                case "nd": case "roadster": case "mx5": return CarStyle.Roadster;
                case "rx7": case "rx-7": case "fd": return CarStyle.Rx7;
                default: return CarStyle.Default;
            }
        }

        public CarModel(string name, Color bodyColor, int layer, bool withMast, CarStyle style = CarStyle.Default)
        {
            Style = style;
            bool tt02 = style != CarStyle.Default;
            Wheelbase = tt02 ? 0.257f : 0.210f;     // TT-02 公称 / M-05 (sim.yaml wheelbase_m)
            HalfTrack = tt02 ? 0.084f : 0.082f;     // タイヤ外面がボディ側面と面一になる位置
            TireRadius = tt02 ? 0.032f : 0.030f;    // TT-02 約 64 mm / hw_params.yaml tire_diameter_m 0.060
            TireWidth = tt02 ? 0.028f : 0.026f;
            if (tt02)
            {
                // 実車の寸法から (CarModel.Shell.cs)。タイヤの外面はフェンダーの 1 cm 内側
                RealDims(style, out float rk, out float tireR, out float tireW, out float side);
                TireRadius = tireR * rk; TireWidth = tireW * rk; HalfTrack = (side - 0.01f - tireW * 0.5f) * rk;
            }
            Root = new GameObject(name).transform;
            var lit = Resources.Load<Material>("Mat_Lit");
            Material Mat(Color c, float smooth, float metal = 0f)
            {
                var m = new Material(lit) { color = c };
                m.SetFloat("_Glossiness", smooth);
                m.SetFloat("_Metallic", metal);
                return m;
            }
            // 車体の塗装: 787B・ND・RX-7 は下地 (色と金属感) の上にクリア層を重ねる (Paint で印を付け、AddClearCoat が 2 枚目の材質を足す)。
            // 従来のボディ (ミニカーの会場・センサ画像に写る相手) は今まで通り 1 層
            var paint = Mat(bodyColor, 0.82f, 0.45f);
            var glass = Mat(new Color(0.04f, 0.05f, 0.07f), 0.93f);
            var black = Mat(new Color(0.03f, 0.03f, 0.03f), 0.25f);
            var tire = Mat(new Color(0.05f, 0.05f, 0.05f), 0.08f);
            var rim = Mat(new Color(0.78f, 0.79f, 0.80f), 0.75f, 0.9f);
            var lamp = Mat(new Color(0.95f, 0.95f, 0.9f), 0.9f);
            var tail = Mat(new Color(0.75f, 0.05f, 0.05f), 0.8f);
            var alu = Mat(new Color(0.55f, 0.56f, 0.58f), 0.6f, 0.8f);

            // ---- ボディ (ロール・ピッチはこの下に付ける) ----
            m_Body = new GameObject("Body").transform;
            m_Body.SetParent(Root, false);
            // ロール軸は車体中央・低い位置 (前後輪の中間)
            m_Body.localPosition = new Vector3(0f, 0.03f, Wheelbase * 0.5f);
            var hull = new GameObject("Hull").transform;
            hull.SetParent(m_Body, false);
            hull.localPosition = new Vector3(0f, -0.03f, -Wheelbase * 0.5f);

            switch (style)
            {
                case CarStyle.B787: BuildB787(hull, withMast, Mat, black, glass, lamp, tail, alu, lit); break;
                case CarStyle.Roadster: BuildRoadster(hull, withMast, Mat, black, glass, lamp, tail, alu, lit); break;
                case CarStyle.Rx7: BuildRx7(hull, withMast, Mat, black, glass, lamp, tail, alu, lit); break;
                default: BuildDefault(hull, withMast, paint, black, glass, lamp, tail, alu); break;
            }

            AddClearCoat(lit);

            // ---- タイヤ (車体に固定しない: ロールしてもタイヤは接地したまま) ----
            int k = 0;
            foreach (float z in new[] { 0f, Wheelbase })
            {
                foreach (float sx in new[] { -1f, 1f })
                {
                    var pivot = new GameObject(z > 0 ? "FrontWheelPivot" : "RearWheelPivot").transform;
                    pivot.SetParent(Root, false);
                    pivot.localPosition = new Vector3(sx * HalfTrack, TireRadius - TireSink, z);
                    if (z > 0) m_SteerPivots[sx < 0 ? 0 : 1] = pivot;
                    var spin = new GameObject("Spin").transform;
                    spin.SetParent(pivot, false);
                    m_Spinners[k++] = spin;

                    if (tt02)
                    {
                        // 787B は白、RX-7 は金色の 6 本スポーク、ND は銀の 5 本
                        var wrim = style == CarStyle.B787 ? Mat(new Color(0.93f, 0.93f, 0.91f), 0.6f, 0.1f)
                                 : style == CarStyle.Rx7 ? Mat(new Color(0.78f, 0.60f, 0.24f), 0.72f, 0.9f) : rim;
                        BuildWheel(spin, sx, tire, wrim, black, alu, style == CarStyle.Rx7 ? 6 : 5);
                        continue;
                    }
                    var t = Prim(PrimitiveType.Cylinder, "Tire", spin, tire);
                    t.localRotation = Quaternion.Euler(0f, 0f, 90f);
                    t.localScale = new Vector3(TireRadius * 2f, TireWidth * 0.5f, TireRadius * 2f);
                    var r = Prim(PrimitiveType.Cylinder, "Rim", spin, rim);
                    r.localRotation = Quaternion.Euler(0f, 0f, 90f);
                    r.localPosition = new Vector3(sx * 0.002f, 0f, 0f);
                    r.localScale = new Vector3(TireRadius * 1.35f, TireWidth * 0.5f + 0.0008f, TireRadius * 1.35f);
                    // スポーク 5 本 (回転が見えるように)
                    for (int s = 0; s < 5; s++)
                    {
                        var sp = Cube("Spoke", spin, new Vector3(sx * (TireWidth * 0.5f + 0.0012f), 0f, 0f),
                                      new Vector3(0.0015f, TireRadius * 1.25f, 0.004f), alu);
                        sp.localRotation = Quaternion.Euler(s * 36f, 0f, 0f);
                    }
                }
            }

            SetLayer(Root, layer);
        }

        // ------------------------------------------------------------------
        // 塗装のクリア層。Built-in の Standard は 1 層しか持てないので、同じメッシュをもう 1 回、色が透けて映り込みと
        // ハイライトだけが残る材質 (Transparent・α 0・滑らかさ 0.95) で重ねて描く。HDRP では RenderCompat が
        // この 2 枚目を外し、下地の材質に HDRP/Lit のクリアコート (_CoatMask) を付ける
        readonly System.Collections.Generic.List<Material> m_Paints = new System.Collections.Generic.List<Material>();

        Material Paint(Material m)
        {
            m.name = "CarPaint";
            m_Paints.Add(m);
            return m;
        }

        void AddClearCoat(Material lit)
        {
            if (m_Paints.Count == 0) return;
            var coat = new Material(lit) { name = "ClearCoat", color = new Color(1f, 1f, 1f, 0f) };
            coat.SetFloat("_Glossiness", 0.95f);
            coat.SetFloat("_Metallic", 0f);
            ProcTex.MakeTransparent(coat);
            foreach (var r in Root.GetComponentsInChildren<MeshRenderer>(true))
                if (m_Paints.Contains(r.sharedMaterial)) r.sharedMaterials = new[] { r.sharedMaterial, coat };
        }

        // 従来のボディ (M-05 + FD 系クーペ・濃紺など)
        void BuildDefault(Transform hull, bool withMast, Material paint, Material black, Material glass,
                          Material lamp, Material tail, Material alu)
        {
            // 断面 (z: 後軸からの前後位置, 半幅, 下端, 上端)。ホイールアーチは下端を上げて表す
            var hullSections = new[]
            {
                new Vector4(-0.098f, 0.070f, 0.036f, 0.066f),   // リアバンパー
                new Vector4(-0.088f, 0.088f, 0.024f, 0.078f),
                new Vector4(-0.048f, 0.094f, 0.014f, 0.083f),
                new Vector4(-0.036f, 0.096f, 0.062f, 0.084f),   // 後輪アーチ (タイヤ上端 0.060 のすぐ上)
                new Vector4( 0.036f, 0.096f, 0.062f, 0.085f),
                new Vector4( 0.048f, 0.094f, 0.014f, 0.085f),
                new Vector4( 0.166f, 0.093f, 0.014f, 0.078f),
                new Vector4( 0.176f, 0.096f, 0.062f, 0.075f),   // 前輪アーチ
                new Vector4( 0.246f, 0.093f, 0.062f, 0.060f),
                new Vector4( 0.258f, 0.088f, 0.020f, 0.054f),
                new Vector4( 0.290f, 0.072f, 0.026f, 0.042f),   // ノーズ
                new Vector4( 0.303f, 0.050f, 0.030f, 0.036f),
            };
            Part("Paint", hull, Loft(hullSections, 4.0f, true), paint);

            // キャビン (ガラス) と屋根
            var cabin = new[]
            {
                new Vector4(-0.030f, 0.070f, 0.078f, 0.084f),
                new Vector4( 0.010f, 0.074f, 0.080f, 0.112f),
                new Vector4( 0.080f, 0.072f, 0.080f, 0.118f),
                new Vector4( 0.120f, 0.070f, 0.079f, 0.110f),
                new Vector4( 0.175f, 0.068f, 0.076f, 0.079f),
            };
            Part("Glass", hull, Loft(cabin, 3.0f, true), glass);
            Part("Roof", hull, Loft(new[]
            {
                new Vector4(0.010f, 0.058f, 0.100f, 0.1135f),
                new Vector4(0.080f, 0.060f, 0.104f, 0.1195f),
                new Vector4(0.118f, 0.056f, 0.100f, 0.1115f),
            }, 3.0f, true), paint);

            // リアウイング
            Cube("WingPlate", hull, new Vector3(0f, 0.104f, -0.082f), new Vector3(0.17f, 0.004f, 0.030f), paint, -6f);
            Cube("WingStayL", hull, new Vector3(-0.055f, 0.092f, -0.078f), new Vector3(0.004f, 0.022f, 0.014f), black);
            Cube("WingStayR", hull, new Vector3(0.055f, 0.092f, -0.078f), new Vector3(0.004f, 0.022f, 0.014f), black);
            // ライト
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("HeadLamp", hull, new Vector3(sx * 0.045f, 0.047f, 0.286f), new Vector3(0.030f, 0.008f, 0.012f), lamp, 12f);
                Cube("TailLamp", hull, new Vector3(sx * 0.060f, 0.064f, -0.095f), new Vector3(0.026f, 0.010f, 0.006f), tail);
            }
            Cube("Grille", hull, new Vector3(0f, 0.034f, 0.300f), new Vector3(0.060f, 0.008f, 0.006f), black);

            // センサマスト (屋根の上の支柱とカメラ)。★センサの取付高さは sim.yaml の
            // cam_mount_height_m (0.12m) が正で、ここは見た目だけ。
            if (withMast)
            {
                Cube("MastPlate", hull, new Vector3(0f, 0.121f, 0.070f), new Vector3(0.050f, 0.004f, 0.050f), black);
                var rod = Prim(PrimitiveType.Cylinder, "MastRod", hull, alu);
                rod.localPosition = new Vector3(0f, 0.185f, 0.070f);
                rod.localScale = new Vector3(0.008f, 0.062f, 0.008f);
                Cube("MastSensor", hull, new Vector3(0f, 0.252f, 0.074f), new Vector3(0.026f, 0.022f, 0.030f), black);
            }
        }

        delegate Material MatFn(Color c, float smooth, float metal = 0f);

        // センサマスト (見た目だけ。取付高さは vehicle_profile.camera が正)
        void Mast(Transform hull, Material black, Material alu, float z, float baseY)
        {
            Cube("MastPlate", hull, new Vector3(0f, baseY + 0.002f, z), new Vector3(0.040f, 0.004f, 0.040f), black);
            var rod = Prim(PrimitiveType.Cylinder, "MastRod", hull, alu);
            rod.localPosition = new Vector3(0f, baseY + 0.034f, z);
            rod.localScale = new Vector3(0.008f, 0.032f, 0.008f);
            Cube("MastSensor", hull, new Vector3(0f, baseY + 0.074f, z + 0.004f), new Vector3(0.026f, 0.020f, 0.028f), black);
        }

        // v [m/s], steer [rad, 左正], aLat [m/s², 左正]
        public void Apply(float v, float steer, float aLat, float dt)
        {
            // 前輪の切れ角 (ROS の左正 → Unity の y 軸回りは負)
            foreach (var p in m_SteerPivots)
                p.localRotation = Quaternion.Euler(0f, -steer * Mathf.Rad2Deg, 0f);

            m_SpinDeg = Mathf.Repeat(m_SpinDeg + v / (TireRadius * Scale) * Mathf.Rad2Deg * dt, 360f);
            foreach (var s in m_Spinners)
                s.localRotation = Quaternion.Euler(m_SpinDeg, 0f, 0f);

            float aLong = 0f;
            if (m_HasPrev && dt > 1e-4f) aLong = Mathf.Clamp((v - m_PrevV) / dt, -12f, 12f);
            m_PrevV = v;
            m_HasPrev = true;

            // 旋回外側へロール、加速でノーズアップ・減速でノーズダウン。サスの遅れを一次遅れで
            float a = 1f - Mathf.Exp(-dt / 0.12f);
            m_Roll = Mathf.Lerp(m_Roll, Mathf.Clamp(aLat * RollDegPerMs2, -RollMaxDeg, RollMaxDeg), a);
            m_Pitch = Mathf.Lerp(m_Pitch, Mathf.Clamp(-aLong * PitchDegPerMs2, -PitchMaxDeg, PitchMaxDeg), a);
            m_Body.localRotation = Quaternion.Euler(m_Pitch, 0f, m_Roll);
        }

        // ------------------------------------------------------------------
        static Transform Prim(PrimitiveType type, string name, Transform parent, Material mat)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            Object.Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(parent, false);
            go.GetComponent<Renderer>().sharedMaterial = mat;
            return go.transform;
        }

        static Transform Cube(string name, Transform parent, Vector3 pos, Vector3 size, Material mat, float pitchDeg = 0f)
        {
            var t = Prim(PrimitiveType.Cube, name, parent, mat);
            t.localPosition = pos;
            t.localRotation = Quaternion.Euler(pitchDeg, 0f, 0f);
            t.localScale = size;
            return t;
        }

        static void Part(string name, Transform parent, Mesh mesh, Material mat)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
        }

        static void SetLayer(Transform t, int layer)
        {
            t.gameObject.layer = layer;
            foreach (Transform c in t) SetLayer(c, layer);
        }

        // 断面 (z, 半幅, 下端, 上端) を超楕円で結んだ閉じた胴体。n が大きいほど角張る
        static Mesh Loft(Vector4[] sec, float n, bool caps)
        {
            const int Ring = 28;
            var verts = new System.Collections.Generic.List<Vector3>();
            var tris = new System.Collections.Generic.List<int>();
            float e = 2f / n;
            foreach (var s in sec)
            {
                float cy = (s.z + s.w) * 0.5f, hh = (s.w - s.z) * 0.5f;
                for (int i = 0; i < Ring; i++)
                {
                    float th = i * Mathf.PI * 2f / Ring;
                    float c = Mathf.Cos(th), sn = Mathf.Sin(th);
                    float x = s.y * Mathf.Sign(c) * Mathf.Pow(Mathf.Abs(c), e);
                    float y = cy + hh * Mathf.Sign(sn) * Mathf.Pow(Mathf.Abs(sn), e);
                    verts.Add(new Vector3(x, y, s.x));
                }
            }
            for (int j = 0; j < sec.Length - 1; j++)
                for (int i = 0; i < Ring; i++)
                {
                    int a = j * Ring + i, b = j * Ring + (i + 1) % Ring;
                    int c = a + Ring, d = b + Ring;
                    tris.AddRange(new[] { a, c, b, b, c, d });
                }
            if (caps)
            {
                // 前後の蓋 (扇形)
                foreach (int j in new[] { 0, sec.Length - 1 })
                {
                    var s = sec[j];
                    int center = verts.Count;
                    verts.Add(new Vector3(0f, (s.z + s.w) * 0.5f, s.x));
                    for (int i = 0; i < Ring; i++)
                    {
                        int a = j * Ring + i, b = j * Ring + (i + 1) % Ring;
                        if (j == 0) tris.AddRange(new[] { center, a, b });
                        else tris.AddRange(new[] { center, b, a });
                    }
                }
            }
            var mesh = new Mesh();
            mesh.SetVertices(verts);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
