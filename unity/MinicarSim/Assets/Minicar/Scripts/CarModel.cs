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

    public class CarModel
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
                    case CarStyle.B787: return 0.486f;
                    case CarStyle.Roadster: return 0.428f;
                    case CarStyle.Rx7: return 0.448f;
                    default: return 0.401f;
                }
            }
        }
        public float Scale { get; private set; } = 1f;

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
                case CarStyle.B787: BuildB787(hull, withMast, Mat, black, glass, lamp, tail, alu); break;
                case CarStyle.Roadster: BuildRoadster(hull, withMast, Mat, black, glass, lamp, tail, alu); break;
                case CarStyle.Rx7: BuildRx7(hull, withMast, Mat, black, glass, lamp, tail, alu); break;
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

        /// ソウルレッド (ロードスター): 鮮やかな赤の金属的な下地 (ハイライトが赤く光って広がる) ＋ クリア層。
        /// 下地の拡散は少なく (金属 0.8)、陰は深い赤。下地の滑らかさを 0.62 に下げてハイライトの赤い「にじみ」を広げる
        Material SoulRed(MatFn Mat) => Paint(Mat(new Color(0.66f, 0.015f, 0.04f), 0.62f, 0.80f));

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

        // ------------------------------------------------------------------
        // 787B: 低く長いプロトタイプ。オレンジ地にグリーンの帯、閉じたキャノピー、大きなリアウイング
        void BuildB787(Transform hull, bool withMast, MatFn Mat, Material black, Material glass,
                       Material lamp, Material tail, Material alu)
        {
            var orange = Paint(Mat(new Color(0.96f, 0.42f, 0.06f), 0.80f, 0.15f));
            var green = Paint(Mat(new Color(0.05f, 0.55f, 0.30f), 0.80f, 0.15f));
            var body = new[]
            {
                new Vector4(-0.112f, 0.088f, 0.024f, 0.058f),   // テール
                new Vector4(-0.098f, 0.097f, 0.016f, 0.072f),
                new Vector4(-0.044f, 0.100f, 0.014f, 0.082f),   // 後フェンダーの峰
                new Vector4(-0.034f, 0.100f, 0.068f, 0.083f),   // 後輪アーチ
                new Vector4( 0.034f, 0.100f, 0.068f, 0.080f),
                new Vector4( 0.046f, 0.098f, 0.014f, 0.071f),
                new Vector4( 0.205f, 0.092f, 0.014f, 0.061f),   // くびれ
                new Vector4( 0.221f, 0.099f, 0.068f, 0.066f),   // 前輪アーチ
                new Vector4( 0.293f, 0.099f, 0.068f, 0.058f),
                new Vector4( 0.306f, 0.094f, 0.017f, 0.052f),
                new Vector4( 0.346f, 0.080f, 0.019f, 0.036f),   // ノーズ
                new Vector4( 0.374f, 0.050f, 0.021f, 0.026f),
            };
            Part("Paint", hull, Loft(body, 4.0f, true), orange);
            // グリーンの帯: 胴の下半分を一周する薄い帯 (ボディより 1.5 mm 外側)
            var belt = new[]
            {
                new Vector4(-0.106f, 0.0905f, 0.020f, 0.040f),
                new Vector4(-0.044f, 0.1015f, 0.016f, 0.040f),
                new Vector4( 0.046f, 0.0995f, 0.016f, 0.040f),
                new Vector4( 0.205f, 0.0935f, 0.016f, 0.040f),
                new Vector4( 0.306f, 0.0955f, 0.018f, 0.038f),
                new Vector4( 0.360f, 0.0700f, 0.020f, 0.030f),
            };
            Part("GreenBelt", hull, Loft(belt, 4.0f, true), green);
            // ノーズ中央の緑
            Cube("NoseStripe", hull, new Vector3(0f, 0.050f, 0.318f), new Vector3(0.030f, 0.003f, 0.080f), green, 10f);
            // キャノピー (閉じた操縦席) と、その後ろのエンジンカウルの背
            Part("Canopy", hull, Loft(new[]
            {
                new Vector4(0.040f, 0.030f, 0.076f, 0.080f),
                new Vector4(0.080f, 0.048f, 0.076f, 0.106f),
                new Vector4(0.140f, 0.046f, 0.072f, 0.100f),
                new Vector4(0.190f, 0.034f, 0.064f, 0.068f),
            }, 2.6f, true), glass);
            Part("EngineCover", hull, Loft(new[]
            {
                new Vector4(-0.100f, 0.026f, 0.060f, 0.074f),
                new Vector4(-0.020f, 0.036f, 0.070f, 0.094f),
                new Vector4( 0.050f, 0.040f, 0.074f, 0.100f),
            }, 3.0f, true), orange);
            // ヘッドライトのカバー
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("HeadLamp", hull, new Vector3(sx * 0.060f, 0.050f, 0.328f), new Vector3(0.030f, 0.010f, 0.018f), lamp, 18f);
                Cube("TailLamp", hull, new Vector3(sx * 0.066f, 0.050f, -0.113f), new Vector3(0.022f, 0.008f, 0.004f), tail);
            }
            // リアウイング (黒い翼・緑の翼端板)
            Cube("WingPlate", hull, new Vector3(0f, 0.110f, -0.100f), new Vector3(0.200f, 0.004f, 0.040f), black, -8f);
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("WingEnd", hull, new Vector3(sx * 0.101f, 0.100f, -0.100f), new Vector3(0.003f, 0.036f, 0.050f), green);
                Cube("WingStay", hull, new Vector3(sx * 0.030f, 0.090f, -0.096f), new Vector3(0.004f, 0.030f, 0.016f), black);
            }
            if (withMast) Mast(hull, black, alu, 0.110f, 0.120f);
        }

        // ロードスター ND: 幌を開けたオープン 2 シーター (右ハンドル)。ソウルレッドのメタリック
        void BuildRoadster(Transform hull, bool withMast, MatFn Mat, Material black, Material glass,
                           Material lamp, Material tail, Material alu)
        {
            var red = SoulRed(Mat);
            var seat = Mat(new Color(0.12f, 0.11f, 0.11f), 0.35f);
            var body = new[]
            {
                new Vector4(-0.080f, 0.078f, 0.034f, 0.066f),   // リア
                new Vector4(-0.070f, 0.092f, 0.022f, 0.078f),
                new Vector4(-0.040f, 0.096f, 0.016f, 0.083f),
                new Vector4(-0.034f, 0.097f, 0.068f, 0.083f),   // 後輪アーチ
                new Vector4( 0.034f, 0.097f, 0.068f, 0.080f),
                new Vector4( 0.044f, 0.095f, 0.016f, 0.077f),
                new Vector4( 0.205f, 0.093f, 0.016f, 0.073f),
                new Vector4( 0.219f, 0.097f, 0.068f, 0.074f),   // 前輪アーチ
                new Vector4( 0.292f, 0.097f, 0.068f, 0.064f),
                new Vector4( 0.304f, 0.092f, 0.020f, 0.058f),
                new Vector4( 0.334f, 0.078f, 0.024f, 0.048f),   // ノーズ
                new Vector4( 0.348f, 0.056f, 0.028f, 0.040f),
            };
            Part("Paint", hull, Loft(body, 3.2f, true), red);
            // オープンの室内: ボディ上面より少し高い黒い「くぼみ」で開口部を表す
            Part("Cockpit", hull, Loft(new[]
            {
                new Vector4(-0.008f, 0.060f, 0.072f, 0.0815f),
                new Vector4( 0.020f, 0.076f, 0.072f, 0.0820f),
                new Vector4( 0.120f, 0.076f, 0.071f, 0.0800f),
                new Vector4( 0.142f, 0.066f, 0.071f, 0.0790f),
            }, 4.0f, true), black);
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("SeatBase", hull, new Vector3(sx * 0.036f, 0.084f, 0.050f), new Vector3(0.050f, 0.008f, 0.050f), seat);
                Cube("SeatBack", hull, new Vector3(sx * 0.036f, 0.100f, 0.022f), new Vector3(0.048f, 0.040f, 0.010f), seat, -14f);
                Cube("RollHoop", hull, new Vector3(sx * 0.036f, 0.104f, 0.008f), new Vector3(0.040f, 0.026f, 0.006f), black, -10f);
                Cube("HeadLamp", hull, new Vector3(sx * 0.060f, 0.054f, 0.330f), new Vector3(0.034f, 0.006f, 0.010f), lamp, 14f);
                Cube("TailLamp", hull, new Vector3(sx * 0.064f, 0.066f, -0.080f), new Vector3(0.026f, 0.008f, 0.005f), tail);
            }
            // ステアリング (右ハンドル = +x)
            var wheel = Prim(PrimitiveType.Cylinder, "SteeringWheel", hull, black);
            wheel.localPosition = new Vector3(0.036f, 0.098f, 0.098f);
            wheel.localRotation = Quaternion.Euler(-62f, 0f, 0f);
            wheel.localScale = new Vector3(0.030f, 0.002f, 0.030f);
            // フロントガラス (後ろへ傾けた板と黒い枠)
            Cube("Windshield", hull, new Vector3(0f, 0.098f, 0.150f), new Vector3(0.150f, 0.040f, 0.003f), glass, -58f);
            Cube("ShieldFrame", hull, new Vector3(0f, 0.115f, 0.140f), new Vector3(0.152f, 0.004f, 0.006f), black, -58f);
            if (withMast) Mast(hull, black, alu, 0.080f, 0.060f);
        }

        // RX-7 (FD3S): 曲面だけでできた低いクーペ。全長 4,285 × 全幅 1,760 × 全高 1,230 mm・WB 2,425 mm (模型の縮尺 0.1046)。
        // 低いボンネットの両脇に張り出した前フェンダーの峰 (運転席から見える)、ぐっと絞った客室とダブルバブルの屋根、
        // 盛り上がった後フェンダー、なだらかに落ちるハッチ、尾端の羽根、丸 3 灯のテール、閉じたリトラクタブルライトと五角形の開口
        void BuildRx7(Transform hull, bool withMast, MatFn Mat, Material black, Material glass,
                          Material lamp, Material tail, Material alu)
        {
            var yellow = Paint(Mat(new Color(0.97f, 0.77f, 0.06f), 0.80f, 0.25f));
            // 胴: S(z, 半幅, 下端, 上端の中央, 峰の高さ)。車輪のアーチは BodyLoft が車軸の位置から丸く抜く
            var body = new[]
            {
                S(-0.095f, 0.058f, 0.036f, 0.072f, 0.000f),   // 尾端 (上から見て丸く絞る)
                S(-0.090f, 0.079f, 0.026f, 0.082f, 0.001f),
                S(-0.080f, 0.088f, 0.020f, 0.088f, 0.003f),
                S(-0.066f, 0.091f, 0.017f, 0.091f, 0.006f),   // 後フェンダーの盛り上がり
                S(-0.010f, 0.092f, 0.016f, 0.091f, 0.008f),
                S( 0.050f, 0.090f, 0.016f, 0.088f, 0.006f),
                S( 0.110f, 0.087f, 0.016f, 0.085f, 0.004f),   // ドア (くびれ)
                S( 0.160f, 0.088f, 0.016f, 0.080f, 0.006f),   // カウル
                S( 0.220f, 0.090f, 0.016f, 0.074f, 0.009f),   // 前フェンダーの峰 (ボンネットより 9 mm = 実車 9 cm 高い)
                S( 0.275f, 0.089f, 0.017f, 0.066f, 0.008f),
                S( 0.315f, 0.081f, 0.019f, 0.056f, 0.005f),
                S( 0.340f, 0.072f, 0.021f, 0.048f, 0.003f),   // ノーズ (低く、上から見ると丸い)
                S( 0.357f, 0.054f, 0.021f, 0.043f, 0.000f),
            };
            Part("Paint", hull, BodyLoft(body, 3.0f, 0.040f), yellow);
            // 客室: 胴より細く絞り (上ほど狭い)、屋根は左右 2 つの膨らみ (ダブルバブル)
            var cabin = new[]
            {
                S(-0.074f, 0.056f, 0.080f, 0.086f, 0.000f),   // ハッチの尾 (羽根の付け根)
                S(-0.040f, 0.062f, 0.080f, 0.100f, 0.000f),
                S( 0.000f, 0.065f, 0.080f, 0.112f, 0.002f),
                S( 0.050f, 0.066f, 0.080f, 0.1225f, 0.004f),  // 屋根の頂点 (全高 1,230 mm)
                S( 0.090f, 0.064f, 0.080f, 0.1215f, 0.004f),
                S( 0.120f, 0.060f, 0.080f, 0.110f, 0.002f),
                S( 0.152f, 0.054f, 0.078f, 0.088f, 0.000f),   // 前窓の付け根
            };
            Part("Glass", hull, BodyLoft(cabin, 2.4f, 0f, 0.55f), glass);
            // 屋根の板 (ガラスの上に黄色の屋根。ダブルバブルの膨らみごと 1 mm 外へ)
            Part("Roof", hull, BodyLoft(new[]
            {
                S(0.010f, 0.058f, 0.104f, 0.1145f, 0.0025f),
                S(0.050f, 0.061f, 0.108f, 0.1235f, 0.0042f),
                S(0.092f, 0.059f, 0.106f, 0.1225f, 0.0042f),
                S(0.112f, 0.055f, 0.100f, 0.1160f, 0.0025f),
            }, 2.4f, 0f, 0.55f), yellow);
            // ノーズ: 閉じたリトラクタブルライト (面一なので形には出さない)・五角形の開口・両脇のダクト・つり目の小さなランプ
            foreach (float sx in new[] { -1f, 1f })
            {
                Cube("SideDuct", hull, new Vector3(sx * 0.052f, 0.032f, 0.348f), new Vector3(0.020f, 0.008f, 0.006f), black, 30f);
                Cube("FoxEyeLamp", hull, new Vector3(sx * 0.064f, 0.043f, 0.336f), new Vector3(0.020f, 0.005f, 0.010f), lamp, 34f);
                Cube("SideVent", hull, new Vector3(sx * 0.0895f, 0.050f, 0.188f), new Vector3(0.002f, 0.010f, 0.020f), black);
                Cube("Mirror", hull, new Vector3(sx * 0.073f, 0.085f, 0.150f), new Vector3(0.012f, 0.008f, 0.010f), yellow);
                // 丸 3 灯のテール (外側 2 つが赤、内側がバック灯)
                for (int k = 0; k < 3; k++)
                {
                    var t = Prim(PrimitiveType.Cylinder, "TailLamp", hull, k == 0 ? lamp : tail);
                    t.localPosition = new Vector3(sx * (0.022f + 0.014f * k), 0.062f, -0.0950f + 0.0018f * k);
                    t.localRotation = Quaternion.Euler(90f, 0f, 0f);
                    t.localScale = new Vector3(0.012f, 0.002f, 0.012f);
                }
                Cube("WingStay", hull, new Vector3(sx * 0.074f, 0.086f, -0.076f), new Vector3(0.006f, 0.014f, 0.016f), yellow, -10f);
            }
            Cube("NoseOpening", hull, new Vector3(0f, 0.031f, 0.353f), new Vector3(0.040f, 0.011f, 0.008f), black, 25f);
            // 尾端の黒い帯 (テールランプの台)
            Cube("TailPanel", hull, new Vector3(0f, 0.062f, -0.0952f), new Vector3(0.100f, 0.015f, 0.002f), black);
            Cube("WingPlate", hull, new Vector3(0f, 0.094f, -0.080f), new Vector3(0.156f, 0.003f, 0.020f), yellow, -6f);
            if (withMast) Mast(hull, black, alu, 0.060f, 0.124f);
        }

        // BodyLoft の断面: z, 半幅, 下端, 上端 (中央), 峰 (左右の肩が中央よりどれだけ高いか)
        static float[] S(float z, float hw, float lo, float hi, float hump) => new[] { z, hw, lo, hi, hump };

        /// 曲面の胴。断面を前後方向に Catmull-Rom で細かく (4 mm ごと) つなぎ、断面は超楕円 (n) に
        /// 「肩の峰」(hump: 上側の 72 % 幅あたりを持ち上げる) と「上すぼまり」(pinch: 上ほど幅を絞る) を足す。
        /// archR > 0 なら前後の車軸 (z = 0, Wheelbase) の上を半径 archR の円で抜く (下端を持ち上げる)。
        /// tools/preview_circuit.py が同じ式で描く
        Mesh BodyLoft(float[][] sec, float n, float archR, float pinch = 0f)
        {
            const int Ring = 40;
            float e = 2f / n;
            float z0 = sec[0][0], z1 = sec[sec.Length - 1][0];
            int rings = Mathf.Max(2, Mathf.CeilToInt((z1 - z0) / 0.004f) + 1);
            var verts = new System.Collections.Generic.List<Vector3>();
            var tris = new System.Collections.Generic.List<int>();
            for (int j = 0; j < rings; j++)
            {
                float z = Mathf.Lerp(z0, z1, j / (float)(rings - 1));
                SecAt(sec, z, out float hw, out float lo, out float hi, out float hump);
                if (archR > 0f)
                    foreach (float axle in new[] { 0f, Wheelbase })
                    {
                        float dz = z - axle;
                        if (Mathf.Abs(dz) < archR) lo = Mathf.Max(lo, TireRadius + Mathf.Sqrt(archR * archR - dz * dz) * 0.92f);
                    }
                lo = Mathf.Min(lo, hi - 0.006f);
                float cy = (lo + hi) * 0.5f, hh = (hi - lo) * 0.5f;
                for (int i = 0; i < Ring; i++)
                {
                    float th = i * Mathf.PI * 2f / Ring;
                    float c = Mathf.Cos(th), sn = Mathf.Sin(th);
                    float x = hw * Mathf.Sign(c) * Mathf.Pow(Mathf.Abs(c), e);
                    float y = cy + hh * Mathf.Sign(sn) * Mathf.Pow(Mathf.Abs(sn), e);
                    if (sn > 0f)
                    {
                        float t = Mathf.Abs(x) / Mathf.Max(1e-5f, hw), up = Mathf.Pow(sn, 0.6f);
                        y += hump * Mathf.Exp(-Mathf.Pow((t - 0.72f) / 0.20f, 2f)) * up;
                        x *= 1f - pinch * sn * sn;
                    }
                    verts.Add(new Vector3(x, y, z));
                }
            }
            for (int j = 0; j < rings - 1; j++)
                for (int i = 0; i < Ring; i++)
                {
                    int a = j * Ring + i, b = j * Ring + (i + 1) % Ring;
                    int c = a + Ring, d = b + Ring;
                    tris.AddRange(new[] { a, c, b, b, c, d });
                }
            // 前後の蓋
            foreach (int j in new[] { 0, rings - 1 })
            {
                int center = verts.Count;
                Vector3 m = Vector3.zero;
                for (int i = 0; i < Ring; i++) m += verts[j * Ring + i];
                verts.Add(m / Ring);
                for (int i = 0; i < Ring; i++)
                {
                    int a = j * Ring + i, b = j * Ring + (i + 1) % Ring;
                    if (j == 0) tris.AddRange(new[] { center, a, b });
                    else tris.AddRange(new[] { center, b, a });
                }
            }
            var mesh = new Mesh();
            mesh.SetVertices(verts);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        /// 断面の値を z で補間 (半幅・上端・峰は Catmull-Rom で滑らかに、下端は直線)
        static void SecAt(float[][] sec, float z, out float hw, out float lo, out float hi, out float hump)
        {
            int k = 0;
            while (k < sec.Length - 2 && z > sec[k + 1][0]) k++;
            float za = sec[k][0], zb = sec[k + 1][0];
            float t = Mathf.Clamp01((z - za) / Mathf.Max(1e-6f, zb - za));
            float[] p0 = sec[Mathf.Max(0, k - 1)], p1 = sec[k], p2 = sec[k + 1], p3 = sec[Mathf.Min(sec.Length - 1, k + 2)];
            float CR(int i) => 0.5f * (2f * p1[i] + (-p0[i] + p2[i]) * t + (2f * p0[i] - 5f * p1[i] + 4f * p2[i] - p3[i]) * t * t
                                      + (-p0[i] + 3f * p1[i] - 3f * p2[i] + p3[i]) * t * t * t);
            hw = Mathf.Max(0.002f, CR(1));
            lo = Mathf.Lerp(p1[2], p2[2], t);
            hi = CR(3);
            hump = Mathf.Max(0f, CR(4));
        }

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
