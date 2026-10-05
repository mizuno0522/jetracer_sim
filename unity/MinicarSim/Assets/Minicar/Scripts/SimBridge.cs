// vehicle_sim (ROS) の描画状態を受けてカメラを動かし、/camera/image_raw を配信する。
//
//   /sim/render_state (std_msgs/Float64MultiArray)  ← vehicle_sim (camera_backend:=unity)
//   /camera/image_raw (sensor_msgs/Image, bgr8)       → arrow_detector / lane_detector 等
//
// 物理は vehicle_sim のまま。Unity は「その姿勢から見える画像」を描くだけ。
// カメラ諸元 (解像度・画角・取付高さ・ピッチ・上部クロップ・レート) は
// course.json (= sim.yaml) から取り、vehicle_sim の OpenCV 描画と同じ幾何にする
// (arrow_detector の focal_px と bev_sim.yaml がそのまま使えるように)。
using System;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Rendering;

namespace Minicar
{
    [RequireComponent(typeof(CourseBuilder))]
    public partial class SimBridge : MonoBehaviour
    {
        const string kStateTopic = "/sim/render_state";
        const string kImageTopic = "/camera/image_raw";
        const int kOwnCarLayer = 8;      // センサカメラには写さない (実機でも自車は写らない)
        const int kRivalLayer = 9;       // レース相手 (別ドメインの車)。ゴースト対戦なのでセンサにも写さない
        const int kRival2Layer = 13;     // 3 台レース (race3.sh) の 2 台目の相手
        const string kRivalTopic = "/sim/rival_state";   // race_relay.py が転送する相手の描画状態
        const string kRival2Topic = "/sim/rival2_state";

        // vehicle_sim.RENDER_STATE_FIELDS と同じ並び
        enum F { StampSec, StampNsec, X, Y, Yaw, SimTime, ArrowDir, NarrowDivider, Pitch, OppValid, OppX, OppY, OppYaw, V, Steer, ALat, Distance, Count }

        CourseBuilder m_Course;
        ROSConnection m_Ros;
        Camera m_SensorCam, m_ViewCam, m_RivalCam;
        RenderTexture m_Rt, m_RivalRt;          // m_RivalRt は 2 台レースの相手の車載カメラ (表示専用・配信しない)
        // 実カメラ風の後処理 (course.json の realism)。m_Rt は supersample 倍で描き、m_PostRt (出力解像度) に落とす
        RenderTexture m_PostRt;
        Material m_PostMat;
        RealismData m_Real;
        float m_Exposure = 1f;
        double m_LastYaw = double.NaN, m_LastStamp = double.NaN;
        float m_YawRateAbs;
        RenderTexture SensorOutput => m_PostRt != null ? m_PostRt : m_Rt;
        /// 配信しているセンサ画像 (後処理後・クロップ前)。ML-Agents の画像観測が同じものを使う
        public RenderTexture SensorTexture => SensorOutput;
        /// 最新の /sim/render_state (null = 未着)。並びは F
        public double[] LatestState => m_State;
        public ROSConnection Ros => m_Ros;
        EngineAudio m_Engine;            // 自車のエンジン音 (-sound on|off)
        float m_ViewScale = 1f;          // 追従視点の距離の倍率 (実車スケールでは車体と同じ倍率)
        bool Circuit => m_Course != null && m_Course.Data != null && m_Course.Data.IsCircuit;
        MinicarAgent m_Agent;            // ML-Agents (-mlagents のときだけ)
        CarModel m_OwnCar, m_Opponent, m_Rival, m_Rival2;
        double[] m_RivalState, m_Rival2State;
        string m_OwnLabel, m_RivalLabel, m_Rival2Label;
        int m_Follow;                    // 0 = 自車, 1 = 相手, 2 = 俯瞰
        RvizLayout m_Rviz;               // RViz と同じ表示 (左カメラ / 右コース全景＋速度色)
        // 占有格子 (0 = 自車 /fusion/local_map、1 = 相手 /sim/rival_local_map) と、
        // 格子の stamp に合う姿勢を引くための姿勢履歴 (50 Hz × 64 ≈ 1.3 s)
        readonly OccupancyGridMsg[] m_GridMsg = new OccupancyGridMsg[2];
        readonly PoseHistory[] m_Poses = { new PoseHistory(), new PoseHistory(), new PoseHistory() };

        class PoseHistory
        {
            readonly double[] t = new double[64], x = new double[64], y = new double[64], yaw = new double[64];
            int n;
            public void Add(double[] d)
            {
                int k = n++ % t.Length;
                t[k] = d[(int)F.StampSec] + d[(int)F.StampNsec] * 1e-9;
                x[k] = d[(int)F.X]; y[k] = d[(int)F.Y]; yaw[k] = d[(int)F.Yaw];
            }
            public bool Nearest(double stamp, out double px, out double py, out double pyaw)
            {
                int best = -1;
                double bestDt = double.MaxValue;
                for (int i = 0; i < System.Math.Min(n, t.Length); i++)
                {
                    double dt = System.Math.Abs(t[i] - stamp);
                    if (dt < bestDt) { bestDt = dt; best = i; }
                }
                px = best >= 0 ? x[best] : 0; py = best >= 0 ? y[best] : 0; pyaw = best >= 0 ? yaw[best] : 0;
                return best >= 0;
            }
        }
        AicLayout m_Aic;                 // 自動運転 AI チャレンジと同じ表示 (車載視点の 2 分割 + HUD + ミニマップ)
        enum Layout { Aic, Rviz, Chase }
        Layout m_Layout;
        // 表示用の車体姿勢。状態は 50 Hz・到着は不揃いなので、そのまま置くと画面 (100 fps 超) で
        // カクつく。状態の stamp を時間軸にして、最新の状態から速度とヨーレートで「今」まで補外する。
        // (配信するセンサカメラは補外しない。画像の stamp = その姿勢の stamp を保つ)
        class PoseSmoother
        {
            double m_Stamp = -1, m_X, m_Y, m_Yaw, m_V, m_YawRate, m_Offset;
            public bool Valid => m_Stamp >= 0;

            public void Add(double[] d)
            {
                double stamp = d[(int)F.StampSec] + d[(int)F.StampNsec] * 1e-9;
                double yaw = d[(int)F.Yaw];
                double now = Time.realtimeSinceStartupAsDouble;
                double dt = stamp - m_Stamp;
                if (m_Stamp < 0 || dt <= 0 || dt > 0.5)
                {
                    m_YawRate = 0;
                    m_Offset = now - stamp;
                }
                else
                {
                    double dy = yaw - m_Yaw;
                    while (dy > Math.PI) dy -= 2 * Math.PI;
                    while (dy < -Math.PI) dy += 2 * Math.PI;
                    m_YawRate = dy / dt;
                    // 到着時刻 − stamp の平均 (到着のばらつきを均す)
                    m_Offset += (now - stamp - m_Offset) * 0.05;
                }
                m_Stamp = stamp; m_X = d[(int)F.X]; m_Y = d[(int)F.Y]; m_Yaw = yaw; m_V = d[(int)F.V];
            }

            public void Get(out double x, out double y, out double yaw)
            {
                double dt = Math.Max(0.0, Math.Min(0.06, Time.realtimeSinceStartupAsDouble - m_Offset - m_Stamp));
                yaw = m_Yaw + m_YawRate * dt;
                double mid = m_Yaw + m_YawRate * dt * 0.5;
                x = m_X + m_V * Math.Cos(mid) * dt;
                y = m_Y + m_V * Math.Sin(mid) * dt;
            }
        }
        readonly PoseSmoother[] m_Smooth = { new PoseSmoother(), new PoseSmoother(), new PoseSmoother() };

        bool m_RvizMode => m_Layout == Layout.Rviz;
        Vector3 m_ChasePos;
        bool m_ChaseInit;
        string m_ShotDir;
        float m_NextShot;
        float m_ShotInterval = 2f;
        int m_ShotN;
        double[] m_State;
        bool m_StateDirty;
        int m_ArrowDirShown = -1;
        float m_NextCapture;
        bool m_ReadbackBusy;
        int m_CropRows;
        byte[] m_Out;
        int m_Published;
        float m_RateT0;
        float m_PubRate;
        bool m_FullSensorView;

        void Awake()
        {
            QualitySettings.vSyncCount = 0;       // 垂直同期に縛られると配信レートが 60/n Hz に丸まる
            // 描画の上限。カメラは 30 Hz で配るので 60 で足りる (1 コマおきに配る)。120 では描画だけで CPU を
            // 使い、3 台レースで PC が詰まった (✎ 2026-09-28)。-fps で変えられる
            Application.targetFrameRate = int.Parse(Arg("-fps", "60"));
            RenderQuality.Init();                 // -quality low|medium|high|auto (コースを組む前に決める)
            m_Course = GetComponent<CourseBuilder>();
            float t0 = Time.realtimeSinceStartup;
            m_Course.Build();
            m_BuildSeconds = Time.realtimeSinceStartup - t0;
            Debug.Log($"[SimBridge] course built in {m_BuildSeconds:F2} s (process up {Time.realtimeSinceStartup:F2} s)");

            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.listenForTFMessages = false;
            m_Ros.RosIPAddress = Arg("-rosip", "127.0.0.1");
            m_Ros.RosPort = int.Parse(Arg("-rosport", "10000"));
        }

        void Start()
        {
            var cam = m_Course.Data.camera;
            m_CropRows = Mathf.RoundToInt(cam.height * Mathf.Clamp(cam.crop_top_frac, 0f, 0.6f));
            m_Out = new byte[(cam.height - m_CropRows) * cam.width * 3];

            BuildCars();
            BuildSensorCamera(cam);
            BuildViewCamera();
            // エンジン音: -sound on|off (既定: 実車スケールのコースでボディを選んだときだけ on)。-volume 0〜1
            // ミニカーの会場は電動のラジコンでエンジンが無いので、既定では鳴らさない (2026-10-05 水野)
            string sound = Arg("-sound", "auto");
            if (sound == "auto" || sound == "") sound = Circuit && m_OwnCar.Style != CarStyle.Default ? "on" : "off";
            if (sound == "on")
            {
                var v = m_Course.Data.vehicle;
                string vmaxDef = v != null && v.v_max_mps > 0f ? v.v_max_mps.ToString(System.Globalization.CultureInfo.InvariantCulture) : "3.0";
                float alat = v != null && v.a_lat_max_mps2 > 0f ? v.a_lat_max_mps2 : 4.4f;
                m_Engine = EngineAudio.Create(gameObject, m_OwnCar.Style,
                    float.Parse(Arg("-volume", "0.6"), System.Globalization.CultureInfo.InvariantCulture),
                    float.Parse(Arg("-soundvmax", vmaxDef), System.Globalization.CultureInfo.InvariantCulture), alat);
            }
            // ML-Agents: -mlagents で有効 (sim_mode:=lockstep と mlagents_gateway.py が前提。docs/mlagents.md)
            if (Array.IndexOf(Environment.GetCommandLineArgs(), "-mlagents") >= 0)
                m_Agent = MinicarAgent.Create(this, Arg);
            // -layout aic (既定) | rviz | chase、-route shortcut (既定) | long (周回・ミニマップに使う中心線)
            m_Rviz = new RvizLayout(m_Course.Data);
            // -laps N: N 周でゴール (結果画面を出す)。既定 3 = 予選 (3 周の合計タイム)。0 = 出さない
            m_Aic = new AicLayout(m_Course.Data, Arg("-route", "shortcut") == "long", int.Parse(Arg("-laps", "3")),
                                  new[] { m_OwnLabel, m_RivalLabel, m_Rival2Label },
                                  new[] { m_OwnCar.Root, m_Rival.Root, m_Rival2.Root },
                                  new[] { kOwnCarLayer, kRivalLayer, kRival2Layer });
            // 下段の「CAMERA」は表示用カメラではなく、実際に配信している画像 (後処理後・224×224) をそのまま見せる
            m_Aic.SensorTexture = SensorOutput;
            // -aicview both (既定: 上 = 追従視点・下 = カメラ映像) | chase | camera
            string view = Arg("-aicview", "both");
            m_Aic.ViewMode = view == "chase" ? AicLayout.View.Chase : view == "camera" ? AicLayout.View.Onboard : AicLayout.View.Both;
            string layout = Arg("-layout", "aic");
            SetLayout(layout == "rviz" ? Layout.Rviz : layout == "chase" ? Layout.Chase : Layout.Aic);
            m_Rviz.Update();            // 接続前から右パネルの視野を合わせておく
            RenderQuality.ApplyBuiltin(Circuit);
            // 版 (Built-in / URP / HDRP) ごとの材質・光・空・霞・後処理に置き換える。絵の仕様は 3 版で同じ (docs/hdrp.md)
            RenderCompat.AfterBuild(m_Course, new[] { m_SensorCam, m_RivalCam }, Circuit);

            // -shotdir <dir> [-shotinterval 秒]: 画面 (追従視点 + センサ画像) を一定間隔で PNG で保存 (見た目の確認用)
            // -record <file.mp4> [-recordfps 30] [-recordwidth 1280]: 画面をそのまま ffmpeg で録画 (ScreenRecorder)
            // -recordfrom countdown: 録画をスタートのカウントダウン (/sim/countdown) から始める (起動待ちを録らない)
            m_RecordPending = Arg("-recordfrom", "start") == "countdown";
            if (!m_RecordPending) StartRecording();
            m_ShotDir = Arg("-shotdir", "");
            m_ShotInterval = float.Parse(Arg("-shotinterval", "2"), System.Globalization.CultureInfo.InvariantCulture);
            // -shots "0,1250,…": ROS 無しで決めた位置に車を置いて撮影・計測して終了 (SimBridge.Shots.cs)
            if (Arg("-shots", "") != "") StartShots();
            m_Ros.RegisterPublisher<ImageMsg>(kImageTopic);
            m_Ros.Subscribe<Float64MultiArrayMsg>(kStateTopic, OnState);
            // エピソード seed (vehicle_sim が LATCHED で出す)。照明・床・観戦者を引き直す
            m_Ros.Subscribe<UInt32Msg>("/sim/episode", msg => { m_Course.ApplyEpisode(msg.data); RenderCompat.Refresh(); });
            // 起動時の seed は引数 -seed で受ける (/sim/episode は LATCHED だが endpoint 経由の購読は
            // 接続前の latched メッセージを受け取れないため)。走行中の引き直し (lockstep Reset) は上の購読で
            if (uint.TryParse(Arg("-seed", ""), out uint seed0)) { m_Course.ApplyEpisode(seed0); RenderCompat.Refresh(); }
            // 占有格子 (RViz の FusionGrid と同じもの)。全景に重ねる
            m_Ros.Subscribe<OccupancyGridMsg>("/fusion/local_map", msg => m_GridMsg[0] = msg);
            m_Ros.Subscribe<OccupancyGridMsg>("/sim/rival_local_map", msg => m_GridMsg[1] = msg);
            m_Ros.Subscribe<Float64MultiArrayMsg>(kRivalTopic, msg =>
            {
                if (msg.data != null && msg.data.Length >= (int)F.Count) { m_RivalState = msg.data; m_Poses[1].Add(msg.data); m_Smooth[1].Add(msg.data); }
            });
            // AI チャレンジ表示: スタートのカウントダウン (race_start.py) と周回数 (lap_counter。相手ぶんは race_relay.py)
            m_Ros.Subscribe<Int32Msg>("/sim/countdown", msg =>
            {
                if (m_RecordPending) { m_RecordPending = false; StartRecording(); }
                m_Aic.SetCountdown(msg.data);
            });
            m_Ros.Subscribe<Int32Msg>("/lap_count", msg => m_Aic.SetLapCount(0, msg.data));
            m_Ros.Subscribe<Int32Msg>("/sim/rival_lap_count", msg => m_Aic.SetLapCount(1, msg.data));
            // 3 台レース (race3.sh) の 2 台目の相手
            m_Ros.Subscribe<Float64MultiArrayMsg>(kRival2Topic, msg =>
            {
                if (msg.data != null && msg.data.Length >= (int)F.Count) { m_Rival2State = msg.data; m_Poses[2].Add(msg.data); m_Smooth[2].Add(msg.data); }
            });
            m_Ros.Subscribe<Int32Msg>("/sim/rival2_lap_count", msg => m_Aic.SetLapCount(2, msg.data));
            m_RateT0 = Time.unscaledTime;
        }

        bool m_RecordPending;

        void StartRecording()
        {
            ScreenRecorder.StartIfRequested(gameObject, Arg("-record", ""),
                float.Parse(Arg("-recordfps", "30"), System.Globalization.CultureInfo.InvariantCulture),
                int.Parse(Arg("-recordwidth", "1280")));
        }

        void SetLayout(Layout layout)
        {
            m_Layout = layout;
            m_Aic.Enabled = layout == Layout.Aic;
            m_Ros.ShowHud = layout != Layout.Aic;      // 接続 HUD は左上の P1 パネルに重なる
            m_Rviz.Enabled = layout == Layout.Rviz;
            m_ViewCam.enabled = layout == Layout.Chase;
        }

        internal static string Arg(string name, string def)
        {
            var a = Environment.GetCommandLineArgs();
            for (int i = 0; i < a.Length - 1; i++)
                if (a[i] == name)
                {
                    // launch が空の値を落とすと次の "-xxx" を値と読んでしまうので、それは「無し」とする
                    string v = a[i + 1];
                    if (v.Length > 1 && v[0] == '-' && !char.IsDigit(v[1]) && v[1] != '.') return def;
                    return v;
                }
            return def;
        }

        // ------------------------------------------------------------------
        void BuildSensorCamera(CameraData c)
        {
            // 自分の車体は写さず、相手の車は写す (前を走る車がカメラに入る)。
            // ★LiDAR・物理には相手が居ない (ゴースト対戦) ので、青の制御に入る画像と
            //   測距が食い違う。-camghost を付けると相手も写さない (センサ入力を揃える)。
            bool ghost = Array.IndexOf(Environment.GetCommandLineArgs(), "-camghost") >= 0;
            m_Real = m_Course.Realism ?? new RealismData();
            m_SensorCam = MakeSensorCamera(c, "SensorCamera", out m_Rt, kOwnCarLayer, ghost ? kRivalLayer : -1,
                                           m_Real.enable ? Mathf.Max(1, m_Real.supersample) : 1);
            if (m_Real.enable)
            {
                var shaderMat = Resources.Load<Material>("Mat_SensorPost");
                if (shaderMat == null) Debug.LogError("[SimBridge] Mat_SensorPost が無い (MinicarBuild.MakeMaterials)");
                else
                {
                    m_PostMat = new Material(shaderMat);
                    m_PostRt = new RenderTexture(c.width, c.height, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { name = "SensorPostRT" };
                    ApplyRealismParams();
                }
            }
            m_RivalCam = MakeSensorCamera(c, "RivalSensorCamera", out m_RivalRt, kRivalLayer, ghost ? kOwnCarLayer : -1);
        }

        Camera MakeSensorCamera(CameraData c, string name, out RenderTexture rt, int selfLayer, int hiddenLayer, int ss = 1)
        {
            rt = new RenderTexture(c.width * ss, c.height * ss, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB)
            { name = name + "RT", antiAliasing = 1 };
            var go = new GameObject(name);
            var cam = go.AddComponent<Camera>();
            cam.nearClipPlane = Circuit ? 0.2f : 0.02f;
            cam.farClipPlane = Circuit ? 50000f : 30f;      // 実車スケール: 富士山 (18 km 先・裾は 30 km まで) と雲のドーム (40 km) まで
            ApplyIntrinsics(cam, c);
            cam.clearFlags = CameraClearFlags.SolidColor;
            // 背景: realism.background_gray が負なら Unity 既定のスカイボックス (水色)、それ以外は単色
            float bgv = (m_Real != null && m_Real.enable) ? m_Real.background_gray : 110f;
            if (bgv < 0f || Circuit) cam.clearFlags = CameraClearFlags.Skybox;
            byte bg = (byte)Mathf.Clamp(bgv, 0, 255);
            cam.backgroundColor = new Color32(bg, bg, bg, 255);     // 会場の背景 (既定は OpenCV 描画と同じ 110)
            int hide = (1 << selfLayer) | (1 << RvizLayout.OverviewOnlyLayer);
            if (hiddenLayer >= 0) hide |= 1 << hiddenLayer;
            cam.cullingMask = ~hide;
            cam.targetTexture = rt;
            cam.enabled = false;        // 描くのは配信タイミングだけ (Render を手で呼ぶ)
            return cam;
        }

        /// <summary>
        /// センサカメラの投影を vehicle_profile のカメラ幾何に合わせる (vehicle_sim の OpenCV 描画と同じ)。
        /// 歪みありのときは、歪みの逆写像が参照する範囲 render_tan_* をピンホールで描き (非対称・非正方の off-axis 透視)、
        /// SensorPost が出力画素ごとに歪みを解いてサンプルする。古い course.json は fov_deg の正方ピンホール。
        /// </summary>
        public static void ApplyIntrinsics(Camera cam, CameraData c)
        {
            if (c.HasIntrinsics)
            {
                float n = cam.nearClipPlane;
                // 正規化 y は下向き、Unity のビュー空間は上向きなので上下を入れ替える
                cam.projectionMatrix = Matrix4x4.Frustum(
                    c.render_tan_x0 * n, c.render_tan_x1 * n,
                    -c.render_tan_y1 * n, -c.render_tan_y0 * n, n, cam.farClipPlane);
                return;
            }
            // vehicle_sim と同じピンホール: 水平画角 fov → 焦点距離 f → 垂直画角
            float f = (c.width * 0.5f) / Mathf.Tan(c.fov_deg * 0.5f * Mathf.Deg2Rad);
            cam.fieldOfView = 2f * Mathf.Atan((c.height * 0.5f) / f) * Mathf.Rad2Deg;
            cam.aspect = (float)c.width / c.height;
        }

        /// 歪み r(1 + k1 r² + k2 r⁴) が単調に増える最大の r² (cam_geom.r_max と同じ)。単調なら大きな値
        static float MonotonicR2(float k1, float k2)
        {
            // d/dr = 1 + 3 k1 s + 5 k2 s² (s = r²) が 0 になる最小の正の s
            const float kInf = 1e6f;
            if (Mathf.Abs(k2) < 1e-12f) return k1 < 0f ? -1f / (3f * k1) : kInf;
            float a = 5f * k2, b = 3f * k1, disc = b * b - 4f * a;
            if (disc < 0f) return kInf;
            float sq = Mathf.Sqrt(disc), s1 = (-b - sq) / (2f * a), s2 = (-b + sq) / (2f * a);
            float best = kInf;
            if (s1 > 0f) best = Mathf.Min(best, s1);
            if (s2 > 0f) best = Mathf.Min(best, s2);
            return best;
        }

        void ApplyRealismParams()
        {
            var r = m_Real;
            int ss = Mathf.Max(1, r.supersample);
            var c = m_Course.Data.camera;
            if (c.HasIntrinsics)
            {
                m_PostMat.SetFloat("_Distort", 1f);
                m_PostMat.SetVector("_Intr", new Vector4(c.fx, c.fy, c.cx, c.cy));
                m_PostMat.SetVector("_Dist", new Vector4(c.k1, c.k2, MonotonicR2(c.k1, c.k2), 0f));
                m_PostMat.SetVector("_Tan", new Vector4(c.render_tan_x0, c.render_tan_x1, c.render_tan_y0, c.render_tan_y1));
                m_PostMat.SetVector("_OutSize", new Vector4(c.width, c.height, 0f, 0f));
            }
            else m_PostMat.SetFloat("_Distort", 0f);
            m_PostMat.SetFloat("_Vignette", r.vignette);
            m_PostMat.SetFloat("_BlurPx", r.blur_px * ss);
            m_PostMat.SetFloat("_Gamma", r.gamma);
            m_PostMat.SetFloat("_Noise", r.noise);
            m_PostMat.SetFloat("_PostDark", r.post_dark);
            var p = r.posts ?? new float[0];
            m_PostMat.SetVector("_Post0", p.Length >= 3 ? new Vector4(p[0], p[1], p[2], 0.006f) : new Vector4(-1, -1, 2, 0.001f));
            m_PostMat.SetVector("_Post1", p.Length >= 6 ? new Vector4(p[3], p[4], p[5], 0.006f) : new Vector4(-1, -1, 2, 0.001f));
        }

        void BuildViewCamera()
        {
            // 画面表示用の追従カメラ (配信には使わない)
            m_ViewCam = new GameObject("ChaseCamera").AddComponent<Camera>();
            m_ViewCam.fieldOfView = 50f;
            m_ViewCam.nearClipPlane = Circuit ? 0.5f : 0.02f;
            m_ViewCam.farClipPlane = Circuit ? 50000f : 60f;
            m_ViewCam.clearFlags = Circuit ? CameraClearFlags.Skybox : CameraClearFlags.SolidColor;   // サーキットは空を描く
            m_ViewCam.backgroundColor = new Color32(40, 42, 46, 255);
            // 接続前 (/sim/render_state 未着) はコース全体を斜め上から見せる。
            // 原点のままだと床下から写って何も見えない
            PlaceOverview();
        }

        // 俯瞰: コース全体を斜め上から (ミニカーは従来の位置、サーキットは外接矩形から)
        void PlaceOverview()
        {
            if (!Circuit)
            {
                m_ViewCam.transform.position = RosFrame.ToUnity(5.1f, -3.2f, 6.5f);
                m_ViewCam.transform.LookAt(RosFrame.ToUnity(5.1f, 3.2f, 0f));
                return;
            }
            var b = m_Course.CircuitBounds;
            float cx = (b[0] + b[2]) * 0.5f, cy = (b[1] + b[3]) * 0.5f, w = b[2] - b[0], h = b[3] - b[1];
            m_ViewCam.transform.position = RosFrame.ToUnity(cx, cy - h * 0.9f, Mathf.Max(w, h) * 0.55f);
            m_ViewCam.transform.LookAt(RosFrame.ToUnity(cx, cy, 0f));
        }

        void BuildCars()
        {
            // 自車: 動画の濃紺メタリック。センサカメラには写さない
            // -owncar / -rivalcar / -rival2car: b787 | nd | rx7 (省略で従来の見た目)。見た目だけで物理は変わらない
            // 屋根のセンサマスト (カメラの柱) はミニカーの会場では付けない (2026-10-05 水野)
            bool mast = Circuit;
            m_OwnCar = new CarModel("OwnCar", new Color(0.06f, 0.10f, 0.42f), kOwnCarLayer, mast,
                                    CarModel.ParseStyle(Arg("-owncar", "")));
            m_Opponent = new CarModel("Opponent", new Color(0.85f, 0.85f, 0.83f), 0, false);
            m_Opponent.Root.gameObject.SetActive(false);
            // レース相手 (黄)。/sim/rival_state が来たときだけ出す
            m_Rival = new CarModel("Rival", new Color(0.95f, 0.72f, 0.05f), kRivalLayer, mast,
                                   CarModel.ParseStyle(Arg("-rivalcar", "")));
            m_Rival.Root.gameObject.SetActive(false);
            m_OwnLabel = Arg("-ownlabel", "BLUE");
            m_RivalLabel = Arg("-rivallabel", "YELLOW");
            // 3 台レースの 2 台目の相手 (緑)。/sim/rival2_state が来たときだけ出す
            m_Rival2 = new CarModel("Rival2", new Color(0.20f, 0.75f, 0.30f), kRival2Layer, mast,
                                    CarModel.ParseStyle(Arg("-rival2car", "")));
            // 実車スケールのコース: 車体を vehicle_profile の全長に合わせて拡大する (見た目だけ)
            var veh = m_Course.Data.vehicle;
            if (m_Course.Data.IsCircuit && veh != null && veh.length_m > 0f)
            {
                // 実車の 3 台 (787B・ND・RX-7) は、それぞれ自分の実車の寸法にする (自車の全長に合わせると、車種の違う相手の大きさが狂う)。
                // 従来のボディだけ、vehicle_profile の全長に合わせて拡大する
                foreach (var car in new[] { m_OwnCar, m_Opponent, m_Rival, m_Rival2 })
                    car.SetScale(car.RealScale > 0f ? car.RealScale : veh.length_m / car.ModelLength);
                // センサマストは実車には無いので、拡大した見た目では隠す
                foreach (var car in new[] { m_OwnCar, m_Rival, m_Rival2 })
                    foreach (var t in car.Root.GetComponentsInChildren<Transform>(true))
                        if (t.name.StartsWith("Mast")) t.gameObject.SetActive(false);
            }
            else
            {
                // ミニカーの会場: 実車の 3 台はどれも実車の 1/10 (車種ごとの大きさの違いを保つ)
                foreach (var car in new[] { m_OwnCar, m_Rival, m_Rival2 })
                    if (car.RealScale > 0f) car.SetScale(0.1f * car.RealScale);
            }
            m_ViewScale = m_Course.Data.IsCircuit ? m_OwnCar.Scale : 1f;
            m_Rival2.Root.gameObject.SetActive(false);
            m_Rival2Label = Arg("-rival2label", "GREEN");
        }

        // ------------------------------------------------------------------
        void OnState(Float64MultiArrayMsg msg)
        {
            if (msg.data == null || msg.data.Length < (int)F.Count) return;
            m_State = msg.data;
            m_StateDirty = true;
            m_Poses[0].Add(msg.data);
            m_Smooth[0].Add(msg.data);
        }

        double S(F f) => m_State[(int)f];

        void Update()
        {
            PollKeys();
            m_Aic.Update();
            if (m_State == null) return;

            if (m_StateDirty)
            {
                m_StateDirty = false;
                ApplyState();
                RenderCompat.Tick();
            }

            UpdateVisuals(Time.deltaTime);

            var cam = m_Course.Data.camera;
            float period = 1f / Mathf.Max(1f, cam.rate_hz);
            if (Time.unscaledTime >= m_NextCapture && !m_ReadbackBusy)
            {
                // 予定時刻を period ずつ進める (「今 + period」だとフレームの端数ぶん
                // 毎回遅れて、30 Hz 指定が 20 Hz に落ちていた)
                m_NextCapture += period;
                if (Time.unscaledTime - m_NextCapture > period) m_NextCapture = Time.unscaledTime + period;
                Capture();
            }

            if (Time.unscaledTime - m_RateT0 >= 2f)
            {
                m_PubRate = m_Published / (Time.unscaledTime - m_RateT0);
                m_Published = 0;
                m_RateT0 = Time.unscaledTime;
            }
        }

        void UpdateVisuals(float dt)
        {
            m_Smooth[0].Get(out double sx, out double sy, out double syaw);
            m_OwnCar.Root.SetPositionAndRotation(RosFrame.ToUnity(sx, sy, 0.0), RosFrame.Yaw(syaw));
            m_OwnCar.Apply((float)S(F.V), (float)S(F.Steer), (float)S(F.ALat), dt);
            if (m_Engine != null) m_Engine.SetState((float)S(F.V), (float)S(F.ALat), dt);
            m_Rviz.AddSample(0, S(F.X), S(F.Y), S(F.V), S(F.Distance));
            if (m_RivalState != null)
                m_Rviz.AddSample(1, m_RivalState[(int)F.X], m_RivalState[(int)F.Y], m_RivalState[(int)F.V], m_RivalState[(int)F.Distance]);
            for (int gi = 0; gi < m_GridMsg.Length; gi++)
            {
                var g = m_GridMsg[gi];
                if (g == null) continue;
                m_GridMsg[gi] = null;
                double t = g.header.stamp.sec + g.header.stamp.nanosec * 1e-9;
                if (m_Poses[gi].Nearest(t, out double px, out double py, out double pyaw))
                    m_Rviz.SetGrid(gi, g, px, py, pyaw);
            }
            m_Rviz.Update();
            m_Aic.SetState(0, S(F.X), S(F.Y), S(F.Yaw), S(F.V), S(F.SimTime), S(F.Distance));
            if (m_RivalState != null)
                m_Aic.SetState(1, m_RivalState[(int)F.X], m_RivalState[(int)F.Y], m_RivalState[(int)F.Yaw], m_RivalState[(int)F.V],
                               m_RivalState[(int)F.SimTime], m_RivalState[(int)F.Distance]);
            if (m_Rival2State != null)
            {
                double[] r2 = m_Rival2State;
                m_Aic.SetState(2, r2[(int)F.X], r2[(int)F.Y], r2[(int)F.Yaw], r2[(int)F.V], r2[(int)F.SimTime], r2[(int)F.Distance]);
                m_Rival2.Root.gameObject.SetActive(true);
                m_Smooth[2].Get(out double gx2, out double gy2, out double gyaw2);
                m_Rival2.Root.SetPositionAndRotation(RosFrame.ToUnity(gx2, gy2, 0.0), RosFrame.Yaw(gyaw2));
                m_Rival2.Apply((float)r2[(int)F.V], (float)r2[(int)F.Steer], (float)r2[(int)F.ALat], dt);
            }
            if (m_Opponent.Root.gameObject.activeSelf) m_Opponent.Apply(0.6f, 0f, 0f, dt);
            if (m_RivalState != null)
            {
                double[] r = m_RivalState;
                m_Rival.Root.gameObject.SetActive(true);
                m_Smooth[1].Get(out double rx, out double ry, out double ryaw);
                m_Rival.Root.SetPositionAndRotation(RosFrame.ToUnity(rx, ry, 0.0), RosFrame.Yaw(ryaw));
                m_Rival.Apply((float)r[(int)F.V], (float)r[(int)F.Steer], (float)r[(int)F.ALat], dt);
            }

            if (m_Follow == 2)
            {
                // 俯瞰: コース全体を斜め上から
                PlaceOverview();
                m_ChaseInit = false;
                return;
            }
            // 追従カメラ: 車の斜め後ろ上から。位置だけ一次遅れで追い、揺れを抑える
            var car = (m_Follow == 1 && m_RivalState != null) ? m_Rival.Root : m_OwnCar.Root;
            float k = m_ViewScale;          // 実車スケールでは距離も車体と同じ倍率
            Vector3 center = car.position + car.forward * 0.105f * k;
            Vector3 want = center - car.forward * 0.95f * k + car.right * 0.25f * k + Vector3.up * 0.42f * k;
            if (!m_ChaseInit) { m_ChasePos = want; m_ChaseInit = true; }
            m_ChasePos = Vector3.Lerp(m_ChasePos, want, 1f - Mathf.Exp(-dt / 0.18f));
            m_ViewCam.transform.position = m_ChasePos;
            m_ViewCam.transform.LookAt(center + (Vector3.up * 0.06f + car.forward * 0.25f) * k);

            if (m_ShotDir != "" && !m_ShotMode && Time.unscaledTime >= m_NextShot)
            {
                m_NextShot = Time.unscaledTime + m_ShotInterval;
                ScreenCapture.CaptureScreenshot(System.IO.Path.Combine(m_ShotDir, $"shot_{m_ShotN++:000}.png"));
            }
        }

        bool m_KeysAvailable = true;

        void PollKeys()
        {
            // 旧 Input Manager が無効な設定だと例外になるので、その場合はキー操作を諦める
            if (!m_KeysAvailable) return;
            try
            {
                if (Input.GetKeyDown(KeyCode.C)) m_FullSensorView = !m_FullSensorView;
                if (Input.GetKeyDown(KeyCode.V))
                {
                    // AI チャレンジ表示では画面構成 (2 段 / 追従のみ / カメラのみ)、追従表示では追う車を切り替える
                    if (m_Layout == Layout.Aic) m_Aic.ViewMode = (AicLayout.View)(((int)m_Aic.ViewMode + 1) % 3);
                    else { m_Follow = (m_Follow + 1) % 3; m_ChaseInit = false; }
                }
                if (Input.GetKeyDown(KeyCode.Escape) && m_Layout == Layout.Aic && m_Aic.ShowingResult) Application.Quit();
                if (Input.GetKeyDown(KeyCode.L)) SetLayout((Layout)(((int)m_Layout + 1) % 3));
                if (Input.GetKeyDown(KeyCode.M) && m_Engine != null) m_Engine.Muted = !m_Engine.Muted;
            }
            catch (InvalidOperationException)
            {
                m_KeysAvailable = false;
            }
        }

        void ApplyState()
        {
            var cam = m_Course.Data.camera;
            double x = S(F.X), y = S(F.Y), yaw = S(F.Yaw);
            double stamp = S(F.StampSec) + S(F.StampNsec) * 1e-9;
            if (!double.IsNaN(m_LastStamp) && stamp - m_LastStamp > 1e-4 && stamp - m_LastStamp < 0.5)
            {
                double dy = yaw - m_LastYaw;
                while (dy > Math.PI) dy -= 2 * Math.PI;
                while (dy < -Math.PI) dy += 2 * Math.PI;
                m_YawRateAbs = (float)Math.Abs(dy / (stamp - m_LastStamp));
            }
            m_LastYaw = yaw; m_LastStamp = stamp;

            // 車体 (表示) は UpdateVisuals が毎フレーム滑らかに置く。センサカメラはこの stamp の姿勢そのまま
            m_SensorCam.transform.SetPositionAndRotation(
                RosFrame.ToUnity(x, y, cam.mount_height_m),
                RosFrame.YawPitch(yaw, cam.pitch_deg * Mathf.Deg2Rad));

            bool opp = S(F.OppValid) > 0.5;
            m_Opponent.Root.gameObject.SetActive(opp);
            if (opp)
                m_Opponent.Root.SetPositionAndRotation(RosFrame.ToUnity(S(F.OppX), S(F.OppY), 0.0), RosFrame.Yaw(S(F.OppYaw)));

            if (m_Course.Divider != null)
                m_Course.Divider.SetActive(S(F.NarrowDivider) > 0.5);

            int dir = (int)S(F.ArrowDir);
            if (dir != m_ArrowDirShown && m_Course.ArrowFrontMaterial != null)
            {
                m_ArrowDirShown = dir;
                var old = m_Course.ArrowFrontMaterial.mainTexture;
                m_Course.ArrowFrontMaterial.mainTexture = ArrowTexture.Make(dir);
                if (old != null) Destroy(old);
            }

            // ③ライトかく乱: 7 色を 2 秒ごと・明滅は sim 時間基準 (vehicle_sim の _scene_light と同じ周期)
            var light = m_Course.DisturbLight;
            if (light != null)
            {
                double t = S(F.SimTime);
                light.intensity = 2.5f * (1f + 0.46f * Mathf.Sin((float)(t * 6.0)));
                light.color = kTints[(int)(t / 2.0) % kTints.Length];
            }
        }

        static readonly Color[] kTints =
        {
            new Color(1f, 1f, 1f), new Color(1f, 0.55f, 0.55f), new Color(0.55f, 1f, 0.55f),
            new Color(0.55f, 0.55f, 1f), new Color(1f, 0.95f, 0.5f), new Color(0.5f, 0.95f, 1f),
            new Color(1f, 0.55f, 0.95f),
        };

        // ------------------------------------------------------------------
        void Capture()
        {
            // この画像はこの姿勢 (= この stamp の状態) から描いたもの
            var stamp = new TimeMsg((int)S(F.StampSec), (uint)S(F.StampNsec));
            m_SensorCam.Render();
            if (m_RivalState != null && m_RvizMode)
            {
                // 相手の車載カメラ (RViz 表示の左パネルでだけ使う): 同じ取付・同じ画角で、相手の姿勢から描く (表示専用)
                var c = m_Course.Data.camera;
                m_RivalCam.transform.SetPositionAndRotation(
                    RosFrame.ToUnity(m_RivalState[(int)F.X], m_RivalState[(int)F.Y], c.mount_height_m),
                    RosFrame.YawPitch(m_RivalState[(int)F.Yaw], c.pitch_deg * Mathf.Deg2Rad));
                m_RivalCam.Render();
            }
            if (m_PostMat != null)
            {
                int ss = Mathf.Max(1, m_Real.supersample);
                m_PostMat.SetFloat("_Exposure", m_Exposure);
                m_PostMat.SetFloat("_Seed", (float)(Time.unscaledTimeAsDouble % 97.0));
                m_PostMat.SetFloat("_MotionPx", m_Real.motion_px_per_rad_s * m_YawRateAbs * ss);
                Graphics.Blit(m_Rt, m_PostRt, m_PostMat);
            }
            m_ReadbackBusy = true;
            AsyncGPUReadback.Request(SensorOutput, 0, TextureFormat.RGB24, req => OnReadback(req, stamp));
        }

        void OnReadback(AsyncGPUReadbackRequest req, TimeMsg stamp)
        {
            m_ReadbackBusy = false;
            if (req.hasError) return;
            var src = req.GetData<byte>();
            var c = m_Course.Data.camera;
            int w = c.width, h = c.height, outH = h - m_CropRows;
            // GPU の行は下から上。画像の上端から m_CropRows 行を捨て、BGR に並べ替える
            for (int r = 0; r < outH; r++)
            {
                int srcRow = h - 1 - (r + m_CropRows);
                int so = srcRow * w * 3, oo = r * w * 3;
                for (int i = 0; i < w; i++, so += 3, oo += 3)
                {
                    m_Out[oo] = src[so + 2];
                    m_Out[oo + 1] = src[so + 1];
                    m_Out[oo + 2] = src[so];
                }
            }
            if (m_PostMat != null && m_Real.exposure_target > 0f)
            {
                // 自動露出の模擬: 平均輝度が目標に近づくよう一次遅れでゲインを動かす (暗幕付近で全体が持ち上がる)
                long sum = 0; int n = outH * w * 3;
                for (int i = 0; i < n; i += 9) sum += m_Out[i];
                float mean = sum / (float)(n / 9);
                float want = m_Exposure * m_Real.exposure_target / Mathf.Max(8f, mean);
                float a = Mathf.Clamp01((1f / Mathf.Max(1f, c.rate_hz)) / Mathf.Max(0.05f, m_Real.exposure_tau_s));
                m_Exposure = Mathf.Clamp(m_Exposure + (want - m_Exposure) * a, 0.4f, 3.0f);
            }
            var msg = new ImageMsg(new HeaderMsg(stamp, "camera_link"), (uint)outH, (uint)w, "bgr8", 0, (uint)(w * 3), m_Out);
            m_Ros.Publish(kImageTopic, msg);
            m_Published++;
        }

        // ------------------------------------------------------------------
        void OnGUI()
        {
            if (m_Rt == null) return;
            if (m_Layout == Layout.Aic && !m_FullSensorView)
            {
                // AI チャレンジ表示: HUD はレイアウト側が全部描く (接続待ちのときだけ案内を出す)
                m_Aic.OnGUI();
                if (m_State == null)
                    GUI.Label(new Rect(12, Screen.height - 28, 600, 22), "waiting /sim/render_state   [L] layout  [C] sensor");
                return;
            }
            if (m_RvizMode && !m_FullSensorView)
            {
                var cams = new List<(Texture, string)> { (SensorOutput, m_RivalState != null ? $"{m_OwnLabel}  /camera/image_raw" : "CameraView  /camera/image_raw") };
                if (m_RivalState != null) cams.Add((m_RivalRt, $"{m_RivalLabel}  camera (view only)"));
                m_Rviz.OnGUI(cams, Mathf.Clamp(m_Course.Data.camera.crop_top_frac, 0f, 0.6f));
            }
            else if (m_FullSensorView)
            {
                GUI.DrawTexture(new Rect(0, 0, Screen.width, Screen.height), SensorOutput, ScaleMode.ScaleToFit, false);
            }
            else if (m_Layout == Layout.Chase)
            {
                var so = SensorOutput;
                float s = Mathf.Max(1f, Mathf.Floor(Screen.height * 0.45f / so.height));
                var r = new Rect(10, Screen.height - so.height * s - 10, so.width * s, so.height * s);
                GUI.DrawTexture(r, so, ScaleMode.StretchToFill, false);
            }
            string state = m_State == null ? "waiting /sim/render_state" : $"sim t={S(F.SimTime):F1}s";
            GUI.Label(new Rect(Screen.width - 520, 10, 510, 22),
                      $"{state}   /camera/image_raw {m_PubRate:F1} Hz   [L] layout  [C] sensor  [V] chase");
            if (m_State != null && m_RivalState != null)
            {
                // レース表示: 走行距離で順位と差
                double own = S(F.Distance), riv = m_RivalState[(int)F.Distance];
                string lead = own >= riv ? m_OwnLabel : m_RivalLabel;
                var style = new GUIStyle(GUI.skin.label) { fontSize = 22, fontStyle = FontStyle.Bold };
                style.normal.textColor = Color.white;
                GUI.Box(new Rect(Screen.width - 430, 40, 420, 104), GUIContent.none);
                GUI.Label(new Rect(Screen.width - 420, 44, 410, 30), $"{m_OwnLabel}   {own:F1} m   {S(F.V):F2} m/s", style);
                GUI.Label(new Rect(Screen.width - 420, 76, 410, 30), $"{m_RivalLabel}   {riv:F1} m   {m_RivalState[(int)F.V]:F2} m/s", style);
                GUI.Label(new Rect(Screen.width - 420, 108, 410, 30), $"LEAD: {lead}  (+{System.Math.Abs(own - riv):F1} m)", style);
            }
        }

        void OnDestroy()
        {
            if (m_Rt != null) m_Rt.Release();
            if (m_PostRt != null) m_PostRt.Release();
            if (m_RivalRt != null) m_RivalRt.Release();
        }
    }
}
