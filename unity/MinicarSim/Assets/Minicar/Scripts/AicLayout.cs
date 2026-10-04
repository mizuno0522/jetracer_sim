// 自動運転 AI チャレンジのシミュレータ画面と同じ表示仕様のレイアウト。
//   画面   = 走っている台数で縦に分割 (2 台 = 左 P1 / 右 P2、3 台 = P1 / P2 / P3。2026-09-28 に 3 台対応)。各車とも上 = 追従視点 (車が見える)、
//            下 = カメラ映像 (センサカメラと同じ取付・画角)。[V] で 2 段 / 追従のみ / カメラのみ
//   左上/右上/左上の下 = P1 / P2 / P3 パネル (Auto・Drive バッジ、順位、km/h、Lap n / 秒  Sec k)
//   上中央 = 円形ミニマップ (TOP = 北が上の固定表示、白いコース線、車は色つきの丸)
//   右上隅 = FPS
//   スタート = "Waiting for start flag..." → 5,4,3,2,1 → GO (/sim/countdown。race_start.py が送る)
//   ゴール   = 全車が -laps 周 (既定 3 = 予選) を終えたら WINNER の結果表
//              (順位・車名・FIN・合計タイム、各周のラップタイムとベスト)
// 周回・ラップタイム・セクタは course.json の中心線上の進捗から Unity 側で数える
// (各車が自分のスタート位置から数える = race_manager と同じ考え方。時間は sim 時間)。
// /lap_count (lap_counter) が来ていれば周回の区切りとゴール判定はそちらに合わせる。
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public class AicLayout
    {
        const int kSectors = 8;                  // 1 周を進捗で 8 等分して Sec 1〜8
        const float kRefHeight = 768f;           // 参考動画の画面高さ。HUD の寸法はこの高さ基準
        static readonly Color kP1 = new Color32(77, 163, 255, 255);
        static readonly Color kP2 = new Color32(255, 210, 30, 255);
        static readonly Color kP3 = new Color32(60, 200, 90, 255);
        // 何台まで扱うか (決勝は 3 台。2026-09-28 に 2 → 3)
        public const int kMaxCars = 3;
        static readonly Color[] kCols = { kP1, kP2, kP3 };
        static readonly string[] kRankNames = { "1st", "2nd", "3rd" };
        static readonly Color kPanelBg = new Color(0.16f, 0.24f, 0.36f, 0.60f);
        static readonly Color kSky = new Color32(140, 178, 218, 255);

        // 中心線上の進捗から周回・ラップタイム・セクタを数える
        public class Tracker
        {
            readonly Vector2[] m_C;
            readonly float[] m_Cum;
            public readonly float Total;
            int m_Idx = -1;
            double m_PrevS, m_LastDistance = -1, m_LapStart, m_RaceStart, m_SimTime;
            int m_OfficialLaps = -1;             // /lap_count (完了した周回数)。未受信は -1
            public double StartS, Travelled, LapTime, FinishTime;
            public bool Started, Finished;
            public int TargetLaps;               // -laps。0 ならゴール判定なし
            public float SpeedKmh, Yaw;
            public Vector2 Pos;
            public readonly List<double> LapTimes = new List<double>();   // 終えた周のラップタイム

            public Tracker(Vector2[] center)
            {
                m_C = center;
                m_Cum = new float[center.Length];
                for (int i = 1; i < center.Length; i++)
                    m_Cum[i] = m_Cum[i - 1] + Vector2.Distance(center[i - 1], center[i]);
                Total = Mathf.Max(0.1f, m_Cum[center.Length - 1]);
            }

            int Completed => m_OfficialLaps >= 0 ? m_OfficialLaps : (int)System.Math.Floor(Travelled / Total);
            public int Lap => Finished ? TargetLaps : Completed + 1;
            public int Sector => Finished ? kSectors : Mathf.Clamp((int)(Frac * kSectors) + 1, 1, kSectors);
            double Frac => Travelled / Total - System.Math.Floor(Travelled / Total);

            // スタート合図 (GO)。合図が来ない起動方法では、動き出した時刻をスタートにする
            public void MarkStart()
            {
                if (Started) return;
                Started = true;
                LapTimes.Clear();
                m_RaceStart = m_LapStart = m_SimTime;
            }

            public void SetLapCount(int laps)
            {
                if (Started && !Finished && m_OfficialLaps >= 0 && laps > m_OfficialLaps)
                {
                    // lap_counter の区切りに合わせる (累積ヨーで数えるので線より少し手前で周回が付く)
                    double snap = (double)laps * Total;
                    if (System.Math.Abs(Travelled - snap) < Total * 0.5) Travelled = snap;
                    m_OfficialLaps = laps;
                    CompleteLap();
                    return;
                }
                m_OfficialLaps = laps;
            }

            // 1 周終えた: ラップタイムを記録し、規定周回ならゴール (表示は最終周のタイムで止める)
            void CompleteLap()
            {
                LapTime = m_SimTime - m_LapStart;
                LapTimes.Add(LapTime);
                if (TargetLaps > 0 && Completed >= TargetLaps)
                {
                    Finished = true;
                    FinishTime = m_SimTime - m_RaceStart;
                    return;
                }
                m_LapStart = m_SimTime;
            }

            public void Update(double x, double y, double yaw, double v, double simTime, double distance)
            {
                // vehicle_sim はレース開始 (/odom/reset) で sim 時間を 0 に戻す。GO の直後に来るので、
                // 巻き戻ったらそこをスタート時刻にする (= race_manager の経過時間と同じ起点)
                if (simTime < m_SimTime - 0.5 && Started && !Finished) m_RaceStart = m_LapStart = simTime;
                m_SimTime = simTime;
                Yaw = (float)yaw;
                Pos = new Vector2((float)x, (float)y);
                SpeedKmh = Mathf.Abs((float)v) * 3.6f;
                // 初回、または /odom/reset (レース開始) で走行距離が巻き戻ったら数え直す
                bool reset = m_Idx < 0 || distance < m_LastDistance - 0.5;
                m_LastDistance = distance;
                double s = Progress(Pos, reset);
                if (reset)
                {
                    StartS = m_PrevS = s;
                    Travelled = 0; LapTime = 0; Started = false; Finished = false;
                    LapTimes.Clear();
                }
                if (Finished) return;
                if (!Started)
                {
                    if (System.Math.Abs(v) < 0.05) { m_PrevS = s; return; }
                    MarkStart();
                }
                double ds = s - m_PrevS;
                if (ds > Total * 0.5) ds -= Total;
                if (ds < -Total * 0.5) ds += Total;
                m_PrevS = s;
                int lapBefore = Lap;
                Travelled = System.Math.Max(0, Travelled + ds);
                if (m_OfficialLaps < 0 && Lap > lapBefore)
                {
                    CompleteLap();
                    if (Finished) return;
                }
                LapTime = simTime - m_LapStart;
            }

            // 最寄りの中心線上の道のり。隣のレーンへ飛ばないよう、前回の近傍だけを探す
            double Progress(Vector2 p, bool global)
            {
                int n = m_C.Length - 1;          // 区間数 (末尾 = 先頭の閉ループ)
                int from = global ? 0 : m_Idx - 40, to = global ? n - 1 : m_Idx + 40;
                float best = float.MaxValue, bestT = 0f;
                int bestI = 0;
                for (int k = from; k <= to; k++)
                {
                    int i = ((k % n) + n) % n;
                    Vector2 a = m_C[i], ab = m_C[i + 1] - a;
                    float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / Mathf.Max(1e-9f, ab.sqrMagnitude));
                    float d = (a + ab * t - p).sqrMagnitude;
                    if (d < best) { best = d; bestI = i; bestT = t; }
                }
                m_Idx = bestI;
                return m_Cum[bestI] + (m_Cum[bestI + 1] - m_Cum[bestI]) * bestT;
            }
        }

        readonly Camera[] m_Chase = new Camera[kMaxCars], m_Onboard = new Camera[kMaxCars];
        readonly Tracker[] m_Track = new Tracker[kMaxCars];
        readonly bool[] m_Alive = new bool[kMaxCars];
        readonly Vector2[] m_Center;
        Vector2 m_MapMid;
        float m_MapHalf;                          // ミニマップに収める半径 (m)
        readonly CourseData m_Data;
        readonly string[] m_Labels;
        Texture2D m_White, m_MapTex, m_DotTex, m_ArrowTex;
        GUIStyle m_Name, m_Badge, m_Rank, m_Speed, m_LapStyle, m_Fps, m_Top;
        GUIStyle m_LapLine, m_Big, m_Wait, m_Winner, m_WinName, m_WinTime, m_Head, m_Row, m_RowBold, m_Foot;
        int m_Countdown = -1;                     // /sim/countdown の最新値 (5..1、0 = GO)。-1 = まだ
        float m_GoUntil;
        float m_StyleScale;
        float m_FpsShown, m_FpsT0;
        int m_FpsFrames;
        bool m_Enabled;

        public AicLayout(CourseData data, bool longRoute, int laps, string[] labels, Transform[] cars, int[] selfLayers)
        {
            m_Data = data;
            m_Labels = labels;
            float[] c = longRoute ? data.centerline_long : data.centerline_shortcut;
            if (c == null || c.Length < 6) c = new float[] { 0, 0, 1, 0, 1, 1, 0, 0 };
            m_Center = new Vector2[c.Length / 2];
            Vector2 lo = new Vector2(float.MaxValue, float.MaxValue), hi = -lo;
            for (int i = 0; i < m_Center.Length; i++)
            {
                m_Center[i] = new Vector2(c[i * 2], c[i * 2 + 1]);
                lo = Vector2.Min(lo, m_Center[i]); hi = Vector2.Max(hi, m_Center[i]);
            }
            // ミニマップには壁も描くので、壁まで含めて円に収める
            foreach (var w in data.walls ?? new WallData[0])
            {
                lo = Vector2.Min(lo, Vector2.Min(new Vector2(w.x0, w.y0), new Vector2(w.x1, w.y1)));
                hi = Vector2.Max(hi, Vector2.Max(new Vector2(w.x0, w.y0), new Vector2(w.x1, w.y1)));
            }
            m_MapMid = (lo + hi) * 0.5f;
            m_MapHalf = (hi - m_MapMid).magnitude / 0.97f;   // 四隅が円の縁のすぐ内側
            for (int i = 0; i < kMaxCars; i++) m_Track[i] = new Tracker(m_Center) { TargetLaps = laps };

            for (int i = 0; i < kMaxCars; i++)
            {
                m_Chase[i] = MakeChaseCamera($"AicChaseP{i + 1}", cars[i]);
                m_Onboard[i] = MakeOnboardCamera($"AicOnboardP{i + 1}", cars[i], data.camera, selfLayers[i]);
                if (data.IsCircuit)
                {
                    // 実車スケール: 遠くまで描く。車体 (Root) は拡大してあるので追従視点の距離は自動で伸びる
                    foreach (var cam in new[] { m_Chase[i], m_Onboard[i] })
                    {
                        cam.nearClipPlane = 0.2f;
                        cam.farClipPlane = 12000f;
                        cam.clearFlags = CameraClearFlags.Skybox;
                    }
                }
            }
            m_FpsT0 = Time.unscaledTime;
        }

        // 追従視点: 車の後ろ上から前方 (何が走っているかが分かる)。車体 (Root) に固定で、Root は
        // SimBridge が時刻ベースで滑らかに動かすので、視点と車の動きがずれない。
        // ロール・ピッチは Body 側だけに掛かるので、視点は揺れない
        static Camera MakeChaseCamera(string name, Transform car)
        {
            var cam = MakeCamera(name, car, ~(1 << RvizLayout.OverviewOnlyLayer));
            cam.transform.localPosition = new Vector3(0f, 0.48f, -0.60f);
            cam.transform.localRotation = Quaternion.LookRotation(new Vector3(0f, -0.38f, 1.60f));
            cam.fieldOfView = 58f;
            return cam;
        }

        // カメラ映像: センサカメラ (/camera/image_raw) と同じ取付位置・ピッチ・画角を、画面の解像度で描く。
        // センサと同じく自車は写さない (selfLayer)
        static Camera MakeOnboardCamera(string name, Transform car, CameraData c, int selfLayer)
        {
            var cam = MakeCamera(name, car, ~((1 << RvizLayout.OverviewOnlyLayer) | (1 << selfLayer)));
            // 取付高さは世界の長さ [m]。車体 (親) を拡大していても同じ高さになるよう縮尺で割る
            cam.transform.localPosition = new Vector3(0f, c.mount_height_m / Mathf.Max(1e-3f, car.lossyScale.y), 0f);
            cam.transform.localRotation = Quaternion.Euler(c.pitch_deg, 0f, 0f);
            float f = (c.width * 0.5f) / Mathf.Tan(c.fov_deg * 0.5f * Mathf.Deg2Rad);
            cam.fieldOfView = 2f * Mathf.Atan((c.height * 0.5f) / f) * Mathf.Rad2Deg;
            return cam;
        }

        static Camera MakeCamera(string name, Transform car, int cullingMask)
        {
            var cam = new GameObject(name).AddComponent<Camera>();
            cam.transform.SetParent(car, false);
            cam.nearClipPlane = 0.02f;
            cam.farClipPlane = 60f;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = kSky;
            cam.cullingMask = cullingMask;
            cam.depth = 2;
            cam.enabled = false;
            return cam;
        }

        // 画面の構成。[V] で切替: 上 = 追従視点・下 = カメラ映像 (既定) → 追従視点のみ → カメラ映像のみ
        public enum View { Both, Chase, Onboard }
        /// P1 の下段に出す実際のセンサ画像 (/camera/image_raw と同じもの)。null なら表示用カメラで描く
        public Texture SensorTexture;
        public View ViewMode = View.Both;

        public bool Enabled
        {
            get => m_Enabled;
            set { m_Enabled = value; ApplyRects(); }
        }

        // index 0 = P1 (自車)、1 = P2・2 = P3 (レースの相手)
        public void SetState(int index, double x, double y, double yaw, double v, double simTime, double distance)
        {
            m_Alive[index] = true;
            m_Track[index].Update(x, y, yaw, v, simTime, distance);
        }

        public void SetLapCount(int index, int laps) => m_Track[index].SetLapCount(laps);

        // /sim/countdown: 5..1 = カウントダウン、0 = GO (両車のスタート時刻にする)
        public void SetCountdown(int n)
        {
            if (n == 0)
            {
                if (m_Countdown == 0) return;     // GO は取りこぼし対策で 2 回来る
                m_GoUntil = Time.unscaledTime + 1.2f;
                foreach (var t in m_Track) t.MarkStart();
            }
            m_Countdown = n;
        }

        // 走っている全車がゴールしたら結果画面
        public bool ShowingResult
        {
            get
            {
                if (!m_Alive[0]) return false;
                for (int i = 0; i < kMaxCars; i++)
                    if (m_Alive[i] && !m_Track[i].Finished) return false;
                return true;
            }
        }

        // 画面に出す車 (P1 は常に。相手は状態が届いてから)。列の順 = 番号順
        List<int> Shown()
        {
            var s = new List<int> { 0 };
            for (int i = 1; i < kMaxCars; i++) if (m_Alive[i]) s.Add(i);
            return s;
        }

        // 順位順 (道のりの長い順)
        List<int> Ranked()
        {
            var s = Shown();
            s.Sort((a, b) => RaceScore(b).CompareTo(RaceScore(a)));
            return s;
        }

        public void Update()
        {
            m_FpsFrames++;
            float dt = Time.unscaledTime - m_FpsT0;
            if (dt >= 0.5f) { m_FpsShown = m_FpsFrames / dt; m_FpsFrames = 0; m_FpsT0 = Time.unscaledTime; }
            ApplyRects();
        }

        // 3 台のときは自動運転 AI チャレンジと同じ構成: 上段 = 左 P2 / 右 P3、下段 = P1 (横幅いっぱい)、
        // どれも追従視点だけ。パネルは P2 左上・P3 右上・P1 左下、ミニマップは画面の中央
        bool Three => Shown().Count == 3;

        void ApplyRects()
        {
            var shown = Shown();
            int n = shown.Count;
            if (n == 3)
            {
                var rects = new[] { new Rect(0f, 0f, 1f, 0.5f), new Rect(0f, 0.5f, 0.5f, 0.5f), new Rect(0.5f, 0.5f, 0.5f, 0.5f) };
                for (int i = 0; i < kMaxCars; i++)
                {
                    m_Chase[i].enabled = m_Enabled;
                    m_Chase[i].rect = rects[i];
                    m_Onboard[i].enabled = false;
                }
                return;
            }
            for (int i = 0; i < kMaxCars; i++)
            {
                int col = shown.IndexOf(i);
                bool on = m_Enabled && col >= 0;
                float w = 1f / n, x = Mathf.Max(0, col) * w;
                m_Chase[i].enabled = on && ViewMode != View.Onboard;
                m_Onboard[i].enabled = on && ViewMode != View.Chase && !(i == 0 && SensorTexture != null);
                m_Chase[i].rect = ViewMode == View.Both ? new Rect(x, 0.5f, w, 0.5f) : new Rect(x, 0f, w, 1f);
                m_Onboard[i].rect = ViewMode == View.Both ? new Rect(x, 0f, w, 0.5f) : new Rect(x, 0f, w, 1f);
            }
        }

        // P1 のカメラ枠に実際のセンサ画像を縦横比を保って描く (224×224 は正方形なので左右は黒)
        void DrawSensorPane(int n)
        {
            if (SensorTexture == null || ViewMode == View.Chase) return;
            float w = Screen.width / (float)n;
            float y0 = ViewMode == View.Both ? Screen.height * 0.5f : 0f;
            float h = ViewMode == View.Both ? Screen.height * 0.5f : Screen.height;
            var r = new Rect(0f, y0, w, h);
            GUI.color = Color.black;
            GUI.DrawTexture(r, m_White);
            GUI.color = Color.white;
            GUI.DrawTexture(r, SensorTexture, ScaleMode.ScaleToFit, false);
        }

        // 同じ位置から数えた道のり (順位用)。P2・P3 は P1 のスタートとの前後差を足す
        double RaceScore(int i)
        {
            double off = 0;
            if (i != 0)
            {
                float total = m_Track[0].Total;
                off = m_Track[i].StartS - m_Track[0].StartS;
                if (off > total * 0.5) off -= total;
                if (off < -total * 0.5) off += total;
            }
            return m_Track[i].Travelled + off;
        }

        // ------------------------------------------------------------------
        public void OnGUI()
        {
            if (!m_Enabled) return;
            float k = Screen.height / kRefHeight;
            BuildStyles(k);
            var shown = Shown();
            int ncol = shown.Count;
            if (ncol == 3)
            {
                OnGuiThree(k);
                return;
            }
            if (ViewMode == View.Onboard) DrawSensorPane(ncol);

            for (int c = 1; c < ncol; c++)
            {
                GUI.color = Color.black;
                GUI.DrawTexture(new Rect(Screen.width * c / (float)ncol - 1f, 0, 2f, Screen.height), m_White);
                GUI.color = Color.white;
            }
            if (ViewMode == View.Both)
            {
                GUI.color = Color.black;
                GUI.DrawTexture(new Rect(0, Screen.height * 0.5f - 1f, Screen.width, 2f), m_White);
                GUI.color = Color.white;
                DrawSensorPane(ncol);
                // 下段の見出し (カメラ映像 = センサカメラと同じ取付・画角)
                for (int i = 0; i < ncol; i++)
                {
                    float px = Screen.width * i / (float)ncol + 8f * k;
                    GUI.color = new Color(0.05f, 0.07f, 0.10f, 0.7f);
                    string label = i == 0 && SensorTexture != null ? "CAMERA  /camera/image_raw (実配信)" : "CAMERA";
                    float lw = (i == 0 && SensorTexture != null ? 230f : 66f) * k;
                    GUI.DrawTexture(new Rect(px, Screen.height * 0.5f + 6f * k, lw, 18f * k), m_White);
                    GUI.color = Color.white;
                    GUI.Label(new Rect(px, Screen.height * 0.5f + 6f * k, lw, 18f * k), label, m_Top);
                }
            }

            // パネル: P1 は左上、P2 は右上、P3 は P1 の下 (上端の中央はミニマップ)
            var ranked = Ranked();
            foreach (int i in shown)
            {
                float px = i == 1 ? Screen.width - (12f + 184f) * k : 12f * k;
                float py = i == 2 ? (12f + 124f) * k : 12f * k;
                DrawPanel(i, px, py, k, $"P{i + 1}", kCols[i], ranked.IndexOf(i) + 1);
            }
            DrawMinimap(k);

            GUI.Label(new Rect(Screen.width - 110f * k, 0, 106f * k, 18f * k), $"{m_FpsShown:0} FPS", m_Fps);

            if (ShowingResult) DrawResult(k);
            else DrawStartSignal(k);
        }

        // 3 台 (自動運転 AI チャレンジの構成)
        void OnGuiThree(float k)
        {
            GUI.color = Color.black;
            GUI.DrawTexture(new Rect(0, Screen.height * 0.5f - 1f, Screen.width, 2f), m_White);          // 上段と下段
            GUI.DrawTexture(new Rect(Screen.width * 0.5f - 1f, 0, 2f, Screen.height * 0.5f), m_White);   // 上段の左右
            GUI.color = Color.white;
            var ranked = Ranked();
            DrawPanel(1, 12f * k, 12f * k, k, "P2", kCols[1], ranked.IndexOf(1) + 1);
            DrawPanel(2, Screen.width - (12f + 184f) * k, 12f * k, k, "P3", kCols[2], ranked.IndexOf(2) + 1);
            DrawPanel(0, 12f * k, Screen.height - (12f + 118f) * k, k, "P1", kCols[0], ranked.IndexOf(0) + 1);
            DrawMinimap(k);
            GUI.Label(new Rect(Screen.width - 110f * k, 0, 106f * k, 18f * k), $"{m_FpsShown:0} FPS", m_Fps);
            if (ShowingResult) DrawResult(k);
            else DrawStartSignal(k);
        }

        // スタート前: "Waiting for start flag..." → 5..1 → GO (画面中央、白の太字に影)
        void DrawStartSignal(float k)
        {
            if (m_Countdown == 0 && Time.unscaledTime < m_GoUntil)
            {
                Shadowed(new Rect(0, Screen.height * 0.5f - 40f * k, Screen.width, 70f * k), "GO", m_Big, k);
                return;
            }
            if (m_Track[0].Started) return;       // 走行中 (合図なしで走り出した単独起動も含む)
            if (m_Countdown > 0)
                Shadowed(new Rect(0, Screen.height * 0.5f - 40f * k, Screen.width, 70f * k), m_Countdown.ToString(), m_Big, k);
            else
                Shadowed(new Rect(Screen.width * 0.5f - 195f * k, Screen.height * 0.30f, 390f * k, 110f * k), "Waiting for start flag...", m_Wait, k);
        }

        void Shadowed(Rect r, string text, GUIStyle style, float k)
        {
            var col = style.normal.textColor;
            style.normal.textColor = new Color(0f, 0f, 0f, 0.55f);
            GUI.Label(new Rect(r.x + 2f * k, r.y + 2f * k, r.width, r.height), text, style);
            style.normal.textColor = col;
            GUI.Label(r, text, style);
        }

        // ゴール: 画面を暗くして中央に結果表 (WINNER / 車名 / タイム、RANK・VEHICLE・STATUS・TIME)
        void DrawResult(float k)
        {
            GUI.color = new Color(0.10f, 0.12f, 0.25f, 0.55f);
            GUI.DrawTexture(new Rect(0, 0, Screen.width, Screen.height), m_White);
            float w = 700f * k, h = 520f * k, x = (Screen.width - w) * 0.5f, y = 125f * k;
            GUI.color = new Color(0.20f, 0.22f, 0.38f, 0.93f);
            GUI.DrawTexture(new Rect(x, y, w, h), m_White);
            GUI.color = new Color32(40, 200, 200, 255);
            GUI.DrawTexture(new Rect(x, y, w, 3f * k), m_White);
            GUI.DrawTexture(new Rect(x, y + h - 3f * k, w, 3f * k), m_White);

            var order = Shown();
            order.Sort((a, b) => m_Track[a].FinishTime.CompareTo(m_Track[b].FinishTime));
            bool two = order.Count > 1;
            int win = order[0];
            GUI.color = Color.white;
            GUI.Label(new Rect(x, y + 14f * k, w, 70f * k), two ? "WINNER" : "FINISH", m_Winner);
            GUI.Label(new Rect(x, y + 80f * k, w, 42f * k), m_Labels[win], m_WinName);
            GUI.Label(new Rect(x, y + 120f * k, w, 28f * k), $"{m_Track[win].FinishTime:0.00}s", m_WinTime);

            GUI.color = new Color(1f, 1f, 1f, 0.35f);
            GUI.DrawTexture(new Rect(x + 40f * k, y + 158f * k, w - 80f * k, 1f), m_White);
            GUI.DrawTexture(new Rect(x + 40f * k, y + h - 36f * k, w - 80f * k, 1f), m_White);
            GUI.color = Color.white;
            float hy = y + 172f * k;
            GUI.Label(new Rect(x + 30f * k, hy, 80f * k, 22f * k), "RANK", m_Head);
            GUI.Label(new Rect(x + 126f * k, hy, 200f * k, 22f * k), "VEHICLE", m_Head);
            GUI.Label(new Rect(x + 382f * k, hy, 100f * k, 22f * k), "STATUS", m_Head);
            m_Head.alignment = TextAnchor.MiddleRight;
            GUI.Label(new Rect(x + 470f * k, hy, 200f * k, 22f * k), "TIME", m_Head);
            m_Head.alignment = TextAnchor.MiddleLeft;

            for (int n = 0; n < order.Count; n++)
            {
                int i = order[n];
                float ry = y + (200f + 74f * n) * k;
                bool first = n == 0;
                Color accent = first ? (Color)new Color32(255, 240, 30, 255) : new Color32(225, 228, 235, 255);
                if (first)
                {
                    GUI.color = new Color(1f, 1f, 1f, 0.10f);
                    GUI.DrawTexture(new Rect(x + 30f * k, ry, w - 60f * k, 40f * k), m_White);
                }
                // 順位の箱 (左に色の帯)
                GUI.color = first ? new Color(0.55f, 0.52f, 0.30f, 0.9f) : new Color(0.45f, 0.46f, 0.52f, 0.9f);
                GUI.DrawTexture(new Rect(x + 38f * k, ry + 6f * k, 52f * k, 28f * k), m_White);
                GUI.color = accent;
                GUI.DrawTexture(new Rect(x + 38f * k, ry + 6f * k, 3f * k, 28f * k), m_White);
                GUI.color = kCols[i];
                GUI.DrawTexture(new Rect(x + 110f * k, ry + 10f * k, 10f * k, 20f * k), m_White);
                GUI.color = Color.white;
                var st = first ? m_RowBold : m_Row;
                st.alignment = TextAnchor.MiddleCenter;
                st.normal.textColor = accent;
                GUI.Label(new Rect(x + 41f * k, ry + 6f * k, 49f * k, 28f * k), kRankNames[n], st);
                st.alignment = TextAnchor.MiddleLeft;
                st.normal.textColor = first ? Color.white : new Color(0.85f, 0.86f, 0.9f);
                GUI.Label(new Rect(x + 126f * k, ry, 250f * k, 40f * k), m_Labels[i], st);
                m_Head.normal.textColor = new Color32(120, 255, 140, 255);
                GUI.Label(new Rect(x + 382f * k, ry, 100f * k, 40f * k), "FIN", m_Head);
                m_Head.normal.textColor = new Color(0.86f, 0.88f, 0.93f);
                m_RowBold.alignment = TextAnchor.MiddleRight;
                m_RowBold.normal.textColor = first ? accent : Color.white;
                GUI.Label(new Rect(x + 470f * k, ry, 200f * k, 40f * k), $"{m_Track[i].FinishTime:0.00}s", m_RowBold);
                m_RowBold.alignment = TextAnchor.MiddleLeft;

                // 各周のラップタイムとベスト (予選 = 3 周の合計タイム勝負の内訳)
                var laps = m_Track[i].LapTimes;
                if (laps.Count > 0)
                {
                    double best = double.MaxValue;
                    var sb = new System.Text.StringBuilder();
                    for (int l = 0; l < laps.Count; l++)
                    {
                        best = System.Math.Min(best, laps[l]);
                        sb.Append($"L{l + 1} {laps[l]:0.00}s    ");
                    }
                    sb.Append($"BEST {best:0.00}s");
                    GUI.Label(new Rect(x + 126f * k, ry + 40f * k, w - 160f * k, 24f * k), sb.ToString(), m_LapLine);
                }
            }
            GUI.Label(new Rect(x, y + h - 34f * k, w, 28f * k), "Esc: Quit", m_Foot);
        }

        void DrawPanel(int i, float x, float y, float k, string name, Color col, int rank)
        {
            var tr = m_Track[i];
            GUI.color = kPanelBg;
            GUI.DrawTexture(new Rect(x, y, 184f * k, 118f * k), m_White);

            // 1 段目: P1 / Auto / Drive
            GUI.color = new Color(0.05f, 0.07f, 0.10f, 0.85f);
            GUI.DrawTexture(new Rect(x + 4f * k, y + 5f * k, 176f * k, 32f * k), m_White);
            GUI.color = Color.white;
            m_Name.normal.textColor = col;
            GUI.Label(new Rect(x + 6f * k, y + 4f * k, 40f * k, 34f * k), name, m_Name);
            // Auto = 自動運転中 (状態を受信中は緑)。Drive = 走行中は明るく
            Badge(new Rect(x + 46f * k, y + 7f * k, 36f * k, 28f * k), "A", "uto",
                  m_Alive[i] ? new Color(0.02f, 0.47f, 0.24f) : new Color(0.2f, 0.2f, 0.2f), m_Alive[i], k);
            Badge(new Rect(x + 84f * k, y + 7f * k, 40f * k, 28f * k), "D", "rive",
                  tr.SpeedKmh > 0.2f ? new Color(0.10f, 0.13f, 0.17f) : new Color(0.02f, 0.02f, 0.03f), false, k);

            // 2 段目: 順位 (1st は金、2nd は銀)
            m_Rank.normal.textColor = rank == 1 ? new Color32(255, 214, 40, 255) : new Color32(208, 214, 224, 255);
            GUI.Label(new Rect(x + 6f * k, y + 36f * k, 170f * k, 34f * k), kRankNames[Mathf.Clamp(rank - 1, 0, kMaxCars - 1)], m_Rank);

            // 3 段目: 速度、4 段目: 周回 / ラップタイム / セクタ
            GUI.Label(new Rect(x + 6f * k, y + 70f * k, 176f * k, 22f * k), $"{tr.SpeedKmh:0.0} km/h", m_Speed);
            string of = tr.TargetLaps > 0 ? $"/{tr.TargetLaps}" : "";      // 予選は 3 周: Lap 2/3
            string lap = tr.Started ? $"Lap {tr.Lap}{of} / {tr.LapTime:0.00}s  Sec {tr.Sector}" : "Lap -- / --.--s  Sec --";
            GUI.Label(new Rect(x + 6f * k, y + 92f * k, 178f * k, 22f * k), lap, m_LapStyle);
        }

        void Badge(Rect r, string head, string tail, Color bg, bool lamp, float k)
        {
            GUI.color = bg;
            GUI.DrawTexture(r, m_White);
            GUI.color = Color.white;
            int big = Mathf.Max(8, Mathf.RoundToInt(17f * k)), small = Mathf.Max(6, Mathf.RoundToInt(9f * k));
            GUI.Label(r, $"<size={big}>{head}</size><size={small}>{tail}</size>", m_Badge);
            if (lamp)
            {
                GUI.color = new Color(0.45f, 1f, 0.55f);
                GUI.DrawTexture(new Rect(r.xMax - 10f * k, r.y + 3f * k, 7f * k, 7f * k), m_DotTex);
                GUI.color = Color.white;
            }
        }

        void DrawMinimap(float k)
        {
            float size = 256f * k, cx = Screen.width * 0.5f;
            // 3 台のときは画面の中央 (上段と下段の境目)。見出し TOP は上端の中央のまま
            float top = Three ? Screen.height * 0.5f - size * 0.5f : 6f * k;
            var r = new Rect(cx - size * 0.5f, top, size, size);
            GUI.color = new Color(0.10f, 0.13f, 0.18f, 0.85f);
            GUI.DrawTexture(new Rect(cx - 60f * k, 0, 120f * k, 22f * k), m_White);
            GUI.color = Color.white;
            GUI.DrawTexture(r, m_MapTex);
            GUI.Label(new Rect(cx - 60f * k, 0, 120f * k, 22f * k), "TOP", m_Top);

            // 後ろの車から描く (先頭の車の丸が上に重なる)
            var ranked = Ranked();
            for (int n = ranked.Count - 1; n >= 0; n--)
            {
                int i = ranked[n];
                if (!m_Alive[i]) continue;
                Vector2 uv = MapUv(m_Track[i].Pos);
                float d = 20f * k;
                var c = new Vector2(r.x + uv.x * size, r.y + (1f - uv.y) * size);
                // 進行方向の矢じり (テクスチャは上向き = 北 = yaw 90°。GUI の回転は時計回り)
                var saved = GUI.matrix;
                GUIUtility.RotateAroundPivot(90f - m_Track[i].Yaw * Mathf.Rad2Deg, c);
                GUI.color = Color.white;
                GUI.DrawTexture(new Rect(c.x - d * 0.48f, c.y - d * 1.45f, d * 0.96f, d * 1.0f), m_ArrowTex);
                GUI.matrix = saved;
                GUI.DrawTexture(new Rect(c.x - d * 0.5f - 2f * k, c.y - d * 0.5f - 2f * k, d + 4f * k, d + 4f * k), m_DotTex);
                GUI.color = kCols[i];
                GUI.DrawTexture(new Rect(c.x - d * 0.5f, c.y - d * 0.5f, d, d), m_DotTex);
            }
            GUI.color = Color.white;
        }

        // ROS 座標 → ミニマップ内の 0〜1 (u = 右が +x、v = 上が +y = 北)
        Vector2 MapUv(Vector2 p) => (p - m_MapMid) / (2f * m_MapHalf) + new Vector2(0.5f, 0.5f);

        // ------------------------------------------------------------------
        void BuildStyles(float k)
        {
            if (m_White == null)
            {
                m_White = new Texture2D(1, 1); m_White.SetPixel(0, 0, Color.white); m_White.Apply();
                m_DotTex = MakeDisk(64, Color.white, Color.white, 0f);
                m_ArrowTex = MakeArrow(64);
                m_MapTex = MakeMap(512);
            }
            if (m_Name != null && Mathf.Approximately(m_StyleScale, k)) return;
            m_StyleScale = k;
            GUIStyle S(float size, TextAnchor a)
            {
                var s = new GUIStyle(GUI.skin.label)
                {
                    fontSize = Mathf.Max(8, Mathf.RoundToInt(size * k)), fontStyle = FontStyle.Bold,
                    alignment = a, richText = true, wordWrap = false, clipping = TextClipping.Overflow,
                    padding = new RectOffset(0, 0, 0, 0),
                };
                s.normal.textColor = Color.white;
                return s;
            }
            m_Name = S(25f, TextAnchor.MiddleLeft);
            m_Badge = S(12f, TextAnchor.MiddleCenter);
            m_Rank = S(27f, TextAnchor.MiddleLeft);
            m_Speed = S(16f, TextAnchor.MiddleLeft);
            m_LapStyle = S(14f, TextAnchor.MiddleLeft);
            m_Fps = S(13f, TextAnchor.UpperRight);
            m_Top = S(11f, TextAnchor.MiddleCenter);
            m_Big = S(52f, TextAnchor.MiddleCenter);
            m_Wait = S(38f, TextAnchor.MiddleCenter);
            m_Wait.wordWrap = true;
            m_Wait.clipping = TextClipping.Overflow;
            m_Winner = S(58f, TextAnchor.MiddleCenter);
            m_Winner.normal.textColor = new Color32(255, 240, 30, 255);
            m_WinName = S(32f, TextAnchor.MiddleCenter);
            m_WinTime = S(19f, TextAnchor.MiddleCenter);
            m_WinTime.fontStyle = FontStyle.Normal;
            m_WinTime.normal.textColor = new Color32(40, 235, 225, 255);
            m_Head = S(12f, TextAnchor.MiddleLeft);
            m_Head.normal.textColor = new Color(0.86f, 0.88f, 0.93f);
            m_Row = S(18f, TextAnchor.MiddleLeft);
            m_Row.fontStyle = FontStyle.Normal;
            m_RowBold = S(18f, TextAnchor.MiddleLeft);
            m_LapLine = S(13f, TextAnchor.MiddleLeft);
            m_LapLine.fontStyle = FontStyle.Normal;
            m_LapLine.normal.textColor = new Color(0.72f, 0.80f, 0.92f);
            m_Foot = S(14f, TextAnchor.MiddleCenter);
            m_Foot.fontStyle = FontStyle.Normal;
            m_Foot.normal.textColor = new Color(0.88f, 0.9f, 0.94f);
        }

        static Texture2D MakeDisk(int n, Color fill, Color edge, float edgePx)
        {
            var tex = new Texture2D(n, n, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
            var px = new Color[n * n];
            float r = n * 0.5f - 1f;
            for (int y = 0; y < n; y++)
                for (int x = 0; x < n; x++)
                {
                    float d = Mathf.Sqrt((x + 0.5f - n * 0.5f) * (x + 0.5f - n * 0.5f) + (y + 0.5f - n * 0.5f) * (y + 0.5f - n * 0.5f));
                    Color c = d > r - edgePx ? edge : fill;
                    c.a *= Mathf.Clamp01(r - d + 0.5f);      // 縁を 1px でぼかす
                    px[y * n + x] = c;
                }
            tex.SetPixels(px);
            tex.Apply();
            return tex;
        }

        static Texture2D MakeArrow(int n)
        {
            var tex = new Texture2D(n, n, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
            var px = new Color[n * n];
            for (int y = 0; y < n; y++)
                for (int x = 0; x < n; x++)
                {
                    // 上向きの三角 (行 0 = 下)。高さ y での半幅は上へ行くほど狭い
                    float half = (1f - (y + 0.5f) / n) * n * 0.42f;
                    float a = Mathf.Clamp01(half - Mathf.Abs(x + 0.5f - n * 0.5f) + 0.5f);
                    px[y * n + x] = new Color(1f, 1f, 1f, a);
                }
            tex.SetPixels(px);
            tex.Apply();
            return tex;
        }

        // ミニマップの下絵 (行 0 = 下 = 南): 半透明の紺の円盤と縁の輪、走行レーンの帯、
        // コースの壁 (白 / 赤)、白い中心線、スタートライン
        Texture2D MakeMap(int n)
        {
            var px = new Color[n * n];
            var mask = new float[n * n];
            float R = n * 0.5f - 1f;
            for (int y = 0; y < n; y++)
                for (int x = 0; x < n; x++)
                {
                    float d = new Vector2(x + 0.5f - n * 0.5f, y + 0.5f - n * 0.5f).magnitude;
                    float t = Mathf.Clamp01(d / R);
                    // 中心がやや明るく、縁へ向けて濃くなる
                    Color c = Color.Lerp(new Color(0.16f, 0.22f, 0.38f, 0.62f), new Color(0.07f, 0.10f, 0.20f, 0.74f), t * t);
                    float ring = Mathf.Clamp01(1f - Mathf.Abs(d - (R - 3f)) / 2.5f);
                    c = Color.Lerp(c, new Color(0.75f, 0.85f, 1f, 0.85f), ring * 0.8f);
                    c.a *= Mathf.Clamp01(R - d + 0.5f);
                    px[y * n + x] = c;
                }

            float pxPerM = n / (2f * m_MapHalf);
            void Line(Vector2 a, Vector2 b, float r)
            {
                a = MapUv(a) * n; b = MapUv(b) * n;
                int steps = Mathf.Max(1, Mathf.CeilToInt(Vector2.Distance(a, b) / Mathf.Max(0.5f, r * 0.5f)));
                for (int q = 0; q <= steps; q++)
                {
                    Vector2 p = Vector2.Lerp(a, b, (float)q / steps);
                    for (int y = Mathf.Max(0, Mathf.FloorToInt(p.y - r - 1f)); y <= Mathf.Min(n - 1, Mathf.CeilToInt(p.y + r + 1f)); y++)
                        for (int x = Mathf.Max(0, Mathf.FloorToInt(p.x - r - 1f)); x <= Mathf.Min(n - 1, Mathf.CeilToInt(p.x + r + 1f)); x++)
                        {
                            float cover = Mathf.Clamp01(r - Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f), p) + 0.5f);
                            if (cover > mask[y * n + x]) mask[y * n + x] = cover;
                        }
                }
            }
            // mask に描いた線を 1 色で重ねる (半透明の色が重なって濃くならないよう、層ごとに 1 回だけ合成)
            void Flush(Color col)
            {
                for (int i = 0; i < px.Length; i++)
                {
                    float m = mask[i] * col.a;
                    mask[i] = 0f;
                    if (m <= 0f || px[i].a <= 0f) continue;
                    Color c = Color.Lerp(px[i], col, m);
                    c.a = px[i].a + (1f - px[i].a) * m;
                    px[i] = c;
                }
            }

            // 走行レーン (ミニカーは幅 0.6 m、サーキットはコース幅) の帯
            float laneHalf = m_Data.IsCircuit && m_Data.circuit != null && m_Data.circuit.width_m > 0f ? m_Data.circuit.width_m * 0.5f : 0.30f;
            for (int s = 0; s < m_Center.Length - 1; s++) Line(m_Center[s], m_Center[s + 1], Mathf.Max(1.2f, laneHalf * pxPerM));
            Flush(new Color(0.42f, 0.50f, 0.66f, 0.55f));
            // 壁: 白と赤 (⑤狭い道の中央仕切りも含む。予選でも設置される)
            foreach (string color in new[] { "white", "red" })
            {
                foreach (var w in m_Data.walls ?? new WallData[0])
                    if ((w.color == "red") == (color == "red"))
                        Line(new Vector2(w.x0, w.y0), new Vector2(w.x1, w.y1), 1.3f);
                Flush(color == "red" ? new Color(1f, 0.42f, 0.42f, 0.95f) : new Color(0.88f, 0.92f, 1f, 0.9f));
            }
            // 中心線・参照線は描かない (✎ 2026-09-29、ユーザー: 決勝はコースが分岐し追い抜きもあるので、
            // 1 本の線は実際の走りと合わない)。周回・セクタの判定は描画と無関係に中心線で行う
            // スタートライン (中心線の始点を横切る)
            Vector2 dir = (m_Center[1] - m_Center[0]).normalized, nrm = new Vector2(-dir.y, dir.x);
            Line(m_Center[0] - nrm * laneHalf * 1.6f, m_Center[0] + nrm * laneHalf * 1.6f, 2.6f);
            Flush(new Color(1f, 0.92f, 0.25f, 1f));

            var tex = new Texture2D(n, n, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
            tex.SetPixels(px);
            tex.Apply();
            return tex;
        }
    }
}
