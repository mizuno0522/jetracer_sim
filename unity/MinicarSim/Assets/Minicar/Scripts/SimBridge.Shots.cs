// 撮影・計測モード (見た目の確認と描画の重さの記録用)。ROS が無くても動く。
//
//   -shots "0,1250,2600,3300"   コントロールラインからの距離 s [m] の位置に自車を置いて撮る (コースの長さで折り返す)
//   -shotdir <dir>               保存先 (既定 ./shots)
//   -shotsize 1920x1080          追従視点・俯瞰の解像度 (車載カメラは配信している画像そのままの大きさ)
//   -shotviews grandstand,panasonic,scenic,carfront,carside,carrear   名所と車の確認用の視点を追加で撮る (<番号>_view_<名前>.png)
//   -bench 600                   撮影のあと、車をコースに沿って動かしながら N フレームの描画時間を計る (0 = 計らない)
//
// 1 つの位置につき 3 枚: <番号>_s<距離>_chase.png (追従視点)・_onboard.png (配信しているセンサ画像 = 後処理後)・
// _overview.png (俯瞰)。ライバル 2 台を前 (車長の 7 倍) と斜め前に置く。速度 0 で止めて撮る。
// 計測結果は <dir>/bench.md (GPU・Unity の版・コースの組み立て時間・平均 fps・最悪フレーム・センサ配信レート)。
// 全部終わったらアプリを終了する。並べた一覧は tools/shot_sheet.py で作る。
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace Minicar
{
    public partial class SimBridge
    {
        bool m_ShotMode;
        float m_BuildSeconds;
        float[] m_PathS;          // 中心線の弧長 (n + 1 個。最後 = 1 周)
        Vector2[] m_PathP;        // 中心線 (閉ループ)

        void StartShots()
        {
            m_ShotMode = true;
            if (m_ShotDir == "") m_ShotDir = "shots";
            Directory.CreateDirectory(m_ShotDir);
            StartCoroutine(RunShots());
        }

        IEnumerator RunShots()
        {
            yield return null;                  // Start が全部終わってから
            var inv = CultureInfo.InvariantCulture;
            BuildPath();
            var list = new List<float>();
            foreach (var t in Arg("-shots", "0").Split(','))
                if (float.TryParse(t.Trim(), NumberStyles.Float, inv, out float v)) list.Add(v);
            ParseSize(Arg("-shotsize", "1920x1080"), out int w, out int h);
            float len = CarLength();
            Debug.Log($"[Shots] {list.Count} positions, lap {m_PathS[m_PathS.Length - 1]:F1} m, car {len:F2} m → {m_ShotDir}");

            int idx = 0;
            foreach (float s in list)
            {
                SetCars(s, 0.0);
                m_Follow = 0;
                m_ChaseInit = false;
                // テクスチャの読み込み・影・センサ画像 (15〜30 Hz) と自動露出が落ち着くまで待つ
                for (int k = 0; k < 45; k++) { SetCars(s, 0.0); yield return null; }
                yield return new WaitForEndOfFrame();
                string tag = $"{idx:00}_s{Mathf.RoundToInt(s):00000}";
                Save(Grab(m_ViewCam, w, h), $"{tag}_chase.png");
                Save(GrabRt(SensorOutput), $"{tag}_onboard.png");
                PlaceOverview();
                Save(Grab(m_ViewCam, w, h), $"{tag}_overview.png");
                m_ChaseInit = false;
                idx++;
            }

            // 名所と車の確認用の視点 (-shotviews)。車は最初の位置 (panasonic はコーナーの手前) に置く
            foreach (var raw in Arg("-shotviews", "").Split(','))
            {
                string view = raw.Trim().ToLowerInvariant();
                if (view == "") continue;
                float s = list.Count > 0 ? list[0] : 0f;
                if (!PlaceView(view, ref s, len, out Vector3 eye, out Vector3 look, out float fov))
                { Debug.LogWarning($"[Shots] unknown or unavailable view: {view}"); continue; }
                for (int k = 0; k < 45; k++) { SetCars(s, 0.0); yield return null; }
                yield return new WaitForEndOfFrame();
                float oldFov = m_ViewCam.fieldOfView;
                m_ViewCam.transform.position = eye;
                m_ViewCam.transform.rotation = Quaternion.LookRotation(look - eye, Vector3.up);
                m_ViewCam.fieldOfView = fov;
                Save(Grab(m_ViewCam, w, h), $"{idx:00}_view_{view}.png");
                m_ViewCam.fieldOfView = oldFov;
                m_ChaseInit = false;
                idx++;
            }

            int frames = int.Parse(Arg("-bench", "600"));
            var report = new StringBuilder();
            report.AppendLine("# 描画の計測 (SimBridge -shots / -bench)");
            report.AppendLine();
            report.AppendLine("| 項目 | 値 |");
            report.AppendLine("| --- | --- |");
            report.AppendLine($"| 日時 | {DateTime.Now:yyyy-MM-dd HH:mm} |");
            report.AppendLine($"| GPU | {SystemInfo.graphicsDeviceName} ({SystemInfo.graphicsDeviceType}, {SystemInfo.graphicsMemorySize} MB) |");
            report.AppendLine($"| CPU | {SystemInfo.processorType} ×{SystemInfo.processorCount} |");
            report.AppendLine($"| Unity | {Application.unityVersion} |");
            report.AppendLine($"| コース | {Arg("-course", "course.json")} (circuit: {Circuit}) |");
            report.AppendLine($"| 画質 | {RenderQuality.Current} ({RenderQuality.Reason})、描画 {RenderCompat.PipelineName} |");
            report.AppendLine($"| 画面 | {Screen.width}x{Screen.height}、レイアウト {m_Layout} |");
            report.AppendLine($"| コースの組み立て | {m_BuildSeconds:F2} s |");
            if (frames > 0)
            {
                // 上限を外して計る。車は実車スケールなら 50 m/s・ミニカーなら 2 m/s でコースを進む
                int oldRate = Application.targetFrameRate;
                Application.targetFrameRate = -1;
                double v = Circuit ? 50.0 : 2.0;
                float s0 = list.Count > 0 ? list[0] : 0f, t0 = Time.realtimeSinceStartup;
                var dts = new List<float>(frames);
                int warm = 60;
                for (int k = 0; k < frames + warm; k++)
                {
                    SetCars(s0 + (float)(v * (Time.realtimeSinceStartup - t0)), v);
                    yield return null;
                    if (k >= warm) dts.Add(Time.unscaledDeltaTime);
                }
                Application.targetFrameRate = oldRate;
                dts.Sort();
                float sum = 0f;
                foreach (float d in dts) sum += d;
                float avg = sum / dts.Count, p99 = dts[Mathf.Min(dts.Count - 1, (int)(dts.Count * 0.99f))], worst = dts[dts.Count - 1];
                report.AppendLine($"| 計測フレーム | {dts.Count} (車速 {v} m/s で走行) |");
                report.AppendLine($"| 平均 | {1f / avg:F1} fps ({avg * 1000f:F2} ms) |");
                report.AppendLine($"| 99 % / 最悪 | {p99 * 1000f:F2} ms / {worst * 1000f:F2} ms |");
                report.AppendLine($"| センサ配信 | {m_PubRate:F1} Hz (ROS 無しでも描画と読み出しは同じ) |");
                Debug.Log($"[Shots] bench: {1f / avg:F1} fps avg, p99 {p99 * 1000f:F2} ms, worst {worst * 1000f:F2} ms");
            }
            report.AppendLine();
            report.AppendLine("lockstep の 1 判断あたりの時間は gateway のログ (decisions=… /s) で計る (docs/mlagents.md)。");
            File.WriteAllText(Path.Combine(m_ShotDir, "bench.md"), report.ToString());
            Debug.Log($"[Shots] done → {m_ShotDir}");
            yield return null;
            Application.Quit();
        }

        // ------------------------------------------------------------------ 視点
        /// -shotviews の視点。grandstand・panasonic・scenic は tools/preview_circuit.py の同名の視点と同じ位置・向き (サーキットだけ)。
        /// carfront・carside・carrear は自車を前斜め・真横・後ろ斜めから見る (距離は車長の倍数)
        bool PlaceView(string view, ref float s, float len, out Vector3 eye, out Vector3 look, out float fov)
        {
            eye = look = Vector3.zero; fov = 46f;
            PoseAt(s, 0f, out double cx, out double cy, out double yaw);
            var car = new Vector2((float)cx, (float)cy);
            var t = new Vector2(Mathf.Cos((float)yaw), Mathf.Sin((float)yaw));
            var nrm = new Vector2(-t.y, t.x);
            if (view == "carfront" || view == "carside" || view == "carrear")
            {
                Vector2 off = view == "carfront" ? t * 1.25f + nrm * 0.95f : view == "carside" ? nrm * 1.7f : t * -1.25f + nrm * 0.95f;
                Vector2 e = car + off * len;
                eye = RosFrame.ToUnity(e.x, e.y, (view == "carside" ? 0.16f : 0.30f) * len);
                look = RosFrame.ToUnity(car.x, car.y, 0.13f * len);
                fov = 32f;
                return true;
            }
            var c = m_Course.Data.circuit;
            if (!Circuit || c == null || c.bounds == null || c.bounds.Length < 4) return false;
            var mid = new Vector2((c.bounds[0] + c.bounds[2]) * 0.5f, (c.bounds[1] + c.bounds[3]) * 0.5f);
            Vector2 fuji = Landscape.FujiDir, peak = mid + fuji * Landscape.MountainDist;
            float lookZ = Landscape.MountainH * 0.32f;
            if (view == "grandstand")
            {
                Vector2 p0 = m_PathP[0], n0 = Normal(0), mean = Vector2.zero;
                foreach (var q in m_PathP) mean += q;
                mean /= m_PathP.Length;
                float inside = Vector2.Dot(mean - p0, n0) > 0f ? 1f : -1f;
                Vector2 e = p0 - n0 * inside * (c.width_m * 0.5f + c.runoff_m + 15f);
                eye = RosFrame.ToUnity(e.x, e.y, 8f);
            }
            else if (view == "panasonic")
            {
                CornerData pc = null;
                if (c.corners != null) foreach (var k in c.corners) if (k.name != null && k.name.Contains("パナソニック")) pc = k;
                if (pc == null) return false;
                var corner = new Vector2(pc.x, pc.y);
                Vector2 e = corner - fuji * 70f + new Vector2(-fuji.y, fuji.x) * 25f;
                eye = RosFrame.ToUnity(e.x, e.y, 4f);
                int best = 0;
                for (int i = 1; i < m_PathP.Length; i++)
                    if ((m_PathP[i] - corner).sqrMagnitude < (m_PathP[best] - corner).sqrMagnitude) best = i;
                s = m_PathS[best] - 3f * len;       // 車はコーナーに入る所
            }
            else if (view == "scenic")
            {
                Vector2 e = car - t * 8f + nrm * 6f;
                eye = RosFrame.ToUnity(e.x, e.y, 2.5f);
                lookZ = Landscape.MountainH * 0.40f;
                fov = 50f;
            }
            else return false;
            look = RosFrame.ToUnity(peak.x, peak.y, lookZ);
            return true;
        }

        Vector2 Normal(int i)
        {
            Vector2 t = (m_PathP[(i + 1) % m_PathP.Length] - m_PathP[i]).normalized;
            return new Vector2(-t.y, t.x);
        }

        // ------------------------------------------------------------------ 位置
        void BuildPath()
        {
            var d = m_Course.Data;
            float[] f = d.centerline_shortcut != null && d.centerline_shortcut.Length >= 8 ? d.centerline_shortcut : d.centerline_long;
            var p = new List<Vector2>();
            if (f != null)
                for (int i = 0; i + 1 < f.Length; i += 2) p.Add(new Vector2(f[i], f[i + 1]));
            if (p.Count < 2) { p.Clear(); p.Add(Vector2.zero); p.Add(Vector2.right); }
            if (p.Count > 2 && (p[0] - p[p.Count - 1]).sqrMagnitude < 1e-6f) p.RemoveAt(p.Count - 1);
            m_PathP = p.ToArray();
            m_PathS = new float[m_PathP.Length + 1];
            for (int i = 0; i < m_PathP.Length; i++)
                m_PathS[i + 1] = m_PathS[i] + (m_PathP[(i + 1) % m_PathP.Length] - m_PathP[i]).magnitude;
        }

        void PoseAt(float s, float lateral, out double x, out double y, out double yaw)
        {
            int n = m_PathP.Length;
            float total = Mathf.Max(1e-3f, m_PathS[n]);
            s = Mathf.Repeat(s, total);
            int i = Array.BinarySearch(m_PathS, s);
            if (i < 0) i = ~i - 1;
            i = Mathf.Clamp(i, 0, n - 1);
            Vector2 a = m_PathP[i], b = m_PathP[(i + 1) % n];
            float seg = Mathf.Max(1e-6f, m_PathS[i + 1] - m_PathS[i]);
            Vector2 p = Vector2.Lerp(a, b, (s - m_PathS[i]) / seg);
            Vector2 t = (b - a).normalized;
            p += new Vector2(-t.y, t.x) * lateral;          // 左が正
            x = p.x; y = p.y; yaw = Math.Atan2(t.y, t.x);
        }

        float CarLength()
        {
            var v = m_Course.Data.vehicle;
            return v != null && v.length_m > 0f ? v.length_m : 0.45f;
        }

        double[] FakeState(float s, float lateral, double stamp, double v)
        {
            var d = new double[(int)F.Count];
            PoseAt(s, lateral, out double x, out double y, out double yaw);
            d[(int)F.StampSec] = Math.Floor(stamp);
            d[(int)F.StampNsec] = (stamp - Math.Floor(stamp)) * 1e9;
            d[(int)F.X] = x; d[(int)F.Y] = y; d[(int)F.Yaw] = yaw;
            d[(int)F.SimTime] = stamp;
            d[(int)F.V] = v;
            d[(int)F.Distance] = s;
            return d;
        }

        /// 自車を s に、ライバル 2 台を前と斜め前に置く (/sim/render_state などを受けたのと同じ扱い)
        void SetCars(float s, double v)
        {
            double now = Time.realtimeSinceStartupAsDouble;
            float L = CarLength();
            m_State = FakeState(s, 0f, now, v);
            m_StateDirty = true;
            m_Poses[0].Add(m_State);
            m_Smooth[0].Add(m_State);
            m_RivalState = FakeState(s + 7f * L, 0.8f * L, now, v);
            m_Poses[1].Add(m_RivalState);
            m_Smooth[1].Add(m_RivalState);
            m_Rival2State = FakeState(s + 3.5f * L, -0.8f * L, now, v);
            m_Poses[2].Add(m_Rival2State);
            m_Smooth[2].Add(m_Rival2State);
        }

        // ------------------------------------------------------------------ 撮影
        static void ParseSize(string s, out int w, out int h)
        {
            w = 1920; h = 1080;
            var p = s.ToLowerInvariant().Split('x');
            if (p.Length == 2 && int.TryParse(p[0], out int a) && int.TryParse(p[1], out int b) && a >= 64 && b >= 64) { w = a; h = b; }
        }

        /// カメラを w×h で 1 回描いて読む (画面の HUD は入らない)。4x MSAA で描いてから解決して読む
        static Texture2D Grab(Camera cam, int w, int h)
        {
            var msaa = RenderTexture.GetTemporary(w, h, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB, 4);
            var rt = RenderTexture.GetTemporary(w, h, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            var prev = cam.targetTexture;
            cam.targetTexture = msaa;
            cam.aspect = (float)w / h;
            cam.Render();
            cam.targetTexture = prev;
            cam.ResetAspect();
            Graphics.Blit(msaa, rt);
            var tex = ReadRt(rt);
            RenderTexture.ReleaseTemporary(msaa);
            RenderTexture.ReleaseTemporary(rt);
            return tex;
        }

        static Texture2D GrabRt(RenderTexture src) => src == null ? null : ReadRt(src);

        static Texture2D ReadRt(RenderTexture rt)
        {
            var prev = RenderTexture.active;
            RenderTexture.active = rt;
            var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
            tex.Apply();
            RenderTexture.active = prev;
            return tex;
        }

        void Save(Texture2D tex, string file)
        {
            if (tex == null) return;
            File.WriteAllBytes(Path.Combine(m_ShotDir, file), tex.EncodeToPNG());
            Destroy(tex);
        }
    }
}
