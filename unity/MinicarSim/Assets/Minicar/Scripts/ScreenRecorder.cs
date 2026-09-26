// Unity の画面 (追従視点 + HUD + センサ画像) を ffmpeg へ直接流して録画する。
//
//   MinicarSim.x86_64 ... -record ~/Videos/race.mp4 [-recordfps 30]
//
// デスクトップを経由しない (x11grab を使わない) ので、Wayland/XWayland でも
// 黒画面やカクつきが出ない。フレームは実時間に合わせて並べる: 描画が遅れた
// 区間は直前のフレームを複製するので、動画の長さと速さは実時間どおりになる。
// ffmpeg (libx264 ultrafast) は別プロセスで動くので Unity の描画は止めない。
using System;
using System.Collections;
using System.Diagnostics;
using System.IO;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using Debug = UnityEngine.Debug;

namespace Minicar
{
    public class ScreenRecorder : MonoBehaviour
    {
        Process m_Ffmpeg;
        Stream m_Pipe;
        RenderTexture m_Rt, m_Small;
        byte[] m_Frame;
        bool m_HasFrame, m_Busy;
        int m_W, m_H;
        float m_Fps;
        double m_T0;
        long m_Written;

        public static void StartIfRequested(GameObject host, string path, float fps, int width)
        {
            if (string.IsNullOrEmpty(path)) return;
            var rec = host.AddComponent<ScreenRecorder>();
            rec.Begin(path, fps, width);
        }

        void Begin(string path, float fps, int width)
        {
            path = path.StartsWith("~/") ? Path.Combine(Environment.GetEnvironmentVariable("HOME") ?? "", path.Substring(2)) : path;
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path)));
            // 画面を GPU 上で録画サイズ (既定 幅 1280) に縮めてから取り出す。
            // フル解像度のまま読み出すと転送と符号化が重く、センサ画像の配信が 30→21 Hz に落ちた。
            // libx264 は偶数サイズが必要
            m_W = Mathf.Min(width, Screen.width) & ~1;
            m_H = Mathf.RoundToInt((float)m_W * Screen.height / Screen.width) & ~1;
            m_Fps = Mathf.Clamp(fps, 5f, 60f);
            m_Rt = new RenderTexture(Screen.width, Screen.height, 0, RenderTextureFormat.ARGB32);
            m_Small = new RenderTexture(m_W, m_H, 0, RenderTextureFormat.ARGB32);
            m_Frame = new byte[m_W * m_H * 4];

            var psi = new ProcessStartInfo("ffmpeg",
                $"-y -loglevel error -f rawvideo -pix_fmt rgba -s {m_W}x{m_H} -r {m_Fps} -i - " +
                "-vf vflip -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p " +
                // 断片化 mp4: Unity が強制終了されても、そこまでの動画は再生できる
                $"-movflags +frag_keyframe+empty_moov+default_base_moof \"{path}\"")
            {
                UseShellExecute = false,
                RedirectStandardInput = true,
                CreateNoWindow = true,
            };
            try
            {
                m_Ffmpeg = Process.Start(psi);
                m_Pipe = m_Ffmpeg.StandardInput.BaseStream;
            }
            catch (Exception e)
            {
                Debug.LogError($"[ScreenRecorder] ffmpeg を起動できない (sudo apt install ffmpeg): {e.Message}");
                Destroy(this);
                return;
            }
            m_T0 = Time.realtimeSinceStartupAsDouble;
            Debug.Log($"[ScreenRecorder] 録画開始 {m_W}x{m_H} {m_Fps}fps → {path}");
            StartCoroutine(CaptureLoop());
        }

        IEnumerator CaptureLoop()
        {
            var eof = new WaitForEndOfFrame();
            double nextGrab = 0;
            while (m_Pipe != null)
            {
                yield return eof;     // IMGUI (HUD) まで描き終わった後の画面
                double now = Time.realtimeSinceStartupAsDouble - m_T0;
                if (!m_Busy && now >= nextGrab)
                {
                    nextGrab = now + 1.0 / m_Fps * 0.9;
                    if (m_Rt.width != Screen.width || m_Rt.height != Screen.height)
                    {
                        // 窓の大きさが変わったら取り込み先だけ作り直す (録画サイズは固定)
                        m_Rt.Release();
                        m_Rt = new RenderTexture(Screen.width, Screen.height, 0, RenderTextureFormat.ARGB32);
                    }
                    ScreenCapture.CaptureScreenshotIntoRenderTexture(m_Rt);
                    Graphics.Blit(m_Rt, m_Small);
                    m_Busy = true;
                    AsyncGPUReadback.Request(m_Small, 0, TextureFormat.RGBA32, OnReadback);
                }
                WriteDueFrames(now);
            }
        }

        void OnReadback(AsyncGPUReadbackRequest req)
        {
            m_Busy = false;
            if (req.hasError || m_Pipe == null) return;
            NativeArray<byte>.Copy(req.GetData<byte>(), m_Frame, m_Frame.Length);
            m_HasFrame = true;
        }

        // 実時間で「今までに出ているべき枚数」まで最新フレームを書く (遅れた分は複製)
        void WriteDueFrames(double now)
        {
            if (!m_HasFrame) return;
            long due = (long)(now * m_Fps);
            try
            {
                while (m_Written < due)
                {
                    m_Pipe.Write(m_Frame, 0, m_Frame.Length);
                    m_Written++;
                }
            }
            catch (IOException e)
            {
                Debug.LogError($"[ScreenRecorder] ffmpeg への書き込みに失敗: {e.Message}");
                Stop();
            }
        }

        void Stop()
        {
            if (m_Pipe == null) return;
            try { m_Pipe.Flush(); m_Pipe.Close(); } catch (Exception) { }
            m_Pipe = null;
            // ffmpeg に mp4 の moov を書き終えさせる (これを待たないと再生できないファイルになる)
            if (m_Ffmpeg != null && !m_Ffmpeg.WaitForExit(10000)) m_Ffmpeg.Kill();
            Debug.Log($"[ScreenRecorder] 録画終了 {m_Written} フレーム");
        }

        void OnApplicationQuit() => Stop();
        void OnDestroy()
        {
            Stop();
            if (m_Rt != null) m_Rt.Release();
            if (m_Small != null) m_Small.Release();
        }
    }
}
