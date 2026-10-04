// 自車のエンジン音・走行音をその場で合成する (音源ファイルは使わない)。
//
// エンジン: 燃焼ごとの短い圧力パルス (気筒・ローターごとに少し強さが違い、毎回わずかにばらつく) を
//           排気管の共鳴 (帯域フィルタ 3 つ = 排気系の響き) と管の往復 (くし形フィルタ) に通し、踏むほど強く歪ませる。
//   787B : 4 ローター R26B (エキセン軸 1 回転で 4 回燃焼・9,000 rpm で 600 Hz)。鋭いパルスと高い共鳴で金属的な絶叫、
//          アクセルを戻すとアフターファイヤーの破裂音、ストレートカットのギアの唸り
//   RX-7 : 2 ローター 13B (1 回転で 2 回) ＋ ターボの過給音。アイドルのばらつき (ロータリー特有の「ブロロ」)
//   ND   : 直列 4 気筒 (1 回転で 2 回・4 回で 1 巡) の低めで丸い音、オープンなので風切り音が大きい
// ほかに ロードノイズ (速度)・風切り音 (速度²)・タイヤのスキール (横加速度が限界付近 / 強いブレーキ)。
//
// 入力は /sim/render_state の車速・横加速度だけ (表示用)。アクセル開度は車速の変化から推定する。
// ミニカーの車速は -soundvmax [m/s] を実車の最高速に当てて回転数を決める (見た目の演出で、物理には関係しない)。
// tools/engine_sound.py が同じ式で WAV を書き出す (Unity 無しで聴いて確かめる)。式を変えたらそちらも合わせる。
using UnityEngine;

namespace Minicar
{
    [RequireComponent(typeof(AudioSource))]
    public class EngineAudio : MonoBehaviour
    {
        struct Voice
        {
            public int Pulses;          // エキセン軸 / クランク 1 回転あたりの燃焼回数
            public int Cycle;           // 燃焼の強さのくせが一巡する回数 (ローター数・気筒数)
            public float Idle, Red;     // アイドル・レッドゾーン [rpm]
            public float[] Gears;       // 各ギアで最高速の何割まで回せるか (最終段 = 1)
            public float Sharp;         // パルスの鋭さ (周期に対する減衰の速さ。大きいほど高音が強い)
            public float Rasp;          // 燃焼ごとのばらつき・雑音 (ロータリーのざらつき)
            public float[] FormHz;      // 排気系の共鳴 [Hz]
            public float[] FormGain;
            public float FormQ;
            public float PipeHz, PipeFb; // 管の往復の共鳴 [Hz] とその強さ
            public float Drive;         // 歪ませる強さ
            public float Pop;           // アクセルを戻したときの破裂音
            public float Whine;         // ギアの唸り
            public float Turbo;         // 過給音の量
            public float Wind;          // 風切り音の量
            public float Level;         // 全体の音量の合わせ
            public float Direct;        // 共鳴を通さない素の成分 (基音の強さ)。0 なら 0.25
            public float CutHi;         // 全開・高回転での高域の上限 [Hz]。0 なら 9000
            public float Tilt;          // さらに高域を落とす 2 段目 [Hz]。0 なら無し
            public float Wobble;        // 燃焼ごとの回転の揺らぎ (きれいすぎる倍音を崩す)
        }

        static Voice VoiceOf(CarStyle s)
        {
            switch (s)
            {
                case CarStyle.B787:
                    // 実車の録音と帯域ごとの強さを比べて合わせた (2026-10-05)。400〜1000 Hz が中心で、2.5 kHz より上は少ない
                    return new Voice { Pulses = 4, Cycle = 4, Idle = 1900, Red = 9000, Gears = new[] { 0.36f, 0.52f, 0.67f, 0.83f, 1f },
                                       Sharp = 5f, Rasp = 0.45f, FormHz = new[] { 230f, 640f, 1500f }, FormGain = new[] { 1.3f, 1f, 0.75f },
                                       FormQ = 1.6f, PipeHz = 230f, PipeFb = 0.42f, Drive = 2.6f, Pop = 1f, Whine = 0.008f,
                                       Turbo = 0f, Wind = 0.5f, Level = 0.55f, Direct = 0.6f, CutHi = 5000f, Tilt = 3200f, Wobble = 0.02f };
                case CarStyle.Rx7:
                    return new Voice { Pulses = 2, Cycle = 2, Idle = 900, Red = 8000, Gears = new[] { 0.27f, 0.42f, 0.58f, 0.75f, 1f },
                                       Sharp = 9f, Rasp = 0.40f, FormHz = new[] { 620f, 1500f, 3100f }, FormGain = new[] { 1f, 0.7f, 0.35f },
                                       FormQ = 2.0f, PipeHz = 190f, PipeFb = 0.35f, Drive = 2.0f, Pop = 0.45f, Whine = 0f,
                                       Turbo = 1f, Wind = 0.7f, Level = 0.65f };
                default:   // Roadster (と従来ボディ)
                    return new Voice { Pulses = 2, Cycle = 4, Idle = 750, Red = 7500, Gears = new[] { 0.25f, 0.39f, 0.54f, 0.69f, 0.84f, 1f },
                                       Sharp = 6f, Rasp = 0.10f, FormHz = new[] { 260f, 720f, 1900f }, FormGain = new[] { 1f, 0.6f, 0.25f },
                                       FormQ = 1.6f, PipeHz = 130f, PipeFb = 0.30f, Drive = 1.4f, Pop = 0f, Whine = 0f,
                                       Turbo = 0f, Wind = 1.3f, Level = 0.8f };
            }
        }

        // 気筒・ローターごとの燃焼の強さのくせ (Cycle 個を巡回)
        static readonly float[] kBias = { 0.00f, 0.18f, -0.12f, 0.07f };

        public bool Muted { get; set; }
        public int Gear => m_Gear;
        public float Rpm => m_Rpm;

        Voice m_Voice;
        float m_Volume = 0.6f, m_VMax = 3f, m_ALatMax = 4.4f;
        int m_Gear = 1;
        float m_PrevV, m_ALongF, m_ShiftDip;
        bool m_HasPrev;

        // 音声スレッドと共有する値 (毎フレーム書き、音声側は読むだけ)
        volatile float m_Rpm, m_Thr, m_SpeedFrac, m_Slip, m_Gain;
        int m_SampleRate;
        double m_Phase;
        long m_Event;
        float m_Wob, m_Lp2;
        float m_Since, m_Amp, m_Hp, m_HpX, m_Lp, m_Road, m_Wind1, m_Wind2, m_SqY1, m_SqY2, m_TurboPh, m_WhinePh, m_Smooth;
        float m_PopEnv, m_PrevThr;
        readonly float[] m_Fx1 = new float[3], m_Fx2 = new float[3], m_Fy1 = new float[3], m_Fy2 = new float[3];
        float[] m_Pipe;
        int m_PipeIdx;
        uint m_Rng = 22222;

        /// vmax: この車速で最高速 (最終段のレッドゾーン)、aLatMax: スキールが鳴り始める横加速度の基準 [m/s²]
        public static EngineAudio Create(GameObject host, CarStyle style, float volume, float vmax, float aLatMax = 4.4f)
        {
            // 音を聴く耳 (AudioListener) が無ければ付ける。カメラはコードで作っているので無いことがある
            if (Object.FindFirstObjectByType<AudioListener>() == null) host.AddComponent<AudioListener>();
            var go = new GameObject("EngineAudio");
            go.transform.SetParent(host.transform, false);
            var src = go.AddComponent<AudioSource>();
            var e = go.AddComponent<EngineAudio>();
            e.m_Voice = VoiceOf(style);
            e.m_Volume = Mathf.Clamp01(volume);
            e.m_VMax = Mathf.Max(0.1f, vmax);
            e.m_ALatMax = Mathf.Max(0.5f, aLatMax);
            e.m_SampleRate = AudioSettings.outputSampleRate;
            e.m_Pipe = new float[Mathf.Max(8, Mathf.RoundToInt((e.m_SampleRate > 0 ? e.m_SampleRate : 48000) / e.m_Voice.PipeHz))];
            // OnAudioFilterRead を確実に呼ばせるため、無音のループを鳴らしておく (中身はフィルタで上書き)
            src.clip = AudioClip.Create("silence", e.m_SampleRate, 1, e.m_SampleRate, false);
            src.loop = true;
            src.spatialBlend = 0f;
            src.playOnAwake = false;
            src.Play();
            return e;
        }

        /// v [m/s]・aLat [m/s²] を毎フレーム渡す
        public void SetState(float v, float aLat, float dt)
        {
            if (dt <= 1e-4f) return;
            float aLong = m_HasPrev ? Mathf.Clamp((v - m_PrevV) / dt, -12f, 12f) : 0f;
            m_PrevV = v; m_HasPrev = true;
            m_ALongF += (aLong - m_ALongF) * (1f - Mathf.Exp(-dt / 0.15f));

            float frac = Mathf.Clamp01(Mathf.Abs(v) / m_VMax);
            var vc = m_Voice;
            // 自動変速: 上がり切ったらシフトアップ、下がったらダウン
            float RpmAt(int g) => Mathf.Max(vc.Idle, vc.Red * frac / vc.Gears[g - 1]);
            if (RpmAt(m_Gear) > vc.Red * 0.96f && m_Gear < vc.Gears.Length) { m_Gear++; m_ShiftDip = 0.09f; }
            else if (m_Gear > 1 && RpmAt(m_Gear - 1) < vc.Red * 0.70f) m_Gear--;
            float rpm = Mathf.Min(vc.Red, RpmAt(m_Gear));

            // アクセル開度の推定: 加速していれば踏んでいる。一定速でも少しは踏んでいる
            float thr = Mathf.Clamp01(m_ALongF / (0.35f * m_ALatMax) + (frac > 0.03f ? 0.25f : 0.05f));
            if (m_ShiftDip > 0f) { m_ShiftDip -= dt; thr *= 0.3f; }
            // スキール: 横加速度の上限 (ミニカーは実測のフルロック 0.45 g、実車は μg) の 85 % 超、または強いブレーキ
            float slip = Mathf.Clamp01((Mathf.Abs(aLat) / m_ALatMax - 0.85f) * 5f)
                       + (m_ALongF < -0.7f * m_ALatMax && frac > 0.15f ? 0.5f : 0f);

            m_Rpm = rpm; m_Thr = thr; m_SpeedFrac = frac; m_Slip = Mathf.Clamp01(slip);
            m_Gain = Muted ? 0f : m_Volume;
        }

        float Noise()
        {
            // xorshift の白色雑音 (-1〜1)。音声スレッドで UnityEngine.Random は使えない
            m_Rng ^= m_Rng << 13; m_Rng ^= m_Rng >> 17; m_Rng ^= m_Rng << 5;
            return (m_Rng & 0xFFFFFF) / 8388608f - 1f;
        }

        void OnAudioFilterRead(float[] data, int channels)
        {
            if (m_Pipe == null) return;
            float sr = m_SampleRate > 0 ? m_SampleRate : 48000f;
            Render(data, channels, sr);
        }

        /// 1 ブロック分を合成する (値は SetState がフレームごとに更新。tools/engine_sound.py と同じ処理)
        void Render(float[] data, int channels, float sr)
        {
            var vc = m_Voice;
            float rpm = m_Rpm, thr = m_Thr, frac = m_SpeedFrac, slip = m_Slip, gain = m_Gain;
            float load = rpm / vc.Red;
            float f0 = rpm / 60f * vc.Pulses;                       // 燃焼の基本周波数 [Hz]
            float tau = 1f / (f0 * vc.Sharp * (0.7f + 0.6f * thr));  // パルスの減衰 [s] (踏むほど鋭く)
            float jitter = vc.Rasp * Mathf.Lerp(1.0f, 0.35f, load);   // 低回転ほどばらつく (アイドルの「ブロロ」)
            float grit = vc.Rasp * 0.35f * Mathf.Lerp(1.0f, 0.15f, load);
            float cut = Mathf.Lerp(1800f, vc.CutHi > 0f ? vc.CutHi : 9000f, Mathf.Clamp01(0.4f * thr + 0.6f * load));
            float direct = vc.Direct > 0f ? vc.Direct : 0.25f;
            float aTilt = vc.Tilt > 0f ? 1f - Mathf.Exp(-2f * Mathf.PI * vc.Tilt / sr) : 1f;
            float aLp = 1f - Mathf.Exp(-2f * Mathf.PI * cut / sr);
            float aHp = Mathf.Exp(-2f * Mathf.PI * 30f / sr);
            float drive = 1f + vc.Drive * (0.3f + 0.7f * thr);
            float engAmp = vc.Level * (0.12f + 0.30f * thr) * (0.55f + 0.45f * load);
            // アクセルを急に戻した高回転では、しばらく破裂音 (アフターファイヤー)
            if (m_PrevThr - thr > 0.25f && load > 0.45f) m_PopEnv = 1f;
            m_PrevThr = thr;
            float roadAmp = 0.16f * Mathf.Min(1f, frac * 1.4f);
            float windAmp = 0.08f * vc.Wind * frac * frac;
            float sqAmp = 0.10f * slip;
            float turboAmp = 0.02f * vc.Turbo * thr * load;
            float turboF = 2200f + rpm * 0.55f;
            float whineAmp = vc.Whine * load * (0.4f + 0.6f * thr), whineF = rpm / 60f * 23f;
            // 排気系の共鳴 (RBJ の帯域フィルタ・ピーク 0 dB)
            float b0_0 = 0, a1_0 = 0, a2_0 = 0, b0_1 = 0, a1_1 = 0, a2_1 = 0, b0_2 = 0, a1_2 = 0, a2_2 = 0;
            Bpf(vc.FormHz[0], vc.FormQ, sr, out b0_0, out a1_0, out a2_0);
            Bpf(vc.FormHz[1], vc.FormQ, sr, out b0_1, out a1_1, out a2_1);
            Bpf(vc.FormHz[2], vc.FormQ, sr, out b0_2, out a1_2, out a2_2);
            // スキールの共振 (1.15 kHz、Q 高め)
            float w0 = 2f * Mathf.PI * 1150f / sr, r = 0.995f;
            float b1 = 2f * r * Mathf.Cos(w0), b2 = -r * r;
            float aRoad = 1f - Mathf.Exp(-2f * Mathf.PI * 380f / sr);
            float aW1 = 1f - Mathf.Exp(-2f * Mathf.PI * 2500f / sr), aW2 = 1f - Mathf.Exp(-2f * Mathf.PI * 500f / sr);
            float dt = 1f / sr;

            for (int i = 0; i < data.Length; i += channels)
            {
                // ---- 燃焼のパルス
                m_Phase += f0 * (1f + vc.Wobble * m_Wob) * dt;
                m_Since += dt;
                if (m_Phase >= 1.0)
                {
                    m_Phase -= System.Math.Floor(m_Phase);
                    int k = (int)(m_Event++ % vc.Cycle);
                    float a = 1f + vc.Rasp * kBias[k % kBias.Length] + jitter * 0.5f * Noise();
                    if (m_PopEnv > 0.05f && vc.Pop > 0f && Noise() > 0.55f) a += vc.Pop * 3.5f * m_PopEnv * (0.5f + 0.5f * Noise());
                    m_Amp = a;
                    if (vc.Wobble > 0f) m_Wob += (Noise() - m_Wob) * 0.15f;
                    m_Since = (float)m_Phase / Mathf.Max(1f, f0);
                }
                float env = Mathf.Exp(-m_Since / tau);
                // 燃焼の雑音は低回転ほど多い (高回転は澄んだ「パーン」、低回転は粒の立った「ボッボッ」)
                float x = m_Amp * env * (1f + grit * Noise());
                // 直流を切る
                float hp = aHp * (m_Hp + x - m_HpX); m_HpX = x; m_Hp = hp;
                // 共鳴 3 つ + 少しの素通し
                float f = direct * hp;
                f += vc.FormGain[0] * Biquad(0, hp, b0_0, a1_0, a2_0);
                f += vc.FormGain[1] * Biquad(1, hp, b0_1, a1_1, a2_1);
                f += vc.FormGain[2] * Biquad(2, hp, b0_2, a1_2, a2_2);
                // 管の往復 (くし形)
                float pipe = f + vc.PipeFb * m_Pipe[m_PipeIdx];
                m_Pipe[m_PipeIdx] = pipe;
                if (++m_PipeIdx >= m_Pipe.Length) m_PipeIdx = 0;
                // 歪み → 高域を少し丸める
                float sat = (float)(System.Math.Tanh(drive * pipe) / System.Math.Tanh(drive));
                m_Lp += (sat - m_Lp) * aLp;
                m_Lp2 += (m_Lp - m_Lp2) * aTilt;
                float eng = m_Lp2 * engAmp;

                float n = Noise();
                m_Road += (n - m_Road) * aRoad;
                m_Wind1 += (n - m_Wind1) * aW1; m_Wind2 += (m_Wind1 - m_Wind2) * aW2;   // 帯域 500〜2500 Hz
                float wind = (m_Wind1 - m_Wind2) * 2f;
                float sq = n * 0.01f + b1 * m_SqY1 + b2 * m_SqY2; m_SqY2 = m_SqY1; m_SqY1 = sq;
                m_TurboPh += turboF * dt; if (m_TurboPh > 1f) m_TurboPh -= 1f;
                float turbo = Mathf.Sin(2f * Mathf.PI * m_TurboPh) * turboAmp;
                m_WhinePh += whineF * dt; if (m_WhinePh > 1f) m_WhinePh -= 1f;
                float whine = Mathf.Sin(2f * Mathf.PI * m_WhinePh) * whineAmp;

                float y = eng + m_Road * roadAmp * 3f + wind * windAmp + sq * sqAmp + turbo + whine;
                // 音量の急変を避ける
                m_Smooth += (gain - m_Smooth) * 0.0005f;
                y *= m_Smooth;
                for (int c = 0; c < channels; c++) data[i + c] = y;
            }
            m_PopEnv *= Mathf.Exp(-(data.Length / channels) * dt / 0.6f);   // 破裂音は 0.6 s ほどで収まる
        }

        static void Bpf(float hz, float q, float sr, out float b0, out float a1, out float a2)
        {
            float w = 2f * Mathf.PI * Mathf.Min(hz, sr * 0.45f) / sr, alpha = Mathf.Sin(w) / (2f * q), a0 = 1f + alpha;
            b0 = alpha / a0; a1 = -2f * Mathf.Cos(w) / a0; a2 = (1f - alpha) / a0;
        }

        float Biquad(int k, float x, float b0, float a1, float a2)
        {
            // b1 = 0, b2 = −b0 (帯域フィルタ)
            float y = b0 * x - b0 * m_Fx2[k] - a1 * m_Fy1[k] - a2 * m_Fy2[k];
            m_Fx2[k] = m_Fx1[k]; m_Fx1[k] = x; m_Fy2[k] = m_Fy1[k]; m_Fy1[k] = y;
            return y;
        }
    }
}
