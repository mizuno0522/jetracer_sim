// 自車のエンジン音・走行音をその場で合成する (音源ファイルは使わない)。
//
// 787B : 4 ローター (エキセン軸 1 回転で 4 回の燃焼) の高く鋭い音
// RX-7 : 2 ローター (1 回転で 2 回) ＋ ターボの過給音
// ND   : 直列 4 気筒 (1 回転で 2 回) の低めで丸い音、オープンなので風切り音が大きい
// ほかに ロードノイズ (速度)・風切り音 (速度²)・タイヤのスキール (横加速度が限界付近 / 強いブレーキ)。
//
// 入力は /sim/render_state の車速・横加速度だけ (表示用)。アクセル開度は車速の変化から推定する。
// ミニカーの車速は -soundvmax [m/s] を実車の最高速に当てて回転数を決める (見た目の演出で、物理には関係しない)。
using UnityEngine;

namespace Minicar
{
    [RequireComponent(typeof(AudioSource))]
    public class EngineAudio : MonoBehaviour
    {
        struct Voice
        {
            public int Pulses;          // エキセン軸 / クランク 1 回転あたりの燃焼回数
            public float Idle, Red;     // アイドル・レッドゾーン [rpm]
            public float[] Gears;       // 各ギアで最高速の何割まで回せるか (最終段 = 1)
            public float Rasp;          // ロータリーらしいざらつき (燃焼ノイズの量)
            public float Bright;        // 倍音の強さ (高いほど鋭い)
            public float Turbo;         // 過給音の量
            public float Wind;          // 風切り音の量
        }

        static Voice VoiceOf(CarStyle s)
        {
            switch (s)
            {
                case CarStyle.B787:
                    return new Voice { Pulses = 4, Idle = 2200, Red = 9000, Gears = new[] { 0.36f, 0.52f, 0.67f, 0.83f, 1f },
                                       Rasp = 0.35f, Bright = 0.9f, Turbo = 0f, Wind = 0.6f };
                case CarStyle.Rx7:
                    return new Voice { Pulses = 2, Idle = 900, Red = 8000, Gears = new[] { 0.27f, 0.42f, 0.58f, 0.75f, 1f },
                                       Rasp = 0.28f, Bright = 0.6f, Turbo = 1f, Wind = 0.7f };
                default:   // Roadster (と従来ボディ)
                    return new Voice { Pulses = 2, Idle = 750, Red = 7500, Gears = new[] { 0.25f, 0.39f, 0.54f, 0.69f, 0.84f, 1f },
                                       Rasp = 0.06f, Bright = 0.35f, Turbo = 0f, Wind = 1.3f };
            }
        }

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
        float m_Lp, m_Road, m_Wind1, m_Wind2, m_SqY1, m_SqY2, m_TurboPh, m_Smooth;
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
            float sr = m_SampleRate > 0 ? m_SampleRate : 48000f;
            var vc = m_Voice;
            float rpm = m_Rpm, thr = m_Thr, frac = m_SpeedFrac, slip = m_Slip, gain = m_Gain;
            float f0 = rpm / 60f * vc.Pulses;                       // 燃焼の基本周波数 [Hz]
            float cut = Mathf.Lerp(500f, 4000f, Mathf.Clamp01(0.35f * thr + 0.65f * rpm / vc.Red)) * (0.6f + 0.6f * vc.Bright);
            float aLp = 1f - Mathf.Exp(-2f * Mathf.PI * cut / sr);
            float engAmp = (0.10f + 0.32f * thr) * (0.6f + 0.4f * rpm / vc.Red);
            float roadAmp = 0.16f * Mathf.Min(1f, frac * 1.4f);
            float windAmp = 0.08f * vc.Wind * frac * frac;
            float sqAmp = 0.10f * slip;
            float turboAmp = 0.02f * vc.Turbo * thr * rpm / vc.Red;
            float turboF = 2200f + rpm * 0.55f;
            // スキールの共振 (1.15 kHz、Q 高め)
            float w0 = 2f * Mathf.PI * 1150f / sr, r = 0.995f;
            float b1 = 2f * r * Mathf.Cos(w0), b2 = -r * r;
            float aRoad = 1f - Mathf.Exp(-2f * Mathf.PI * 380f / sr);
            float aW1 = 1f - Mathf.Exp(-2f * Mathf.PI * 2500f / sr), aW2 = 1f - Mathf.Exp(-2f * Mathf.PI * 500f / sr);

            for (int i = 0; i < data.Length; i += channels)
            {
                m_Phase += f0 / sr;
                if (m_Phase > 1e6) m_Phase -= 1e6;
                float ph = (float)(m_Phase - System.Math.Floor(m_Phase));
                float tw = 2f * Mathf.PI * ph;
                // 倍音の重ね合わせ (Bright で高次を強く)。直列 4 気筒は半次のゆらぎを少し足す
                float e = Mathf.Sin(tw) + 0.6f * vc.Bright * Mathf.Sin(2f * tw + 0.3f)
                        + 0.45f * vc.Bright * Mathf.Sin(3f * tw + 0.9f) + 0.25f * vc.Bright * Mathf.Sin(5f * tw);
                if (vc.Pulses == 2 && vc.Rasp < 0.1f) e += 0.25f * Mathf.Sin(0.5f * tw);
                // 燃焼ノイズ: 各燃焼の直後だけ雑音を入れる (ロータリーのざらつき)
                float burst = Mathf.Exp(-ph * 9f);
                e += vc.Rasp * (0.6f + thr) * burst * Noise();
                m_Lp += (e - m_Lp) * aLp;
                float eng = (float)System.Math.Tanh(1.6f * m_Lp) * engAmp;

                float n = Noise();
                m_Road += (n - m_Road) * aRoad;
                m_Wind1 += (n - m_Wind1) * aW1; m_Wind2 += (m_Wind1 - m_Wind2) * aW2;   // 帯域 500〜2500 Hz
                float wind = (m_Wind1 - m_Wind2) * 2f;
                float sq = n * 0.01f + b1 * m_SqY1 + b2 * m_SqY2; m_SqY2 = m_SqY1; m_SqY1 = sq;
                m_TurboPh += turboF / sr; if (m_TurboPh > 1f) m_TurboPh -= 1f;
                float turbo = Mathf.Sin(2f * Mathf.PI * m_TurboPh) * turboAmp;

                float y = eng + m_Road * roadAmp * 3f + wind * windAmp + sq * sqAmp + turbo;
                // 音量の急変を避ける
                m_Smooth += (gain - m_Smooth) * 0.0005f;
                y *= m_Smooth;
                for (int c = 0; c < channels; c++) data[i + c] = y;
            }
        }
    }
}
