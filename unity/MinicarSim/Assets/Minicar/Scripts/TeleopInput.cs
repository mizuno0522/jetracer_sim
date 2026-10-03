// 人の運転入力。プロコン (Nintendo Switch Pro Controller) などのゲームパッドは ROS の joy ノードが読み、
// tools/teleop/joy_teleop.py が割り当て (どのスティック・ボタンが何か) を 1 か所で解いて
// /teleop/cmd (std_msgs/Float64MultiArray) に正規化した値を出す。ここはそれを受けるだけ。
// /teleop/cmd が 0.5 s 来ていなければキーボード (←→ / A D でハンドル、↑ / W でアクセル、↓ / S でブレーキ) を使う。
//
// /teleop/cmd の並び: [steer (-1〜1, 左が正), throttle (-1〜1, 負はブレーキ), 記録ボタンの押下回数, 非常停止 (0/1)]
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Minicar
{
    public class TeleopInput
    {
        public const string Topic = "/teleop/cmd";
        float m_Steer, m_Thr, m_LastMsg = -10f, m_KeySteer;
        int m_RecCount = -1, m_RecEdges;
        bool m_EStop, m_KeysOk = true;

        public TeleopInput(ROSConnection ros)
        {
            ros.Subscribe<Float64MultiArrayMsg>(Topic, msg =>
            {
                var d = msg.data;
                if (d == null || d.Length < 2) return;
                m_Steer = Mathf.Clamp((float)d[0], -1f, 1f);
                m_Thr = Mathf.Clamp((float)d[1], -1f, 1f);
                if (d.Length >= 3)
                {
                    int c = (int)d[2];
                    if (m_RecCount >= 0 && c != m_RecCount) m_RecEdges++;
                    m_RecCount = c;
                }
                m_EStop = d.Length >= 4 && d[3] > 0.5;
                m_LastMsg = Time.unscaledTime;
            });
        }

        /// ゲームパッドの値が新しいか (来ていなければキーボード)
        public bool PadActive => Time.unscaledTime - m_LastMsg < 0.5f;
        public bool EStop => PadActive && m_EStop;

        /// 記録ボタンが押された回数 (前回の呼び出しから)。押されたら 1 以上
        public int TakeRecordPresses() { int n = m_RecEdges; m_RecEdges = 0; return n; }

        /// steer (-1〜1, 左正)・throttle (-1〜1, 負はブレーキ)
        public void Read(float dt, out float steer, out float throttle)
        {
            if (PadActive)
            {
                steer = m_EStop ? 0f : m_Steer;
                throttle = m_EStop ? -1f : m_Thr;
                return;
            }
            float want = 0f, thr = 0f;
            if (m_KeysOk)
            {
                try
                {
                    if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A)) want += 1f;
                    if (Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D)) want -= 1f;
                    if (Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.W)) thr += 1f;
                    if (Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S)) thr -= 1f;
                }
                catch (System.InvalidOperationException) { m_KeysOk = false; }
            }
            // キーは 0/1 なので、ハンドルだけ一次遅れで滑らかに (0.15 s)
            m_KeySteer += (want - m_KeySteer) * (1f - Mathf.Exp(-dt / 0.15f));
            steer = m_KeySteer;
            throttle = thr;
        }
    }
}
