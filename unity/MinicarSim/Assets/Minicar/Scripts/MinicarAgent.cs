// ML-Agents のエージェント。物理は vehicle_sim のまま (設計 P3)。
//
//   Unity (この Agent) ──/mlagents/action──▶ tools/mlagents/mlagents_gateway.py ──/sim/step×2, /sim/reset──▶ vehicle_sim (lockstep)
//                      ◀──/mlagents/obs──── (報酬・終端・IMU を StepInfo から組んで返す)
//
// 1 判断 = /sim/step 2 回 = 1/15 s (カメラ 15 Hz に合わせる)。判断ごとに sim は止まって待つので、
// Academy の自動ステップを止め、gateway の返事が来たら次の EnvironmentStep を回す。
//
// 観測: 車載カメラ画像 (SimBridge が配信しているものを縮小。-mlres 既定 96) ＋ IMU 6 値 ＋ 直前の行動 2 値
//       (真の車速・位置は入れない。実機で取れないため)
// 行動: 連続 2 値 [舵 -1〜1 (左正), 速度 -1〜1]。gateway が cmd_shaper と同じ制限をかけて ActuatorCmd にする
//
// 起動引数:
//   -mlagents             これを付けたときだけ作る
//   -mlres 96             画像観測の一辺 [px]
//   -demo <名前>          人の運転を .demo に記録 (HeuristicOnly)。-demodir <dir> (既定 ~/jetracer_demos)
//   -heuristic            記録せず人が運転するだけ (学習器なし)
//   -spawn random|start   エピソード開始位置 (既定: 学習は random、人の運転は start)
//   -maxsteps 1800        1 エピソードの判断回数の上限 (15 Hz で 2 分)
using System;
using System.IO;
using RosMessageTypes.Std;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Demonstrations;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Minicar
{
    public class MinicarAgent : Agent
    {
        public const string BehaviorName = "MinicarDriver";
        const string kActionTopic = "/mlagents/action";
        const string kObsTopic = "/mlagents/obs";
        const int kVecObs = 8;
        const float kHumanPeriod = 1f / 15f;     // 人が運転するときは実時間 15 Hz で進める
        const float kResendAfter = 3f;           // gateway の返事が来なければ同じ要求を送り直す [s]

        // /mlagents/obs の並び (gateway と同じ)
        enum O { Seq, Kind, Reward, Terminated, Truncated, LapDone, Progress, Cte, Speed, Ax, Ay, Az, Gx, Gy, Gz, Collision, OffTrack, Count }

        enum Phase { NeedReset, WaitReset, Ready, WaitStep }

        SimBridge m_Bridge;
        ROSConnection m_Ros;
        TeleopInput m_Teleop;
        RenderTexture m_Small;
        DemonstrationRecorder m_Recorder;
        bool m_Human;
        string m_Spawn;
        Phase m_Phase = Phase.NeedReset;
        double m_Seq;
        double[] m_Pending;           // 返事待ちの要求 (送り直し用)
        float m_SentAt, m_NextHuman;
        bool m_DiscardStep;           // エピソードが切れた直後に届く、前のエピソードの step の返事を捨てる
        readonly float[] m_Imu = new float[6];
        float m_PrevSteer, m_PrevSpeed;
        uint m_Episode;
        float m_EpProgress, m_EpReturn;
        int m_EpLaps, m_EpSteps;
        string m_Status = "waiting gateway";

        public static MinicarAgent Create(SimBridge bridge, Func<string, string, string> arg)
        {
            var args = Environment.GetCommandLineArgs();
            string demo = arg("-demo", "");
            bool human = demo != "" || Array.IndexOf(args, "-heuristic") >= 0;
            int res = Mathf.Clamp(int.Parse(arg("-mlres", "96")), 36, 224);

            var go = new GameObject("MinicarAgent");
            go.SetActive(false);                      // 部品をそろえてから有効にする (Agent の初期化は OnEnable)

            var small = new RenderTexture(res, res, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { name = "AgentObsRT" };
            small.Create();

            var bp = go.AddComponent<BehaviorParameters>();
            bp.BehaviorName = BehaviorName;
            bp.BrainParameters.VectorObservationSize = kVecObs;
            bp.BrainParameters.NumStackedVectorObservations = 3;
            bp.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(2);
            // 学習器が繋がっていれば学習、繋がっていなければ Heuristic (人の運転) になる
            bp.BehaviorType = human ? BehaviorType.HeuristicOnly : BehaviorType.Default;

            var cam = go.AddComponent<RenderTextureSensorComponent>();
            cam.RenderTexture = small;
            cam.SensorName = "OnboardCamera";
            cam.Grayscale = false;
            cam.CompressionType = SensorCompressionType.PNG;

            var agent = go.AddComponent<MinicarAgent>();
            agent.m_Bridge = bridge;
            agent.m_Ros = bridge.Ros;
            agent.m_Small = small;
            agent.m_Human = human;
            agent.m_Spawn = arg("-spawn", human ? "start" : "random");
            agent.MaxStep = int.Parse(arg("-maxsteps", "1800"));

            if (demo != "")
            {
                string dir = arg("-demodir", Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "jetracer_demos"));
                Directory.CreateDirectory(dir);
                var rec = go.AddComponent<DemonstrationRecorder>();
                rec.DemonstrationName = demo;
                rec.DemonstrationDirectory = dir;
                rec.NumStepsToRecord = 0;             // 0 = 終了するまで
                rec.Record = true;
                agent.m_Recorder = rec;
                Debug.Log($"[MinicarAgent] 人の運転を記録: {dir}/{demo}.demo (終了時に書き出し)");
            }

            // 判断のたびに sim を進めるので、自動ステップは止める
            Academy.Instance.AutomaticSteppingEnabled = false;
            go.SetActive(true);
            return agent;
        }

        public override void Initialize()
        {
            m_Ros.RegisterPublisher<Float64MultiArrayMsg>(kActionTopic);
            m_Ros.Subscribe<Float64MultiArrayMsg>(kObsTopic, OnObs);
            m_Teleop = new TeleopInput(m_Ros);
        }

        // ------------------------------------------------------------------
        // 進行: gateway の返事が来るたびに 1 回ずつ EnvironmentStep を回す
        void Update()
        {
            if (m_Phase == Phase.NeedReset)
            {
                SendRequest(1, 0f, 0f);
                m_Phase = Phase.WaitReset;
                return;
            }
            if (m_Phase == Phase.WaitReset || m_Phase == Phase.WaitStep)
            {
                if (Time.unscaledTime - m_SentAt > kResendAfter && m_Pending != null)
                {
                    Debug.LogWarning("[MinicarAgent] gateway の返事が無いので送り直す (mlagents_gateway.py と sim_mode:=lockstep を確認)");
                    m_Status = "waiting gateway (resend)";
                    Publish(m_Pending);
                }
                return;
            }
            // Ready
            bool humanPaced = m_Human || !Academy.Instance.IsCommunicatorOn;
            if (humanPaced)
            {
                if (Time.unscaledTime < m_NextHuman) return;
                m_NextHuman = Mathf.Max(m_NextHuman + kHumanPeriod, Time.unscaledTime - kHumanPeriod);
            }
            // 画像観測: 配信中のセンサ画像 (この姿勢で描いたもの) を縮小して渡す
            var src = m_Bridge.SensorTexture;
            if (src != null) Graphics.Blit(src, m_Small);
            RequestDecision();
            Academy.Instance.EnvironmentStep();
        }

        public override void OnEpisodeBegin()
        {
            // 判断の途中で切れた (MaxStep) ときは、その step の返事を捨ててから reset する
            if (m_Phase == Phase.WaitStep) m_DiscardStep = true;
            m_Phase = m_DiscardStep ? Phase.WaitStep : Phase.NeedReset;
            m_PrevSteer = m_PrevSpeed = 0f;
            Array.Clear(m_Imu, 0, m_Imu.Length);
            m_EpProgress = m_EpReturn = 0f;
            m_EpLaps = m_EpSteps = 0;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // IMU: 加速度は g 単位 (z は重力を引く)、角速度は π rad/s で割る
            sensor.AddObservation(m_Imu[0] / 9.81f);
            sensor.AddObservation(m_Imu[1] / 9.81f);
            sensor.AddObservation(m_Imu[2] / 9.81f - 1f);
            sensor.AddObservation(m_Imu[3] / Mathf.PI);
            sensor.AddObservation(m_Imu[4] / Mathf.PI);
            sensor.AddObservation(m_Imu[5] / Mathf.PI);
            sensor.AddObservation(m_PrevSteer);
            sensor.AddObservation(m_PrevSpeed);
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            var a = actions.ContinuousActions;
            float steer = Mathf.Clamp(a[0], -1f, 1f), speed = Mathf.Clamp(a[1], -1f, 1f);
            m_PrevSteer = steer; m_PrevSpeed = speed;
            SendRequest(0, steer, speed);
            m_Phase = Phase.WaitStep;
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            m_Teleop.Read(kHumanPeriod, out float steer, out float thr);
            var c = actionsOut.ContinuousActions;
            c[0] = steer;
            // 速度の行動は -1 = 停止 〜 +1 = v_max。アクセル 0 で停止、ブレーキも停止
            c[1] = Mathf.Clamp(thr, 0f, 1f) * 2f - 1f;
        }

        // ------------------------------------------------------------------
        void SendRequest(int kind, float steer, float speed)
        {
            m_Seq += 1;
            if (kind == 1) m_Episode++;
            // [seq, kind (0 = step / 1 = reset), steer, speed, seed, spawn (0 = start / 1 = random)]
            m_Pending = new double[] { m_Seq, kind, steer, speed, m_Episode, m_Spawn == "random" ? 1 : 0 };
            Publish(m_Pending);
        }

        void Publish(double[] d)
        {
            m_SentAt = Time.unscaledTime;
            m_Ros.Publish(kActionTopic, new Float64MultiArrayMsg { data = d });
        }

        void OnObs(Float64MultiArrayMsg msg)
        {
            var d = msg.data;
            if (d == null || d.Length < (int)O.Count) return;
            bool isReset = d[(int)O.Kind] > 0.5;
            if (m_DiscardStep && !isReset)
            {
                // 前のエピソードの step の返事。捨てて reset を出す
                m_DiscardStep = false;
                m_Phase = Phase.NeedReset;
                return;
            }
            if (m_Pending == null || Math.Abs(d[(int)O.Seq] - m_Pending[0]) > 0.5) return;   // 古い返事
            m_Pending = null;
            for (int i = 0; i < 6; i++) m_Imu[i] = (float)d[(int)O.Ax + i];
            if (isReset)
            {
                m_Phase = Phase.Ready;
                m_Status = m_Human ? "human driving" : (Academy.Instance.IsCommunicatorOn ? "training" : "heuristic (no trainer)");
                m_NextHuman = Time.unscaledTime;
                return;
            }
            float r = (float)d[(int)O.Reward];
            AddReward(r);
            m_EpReturn += r;
            m_EpProgress += (float)d[(int)O.Progress];
            if (d[(int)O.LapDone] > 0.5) m_EpLaps++;
            m_EpSteps++;
            m_Phase = Phase.Ready;
            // 人の運転中に記録ボタン (プロコンの +) が押されたら、そのエピソードを区切る
            bool cut = m_Human && m_Teleop.TakeRecordPresses() > 0;
            if (d[(int)O.Terminated] > 0.5 || cut)
            {
                var stats = Academy.Instance.StatsRecorder;
                stats.Add("minicar/progress_m", m_EpProgress);
                stats.Add("minicar/laps", m_EpLaps);
                stats.Add("minicar/crash", d[(int)O.Collision] > 0.5 || d[(int)O.OffTrack] > 0.5 ? 1f : 0f);
                EndEpisode();       // → OnEpisodeBegin → reset
            }
        }

        void OnGUI()
        {
            var style = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            style.normal.textColor = Color.white;
            string pad = m_Teleop != null && m_Teleop.PadActive ? "pad" : "keyboard";
            string rec = m_Recorder != null ? "  ● REC" : "";
            GUI.Label(new Rect(12, Screen.height - 52, 900, 22),
                $"ML-Agents: {m_Status}{rec}   ep {m_Episode}  step {m_EpSteps}  progress {m_EpProgress:F1} m  laps {m_EpLaps}  return {m_EpReturn:F1}  input {pad}", style);
        }
    }
}
