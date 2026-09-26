// ROS (x 前/東, y 左/北, z 上, 右手系) ↔ Unity (x 右, y 上, z 前, 左手系)。
// 座標変換はここにしか書かない。
using UnityEngine;

namespace Minicar
{
    public static class RosFrame
    {
        public static Vector3 ToUnity(float x, float y, float z) => new Vector3(-y, z, x);

        public static Vector3 ToUnity(double x, double y, double z) =>
            new Vector3((float)-y, (float)z, (float)x);

        // ROS のヨー (z 軸まわり反時計回り) → Unity の y 軸まわり回転。
        // ROS の前方 (cosψ, sinψ) は Unity で (-sinψ, 0, cosψ) = Euler(0, -ψ, 0) の前方。
        public static Quaternion Yaw(double yawRad) =>
            Quaternion.Euler(0f, (float)(-yawRad * Mathf.Rad2Deg), 0f);

        // ヨーに下向きピッチ (正で下向き) を足した姿勢
        public static Quaternion YawPitch(double yawRad, double pitchDownRad) =>
            Quaternion.Euler((float)(pitchDownRad * Mathf.Rad2Deg),
                             (float)(-yawRad * Mathf.Rad2Deg), 0f);
    }
}
