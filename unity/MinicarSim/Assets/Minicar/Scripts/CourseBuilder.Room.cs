// ミニカーの会場を、実際にコースを組んだ部屋らしく見せる (表示・動画用)。参考: 会場の動画 (2026-10-05 水野)。
//
//   -venue room | plain   既定: HDRP 版 (unity/MinicarSimHDRP) は room、Built-in 版は plain
//
// plain = 今までの会場 (灰色の床・カプセルの観戦者)。学習・検出器の評価に使う Built-in 版のセンサ画像は変えない。
// room  = 窓のある会議室: クリーム色の壁、アルミサッシの窓、蛍光灯の並ぶ低い天井、濃いグレーのパンチカーペット、
//         窓際の長椅子、巻いたカーペット、重ねた椅子。コースの床の区域 (人工芝・滑り板・風呂マット・坂の板) と
//         壁板も実物の質感に寄せる。コースの形・寸法・色の区別 (白 / 赤の壁、駐車枠の色) は plain と同じ。
// 寸法はすべて ROS 座標 [m] (x = コースの長手、y = 奥行き、z = 上)。コースは x 0〜10.3、y 0〜6.1。窓は +y の壁。
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public partial class CourseBuilder
    {
        public bool Room { get; private set; }
        // 窓と蛍光灯のある明るい部屋なので、plain (検出器の明るさに合わせた控えめな光) より明るくする
        float RoomLight => Room ? 1.9f : 1f;
        float RoomAmbient => Room ? 2.1f : 1f;

        // 部屋の内のり
        const float kRx0 = -3.2f, kRx1 = 13.0f, kRy0 = -4.6f, kRy1 = 8.4f, kRz = 2.7f;
        Transform m_RoomRoot;

        static bool WantRoom()
        {
#if MINICAR_HDRP
            const string def = "room";
#else
            const string def = "plain";
#endif
            return SimBridge.Arg("-venue", def).ToLowerInvariant() == "room";
        }

        // ------------------------------------------------------------------ 部品
        // ROS 座標の箱 (sx・sy・sz = x・y・z 方向の長さ、yawDeg = z 軸まわり)
        GameObject RBox(string name, float cx, float cy, float cz, float sx, float sy, float sz, Material mat, float yawDeg = 0f, bool shadow = true)
        {
            var go = Box(name, RosFrame.ToUnity(cx, cy, cz), new Vector3(sy, sz, sx), RosFrame.Yaw(yawDeg * Mathf.Deg2Rad), mat);
            go.transform.SetParent(m_RoomRoot, true);
            go.layer = RvizLayout.SensorOnlyLayer;
            if (!shadow) go.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            return go;
        }

        // 円柱 (axis = 'x'・'y'・'z' のどれに沿うか)
        GameObject RCyl(string name, float cx, float cy, float cz, char axis, float len, float dia, Material mat, bool shadow = true)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            Destroy(go.GetComponent<Collider>());
            go.name = name;
            go.transform.SetParent(m_RoomRoot, false);
            go.transform.position = RosFrame.ToUnity(cx, cy, cz);
            go.transform.rotation = axis == 'z' ? Quaternion.identity : axis == 'x' ? Quaternion.Euler(90f, 0f, 0f) : Quaternion.Euler(0f, 0f, 90f);
            go.transform.localScale = new Vector3(dia, len * 0.5f, dia);
            go.GetComponent<Renderer>().sharedMaterial = mat;
            go.layer = RvizLayout.SensorOnlyLayer;
            if (!shadow) go.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            return go;
        }

        // 片面の板 (4 隅は ROS 座標。normal の側からだけ見える)。部屋の外に出たカメラからは透ける
        GameObject RQuad(string name, Vector3 tl, Vector3 tr, Vector3 br, Vector3 bl, Vector3 normal, Material mat, int layer)
        {
            Vector3 U(Vector3 p) => RosFrame.ToUnity(p.x, p.y, p.z);
            var go = MeshObject(name, QuadMesh(U(tl), U(tr), U(br), U(bl), U(normal)), mat);
            go.transform.SetParent(m_RoomRoot, true);
            go.layer = layer;
            go.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            return go;
        }

        // 鉛直の壁 (a → b、高さ z0〜z1、inward の側から見える)
        GameObject WallQuad(string name, float ax, float ay, float bx, float by, float z0, float z1, Vector3 inward, Material mat) =>
            RQuad(name, new Vector3(ax, ay, z1), new Vector3(bx, by, z1), new Vector3(bx, by, z0), new Vector3(ax, ay, z0), inward, mat, RvizLayout.SensorOnlyLayer);

        Material LitN(Texture2D albedo, Texture2D normal, float smooth, float bump, Vector2 tiling)
        {
            var m = Lit(new Color32(255, 255, 255, 255), albedo, smooth, tiling);
            if (normal != null)
            {
                m.SetTexture("_BumpMap", normal);
                m.SetTextureScale("_BumpMap", tiling);
                m.SetFloat("_BumpScale", bump);
                m.EnableKeyword("_NORMALMAP");
            }
            return m;
        }

        // 光の影響を受けずにそのままの色で出る材質 (蛍光灯・窓の外・画面)
        Material Flat(Color32 c) => new Material(m_Unlit) { mainTexture = VenueTex.Solid(c) };
        Material Flat(Texture2D t) => new Material(m_Unlit) { mainTexture = t };

        // ------------------------------------------------------------------ コースの質感 (room のときだけ)
        Material m_RoomCarpet;

        Material RoomCarpet(float sizeA, float sizeB)
        {
            VenueTex.Carpet(512, 41, out var a, out var n);
            return LitN(a, n, 0.03f, 1.0f, new Vector2(sizeA / 0.8f, sizeB / 0.8f));
        }

        Material RoomArea(string name, AreaData a)
        {
            float lx = a.x1 - a.x0, ly = a.y1 - a.y0;
            switch (name)
            {
                case "MU_HIGH":                              // 高 μ 路: 人工芝 (芝丈 30 mm・フレッシュグリーン。規約 解説④)
                {
                    VenueTex.Turf(256, 42, out var t, out var n);
                    return LitN(t, n, 0.02f, 1.4f, new Vector2(ly / 0.5f, lx / 0.5f));
                }
                case "ROUGH":                                // でこぼこ道: 水色の風呂すべり止めマット (ストーン柄・40×80 cm) を 4 枚 (規約 解説⑦)
                {
                    VenueTex.BathMat(256, 512, 49, out var t, out var n);
                    return LitN(t, n, 0.50f, 1.8f, new Vector2(Mathf.Max(1f, Mathf.Round(ly / 0.8f)), Mathf.Max(1f, Mathf.Round(lx / 0.4f))));
                }
                case "MU_LOW":                               // 低 μ 路: 白い PTFE シート。つやがあって照明が映る
                {
                    VenueTex.Board(128, 43, new Color32(238, 238, 234, 255), 0.012f, out var t, out var n);
                    return LitN(t, n, 0.88f, 0.15f, new Vector2(2f, 2f));
                }
                default:                                     // 坂道の白い板 (つや消し)
                {
                    VenueTex.Board(128, 44, new Color32(226, 224, 216, 255), 0.03f, out var t, out var n);
                    return LitN(t, n, 0.35f, 0.3f, new Vector2(2f, 2f));
                }
            }
        }

        // 区域を「置いてある物」にする: 厚みのある板 + 縁のテープ
        void RoomAreaSlab(string name, AreaData a, float z)
        {
            float thick = name == "MU_HIGH" ? 0.010f : name == "ROUGH" ? 0.012f : 0.004f;
            if (name == "MU_LOW") z += 0.008f;           // 低 μ 路の板は芝を切り抜いた所に置いてある (芝の上端より少し上に見せる)
            float cx = (a.x0 + a.x1) * 0.5f, cy = (a.y0 + a.y1) * 0.5f, lx = a.x1 - a.x0, ly = a.y1 - a.y0;
            var go = Box(name, RosFrame.ToUnity(cx, cy, z + thick * 0.5f), new Vector3(ly, thick, lx), Quaternion.identity, RoomArea(name, a));
            go.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            if (name == "SLOPE") return;
            // 縁を留めるテープ: 芝と低 μ 路は緑、風呂マットは青 (マットどうしの継ぎ目にも貼る)
            var tape = Lit(name == "ROUGH" ? new Color32(30, 84, 200, 255) : new Color32(24, 150, 96, 255), null, 0.55f);
            const float w = 0.05f;
            float zt = z + thick + 0.0006f;
            if (name == "MU_HIGH") zt = z + 0.0125f;
            FloorRect(name + "TapeS", a.x0 - w * 0.5f, a.y0 - w * 0.5f, a.x1 + w * 0.5f, a.y0 + w * 0.5f, zt, tape);
            FloorRect(name + "TapeN", a.x0 - w * 0.5f, a.y1 - w * 0.5f, a.x1 + w * 0.5f, a.y1 + w * 0.5f, zt, tape);
            FloorRect(name + "TapeW", a.x0 - w * 0.5f, a.y0, a.x0 + w * 0.5f, a.y1, zt, tape);
            FloorRect(name + "TapeE", a.x1 - w * 0.5f, a.y0, a.x1 + w * 0.5f, a.y1, zt, tape);
            if (name == "ROUGH")
            {
                FloorRect("RoughSeamX", cx - w * 0.5f, a.y0, cx + w * 0.5f, a.y1, zt, tape);
                FloorRect("RoughSeamY", a.x0, cy - w * 0.5f, a.x1, cy + w * 0.5f, zt, tape);
            }
        }

        // 駐車枠の番号 (room): 枠と同じ色のテープを文字の形に貼ってある (規約 解説⑧)。LabelTexture の点の並びを 1 点ずつテープの片にする
        void RoomFloorLabel(SlotData s, Color32 col, float w)
        {
            var tex = LabelTexture.Make(s.name, new Color32(255, 255, 255, 255), new Color32(0, 0, 0, 255));
            const int cell = 8;
            int cols = tex.width / cell, rows = tex.height / cell;
            float h = w * tex.height / tex.width, sx = w / cols, sy = h / rows;
            float cx = (s.x0 + s.x1) * 0.5f, cy = (s.y0 + s.y1) * 0.5f;
            var px = tex.GetPixels32();
            var tape = Lit(col, null, 0.5f);
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                {
                    if (px[(r * cell + cell / 2) * tex.width + c * cell + cell / 2].r < 128) continue;
                    int c1 = c;
                    while (c1 + 1 < cols && px[(r * cell + cell / 2) * tex.width + (c1 + 1) * cell + cell / 2].r >= 128) c1++;
                    // テクスチャの u は -x 向き、v は -y 向き (FloorLabel と同じ向き)
                    float xa = cx + w * 0.5f - (c1 + 1) * sx, xb = cx + w * 0.5f - c * sx;
                    float ya = cy + h * 0.5f - (r + 1) * sy, yb = cy + h * 0.5f - r * sy;
                    FloorRect($"{s.name}_label", xa, ya, xb, yb, 0.0062f, tape).GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    c = c1;
                }
            Destroy(tex);
        }

        // 壁板の足 (規約「その他使用部材詳細」): 鉄板 150×150×4 mm に M20 のボルトを立て、塩ビ管 (VP25) で板を床から 30 mm 浮かせる
        void BuildWallFeet()
        {
            var steel = Lit(new Color32(96, 98, 102, 255), Speckle(64, 0.10f, 51), 0.45f);
            steel.SetFloat("_Metallic", 0.7f);
            var bolt = Lit(new Color32(176, 178, 182, 255), null, 0.7f);
            bolt.SetFloat("_Metallic", 0.9f);
            var pvc = Lit(new Color32(150, 152, 156, 255), null, 0.35f);
            var seen = new HashSet<long>();
            float top = Data.wall_base_m + Data.wall_height_m;
            var walls = new List<WallData>(Data.walls);
            walls.Sort((p, q) => p.divider.CompareTo(q.divider));       // 端点を共有するときは常設の壁の足にする (仕切りは最後)
            foreach (var w in walls)
            {
                Vector2 a = new Vector2(w.x0, w.y0), b = new Vector2(w.x1, w.y1);
                if ((b - a).sqrMagnitude < 1e-4f) continue;
                Vector2 dir = (b - a).normalized, nrm = new Vector2(-dir.y, dir.x);
                float yaw = Mathf.Atan2(dir.y, dir.x);
                foreach (var p in new[] { a, b })
                {
                    long key = (long)Mathf.RoundToInt(p.x * 20f) * 100000 + Mathf.RoundToInt(p.y * 20f);
                    if (!seen.Add(key)) continue;
                    Vector2 q = p + nrm * 0.028f;                 // ボルトは板の横に立ち、バンドで板を抱える
                    // ⑤狭い道の仕切りは出し入れされるので、その足は仕切りと一緒に出入りさせる
                    Transform owner = w.divider && Divider != null ? Divider.transform : m_Root;
                    var plate = Box("WallFootPlate", RosFrame.ToUnity(q.x, q.y, 0.002f), new Vector3(0.15f, 0.004f, 0.15f), RosFrame.Yaw(yaw), steel);
                    plate.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    plate.transform.SetParent(owner, true);
                    foreach (var (h, dia, mat, z0) in new[] { (top + 0.012f, 0.020f, bolt, 0.004f), (0.050f, 0.031f, pvc, 0.004f), (0.013f, 0.034f, bolt, 0.004f) })
                    {
                        var c = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                        Destroy(c.GetComponent<Collider>());
                        c.name = "WallFootBolt";
                        c.transform.position = RosFrame.ToUnity(q.x, q.y, z0 + h * 0.5f);
                        c.transform.localScale = new Vector3(dia, h * 0.5f, dia);
                        c.transform.SetParent(owner, true);
                        c.GetComponent<Renderer>().sharedMaterial = mat;
                    }
                }
            }
        }

        // ------------------------------------------------------------------ 部屋
        void BuildRoom()
        {
            m_RoomRoot = new GameObject("Room").transform;
            m_RoomRoot.SetParent(m_Root, false);
            int S = RvizLayout.SensorOnlyLayer, O = RvizLayout.OverheadLayer;
            float lx = kRx1 - kRx0, ly = kRy1 - kRy0;
            Vector3 up = new Vector3(0, 0, 1);

            // ---- 床: ベージュの長尺シート。その上に、コースより広くパンチカーペットを敷いてある
            VenueTex.Board(128, 45, new Color32(196, 172, 134, 255), 0.05f, out var vinA, out var vinN);
            var vinyl = LitN(vinA, vinN, 0.55f, 0.2f, new Vector2(ly / 2f, lx / 2f));
            RQuad("RoomFloor", new Vector3(kRx0, kRy1, -0.012f), new Vector3(kRx1, kRy1, -0.012f), new Vector3(kRx1, kRy0, -0.012f), new Vector3(kRx0, kRy0, -0.012f), up, vinyl, S);
            float cx0 = -1.25f, cx1 = kRx1 - 0.02f, cy0 = -2.1f, cy1 = kRy1 - 0.02f;
            RQuad("RoomCarpet", new Vector3(cx0, cy1, -0.004f), new Vector3(cx1, cy1, -0.004f), new Vector3(cx1, cy0, -0.004f), new Vector3(cx0, cy0, -0.004f), up,
                  RoomCarpet(cy1 - cy0, cx1 - cx0), S);

            // ---- 壁: クリーム色の塗り壁と濃い茶の巾木
            var cream = Lit(new Color32(232, 224, 200, 255), Speckle(128, 0.035f, 31), 0.12f, new Vector2(6, 2));
            var skirt = Lit(new Color32(74, 58, 46, 255), null, 0.3f);
            var sash = Lit(new Color32(168, 172, 176, 255), null, 0.6f);
            void Skirt(float ax, float ay, float bx, float by, Vector3 inward)
            {
                Vector3 m = new Vector3((ax + bx) * 0.5f, (ay + by) * 0.5f, 0f) + inward * 0.006f;
                bool alongX = Mathf.Abs(bx - ax) > Mathf.Abs(by - ay);
                float len = Mathf.Abs(alongX ? bx - ax : by - ay);
                RBox("Skirting", m.x, m.y, 0.04f, alongX ? len : 0.012f, alongX ? 0.012f : len, 0.08f, skirt, 0f, false);
            }
            WallQuad("WallS", kRx0, kRy0, kRx1, kRy0, 0f, kRz, new Vector3(0, 1, 0), cream); Skirt(kRx0, kRy0, kRx1, kRy0, new Vector3(0, 1, 0));
            WallQuad("WallW", kRx0, kRy1, kRx0, kRy0, 0f, kRz, new Vector3(1, 0, 0), cream); Skirt(kRx0, kRy1, kRx0, kRy0, new Vector3(1, 0, 0));
            WallQuad("WallE", kRx1, kRy0, kRx1, kRy1, 0f, kRz, new Vector3(-1, 0, 0), cream); Skirt(kRx1, kRy0, kRx1, kRy1, new Vector3(-1, 0, 0));

            // ---- 窓の壁 (+y): 腰壁と垂れ壁、柱型、3 連のアルミサッシ。窓の外は明るい屋外
            const float sill = 0.82f, head = 2.18f;
            WallQuad("WallNSill", kRx1, kRy1, kRx0, kRy1, 0f, sill, new Vector3(0, -1, 0), cream);
            WallQuad("WallNHead", kRx1, kRy1, kRx0, kRy1, head, kRz, new Vector3(0, -1, 0), cream);
            Skirt(kRx0, kRy1, kRx1, kRy1, new Vector3(0, -1, 0));
            var outside = Flat(VenueTex.Outside(512, 256, 46));
            float[] wx0 = { -1.7f, 3.3f, 8.3f }, wx1 = { 1.9f, 6.9f, 11.9f };
            float[] px0 = { kRx0, 1.9f, 6.9f, 11.9f }, px1 = { -1.7f, 3.3f, 8.3f, kRx1 };
            for (int i = 0; i < 4; i++)
            {
                float pc = (px0[i] + px1[i]) * 0.5f, pw = px1[i] - px0[i];
                RBox("Pillar", pc, kRy1 - 0.14f, kRz * 0.5f, pw, 0.30f, kRz, cream, 0f, false);
            }
            for (int i = 0; i < 3; i++)
            {
                float a = wx0[i], b = wx1[i], c = (a + b) * 0.5f, w = b - a;
                // 屋外 (窓ごとに絵の別の所を見せる)
                var view = RQuad("WindowView", new Vector3(b, kRy1 + 0.10f, head), new Vector3(a, kRy1 + 0.10f, head), new Vector3(a, kRy1 + 0.10f, sill), new Vector3(b, kRy1 + 0.10f, sill),
                                 new Vector3(0, -1, 0), outside, S);
                var mesh = view.GetComponent<MeshFilter>().sharedMesh;
                float u0 = i * 0.33f;
                mesh.uv = new[] { new Vector2(u0, 1), new Vector2(u0 + 0.33f, 1), new Vector2(u0 + 0.33f, 0), new Vector2(u0, 0) };
                // 窓台と枠、縦の方立て (4 枚の引き違い)、腰の高さの中桟
                RBox("WindowSill", c, kRy1 - 0.05f, sill - 0.015f, w, 0.16f, 0.03f, sash, 0f, false);
                RBox("SashTop", c, kRy1 + 0.02f, head - 0.025f, w, 0.06f, 0.05f, sash, 0f, false);
                RBox("SashBottom", c, kRy1 + 0.02f, sill + 0.025f, w, 0.06f, 0.05f, sash, 0f, false);
                RBox("SashRail", c, kRy1 + 0.02f, sill + 0.42f, w, 0.05f, 0.045f, sash, 0f, false);
                for (int k = 0; k <= 4; k++)
                    RBox("SashMullion", a + k * w / 4f, kRy1 + 0.02f, (sill + head) * 0.5f, k % 4 == 0 ? 0.06f : 0.045f, 0.06f, head - sill, sash, 0f, false);
            }
            // 柱の掛け時計と掲示
            var white = Lit(new Color32(240, 240, 236, 255), null, 0.4f);
            var dark = Lit(new Color32(34, 34, 36, 255), null, 0.4f);
            RCyl("ClockRim", 2.6f, kRy1 - 0.300f, 2.28f, 'y', 0.03f, 0.34f, dark, false);
            RCyl("ClockFace", 2.6f, kRy1 - 0.312f, 2.28f, 'y', 0.03f, 0.30f, white, false);
            RBox("ClockHandH", 2.63f, kRy1 - 0.330f, 2.30f, 0.07f, 0.004f, 0.012f, dark, 0f, false);
            RBox("ClockHandM", 2.6f, kRy1 - 0.330f, 2.33f, 0.010f, 0.004f, 0.11f, dark, 0f, false);
            RBox("Notice", 7.6f, kRy1 - 0.295f, 1.55f, 0.21f, 0.004f, 0.30f, white, 0f, false);
            RBox("Notice", 7.3f, kRy1 - 0.295f, 1.50f, 0.30f, 0.004f, 0.21f, white, 0f, false);
            // 奥の壁 (+x): 額と掲示、出入口
            var woodFrame = Lit(new Color32(120, 84, 52, 255), null, 0.35f);
            RBox("PictureFrame", kRx1 - 0.02f, 3.4f, 2.05f, 0.03f, 0.86f, 0.46f, woodFrame, 0f, false);
            RBox("Picture", kRx1 - 0.037f, 3.4f, 2.05f, 0.004f, 0.76f, 0.36f, Flat(new Color32(214, 206, 184, 255)), 0f, false);
            RBox("Poster", kRx1 - 0.012f, 6.3f, 1.75f, 0.004f, 0.52f, 0.73f, Flat(VenueTex.Poster(47)), 0f, false);
            var doorMat = Lit(new Color32(186, 176, 150, 255), null, 0.35f);
            RBox("Door", kRx1 - 0.02f, -2.6f, 1.03f, 0.04f, 0.86f, 2.06f, doorMat, 0f, false);
            RBox("DoorFrame", kRx1 - 0.015f, -2.6f, 2.10f, 0.05f, 0.98f, 0.07f, skirt, 0f, false);
            RBox("DoorFrame", kRx1 - 0.015f, -3.07f, 1.05f, 0.05f, 0.06f, 2.1f, skirt, 0f, false);
            RBox("DoorFrame", kRx1 - 0.015f, -2.13f, 1.05f, 0.05f, 0.06f, 2.1f, skirt, 0f, false);
            RCyl("DoorKnob", kRx1 - 0.07f, -2.28f, 1.0f, 'x', 0.06f, 0.05f, sash, false);

            // ---- 天井: 白い面に直付けの蛍光灯が列になって並ぶ。窓側に天井付けのエアコンと配管
            var ceil = Lit(new Color32(238, 236, 228, 255), Speckle(128, 0.03f, 32), 0.08f, new Vector2(8, 8));
            RQuad("RoomCeiling", new Vector3(kRx0, kRy0, kRz), new Vector3(kRx1, kRy0, kRz), new Vector3(kRx1, kRy1, kRz), new Vector3(kRx0, kRy1, kRz), -up, ceil, O);
            var tube = Flat(new Color32(255, 253, 244, 255));
            var over = new List<GameObject>();
            for (int r = 0; r < 5; r++)
            {
                float y = kRy0 + 1.3f + r * (ly - 2.6f) / 4f;
                for (int k = 0; k < 6; k++)
                {
                    float x = kRx0 + 1.5f + k * (lx - 3.0f) / 5f;
                    over.Add(RBox("LampBase", x, y, kRz - 0.025f, 1.26f, 0.17f, 0.05f, white, 0f, false));
                    over.Add(RBox("LampTube", x, y - 0.035f, kRz - 0.062f, 1.20f, 0.030f, 0.030f, tube, 0f, false));
                    over.Add(RBox("LampTube", x, y + 0.035f, kRz - 0.062f, 1.20f, 0.030f, 0.030f, tube, 0f, false));
                }
            }
            var acBody = Lit(new Color32(226, 224, 214, 255), null, 0.3f);
            foreach (float x in new[] { 2.6f, 7.6f })
            {
                over.Add(RBox("AirCon", x, kRy1 - 1.05f, kRz - 0.14f, 1.25f, 0.68f, 0.28f, acBody, 0f, false));
                over.Add(RBox("AirConGrille", x, kRy1 - 1.40f, kRz - 0.20f, 1.05f, 0.02f, 0.07f, dark, 0f, false));
            }
            over.Add(RBox("Duct", (kRx0 + kRx1) * 0.5f, kRy1 - 0.36f, kRz - 0.16f, lx, 0.11f, 0.11f, acBody, 0f, false));
            over.Add(RBox("Duct", (kRx0 + kRx1) * 0.5f, kRy1 - 0.36f, kRz - 0.31f, lx, 0.07f, 0.07f, acBody, 0f, false));
            foreach (var g in over) g.layer = O;

            // ---- 備品
            // 窓際の長椅子 (えんじ色の 3 人掛け)
            var seat = Lit(new Color32(112, 40, 44, 255), Speckle(64, 0.06f, 33), 0.35f);
            var metal = Lit(new Color32(30, 30, 32, 255), null, 0.5f);
            void Bench(float x, float y)
            {
                for (int k = -1; k <= 1; k++)
                {
                    RBox("BenchSeat", x + k * 0.55f, y, 0.41f, 0.52f, 0.46f, 0.07f, seat);
                    RBox("BenchBack", x + k * 0.55f, y + 0.25f, 0.70f, 0.52f, 0.06f, 0.36f, seat);
                }
                RBox("BenchBeam", x, y, 0.35f, 1.66f, 0.06f, 0.05f, metal);
                RBox("BenchBeam", x, y + 0.26f, 0.55f, 1.66f, 0.03f, 0.03f, metal);
                foreach (float dx in new[] { -0.72f, 0.72f })
                {
                    RBox("BenchLeg", x + dx, y, 0.17f, 0.04f, 0.04f, 0.33f, metal);
                    RBox("BenchFoot", x + dx, y, 0.015f, 0.05f, 0.50f, 0.03f, metal);
                    RBox("BenchArm", x + dx, y + 0.26f, 0.55f, 0.03f, 0.03f, 0.40f, metal);
                }
            }
            foreach (float x in new[] { -0.75f, 1.0f, 4.2f, 5.95f, 9.2f, 10.95f }) Bench(x, kRy1 - 0.62f);
            // 濃紺のついたて
            RBox("Partition", 5.3f, 7.05f, 0.80f, 1.85f, 0.035f, 0.95f, Lit(new Color32(40, 48, 62, 255), null, 0.3f));
            foreach (float dx in new[] { -0.85f, 0.85f })
            {
                RBox("PartitionLeg", 5.3f + dx, 7.05f, 0.18f, 0.03f, 0.03f, 0.36f, metal);
                RBox("PartitionFoot", 5.3f + dx, 7.05f, 0.015f, 0.04f, 0.42f, 0.03f, metal);
            }
            // 余ったカーペットの巻き (コースの手前側の脇)
            var roll = RoomCarpet(1.2f, 3.0f);
            roll.color = new Color(0.62f, 0.62f, 0.64f);
            RCyl("CarpetRoll", -0.80f, 1.9f, 0.17f, 'y', 2.0f, 0.34f, roll);
            RCyl("CarpetRoll", -0.95f, 4.3f, 0.16f, 'y', 1.9f, 0.32f, roll);
            RCyl("CarpetRoll", -1.25f, -0.4f, 0.19f, 'y', 2.1f, 0.38f, roll);
            // 重ねた白い椅子
            var plastic = Lit(new Color32(236, 236, 232, 255), null, 0.55f);
            var pad = Lit(new Color32(52, 52, 56, 255), null, 0.3f);
            for (int s = 0; s < 4; s++)
            {
                float x = kRx0 + 0.45f, y = -1.2f + s * 0.52f;
                int n = 5 + (s * 3) % 4;
                for (int k = 0; k < n; k++) RBox("ChairStack", x, y, 0.42f + k * 0.085f, 0.44f, 0.42f, 0.035f, plastic);
                RBox("ChairStackPad", x, y, 0.42f + n * 0.085f - 0.04f, 0.36f, 0.34f, 0.03f, pad);
                foreach (float dx in new[] { -0.19f, 0.19f })
                    foreach (float dy in new[] { -0.18f, 0.18f })
                        RBox("ChairStackLeg", x + dx, y + dy, 0.21f, 0.025f, 0.025f, 0.42f, plastic);
            }
            // 運営の机とノート PC (駐車枠の手前)
            var top = Lit(new Color32(222, 214, 196, 255), null, 0.45f);
            void Desk(float x, float y)
            {
                RBox("DeskTop", x, y, 0.70f, 1.80f, 0.60f, 0.03f, top);
                foreach (float dx in new[] { -0.84f, 0.84f })
                    foreach (float dy in new[] { -0.26f, 0.26f })
                        RBox("DeskLeg", x + dx, y + dy, 0.345f, 0.03f, 0.03f, 0.69f, metal);
                RBox("Laptop", x - 0.35f, y, 0.722f, 0.32f, 0.22f, 0.014f, Lit(new Color32(60, 62, 66, 255), null, 0.5f));
                RBox("LaptopLid", x - 0.35f, y - 0.115f, 0.83f, 0.32f, 0.012f, 0.21f, Lit(new Color32(60, 62, 66, 255), null, 0.5f));
                RBox("LaptopScreen", x - 0.35f, y - 0.108f, 0.83f, 0.29f, 0.002f, 0.18f, Flat(VenueTex.Screen(48)), 0f, false);
            }
            Desk(3.0f, -3.4f);
            Desk(7.2f, -3.4f);
        }
    }

    /// 会場 (room) の質感のテクスチャ。実行時に作る (画像ファイルを持たない)。どれも上下左右がつながる
    public static class VenueTex
    {
        delegate void PixelFn(float u, float v, int x, int y, out Color albedo, out float height);

        public static Texture2D Solid(Color32 c)
        {
            var t = new Texture2D(2, 2, TextureFormat.RGB24, false);
            t.SetPixels32(new[] { c, c, c, c });
            t.Apply(false);
            return t;
        }

        // albedo と、その高さから作った法線マップ (ProcTex.Make と同じ詰め方: x = A・y = G)
        static void Make(int w, int h, PixelFn fn, float bump, out Texture2D albedo, out Texture2D normal, string name)
        {
            var px = new Color32[w * h];
            var hh = new float[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    fn((x + 0.5f) / w, (y + 0.5f) / h, x, y, out Color c, out float ht);
                    px[y * w + x] = c;
                    hh[y * w + x] = ht;
                }
            albedo = new Texture2D(w, h, TextureFormat.RGBA32, true) { name = name, wrapMode = TextureWrapMode.Repeat, anisoLevel = 8 };
            albedo.SetPixels32(px);
            albedo.Apply(true);
            var np = new Color32[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    float dx = hh[y * w + (x + 1) % w] - hh[y * w + (x - 1 + w) % w];
                    float dy = hh[((y + 1) % h) * w + x] - hh[((y - 1 + h) % h) * w + x];
                    Vector3 n = new Vector3(-dx * bump, -dy * bump, 1f).normalized;
                    byte nx = (byte)Mathf.Clamp((n.x * 0.5f + 0.5f) * 255f, 0, 255);
                    byte ny = (byte)Mathf.Clamp((n.y * 0.5f + 0.5f) * 255f, 0, 255);
                    np[y * w + x] = new Color32(255, ny, ny, nx);
                }
            normal = new Texture2D(w, h, TextureFormat.RGBA32, true, true) { name = name + "_N", wrapMode = TextureWrapMode.Repeat, anisoLevel = 8 };
            normal.SetPixels32(np);
            normal.Apply(true);
        }

        /// パンチカーペット (規約: リックパンチカーペット・色は濃いグレー)。濃淡の繊維が混じったフェルトで、踏まれた所に大きなむらと
        /// 白っぽい擦れがある。1 枚 0.8 m
        public static void Carpet(int size, int seed, out Texture2D a, out Texture2D n)
        {
            Make(size, size, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float big = ProcTex.Fbm(u, v, 3, 4, seed);
                float mid = ProcTex.Fbm(u, v, 20, 3, seed + 5);
                float fib = ProcTex.Hash(x, y, seed + 9);
                float fib2 = ProcTex.Hash(x / 2, y / 2, seed + 11);
                float fleck = fib > 0.90f ? (fib - 0.90f) / 0.10f : 0f;                   // 明るい繊維
                float dark = fib < 0.10f ? 1f - fib / 0.10f : 0f;                         // 黒い繊維
                float scuff = ProcTex.Smooth(0.56f, 0.80f, ProcTex.Fbm(u, v, 5, 3, seed + 17));
                float g = 0.20f + 0.026f * (big - 0.5f) + 0.026f * (mid - 0.5f) + 0.035f * (fib2 - 0.5f) + 0.11f * fleck - 0.06f * dark + 0.030f * scuff;
                c = new Color(g * 0.98f, g * 0.99f, g * 1.03f, 1f);
                h = 0.6f * fib + 0.5f * fib2 + 0.3f * mid;
            }, 1.6f, out a, out n, "Carpet");
        }

        /// 人工芝 (規約: 芝丈 30 mm・フレッシュグリーン)。葉ごとに明るさが違い、枯れ色の葉が混じる。1 枚 0.5 m
        public static void Turf(int size, int seed, out Texture2D a, out Texture2D n)
        {
            Make(size, size, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float blade = ProcTex.Hash(x, y / 3, seed);
                float tip = ProcTex.Hash(x, y, seed + 3);
                float patch = ProcTex.Fbm(u, v, 4, 3, seed + 7);
                float k = 0.50f + 0.60f * blade + 0.22f * (tip - 0.5f) + 0.26f * (patch - 0.5f);
                float dry = ProcTex.Hash(x / 2, y / 5, seed + 13) > 0.93f ? 1f : 0f;      // 枯れ色の葉が混ぜてある (リアル感をうたう製品)
                c = Color.Lerp(new Color(0.16f * k, 0.52f * k, 0.12f * k, 1f), new Color(0.50f * k, 0.50f * k, 0.22f * k, 1f), dry * 0.8f);
                h = blade + 0.5f * tip;
            }, 2.2f, out a, out n, "Turf");
        }

        /// 風呂すべり止めマット (規約: 40×80 cm・ストーン柄・ブルー)。不ぞろいな丸い石の盛り上がりが敷き詰めてあり、石の間は
        /// 溝で色が濃い。縁は平らな帯。1 枚ぶん (w : h = 1 : 2)
        public static void BathMat(int w, int h, int seed, out Texture2D a, out Texture2D n)
        {
            const int gx = 9, gy = 18;                        // 石の格子 (1 個 ≈ 4.4 cm)
            Make(w, h, (float u, float v, int x, int y, out Color c, out float ht) =>
            {
                float fu = u * gx, fv = v * gy;
                int iu = Mathf.FloorToInt(fu), iv = Mathf.FloorToInt(fv);
                float d1 = 9f, d2 = 9f;
                for (int dj = -1; dj <= 1; dj++)
                    for (int di = -1; di <= 1; di++)
                    {
                        int ci = iu + di, cj = iv + dj;
                        int wi = ((ci % gx) + gx) % gx, wj = ((cj % gy) + gy) % gy;
                        float px = ci + 0.5f + 0.62f * (ProcTex.Hash(wi, wj, seed) - 0.5f);
                        float py = cj + 0.5f + 0.62f * (ProcTex.Hash(wi, wj, seed + 1) - 0.5f);
                        float d = Mathf.Sqrt((fu - px) * (fu - px) + (fv - py) * (fv - py));
                        if (d < d1) { d2 = d1; d1 = d; } else if (d < d2) d2 = d;
                    }
                float stone = ProcTex.Smooth(0.02f, 0.30f, d2 - d1);                      // 石の境目で 0、石の中で 1
                float dome = stone * (1f - 0.55f * Mathf.Clamp01(d1 * 1.6f) * Mathf.Clamp01(d1 * 1.6f));
                float edge = Mathf.Min(Mathf.Min(u, 1f - u) * 2f, Mathf.Min(v, 1f - v) * 4f) ;   // 縁からの距離 (幅に対する比)
                float rim = 1f - ProcTex.Smooth(0.03f, 0.06f, edge);                       // 縁の平らな帯
                dome *= 1f - rim;
                float k = 0.70f + 0.36f * dome + 0.10f * rim;
                c = new Color(0.22f * k, 0.70f * k, 0.95f * k, 1f);
                ht = dome * 1.4f + 0.3f * rim;
            }, 3.2f, out a, out n, "BathMat");
        }

        /// 板 (白い化粧板・長尺シート): ほぼ一様で、ごく弱いむらと細い擦り傷
        public static void Board(int size, int seed, Color32 col, float amp, out Texture2D a, out Texture2D n)
        {
            Color bc = col;
            Make(size, size, (float u, float v, int x, int y, out Color c, out float h) =>
            {
                float m = ProcTex.Fbm(u, v, 4, 3, seed);
                float s = ProcTex.Hash(x / 9, y, seed + 3) > 0.985f ? 1f : 0f;            // 横の擦り傷
                float k = 1f + amp * (m - 0.5f) * 2f - 0.5f * amp * s;
                c = new Color(bc.r * k, bc.g * k, bc.b * k, 1f);
                h = 0.5f * m - 0.6f * s;
            }, 1.0f, out a, out n, "Board");
        }

        /// 窓の外: 白く飛んだ空、向かいの建物 (明るいグレーに窓の帯)、手前の木の緑
        public static Texture2D Outside(int w, int h, int seed)
        {
            var px = new Color32[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    float u = (x + 0.5f) / w, v = (y + 0.5f) / h;
                    Color c = Color.Lerp(new Color(0.93f, 0.95f, 0.97f), new Color(1f, 1f, 1f), ProcTex.Smooth(0.55f, 1f, v));
                    float roof = 0.74f + 0.05f * Mathf.Floor(ProcTex.VNoise(u, 0.5f, 5, seed) * 3f) / 3f;
                    if (v < roof)
                    {
                        float band = Mathf.Repeat(v * 9f, 1f);
                        float win = band > 0.35f && band < 0.75f && Mathf.Repeat(u * 40f, 1f) > 0.12f ? 1f : 0f;
                        c = Color.Lerp(new Color(0.86f, 0.87f, 0.86f), new Color(0.62f, 0.68f, 0.74f), win * 0.7f);
                    }
                    float tree = 0.30f + 0.22f * ProcTex.Fbm(u, 0.3f, 6, 3, seed + 3);
                    if (v < tree)
                    {
                        float k = 0.7f + 0.6f * ProcTex.Fbm(u, v * 2f, 24, 3, seed + 5);
                        c = new Color(0.36f * k, 0.50f * k, 0.30f * k);
                    }
                    px[y * w + x] = Color.Lerp(c, Color.white, 0.45f);
                }
            var t = new Texture2D(w, h, TextureFormat.RGB24, true) { name = "Outside", wrapMode = TextureWrapMode.Clamp };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }

        /// 壁の掲示 (青と緑の写真ふうの紙)
        public static Texture2D Poster(int seed)
        {
            const int w = 64, h = 96;
            var px = new Color32[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    float u = (x + 0.5f) / w, v = (y + 0.5f) / h;
                    Color c = v > 0.55f ? Color.Lerp(new Color(0.35f, 0.62f, 0.90f), new Color(0.75f, 0.88f, 0.98f), ProcTex.Fbm(u, v, 4, 3, seed))
                                        : Color.Lerp(new Color(0.25f, 0.60f, 0.30f), new Color(0.55f, 0.78f, 0.40f), ProcTex.Fbm(u, v, 6, 3, seed + 2));
                    if (v < 0.12f || u < 0.05f || u > 0.95f || v > 0.96f) c = new Color(0.96f, 0.96f, 0.94f);
                    px[y * w + x] = c;
                }
            var t = new Texture2D(w, h, TextureFormat.RGB24, true) { name = "Poster", wrapMode = TextureWrapMode.Clamp };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }

        /// ノート PC の画面 (暗い地に文字の行とグラフ)
        public static Texture2D Screen(int seed)
        {
            const int w = 96, h = 60;
            var px = new Color32[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    Color c = new Color(0.07f, 0.09f, 0.13f);
                    int row = y / 4;
                    if (x < 50 && y % 4 < 2 && x > 4 && x < 6 + (int)(40 * ProcTex.Hash(row, 0, seed))) c = new Color(0.55f, 0.85f, 0.60f);
                    if (x > 56 && x < 92 && y > 8 && y < 52)
                    {
                        c = new Color(0.12f, 0.15f, 0.22f);
                        float g = 30f + 14f * Mathf.Sin(x * 0.35f) + 6f * Mathf.Sin(x * 1.1f);
                        if (Mathf.Abs(y - g) < 1.2f) c = new Color(0.95f, 0.80f, 0.25f);
                    }
                    px[y * w + x] = c;
                }
            var t = new Texture2D(w, h, TextureFormat.RGB24, true) { name = "Screen", wrapMode = TextureWrapMode.Clamp };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }
    }
}
