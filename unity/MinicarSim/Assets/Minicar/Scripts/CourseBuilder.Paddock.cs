// サーキットのメインストレート: 観客の入った大きな観客席 (屋根つき) と、窓・ガレージ・看板のあるピット棟、ピットウォール。
// 絵はすべて実行時に作る (画像ファイルなし)。看板の文字は DotFont の点の文字。
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public partial class CourseBuilder
    {
        static Texture2D Tex(int w, int h, Color32[] px, string name)
        {
            var t = new Texture2D(w, h, TextureFormat.RGBA32, true) { name = name, wrapMode = TextureWrapMode.Repeat, anisoLevel = 8 };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }

        /// 観客: 1 人 = 幅 8 × 高さ 16 画素 (服の色の体と、その上の頭)。席の 1 割強は空席 (白い椅子)。32 × 16 人ぶん
        static Texture2D CrowdTex(int seed)
        {
            const int W = 256, H = 256;
            var px = new Color32[W * H];
            Color32[] shirts = { new Color32(200, 30, 36, 255), new Color32(200, 30, 36, 255), new Color32(235, 235, 232, 255), new Color32(235, 235, 232, 255),
                                 new Color32(30, 60, 150, 255), new Color32(28, 28, 30, 255), new Color32(238, 200, 40, 255), new Color32(240, 120, 30, 255),
                                 new Color32(60, 140, 80, 255), new Color32(120, 120, 126, 255) };
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++)
                {
                    int cx = x / 8, cy = y / 16, lx = x % 8, ly = y % 16;
                    float r = ProcTex.Hash(cx, cy, seed), s = ProcTex.Hash(cx, cy, seed + 1), q = ProcTex.Hash(cx, cy, seed + 2);
                    Color32 c = new Color32(196, 198, 200, 255);                     // 椅子・段
                    if (ly < 2) c = new Color32(120, 122, 124, 255);                  // 段の影
                    if (r > 0.12f && lx >= 1 && lx <= 6)
                    {
                        if (ly >= 2 && ly < 10) c = shirts[(int)(s * shirts.Length) % shirts.Length];
                        else if (ly >= 10 && ly < 15 && lx >= 2 && lx <= 5)
                            c = q < 0.35f ? new Color32(236, 236, 232, 255) : q < 0.5f ? new Color32(205, 30, 36, 255) : q < 0.75f ? new Color32(40, 34, 30, 255) : new Color32(222, 180, 150, 255);   // 帽子・髪・顔
                    }
                    px[y * W + x] = c;
                }
            return Tex(W, H, px, "Crowd2");
        }

        /// 看板の帯 (横に 4 枚、1 枚 12 m × 高さ 1.2 m ぶん)
        static readonly (string text, Color32 bg, Color32 fg)[] kBoards =
        {
            ("PANASONIC", new Color32(238, 238, 236, 255), new Color32(20, 60, 160, 255)),
            ("NGK", new Color32(200, 26, 34, 255), new Color32(245, 245, 245, 255)),
            ("FUJI SPEEDWAY", new Color32(24, 52, 130, 255), new Color32(245, 245, 245, 255)),
            ("DUNLOP", new Color32(240, 200, 30, 255), new Color32(20, 20, 22, 255)),
        };

        static Color32 BoardColor(float xm, float ym, float boardH)
        {
            int k = (int)(xm / 12f) % kBoards.Length;
            var b = kBoards[k];
            float lx = xm - Mathf.Floor(xm / 12f) * 12f, th = boardH * 0.6f, tw = DotFont.Width(b.text, th);
            if (tw > 11f) { th *= 11f / tw; tw = 11f; }
            if (lx < 0.06f || lx > 11.94f) return new Color32(60, 62, 66, 255);
            return DotFont.At(b.text, lx - (12f - tw) * 0.5f, ym - (boardH - th) * 0.5f, th) ? b.fg : b.bg;
        }

        static Texture2D BoardTex()
        {
            const int W = 2048, H = 64;
            var px = new Color32[W * H];
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++)
                    px[y * W + x] = BoardColor(x * 48f / W, y * 1.2f / H, 1.2f);
            return Tex(W, H, px, "Boards");
        }

        /// ピット棟の正面 (横 48 m × 高さ 10 m): 1 階はガレージ、その上に看板の帯、2〜3 階は窓、屋上の縁
        static Texture2D PitFacadeTex()
        {
            const int W = 2048, H = 512;
            var px = new Color32[W * H];
            var wall = new Color32(214, 216, 218, 255); var frame = new Color32(120, 124, 130, 255);
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++)
                {
                    float xm = x * 48f / W, ym = y * 10f / H;
                    Color32 c = wall;
                    float bay = xm - Mathf.Floor(xm / 6f) * 6f;
                    if (ym < 3.5f)
                    {
                        // ガレージ: 奥が暗い開口。上ほど暗く、床に近いと少し明るい
                        if (bay > 0.35f && bay < 5.65f && ym < 3.3f)
                        {
                            byte v = (byte)(26 + 26 * Mathf.Clamp01(1f - ym / 1.6f));
                            c = new Color32(v, v, (byte)(v + 4), 255);
                            if (ym < 1.5f && ProcTex.Hash((int)(xm * 2.2f), (int)(xm / 6f), 77) > 0.55f) c = new Color32(150, 40, 40, 255);      // 工具箱など
                        }
                    }
                    else if (ym < 4.8f) c = BoardColor(xm, ym - 3.55f, 1.2f);
                    else if (ym < 8.7f)
                    {
                        // 窓: 1.5 m ごとの枠、階の境目 (6.6〜7.0 m) は壁
                        float mx = xm - Mathf.Floor(xm / 1.5f) * 1.5f;
                        bool spandrel = ym > 6.55f && ym < 7.05f, mullion = mx < 0.07f;
                        if (spandrel) c = wall;
                        else if (mullion || ym < 4.95f || ym > 8.55f) c = frame;
                        else
                        {
                            float sky = Mathf.Clamp01((ym - 4.9f) / 3.8f), n = ProcTex.Hash((int)(xm / 1.5f), (int)(ym > 6.8f ? 1 : 0), 78);
                            c = new Color32((byte)(40 + 50 * sky + 30 * n), (byte)(62 + 60 * sky + 26 * n), (byte)(84 + 70 * sky + 20 * n), 255);
                        }
                    }
                    else c = ym > 9.5f ? new Color32(70, 72, 76, 255) : new Color32(150, 152, 156, 255);
                    px[y * W + x] = c;
                }
            return Tex(W, H, px, "PitFacade");
        }

        // 観客席 (コントロールライン付近の外側) とピット棟 (内側)。メインストレートの向きに沿って置く
        void BuildPaddock(float bar)
        {
            int n = m_C.Length;
            Vector2 t = (m_C[1] - m_C[0]).normalized, nrm = m_N[0];
            float yaw = Mathf.Atan2(t.y, t.x);
            Vector2 mid = Vector2.zero;
            foreach (var p in m_C) mid += p;
            mid /= n;
            float inSide = Vector2.Dot(mid - m_C[0], nrm) > 0f ? 1f : -1f;      // コースの重心の側が内側
            Quaternion rot = RosFrame.Yaw(yaw);
            Vector2 o = m_C[0] + t * 60f;                                        // 建物の中心はコントロールラインの少し先
            System.Func<float, float, float, Vector3> P = (along, across, z) =>
            {
                Vector2 q = o + t * along + nrm * across;
                return RosFrame.ToUnity(q.x, q.y, z);
            };

            // ---- 観客席: 30 段 (奥行 25.5 m・高さ 15 m)、長さ 420 m。観客は席の傾きに沿った 1 枚の絵
            var concrete = Lit(new Color32(150, 150, 146, 255), Speckle(128, 0.25f, 92), 0.1f, new Vector2(20f, 2f));
            var white = Lit(new Color32(226, 228, 230, 255), null, 0.4f);
            var steel = Lit(new Color32(120, 124, 130, 255), null, 0.5f);
            float side = -inSide, len = 420f, d0 = bar + 9f, z0 = 2.6f, depth = 25.5f, rise = 15f;
            Box("StandFront", P(0f, side * (d0 - 0.2f), z0 * 0.5f), new Vector3(0.4f, z0, len), rot, concrete);
            for (int r = 0; r < 5; r++)        // 席の下の躯体 (大きな 5 段)
            {
                float dd = depth / 5f, zt = z0 + rise * r / 5f;
                Box("StandBody", P(0f, side * (d0 + (r + 0.5f) * dd), zt * 0.5f), new Vector3(dd, zt, len), rot, concrete);
            }
            var cv = new List<Vector3>(); var cu = new List<Vector2>(); var ct = new List<int>();
            float slope = Mathf.Sqrt(depth * depth + rise * rise);
            Quad(cv, ct, cu, P(-len * 0.5f, side * d0, z0 + 0.5f), P(len * 0.5f, side * d0, z0 + 0.5f),
                 P(len * 0.5f, side * (d0 + depth), z0 + rise + 0.5f), P(-len * 0.5f, side * (d0 + depth), z0 + rise + 0.5f), 0f, len / 16f, slope / 15.8f);
            var crowd = Lit(new Color32(255, 255, 255, 255), CrowdTex(91), 0.03f);
            MeshObject("StandCrowd", ToMesh(cv, cu, ct), crowd);
            // 前の壁の看板
            var boards = Lit(new Color32(255, 255, 255, 255), BoardTex(), 0.3f);
            var bv = new List<Vector3>(); var bu = new List<Vector2>(); var bt = new List<int>();
            // 文字は見る側から読める向きに: コースの進む向きの左を向いて見る面はそのまま、右を向いて見る面は横を反転する
            System.Action<List<Vector3>, List<int>, List<Vector2>, float, float, float, float, float, float> Face = (v, tr, uv, half, across, zLo, zHi, facing, vTop) =>
            {
                float u1 = 2f * half / 48f;
                Quad(v, tr, uv, P(-half, across, zLo), P(half, across, zLo), P(half, across, zHi), P(-half, across, zHi), facing > 0f ? 0f : u1, facing > 0f ? u1 : 0f, vTop);
            };
            Face(bv, bt, bu, len * 0.5f, side * (d0 - 0.45f), 1.2f, 2.4f, side, 1f);
            // 屋根: 奥の壁から前へ 27 m 張り出す。10 m ごとの梁と、前の縁
            float zr = z0 + rise + 7f, dBack = d0 + depth + 1.5f;
            Box("StandBack", P(0f, side * (dBack + 0.3f), zr * 0.5f), new Vector3(0.6f, zr, len), rot, concrete);
            Box("StandRoof", P(0f, side * (dBack - 13.5f), zr), new Vector3(27f, 0.35f, len + 4f), rot, white);
            Box("StandRoofEdge", P(0f, side * (dBack - 27f), zr - 0.5f), new Vector3(0.4f, 1.3f, len + 4f), rot, white);
            for (float a = -len * 0.5f; a <= len * 0.5f + 0.1f; a += 10f)
            {
                Box("StandRib", P(a, side * (dBack - 13.5f), zr + 0.7f), new Vector3(27f, 1.1f, 0.35f), rot, steel);
                if (Mathf.Abs(a % 40f) < 0.1f) Box("StandPillar", P(a, side * (dBack - 6f), zr * 0.5f), new Vector3(0.5f, zr, 0.5f), rot, steel);
            }

            // ---- ピット棟: 長さ 288 m・奥行 14 m・高さ 10 m。正面と裏に窓・ガレージ・看板の絵
            var wall = Lit(new Color32(206, 208, 212, 255), Speckle(128, 0.15f, 93), 0.3f, new Vector2(30f, 1f));
            float plen = 288f, pd = bar + 15f;
            Box("PitBuilding", P(0f, inSide * (pd + 7f), 5f), new Vector3(13.8f, 10f, plen), rot, wall);
            Box("PitRoof", P(0f, inSide * (pd + 6f), 10.2f), new Vector3(17f, 0.4f, plen + 2f), rot, Lit(new Color32(92, 94, 98, 255), null, 0.3f));
            var facade = Lit(new Color32(255, 255, 255, 255), PitFacadeTex(), 0.35f);
            var fv = new List<Vector3>(); var fu = new List<Vector2>(); var ft = new List<int>();
            Face(fv, ft, fu, plen * 0.5f, inSide * (pd - 0.05f), 0f, 10f, inSide, 1f);        // コース側
            Face(fv, ft, fu, plen * 0.5f, inSide * (pd + 14.05f), 0f, 10f, -inSide, 1f);      // パドック側
            MeshObject("PitFacade", ToMesh(fv, fu, ft), facade);
            // ピットウォール (コースとピットレーンの間の低い壁) と、その上の金網。壁に看板
            float pw = bar + 3.5f;
            Box("PitWall", P(0f, inSide * pw, 0.6f), new Vector3(0.3f, 1.2f, plen), rot, white);
            Face(bv, bt, bu, plen * 0.5f, inSide * (pw - 0.16f), 0.05f, 1.2f, inSide, 1f);
            Face(bv, bt, bu, plen * 0.5f, inSide * (pw + 0.16f), 0.05f, 1.2f, -inSide, 1f);
            MeshObject("Boards", ToMesh(bv, bu, bt), boards);
        }
    }
}
