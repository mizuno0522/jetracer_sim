// ⑥矢印信号の LED 掲示板 (96x16 ドット) のテクスチャ。
// 図柄と色は vehicle_sim._arrow_texture と同じ (parking-signal.pdf の点灯パターン)。
//   1 = 左 (シアン) / 2 = 右 (マゼンタ) / それ以外 = 中・直進 (イエロー)
using UnityEngine;

namespace Minicar
{
    public static class ArrowTexture
    {
        const int Cols = 96, Rows = 16, Cell = 4;

        public static Texture2D Make(int dir)
        {
            var grid = new bool[Rows, Cols];
            Color32 col;
            if (dir == 1)
            {
                col = new Color32(0, 255, 255, 255);
                Fill(grid, new[] { (0.04f, 1.00f), (0.21f, 0.00f), (0.21f, 1.00f) });
                Fill(grid, new[] { (0.21f, 0.42f), (0.96f, 0.42f), (0.96f, 1.00f), (0.21f, 1.00f) });
            }
            else if (dir == 2)
            {
                col = new Color32(255, 0, 255, 255);
                Fill(grid, new[] { (0.96f, 1.00f), (0.79f, 0.00f), (0.79f, 1.00f) });
                Fill(grid, new[] { (0.04f, 0.42f), (0.79f, 0.42f), (0.79f, 1.00f), (0.04f, 1.00f) });
            }
            else
            {
                col = new Color32(255, 255, 0, 255);
                Fill(grid, new[] { (0.55f, 0.00f), (0.30f, 0.62f), (0.80f, 0.62f) });
                Fill(grid, new[] { (0.03f, 0.62f), (0.97f, 0.62f), (0.97f, 0.72f), (0.03f, 0.72f) });
                Fill(grid, new[] { (0.42f, 0.62f), (0.68f, 0.62f), (0.68f, 1.00f), (0.42f, 1.00f) });
            }

            int tw = Cols * Cell, th = Rows * Cell;
            var px = new Color32[tw * th];
            var off = new Color32(8, 8, 8, 255);
            for (int i = 0; i < px.Length; i++) px[i] = off;
            for (int gy = 0; gy < Rows; gy++)
                for (int gx = 0; gx < Cols; gx++)
                {
                    if (!grid[gy, gx]) continue;
                    // 点灯セルを「粒」として描く (周囲 1px を消灯のまま残す)
                    for (int dy = 0; dy < Cell - 1; dy++)
                        for (int dx = 0; dx < Cell - 1; dx++)
                        {
                            int ty = th - 1 - (gy * Cell + dy);   // テクスチャは下の行から
                            px[ty * tw + gx * Cell + dx] = col;
                        }
                }
            var t = new Texture2D(tw, th, TextureFormat.RGB24, true)
            { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }

        // 正規化座標 (x 右, y 下) の多角形に中心が入るセルを点灯
        static void Fill(bool[,] grid, (float x, float y)[] poly)
        {
            for (int gy = 0; gy < Rows; gy++)
                for (int gx = 0; gx < Cols; gx++)
                {
                    float px = (gx + 0.5f) / Cols, py = (gy + 0.5f) / Rows;
                    bool inside = false;
                    for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i++)
                    {
                        var a = poly[i];
                        var b = poly[j];
                        if ((a.y > py) != (b.y > py) &&
                            px < (b.x - a.x) * (py - a.y) / (b.y - a.y) + a.x)
                            inside = !inside;
                    }
                    if (inside) grid[gy, gx] = true;
                }
        }
    }
}
