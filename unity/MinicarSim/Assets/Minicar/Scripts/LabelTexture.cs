// 駐車枠の番号 (P1 / P2 / P3) など、短い英数字を 5x7 ドット文字で描いたテクスチャ。
// フォントアセットに依存しないので、バッチビルドでも確実に出る (ArrowTexture と同じ流儀)。
using System.Collections.Generic;
using UnityEngine;

namespace Minicar
{
    public static class LabelTexture
    {
        const int Cell = 8, Gap = 1, Margin = 1;   // セル [px]・文字間 [セル]・余白 [セル]

        static readonly Dictionary<char, string[]> kGlyph = new Dictionary<char, string[]>
        {
            ['P'] = new[] { "11110", "10001", "10001", "11110", "10000", "10000", "10000" },
            ['0'] = new[] { "01110", "10001", "10011", "10101", "11001", "10001", "01110" },
            ['1'] = new[] { "00100", "01100", "00100", "00100", "00100", "00100", "01110" },
            ['2'] = new[] { "01110", "10001", "00001", "00010", "00100", "01000", "11111" },
            ['3'] = new[] { "11110", "00001", "00001", "01110", "00001", "00001", "11110" },
            ['4'] = new[] { "00010", "00110", "01010", "10010", "11111", "00010", "00010" },
            ['5'] = new[] { "11111", "10000", "11110", "00001", "00001", "10001", "01110" },
            ['6'] = new[] { "00110", "01000", "10000", "11110", "10001", "10001", "01110" },
            ['7'] = new[] { "11111", "00001", "00010", "00100", "01000", "01000", "01000" },
            ['8'] = new[] { "01110", "10001", "10001", "01110", "10001", "10001", "01110" },
            ['9'] = new[] { "01110", "10001", "10001", "01111", "00001", "00010", "01100" },
        };

        public static Texture2D Make(string text, Color32 ink, Color32 bg)
        {
            int cols = Margin * 2 + text.Length * 5 + (text.Length - 1) * Gap;
            int rows = Margin * 2 + 7;
            int tw = cols * Cell, th = rows * Cell;
            var px = new Color32[tw * th];
            for (int i = 0; i < px.Length; i++) px[i] = bg;
            for (int ci = 0; ci < text.Length; ci++)
            {
                if (!kGlyph.TryGetValue(char.ToUpperInvariant(text[ci]), out var g)) continue;
                int gx0 = Margin + ci * (5 + Gap);
                for (int gy = 0; gy < 7; gy++)
                    for (int gx = 0; gx < 5; gx++)
                    {
                        if (g[gy][gx] != '1') continue;
                        for (int dy = 0; dy < Cell; dy++)
                            for (int dx = 0; dx < Cell; dx++)
                            {
                                int ty = th - 1 - ((Margin + gy) * Cell + dy);   // テクスチャは下の行から
                                px[ty * tw + (gx0 + gx) * Cell + dx] = ink;
                            }
                    }
            }
            var t = new Texture2D(tw, th, TextureFormat.RGB24, true)
            { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
            t.SetPixels32(px);
            t.Apply(true);
            return t;
        }
    }
}
