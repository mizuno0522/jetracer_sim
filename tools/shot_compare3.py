#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3 つの版 (HDRP・URP・Built-in) を同じ位置で撮った画像を、行 = 視点・列 = 版 で 1 枚に並べ、HDRP との色の差を数える。
  python3 tools/shot_compare3.py shots/cmp3_minicar_x/hdrp shots/cmp3_minicar_x/urp shots/cmp3_minicar_x/builtin --out sheet.png
差 = 画像を 6×4 の升目に分け、升目ごとの平均色の差 (0〜255) を平均した値。1 つ目のフォルダが基準 (HDRP)。
同じ絵に見える目安は 10 以下 (ミラーボールの色は時刻で変わるので、会場の画像は数だけ少し大きく出る)。
"""
import argparse
import os

import numpy as np
from PIL import Image, ImageDraw


def grid(im):
    a = np.asarray(im.convert('RGB').resize((240, 136)), dtype=float)
    return a.reshape(4, 34, 6, 40, 3).mean(axis=(1, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--width', type=int, default=640)
    a = ap.parse_args()
    names = [f for f in sorted(os.listdir(a.dirs[0])) if f.endswith('.png') and f != 'sheet.png'
             and all(os.path.exists(os.path.join(d, f)) for d in a.dirs)]
    if not names:
        raise SystemExit('同じ名前の画像が無い')
    w = a.width
    rows = []
    for n in names:
        ims = [Image.open(os.path.join(d, n)).convert('RGB') for d in a.dirs]
        ref = grid(ims[0])
        diff = [float(np.abs(grid(i) - ref).mean()) for i in ims]
        ims = [i.resize((w, max(1, round(i.height * w / i.width)))) for i in ims]
        rows.append((n, ims, diff))
    sheet = Image.new('RGB', (w * len(a.dirs), sum(r[1][0].height for r in rows)), (20, 20, 20))
    dr = ImageDraw.Draw(sheet)
    y = 0
    print(f"{'画像':28s} " + ' '.join(f'{os.path.basename(os.path.normpath(d)):>9s}' for d in a.dirs))
    for n, ims, diff in rows:
        for k, im in enumerate(ims):
            sheet.paste(im, (w * k, y))
            label = os.path.basename(os.path.normpath(a.dirs[k])) + ('' if k == 0 else f'  diff {diff[k]:.1f}')
            dr.rectangle([w * k, y, w * k + 8 + 7 * len(label), y + 14], fill=(0, 0, 0))
            dr.text((w * k + 4, y + 2), label, fill=(255, 255, 0))
        print(f'{n:28s} ' + ' '.join(f'{d:9.1f}' for d in diff))
        y += ims[0].height
    sheet.save(a.out)
    print('→', a.out)


if __name__ == '__main__':
    main()
