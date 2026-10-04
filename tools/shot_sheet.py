#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimBridge の撮影モード (-shots) が保存した画像を 1 枚の一覧にする。
  python3 tools/shot_sheet.py shots/fuji_rx7_20261004_1700            # → <dir>/sheet.png (行 = 位置、列 = 追従・車載・俯瞰)
  python3 tools/shot_sheet.py shots/A shots/B --out /tmp/ab.png        # 2 つを左右に並べる (変更前・変更後、low・high)
"""
import argparse
import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

VIEWS = ('chase', 'onboard', 'overview')


def font(sz):
    for name in ('DejaVuSans-Bold.ttf', 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, sz)
        except OSError:
            continue
    return ImageFont.load_default()


def load(d):
    rows = {}
    for f in sorted(os.listdir(d)):
        m = re.match(r'(\d+)_s(\d+)_(chase|onboard|overview)\.png$', f)
        if m:
            rows.setdefault((int(m.group(1)), int(m.group(2))), {})[m.group(3)] = os.path.join(d, f)
    return [(k, rows[k]) for k in sorted(rows)]


def fit(path, w, h):
    tile = Image.new('RGB', (w, h), (24, 26, 30))
    if path and os.path.exists(path):
        im = Image.open(path).convert('RGB')
        k = min(w / im.width, h / im.height)
        im = im.resize((max(1, int(im.width * k)), max(1, int(im.height * k))), Image.LANCZOS)
        tile.paste(im, ((w - im.width) // 2, (h - im.height) // 2))
    return tile


def sheet(d, tw=640, th=360):
    rows = load(d)
    if not rows:
        sys.exit(f'{d}: 撮影画像 (NN_sXXXXX_chase.png など) が無い')
    head = 34
    out = Image.new('RGB', (tw * 3, head + th * len(rows)), (16, 17, 20))
    dr = ImageDraw.Draw(out)
    dr.text((8, 6), os.path.basename(os.path.normpath(d)), font=font(20), fill=(235, 235, 235))
    for r, ((_, s), views) in enumerate(rows):
        for c, v in enumerate(VIEWS):
            out.paste(fit(views.get(v), tw, th), (c * tw, head + r * th))
            dr.text((c * tw + 8, head + r * th + 6), f's={s} m  {v}', font=font(18), fill=(255, 255, 255))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--out')
    a = ap.parse_args()
    ims = [sheet(d) for d in a.dirs]
    if len(ims) == 1:
        res = ims[0]
    else:
        h = max(i.height for i in ims)
        res = Image.new('RGB', (sum(i.width for i in ims) + 8 * (len(ims) - 1), h), (0, 0, 0))
        x = 0
        for i in ims:
            res.paste(i, (x, 0))
            x += i.width + 8
    out = a.out or os.path.join(a.dirs[0], 'sheet.png')
    res.save(out)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
