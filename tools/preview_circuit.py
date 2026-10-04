#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity を使わずに、サーキット (course_<name>_<profile>.json) の見た目を 1 枚の画像で確かめる。
路面のテクスチャは Unity の ProcTex.cs と同じ式で作り、コースの形・幅・縁石・グラベル・走行ラインのタイヤ痕も
CourseBuilder.Circuit.cs と同じ規則で置く。地形・森・山・雲は Landscape.cs と同じ式。地面は画素ごとに光線を飛ばして
地形の高さに当て、ガードレール・観客席は多角形、木はカメラに向けた絵の板として深度で隠しながら重ねる。
★Unity の描画そのものではない (影・反射・遠景のかすみは近似。Unity の木は十字の板)。形と質感の確認用。

  python3 tools/preview_circuit.py --course unity/course_fuji_real_rx7.json --s 1250 --out /tmp/fuji.png [--hud]
  --s     画面に入れたい自車の位置 (コントロールラインからの距離 m)
  --view  chase (追従視点) | onboard (車載カメラ) | scenic (コース脇から山の方向)
"""
import argparse
import json
import math
import os
import re

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from scipy.spatial import cKDTree


# ------------------------------------------------------------ ProcTex と同じノイズ
def hash2(x, y, s):
    with np.errstate(over='ignore'):
        x = np.asarray(x, dtype=np.int64).astype(np.uint32)
        y = np.asarray(y, dtype=np.int64).astype(np.uint32)
        h = x * np.uint32(374761393) + y * np.uint32(668265263) + np.uint32((s * 2246822519) & 0xFFFFFFFF)
        h = (h ^ (h >> np.uint32(13))) * np.uint32(1274126177)
        h = h ^ (h >> np.uint32(16))
    return (h & np.uint32(0xFFFFFF)).astype(np.float64) / 16777215.0


def vnoise(u, v, period, seed):
    x, y = u * period, v * period
    x0, y0 = np.floor(x).astype(np.int64), np.floor(y).astype(np.int64)
    fx, fy = x - x0, y - y0
    fx = fx * fx * (3 - 2 * fx)
    fy = fy * fy * (3 - 2 * fy)
    xa, xb, ya, yb = x0 % period, (x0 + 1) % period, y0 % period, (y0 + 1) % period
    a, b = hash2(xa, ya, seed), hash2(xb, ya, seed)
    c, d = hash2(xa, yb, seed), hash2(xb, yb, seed)
    return (a + (b - a) * fx) + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fy


def fbm(u, v, base, octaves, seed):
    s, amp, norm, p = 0.0, 0.5, 0.0, base
    for o in range(octaves):
        s = s + amp * vnoise(u, v, p, seed + o * 101)
        norm += amp
        amp *= 0.5
        p *= 2
    return s / norm


def grid(w, h):
    y, x = np.mgrid[0:h, 0:w]
    return (x + 0.5) / w, (y + 0.5) / h, x, y


def lerp(a, b, t):
    return a + (b - a) * t


def asphalt(size, seed, light):
    u, v, x, y = grid(size, size)
    big, mid = fbm(u, v, 3, 4, seed), fbm(u, v, 24, 3, seed + 7)
    r = hash2(x, y, seed + 13)
    stone = np.where(r > 0.90, (r - 0.90) / 0.10, 0)
    pore = np.where(r < 0.06, 1 - r / 0.06, 0)
    g = light + 0.07 * (big - 0.5) + 0.05 * (mid - 0.5) + 0.20 * stone - 0.10 * pore
    return np.stack([g * 0.98, g * 0.99, g * 1.03], -1), 0.6 * mid + 0.8 * stone - 0.8 * pore


def grass(size, seed):
    u, v, x, y = grid(size, size)
    big, mid = fbm(u, v, 4, 4, seed), fbm(u, v, 32, 3, seed + 5)
    blade = hash2(x, y, seed + 9)
    stripe = np.where((np.floor(v * 2).astype(int) & 1) == 0, 1.06, 0.94)
    k = (0.85 + 0.3 * big) * stripe * (0.9 + 0.2 * blade)
    t = np.clip(0.25 + 0.6 * (mid - 0.5) + 0.4 * (big - 0.5), 0, 1)[..., None]
    lush, dry = np.array([0.20, 0.36, 0.14]), np.array([0.42, 0.44, 0.22])
    return lerp(lush, dry, t) * k[..., None], 0.5 * blade + 0.5 * mid


def gravel(size, seed):
    u, v, x, y = grid(size, size)
    cell, r = vnoise(u, v, 96, seed), hash2(x, y, seed + 3)
    k = 0.75 + 0.35 * cell + 0.15 * (r - 0.5)
    return np.stack([0.66 * k, 0.60 * k, 0.50 * k], -1), cell + 0.3 * r


def kerb(w, h, seed):
    u, v, x, y = grid(w, h)
    red = (v < 0.5)[..., None]
    wear, grit = fbm(u, v, 8, 4, seed), hash2(x, y, seed + 1)
    base = np.where(red, np.array([0.78, 0.12, 0.10]), np.array([0.92, 0.91, 0.88]))
    worn = (np.clip((wear - 0.62) * 4, 0, 1) * (0.4 + 0.6 * u))[..., None]
    c = lerp(base, np.array([0.35, 0.34, 0.33]), worn) * (0.92 + 0.12 * grit)[..., None]
    return c, 0.5 + 0.5 * np.sin(v * np.pi * 24) * (1 - u) + 0.2 * grit


def line_tex(seed):
    u, v, x, y = grid(32, 256)
    k = 0.80 + 0.18 * fbm(u, v, 16, 3, seed) + 0.04 * hash2(x, y, seed)
    return np.stack([0.93 * k, 0.93 * k, 0.90 * k], -1)


def rubber_tex(seed):
    u, v, x, y = grid(128, 512)
    streak = fbm(u, v * 0.125, 32, 3, seed)
    across = np.sin(u * np.pi)
    return np.clip((streak - 0.35) * 2.2, 0, 1) * across * across * 0.55


def normals(hm, bump):
    dx = np.roll(hm, -1, 1) - np.roll(hm, 1, 1)
    dy = np.roll(hm, -1, 0) - np.roll(hm, 1, 0)
    n = np.stack([-dx * bump, -dy * bump, np.ones_like(hm)], -1)
    return n / np.linalg.norm(n, axis=-1, keepdims=True)


def sample(tex, u, v):
    """繰り返し (repeat) のバイリニア"""
    h, w = tex.shape[:2]
    x, y = (u % 1.0) * w - 0.5, (v % 1.0) * h - 0.5
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x - x0, y - y0
    if tex.ndim == 3:
        fx, fy = fx[..., None], fy[..., None]
    x0, x1, y0, y1 = x0 % w, (x0 + 1) % w, y0 % h, (y0 + 1) % h
    a, b, c, d = tex[y0, x0], tex[y0, x1], tex[y1, x0], tex[y1, x1]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def smooth(a, b, x):
    t = np.clip((x - a) / (b - a), 0, 1)
    return t * t * (3 - 2 * t)


def fbmw(x, y, scale, octaves, seed):
    """ProcTex.FbmW: 世界座標のノイズ (scale [m] が 1 格子、周期 256 格子)"""
    u, v = np.asarray(x, float) / (scale * 256), np.asarray(y, float) / (scale * 256)
    return fbm(u - np.floor(u), v - np.floor(v), 256, octaves, seed)


def tree_tex(kind, seed):
    """ProcTex.Tree: 木の絵 (256×512 RGBA、0〜1)。行 0 が下。枝と葉の固まりを奥から描く"""
    W, H = 256, 512
    cnt = [0]
    def R():
        r = float(hash2(cnt[0], kind, seed)); cnt[0] += 1; return r
    L = np.array([-0.55, 0.62, 0.56]); L /= np.linalg.norm(L)
    img = np.zeros((H, W, 4))
    if kind == 0:
        leafA, leafB = np.array([0.20, 0.30, 0.10]), np.array([0.44, 0.52, 0.20])
    else:
        leafA, leafB = np.array([0.10, 0.20, 0.12]), np.array([0.20, 0.32, 0.17])
    img[..., :3] = leafA * 0.6
    hue = (R() - 0.5) * 0.25
    # 描く要素: (z, kind 0=円 1=線分, params..., color)
    items = []
    def bark(k): return np.array([0.30, 0.24, 0.18]) * k
    # 幹
    top = 0.45 * H if kind == 0 else 0.95 * H
    w0, w1 = (15.0, 7.0) if kind == 0 else (9.0, 1.5)
    items.append((0.0, 1, (128.0, 0.0, 128.0 + (R() - 0.5) * 8, top, w0, w1), bark(0.9)))
    if kind == 0:
        nl = 6
        lobes = [(128.0, 0.58 * H, 0.0, 100.0, 125.0, 100.0)]
        for i in range(nl):
            a = R() * math.pi * 2
            lobes.append((128 + math.cos(a) * (40 + 30 * R()), 0.56 * H + (R() - 0.45) * 170, math.sin(a) * 45,
                          45 + 25 * R(), 45 + 25 * R(), 45 + 25 * R()))
        for (cx, cy, cz, rx, ry, rz) in lobes[1:]:   # 枝
            items.append((cz * 0.3, 1, (128.0, 0.28 * H + R() * 0.14 * H, cx, cy, 6.0, 1.5), bark(0.7)))
        n = 5200
        for i in range(n):
            j = int(R() * (nl + 4))
            lb = lobes[0 if j > nl else j]
            cx, cy, cz, rx, ry, rz = lb
            th, ph = R() * math.pi * 2, math.acos(2 * R() - 1)
            dx, dy, dz = math.sin(ph) * math.cos(th), math.cos(ph), math.sin(ph) * math.sin(th)
            rr = 0.55 + 0.45 * math.sqrt(R())
            x, y, z = cx + dx * rx * rr, cy + dy * ry * rr, cz + dz * rz * rr
            nx, ny, nz = dx + (R() - 0.5) * 0.8, dy + (R() - 0.5) * 0.8, dz + (R() - 0.5) * 0.8
            nn = math.sqrt(nx * nx + ny * ny + nz * nz)
            dif = max(0.0, (nx * L[0] + ny * L[1] + nz * L[2]) / nn)
            ao = (0.45 + 0.55 * rr ** 2) * (0.72 + 0.28 * min(1.0, max(0.0, (y - 0.30 * H) / (0.6 * H))))
            k = (0.36 + 0.85 * dif) * ao * (0.85 + 0.3 * R())
            c = lerp(leafA, leafB, min(1.0, max(0.0, R() * 0.6 + dif * 0.5 + hue)))
            items.append((z, 0, (x, y, 2.2 + 2.6 * R()), c * k))
    else:
        y0, yt = 0.10 * H, 0.97 * H
        y = y0
        while y < yt:
            f = (yt - y) / (yt - y0)
            Lb = (f ** 0.9) * 96 + 6
            nb = 5 + int(R() * 3)
            a0 = R() * math.pi * 2
            for b in range(nb):
                a = a0 + b * 2 * math.pi / nb + (R() - 0.5) * 0.6
                lb = Lb * (0.8 + 0.35 * R())
                droop = 0.30 * lb
                ex, ez, ey = math.cos(a) * lb, math.sin(a) * lb, y - droop
                items.append((ez * 0.5, 1, (128.0, y, 128 + ex, ey, 2.2, 0.8), bark(0.55)))
                m = int(lb / 2.2) + 3
                for j in range(m):
                    t = 0.15 + 0.85 * R()
                    sx = (R() - 0.5) * (4 + 9 * t)
                    px = 128 + ex * t + (-math.sin(a)) * sx
                    pz = ez * t + math.cos(a) * sx
                    py = y - droop * t * t + (R() - 0.3) * 6
                    nx, ny, nz = math.cos(a), 0.45, math.sin(a)
                    nn = math.sqrt(nx * nx + ny * ny + nz * nz)
                    dif = max(0.0, (nx * L[0] + ny * L[1] + nz * L[2]) / nn)
                    ao = (0.35 + 0.65 * t) * (0.75 + 0.25 * (1 - f))
                    k = (0.30 + 0.90 * dif) * ao * (0.85 + 0.3 * R())
                    c = lerp(leafA, leafB, min(1.0, max(0.0, R() * 0.5 + dif * 0.4 + hue)))
                    items.append((pz, 0, (px, py, 2.0 + 2.2 * R()), c * k))
            y += 9 + 5 * R()
    items.sort(key=lambda q: q[0])
    yy, xx = np.mgrid[0:H, 0:W]
    for z, typ, p, c in items:
        if typ == 0:
            x, y, r = p
            x0, x1 = max(0, int(x - r)), min(W, int(x + r) + 1)
            y0_, y1 = max(0, int(y - r)), min(H, int(y + r) + 1)
            if x0 >= x1 or y0_ >= y1:
                continue
            m = (xx[y0_:y1, x0:x1] + 0.5 - x) ** 2 + (yy[y0_:y1, x0:x1] + 0.5 - y) ** 2 <= r * r
            img[y0_:y1, x0:x1][m] = [*c, 1.0]
        else:
            ax, ay, bx, by, wa, wb = p
            x0, x1 = max(0, int(min(ax, bx) - wa)), min(W, int(max(ax, bx) + wa) + 1)
            y0_, y1 = max(0, int(min(ay, by) - wa)), min(H, int(max(ay, by) + wa) + 1)
            X, Y = xx[y0_:y1, x0:x1] + 0.5, yy[y0_:y1, x0:x1] + 0.5
            vx, vy = bx - ax, by - ay
            ll = vx * vx + vy * vy
            t = np.clip(((X - ax) * vx + (Y - ay) * vy) / ll, 0, 1)
            d = np.hypot(X - ax - t * vx, Y - ay - t * vy)
            m = d <= (wa + (wb - wa) * t) * 0.5
            shade = 0.8 + 0.4 * (X - ax - t * vx) / np.maximum(1, wa)
            col = c[None, :] * np.clip(shade, 0.5, 1.3)[..., None]
            img[y0_:y1, x0:x1][m] = np.concatenate([col, np.ones((*col.shape[:-1], 1))], -1)[m]
    return img


def clouds_tex(seed):
    """ProcTex.Clouds: 1024×256 (u = 方位、v = 仰角 / 90°)。RGBA"""
    u, v, x, y = grid(1024, 256)
    f = fbm(u, v * 0.35, 12, 6, seed)
    cover = np.clip((f - 0.50) * 4.5, 0, 1) * smooth(0, 0.12, v) * (1 - smooth(0.55, 1, v))
    below = fbm(u, v * 0.35 - 0.012, 12, 6, seed)
    shade = 0.74 + 0.30 * np.clip((f - below) * 6 + 0.5, 0, 1) + 0.06 * fbm(u, v, 64, 2, seed + 3)
    return np.stack([shade, shade, shade * 1.02, cover * 0.9], -1)


def grass_detail(size, seed):
    u, v, x, y = grid(size, size)
    big, mid = fbm(u, v, 4, 4, seed), fbm(u, v, 32, 3, seed + 5)
    blade = hash2(x, y, seed + 9)
    return 0.5 * (0.70 + 0.30 * big + 0.24 * mid + 0.20 * blade), 0.5 * blade + 0.5 * mid


class Landscape:
    """Landscape.cs と同じ地形・色・木の置き方"""
    INNER, OUTER = 1500.0, 30000.0
    MD, MH, MR, ML = 18000.0, 3200.0, 30000.0, 6500.0
    FUJI = np.array([-0.25, -0.97]) / np.hypot(-0.25, -0.97)
    TREE_GRID, SKY_R = 13.0, 40000.0

    def __init__(self, tr, bounds):
        self.bar = tr.bar
        self.b = bounds
        self.start = tr.C[0]
        self.mid = np.array([(bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2])
        self.peak = self.mid + self.FUJI * self.MD
        left = np.array([-self.FUJI[1], self.FUJI[0]])
        self.hoei = self.peak + left * 3300
        self.hoei_c = self.hoei + (self.hoei - self.peak) / np.linalg.norm(self.hoei - self.peak) * 450
        self.mid_m = float(self.mountain(np.array(self.mid[0]), np.array(self.mid[1]))[0])
        self.xs = self.axis(bounds[0], bounds[2], self.mid[0])
        self.ys = self.axis(bounds[1], bounds[3], self.mid[1])
        X, Y = np.meshgrid(self.xs, self.ys)
        bx = np.maximum(0, np.maximum(bounds[0] - X, X - bounds[2]))
        by = np.maximum(0, np.maximum(bounds[1] - Y, Y - bounds[3]))
        D = np.hypot(bx, by)
        near = D < 400
        kd = cKDTree(tr.C[::2])
        D[near] = kd.query(np.stack([X[near], Y[near]], -1))[0]
        self.D = D
        self.H = self.height(X, Y, D)

    def axis(self, lo, hi, mid):
        x0, x1 = lo - self.INNER, hi + self.INNER
        n = math.ceil((x1 - x0) / 25)
        a = [x0 + k * 25 for k in range(n + 1)]
        last, s = a[-1], 25.0
        while last < mid + self.OUTER:
            s = min(150.0 if last - mid < 10000 else 400.0, s * 1.08)
            last += s
            a.append(last)
        first, s, lead = a[0], 25.0, []
        while first > mid - self.OUTER:
            s = min(150.0 if mid - first < 10000 else 400.0, s * 1.08)
            first -= s
            lead.append(first)
        return np.array(lead[::-1] + a)

    def mountain(self, x, y):
        dx, dy = x - self.peak[0], y - self.peak[1]
        r = np.hypot(dx, dy)
        u = np.arctan2(dy, dx) / (2 * np.pi) + 0.5
        g = fbm(u, np.clip(r / self.MR, 0, 1) * 0.5, 90, 3, 311)
        re = np.maximum(r, 350.0)
        e0 = math.exp(-self.MR / self.ML)
        skirt = (np.exp(-re / self.ML) - e0) / (math.exp(-350.0 / self.ML) - e0)
        m = self.MH * skirt * (1 + 0.18 * (g - 0.5) * np.minimum(1, r / 2500))
        m = m + 330 * np.exp(-((x - self.hoei[0]) ** 2 + (y - self.hoei[1]) ** 2) / 1100 ** 2) \
              - 380 * np.exp(-((x - self.hoei_c[0]) ** 2 + (y - self.hoei_c[1]) ** 2) / 600 ** 2)
        m = np.maximum(0, m)
        inside = r < self.MR
        return np.where(inside, m, 0.0), np.where(inside, g, 0.5)

    def height(self, x, y, d):
        ramp = smooth(self.bar + 25, self.bar + 350, d)
        mx, my = x - self.mid[0], y - self.mid[1]
        amp = 1 + 0.8 * smooth(1500, 7000, np.hypot(mx, my))
        hill = 34 * (fbmw(x, y, 110, 4, 301) - 0.45) + 10 * (fbmw(x, y, 30, 3, 307) - 0.5)
        mtn = self.mountain(x, y)[0]
        foot = mtn - self.mid_m
        hill = hill * (1 - smooth(300, 1200, mtn))
        return -0.4 + ramp * (hill * amp + foot)

    def forest(self, x, y, d):
        return smooth(0.30, 0.44, fbmw(x, y, 400, 4, 321)) * smooth(self.bar + 18, self.bar + 34, d)

    def sample(self, F, x, y):
        i = np.clip(np.searchsorted(self.xs, x, 'right') - 1, 0, len(self.xs) - 2)
        j = np.clip(np.searchsorted(self.ys, y, 'right') - 1, 0, len(self.ys) - 2)
        fx = np.clip((x - self.xs[i]) / (self.xs[i + 1] - self.xs[i]), 0, 1)
        fy = np.clip((y - self.ys[j]) / (self.ys[j + 1] - self.ys[j]), 0, 1)
        a, b, c, e = F[j, i], F[j, i + 1], F[j + 1, i], F[j + 1, i + 1]
        return (a + (b - a) * fx) + ((c + (e - c) * fx) - (a + (b - a) * fx)) * fy

    def ground(self, x, y, h, d):
        m, g = self.mountain(x, y)
        tone = fbmw(x, y, 60, 3, 331)
        meadow = lerp(np.array([0.30, 0.41, 0.18]), np.array([0.46, 0.46, 0.26]), smooth(0.35, 0.75, tone)[..., None])
        canopy = np.array([0.12, 0.19, 0.11]) * (0.75 + 0.5 * fbmw(x, y, 14, 3, 333))[..., None]
        f = np.maximum(self.forest(x, y, d), smooth(150, 500, m) * 0.9) * (1 - smooth(1700, 1900, m))
        c = lerp(meadow, canopy, f[..., None])
        c = lerp(np.array([0.27, 0.38, 0.17]), c, smooth(self.bar + 10, self.bar + 30, d)[..., None])
        n = fbmw(x, y, 80, 3, 337)
        c = lerp(c, np.array([0.34, 0.31, 0.22]), smooth(1700, 1900, m + 120 * (n - 0.5))[..., None])
        rock = np.array([0.30, 0.24, 0.23]) * (0.8 + 0.4 * g)[..., None]
        c = lerp(c, rock, smooth(1900, 2100, m + 120 * (n - 0.5))[..., None])
        sl = 1650 - 1300 * (g - 0.5) + 120 * (n - 0.5)
        c = lerp(c, np.array([0.94, 0.95, 0.98]), smooth(sl, sl + 40, m)[..., None])
        return c * lerp(1, 0.80 + 0.4 * g, smooth(400, 900, m))[..., None]

    def color_map(self, size, cache=None):
        if cache and os.path.exists(cache):
            return np.load(cache)
        x0, x1, y0, y1 = self.xs[0], self.xs[-1], self.ys[0], self.ys[-1]
        out = np.zeros((size, size, 3))
        xx = x0 + (np.arange(size) + 0.5) / size * (x1 - x0)
        for j0 in range(0, size, 256):
            yy = y0 + (np.arange(j0, min(size, j0 + 256)) + 0.5) / size * (y1 - y0)
            X, Y = np.meshgrid(xx, yy)
            out[j0:j0 + len(yy)] = self.ground(X, Y, self.sample(self.H, X, Y), self.sample(self.D, X, Y))
        if cache:
            np.save(cache, out)
        return out

    def trees(self):
        b, G = self.b, self.TREE_GRID
        x0, y0, x1, y1 = b[0] - self.INNER, b[1] - self.INNER, b[2] + self.INNER, b[3] + self.INNER
        nx, ny = math.ceil((x1 - x0) / G), math.ceil((y1 - y0) / G)
        J, I = np.mgrid[0:ny, 0:nx]
        x = x0 + (I + 0.5 + 0.8 * (hash2(I, J, 401) - 0.5)) * G
        y = y0 + (J + 0.5 + 0.8 * (hash2(I, J, 402) - 0.5)) * G
        d = self.sample(self.D, x, y)
        keep = d >= self.bar + 22 + 25 * hash2(I, J, 403)
        keep &= np.hypot(x - self.start[0], y - self.start[1]) >= 380
        f = self.forest(x, y, d)
        keep &= hash2(I, J, 404) < np.maximum(f * 0.92, 0.015)
        conif = smooth(0.38, 0.62, fbmw(x, y, 600, 2, 341))
        c = hash2(I, J, 405) < conif
        s = hash2(I, J, 406)
        h = np.where(c, 12 + 10 * s, 9 + 8 * s) * np.where(f < 0.3, 1.15, 1.0)
        w = h * 0.5 * (0.9 + 0.2 * hash2(I, J, 407))
        spr = np.where(c, 3, 0) + np.minimum(2, (hash2(I, J, 409) * 3).astype(int))
        z = self.sample(self.H, x, y) - 0.5
        k = keep
        return np.stack([x[k], y[k], z[k], h[k], w[k], spr[k]], -1)


# ------------------------------------------------------------ コースの幾何 (CourseBuilder.Circuit と同じ規則)
class Track:
    def __init__(self, data):
        f = np.array(data['centerline_shortcut'], float).reshape(-1, 2)
        if np.hypot(*(f[0] - f[-1])) < 1e-6:
            f = f[:-1]
        self.C = f
        n = len(f)
        t = np.roll(f, -1, 0) - np.roll(f, 1, 0)
        t /= np.linalg.norm(t, axis=1, keepdims=True)
        self.T, self.N = t, np.stack([-t[:, 1], t[:, 0]], 1)
        seg = np.linalg.norm(np.roll(f, -1, 0) - f, axis=1)
        self.S = np.concatenate([[0], np.cumsum(seg)])
        self.total = self.S[-1]
        K = np.zeros(n)
        for i in range(n):
            a, p, q = f[i - 3], f[i], f[(i + 3) % n]
            ab, bc, ca = np.linalg.norm(p - a), np.linalg.norm(q - p), np.linalg.norm(a - q)
            cr = (p[0] - a[0]) * (q[1] - a[1]) - (q[0] - a[0]) * (p[1] - a[1])
            K[i] = 2 * cr / (ab * bc * ca)
        self.K = K
        c = data['circuit']
        self.half, self.bar, self.kw = c['width_m'] / 2, c['width_m'] / 2 + c['runoff_m'], c['kerb_width_m']
        self.kmin = c['kerb_min_curvature']
        idx = np.arange(n)
        win = lambda w: np.array([np.abs(K[(idx + d) % n]) for d in range(-w, w + 1)]).max(0)
        self.k3, self.k8 = win(3), win(8)
        self.ksign = np.sign(np.array([K[(idx + d) % n] for d in range(-8, 9)]).sum(0))
        step = self.total / n
        w = max(1, round(60 / step))
        sm = np.array([K[(idx + d) % n] for d in range(-w, w + 1)]).mean(0)
        o = np.clip(sm * 450, -(self.half - 2.5), self.half - 2.5)
        self.line = np.array([o[(idx + d) % n] for d in range(-10, 11)]).mean(0)
        self.brake = np.zeros(n, bool)
        for e in self.entries():
            ds = (self.S[e] - self.S[:-1]) % self.total
            self.brake |= ds <= 150
        # 光線の当たった点から中心線への最近点 (0.5 m 刻みに細かくして探す)
        sub = 4
        P, I, F = [], [], []
        for k in range(sub):
            fr = k / sub
            P.append(f + (np.roll(f, -1, 0) - f) * fr)
            I.append(idx)
            F.append(np.full(n, fr))
        self.P, self.I, self.F = np.vstack(P), np.concatenate(I), np.concatenate(F)
        self.tree = cKDTree(self.P)

    def entries(self):
        n = len(self.C)
        run, out = 0.0, []
        for k in range(2 * n):
            i = k % n
            ds = self.S[i + 1] - self.S[i]
            if abs(self.K[i]) < 1 / 600:
                run += ds
            else:
                if run >= 300 and k >= n and i not in out:
                    out.append(i)
                run = 0
        return out

    def locate(self, xy):
        _, j = self.tree.query(xy)
        i, fr = self.I[j], self.F[j]
        p = self.C[i] + (self.C[(i + 1) % len(self.C)] - self.C[i]) * fr[:, None]
        d = ((xy - p) * self.N[i]).sum(1)
        s = self.S[i] + (self.S[i + 1] - self.S[i]) * fr + ((xy - p) * self.T[i]).sum(1)
        return i, d, s

    def pose(self, s):
        s %= self.total
        i = int(np.searchsorted(self.S, s, side='right') - 1)
        fr = (s - self.S[i]) / max(1e-6, self.S[i + 1] - self.S[i])
        p = self.C[i] + (self.C[(i + 1) % len(self.C)] - self.C[i]) * fr
        return i, p, self.T[i]


# ------------------------------------------------------------ 描画
STYLE = {'real_rx7': 'BuildRx7', 'real_nd': 'BuildRoadster', 'real_b787': 'BuildB787'}
SUN = np.array([0.42, 0.64, 0.643])      # CourseBuilder.SunDir (富士山の反対側の空・仰角 40°)
SUN /= np.linalg.norm(SUN)
FOG = np.array([0.72, 0.79, 0.86])
FOG_DENSITY = 0.000055                   # Unity: 指数のかすみ (CourseBuilder.BuildOutdoorLighting)


def fogk(d):
    return 1 - np.exp(-FOG_DENSITY * np.asarray(d, float))


def render(data, s_car, view, W, H, ss=2):
    tr = Track(data)
    land = Landscape(tr, data['circuit']['bounds'])
    W2, H2 = W * ss, H * ss
    i, p, t = tr.pose(s_car)
    nrm = np.array([-t[1], t[0]])
    car_xy = p + nrm * tr.line[i] * 0.85
    veh = data.get('vehicle', {})
    L = veh.get('length_m', 4.3)
    k = L / {'real_rx7': 4.289 * 0.257 / 2.425, 'real_nd': 3.939 * 0.257 / 2.310, 'real_b787': 4.826 * 0.257 / 2.662}.get(veh.get('name', ''), 0.45)
    if view == 'chase':
        eye = np.array([*(car_xy - t * 0.95 * k + nrm * -0.25 * k), 0.42 * k])
        look = np.array([*(car_xy + t * 0.35 * k), 0.06 * k])
        vfov = 50.0
    elif view in ('grandstand', 'panasonic'):
        # 名所: メインスタンドの上段からピット越しの富士山 / 最終のパナソニックコーナーの外から、コーナーと富士山
        n0 = tr.N[0]
        inside = 1 if ((tr.C.mean(0) - tr.C[0]) @ n0) > 0 else -1
        if view == 'grandstand':
            eye = np.array([*(tr.C[0] - n0 * inside * (tr.bar + 15)), 8.0])
        else:
            pc = [c for c in data['circuit']['corners'] if 'パナソニック' in c['name']][0]
            corner = np.array([pc['x'], pc['y']])
            eye = np.array([*(corner - land.FUJI * 70 + np.array([-land.FUJI[1], land.FUJI[0]]) * 25), 4.0])
        look = np.array([*(land.peak), land.MH * 0.32])
        vfov = 46.0
    elif view == 'scenic':        # 景色の確認用: コースの脇から遠景 (山の方向) を見る
        eye = np.array([*(car_xy - t * 8 + nrm * 6), 2.5])
        look = np.array([*(land.peak), land.MH * 0.40])
        vfov = 50.0
    else:
        cam = data['camera']
        eye = np.array([*(car_xy + t * 1.5), cam['mount_height_m']])
        look = eye + np.array([*t, -math.tan(math.radians(cam['pitch_deg']))])
        vfov = 2 * math.degrees(math.atan(math.tan(math.radians(cam.get('fov_deg', 100)) / 2)))
    fwd = look - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    f = (H2 / 2) / math.tan(math.radians(vfov) / 2)
    ys, xs = np.mgrid[0:H2, 0:W2]
    dirs = (fwd[None, None] * f + right[None, None] * (xs - W2 / 2)[..., None] - up[None, None] * (ys - H2 / 2)[..., None])
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    D = dirs.reshape(-1, 3)
    NP = len(D)
    img = np.zeros((NP, 3))
    zbuf = np.full(NP, np.inf)

    # ---- 地形に光線を当てる: 方位を細かく分け、方位ごとに手前から高さを調べる (仰角の累積最大と比べる)
    hn = np.hypot(D[:, 0], D[:, 1])
    tan_e = D[:, 2] / hn
    az0 = math.atan2(fwd[1], fwd[0])
    rel = (np.arctan2(D[:, 1], D[:, 0]) - az0 + np.pi) % (2 * np.pi) - np.pi
    NB = 3000
    lo, hi = rel.min(), rel.max() + 1e-9
    bi = np.clip(((rel - lo) / (hi - lo) * NB).astype(int), 0, NB - 1)
    baz = az0 + lo + (np.arange(NB) + 0.5) / NB * (hi - lo)
    r = 0.5 * 1.006 ** np.arange(int(math.log(60000) / math.log(1.006)) + 1)
    PX = eye[0] + np.cos(baz)[:, None] * r[None]
    PY = eye[1] + np.sin(baz)[:, None] * r[None]
    hh = land.sample(land.H, PX, PY)
    M = np.maximum.accumulate((hh - eye[2]) / r[None], axis=1)
    order = np.argsort(bi, kind='stable')
    starts = np.searchsorted(bi[order], np.arange(NB + 1))
    rr = np.full(NP, np.nan)
    for bb in range(NB):
        idx = order[starts[bb]:starts[bb + 1]]
        if len(idx) == 0:
            continue
        te = tan_e[idx]
        kk = np.searchsorted(M[bb], te)
        hit = kk < len(r)
        k1 = np.clip(kk, 1, len(r) - 1)
        r0, r1 = r[k1 - 1], r[k1]
        f0 = hh[bb, k1 - 1] - eye[2] - r0 * te
        f1 = hh[bb, k1] - eye[2] - r1 * te
        x = np.where(kk == 0, r[0], r0 + (r1 - r0) * np.clip(-f0 / np.where(f1 - f0 == 0, 1, f1 - f0), 0, 1))
        rr[idx[hit]] = x[hit]
    hitm = ~np.isnan(rr)
    th = np.where(hitm, rr / np.maximum(hn, 1e-9), np.inf)
    HP = eye[None] + D * np.where(hitm, th, 0)[:, None]

    # ---- 空と雲 (雲はドームの絵。光線の当たらない所だけ)
    sky_m = ~hitm
    el = np.clip(D[:, 2], -1, 1)
    sky = lerp(np.array([0.80, 0.86, 0.92]), np.array([0.36, 0.56, 0.80]), np.clip(el * 3.0, 0, 1)[:, None])
    C = np.array([*land.mid, -200.0])
    oc = eye - C
    bq = D @ oc
    tq = -bq + np.sqrt(np.maximum(0, bq * bq - (oc @ oc - land.SKY_R ** 2)))
    Q = eye[None] + D * tq[:, None] - C
    cu = (np.arctan2(Q[:, 1], Q[:, 0]) / (2 * np.pi)) % 1.0
    cv = np.arcsin(np.clip(Q[:, 2] / land.SKY_R, -1, 1)) / (np.pi / 2)
    CT = clouds_tex(77)
    cs = np.zeros((NP, 4))
    up_m = sky_m & (cv > 0) & (cv < 0.98)
    cs[up_m] = sample(CT, cu[up_m], cv[up_m])
    img[sky_m] = lerp(sky[sky_m], cs[sky_m, :3], cs[sky_m, 3:4])

    # ---- 平らな帯 (コース・ランオフ) は z = 0.02 の面で取り直す
    near = hitm & (land.sample(land.D, HP[:, 0], HP[:, 1]) < tr.bar + 3) & (D[:, 2] < -1e-4)
    tp = np.where(near, (0.02 - eye[2]) / np.where(near, D[:, 2], -1), 0)
    gp_all = eye[None, :2] + D[:, :2] * tp[:, None]
    flat = np.zeros(NP, bool)
    if near.any():
        _, dd_, _ = tr.locate(gp_all[near])
        sub = np.abs(dd_) < tr.bar
        nidx = np.nonzero(near)[0]
        flat[nidx[sub]] = True
    terr = hitm & ~flat
    print(f'  地形 {terr.sum()} 点・コース {flat.sum()} 点・空 {sky_m.sum()} 点')

    # ---- 地形: 色の地図 × 芝の細部 (×2)。法線は地形の傾き + 近くは芝の凹凸
    x, y = HP[terr, 0], HP[terr, 1]
    dist = th[terr]
    cache = os.path.join(os.environ.get('TMPDIR', '/tmp'), f'preview_landmap_{abs(hash(tuple(np.round(land.b, 2))))}.npy')
    CM = land.color_map(2048, cache)
    u0 = (x - land.xs[0]) / (land.xs[-1] - land.xs[0])
    v0 = (y - land.ys[0]) / (land.ys[-1] - land.ys[0])
    col = sample(CM, np.clip(u0, 0, 0.9999), np.clip(v0, 0, 0.9999))
    GDa, GDh = grass_detail(512, 21)
    GDn = normals(GDh, 1.5)
    fade = np.clip(dist / 120, 0, 1)
    det = lerp(sample(GDa, x / 12, y / 12), 0.535, fade)
    col = col * (2 * det)[:, None]
    e = np.clip(dist * 0.004, 1, 60)
    hxp, hxm = land.sample(land.H, x + e, y), land.sample(land.H, x - e, y)
    hyp, hym = land.sample(land.H, x, y + e), land.sample(land.H, x, y - e)
    nt = np.stack([-(hxp - hxm) / (2 * e), -(hyp - hym) / (2 * e), np.ones_like(x)], -1)
    dn = sample(GDn, x / 12, y / 12)
    nt[:, :2] += dn[:, :2] * 0.6 * (1 - fade)[:, None]
    nt /= np.linalg.norm(nt, axis=1, keepdims=True)
    lit = col * (0.48 + 0.78 * np.clip(nt @ SUN, 0, 1))[:, None]
    img[terr] = lerp(lit, FOG, fogk(dist)[:, None])
    zbuf[terr] = dist

    # ---- コースの帯 (CourseBuilder.Circuit と同じ規則)
    gp = gp_all[flat]
    dist = tp[flat]
    ii, d, s = tr.locate(gp)
    ad = np.abs(d)
    A1, H1 = asphalt(512, 11, 0.22)
    A2, H2h = asphalt(512, 31, 0.30)
    VA, VH = gravel(256, 41)
    KA, KH = kerb(128, 256, 71)
    LA, RB = line_tex(61), rubber_tex(51)
    N1, N2, NV, NK = normals(H1, 2.2), normals(H2h, 2.2), normals(VH, 3.0), normals(KH, 1.2)
    col = np.zeros((len(gp), 3))
    nm = np.zeros((len(gp), 3))
    nm[:, 2] = 1
    smooth_ = np.zeros(len(gp))

    def put(mask, alb, nmap, u, v, bumpk, sm):
        if not mask.any():
            return
        col[mask] = sample(alb, u[mask], v[mask])
        if nmap is not None:
            nn = sample(nmap, u[mask], v[mask])
            nn[:, :2] *= bumpk
            nm[mask] = nn / np.linalg.norm(nn, axis=1, keepdims=True)
        smooth_[mask] = sm
    run = ad >= tr.half
    put(run, A2, N2, d / 6, s / 6, 0.5, 0.12)
    grv = run & (ad > tr.half + tr.kw + 1) & (ad < tr.bar - 1) & (tr.k8[ii] > 1 / 140) & (np.sign(d) == -tr.ksign[ii])
    put(grv, VA, NV, d / 4, s / 4, 0.9, 0.02)
    trk = ad < tr.half
    put(trk, A1, N1, d / 6, s / 6, 0.8, 0.22)
    lu = (d - (tr.line[ii] - 1.3)) / 2.6
    on = trk & (lu > 0) & (lu < 1)
    a = np.zeros(len(gp))
    a[on] = sample(RB, lu[on], s[on] / 60) * 0.45
    lu2 = (d - (tr.line[ii] - 1.1)) / 2.2
    on2 = trk & (lu2 > 0) & (lu2 < 1) & tr.brake[ii]
    a2 = np.zeros(len(gp))
    a2[on2] = sample(RB, lu2[on2], s[on2] / 60 + 0.37)
    for aa in (a, a2):
        col = col * (1 - aa[:, None]) + 0.05 * aa[:, None]
    ln = (ad > tr.half - 0.30) & (ad < tr.half - 0.05)
    col[ln] = sample(LA, (ad[ln] - tr.half + 0.30) / 0.25, s[ln] / 8)
    smooth_[ln] = 0.35
    kb = (ad >= tr.half) & (ad < tr.half + tr.kw) & (tr.k3[ii] >= tr.kmin)
    put(kb, KA, NK, (ad - tr.half) / tr.kw, s / 6, 1.0, 0.35)
    cl_m = ((np.abs(s % tr.total) < 0.6) | (np.abs(s % tr.total - tr.total) < 0.6)) & trk
    chk = ((np.floor((d + tr.half) / 0.6) + np.floor((s % tr.total + 0.6) / 0.6)) % 2 == 0)
    col[cl_m] = np.where(chk[cl_m, None], 0.9, 0.08)
    ndl = np.clip(nm @ SUN, 0, 1)
    hv = SUN[None] - D[flat]
    hv /= np.linalg.norm(hv, axis=1, keepdims=True)
    spec = np.clip((nm * hv).sum(1), 0, 1) ** (8 + 120 * smooth_) * smooth_ * 0.6
    lit = col * (0.48 + 0.78 * ndl)[:, None] + spec[:, None]
    img[flat] = lerp(lit, FOG, fogk(dist)[:, None])
    zbuf[flat] = dist
    img = img.reshape(H2, W2, 3)
    zbuf = zbuf.reshape(H2, W2)

    def proj(P):
        q = np.asarray(P, float) - eye
        z = q @ fwd
        return np.stack([W2 / 2 + (q @ right) / z * f, H2 / 2 - (q @ up) / z * f], -1), z

    # ---- 多角形 (観客席・ピット・ガードレール): 奥から塗り、深度も別の層に描いて地形と比べる
    polys = []
    n = len(tr.C)
    t0, n0 = tr.T[0], tr.N[0]
    inside = 1 if ((tr.C.mean(0) - tr.C[0]) @ n0) > 0 else -1
    out = -inside
    for seg in range(-13, 13):
        a0, a1 = seg * 10.0, (seg + 1) * 10.0
        for rr_ in range(14):
            d0 = tr.bar + 6 - 0.45 + rr_ * 0.9
            z = 0.4 + rr_ * 0.45
            base = tr.C[0] + n0 * out * d0
            P = [[*(base + t0 * a0), z], [*(base + t0 * a1), z], [*(base + n0 * out * 0.9 + t0 * a1), z], [*(base + n0 * out * 0.9 + t0 * a0), z]]
            dd = np.hypot(*(base + t0 * (a0 + a1) / 2 - eye[:2]))
            polys.append((dd + 0.001 * rr_, P, 'crowd'))
            F = [[*(base + t0 * a0), z - 0.45], [*(base + t0 * a1), z - 0.45], [*(base + t0 * a1), z], [*(base + t0 * a0), z]]
            polys.append((dd + 0.001 * rr_ - 0.0005, F, np.array([0.55, 0.55, 0.53])))
        rb = tr.C[0] + n0 * out * (tr.bar + 4)
        R = [[*(rb + t0 * a0), 10.5], [*(rb + t0 * a1), 10.5], [*(rb + n0 * out * 16 + t0 * a1), 10.5], [*(rb + n0 * out * 16 + t0 * a0), 10.5]]
        polys.append((np.hypot(*(rb + t0 * a0 - eye[:2])) - 5, R, np.array([0.62, 0.64, 0.67])))
    for seg in range(-15, 15):
        a0, a1 = seg * 10.0, (seg + 1) * 10.0
        fb = tr.C[0] + n0 * inside * (tr.bar + 15)
        Fw = [[*(fb + t0 * a0), 0], [*(fb + t0 * a1), 0], [*(fb + t0 * a1), 9], [*(fb + t0 * a0), 9]]
        Dw = [[*(fb + t0 * a0 - n0 * inside * 0.1), 0], [*(fb + t0 * a1 - n0 * inside * 0.1), 0], [*(fb + t0 * a1 - n0 * inside * 0.1), 4.2], [*(fb + t0 * a0 - n0 * inside * 0.1), 4.2]]
        dd = np.hypot(*(fb + t0 * a0 - eye[:2]))
        polys.append((dd, Fw, np.array([0.70, 0.71, 0.73])))
        polys.append((dd - 0.01, Dw, np.array([0.16, 0.18, 0.21])))
    for side in (1, -1):
        Qb = tr.C + tr.N * side * tr.bar
        for i0 in range(n):
            dd = np.hypot(*(Qb[i0] - eye[:2]))
            if dd > 900:
                continue
            i1 = (i0 + 1) % n
            for (h0, h1, g0) in ((0.0, 0.35, 0.30), (0.35, 0.64, 0.74), (0.64, 0.92, 0.58), (0.92, 1.0, 0.30)):
                P = [[*Qb[i0], h0], [*Qb[i1], h0], [*Qb[i1], h1], [*Qb[i0], h1]]
                polys.append((dd, P, lerp(np.array([g0, g0 * 1.01, g0 * 1.04]) * 1.05, FOG, float(fogk(dd)))))
    polys.sort(key=lambda q: -q[0])
    rng = np.random.default_rng(91)
    crowd = (np.stack([rng.uniform(0.2, 0.95, (H2, W2)) for _ in range(3)], -1) * 255).astype(np.uint8)
    crowd[rng.uniform(size=(H2, W2)) < 0.25] = (140, 142, 146)
    crowd_im = Image.fromarray(crowd).resize((W2 // 3, H2 // 3), Image.NEAREST).resize((W2, H2), Image.NEAREST)
    layer = Image.new('RGB', (W2, H2), (0, 0, 0))
    dlay = Image.new('F', (W2, H2), float('inf'))
    dr, ddr = ImageDraw.Draw(layer), ImageDraw.Draw(dlay)
    for dd, P, c in polys:
        sp, z = proj(P)
        if (z < 0.5).any():
            continue
        if (sp[:, 0] < -W2).all() or (sp[:, 0] > 2 * W2).all() or (sp[:, 1] < -H2).all() or (sp[:, 1] > 2 * H2).all():
            continue
        pts = [tuple(q) for q in sp]
        ddr.polygon(pts, fill=float(np.linalg.norm(np.asarray(P) - eye, axis=1).min()))
        if isinstance(c, str):
            m = Image.new('L', (W2, H2), 0)
            ImageDraw.Draw(m).polygon(pts, fill=255)
            layer.paste(crowd_im, (0, 0), m)
            continue
        dr.polygon(pts, fill=tuple(int(v) for v in np.clip(np.array(c) * 255, 0, 255)))
    pd = np.asarray(dlay)
    pm = pd < zbuf
    img[pm] = np.asarray(layer).astype(float)[pm] / 255
    zbuf[pm] = pd[pm]

    # ---- 木 (Landscape.Trees と同じ位置・同じ絵)。カメラに向けた板として奥から描き、深度で隠す
    T = land.trees()
    rel = T[:, :2] - eye[:2]
    dist_t = np.hypot(rel[:, 0], rel[:, 1])
    vis = (dist_t < 4000) & ((rel @ fwd[:2]) > 1)
    T, dist_t = T[vis], dist_t[vis]
    T = T[np.argsort(-dist_t)]
    sprites = [tree_tex(0 if kk < 3 else 1, 500 + kk)[::-1] for kk in range(6)]
    spr_im = [Image.fromarray((np.clip(sp_, 0, 1) * 255).astype(np.uint8), 'RGBA') for sp_ in sprites]
    tl = 0.48 + 0.78 * max(0.0, SUN[2])
    drawn = 0
    for (tx, ty, tz, h, w, spn) in T:
        bp, bz = proj([[tx, ty, tz]])
        tp_, _ = proj([[tx, ty, tz + h]])
        if bz[0] < 1:
            continue
        x0, yb, yt = bp[0, 0], bp[0, 1], tp_[0, 1]
        wp = w * f / bz[0]
        hp = yb - yt
        X0, X1 = int(round(x0 - wp / 2)), int(round(x0 + wp / 2))
        Y0, Y1 = int(round(yt)), int(round(yb))
        if X1 <= X0:
            X1 = X0 + 1
        if Y1 <= Y0:
            Y1 = Y0 + 1
        if X1 < 0 or X0 >= W2 or Y1 < 0 or Y0 >= H2:
            continue
        sim = np.asarray(spr_im[int(spn)].resize((X1 - X0, Y1 - Y0), Image.BOX)).astype(float) / 255
        cx0, cy0 = max(0, X0), max(0, Y0)
        cx1, cy1 = min(W2, X1), min(H2, Y1)
        sub = sim[cy0 - Y0:cy1 - Y0, cx0 - X0:cx1 - X0]
        dpt = math.sqrt(bz[0] ** 2 + (h / 2) ** 2)
        m = (sub[..., 3] >= 0.45) & (zbuf[cy0:cy1, cx0:cx1] > dpt)
        if not m.any():
            continue
        c = lerp(sub[..., :3] * tl, FOG, float(fogk(dpt)))
        reg = img[cy0:cy1, cx0:cx1]
        reg[m] = c[m]
        zbuf[cy0:cy1, cx0:cx1][m] = dpt
        drawn += 1
    print(f'  木 {drawn} 本')
    im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    if view == 'chase':
        draw_car(im, proj, eye, car_xy, t, nrm, k, STYLE.get(veh.get('name', ''), 'BuildRx7'))
    im = im.resize((W, H), Image.LANCZOS)
    yy, xx = np.mgrid[0:H, 0:W]
    vg = 1 - 0.22 * (((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2) / 2
    arr = np.asarray(im).astype(float) * vg[..., None]
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8)), tr


CARMODEL = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'unity', 'MinicarSim', 'Assets', 'Minicar', 'Scripts', 'CarModel.cs')
PAINT = {'BuildRx7': (0.97, 0.745, 0.055), 'BuildRoadster': (0.66, 0.015, 0.04), 'BuildB787': (0.96, 0.42, 0.06)}
METAL = {'BuildRx7': 0.25, 'BuildRoadster': 0.80, 'BuildB787': 0.15}     # CarModel の下地の金属感 (ソウルレッドは 0.8)
MATS = {'yellow': None, 'red': None, 'orange': None, 'paint': None, 'green': (0.05, 0.55, 0.30), 'black': (0.03, 0.03, 0.03),
        'glass': (0.04, 0.05, 0.07), 'lamp': (0.95, 0.95, 0.90), 'tail': (0.75, 0.05, 0.05), 'alu': (0.55, 0.56, 0.58), 'seat': (0.12, 0.11, 0.11)}


SHELLMODEL = CARMODEL.replace('CarModel.cs', 'CarModel.Shell.cs')
SHELL_K = 6                      # CarModel.Shell: 1 区間の分割 (K)・ベジェ 7 本・半断面 43 点
SHELL_NU = 7 * SHELL_K + 1


def _f(t):
    return float(t.strip().rstrip('f'))


def shell_spec(method):
    """CarModel.Shell.cs から、シェルの車体 (断面の表 St(…)・縮尺・車軸・アーチ) を読む。無ければ None"""
    src = open(SHELLMODEL, encoding='utf-8').read()
    m = re.search(r'void %s\(.*?new Shell\((\w+), k, new\[\] \{([^}]*)\}, ([\d.]+)f, TireRadius / k, ([\d.]+)f\)' % method, src, re.S)
    if not m:
        return None
    tab = re.search(r'%s =\s*\{(.*?)\n        \};' % m.group(1), src, re.S).group(1)
    secs = [[_f(x) for x in v.split(',')] for v in re.findall(r'St\(([^)]*)\)', tab)]
    kc = {'BuildRx7': 'kRx7Scale', 'BuildRoadster': 'kNdScale', 'BuildB787': 'kB787Scale'}[method]
    sc = re.search(r'const float %s = ([\d.]+)f / ([\d.]+)f' % kc, src)
    scale = float(sc.group(1)) / float(sc.group(2))
    tr = re.search(r'k = %s; tireR = ([\d.]+)f' % kc, src)          # RealDims
    return dict(secs=secs, scale=scale, axles=[_f(x) for x in m.group(2).split(',')], arch_r=float(m.group(3)),
                arch_y=float(tr.group(1)), well_x=float(m.group(4)))


class Shell:
    """CarModel.Shell と同じ式 (実車の m)。断面 = 制御点 9 個の 2 次 B スプライン、前後は単調な 3 次補間、端は丸く閉じる"""

    def __init__(self, sp):
        self.__dict__.update(sp)
        S = np.array(self.secs)
        self.S = S
        n = len(S)
        d = (S[1:, 1:] - S[:-1, 1:]) / (S[1:, :1] - S[:-1, :1])
        T = np.zeros((n, S.shape[1] - 1))
        T[0], T[-1] = d[0], d[-1]
        for k in range(1, n - 1):
            ok = d[k - 1] * d[k] > 0
            T[k] = np.where(ok, 2 * d[k - 1] * d[k] / np.where(ok, d[k - 1] + d[k], 1), 0)
        self.T = T

    def controls(self, z, f):
        S, T = self.S, self.T
        z = min(max(z, S[0, 0]), S[-1, 0])
        k = 0
        while k < len(S) - 2 and z > S[k + 1, 0]:
            k += 1
        h = S[k + 1, 0] - S[k, 0]
        t = (z - S[k, 0]) / h
        v = ((2 * t ** 3 - 3 * t ** 2 + 1) * S[k, 1:] + (t ** 3 - 2 * t ** 2 + t) * h * T[k]
             + (-2 * t ** 3 + 3 * t ** 2) * S[k + 1, 1:] + (t ** 3 - t ** 2) * h * T[k + 1])
        yb, wb, w2, y2, w3, y3, w4, y4, w5, y5, w6, y6, y7 = v
        c = np.array([[0, yb], [wb * 0.6, yb], [wb, yb], [w2, y2], [w3, y3], [w4, y4], [w5, y5], [w6, y6], [0, y7]])
        if f < 1:
            mid = np.array([0, (c[0, 1] + c[8, 1]) / 2])
            c = mid + (c - mid) * f
        return c

    @staticmethod
    def profile(c):
        K = SHELL_K
        p = np.zeros((SHELL_NU, 2))
        tt = np.arange(K) / K
        for s in range(7):
            a = c[0] if s == 0 else (c[s] + c[s + 1]) / 2
            e = c[8] if s == 6 else (c[s + 1] + c[s + 2]) / 2
            p[s * K:(s + 1) * K] = ((1 - tt) ** 2)[:, None] * a + (2 * (1 - tt) * tt)[:, None] * c[s + 1] + (tt ** 2)[:, None] * e
        p[-1] = c[8]
        return p

    @staticmethod
    def _resample(src, n):
        seg = np.linalg.norm(np.diff(src, axis=0), axis=1)
        cum = np.concatenate([[0], np.cumsum(seg)])
        d = np.linspace(0, cum[-1], n)
        return np.stack([np.interp(d, cum, src[:, 0]), np.interp(d, cum, src[:, 1])], 1)

    def section(self, z, f):
        K = SHELL_K
        c = self.controls(z, f)
        p = self.profile(c)
        ya = -1.0
        if f >= 1:
            for za in self.axles:
                if abs(z - za) < self.arch_r:
                    ya = self.arch_y + math.sqrt(self.arch_r ** 2 - (z - za) ** 2)
                    break
        if ya < 0:
            return p, 0
        ya = min(ya, c[5, 1] - 0.06)
        i1 = 2 * K
        while i1 < 5 * K and p[i1, 1] < ya:
            i1 += 1
        if i1 <= 2 * K:
            return p, 0
        a, b = p[i1 - 1], p[i1]
        L = a + (b - a) * min(1, max(0, (ya - a[1]) / max(1e-5, b[1] - a[1])))
        lip = 3 * K
        up = np.vstack([L[None], p[i1:5 * K + 1]])
        well = np.array([p[0], [self.well_x - 0.03, p[0, 1]], [self.well_x, ya + 0.03], [L[0] - 0.03, ya + 0.03], L])
        p[lip:5 * K + 1] = self._resample(up, 5 * K - lip + 1)
        p[0:lip + 1] = self._resample(well, lip + 1)
        return p, lip

    def rings(self):
        z0, z1 = self.S[0, 0], self.S[-1, 0]
        zs = list(np.arange(z0, z1, 0.03)) + [z1] + list(self.S[:, 0])
        for za in self.axles:
            zs += [za + self.arch_r * 0.999 * math.cos(math.pi * i / 20) for i in range(21)]
            zs += [za - self.arch_r * 1.001, za + self.arch_r * 1.001]
        zs.sort()
        cf, cd = (0.93, 0.78, 0.52, 0.0), (0.008, 0.016, 0.021, 0.022)
        out = [(z0, z0 - cd[i], cf[i]) for i in (3, 2, 1, 0)]
        prev = -1e9
        for z in zs:
            if z < z0 or z > z1 or z - prev < 0.0005:
                continue
            out.append((z, z, 1.0))
            prev = z
        out += [(z1, z1 + cd[i], cf[i]) for i in range(4)]
        return out


def rx7_mat(z, x, y, u):
    """CarModel.Shell.PaintRx7 の大きな塗り分けだけ (窓・黒い樹脂・尾端の帯)。細い合わせ目は省く"""
    x = abs(x)
    fr = (u - 5.5) / 0.9
    if (5.55 <= u <= 6.35 and -0.06 + fr * 0.42 <= z <= 1.63 - fr * 0.55) or (u > 6.62 and (1.07 <= z <= 1.74 or -0.40 <= z <= 0.33)):
        return 'glass'
    if (z > 2.75 and y < 0.185) or (z < -0.55 and y < 0.27 and x < 0.62):
        return 'black'
    if z > 3.20 and x < 0.27 and 0.225 <= y <= 0.33:
        return 'black'
    if z < -0.86 and 0.625 <= y <= 0.765 and x < 0.72:
        return 'tail' if any(math.hypot(x - (0.27 + 0.17 * k), y - 0.695) < 0.062 for k in (1, 2)) else 'black'
    return 'paint'


def nd_mat(z, x, y, u):
    """CarModel.Shell.PaintNd の大きな塗り分けだけ (窓・黒い幌)"""
    x = abs(x)
    fr = (u - 5.5) / 0.9
    sr = 0.42 + fr * 0.06
    if (5.55 <= u <= 6.35 and sr <= z <= 1.50 - fr * 0.50) or (u > 6.62 and 1.02 <= z <= 1.59):
        return 'glass'
    if u > 5.5 and -0.25 <= z <= 1.02 and not (u < 6.45 and z > sr):
        return 'black'
    if (z > 2.60 and y < 0.185) or (z < -0.45 and y < 0.27 and x < 0.60) or (z > 3.0 and x < 0.32 and 0.235 <= y <= 0.36):
        return 'black'
    if z < -0.70 and math.hypot(x - 0.52, y - 0.70) < 0.075:
        return 'tail'
    return 'paint'


def b787_mat(z, x, y, u):
    """CarModel.Shell.PaintB787 の大きな塗り分けだけ (キャノピー・オレンジと緑の斜めの帯)"""
    if (5.75 <= u <= 6.35 and 1.28 + (u - 5.75) * 0.2 <= z <= 2.10 - (u - 5.75) * 0.5) or (u > 6.50 and 1.72 <= z <= 2.26):
        return 'glass'
    if y < 0.10:
        return 'black'
    ax = abs(x)                                  # 操縦席を囲む緑のひし形・後端の緑の三角・鼻先の緑の帯 (PaintB787 と同じ式)
    d1 = abs(z - 1.45) / (1.35 if z > 1.45 else 1.60) + ax / 1.70
    d2 = abs(z + 1.17) / 1.05 + ax / 1.05
    return 'green' if d1 < 1 or d2 < 1 or z > 3.30 else 'paint'


SHELL_MAT = {'BuildRx7': rx7_mat, 'BuildRoadster': nd_mat, 'BuildB787': b787_mat}


def shell_quads(method):
    """シェルの車体の面 (模型の大きさ)。(面, 材質, 外向きの基準点)"""
    sp = shell_spec(method)
    if sp is None:
        return None
    sh = Shell(sp)
    k = sh.scale
    matf = SHELL_MAT.get(method, lambda *a: 'paint')
    R = []
    for ze, zp, f in sh.rings():
        p, lip = sh.section(ze, f)
        R.append((zp, p, lip))
    out = []
    for j in range(len(R) - 1):
        (za, pa, la), (zb, pb, lb) = R[j], R[j + 1]
        ref = (0.0, (pa[0, 1] + pa[-1, 1]) / 2 * k, (za + zb) / 2 * k)
        for i in range(SHELL_NU - 1):
            mid = (pa[i] + pa[i + 1] + pb[i] + pb[i + 1]) / 4
            mat = 'black' if i < max(la, lb) - 1 else None
            for sx in (1, -1):
                if mat is None or True:
                    mat = 'black' if i < max(la, lb) - 1 else matf((za + zb) / 2, sx * mid[0], mid[1], (i + 0.5) / SHELL_K)
                q = [(sx * pa[i, 0] * k, pa[i, 1] * k, za * k), (sx * pa[i + 1, 0] * k, pa[i + 1, 1] * k, za * k),
                     (sx * pb[i + 1, 0] * k, pb[i + 1, 1] * k, zb * k), (sx * pb[i, 0] * k, pb[i, 1] * k, zb * k)]
                out.append((q, mat, ref))
    return out


def car_parts(method):
    """CarModel.cs の Build<車>() から、断面 (Loft) と箱 (Cube) を読む。Unity と同じ形を描くため"""
    src = open(CARMODEL, encoding='utf-8').read()
    if f'void {method}(' not in src:         # シェルの車体 (CarModel.Shell.cs)。羽根などの小物は省く
        return [], []
    a = src.index(f'void {method}(')
    b = src.index('\n        }\n', a)
    blk = src[a:b]
    vec = lambda t: [tuple(float(x.rstrip('f')) for x in v.split(',')) for v in re.findall(r'new Vector4\(([^)]*)\)', t)]
    arrays = {m.group(1): vec(m.group(2)) for m in re.finditer(r'var (\w+) = new\[\]\s*\{(.*?)\};', blk, re.S)}
    lofts = []
    for m in re.finditer(r'Part\("(\w+)", hull, Loft\((?:new\[\]\s*\{(.*?)\}|(\w+)), ([\d.]+)f, true\), (\w+)\)', blk, re.S):
        secs = vec(m.group(2)) if m.group(2) else arrays[m.group(3)]
        lofts.append((secs, float(m.group(4)), m.group(5)))
    # 曲面の胴 BodyLoft(new[] { S(z, 半幅, 下端, 上端, 峰), ... }, n, archR[, pinch])
    svec = lambda t: [tuple(float(x.rstrip('f')) for x in v.split(',')) for v in re.findall(r'S\(([^)]*)\)', t)]
    sarrays = {m.group(1): svec(m.group(2)) for m in re.finditer(r'var (\w+) = new\[\]\s*\{(.*?)\};', blk, re.S)}
    for m in re.finditer(r'Part\("(\w+)", hull, BodyLoft\((?:new\[\]\s*\{(.*?)\}|(\w+)), ([\d.]+)f, ([\d.]+)f(?:, ([\d.]+)f)?\), (\w+)\)', blk, re.S):
        secs = svec(m.group(2)) if m.group(2) else sarrays[m.group(3)]
        lofts.append(('body', secs, float(m.group(4)), float(m.group(5)), float(m.group(6) or 0), m.group(7)))
    cubes = []
    for m in re.finditer(r'Cube\("(\w+)", hull, new Vector3\(([^)]*)\), new Vector3\(([^)]*)\), (\w+)(?:, (-?[\d.]+)f)?\)', blk):
        pos = [float(x.rstrip('f').replace('sx *', '').strip() or 0) if 'sx' not in x else None for x in m.group(2).split(',')]
        if None in pos:      # foreach (sx) の左右対称
            xs = m.group(2).split(',')[0]
            base = float(re.sub(r'sx\s*\*\s*', '', xs).rstrip('f'))
            rest = [float(x.rstrip('f')) for x in m.group(2).split(',')[1:]]
            for sx in (-1, 1):
                cubes.append(((sx * base, *rest), tuple(float(x.rstrip('f')) for x in m.group(3).split(',')), m.group(4), float(m.group(5) or 0)))
        else:
            cubes.append((tuple(pos), tuple(float(x.rstrip('f')) for x in m.group(3).split(',')), m.group(4), float(m.group(5) or 0)))
    return lofts, cubes


def loft_quads(secs, n_exp, ring=28):
    e = 2.0 / n_exp
    rings = []
    for (z, hw, lo, hi) in secs:
        cy, hh = (lo + hi) / 2, (hi - lo) / 2
        r = []
        for i in range(ring):
            th = i * 2 * math.pi / ring
            c, sn = math.cos(th), math.sin(th)
            r.append((hw * math.copysign(abs(c) ** e, c), cy + hh * math.copysign(abs(sn) ** e, sn), z))
        rings.append(r)
    quads = []
    for j in range(len(rings) - 1):
        for i in range(ring):
            quads.append([rings[j][i], rings[j][(i + 1) % ring], rings[j + 1][(i + 1) % ring], rings[j + 1][i]])
    caps = []
    for j, dz in ((0, -1.0), (len(rings) - 1, 1.0)):
        cz = secs[j][0]
        cyy = (secs[j][2] + secs[j][3]) / 2
        for i in range(ring):
            caps.append(([(0, cyy, cz), rings[j][i], rings[j][(i + 1) % ring]], (0.0, cyy, cz - dz)))
    return quads, caps


def bodyloft_quads(secs, n, arch_r, pinch, wheelbase=0.257, tire_r=0.032, ring=40):
    """CarModel.BodyLoft と同じ曲面の胴 (断面を Catmull-Rom で 4 mm ごとに・肩の峰・上すぼまり・車輪のアーチ)"""
    e = 2.0 / n
    z0, z1 = secs[0][0], secs[-1][0]
    rings = max(2, math.ceil((z1 - z0) / 0.004) + 1)

    def sec_at(z):
        k = 0
        while k < len(secs) - 2 and z > secs[k + 1][0]:
            k += 1
        t = min(1.0, max(0.0, (z - secs[k][0]) / max(1e-6, secs[k + 1][0] - secs[k][0])))
        p0, p1, p2, p3 = secs[max(0, k - 1)], secs[k], secs[k + 1], secs[min(len(secs) - 1, k + 2)]
        cr = lambda i: 0.5 * (2 * p1[i] + (-p0[i] + p2[i]) * t + (2 * p0[i] - 5 * p1[i] + 4 * p2[i] - p3[i]) * t * t
                              + (-p0[i] + 3 * p1[i] - 3 * p2[i] + p3[i]) * t ** 3)
        return max(0.002, cr(1)), p1[2] + (p2[2] - p1[2]) * t, cr(3), max(0.0, cr(4))
    R = []
    cent = []
    for j in range(rings):
        z = z0 + (z1 - z0) * j / (rings - 1)
        hw, lo, hi, hump = sec_at(z)
        if arch_r > 0:
            for axle in (0.0, wheelbase):
                dz = z - axle
                if abs(dz) < arch_r:
                    lo = max(lo, tire_r + math.sqrt(arch_r ** 2 - dz ** 2) * 0.92)
        lo = min(lo, hi - 0.006)
        cy, hh = (lo + hi) / 2, (hi - lo) / 2
        ringv = []
        for i in range(ring):
            th = i * 2 * math.pi / ring
            c, sn = math.cos(th), math.sin(th)
            x = hw * math.copysign(abs(c) ** e, c)
            y = cy + hh * math.copysign(abs(sn) ** e, sn)
            if sn > 0:
                tt = abs(x) / max(1e-5, hw)
                y += hump * math.exp(-((tt - 0.72) / 0.20) ** 2) * sn ** 0.6
                x *= 1 - pinch * sn * sn
            ringv.append((x, y, z))
        R.append(ringv)
        cent.append((0.0, cy, z))
    out = []
    for j in range(rings - 1):
        for i in range(ring):
            q = [R[j][i], R[j][(i + 1) % ring], R[j + 1][(i + 1) % ring], R[j + 1][i]]
            out.append((q, cent[j]))
    for j, dz in ((0, -1.0), (rings - 1, 1.0)):
        m = tuple(sum(v[k] for v in R[j]) / ring for k in range(3))
        for i in range(ring):
            out.append(([m, R[j][i], R[j][(i + 1) % ring]], (m[0], m[1], m[2] - dz)))
    return out


def box_quads(pos, size, pitch):
    hx, hy, hz = (v / 2 for v in size)
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    def P(x, y, z):          # x 軸回りに pitch (Unity の Euler(pitch,0,0))
        y2, z2 = y * cp - z * sp, y * sp + z * cp
        return (pos[0] + x, pos[1] + y2, pos[2] + z2)
    v = [P(x, y, z) for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)]
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [[v[i] for i in q] for q in f]


def draw_car(im, proj, eye, xy, t, nrm, k, style):
    """CarModel の断面を拡大して、太陽の向きで陰影をつけて描く (奥から順に)。下に柔らかい影"""
    lofts, cubes = car_parts(style)
    shell = shell_quads(style)
    paint = np.array(PAINT.get(style, (0.6, 0.6, 0.6)))
    def W(p):                # 車体座標 (x 右・y 上・z 前、後軸の真下が原点) → 世界
        x, y, z = p
        w = xy + t * z * k - nrm * x * k
        return np.array([w[0], w[1], y * k])
    items = []                # (面, 材質, 外向きの基準点 [車体座標]): 面の重心から基準点を引いた向きが外
    if shell:
        items += shell
    for lf in lofts:
        if lf[0] == 'body':
            _, secs, nexp, arch_r, pinch, mat = lf
            for q, ref in bodyloft_quads(secs, nexp, arch_r, pinch):
                items.append((q, mat, ref))
            continue
        secs, nexp, mat = lf
        quads, caps = loft_quads(secs, nexp)
        for q in quads:
            zc = sum(p[2] for p in q) / len(q)
            j = min(range(len(secs)), key=lambda k: abs(secs[k][0] - zc))
            items.append((q, mat, (0.0, (secs[j][2] + secs[j][3]) / 2, zc)))
        for q, ref in caps:              # 蓋は前後方向が外 (基準点を内側へずらしてある)
            items.append((q, mat, ref))
    for pos, size, mat, pitch in cubes:
        if mat in ('alu',) or 'Mast' in mat:
            continue
        for q in box_quads(pos, size, pitch):
            items.append((q, mat, pos))
    # タイヤ (前後・左右)
    for zc in (0.0, 0.257):
        for xs in (-0.084, 0.084):
            ring = [(xs + dx, 0.032 + 0.032 * math.sin(a), zc + 0.032 * math.cos(a)) for a in np.linspace(0, 2 * np.pi, 13)[:-1] for dx in (-0.014, 0.014)]
            outer = [r for r in ring if (r[0] - xs) * np.sign(xs) > 0]
            items.append((outer, 'tire', (xs, 0.032, zc)))
            for i in range(12):
                a0, a1 = ring[2 * i], ring[(2 * i + 2) % 24]
                b0, b1 = ring[2 * i + 1], ring[(2 * i + 3) % 24]
                items.append(([a0, a1, b1, b0], 'tire', (xs, 0.032, zc)))
    # 影 (地面に落ちる車体の形を太陽の逆へずらし、ぼかす)
    lay = Image.new('L', im.size, 0)
    ld = ImageDraw.Draw(lay)
    off = -SUN[:2] / SUN[2] * 0.05 * k
    fp = [W((x, 0.0, z)) for x, z in ((-0.1, -0.09), (0.1, -0.09), (0.1, 0.35), (-0.1, 0.35))]
    sp, zz = proj([[*(p[:2] + off), 0.0] for p in fp])
    if (zz > 0.5).all():
        ld.polygon([tuple(q) for q in sp], fill=170)
        lay = lay.filter(ImageFilter.GaussianBlur(radius=max(2, im.size[0] / 300)))
        im.paste(Image.new('RGB', im.size, (12, 12, 14)), (0, 0), lay)
    dr = ImageDraw.Draw(im)
    polys = []
    for q, mat, ref in items:
        P = np.array([W(p) for p in q])
        c = P.mean(0)
        nvec = np.cross(P[1] - P[0], P[2] - P[0])
        if np.linalg.norm(nvec) < 1e-9:
            nvec = np.cross(P[2] - P[0], P[-1] - P[0])
            if np.linalg.norm(nvec) < 1e-9:
                continue
        nvec /= np.linalg.norm(nvec)
        if nvec @ (c - W(ref)) < 0:      # 外向きにそろえる
            nvec = -nvec
        if nvec @ (eye - c) <= 0:        # 裏面は描かない
            continue
        base = paint if MATS.get(mat, 0) is None else np.array(MATS.get(mat, (0.3, 0.3, 0.3)) if mat != 'tire' else (0.05, 0.05, 0.05))
        vdir = (eye - c) / np.linalg.norm(eye - c)
        hv = SUN + vdir
        hv /= np.linalg.norm(hv)
        dif = max(0.0, float(nvec @ SUN))
        gloss = {None: 0.9, 'glass': 1.0}.get(MATS.get(mat, 0) if MATS.get(mat, 0) is None else (None if mat in ('yellow', 'red', 'orange', 'paint') else mat), 0.2)
        spec = max(0.0, float(nvec @ hv)) ** 60 * (0.9 if mat in ('glass',) or MATS.get(mat, 0) is None else 0.15)
        sky = max(0.0, nvec[2]) * (0.35 if mat == 'glass' else 0.12)
        col = base * (0.42 + 0.72 * dif) + spec + sky * np.array([0.55, 0.68, 0.85])
        if MATS.get(mat, 0) is None:
            # 塗装 = 金属的な下地 (色の付いた広いハイライト) + クリア層 (斜めほど空を映す)。CarModel.Paint / AddClearCoat
            mt = METAL.get(style, 0.3)
            glow = max(0.0, float(nvec @ hv)) ** 10 * 1.1 * mt
            fres = 0.04 + 0.96 * (1 - max(0.0, float(nvec @ vdir))) ** 5
            col = base * ((0.42 + 0.72 * dif) * (1 - 0.6 * mt) + glow) + spec + fres * np.array([0.80, 0.86, 0.92]) * 0.8 + sky * np.array([0.55, 0.68, 0.85]) * (1 - mt)
        polys.append((np.linalg.norm(c - eye), P, col))
    # 深度つきで描く (面は 4 mm ほどと小さいので、面ごとの一定の深度で足りる)。奥から順に、手前のものだけを上書き
    polys.sort(key=lambda q: -q[0])
    Wd, Hd = im.size
    zb = np.full((Hd, Wd), np.inf)
    arr = np.asarray(im).copy()
    for dist, P, col in polys:
        sp, z = proj(P)
        if (z < 0.2).any():
            continue
        x0, y0 = np.floor(sp.min(0)).astype(int)
        x1, y1 = np.ceil(sp.max(0)).astype(int) + 1
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(Wd, x1), min(Hd, y1)
        if x1 <= x0 or y1 <= y0:
            continue
        m = Image.new('L', (x1 - x0, y1 - y0), 0)
        ImageDraw.Draw(m).polygon([(q[0] - x0, q[1] - y0) for q in sp], fill=255)
        mk = np.asarray(m) > 0
        d = float(z.mean())
        reg = zb[y0:y1, x0:x1]
        w = mk & (d < reg + 1e-4)
        reg[w] = d
        arr[y0:y1, x0:x1][w] = np.clip(col * 255, 0, 255).astype(np.uint8)
    im.paste(Image.fromarray(arr))


def hud(im, data):
    W, H = im.size
    s = W / 1280
    d = ImageDraw.Draw(im, 'RGBA')

    def font(sz, bold=True):
        for name in ('DejaVuSansCondensed-BoldOblique.ttf', 'DejaVuSans-BoldOblique.ttf', 'DejaVuSans-Bold.ttf'):
            try:
                return ImageFont.truetype(name if bold else name.replace('Bold', ''), int(sz * s))
            except OSError:
                continue
        return ImageFont.load_default()
    ink, dim, ac, warn, good = (244, 246, 248, 255), (244, 246, 248, 160), (46, 230, 200, 255), (255, 77, 61, 255), (56, 224, 123, 255)
    glass = (10, 14, 20, 150)

    def panel(x, y, w, h):
        k = 14 * s
        d.polygon([(x * s + k, y * s), ((x + w) * s, y * s), ((x + w) * s - k, (y + h) * s), (x * s, (y + h) * s)], fill=glass)
    panel(30, 26, 250, 92)
    d.text((52 * s, 34 * s), 'POSITION', font=font(15, False), fill=dim)
    d.text((178 * s, 34 * s), 'LAP', font=font(15, False), fill=dim)
    d.text((48 * s, 52 * s), '2', font=font(52), fill=ink)
    d.text((82 * s, 74 * s), '/3', font=font(24), fill=dim)
    d.text((174 * s, 52 * s), '2', font=font(52), fill=ink)
    d.text((206 * s, 74 * s), '/5', font=font(24), fill=dim)
    panel(470, 20, 340, 104)
    d.text((640 * s, 30 * s), 'CURRENT LAP', font=font(14, False), fill=dim, anchor='mt')
    d.text((640 * s, 48 * s), '1:12.418', font=font(42), fill=ink, anchor='mt')
    d.rectangle([560 * s, 98 * s, 720 * s, 120 * s], fill=warn)
    d.text((640 * s, 109 * s), '+0.426', font=font(17), fill=(255, 255, 255, 255), anchor='mm')
    panel(980, 26, 272, 112)
    for k, (pos, name, col, gap) in enumerate((('1', '787B', (240, 122, 30), '-2.315'), ('2', 'RX-7', (255, 216, 58), 'YOU'), ('3', 'ND', (195, 32, 43), '+4.872'))):
        y = 36 + k * 32
        if gap == 'YOU':
            d.rectangle([996 * s, y * s, 1242 * s, (y + 28) * s], fill=(46, 230, 200, 56))
        d.text((1006 * s, (y + 3) * s), pos, font=font(19), fill=ink)
        d.rectangle([1030 * s, (y + 5) * s, 1035 * s, (y + 24) * s], fill=col + (255,))
        d.text((1044 * s, (y + 3) * s), name, font=font(19, False), fill=ink)
        d.text((1232 * s, (y + 3) * s), gap, font=font(19, False), fill=ac if gap == 'YOU' else dim, anchor='ra')
    # 回転計
    cx, cy, R = 1105 * s, 555 * s, 118 * s
    d.ellipse([cx - R - 18 * s, cy - R - 18 * s, cx + R + 18 * s, cy + R + 18 * s], fill=(10, 14, 20, 170))
    a0, a1 = 135, 405
    d.arc([cx - R, cy - R, cx + R, cy + R], a0, a1, fill=(255, 255, 255, 40), width=int(14 * s))
    d.arc([cx - R, cy - R, cx + R, cy + R], a0 + (a1 - a0) * 8 / 9, a1, fill=(255, 77, 61, 140), width=int(14 * s))
    d.arc([cx - R, cy - R, cx + R, cy + R], a0, a0 + (a1 - a0) * 7350 / 9000, fill=ac, width=int(14 * s))
    for kk in range(10):
        a = math.radians(a0 + (a1 - a0) * kk / 9)
        d.line([(cx + math.cos(a) * (R - 14 * s), cy + math.sin(a) * (R - 14 * s)), (cx + math.cos(a) * (R - 30 * s), cy + math.sin(a) * (R - 30 * s))],
               fill=warn if kk >= 8 else ink, width=max(1, int(3 * s)))
        d.text((cx + math.cos(a) * (R - 48 * s), cy + math.sin(a) * (R - 48 * s)), str(kk), font=font(17, False), fill=warn if kk >= 8 else dim, anchor='mm')
    d.text((cx, cy - 12 * s), '4', font=font(90), fill=ink, anchor='mm')
    d.text((cx, cy + 56 * s), '214', font=font(38), fill=ink, anchor='mm')
    d.text((cx, cy + 84 * s), 'km/h', font=font(15, False), fill=dim, anchor='mm')
    for kk in range(8):
        col = good if kk < 3 else ((255, 210, 58, 255) if kk < 6 else warn)
        on = kk < 6
        x = cx - 77 * s + kk * 22 * s
        d.ellipse([x - 7 * s, cy - R - 45 * s, x + 7 * s, cy - R - 31 * s], fill=col if on else (255, 255, 255, 40))
    # ミニマップ
    panel(30, 150, 250, 150)
    f = np.array(data['centerline_shortcut']).reshape(-1, 2)
    lo, hi = f.min(0), f.max(0)
    sc = min(200 / (hi - lo)[0], 120 / (hi - lo)[1]) * s
    pts = [(155 * s + (x - (lo[0] + hi[0]) / 2) * sc, 225 * s - (y - (lo[1] + hi[1]) / 2) * sc) for x, y in f[::4]]
    d.line(pts + [pts[0]], fill=(255, 255, 255, 70), width=int(8 * s), joint='curve')
    d.line(pts + [pts[0]], fill=ink, width=max(1, int(2.5 * s)), joint='curve')
    for j, c in ((600, (240, 122, 30)), (480, (255, 216, 58)), (1500, (195, 32, 43))):
        x, y = pts[j // 4 % len(pts)]
        d.ellipse([x - 7 * s, y - 7 * s, x + 7 * s, y + 7 * s], fill=c + (255,), outline=(255, 255, 255, 255), width=max(1, int(2 * s)))
    # 入力と G
    panel(30, 560, 300, 130)
    for kk, (lab, v, col) in enumerate((('THR', 0.82, good), ('BRK', 0.0, warn))):
        x = (56 + kk * 34) * s
        d.rectangle([x, 584 * s, x + 20 * s, 664 * s], fill=(255, 255, 255, 36))
        d.rectangle([x, (584 + 80 * (1 - v)) * s, x + 20 * s, 664 * s], fill=col)
        d.text((x + 10 * s, 676 * s), lab, font=font(13, False), fill=dim, anchor='mm')
    d.rectangle([132 * s, 620 * s, 222 * s, 628 * s], fill=(255, 255, 255, 36))
    d.rectangle([177 * s, 620 * s, 201 * s, 628 * s], fill=ac)
    d.text((177 * s, 648 * s), 'STEER', font=font(13, False), fill=dim, anchor='mm')
    gx, gy, gr = 278 * s, 624 * s, 40 * s
    d.ellipse([gx - gr, gy - gr, gx + gr, gy + gr], outline=(255, 255, 255, 80), width=max(1, int(1.5 * s)))
    d.ellipse([gx - gr / 2, gy - gr / 2, gx + gr / 2, gy + gr / 2], outline=(255, 255, 255, 80), width=max(1, int(1.5 * s)))
    d.ellipse([gx + gr * 0.62 - 7 * s, gy - gr * 0.18 - 7 * s, gx + gr * 0.62 + 7 * s, gy - gr * 0.18 + 7 * s], fill=ac)
    d.text((gx, gy + gr + 16 * s), '0.86 G', font=font(14), fill=ink, anchor='mm')
    d.rectangle([470 * s, 676 * s, 810 * s, 706 * s], fill=(10, 14, 20, 128))
    d.text((640 * s, 691 * s), 'FUJI  4,563 m  SECTOR 4/8', font=font(15, False), fill=ink, anchor='mm')
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--course', required=True)
    ap.add_argument('--s', type=float, default=1250.0)
    ap.add_argument('--view', default='chase', choices=['chase', 'onboard', 'scenic', 'grandstand', 'panasonic'])
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', default='1280x720')
    ap.add_argument('--hud', action='store_true')
    a = ap.parse_args()
    W, H = (int(v) for v in a.size.split('x'))
    data = json.load(open(a.course))
    im, _ = render(data, a.s, a.view, W, H)
    if a.hud:
        im = hud(im, data)
    im.save(a.out)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
