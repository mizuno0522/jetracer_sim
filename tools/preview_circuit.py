#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity を使わずに、サーキット (course_<name>_<profile>.json) の見た目を 1 枚の画像で確かめる。
路面のテクスチャは Unity の ProcTex.cs と同じ式で作り、コースの形・幅・縁石・グラベル・走行ラインのタイヤ痕も
CourseBuilder.Circuit.cs と同じ規則で置く。地面は画素ごとに光線を飛ばし、ガードレール・木・観客席は多角形で重ねる。
★Unity の描画そのものではない (影・反射・遠景のかすみは近似)。形と質感の確認用。

  python3 tools/preview_circuit.py --course unity/course_fuji_real_rx7.json --s 1250 --out /tmp/fuji.png [--hud]
  --s     画面に入れたい自車の位置 (コントロールラインからの距離 m)
  --view  chase (追従視点) | onboard (車載カメラ)
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
SUN = np.array([0.35, -0.55, 0.76])      # Unity の Euler(42, -35) を ROS 座標に直したもの (おおよそ)
SUN /= np.linalg.norm(SUN)
FOG = np.array([0.72, 0.79, 0.86])


def render(data, s_car, view, W, H, ss=2):
    tr = Track(data)
    W2, H2 = W * ss, H * ss
    i, p, t = tr.pose(s_car)
    nrm = np.array([-t[1], t[0]])
    car_xy = p + nrm * tr.line[i] * 0.85
    veh = data.get('vehicle', {})
    L = veh.get('length_m', 4.3)
    k = L / {'real_rx7': 0.448, 'real_nd': 0.428, 'real_b787': 0.486}.get(veh.get('name', ''), 0.45)
    if view == 'chase':
        eye = np.array([*(car_xy - t * 0.95 * k + nrm * -0.25 * k), 0.42 * k])
        look = np.array([*(car_xy + t * 0.35 * k), 0.06 * k])
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
    img = np.zeros((H2, W2, 3))
    # 空
    el = np.clip(dirs[..., 2], -1, 1)
    sky = lerp(np.array([0.80, 0.86, 0.92]), np.array([0.36, 0.56, 0.80]), np.clip(el * 3.0, 0, 1)[..., None])
    cl = fbm(np.arctan2(dirs[..., 1], dirs[..., 0]) / (2 * np.pi) % 1.0, np.clip(el * 4, 0, 0.999), 6, 4, 77)
    cloud = np.clip((cl - 0.55) * 3.0, 0, 1) * np.clip(el * 8, 0, 1)
    img[:] = lerp(sky, np.array([0.97, 0.97, 0.98]), (cloud * 0.8)[..., None])
    # 地面
    g = dirs[..., 2] < -1e-4
    tg = -eye[2] / dirs[..., 2][g]
    gp = eye[None, :2] + dirs[g][:, :2] * tg[:, None]
    dist = tg
    _, d, s = tr.locate(gp)
    ii = tr.locate(gp)[0]
    ad = np.abs(d)
    print(f'  地面 {g.sum()} 点を塗る')
    A1, H1 = asphalt(512, 11, 0.22)
    A2, H2h = asphalt(512, 31, 0.30)
    GA, GH = grass(512, 21)
    VA, VH = gravel(256, 41)
    KA, KH = kerb(128, 256, 71)
    LA, RB = line_tex(61), rubber_tex(51)
    N1, N2, NG, NV, NK = normals(H1, 2.2), normals(H2h, 2.2), normals(GH, 1.5), normals(VH, 3.0), normals(KH, 1.2)
    col = np.zeros((len(gp), 3))
    nm = np.zeros((len(gp), 3))
    nm[:, 2] = 1
    smooth = np.zeros(len(gp))

    def put(mask, alb, nmap, u, v, bumpk, sm):
        if not mask.any():
            return
        col[mask] = sample(alb, u[mask], v[mask])
        if nmap is not None:
            nn = sample(nmap, u[mask], v[mask])
            nn[:, :2] *= bumpk
            nm[mask] = nn / np.linalg.norm(nn, axis=1, keepdims=True)
        smooth[mask] = sm
    # 芝 (世界座標で 24 m の繰り返し)
    put(ad >= tr.bar, GA, NG, gp[:, 0] / 24, gp[:, 1] / 24, 0.6, 0.08)
    run = (ad < tr.bar) & (ad >= tr.half)
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
    smooth[ln] = 0.35
    kb = (ad >= tr.half) & (ad < tr.half + tr.kw) & (tr.k3[ii] >= tr.kmin)
    put(kb, KA, NK, (ad - tr.half) / tr.kw, s / 6, 1.0, 0.35)
    # コントロールライン
    cl_m = (np.abs(s % tr.total - 0) < 0.6) | (np.abs(s % tr.total - tr.total) < 0.6)
    cl_m &= trk
    chk = ((np.floor((d + tr.half) / 0.6) + np.floor((s % tr.total + 0.6) / 0.6)) % 2 == 0)
    col[cl_m] = np.where(chk[cl_m, None], 0.9, 0.08)
    # 光: 環境光 + 太陽 (法線マップ込み) + 少しの鏡面
    ndl = np.clip(nm @ SUN, 0, 1)
    vdir = -dirs[g]
    hv = SUN[None] + vdir
    hv /= np.linalg.norm(hv, axis=1, keepdims=True)
    spec = np.clip((nm * hv).sum(1), 0, 1) ** (8 + 120 * smooth) * smooth * 0.6
    lit = col * (0.48 + 0.78 * ndl)[:, None] + spec[:, None]
    fogk = np.clip((dist - 600) / 19400, 0, 1)[:, None]
    img[g] = lerp(lit, FOG, fogk)

    # 多角形 (遠い順): 山・木・ガードレール・観客席
    def proj(P):
        q = np.asarray(P, float) - eye
        z = q @ fwd
        return np.stack([W2 / 2 + (q @ right) / z * f, H2 / 2 - (q @ up) / z * f], -1), z
    im = Image.fromarray((np.clip(img, 0, 1) ** (1 / 1.0) * 255).astype(np.uint8))
    dr = ImageDraw.Draw(im, 'RGBA')
    polys = []
    b = data['circuit']['bounds']
    dirm = np.array([-0.70, 0.72])
    mid = np.array([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2]) + dirm / np.linalg.norm(dirm) * 9000
    ang = np.linspace(0, 2 * np.pi, 49)
    # 山 (2 段: 地肌と雪)
    Hm, R0, RT, SN = 1800, 3000, 420, 1250             # CourseBuilder.BuildMountain と同じ
    rz = lambda z: R0 + (RT - R0) * z / Hm
    for (z0, z1, r0, r1, c0) in ((0, SN, R0, rz(SN), (0.30, 0.36, 0.46)), (SN, Hm, rz(SN), RT, (0.92, 0.94, 0.97))):
        for a0 in range(48):
            a1 = a0 + 1
            wob = lambda a: 1 + 0.04 * math.sin(a * 5) + 0.03 * math.sin(a * 11 + 1)
            P = [[*(mid + r0 * wob(ang[a0]) * np.array([math.cos(ang[a0]), math.sin(ang[a0])])), z0],
                 [*(mid + r0 * wob(ang[a1]) * np.array([math.cos(ang[a1]), math.sin(ang[a1])])), z0],
                 [*(mid + r1 * wob(ang[a1]) * np.array([math.cos(ang[a1]), math.sin(ang[a1])])), z1],
                 [*(mid + r1 * wob(ang[a0]) * np.array([math.cos(ang[a0]), math.sin(ang[a0])])), z1]]
            nrm3 = np.array([math.cos((ang[a0] + ang[a1]) / 2), math.sin((ang[a0] + ang[a1]) / 2), 0.35])
            sh = 0.60 + 0.55 * max(0, float(nrm3 @ SUN) / np.linalg.norm(nrm3))
            c = lerp(np.array(c0) * sh, FOG, (9000 - 600) / 19400 + (0.12 if z0 == 0 else 0))
            polys.append((1e9 - a0, P, c, 1.0))
    # 観客席 (コントロールラインの外側・階段 14 段) とピット棟 (内側)。CourseBuilder.BuildPaddock と同じ寸法
    n = len(tr.C)
    t0, n0 = tr.T[0], tr.N[0]
    inside = 1 if ((tr.C.mean(0) - tr.C[0]) @ n0) > 0 else -1
    out = -inside
    for seg in range(-13, 13):
        a0, a1 = seg * 10.0, (seg + 1) * 10.0
        for r in range(14):
            d0 = tr.bar + 6 - 0.45 + r * 0.9
            z = 0.4 + r * 0.45
            base = tr.C[0] + n0 * out * d0
            P = [[*(base + t0 * a0), z], [*(base + t0 * a1), z], [*(base + n0 * out * 0.9 + t0 * a1), z], [*(base + n0 * out * 0.9 + t0 * a0), z]]
            dd = np.hypot(*(base + t0 * (a0 + a1) / 2 - eye[:2]))
            polys.append((dd + 0.001 * r, P, 'crowd', 1.0))
            F = [[*(base + t0 * a0), z - 0.45], [*(base + t0 * a1), z - 0.45], [*(base + t0 * a1), z], [*(base + t0 * a0), z]]
            polys.append((dd + 0.001 * r - 0.0005, F, np.array([0.55, 0.55, 0.53]), 1.0))
        rb = tr.C[0] + n0 * out * (tr.bar + 4)
        R = [[*(rb + t0 * a0), 10.5], [*(rb + t0 * a1), 10.5], [*(rb + n0 * out * 16 + t0 * a1), 10.5], [*(rb + n0 * out * 16 + t0 * a0), 10.5]]
        polys.append((np.hypot(*(rb + t0 * a0 - eye[:2])) - 5, R, np.array([0.62, 0.64, 0.67]), 1.0))
    for seg in range(-15, 15):
        a0, a1 = seg * 10.0, (seg + 1) * 10.0
        fb = tr.C[0] + n0 * inside * (tr.bar + 15)
        Fw = [[*(fb + t0 * a0), 0], [*(fb + t0 * a1), 0], [*(fb + t0 * a1), 9], [*(fb + t0 * a0), 9]]
        Dw = [[*(fb + t0 * a0 - n0 * inside * 0.1), 0], [*(fb + t0 * a1 - n0 * inside * 0.1), 0], [*(fb + t0 * a1 - n0 * inside * 0.1), 4.2], [*(fb + t0 * a0 - n0 * inside * 0.1), 4.2]]
        dd = np.hypot(*(fb + t0 * a0 - eye[:2]))
        polys.append((dd, Fw, np.array([0.70, 0.71, 0.73]), 1.0))
        polys.append((dd - 0.01, Dw, np.array([0.16, 0.18, 0.21]), 1.0))
    # ガードレール (カメラから 700 m 以内)
    for side in (1, -1):
        Q = tr.C + tr.N * side * tr.bar
        for i0 in range(n):
            if np.hypot(*(Q[i0] - eye[:2])) > 700:
                continue
            i1 = (i0 + 1) % n
            for (h0, h1, g0) in ((0.0, 0.35, 0.30), (0.35, 0.64, 0.74), (0.64, 0.92, 0.58), (0.92, 1.0, 0.30)):
                P = [[*Q[i0], h0], [*Q[i1], h0], [*Q[i1], h1], [*Q[i0], h1]]
                dd = np.hypot(*(Q[i0] - eye[:2]))
                c = lerp(np.array([g0, g0 * 1.01, g0 * 1.04]) * 1.05, FOG, min(1, max(0, (dd - 600) / 19400)))
                polys.append((dd, P, c, 1.0))
    # 木
    placed = 0
    for kk in range(6000):
        if placed >= 1400:
            break
        ii0 = int(hash2(kk, 1, 7) * n) % n
        if tr.S[ii0] < 350 or tr.S[ii0] > tr.total - 350:
            continue
        side = -1 if hash2(kk, 2, 7) < 0.5 else 1
        dd0 = tr.bar + 25 + 100 * hash2(kk, 3, 7)
        pp = tr.C[ii0] + tr.N[ii0] * side * dd0 + tr.T[ii0] * (hash2(kk, 4, 7) - 0.5) * 8
        if (np.sum((tr.C[::2] - pp) ** 2, axis=1) < (tr.bar + 20) ** 2).any():
            continue
        placed += 1
        dd = np.hypot(*(pp - eye[:2]))
        if dd > 1500:
            continue
        h = 9 + 9 * hash2(kk, 5, 7)
        r = h * (0.26 + 0.08 * hash2(kk, 6, 7))
        greens = [(0.16, 0.29, 0.16), (0.22, 0.35, 0.17), (0.13, 0.24, 0.15)]
        base = greens[placed % 3]
        side_v = np.array([-(pp - eye[:2])[1], (pp - eye[:2])[0]])
        side_v /= max(1e-6, np.linalg.norm(side_v))
        fogk = min(1, max(0, (dd - 600) / 19400))
        # 樹冠を明暗 2 枚の三角形で (日の当たる側が明るい)
        for (sgn, shade) in ((1, 1.15), (-1, 0.75)):
            P = [[*(pp), h], [*(pp + side_v * r * sgn), h * 0.25], [*(pp), h * 0.25]]
            polys.append((dd - 0.01 * sgn, P, lerp(np.array(base) * shade, FOG, fogk), 1.0))
        polys.append((dd + 0.5, [[*(pp - side_v * 0.25), 0], [*(pp + side_v * 0.25), 0], [*(pp + side_v * 0.25), h * 0.3], [*(pp - side_v * 0.25), h * 0.3]],
                      lerp(np.array([0.27, 0.20, 0.14]), FOG, fogk), 1.0))
    polys.sort(key=lambda q: -q[0])
    rng = np.random.default_rng(91)
    crowd = (np.stack([rng.uniform(0.2, 0.95, (H2, W2)) for _ in range(3)], -1) * 255).astype(np.uint8)
    crowd[rng.uniform(size=(H2, W2)) < 0.25] = (140, 142, 146)
    crowd_im = Image.fromarray(crowd).resize((W2 // 3, H2 // 3), Image.NEAREST).resize((W2, H2), Image.NEAREST)
    for dd, P, c, a in polys:
        sp, z = proj(P)
        if (z < 0.5).any():
            continue
        if (sp[:, 0] < -W2).all() or (sp[:, 0] > 2 * W2).all() or (sp[:, 1] < -H2).all() or (sp[:, 1] > 2 * H2).all():
            continue
        if isinstance(c, str):          # 観客 (色の粒) を多角形の形で貼る
            m = Image.new('L', (W2, H2), 0)
            ImageDraw.Draw(m).polygon([tuple(q) for q in sp], fill=255)
            im.paste(crowd_im, (0, 0), m)
            continue
        cc = tuple(int(v) for v in np.clip(np.array(c) * 255, 0, 255))
        dr.polygon([tuple(q) for q in sp], fill=cc + (int(255 * a),))
    if view == 'chase':
        draw_car(im, proj, eye, car_xy, t, nrm, k, STYLE.get(veh.get('name', ''), 'BuildRx7'))
    im = im.resize((W, H), Image.LANCZOS)
    # 周辺減光
    vg = np.ones((H, W))
    yy, xx = np.mgrid[0:H, 0:W]
    vg = 1 - 0.22 * (((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2) / 2
    arr = np.asarray(im).astype(float) * vg[..., None]
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8)), tr


CARMODEL = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'unity', 'MinicarSim', 'Assets', 'Minicar', 'Scripts', 'CarModel.cs')
PAINT = {'BuildRx7': (0.98, 0.78, 0.05), 'BuildRoadster': (0.62, 0.03, 0.06), 'BuildB787': (0.96, 0.42, 0.06)}
MATS = {'yellow': None, 'red': None, 'orange': None, 'paint': None, 'green': (0.05, 0.55, 0.30), 'black': (0.03, 0.03, 0.03),
        'glass': (0.04, 0.05, 0.07), 'lamp': (0.95, 0.95, 0.90), 'tail': (0.75, 0.05, 0.05), 'alu': (0.55, 0.56, 0.58), 'seat': (0.12, 0.11, 0.11)}


def car_parts(method):
    """CarModel.cs の Build<車>() から、断面 (Loft) と箱 (Cube) を読む。Unity と同じ形を描くため"""
    src = open(CARMODEL, encoding='utf-8').read()
    a = src.index(f'void {method}(')
    b = src.index('\n        }\n', a)
    blk = src[a:b]
    vec = lambda t: [tuple(float(x.rstrip('f')) for x in v.split(',')) for v in re.findall(r'new Vector4\(([^)]*)\)', t)]
    arrays = {m.group(1): vec(m.group(2)) for m in re.finditer(r'var (\w+) = new\[\]\s*\{(.*?)\};', blk, re.S)}
    lofts = []
    for m in re.finditer(r'Part\("(\w+)", hull, Loft\((?:new\[\]\s*\{(.*?)\}|(\w+)), ([\d.]+)f, true\), (\w+)\)', blk, re.S):
        secs = vec(m.group(2)) if m.group(2) else arrays[m.group(3)]
        lofts.append((secs, float(m.group(4)), m.group(5)))
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
    paint = np.array(PAINT.get(style, (0.6, 0.6, 0.6)))
    def W(p):                # 車体座標 (x 右・y 上・z 前、後軸の真下が原点) → 世界
        x, y, z = p
        w = xy + t * z * k - nrm * x * k
        return np.array([w[0], w[1], y * k])
    items = []                # (面, 材質, 外向きの基準点 [車体座標]): 面の重心から基準点を引いた向きが外
    for secs, nexp, mat in lofts:
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
        polys.append((np.linalg.norm(c - eye), P, col))
    polys.sort(key=lambda q: -q[0])
    for _, P, col in polys:
        sp, z = proj(P)
        if (z < 0.2).any():
            continue
        dr.polygon([tuple(q) for q in sp], fill=tuple(int(v) for v in np.clip(col * 255, 0, 255)))


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
    ap.add_argument('--view', default='chase', choices=['chase', 'onboard'])
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
