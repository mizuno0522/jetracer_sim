#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity の EngineAudio.cs と同じ式でエンジン音を合成し、WAV に書き出す (Unity 無しで聴いて確かめる)。
決まった走り方 (アイドル → 全開で 5 速まで → アクセルを戻して減速 → 再加速) を車ごとに鳴らす。

  python3 tools/engine_sound.py --car b787 --out /tmp/787b.wav [--png /tmp/787b.png]   # スペクトログラムも
  python3 tools/engine_sound.py --car all --outdir /tmp/sounds
  --legacy  変更前の合成 (倍音の足し合わせ) で鳴らす (比較用)

★EngineAudio.cs の式を変えたらここも合わせる。
"""
import argparse
import math
import os
import struct
import wave

import numpy as np

VOICES = {
    'b787': dict(pulses=4, cycle=4, idle=2400, red=9000, gears=[0.36, 0.52, 0.67, 0.83, 1.0], sharp=9.0, rasp=0.45,
                 form_hz=[1150, 2600, 4300], form_gain=[1.0, 0.85, 0.45], form_q=2.2, pipe_hz=310, pipe_fb=0.42,
                 drive=2.6, pop=1.0, whine=0.012, turbo=0.0, wind=0.6, level=0.55, vmax=330 / 3.6),
    'rx7': dict(pulses=2, cycle=2, idle=900, red=8000, gears=[0.27, 0.42, 0.58, 0.75, 1.0], sharp=9.0, rasp=0.40,
                form_hz=[620, 1500, 3100], form_gain=[1.0, 0.7, 0.35], form_q=2.0, pipe_hz=190, pipe_fb=0.35,
                drive=2.0, pop=0.45, whine=0.0, turbo=1.0, wind=0.7, level=0.65, vmax=250 / 3.6),
    'nd': dict(pulses=2, cycle=4, idle=750, red=7500, gears=[0.25, 0.39, 0.54, 0.69, 0.84, 1.0], sharp=6.0, rasp=0.10,
               form_hz=[260, 720, 1900], form_gain=[1.0, 0.6, 0.25], form_q=1.6, pipe_hz=130, pipe_fb=0.30,
               drive=1.4, pop=0.0, whine=0.0, turbo=0.0, wind=1.3, level=0.8, vmax=200 / 3.6),
}
BIAS = [0.00, 0.18, -0.12, 0.07]


class Engine:
    def __init__(self, v, sr, legacy=False, alat_max=9.0):
        self.v, self.sr, self.legacy = v, sr, legacy
        self.vmax, self.alat_max = v['vmax'], alat_max
        self.gear, self.prev_v, self.has_prev, self.along_f, self.shift_dip = 1, 0.0, False, 0.0, 0.0
        self.rpm = self.thr = self.frac = self.slip = 0.0
        self.gain = 0.6
        self.rng = 22222
        self.phase, self.event, self.since, self.amp = 0.0, 0, 0.0, 1.0
        self.hp = self.hpx = self.lp = self.road = self.w1 = self.w2 = self.sq1 = self.sq2 = 0.0
        self.turbo_ph = self.whine_ph = self.smooth = 0.0
        self.pop_env, self.prev_thr = 0.0, 0.0
        self.fx1, self.fx2, self.fy1, self.fy2 = [0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3
        self.pipe = [0.0] * max(8, round(sr / v['pipe_hz']))
        self.pipe_idx = 0

    def noise(self):
        r = self.rng
        r ^= (r << 13) & 0xFFFFFFFF
        r ^= r >> 17
        r ^= (r << 5) & 0xFFFFFFFF
        self.rng = r
        return (r & 0xFFFFFF) / 8388608.0 - 1.0

    def set_state(self, speed, alat, dt):
        v = self.v
        along = max(-12.0, min(12.0, (speed - self.prev_v) / dt)) if self.has_prev else 0.0
        self.prev_v, self.has_prev = speed, True
        self.along_f += (along - self.along_f) * (1 - math.exp(-dt / 0.15))
        frac = min(1.0, abs(speed) / self.vmax)
        rpm_at = lambda g: max(v['idle'], v['red'] * frac / v['gears'][g - 1])
        if rpm_at(self.gear) > v['red'] * 0.96 and self.gear < len(v['gears']):
            self.gear += 1
            self.shift_dip = 0.09
        elif self.gear > 1 and rpm_at(self.gear - 1) < v['red'] * 0.70:
            self.gear -= 1
        rpm = min(v['red'], rpm_at(self.gear))
        thr = max(0.0, min(1.0, self.along_f / (0.35 * self.alat_max) + (0.25 if frac > 0.03 else 0.05)))
        if self.shift_dip > 0:
            self.shift_dip -= dt
            thr *= 0.3
        self.rpm, self.thr, self.frac = rpm, thr, frac

    @staticmethod
    def bpf(hz, q, sr):
        w = 2 * math.pi * min(hz, sr * 0.45) / sr
        alpha = math.sin(w) / (2 * q)
        a0 = 1 + alpha
        return alpha / a0, -2 * math.cos(w) / a0, (1 - alpha) / a0

    def render(self, n):
        if self.legacy:
            return self.render_legacy(n)
        v, sr = self.v, self.sr
        rpm, thr, frac, gain = self.rpm, self.thr, self.frac, self.gain
        load = rpm / v['red']
        f0 = rpm / 60 * v['pulses']
        tau = 1 / (f0 * v['sharp'] * (0.7 + 0.6 * thr))
        jitter = v['rasp'] * (1.0 + (0.35 - 1.0) * load)
        grit = v['rasp'] * 0.35 * (1.0 + (0.15 - 1.0) * load)
        cut = 1800 + (9000 - 1800) * max(0, min(1, 0.4 * thr + 0.6 * load))
        a_lp = 1 - math.exp(-2 * math.pi * cut / sr)
        a_hp = math.exp(-2 * math.pi * 30 / sr)
        drive = 1 + v['drive'] * (0.3 + 0.7 * thr)
        tdrive = math.tanh(drive)
        eng_amp = v['level'] * (0.12 + 0.30 * thr) * (0.55 + 0.45 * load)
        if self.prev_thr - thr > 0.25 and load > 0.45:
            self.pop_env = 1.0
        self.prev_thr = thr
        road_amp = 0.16 * min(1, frac * 1.4)
        wind_amp = 0.08 * v['wind'] * frac * frac
        turbo_amp = 0.02 * v['turbo'] * thr * load
        turbo_f = 2200 + rpm * 0.55
        whine_amp = v['whine'] * load * (0.4 + 0.6 * thr)
        whine_f = rpm / 60 * 23
        co = [self.bpf(v['form_hz'][k], v['form_q'], sr) for k in range(3)]
        a_road = 1 - math.exp(-2 * math.pi * 380 / sr)
        a_w1, a_w2 = 1 - math.exp(-2 * math.pi * 2500 / sr), 1 - math.exp(-2 * math.pi * 500 / sr)
        dt = 1 / sr
        out = np.zeros(n)
        fg = v['form_gain']
        pipe, plen = self.pipe, len(self.pipe)
        for i in range(n):
            self.phase += f0 * dt
            self.since += dt
            if self.phase >= 1.0:
                self.phase -= math.floor(self.phase)
                k = self.event % v['cycle']
                self.event += 1
                a = 1 + v['rasp'] * BIAS[k % 4] + jitter * 0.5 * self.noise()
                if self.pop_env > 0.05 and v['pop'] > 0 and self.noise() > 0.55:
                    a += v['pop'] * 3.5 * self.pop_env * (0.5 + 0.5 * self.noise())
                self.amp = a
                self.since = self.phase / max(1.0, f0)
            env = math.exp(-self.since / tau)
            x = self.amp * env * (1 + grit * self.noise())
            hp = a_hp * (self.hp + x - self.hpx)
            self.hpx, self.hp = x, hp
            f = 0.25 * hp
            for k in range(3):
                b0, a1, a2 = co[k]
                y = b0 * hp - b0 * self.fx2[k] - a1 * self.fy1[k] - a2 * self.fy2[k]
                self.fx2[k], self.fx1[k], self.fy2[k], self.fy1[k] = self.fx1[k], hp, self.fy1[k], y
                f += fg[k] * y
            p = f + v['pipe_fb'] * pipe[self.pipe_idx]
            pipe[self.pipe_idx] = p
            self.pipe_idx = (self.pipe_idx + 1) % plen
            sat = math.tanh(drive * p) / tdrive
            self.lp += (sat - self.lp) * a_lp
            eng = self.lp * eng_amp
            nz = self.noise()
            self.road += (nz - self.road) * a_road
            self.w1 += (nz - self.w1) * a_w1
            self.w2 += (self.w1 - self.w2) * a_w2
            wind = (self.w1 - self.w2) * 2
            self.turbo_ph = (self.turbo_ph + turbo_f * dt) % 1.0
            self.whine_ph = (self.whine_ph + whine_f * dt) % 1.0
            y = (eng + self.road * road_amp * 3 + wind * wind_amp + math.sin(2 * math.pi * self.turbo_ph) * turbo_amp
                 + math.sin(2 * math.pi * self.whine_ph) * whine_amp)
            self.smooth += (gain - self.smooth) * 0.0005
            out[i] = y * self.smooth
        self.pop_env *= math.exp(-n * dt / 0.6)
        return out

    def render_legacy(self, n):
        """変更前の EngineAudio (倍音の足し合わせ + 燃焼直後の雑音)。比較用"""
        v, sr = self.v, self.sr
        old = {'b787': (0.35, 0.9), 'rx7': (0.28, 0.6), 'nd': (0.06, 0.35)}[self.name]
        rasp, bright = old
        rpm, thr, frac = self.rpm, self.thr, self.frac
        f0 = rpm / 60 * v['pulses']
        cut = (500 + (4000 - 500) * max(0, min(1, 0.35 * thr + 0.65 * rpm / v['red']))) * (0.6 + 0.6 * bright)
        a_lp = 1 - math.exp(-2 * math.pi * cut / sr)
        eng_amp = (0.10 + 0.32 * thr) * (0.6 + 0.4 * rpm / v['red'])
        out = np.zeros(n)
        for i in range(n):
            self.phase += f0 / sr
            ph = self.phase - math.floor(self.phase)
            tw = 2 * math.pi * ph
            e = math.sin(tw) + 0.6 * bright * math.sin(2 * tw + 0.3) + 0.45 * bright * math.sin(3 * tw + 0.9) + 0.25 * bright * math.sin(5 * tw)
            if v['pulses'] == 2 and rasp < 0.1:
                e += 0.25 * math.sin(0.5 * tw)
            e += rasp * (0.6 + thr) * math.exp(-ph * 9) * self.noise()
            self.lp += (e - self.lp) * a_lp
            self.smooth += (self.gain - self.smooth) * 0.0005
            out[i] = math.tanh(1.6 * self.lp) * eng_amp * self.smooth
        return out




def synth(name, sr=44100, legacy=False, seconds=32.0):
    v = VOICES[name]
    eng = Engine(v, sr, legacy)
    eng.name = name
    block = 1024
    nb = int(seconds * sr / block)
    dt = block / sr
    speed, out = 0.0, []
    a_max = {'b787': 9.0, 'rx7': 6.5, 'nd': 5.0}[name]
    for b in range(nb):
        t = b * dt
        if t < 2.0:
            acc = 0.0
        elif t < 22.0:
            acc = a_max * max(0.08, 1 - speed / v['vmax'])                       # 全開
        elif t < 25.0:
            acc = -11.0 if speed > 15 else 0.0                                     # アクセルを戻してブレーキ
        else:
            acc = a_max * 0.8 * max(0.08, 1 - speed / v['vmax'])                 # 再加速
        speed = max(0.0, speed + acc * dt)
        eng.set_state(speed, 0.0, dt)
        out.append(eng.render(block))
    return np.concatenate(out)


def write_wav(path, x, sr):
    x = np.clip(x / max(1e-6, np.abs(x).max()) * 0.9, -1, 1)
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes((x * 32767).astype('<i2').tobytes())


def spectrogram_png(path, x, sr, title):
    from PIL import Image, ImageDraw
    nfft, hop = 2048, 512
    win = np.hanning(nfft)
    frames = [np.abs(np.fft.rfft(x[i:i + nfft] * win)) for i in range(0, len(x) - nfft, hop)]
    S = 20 * np.log10(np.array(frames).T + 1e-6)
    fmax = 8000
    S = S[: int(fmax / (sr / nfft))]
    S = np.clip((S - (S.max() - 70)) / 70, 0, 1)
    img = (np.flipud(S) ** 0.8 * 255).astype(np.uint8)
    im = Image.fromarray(img).resize((1200, 500)).convert('RGB')
    d = ImageDraw.Draw(im)
    d.text((8, 6), f'{title}  0-{fmax} Hz, {len(x) / sr:.0f} s', fill=(255, 220, 0))
    for hz in (1000, 2000, 4000, 6000):
        y = 500 - hz / fmax * 500
        d.line([(0, y), (12, y)], fill=(255, 255, 255))
        d.text((14, y - 6), f'{hz}', fill=(200, 200, 200))
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='b787', choices=list(VOICES) + ['all'])
    ap.add_argument('--out')
    ap.add_argument('--outdir', default='.')
    ap.add_argument('--png', action='store_true')
    ap.add_argument('--legacy', action='store_true')
    ap.add_argument('--seconds', type=float, default=32.0)
    a = ap.parse_args()
    sr = 44100
    cars = list(VOICES) if a.car == 'all' else [a.car]
    for c in cars:
        x = synth(c, sr, a.legacy, a.seconds)
        path = a.out if (a.out and len(cars) == 1) else os.path.join(a.outdir, f'engine_{c}{"_legacy" if a.legacy else ""}.wav')
        write_wav(path, x, sr)
        print(f'wrote {path}')
        if a.png:
            spectrogram_png(os.path.splitext(path)[0] + '.png', x, sr, f'{c}{" legacy" if a.legacy else ""}')


if __name__ == '__main__':
    main()
