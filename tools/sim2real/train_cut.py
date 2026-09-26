#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CUT (sim→real 画像変換) の学習。A = Unity の /camera/image_raw (extract_frames.py の PNG)、B = 実カメラの画像。
CPU で回せる大きさ (生成器 ngf 32・ResBlock 6・224×224・batch 1) を既定にしている。

  ~/jetracer/venv_sim2real/bin/python tools/sim2real/train_cut.py \
      --sim ~/jetracer/data/sim_frames --real ~/jetracer/data/real_260912 --out ~/jetracer/runs/cut_001 --iters 40000
  (中断しても --resume で続きから。--iters は合計反復数)

出力 (--out):
  ckpt.pt           最新の重み (G, D, MLP, 最適化器, 反復数)
  samples/NNNNNN.png 反復ごとの見本 (上: 入力 sim、下: 変換後。右端に実画像)
  G.onnx            --export か学習終了時。convert_bag.py が使う (入力 1×3×H×W、[-1,1]、RGB)
  log.csv           反復・損失・秒/反復
"""
import argparse
import csv
import glob
import os
import random
import sys
import time

import numpy as np
import torch
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_posts, post_mask, paint_posts, to_tensor_np, from_tensor_np  # noqa: E402
from cut import Generator, Discriminator, PatchSampleMLP, patch_nce_loss  # noqa: E402


def list_images(root, exts=('.png', '.jpg', '.jpeg')):
    out = []
    for dp, _, fs in os.walk(os.path.expanduser(root)):
        out += [os.path.join(dp, f) for f in fs if f.lower().endswith(exts)]
    return sorted(out)


class Pool:
    """画像をランダムに読み、柱を塗って [-1,1] のテンソルにする。"""

    def __init__(self, files, size, mask, flip=True, crop=0):
        self.files, self.size, self.mask, self.flip, self.crop = files, size, mask, flip, crop

    def sample(self):
        f = random.choice(self.files)
        img = Image.open(f).convert('RGB')
        if img.size != (self.size, self.size):
            img = img.resize((self.size, self.size), Image.BILINEAR)
        bgr = np.asarray(img)[:, :, ::-1]
        if self.flip and random.random() < 0.5:
            bgr = bgr[:, ::-1]
        bgr = paint_posts(np.ascontiguousarray(bgr), self.mask)
        if self.crop and self.crop < self.size:
            # 全畳み込みなので切り出しで学習して全体で推論できる (CPU での学習を約 3 倍速く)
            y = random.randint(0, self.size - self.crop)
            x = random.randint(0, self.size - self.crop)
            bgr = bgr[y:y + self.crop, x:x + self.crop]
        return torch.from_numpy(to_tensor_np(np.ascontiguousarray(bgr)))[None]


def export_onnx(G, path, size):
    G.eval()
    x = torch.zeros(1, 3, size, size)
    torch.onnx.export(G, x, path, input_names=['input'], output_names=['output'], opset_version=17,
                      dynamic_axes={'input': {0: 'batch'}, 'output': {0: 'batch'}})
    G.train()
    print('exported', path)


def save_samples(G, A, B, path, n=4):
    G.eval()
    with torch.no_grad():
        cols = []
        for _ in range(n):
            a = A.sample()
            fa = G(a)
            cols.append(np.concatenate([from_tensor_np(a[0].numpy()), from_tensor_np(fa[0].numpy())], 0))
        real = np.concatenate([from_tensor_np(B.sample()[0].numpy()), from_tensor_np(B.sample()[0].numpy())], 0)
        grid = np.concatenate(cols + [real], 1)
    Image.fromarray(grid[:, :, ::-1]).save(path)
    G.train()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', required=True, help='A: sim の画像ディレクトリ')
    ap.add_argument('--real', required=True, help='B: 実画像のディレクトリ (サブディレクトリも探す)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--iters', type=int, default=40000)
    ap.add_argument('--decay-from', type=float, default=0.5, help='この割合から学習率を線形に 0 へ')
    ap.add_argument('--size', type=int, default=224)
    ap.add_argument('--ngf', type=int, default=32)
    ap.add_argument('--blocks', type=int, default=6)
    ap.add_argument('--ndf', type=int, default=64)
    ap.add_argument('--lr', type=float, default=2e-4)
    ap.add_argument('--lambda-nce', type=float, default=1.0)
    ap.add_argument('--patches', type=int, default=256)
    ap.add_argument('--threads', type=int, default=max(1, (os.cpu_count() or 4) - 2))
    ap.add_argument('--sample-every', type=int, default=500)
    ap.add_argument('--save-every', type=int, default=1000)
    ap.add_argument('--crop', type=int, default=0, help='学習時の切り出し [px] (0 = 全体)。見本と推論は全体')
    ap.add_argument('--fast', action='store_true', help='FastCUT: 恒等 NCE を省き λ_NCE=10 (速い。CPU 向け)')
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--export', action='store_true', help='学習せず ckpt から G.onnx だけ書き出す')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()

    torch.set_num_threads(a.threads)
    random.seed(a.seed)
    torch.manual_seed(a.seed)
    out = os.path.expanduser(a.out)
    os.makedirs(os.path.join(out, 'samples'), exist_ok=True)
    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    G = Generator(a.ngf, a.blocks).to(dev)
    D = Discriminator(a.ndf).to(dev)
    # MLP の入力次元は G のエンコーダ特徴の実チャンネル数から決める
    with torch.no_grad():
        chans = [f.shape[1] for f in G.encode(torch.zeros(1, 3, 64, 64, device=dev))]
    H = PatchSampleMLP(chans).to(dev)
    optG = torch.optim.Adam(list(G.parameters()) + list(H.parameters()), lr=a.lr, betas=(0.5, 0.999))
    optD = torch.optim.Adam(D.parameters(), lr=a.lr, betas=(0.5, 0.999))
    it0 = 0
    ck = os.path.join(out, 'ckpt.pt')
    if (a.resume or a.export) and os.path.exists(ck):
        s = torch.load(ck, map_location=dev)
        G.load_state_dict(s['G']); D.load_state_dict(s['D']); H.load_state_dict(s['H'])
        optG.load_state_dict(s['optG']); optD.load_state_dict(s['optD'])
        it0 = s['it']
        print(f'resume from it {it0}')
    if a.export:
        export_onnx(G.cpu(), os.path.join(out, 'G.onnx'), a.size)
        return

    posts = load_posts()
    mask = post_mask(a.size, a.size, posts)
    A = Pool(list_images(a.sim), a.size, mask, crop=a.crop)
    B = Pool(list_images(a.real), a.size, mask, crop=a.crop)
    A_full = Pool(A.files, a.size, mask)
    B_full = Pool(B.files, a.size, mask)
    if a.fast and a.lambda_nce == 1.0:
        a.lambda_nce = 10.0
    print(f'A (sim) {len(A.files)} 枚, B (real) {len(B.files)} 枚, device {dev}, threads {a.threads}, '
          f'G {sum(p.numel() for p in G.parameters()) / 1e6:.2f} M params')
    if not A.files or not B.files:
        raise SystemExit('画像が無い')

    def lr_at(i):
        start = a.decay_from * a.iters
        return a.lr * (1.0 if i < start else max(0.0, 1.0 - (i - start) / max(1.0, a.iters - start)))

    mse = torch.nn.MSELoss()
    logf = open(os.path.join(out, 'log.csv'), 'a', newline='')
    lw = csv.writer(logf)
    if it0 == 0:
        lw.writerow(['it', 'lossD', 'lossG_gan', 'loss_nce', 'loss_nce_idt', 'sec_per_it', 'lr'])
    t_last = time.time()
    acc = np.zeros(4)
    n_acc = 0
    for it in range(it0 + 1, a.iters + 1):
        for o in (optG, optD):
            for g in o.param_groups:
                g['lr'] = lr_at(it)
        real_a, real_b = A.sample().to(dev), B.sample().to(dev)
        # G は real_a と real_b をまとめて通す (後者は NCE の恒等項用。FastCUT では省く)
        if a.fast:
            fake_b, idt_b = G(real_a), None
        else:
            fake = G(torch.cat([real_a, real_b], 0))
            fake_b, idt_b = fake[:1], fake[1:]

        # --- D ---
        optD.zero_grad()
        pf, pr = D(fake_b.detach()), D(real_b)
        lossD = 0.5 * (mse(pf, torch.zeros_like(pf)) + mse(pr, torch.ones_like(pr)))
        lossD.backward()
        optD.step()

        # --- G ---
        optG.zero_grad()
        pf = D(fake_b)
        l_gan = mse(pf, torch.ones_like(pf))

        def nce(src, tgt):
            fq = G.encode(tgt)
            fk = G.encode(src)
            k, ids = H(fk, a.patches)
            q, _ = H(fq, a.patches, ids)
            return sum(patch_nce_loss(qq, kk) for qq, kk in zip(q, k)) / len(q)

        l_nce = nce(real_a, fake_b)
        if a.fast:
            l_idt = torch.zeros(())
            lossG = l_gan + a.lambda_nce * l_nce
        else:
            l_idt = nce(real_b, idt_b)
            lossG = l_gan + a.lambda_nce * 0.5 * (l_nce + l_idt)
        lossG.backward()
        optG.step()

        acc += [lossD.item(), l_gan.item(), l_nce.item(), l_idt.item()]
        n_acc += 1
        if it % 50 == 0:
            spi = (time.time() - t_last) / n_acc
            m = acc / n_acc
            lw.writerow([it, *[f'{v:.4f}' for v in m], f'{spi:.3f}', f'{lr_at(it):.2e}'])
            logf.flush()
            eta = spi * (a.iters - it) / 3600
            print(f'it {it:6d}  D {m[0]:.3f}  G {m[1]:.3f}  NCE {m[2]:.3f}/{m[3]:.3f}  {spi:.2f} s/it  残り {eta:.1f} h', flush=True)
            acc[:] = 0
            n_acc = 0
            t_last = time.time()
        if it % a.sample_every == 0:
            save_samples(G, A_full, B_full, os.path.join(out, 'samples', f'{it:06d}.png'))
        if it % a.save_every == 0 or it == a.iters:
            torch.save(dict(G=G.state_dict(), D=D.state_dict(), H=H.state_dict(), optG=optG.state_dict(),
                            optD=optD.state_dict(), it=it, args=vars(a)), ck + '.tmp')
            os.replace(ck + '.tmp', ck)
    export_onnx(G.cpu(), os.path.join(out, 'G.onnx'), a.size)


if __name__ == '__main__':
    main()
