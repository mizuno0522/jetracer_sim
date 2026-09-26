# -*- coding: utf-8 -*-
"""
CUT (Contrastive Unpaired Translation, Park et al. 2020) の最小実装。
対応づけの無い 2 組の画像 (A = Unity の描画、B = 実カメラ) から A→B の変換器 G を学ぶ。

  - G: ResNet 生成器 (下り 2 段・ResBlock n 個・上り 2 段)。全畳み込みなので入力サイズは自由
  - D: PatchGAN (70×70 相当)。LSGAN
  - PatchNCE: G のエンコーダの中間特徴で「変換前後の同じ場所のパッチ」を正例、他の場所を負例にした対照損失。
    これが幾何 (壁・走路の縁の位置) を保つ役。恒等写像の NCE (B→G(B)) も足す (CUT 既定)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class ResBlock(nn.Module):
    def __init__(self, c):
        super().__init__()
        self.b = nn.Sequential(
            nn.ReflectionPad2d(1), nn.Conv2d(c, c, 3), nn.InstanceNorm2d(c), nn.ReLU(True),
            nn.ReflectionPad2d(1), nn.Conv2d(c, c, 3), nn.InstanceNorm2d(c))

    def forward(self, x):
        return x + self.b(x)


class Generator(nn.Module):
    def __init__(self, ngf=32, n_blocks=6):
        super().__init__()
        L = [nn.ReflectionPad2d(3), nn.Conv2d(3, ngf, 7), nn.InstanceNorm2d(ngf), nn.ReLU(True)]
        c = ngf
        self.nce_layers = [0]                         # 0 = 入力 (pad 後)
        for _ in range(2):
            self.nce_layers.append(len(L))            # 下り畳み込みの直後
            L += [nn.Conv2d(c, c * 2, 3, 2, 1), nn.InstanceNorm2d(c * 2), nn.ReLU(True)]
            c *= 2
        first_block = len(L)
        L += [ResBlock(c) for _ in range(n_blocks)]
        self.nce_layers += [first_block + n_blocks // 2 - 1, first_block + n_blocks - 1]
        for _ in range(2):
            L += [nn.ConvTranspose2d(c, c // 2, 3, 2, 1, output_padding=1), nn.InstanceNorm2d(c // 2), nn.ReLU(True)]
            c //= 2
        L += [nn.ReflectionPad2d(3), nn.Conv2d(c, 3, 7), nn.Tanh()]
        self.model = nn.Sequential(*L)
        self.n_enc = self.nce_layers[-1] + 1

    def forward(self, x):
        return self.model(x)

    def encode(self, x):
        feats = []
        h = x
        for i, layer in enumerate(self.model[:self.n_enc]):
            h = layer(h)
            if i in self.nce_layers:
                feats.append(h)
        return feats


class Discriminator(nn.Module):
    def __init__(self, ndf=64, n_layers=3):
        super().__init__()
        L = [nn.Conv2d(3, ndf, 4, 2, 1), nn.LeakyReLU(0.2, True)]
        c = ndf
        for i in range(1, n_layers):
            L += [nn.Conv2d(c, min(c * 2, ndf * 8), 4, 2, 1), nn.InstanceNorm2d(min(c * 2, ndf * 8)), nn.LeakyReLU(0.2, True)]
            c = min(c * 2, ndf * 8)
        L += [nn.Conv2d(c, min(c * 2, ndf * 8), 4, 1, 1), nn.InstanceNorm2d(min(c * 2, ndf * 8)), nn.LeakyReLU(0.2, True)]
        L += [nn.Conv2d(min(c * 2, ndf * 8), 1, 4, 1, 1)]
        self.model = nn.Sequential(*L)

    def forward(self, x):
        return self.model(x)


class PatchSampleMLP(nn.Module):
    """各層の特徴から同じ場所のパッチを取り、2 層 MLP で射影して L2 正規化する。"""

    def __init__(self, channels, nc=256):
        super().__init__()
        self.mlps = nn.ModuleList([nn.Sequential(nn.Linear(c, nc), nn.ReLU(True), nn.Linear(nc, nc)) for c in channels])

    def forward(self, feats, num_patches=256, patch_ids=None):
        out, ids = [], []
        for i, f in enumerate(feats):
            B, C, H, W = f.shape
            fr = f.permute(0, 2, 3, 1).reshape(B, H * W, C)
            if patch_ids is None:
                pid = torch.randperm(H * W, device=f.device)[:min(num_patches, H * W)]
            else:
                pid = patch_ids[i]
            x = fr[:, pid, :].reshape(-1, C)
            x = F.normalize(self.mlps[i](x), dim=1)
            out.append(x.reshape(B, -1, x.shape[-1]))
            ids.append(pid)
        return out, ids


def patch_nce_loss(q, k, tau=0.07):
    """q, k: (B, P, C)。同じ場所 (対角) が正例、同じ画像の他の場所が負例。"""
    B, P, C = q.shape
    k = k.detach()
    l_pos = (q * k).sum(-1, keepdim=True)                       # (B,P,1)
    l_neg = torch.bmm(q, k.transpose(1, 2))                     # (B,P,P)
    eye = torch.eye(P, device=q.device, dtype=torch.bool)[None]
    l_neg = l_neg.masked_fill(eye, -10.0)
    logits = torch.cat([l_pos, l_neg], dim=2) / tau             # (B,P,1+P)
    target = torch.zeros(B * P, dtype=torch.long, device=q.device)
    return F.cross_entropy(logits.reshape(B * P, -1), target)
