#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
policy_net (画像 ＋ IMU → 先行注視点 u, v と速度係数 s) の模倣学習。build_dataset.py の出力を読む。
出力の ONNX は jetracer_stack/policy_net.py の約束どおり:
  入力 image float32 [1,3,224,224] (RGB, /255, ImageNet 正規化)、imu float32 [1,50,6] (生の値。正規化はモデルの中)
  出力 [1,3] = (u, v を [-1,1] に正規化したもの, s ∈ [0,1])  ← policy_net が u = (y0+1)/2·W などに戻す

  ~/jetracer/venv_sim2real/bin/python tools/policy/train_policy.py --data ~/jetracer/data/policy \
      --val pol_428,pol_429,pol_430 --out ~/jetracer/runs/policy_001 --epochs 10
  ~/jetracer/venv_sim2real/bin/python tools/policy/train_policy.py --out ~/jetracer/runs/policy_001 --export [--zero-imu|--zero-image]
モデル: ResNet18 (ImageNet 事前学習、JetRacer 標準と同じ) ＋ IMU の 1 次元畳み込み。CPU で学習できる大きさ。
"""
import argparse
import glob
import json
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
IMU_SCALE = torch.tensor([9.81, 9.81, 9.81, 1.0, 1.0, 1.0])     # 加速度は g 単位に、角速度は rad/s のまま


class Policy(nn.Module):
    def __init__(self, pretrained=True, width=224, height=224, zero_imu=False, zero_image=False):
        super().__init__()
        w = torchvision.models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
        r = torchvision.models.resnet18(weights=w)
        r.fc = nn.Identity()
        self.cnn = r
        self.imu = nn.Sequential(
            nn.Conv1d(6, 32, 5, padding=2), nn.ReLU(True),
            nn.Conv1d(32, 32, 5, stride=2, padding=2), nn.ReLU(True),
            nn.AdaptiveAvgPool1d(1), nn.Flatten(), nn.Linear(32, 64), nn.ReLU(True))
        self.head = nn.Sequential(nn.Linear(512 + 64, 128), nn.ReLU(True), nn.Dropout(0.2), nn.Linear(128, 3))
        self.register_buffer('imu_scale', IMU_SCALE.clone())
        self.zero_imu, self.zero_image = zero_imu, zero_image     # 陽性対照 (目隠しテスト) 用の書き出し

    def forward(self, image, imu):
        if self.zero_image:
            image = image * 0.0
        if self.zero_imu:
            imu = imu * 0.0
        f = self.cnn(image)
        g = self.imu((imu / self.imu_scale).transpose(1, 2))
        y = self.head(torch.cat([f, g], 1))
        # u, v は [-1,1]、s は [0,1]
        return torch.cat([torch.tanh(y[:, :2]), torch.sigmoid(y[:, 2:3])], 1)


def strong_aug(x):
    """実画像に多い崩れ (手ぶれ・動きのぼけ・白飛び・暗さ・遮り) を足す。形 (注視点の位置) は動かさない。"""
    b = x.shape[0]
    x = x.clamp(1e-4, 1) ** torch.empty(b, 1, 1, 1).uniform_(0.6, 1.6)                     # ガンマ
    over = (torch.rand(b, 1, 1, 1) < 0.2).float()
    x = x * (1 + over * torch.empty(b, 1, 1, 1).uniform_(0.3, 1.2))                          # 白飛び
    x = x * torch.empty(b, 3, 1, 1).uniform_(0.85, 1.15)
    for i in range(b):
        r = float(torch.rand(1))
        if r < 0.35:                                                                          # 動きのぼけ (横 or 縦)
            k = int(torch.randint(3, 12, (1,)))
            w = torch.ones(3, 1, 1, k) / k if torch.rand(1) < 0.6 else torch.ones(3, 1, k, 1) / k
            pad = (k // 2, k - 1 - k // 2, 0, 0) if w.shape[3] > 1 else (0, 0, k // 2, k - 1 - k // 2)
            x[i:i + 1] = F.conv2d(F.pad(x[i:i + 1], pad, mode='replicate'), w, groups=3)
        elif r < 0.5:                                                                         # ピンぼけ
            x[i:i + 1] = F.avg_pool2d(F.pad(x[i:i + 1], (2, 2, 2, 2), mode='replicate'), 5, 1)
        if torch.rand(1) < 0.25:                                                              # 遮り (人・物)
            h, w_ = int(torch.randint(15, 70, (1,))), int(torch.randint(10, 50, (1,)))
            y0, x0 = int(torch.randint(0, 224 - h, (1,))), int(torch.randint(0, 224 - w_, (1,)))
            x[i, :, y0:y0 + h, x0:x0 + w_] = torch.rand(3, 1, 1)
    return x.clamp(0, 1)


class Data:
    """bag ごとの npz と画像 .npy (mmap) を束ねる。"""

    def __init__(self, root, names):
        self.parts = []
        for n in names:
            d = np.load(os.path.join(root, n + '.npz'))
            img = np.load(os.path.join(root, n + '_img.npy'), mmap_mode='r')
            self.parts.append((img, d['imu'], d['y']))
        self.index = [(p, i) for p, (img, _, y) in enumerate(self.parts) for i in range(len(y)) if y[i, 3] > 0.5]

    def __len__(self):
        return len(self.index)

    def batch(self, idx, augment=False, rng=None, strong=False):
        ims, imus, ys = [], [], []
        for k in idx:
            p, i = self.index[k]
            img, imu, y = self.parts[p]
            ims.append(np.asarray(img[i]))
            imus.append(imu[i])
            ys.append(y[i])
        x = torch.from_numpy(np.stack(ims)[:, :, :, ::-1].copy()).permute(0, 3, 1, 2).float() / 255.0   # BGR→RGB
        if augment:
            b = x.shape[0]
            x = x * torch.empty(b, 1, 1, 1).uniform_(0.7, 1.3) + torch.empty(b, 1, 1, 1).uniform_(-0.08, 0.08)   # 明るさ・露出
            x = x * torch.empty(b, 3, 1, 1).uniform_(0.92, 1.08)                                                  # 色味
            if strong:
                x = strong_aug(x)
            x = (x + torch.randn_like(x) * 0.02).clamp(0, 1)
        x = (x - MEAN) / STD
        imu = torch.from_numpy(np.stack(imus))
        y = torch.from_numpy(np.stack(ys))
        tgt = torch.stack([y[:, 0] / 224.0 * 2 - 1, y[:, 1] / 224.0 * 2 - 1, y[:, 2]], 1)
        return x, imu, tgt


def evaluate(model, data, bs=64):
    model.eval()
    errs = []
    with torch.no_grad():
        for s in range(0, len(data), bs):
            x, imu, t = data.batch(list(range(s, min(len(data), s + bs))))
            p = model(x, imu)
            errs.append(torch.abs(p - t))
    e = torch.cat(errs).mean(0)
    model.train()
    return {'u_px': float(e[0] * 112), 'v_px': float(e[1] * 112), 's': float(e[2])}


def export(model, path):
    model.eval()
    torch.onnx.export(model, (torch.zeros(1, 3, 224, 224), torch.zeros(1, 50, 6)), path,
                      input_names=['image', 'imu'], output_names=['y'], opset_version=17,
                      dynamic_axes={'image': {0: 'b'}, 'imu': {0: 'b'}, 'y': {0: 'b'}})
    print('exported', path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default=os.path.expanduser('~/jetracer/data/policy'))
    ap.add_argument('--val', default='', help='検証に回す bag 名の前方一致 (カンマ区切り)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--epochs', type=int, default=10)
    ap.add_argument('--bs', type=int, default=32)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--threads', type=int, default=max(1, (os.cpu_count() or 4) - 1))
    ap.add_argument('--hours', type=float, default=0.0, help='この時間で打ち切る (0 = 無制限)')
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--export', action='store_true')
    ap.add_argument('--zero-imu', action='store_true')
    ap.add_argument('--zero-image', action='store_true')
    ap.add_argument('--init', default='', help='この ckpt.pt の重みから始める (追加学習)')
    ap.add_argument('--strong-aug', action='store_true', help='ぼけ・白飛び・遮りの強い拡張 (実画像向け)')
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    out = os.path.expanduser(a.out)
    os.makedirs(out, exist_ok=True)
    ck = os.path.join(out, 'ckpt.pt')
    model = Policy(pretrained=not (a.export or a.resume or a.init))
    if a.init and not a.export:
        model.load_state_dict(torch.load(os.path.expanduser(a.init), map_location='cpu')['model'])
    if a.export:
        model.load_state_dict(torch.load(ck, map_location='cpu')['model'])
        model.zero_imu, model.zero_image = a.zero_imu, a.zero_image
        suf = '_zero_imu' if a.zero_imu else '_zero_image' if a.zero_image else ''
        export(model, os.path.join(out, f'policy{suf}.onnx'))
        return
    names = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(a.data, '*.npz')))
    vpre = [v for v in a.val.split(',') if v]
    val_names = [n for n in names if any(n.startswith(v) for v in vpre)]
    tr_names = [n for n in names if n not in val_names]
    tr, va = Data(a.data, tr_names), Data(a.data, val_names) if val_names else None
    print(f'train {len(tr)} 枚 ({len(tr_names)} bag), val {len(va) if va else 0} 枚 ({len(val_names)} bag), threads {a.threads}', flush=True)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * (len(tr) // a.bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=max(1, steps), pct_start=0.1)
    ep0, step = 0, 0
    hist = []
    if a.resume and os.path.exists(ck):
        s = torch.load(ck, map_location='cpu')
        model.load_state_dict(s['model']); opt.load_state_dict(s['opt']); sched.load_state_dict(s['sched'])
        ep0, step, hist = s['epoch'], s['step'], s.get('hist', [])
    rng = np.random.default_rng(0)
    t_start = time.time()
    model.train()
    for ep in range(ep0, a.epochs):
        perm = rng.permutation(len(tr))
        t0 = time.time()
        run = 0.0
        nb = len(tr) // a.bs
        for b in range(nb):
            x, imu, t = tr.batch(perm[b * a.bs:(b + 1) * a.bs], augment=True, rng=rng, strong=a.strong_aug)
            p = model(x, imu)
            # u は操舵に直結するので重く。s は小さく
            loss = 2.0 * F.smooth_l1_loss(p[:, 0], t[:, 0], beta=0.05) + F.smooth_l1_loss(p[:, 1], t[:, 1], beta=0.05) \
                + 0.5 * F.smooth_l1_loss(p[:, 2], t[:, 2], beta=0.05)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            step += 1
            run += loss.item()
            if (b + 1) % 50 == 0:
                print(f'ep {ep + 1} {b + 1}/{nb} loss {run / 50:.4f} {(time.time() - t0) / (b + 1):.2f} s/it', flush=True)
                run = 0.0
            if a.hours and time.time() - t_start > a.hours * 3600:
                break
        ev = evaluate(model, va) if va else {}
        hist.append({'epoch': ep + 1, 'minutes': round((time.time() - t0) / 60, 1), **ev})
        print('EPOCH', json.dumps(hist[-1], ensure_ascii=False), flush=True)
        torch.save({'model': model.state_dict(), 'opt': opt.state_dict(), 'sched': sched.state_dict(),
                    'epoch': ep + 1, 'step': step, 'hist': hist}, ck + '.tmp')
        os.replace(ck + '.tmp', ck)
        json.dump(hist, open(os.path.join(out, 'hist.json'), 'w'), ensure_ascii=False, indent=1)
        if a.hours and time.time() - t_start > a.hours * 3600:
            print('時間切れで打ち切り')
            break
    export(model, os.path.join(out, 'policy.onnx'))


if __name__ == '__main__':
    main()
