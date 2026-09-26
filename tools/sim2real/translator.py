# -*- coding: utf-8 -*-
"""CUT の生成器 (G.onnx) で BGR 画像を変換する (ROS に依存しない。convert_bag / eval / 将来のオンラインノードで共用)。"""
import os
import sys

import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_posts, post_mask, paint_posts, to_tensor_np, from_tensor_np  # noqa: E402


class Translator:
    def __init__(self, model, threads=0):
        so = ort.SessionOptions()
        if threads:
            so.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(model, so, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        self.mask = None

    def __call__(self, bgr):
        h, w = bgr.shape[:2]
        if self.mask is None or self.mask.shape != (h, w):
            self.mask = post_mask(h, w, load_posts())
        x = to_tensor_np(paint_posts(bgr, self.mask))[None]
        y = self.sess.run(None, {self.inp: x})[0][0]
        out = from_tensor_np(y)
        out[self.mask] = bgr[self.mask]          # 柱は入力のまま
        return out


