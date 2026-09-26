# -*- coding: utf-8 -*-
"""
NVIDIA JetRacer 標準のコード (jetracer.nvidia_racecar.NvidiaRacecar / jetcam.csi_camera.CSICamera) を、
無改造のまま sim につなぐ互換レイヤ。コードの先頭に次の 2 行を足すだけ:

    import jetracer_compat
    jetracer_compat.install()      # 以後の `from jetracer.nvidia_racecar import NvidiaRacecar` 等が sim 版になる

指令の変換は実機と同じ (正規化値 × gain + offset → PWM → jetracer_bridge の較正の逆写像)。
ROS の環境 (source scripts/sim_env.sh) を読み込んだ端末から Jupyter / python を起動すること。
"""
import sys
import types

from .camera import Camera, CSICamera          # noqa: F401
from .racecar import NvidiaRacecar, Racecar    # noqa: F401


def install():
    """jetracer / jetcam のモジュール名を sim 版に差し替える (本物が入っていても上書き)。"""
    def mod(name, **attrs):
        m = sys.modules.get(name) or types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m
    jr = mod('jetracer', Racecar=Racecar)
    jr.__path__ = getattr(jr, '__path__', [])
    jr.nvidia_racecar = mod('jetracer.nvidia_racecar', NvidiaRacecar=NvidiaRacecar)
    jr.racecar = mod('jetracer.racecar', Racecar=Racecar)
    jc = mod('jetcam')
    jc.__path__ = getattr(jc, '__path__', [])
    jc.csi_camera = mod('jetcam.csi_camera', CSICamera=CSICamera)
    jc.camera = mod('jetcam.camera', Camera=Camera)
