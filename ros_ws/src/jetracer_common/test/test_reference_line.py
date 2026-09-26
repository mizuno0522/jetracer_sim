import math
import numpy as np
from jetracer_common.reference_line import ReferenceLine


def circle(r=1.0, n=200):
    th = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.stack([r * np.cos(th), r * np.sin(th)], axis=1)


def test_circle_curvature_and_cte():
    rl = ReferenceLine(circle(1.0))
    assert abs(rl.total - 2 * math.pi) < 0.02
    s, d, psi, _ = rl.nearest(1.1, 0.0)        # 外側 0.1 m → 反時計回りで右 = 負
    assert abs(d + 0.1) < 0.01
    x, y, psi2, k = rl.lookahead(1.0, 0.0, math.pi / 2)
    assert abs(x) < 0.05 and abs(y - 1.0) < 0.05
    assert abs(k - 1.0) < 0.1


def test_progress_wraps():
    rl = ReferenceLine(circle(1.0))
    assert abs(rl.progress_delta(rl.total - 0.1, 0.1) - 0.2) < 1e-6
    assert rl.progress_delta(0.1, rl.total - 0.1) < 0
