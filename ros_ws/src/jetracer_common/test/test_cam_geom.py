import math
from jetracer_common.cam_geom import CamGeom


def test_round_trip_ground():
    g = CamGeom(224, 224, 120.0, 0.12, 12.0, crop_top_frac=0.1, mount_x_m=0.1)
    for xb, yb in ((0.5, 0.0), (1.0, 0.2), (2.0, -0.3), (0.4, 0.15)):
        u, v, vis = g.project(xb, yb)
        assert vis, (xb, yb, u, v)
        back = g.ground_from_pixel(u, v)
        assert back is not None
        assert abs(back[0] - xb) < 1e-6 and abs(back[1] - yb) < 1e-6


def test_left_is_left_and_far_is_up():
    g = CamGeom(224, 224, 120.0, 0.12, 12.0)
    u0, v0, _ = g.project(1.0, 0.0)
    u1, v1, _ = g.project(1.0, 0.3)     # 左 → 画像では左 (u 小)
    u2, v2, _ = g.project(2.0, 0.0)     # 遠く → 上 (v 小)
    assert u1 < u0 and abs(v1 - v0) < 2.0
    assert v2 < v0 and abs(u2 - u0) < 1e-6


def test_above_horizon_is_none():
    g = CamGeom(224, 224, 120.0, 0.12, 12.0)
    assert g.ground_from_pixel(112, 0) is None or g.ground_from_pixel(112, 0)[0] > 5.0
