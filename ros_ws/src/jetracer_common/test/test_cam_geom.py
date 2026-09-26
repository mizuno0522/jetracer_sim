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


def test_distortion_roundtrip():
    """樽型 (k1<0) と fx≠fy でも、歪み → 逆歪み・投影 → 床への逆投影が元に戻る。"""
    import numpy as np
    g = CamGeom(224, 224, 110.0, 0.10, 45.0, vfov_deg=80.0, k1=-0.05, k2=0.002)
    assert g.fy != g.fx
    xs = np.linspace(-1.2, 1.2, 25)
    xd, yd = g.distort(xs, xs[::-1] * 0.7)
    xu, yu = g.undistort(xd, yd)
    assert np.allclose(xu, xs, atol=1e-6) and np.allclose(yu, xs[::-1] * 0.7, atol=1e-6)
    for xb, yb in [(0.4, 0.0), (0.6, 0.25), (1.2, -0.4), (0.35, 0.15)]:
        u, v, vis = g.project(xb, yb)
        assert vis, (xb, yb, u, v)
        back = g.ground_from_pixel(u, v)
        assert back is not None
        assert abs(back[0] - xb) < 1e-4 and abs(back[1] - yb) < 1e-4


def test_barrel_pulls_edges_in_and_widens_fov():
    """樽型なら、同じ点が中心寄りに写り、画像の端から端の見込み角はピンホール等価の hfov より広い。"""
    g0 = CamGeom(224, 224, 100.0, 0.10, 30.0)
    g1 = CamGeom(224, 224, 100.0, 0.10, 30.0, k1=-0.05)
    u0, _, _ = g0.project(0.8, 0.5)
    u1, _, _ = g1.project(0.8, 0.5)
    assert abs(u1 - 112) < abs(u0 - 112)
    assert g1.true_fov_deg()[0] > 100.5
    assert abs(g0.true_fov_deg()[0] - 100.0) < 1e-6
    assert g1.D()[:2] == [-0.05, 0.0]


def test_rejects_impossible_distortion():
    """画角 120° に k1 = -0.05 (OpenCV の正規化座標) は四隅が折り返す範囲なので起動時に止める。"""
    import pytest
    with pytest.raises(ValueError):
        CamGeom(224, 224, 120.0, 0.10, 30.0, k1=-0.05)
    CamGeom(224, 224, 120.0, 0.10, 30.0, vfov_deg=90.0, k1=-0.02)     # これは可
