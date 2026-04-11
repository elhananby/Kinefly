import numpy as np
import pytest

from kinefly.core.imaging import (
    PhaseCorrelation,
    PolarTransforms,
    TransformException,
    WindowFunctions,
    clip,
    clip_pt,
    filter_median,
    get_angle_from_points_i,
    get_intersection,
    get_projection_onto_axis,
    get_reflection_across_axis,
)


def test_get_angle_from_points_i():
    angle = get_angle_from_points_i(np.array([0, 0]), np.array([1, 0]))
    assert abs(angle) < 1e-10
    angle = get_angle_from_points_i(np.array([0, 0]), np.array([0, 1]))
    assert abs(angle - np.pi / 2) < 1e-10


def test_get_intersection():
    pt = get_intersection(
        np.array([0.0, 1.0]),
        np.array([2.0, 1.0]),
        np.array([1.0, 0.0]),
        np.array([1.0, 2.0]),
    )
    np.testing.assert_allclose(pt, [1.0, 1.0], atol=1e-10)


def test_get_projection_onto_axis():
    pt = get_projection_onto_axis(
        np.array([1.0, 1.0]),
        np.array([0.0, 0.0]),
        np.array([2.0, 0.0]),
    )
    np.testing.assert_allclose(pt, [1.0, 0.0], atol=1e-10)


def test_get_reflection_across_axis():
    pt = get_reflection_across_axis(
        np.array([1.0, 1.0]),
        np.array([0.0, 0.0]),
        np.array([2.0, 0.0]),
    )
    np.testing.assert_allclose(pt, [1.0, -1.0], atol=1e-10)


def test_filter_median():
    data = np.array([1.0, 100.0, 1.0, 1.0, 1.0])
    result = filter_median(data, q=1)
    assert result[2] == 1.0


def test_clip():
    assert clip(5, 0, 10) == 5
    assert clip(-1, 0, 10) == 0
    assert clip(15, 0, 10) == 10


def test_clip_pt():
    assert clip_pt((50, 50), (100, 100)) == (50, 50)
    assert clip_pt((-1, 200), (100, 100)) == (0, 99)


def test_polar_transforms_log():
    pt = PolarTransforms()
    img = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
    result = pt.transform_polar_log(img, 50, 50, nRho=30, nTheta=60)
    assert result.shape == (30, 60)


def test_polar_transforms_elliptical():
    pt = PolarTransforms()
    img = np.random.randint(0, 255, (200, 200), dtype=np.uint8)
    result = pt.transform_polar_elliptical(
        img,
        100,
        100,
        raxial=50,
        rortho=50,
        dradiusStrip=20,
        nRho=20,
        nTheta=40,
        theta_0=-0.5,
        theta_1=0.5,
        rClip=0.8,
    )
    assert result.shape[0] == 20
    assert result.shape[1] == 40


def test_polar_transforms_elliptical_empty_raises():
    pt = PolarTransforms()
    img = np.zeros((10, 10), dtype=np.uint8)
    with pytest.raises(TransformException):
        pt.transform_polar_elliptical(
            img,
            100,
            100,
            raxial=50,
            rortho=50,
            dradiusStrip=20,
            nRho=20,
            nTheta=40,
            theta_0=-0.5,
            theta_1=0.5,
            rClip=0.8,
        )


def test_phase_correlation_zero_shift():
    pc = PhaseCorrelation()
    img = np.random.rand(64, 64).astype(np.float32) * 255
    shift = pc.get_shift(img, img)
    np.testing.assert_allclose(shift, [0.0, 0.0], atol=1.0)


def test_window_functions_hanning_shape():
    wf = WindowFunctions()
    h = wf.create_hanning((32, 64))
    assert h.shape == (32, 64)
    assert h.dtype == np.float32
    assert h[0, 0] < 0.01
    assert h[16, 32] > 0.9


def test_window_functions_tukey_shape():
    wf = WindowFunctions()
    t = wf.create_tukey((32, 64))
    assert t.shape == (32, 64)
    assert t.dtype == np.float32
