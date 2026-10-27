from pathlib import Path

import numpy as np
import pytest

from robot_safecontrol_moveit.pipe_assets.combined_pipe_model import combined_pipe_model

ASSET_ROOT = Path(__file__).parent / "data" / "pipe_assets"


def test_combined_pipe_geometry_matches_analytic_cylinder_reducer_and_bend():
    model = combined_pipe_model()
    assert model["lengthUnit"] == "mm"
    assert len(model["segments"]) == 5
    np.testing.assert_allclose(model["outletCenter"], [270, 0, 260], atol=1e-12)
    assert model["totalLength"] == pytest.approx(350 + 45 * np.pi)
    assert np.max(model["joinReport"]) < 1e-12

    straight = model["segments"][0]
    p, n, pu, pv = straight["evaluate"]([0, 100], [0, np.pi / 2])
    np.testing.assert_allclose(p, [[20, 0, 0], [0, 20, 100]], atol=1e-12)
    np.testing.assert_allclose(n, [[1, 0, 0], [0, 1, 0]], atol=1e-12)
    np.testing.assert_allclose(pu, [[0, 0, 1], [0, 0, 1]])
    np.testing.assert_allclose(pv, [[0, 20, 0], [-20, 0, 0]], atol=1e-12)

    p, n, _, _ = model["segments"][1]["evaluate"]([35], [0])
    np.testing.assert_allclose(p, [[25, 0, 135]], atol=1e-12)
    expected = np.array([[1, 0, -18.75 / 70]])
    expected /= np.linalg.norm(expected, axis=1, keepdims=True)
    np.testing.assert_allclose(n, expected, atol=1e-12)

    p, n, _, _ = model["segments"][2]["evaluate"]([np.pi / 2], [0])
    np.testing.assert_allclose(p, [[90, 0, 230]], atol=1e-12)
    np.testing.assert_allclose(n, [[0, 0, -1]], atol=1e-12)
    with pytest.raises(ValueError, match="u"):
        straight["evaluate"]([101], [0])


def test_demo_models_preserve_full_geometry_and_surface_derivatives():
    from scipy.special import betainc

    from robot_safecontrol_moveit.pipe_assets.pipe_complex_demo_model import (
        pipe_complex_demo_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model

    model = pipe_demo_model()
    u = np.array([0.0, 35.0, 70.0, 105.0, 140.0])
    v = np.linspace(0.0, np.pi, len(u))
    p, n, pu, pv = model["segments"][1]["evaluate"](u, v)
    np.testing.assert_allclose(np.linalg.norm(p[:, :2], axis=1), 35 + 20 * betainc(4, 4, u / 140))
    np.testing.assert_allclose(p[:, 2], 180 + u)
    np.testing.assert_allclose(np.linalg.norm(n, axis=1), 1)
    np.testing.assert_allclose(np.sum(n * pu, axis=1), 0, atol=1e-12)
    np.testing.assert_allclose(np.sum(n * pv, axis=1), 0, atol=1e-12)
    np.testing.assert_allclose(n[2], [0, 1 / np.sqrt(1 + 0.3125**2), -0.3125 / np.sqrt(1 + 0.3125**2)], atol=1e-12)

    model = pipe_complex_demo_model(2)
    assert len(model["segments"]) == 17
    assert model["totalCenterlineLengthMM"] == pytest.approx(2 * (4110 + 1070 * np.pi / 3))
    for segment in model["segments"]:
        lo, hi = segment["uRange"]
        u = np.linspace(lo, hi, 9)
        v = np.linspace(-2, 9, len(u))
        p, n, pu, pv = segment["evaluate"](u, v)
        center = segment["centerline"](u)
        np.testing.assert_allclose(center[[0, -1]], [segment["startCenter"], segment["endCenter"]], atol=1e-10)
        radii = segment["radiusInMM"] + (segment["radiusOutMM"] - segment["radiusInMM"]) * betainc(3, 3, (u - lo) / (hi - lo))
        np.testing.assert_allclose(np.linalg.norm(p - center, axis=1), radii, atol=1e-10)
        np.testing.assert_allclose(np.sum(n * pu, axis=1), 0, atol=1e-10)
        np.testing.assert_allclose(np.sum(n * pv, axis=1), 0, atol=1e-10)
        delta = (hi - lo) * 1e-6
        p1 = segment["evaluate"](u[1:-1] + delta, v[1:-1])[0]
        p0 = segment["evaluate"](u[1:-1] - delta, v[1:-1])[0]
        np.testing.assert_allclose((p1 - p0) / (2 * delta), pu[1:-1], rtol=2e-8, atol=1e-6)
    for a, b in zip(model["segments"][:-1], model["segments"][1:]):
        v = np.linspace(-np.pi, np.pi, 65)
        pa, na, _, _ = a["evaluate"](np.full_like(v, a["uRange"][1]), v)
        pb, nb, _, _ = b["evaluate"](np.full_like(v, b["uRange"][0]), v)
        np.testing.assert_allclose(pa, pb, atol=1e-10)
        np.testing.assert_allclose(na, nb, atol=1e-12)
    with pytest.raises(ValueError, match="lengthScale"):
        pipe_complex_demo_model(0.1)


def test_prepared_model_maps_canvas_to_right_segment_and_normal_offset():
    from robot_safecontrol_moveit.pipe_assets.pipe_complex_demo_model import (
        pipe_complex_demo_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
    from robot_safecontrol_moveit.pipe_assets.pipe_prepare_model import (
        pipe_prepare_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_surface_eval import pipe_surface_eval

    for source in (pipe_demo_model(), combined_pipe_model(), pipe_complex_demo_model()):
        model = pipe_prepare_model(source)
        assert model["totalLength"] == pytest.approx(sum(s["canvasLength"] for s in model["segments"]))
        assert np.max(model["joinReport"]) < 1e-10

    source = pipe_demo_model()
    for segment in source["segments"]:
        segment["thetaSign"] = -1
        segment["thetaOffsetRad"] = 0.3
    model = pipe_prepare_model(source)
    uv = np.array([[0, 0], [180, 90], [320, 450]])
    tcp, surface, normal, ids, local = pipe_surface_eval(model, uv, 5)
    np.testing.assert_array_equal(ids, [1, 2, 2])
    np.testing.assert_allclose(local[:, 0], [0, 0, 140])
    np.testing.assert_allclose(local[:, 1], 0.3 - np.deg2rad(uv[:, 1]))
    np.testing.assert_allclose(np.linalg.norm(surface[:, :2], axis=1), [35, 35, 55])
    np.testing.assert_allclose(tcp - surface, 5 * normal)
    np.testing.assert_allclose(np.linalg.norm(normal, axis=1), 1)
    empty = pipe_surface_eval(model, np.empty((0, 2)), 0)
    assert [a.shape for a in empty] == [(0, 3), (0, 3), (0, 3), (0,), (0, 2)]
    with pytest.raises(ValueError, match="范围"):
        pipe_surface_eval(model, [[321, 0]], 0)
    with pytest.raises(ValueError, match="offsetMM"):
        pipe_surface_eval(model, [[0, 0]], -1)
    with pytest.raises(ValueError, match="mm"):
        pipe_prepare_model(dict(source, lengthUnit="m"))


def test_parameter_curves_preserve_c2_return_paths_and_analytic_sine():
    from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
    from robot_safecontrol_moveit.pipe_assets.pipe_path_curve import pipe_path_curve
    from robot_safecontrol_moveit.pipe_assets.pipe_prepare_model import (
        pipe_prepare_model,
    )

    model = pipe_prepare_model(pipe_demo_model())
    anchors = np.array([[0, 0], [320, 90], [0, 180], [320, 270], [0, 360]])
    curve = pipe_path_curve(model, anchors)
    uv, _, _ = curve["evaluate"](curve["breaks"])
    np.testing.assert_allclose(uv, anchors, atol=1e-10)
    uv, d1, d2 = curve["evaluate"](np.linspace(0, 1, 10001))
    assert np.min(uv[:, 0]) >= -1e-10
    assert np.max(uv[:, 0]) <= 320 + 1e-10
    assert np.any(np.diff(uv[:, 0]) < -0.01)
    assert np.all(np.isfinite(d1)) and np.all(np.isfinite(d2))
    knots = curve["breaks"][1:-1]
    h = np.min(np.diff(curve["breaks"])) * 1e-7
    left = 2 * curve["evaluate"](knots - h)[2] - curve["evaluate"](knots - 2 * h)[2]
    right = 2 * curve["evaluate"](knots + h)[2] - curve["evaluate"](knots + 2 * h)[2]
    assert np.max(np.abs(left - right)) / max(1, np.max(np.abs([left, right]))) < 1e-6

    definition = {"type": "analytic-sine", "xStart": 16, "xEnd": 304,
                  "thetaMidDeg": 180, "thetaAmplitudeDeg": 85, "cycles": 1}
    t = np.linspace(0, 1, 18)
    anchors = np.column_stack((16 + 288 * t, 180 + 85 * np.sin(2 * np.pi * t)))
    curve = pipe_path_curve(model, anchors, definition)
    uv, d1, d2 = curve["evaluate"](t)
    np.testing.assert_allclose(uv, anchors, atol=1e-12)
    np.testing.assert_allclose(d1[:, 0], 288)
    np.testing.assert_allclose(d1[:, 1], 170 * np.pi * np.cos(2 * np.pi * t))
    np.testing.assert_allclose(d2[:, 1], -340 * np.pi**2 * np.sin(2 * np.pi * t))
    with pytest.raises(ValueError, match="匹配"):
        pipe_path_curve(model, anchors + [0, 1], definition)
    with pytest.raises(ValueError, match="参数"):
        curve["evaluate"]([-0.01])


@pytest.mark.parametrize("stem,count", [("pipe_path_20261013_170314", 407),
                                         ("pipe_path_20261013_180806", 480),
                                         ("pipe_path_20261014_165040", 743)])
def test_reference_sampling_reproduces_original_mat_trajectories(stem, count):
    from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat
    from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
    from robot_safecontrol_moveit.pipe_assets.pipe_path_curve import pipe_path_curve
    from robot_safecontrol_moveit.pipe_assets.pipe_prepare_model import (
        pipe_prepare_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_reference_path import (
        pipe_reference_path,
    )

    source = load_mat(ASSET_ROOT / f"{stem}.mat")["trajectory"]
    model = pipe_prepare_model(pipe_demo_model())
    anchors, definition = source["keypointsCanvas"], source["pathDefinition"]
    curve = pipe_path_curve(model, anchors, definition)
    np.testing.assert_allclose(curve["evaluate"](source["pathParameter"])[0], source["canvasUV"], atol=1e-9)
    sample = pipe_reference_path(model, anchors, source["offsetMM"], source["sampleSpacingMM"], definition)
    assert sample["tcpPoints"].shape == (count, 3)
    for field in ("canvasUV", "localUV", "surfacePoints", "tcpPoints", "normals", "toolAxis", "arcLengthMM"):
        np.testing.assert_allclose(sample[field], source[field], atol=2e-7, rtol=0, err_msg=field)
    np.testing.assert_array_equal(sample["segmentId"], source["segmentId"])
    np.testing.assert_allclose(sample["pathParameter"], source["pathParameter"], atol=1e-9, rtol=0)
    assert np.all(np.diff(sample["pathParameter"]) > 0)
    assert np.max(np.diff(sample["arcLengthMM"])) < 1.001


def test_continuous_tcp_curvature_matches_circle_line_helix_and_analytic_sine():
    from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
    from robot_safecontrol_moveit.pipe_assets.pipe_path_curvature import (
        pipe_path_curvature,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_prepare_model import (
        pipe_prepare_model,
    )

    model = pipe_prepare_model(pipe_demo_model())
    cases = [([[80, 0], [80, 90], [80, 180], [80, 270]], 1 / 40, 2e-5),
             ([[10, 32], [55, 32], [100, 32], [170, 32]], 0, 2e-6)]
    x = np.linspace(10, 170, 6)
    b = np.deg2rad(1.1)
    cases.append((np.column_stack((x, 1.1 * x + 5)), 40 * b**2 / (1 + 40**2 * b**2), 2e-5))
    for anchors, expected, tolerance in cases:
        result = pipe_path_curvature(model, anchors, 5)
        valid = result["valid"]
        assert valid.dtype == np.bool_
        assert np.count_nonzero(valid) >= 1201
        assert np.max(np.abs(result["kappa"][valid] - expected)) < tolerance
        assert np.all(np.diff(result["arcLengthMM"]) >= 0)
        assert result["curvatureUnit"] == "1/mm"

    definition = {"type": "analytic-sine", "xStart": 16, "xEnd": 304,
                  "thetaMidDeg": 180, "thetaAmplitudeDeg": 85, "cycles": 1}
    t = np.linspace(0, 1, 18)
    anchors = np.column_stack((16 + 288 * t, 180 + 85 * np.sin(2 * np.pi * t)))
    result = pipe_path_curvature(model, anchors, 5, definition)
    inside = result["valid"] & (result["canvasUV"][:, 0] > 20) & (result["canvasUV"][:, 0] < 175)
    x = result["canvasUV"][inside, 0]
    frequency, amplitude, radius = 2 * np.pi / 288, np.deg2rad(85), 40
    b = amplitude * frequency * np.cos(frequency * (x - 16))
    c = -amplitude * frequency**2 * np.sin(frequency * (x - 16))
    expected = np.sqrt(radius**2 * (c**2 + b**4) + radius**4 * b**6) / (1 + radius**2 * b**2)**1.5
    assert np.max(np.abs(result["kappa"][inside] - expected)) < 3e-5
    assert np.ptp(result["kappa"][inside]) > 0.005


def test_surface_mapping_accepts_every_spatial_model_boundary_and_endpoint():
    from robot_safecontrol_moveit.pipe_assets.pipe_complex_demo_model import (
        pipe_complex_demo_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_prepare_model import (
        pipe_prepare_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_surface_eval import pipe_surface_eval

    for scale in (0.3, 1, 2, 10):
        model = pipe_prepare_model(pipe_complex_demo_model(scale))
        uv = np.column_stack((model["breaks"], np.linspace(0, 1080, len(model["breaks"]))))
        tcp, _surface, _n, segmentId, local = pipe_surface_eval(model, uv, 5)
        assert tcp.shape == (18, 3)
        assert np.all(np.isfinite(tcp))
        np.testing.assert_array_equal(segmentId, np.r_[np.arange(1, 18), 17])
        assert local[-1, 0] == pytest.approx(model["segments"][-1]["uRange"][1])


def test_reference_paths_preserve_return_order_and_empty_or_single_point_shapes():
    from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
    from robot_safecontrol_moveit.pipe_assets.pipe_path_curvature import (
        pipe_path_curvature,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_prepare_model import (
        pipe_prepare_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_reference_path import (
        pipe_reference_path,
    )

    model = pipe_prepare_model(pipe_demo_model())
    paths = ([[0, 0], [30, 40], [75, 10], [125, 70], [180, 20], [240, 100], [320, 60]],
             [[20, 0], [150, 90], [90, 180], [210, 270], [130, 360], [290, 450]],
             [[0, 0], [320, 90], [0, 180], [320, 270], [0, 360]])
    for index, anchors in enumerate(paths):
        sample = pipe_reference_path(model, anchors, 5, 1)
        np.testing.assert_allclose(sample["canvasUV"][[0, -1]], [anchors[0], anchors[-1]], atol=1e-8)
        assert np.all(np.isfinite(sample["tcpPoints"]))
        assert np.all(np.diff(sample["arcLengthMM"]) > 0)
        assert np.min(sample["canvasUV"][:, 0]) >= -1e-8
        assert np.max(sample["canvasUV"][:, 0]) <= 320 + 1e-8
        if index:
            assert np.any(np.diff(sample["canvasUV"][:, 0]) < -0.1)
        curvature = pipe_path_curvature(model, anchors, 5)
        assert np.count_nonzero(curvature["valid"]) >= 3
        assert np.all(curvature["kappa"][curvature["valid"]] >= 0)
    empty = pipe_reference_path(model, [], 5)
    assert empty["tcpPoints"].shape == (0, 3)
    assert empty["arcLengthMM"].shape == (0,)
    point = pipe_reference_path(model, [[10, 30], [10, 30]], 5, 1)
    assert point["tcpPoints"].shape == (1, 3)
    np.testing.assert_array_equal(point["pathParameter"], [0])
    curvature = pipe_path_curvature(model, [[10, 30]], 5)
    np.testing.assert_array_equal(curvature["valid"], [False])
    assert np.isnan(curvature["kappa"][0])
