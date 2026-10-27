from pathlib import Path

import numpy as np
import pytest
from scipy.interpolate import BSpline

from robot_safecontrol_moveit.pipe_assets.fit_pipe_path_nurbs import (
    fit_pipe_path_nurbs,
)
from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat, save_mat

DATA = Path(__file__).parent / "data/pipe_assets"


@pytest.mark.parametrize("stem,count", [
    ("pipe_path_20261013_170314", 407),
    ("pipe_path_20261013_180806", 480),
    ("pipe_path_20261014_165040", 743),
])
def test_source_paths_interpolate_and_export_the_same_tcp_points(stem, count, tmp_path):
    obj, result = fit_pipe_path_nurbs(DATA / f"{stem}.mat", tmp_path / stem, False)
    csv_obj, csv_result = fit_pipe_path_nurbs(DATA / f"{stem}.csv", "", False)
    source = load_mat(DATA / f"{stem}.mat")
    expected = source["data_points"]
    assert expected.shape == (count, 3)
    assert result["length_unit"] == "mm"
    np.testing.assert_allclose(result["data_points"], expected, atol=0, rtol=0)
    np.testing.assert_allclose(obj["ControlPoints"], csv_obj["ControlPoints"], atol=1e-9, rtol=0)
    if count == 480:
        np.testing.assert_allclose(obj["ControlPoints"], np.loadtxt(DATA / "control_points.txt"), atol=1e-9, rtol=0)
        np.testing.assert_allclose(obj["UVector"], np.loadtxt(DATA / "knot_vector.txt"), atol=1e-14, rtol=0)
    np.testing.assert_allclose(csv_result["data_points"], expected, atol=1e-10, rtol=0)
    evaluated = BSpline(obj["UVector"], obj["ControlPoints"], 3)(result["data_parameters"])
    np.testing.assert_allclose(evaluated, expected, atol=1e-9, rtol=0)
    assert result["curve_points"].shape == ((count - 1) * 5 + 1, 3)
    exported = np.genfromtxt(tmp_path / stem / "curve_data.csv", delimiter=",", names=True)
    paired = exported["is_input_point"] == 1
    np.testing.assert_array_equal(exported["input_row"][paired], np.arange(1, count + 1))
    assert np.isnan(exported["point_error_mm"][~paired]).all()
    assert sorted(path.name for path in (tmp_path / stem).iterdir()) == [
        "control_points.txt", "curve_data.csv", "knot_vector.txt",
    ]


def test_geometry_evaluates_analytic_derivatives_and_inverts_source_arclength():
    from robot_safecontrol_moveit.pipe_assets.pipe_nurbs_geometry import (
        pipe_nurbs_geometry,
    )

    line = {"ctrl_pts": np.column_stack([np.linspace(0, 100, 4), np.zeros((4, 2))]),
            "knots": np.r_[np.zeros(4), np.ones(4)], "p": 3, "w": np.ones(4)}
    u = np.linspace(0, 1, 17)
    position, first, second, third = pipe_nurbs_geometry("eval", line, u)
    np.testing.assert_allclose(position[:, 0], 100 * u, atol=3e-14)
    np.testing.assert_allclose(first, np.tile([100, 0, 0], (17, 1)), atol=6e-14)
    np.testing.assert_allclose(second, 0, atol=3e-13)
    np.testing.assert_allclose(third, 0, atol=3e-13)
    source = load_mat(DATA / "nurbs_blocks.mat")
    geometry = pipe_nurbs_geometry("load", DATA)
    arc = pipe_nurbs_geometry("arc", geometry, source["raw_boundary_u"])
    np.testing.assert_allclose(arc["block_s"], source["arc_table"]["block_s"], atol=1e-9, rtol=0)
    assert arc["total_length"] == pytest.approx(478.7913638426983, abs=1e-9)
    target = np.linspace(0, arc["total_length"], 105)
    recovered, residual = pipe_nurbs_geometry("invert", geometry, arc, target)
    assert recovered[0] == 0 and recovered[-1] == 1
    assert np.max(np.abs(residual)) <= 1e-10
    with pytest.raises(ValueError, match="Parameter"):
        pipe_nurbs_geometry("eval", geometry, [-0.1])
    with pytest.raises(ValueError, match="ArcTarget"):
        pipe_nurbs_geometry("invert", geometry, arc, [-1])


def test_curvature_selection_reproduces_saved_original_sample_indices():
    from robot_safecontrol_moveit.pipe_assets.pipe_curvature_keypoints import (
        pipe_curvature_keypoints,
    )

    source = load_mat(DATA / "nurbs_blocks.mat")
    expected = source["peak_selection_audit"]
    indices, audit = pipe_curvature_keypoints(
        source["u_list"], source["kappa_list"], expected["thresholds"]["CurvatureThreshold"],
        source["scan_arc_mm"], source["peak_selection_options"],
    )
    np.testing.assert_array_equal(indices, expected["selected_indices"])
    np.testing.assert_array_equal(audit["raw_indices"], expected["raw_indices"])
    np.testing.assert_array_equal(audit["removed_indices"], expected["removed_indices"])
    with pytest.raises(ValueError, match="Options"):
        pipe_curvature_keypoints([0, 1], [0, 1], .1, [0, 1], {"unknown": 1})


def test_fit_preserves_return_visits_and_rejects_conflicting_or_invalid_inputs(tmp_path):
    points = np.array([[0., 0., 0.], [1., 0., 0.], [1., 0., 0.], [1., 2., 0.], [0., 0., 0.], [0., 3., 1.]])
    filename = tmp_path / "path.mat"
    save_mat(filename, {"data_points": points})
    _, result = fit_pipe_path_nurbs(filename)
    np.testing.assert_array_equal(result["input_indices"], [1, 2, 4, 5, 6])
    np.testing.assert_array_equal(result["removed_input_indices"], [3])
    save_mat(filename, {"data_points": points, "trajectory": {"tcpPoints": points + 1, "lengthUnit": "mm"}})
    with pytest.raises(ValueError, match="ConflictingCoordinates"):
        fit_pipe_path_nurbs(filename)
    save_mat(filename, {"trajectory": {"tcpPoints": points, "lengthUnit": "m"}})
    with pytest.raises(ValueError, match="InvalidUnit"):
        fit_pipe_path_nurbs(filename)
    save_mat(filename, {"data_points": points[:3]})
    with pytest.raises(ValueError, match="TooFewPoints"):
        fit_pipe_path_nurbs(filename)


def test_geometry_rejects_complex_controls_and_nonunit_weights():
    from robot_safecontrol_moveit.pipe_assets.pipe_nurbs_geometry import (
        pipe_nurbs_geometry,
    )

    geometry = pipe_nurbs_geometry("load", DATA)
    controls = geometry["ctrl_pts"].astype(complex)
    controls[0, 0] += 1j
    with pytest.raises(ValueError, match="ControlPoints"):
        pipe_nurbs_geometry("eval", geometry | {"ctrl_pts": controls}, [0., 1.])
    with pytest.raises(ValueError, match="Weights"):
        pipe_nurbs_geometry("eval", geometry | {"w": np.full(len(controls), 2.)}, [0., 1.])


def test_fit_plot_applies_project_fonts_and_preserves_curve_without_changing_backend():
    import matplotlib as mpl
    import matplotlib.pyplot as plt

    backend = mpl.get_backend()
    with mpl.rc_context({"font.family": ["DejaVu Sans"]}):
        obj, result = fit_pipe_path_nurbs(DATA / "pipe_path_20261013_180806.mat", "", True)
        figure = plt.gcf()
        try:
            assert mpl.rcParams["font.family"] == ["Times New Roman", "SimSun"]
            assert mpl.get_backend() == backend
            axes = figure.axes[0]
            assert axes.get_xlabel() == "X (mm)"
            assert axes.get_ylabel() == "Y (mm)"
            assert axes.get_zlabel() == "Z (mm)"
            assert axes.lines[0].get_label() == "NURBS 插值曲线"
            np.testing.assert_array_equal(np.column_stack(axes.lines[0].get_data_3d()), result["curve_points"])
            assert obj["Number_of_ControlPoints"] == 480
        finally:
            plt.close(figure)
