import os
from pathlib import Path

import numpy as np
import pytest

from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat
from robot_safecontrol_moveit.pipe_assets.pipe_pose_math import pipe_pose_math

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path(os.environ.get("PIPE_ASSET_ROOT", ROOT / "tests/data/pipe_assets"))
FIT = SOURCE
SPEED = SOURCE
POSE = SOURCE


@pytest.fixture(scope="module")
def source_pose():
    with np.load(POSE / "pose_samples.npz") as saved:
        result = {key[5:]: saved[key] for key in saved.files if key.startswith("top__")}
        result["pose"] = {key[6:]: saved[key] for key in saved.files if key.startswith("pose__")}
    return result


@pytest.fixture(scope="module")
def source_reference():
    return load_mat(POSE / "pose_reference.mat")["reference"]


def test_pose_math_reproduces_real_axis_polynomial_and_normalization(source_pose, source_reference):
    axis = source_reference["axis"]
    sample = source_pose["pose"]
    length = axis["length_mm"]
    basis = pipe_pose_math("basis", axis["knots"], axis["degree"], sample["arc_mm"] / length)
    jets = [basis[order] @ axis["controls"] / length**order for order in range(3)]
    np.testing.assert_allclose(jets[0], sample["z"], atol=1e-12, rtol=0)
    normalized = pipe_pose_math("normalize", *jets)
    np.testing.assert_allclose(normalized[0], source_pose["tool_axis_ref_W"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(normalized[1], sample["daxis_darc"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(normalized[2], sample["d2axis_darc2"], atol=1e-12, rtol=0)


def test_axis_and_frame_queries_reproduce_real_pose(source_pose, source_reference):
    from robot_safecontrol_moveit.pipe_assets.pipe_axis_smoothing import (
        pipe_axis_smoothing,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_orientation_reference import (
        pipe_orientation_reference,
    )

    sample = source_pose["pose"]
    axis = pipe_axis_smoothing("eval", source_reference["axis"], sample["arc_mm"])
    frame = pipe_orientation_reference("eval", source_reference["orientation"], sample["arc_mm"])
    np.testing.assert_allclose(axis["tool_axis_ref_W"], source_pose["tool_axis_ref_W"], atol=1e-12, rtol=0)
    for field in ("R_WT", "dR_darc", "d2R_darc2", "beta_W", "dbeta_darc_W", "quat_wxyz"):
        np.testing.assert_allclose(frame[field], sample[field], atol=1e-12, rtol=0)


def test_real_axis_rebuild_uses_constrained_qp_and_interval_bounds(source_reference):
    from robot_safecontrol_moveit.pipe_assets.pipe_axis_smoothing import (
        pipe_axis_smoothing,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_options import pipe_pose_options

    original = source_reference["axis"]
    cfg = pipe_pose_options({"show_figures": False})
    rebuilt = pipe_axis_smoothing("build", original["sample_arc_mm"], original["raw_axes"], cfg, source_reference["events_arc_mm"])
    np.testing.assert_allclose(rebuilt["controls"], original["controls"], atol=1e-7, rtol=0)
    assert np.min(rebuilt["norm_lower_bound_by_interval"]) >= cfg["rho_z"]
    assert max(rebuilt["solver"]["output"]["kkt_residuals"].values()) < 1e-7


def test_time_law_preserves_real_arclength_and_integer_period_stretch(source_pose):
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_time_law import (
        pipe_pose_time_law,
    )

    plan = load_mat(SPEED / "nurbs_blocks.mat")["motion_plan"]
    state = pipe_pose_time_law("eval", plan, source_pose["time_series"])
    np.testing.assert_allclose(state["s"], source_pose["pose"]["arc_mm"], atol=1e-10, rtol=0)
    np.testing.assert_allclose(state["v"], source_pose["feedrate_cmd_series"], atol=1e-10, rtol=0)
    np.testing.assert_allclose(state["a"], source_pose["tangent_acc_cmd_series"], atol=1e-10, rtol=0)
    stretched = pipe_pose_time_law("stretch", plan, 1.35)
    assert stretched["pose_time_stretch"] == 2
    slowed = pipe_pose_time_law("eval", stretched, source_pose["time_series"] * 2)
    np.testing.assert_allclose(slowed["s"], state["s"], atol=1e-10, rtol=0)
    np.testing.assert_allclose(slowed["v"], state["v"] / 2, atol=1e-10, rtol=0)
    np.testing.assert_allclose(slowed["a"], state["a"] / 4, atol=1e-10, rtol=0)


def test_original_surface_correspondence_preserves_rows_and_normals(source_pose, source_reference):
    from robot_safecontrol_moveit.pipe_assets.pipe_nurbs_geometry import (
        pipe_nurbs_geometry,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_options import pipe_pose_options
    from robot_safecontrol_moveit.pipe_assets.pipe_surface_correspondence import (
        pipe_surface_correspondence,
    )

    cfg = pipe_pose_options({"show_figures": False})
    geometry = pipe_nurbs_geometry("load", FIT)
    original = source_reference["correspondence"]
    pairs = {"input_indices": original["input_indices"], "data_parameters": original["data_parameters"], "data_points": original["source_snapshot"]["data_points"]}
    correspondence = pipe_surface_correspondence("build", geometry, SOURCE / "pipe_path_20261013_180806.mat", pairs, cfg)
    query = pipe_surface_correspondence("eval", correspondence, source_pose["u_series"])
    np.testing.assert_allclose(query["normal_raw_W"], source_pose["normal_raw_W"], atol=1e-10, rtol=0)
    np.testing.assert_allclose(correspondence["events_u"], original["events_u"], atol=1e-10, rtol=0)
    invalid = dict(pairs, input_indices=np.asarray(pairs["input_indices"])[::-1])
    with pytest.raises(ValueError, match="Pairs"):
        pipe_surface_correspondence("build", geometry, SOURCE / "pipe_path_20261013_180806.mat", invalid, cfg)


def test_full_reference_queries_match_source_pose_and_reject_wrong_geometry(source_pose, source_reference):
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_reference import (
        pipe_pose_reference,
    )

    reference = pipe_pose_reference("load", POSE / "pose_reference.mat")
    result = pipe_pose_reference("eval_arclength_state", reference, source_pose["pose"]["arc_mm"], source_pose["feedrate_cmd_series"], source_pose["tangent_acc_cmd_series"], source_pose["time_series"], source_pose["u_series"])
    np.testing.assert_allclose(result["position_mm"], source_pose["position_series"], atol=1e-9, rtol=0)
    for field in ("normal_raw_W", "tool_axis_ref_W", "R_WT", "omega_W", "alpha_W"):
        np.testing.assert_allclose(result[field], source_pose[field], atol=1e-8, rtol=0)
    wrong = dict(source_reference["geometry"], ctrl_pts=np.asarray(source_reference["geometry"]["ctrl_pts"]) + [1, 0, 0])
    with pytest.raises(ValueError, match="Identity"):
        pipe_pose_reference("load", POSE / "pose_reference.mat", wrong)


def test_analytic_angular_jerk_matches_real_samples_and_keeps_one_sided_nodes(source_pose, source_reference):
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_angular_jerk import (
        pipe_pose_angular_jerk,
    )

    plan = load_mat(SPEED / "nurbs_blocks.mat")["motion_plan"]
    result = pipe_pose_angular_jerk("sample", source_reference, plan, source_pose["time_series"])
    np.testing.assert_allclose(result["angular_jerk_W"], source_pose["angular_jerk_W"], atol=1e-7, rtol=0)
    assert np.any(np.linalg.norm(result["break_left_W"] - result["break_right_W"], axis=1) > 1e-3)
    assert np.count_nonzero(np.all(np.isnan(result["plot_values_W"]), axis=1)) == len(result["break_time_s"])


def test_real_reference_geometry_and_continuous_motion_validation(source_reference):
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_options import pipe_pose_options
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_validation import (
        pipe_pose_validation,
    )

    cfg = pipe_pose_options({"show_figures": False})
    geometry = pipe_pose_validation("geometry", source_reference, cfg)
    assert geometry["passed"]
    assert geometry["fidelity"]["hard_limit"]["status"] == "not_configured"
    assert geometry["fidelity"]["max_rad"] == pytest.approx(np.deg2rad(0.762101317), abs=1e-7)
    plan = load_mat(SPEED / "nurbs_blocks.mat")["motion_plan"]
    motion = pipe_pose_validation("motion", source_reference, plan, cfg)
    assert motion["passed"]
    assert motion["peak_converged"]
    assert motion["omega"]["measured_max"] == pytest.approx(1.4749246270876626, abs=1e-7)


def test_rebuild_schedule_export_and_reload_run_without_matlab(source_pose, source_reference, tmp_path):
    from robot_safecontrol_moveit.pipe_assets.pipe_nurbs_geometry import (
        pipe_nurbs_geometry,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_export import pipe_pose_export
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_options import pipe_pose_options
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_reference import (
        pipe_pose_reference,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_schedule import (
        pipe_pose_schedule,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_validation import (
        pipe_pose_validation,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_surface_correspondence import (
        pipe_surface_correspondence,
    )

    cfg = pipe_pose_options({"show_figures": False})
    geometry = pipe_nurbs_geometry("load", FIT)
    original = source_reference["correspondence"]
    pairs = {"input_indices": original["input_indices"], "data_parameters": original["data_parameters"], "data_points": original["source_snapshot"]["data_points"]}
    correspondence = pipe_surface_correspondence("build", geometry, SOURCE / "pipe_path_20261013_180806.mat", pairs, cfg)
    reference = pipe_pose_reference("build", geometry, correspondence, cfg)
    check = pipe_pose_validation("geometry", reference, cfg)
    assert check["passed"]
    plan, data, pose, schedule = pipe_pose_schedule(reference, load_mat(SPEED / "nurbs_blocks.mat"), cfg)
    assert schedule["passed"]
    expected = load_mat(SPEED / "realtime_interpolation_results.mat")
    np.testing.assert_allclose(data["position"], expected["position_series"], atol=1e-9, rtol=0)
    validation = {"geometry": check, "schedule": schedule, "passed": True}
    exported = pipe_pose_export(tmp_path, reference, plan, data, pose, validation)
    reloaded = pipe_pose_reference("load", tmp_path / "pose_reference.mat", geometry, cfg)
    query = pipe_pose_reference("eval_progress", reloaded, source_pose["u_series"])
    np.testing.assert_allclose(query["R_WT"], source_pose["R_WT"], atol=1e-7, rtol=0)
    saved = load_mat(tmp_path / "ik_input.mat")["ik_input"]
    np.testing.assert_array_equal(saved["angular_jerk_W"], exported["angular_jerk_W"])


def test_a4_entry_builds_verified_output_and_eight_real_figures(tmp_path):
    from matplotlib import pyplot as plt

    from robot_safecontrol_moveit.pipe_assets.A_4_zitai_gouzao import A_4_zitai_gouzao
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_plot import pipe_pose_plot

    result = A_4_zitai_gouzao({"show_figures": False}, SOURCE / "pipe_path_20261013_180806.mat", tmp_path, FIT, SPEED / "nurbs_blocks.mat")
    assert result["validation"]["passed"]
    assert result["ik_input"]["num_points"] == 4756
    assert (tmp_path / "pose_diagnostics.csv").is_file()
    figures = pipe_pose_plot(result["data"], result["pose"])
    assert len(figures) == 8
    assert np.any(np.isnan(figures[-2].axes[0].lines[0].get_ydata()))
    for figure in figures:
        plt.close(figure)


def test_common_slowdown_obeys_angular_limits_and_rejects_minimum_feed_conflict(source_reference):
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_options import pipe_pose_options
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_schedule import (
        pipe_pose_schedule,
    )

    offline = load_mat(SPEED / "nurbs_blocks.mat")
    cfg = pipe_pose_options({"show_figures": False, "omega_max_rad_s": 1.1})
    plan, data, pose, audit = pipe_pose_schedule(source_reference, offline, cfg)
    assert audit["passed"]
    assert audit["time_stretch"] == 2
    assert data["time"][-1] == pytest.approx(19.02)
    assert audit["angular"]["omega"]["status"] == "passed"
    assert np.max(np.linalg.norm(pose["omega_W"], axis=1)) <= 1.1
    no_stretch = pipe_pose_options({"show_figures": False, "omega_max_rad_s": 1.1, "allow_time_stretch": False})
    with pytest.raises(ValueError, match="AngularLimit"):
        pipe_pose_schedule(source_reference, offline, no_stretch)
    minimum = pipe_pose_options({"show_figures": False, "omega_max_rad_s": 1.1, "minimum_cruise_speed_mm_s": 40})
    with pytest.raises(ValueError, match="MinimumFeed"):
        pipe_pose_schedule(source_reference, offline, minimum)


def test_physical_crease_requires_explicit_verified_seam():
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_prepare_model import (
        pipe_pose_prepare_model,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_test_model import (
        pipe_pose_test_model,
    )

    with pytest.raises(ValueError, match="接缝"):
        pipe_pose_prepare_model(pipe_pose_test_model())
    model = pipe_pose_prepare_model(pipe_pose_test_model(), [1])
    assert model["joinReport"][0, 0] == pytest.approx(0)
    assert model["joinReport"][0, 1] > 0.5


def test_axis_configuration_and_source_identity_fail_at_the_public_boundary(source_reference):
    from copy import deepcopy

    from robot_safecontrol_moveit.pipe_assets.pipe_axis_smoothing import (
        pipe_axis_smoothing,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_options import pipe_pose_options
    from robot_safecontrol_moveit.pipe_assets.pipe_surface_correspondence import (
        pipe_surface_correspondence,
    )

    with pytest.raises(ValueError, match="Options"):
        pipe_pose_options({"omega_max_rad_s": -1})
    cfg = pipe_pose_options({"show_figures": False})
    with pytest.raises(ValueError, match="DirectionBranch"):
        pipe_axis_smoothing("build", [0, 1, 2], [[0, 0, 1], [0, 0, -1], [0, 0, -1]], cfg)
    changed = deepcopy(source_reference["correspondence"])
    changed["model_sources"]["pipe_demo_model"] += "\n"
    with pytest.raises(ValueError, match="ModelVersion"):
        pipe_surface_correspondence("model", changed)
