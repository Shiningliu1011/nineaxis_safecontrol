from pathlib import Path

import numpy as np
import pytest

from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat

DATA = Path(__file__).parent / "data/pipe_assets"


def test_saved_integer_plan_reconstructs_original_xyz_motion():
    from robot_safecontrol_moveit.pipe_assets.pipe_feed_profile import pipe_feed_profile

    offline = load_mat(DATA / "nurbs_blocks.mat")
    expected = load_mat(DATA / "realtime_interpolation_results.mat")
    motion = pipe_feed_profile("sample", offline["source_curve"], offline["arc_table"], offline["motion_plan"])
    for field, source, tolerance in (
        ("time", "time_series", 1e-13), ("u", "u_series", 1e-10),
        ("position", "position_series", 2e-8), ("v", "speed_series", 2e-12),
        ("velocity", "velocity_series", 2e-7), ("acceleration", "acceleration_series", 2e-6),
        ("jerk", "jerk_series", 2e-5), ("a", "tangent_acc_cmd_series", 2e-10),
        ("j", "tangent_jerk_cmd_series", 2e-8),
    ):
        np.testing.assert_allclose(motion[field], expected[source], atol=tolerance, rtol=0)
    np.testing.assert_array_equal(motion["boundary_index"], expected["boundary_point_index"])
    np.testing.assert_array_equal(motion["block_index"], expected["block_index"])
    assert motion["v"][0] == motion["v"][-1] == 0
    assert np.all(np.diff(motion["u"]) > 0)


def test_joint_schedule_solves_real_blocks_and_checks_original_physical_limits():
    from robot_safecontrol_moveit.pipe_assets.pipe_joint_schedule import (
        pipe_joint_schedule,
    )

    source = load_mat(DATA / "nurbs_blocks.mat")
    limits = source["limits"]
    plan = pipe_joint_schedule(source["arc_table"], source["seed"], source["raw_boundary_limits"], limits, 1.)
    assert plan["compensation_method"] == "joint_boundary_stage_integer_lp"
    assert plan["shape_ok"]
    assert plan["lp_solver_calls"] > 0
    assert all(item["exitflag"] > 0 for item in plan["candidate_integer_schedules"] if item["feasible"])
    assert all(item["phase"] == "secondary" and item["status"] == 0 for item in plan["candidate_integer_schedules"] if item["feasible"])
    assert np.max(np.abs(plan["length_residual"])) <= 1e-8
    assert np.max(plan["analytic_phase_acceleration"]) <= limits["At"]
    assert np.max(plan["analytic_phase_jerk"]) <= limits["Jt"]
    assert np.all(plan["vs"] <= plan["physical_boundary_limit"][:-1] + 1e-8)
    assert plan["vs"][0] == plan["ve"][-1] == 0
    with pytest.raises(ValueError, match="JointInput"):
        pipe_joint_schedule(source["arc_table"], source["seed"], source["raw_boundary_limits"], limits, 0.)


def test_full_validation_rechecks_dense_geometry_and_saved_motion_limits():
    from robot_safecontrol_moveit.pipe_assets.pipe_feed_profile import pipe_feed_profile

    source = load_mat(DATA / "nurbs_blocks.mat")
    data = pipe_feed_profile("sample", source["source_curve"], source["arc_table"], source["motion_plan"])
    result = pipe_feed_profile("check", source["source_curve"], data, source["motion_plan"], source["limits"])
    assert result["passed"] and result["geometry_ok"]
    assert result["dense_geometry"]["node_side_points"] > 0
    assert result["dense_geometry"]["continuous_extrema"]["smooth_intervals"] == source["validation"]["dense_geometry"]["continuous_extrema"]["smooth_intervals"]
    assert result["max_speed"] == pytest.approx(source["validation"]["max_speed"], abs=1e-7)
    assert result["max_normal_acceleration"] == pytest.approx(source["validation"]["max_normal_acceleration"], abs=1e-4)
    assert result["duration_s"] == pytest.approx(9.51)
    assert result["max_chord_error_mm"] <= source["limits"]["delta"]


def test_build_recomputes_a_fully_checked_schedule_from_source_curve_and_boundaries():
    from robot_safecontrol_moveit.pipe_assets.pipe_feed_profile import pipe_feed_profile

    source = load_mat(DATA / "nurbs_blocks.mat")
    plan, validation = pipe_feed_profile("build", source["source_curve"], source["arc_table"],
                                        {"block_u": source["raw_boundary_u"]}, source["limits"])
    assert validation["passed"]
    assert plan["version"] == 3
    assert plan["lp_solver_calls"] > 0
    assert plan["requires_dense_geometry_validation"]
    assert "boundary_coarsening_audit" in plan
    actual = pipe_feed_profile("sample", source["source_curve"], source["arc_table"], plan)
    assert actual["time"][-1] <= 12
    assert np.linalg.norm(actual["position"][-1] - source["source_curve"]["ctrl_pts"][-1]) < 1e-9
    assert validation["max_normal_acceleration"] <= source["limits"]["An"]
    assert validation["max_normal_jerk"] <= source["limits"]["Jn"]


def test_a2_and_a3_regenerate_from_control_points_and_export_replayable_mat(tmp_path):
    from robot_safecontrol_moveit.pipe_assets.A_2_lixianjisuan import A_2_lixianjisuan
    from robot_safecontrol_moveit.pipe_assets.A_3_shishichabu import A_3_shishichabu

    offline = A_2_lixianjisuan(DATA, tmp_path)
    source = load_mat(DATA / "nurbs_blocks.mat")
    assert offline["validation"]["passed"]
    np.testing.assert_allclose(offline["u_list"], source["u_list"], atol=2e-9, rtol=0)
    np.testing.assert_allclose(offline["raw_boundary_u"], source["raw_boundary_u"], atol=2e-9, rtol=0)
    replay = A_3_shishichabu(tmp_path / "nurbs_blocks.mat", tmp_path)
    saved = load_mat(tmp_path / "realtime_interpolation_results.mat")
    ik = load_mat(tmp_path / "ik_input.mat")["ik_input"]
    assert replay["validation"]["passed"]
    np.testing.assert_array_equal(saved["position_series"], replay["position_series"])
    np.testing.assert_array_equal(ik["velocity_series"], replay["velocity_series"])
    assert np.max(np.abs(np.linalg.norm(replay["velocity_series"], axis=1) - replay["speed_series"])) < 1e-10
    assert replay["speed_series"][0] == replay["speed_series"][-1] == 0


def test_single_block_line_plan_survives_mat_roundtrip_and_rejects_changed_geometry(tmp_path):
    from robot_safecontrol_moveit.pipe_assets.A_2_lixianjisuan import A_2_lixianjisuan
    from robot_safecontrol_moveit.pipe_assets.A_3_shishichabu import A_3_shishichabu

    np.savetxt(tmp_path / "control_points.txt", np.column_stack([np.linspace(0, 100, 4), np.zeros((4, 2))]))
    np.savetxt(tmp_path / "knot_vector.txt", np.r_[np.zeros(4), np.ones(4)][None, :])
    offline = A_2_lixianjisuan(tmp_path, tmp_path)
    assert len(offline["motion_plan"]["na"]) == 1
    replay = A_3_shishichabu(tmp_path / "nurbs_blocks.mat", tmp_path)
    assert replay["validation"]["passed"]
    np.testing.assert_allclose(replay["position_series"][-1], [100, 0, 0], atol=1e-10)
    np.savetxt(tmp_path / "control_points.txt", np.column_stack([np.linspace(0, 101, 4), np.zeros((4, 2))]))
    with pytest.raises(ValueError, match="CurveMismatch"):
        A_3_shishichabu(tmp_path / "nurbs_blocks.mat", tmp_path)
