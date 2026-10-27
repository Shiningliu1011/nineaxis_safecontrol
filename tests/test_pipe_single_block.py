import numpy as np
import pytest

from robot_safecontrol_moveit.pipe_assets.A_2_lixianjisuan import A_2_lixianjisuan
from robot_safecontrol_moveit.pipe_assets.A_3_shishichabu import A_3_shishichabu
from robot_safecontrol_moveit.pipe_assets.fit_pipe_path_nurbs import fit_pipe_path_nurbs
from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat, save_mat
from robot_safecontrol_moveit.pipe_assets.path_io import export_path
from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
from robot_safecontrol_moveit.pipe_assets.pipe_pose_time_law import pipe_pose_time_law
from robot_safecontrol_moveit.pipe_assets.pipe_prepare_model import pipe_prepare_model
from robot_safecontrol_moveit.pipe_assets.pipe_reference_path import pipe_reference_path


@pytest.fixture(scope="module")
def single_block(tmp_path_factory):
    output = tmp_path_factory.mktemp("single-block")
    model = pipe_prepare_model(pipe_demo_model())
    trajectory = pipe_reference_path(model, [[10, 0], [50, 0], [100, 0]], 5, 5)
    settings = {"OffsetMM": 5.0, "SampleSpacingMM": 5.0, "ThetaRangeDeg": [-180, 540]}
    source = export_path(model, trajectory, settings, output / "pipe_path")[0]
    fit_pipe_path_nurbs(source, output / "fit", False)
    generated = A_2_lixianjisuan(output / "fit", output / "plan")
    assert len(generated["motion_plan"]["na"]) == 1
    offline = load_mat(output / "plan/nurbs_blocks.mat")
    assert np.ndim(offline["motion_plan"]["na"]) == 0
    motion = A_3_shishichabu(output / "plan/nurbs_blocks.mat", output / "motion")
    assert motion["validation"]["passed"]
    return {"output": output, "source": source, "offline": offline, "motion": motion}


def test_single_block_mat_time_queries_and_stretch_keep_the_original_motion(single_block):
    plan = single_block["offline"]["motion_plan"]
    duration = single_block["motion"]["time_series"][-1]
    time = np.asarray([0, 0.37 * duration, duration])
    state = pipe_pose_time_law("eval", plan, time)
    np.testing.assert_allclose(state["s"][[0, -1]], [0, 90], atol=1e-9, rtol=0)
    recovered = pipe_pose_time_law("time_at_arc", plan, state["s"])
    np.testing.assert_allclose(recovered, time, atol=1e-9, rtol=0)
    boundaries = pipe_pose_time_law("phase_times", plan)
    assert boundaries[0] == 0
    assert boundaries[-1] == pytest.approx(duration)
    stretched = pipe_pose_time_law("stretch", plan, 2)
    assert np.shape(stretched["na"]) == (1,)
    path = single_block["output"] / "stretched.mat"
    save_mat(path, {"motion_plan": stretched})
    reloaded = load_mat(path)["motion_plan"]
    slowed = pipe_pose_time_law("eval", reloaded, 2 * time)
    np.testing.assert_allclose(slowed["s"], state["s"], atol=1e-9, rtol=0)
    np.testing.assert_allclose(slowed["v"], state["v"] / 2, atol=1e-9, rtol=0)
    np.testing.assert_allclose(slowed["a"], state["a"] / 4, atol=1e-9, rtol=0)


def test_single_block_path_runs_a2_a3_a4_and_pose_mat_replay(single_block):
    from robot_safecontrol_moveit.pipe_assets.A_4_zitai_gouzao import A_4_zitai_gouzao
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_angular_jerk import (
        pipe_pose_angular_jerk,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_export import pipe_pose_export
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_reference import (
        pipe_pose_reference,
    )

    output = single_block["output"]
    result = A_4_zitai_gouzao({"show_figures": False}, single_block["source"], output / "pose", output / "fit", output / "plan/nurbs_blocks.mat")
    assert result["validation"]["passed"]
    assert result["ik_input"]["num_blocks"] == 1
    np.testing.assert_allclose(result["ik_input"]["position_series"], single_block["motion"]["position_series"], atol=1e-9, rtol=0)
    reference = pipe_pose_reference("load", output / "pose/pose_reference.mat")
    query = pipe_pose_reference("eval_progress", reference, [0, 0.5, 1])
    np.testing.assert_allclose(query["tool_axis_ref_W"], np.tile([-1, 0, 0], (3, 1)), atol=1e-10, rtol=0)
    plan = single_block["offline"]["motion_plan"]
    sample = pipe_pose_angular_jerk("sample", reference, plan, result["data"]["time"])
    np.testing.assert_allclose(sample["angular_jerk_W"], result["pose"]["angular_jerk_W"], atol=1e-9, rtol=0)
    replayed = pipe_pose_export(output / "exported-again", reference, plan, result["data"], result["pose"], result["validation"])
    assert replayed["num_blocks"] == 1
    saved = load_mat(output / "exported-again/ik_input.mat")["ik_input"]
    np.testing.assert_array_equal(saved["angular_jerk_W"], replayed["angular_jerk_W"])


@pytest.mark.parametrize("frame_step_mm", [2, 8])
def test_short_real_reference_restores_axis_and_frame_intervals_after_mat_roundtrip(tmp_path, frame_step_mm):
    from robot_safecontrol_moveit.pipe_assets.pipe_axis_smoothing import (
        pipe_axis_smoothing,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_nurbs_geometry import (
        pipe_nurbs_geometry,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_orientation_reference import (
        pipe_orientation_reference,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_options import pipe_pose_options
    from robot_safecontrol_moveit.pipe_assets.pipe_pose_reference import (
        pipe_pose_reference,
    )
    from robot_safecontrol_moveit.pipe_assets.pipe_surface_correspondence import (
        pipe_surface_correspondence,
    )

    model = pipe_prepare_model(pipe_demo_model())
    trajectory = pipe_reference_path(model, [[10, 0], [14, 0]], 5, 1)
    settings = {"OffsetMM": 5.0, "SampleSpacingMM": 1.0, "ThetaRangeDeg": [-180, 540]}
    source = export_path(model, trajectory, settings, tmp_path / "pipe_path_short")[0]
    fit_pipe_path_nurbs(source, tmp_path / "fit", False)
    geometry = pipe_nurbs_geometry("load", tmp_path / "fit")
    cfg = pipe_pose_options({"show_figures": False, "frame_step_mm": frame_step_mm})
    pairs = pipe_surface_correspondence("pairs_csv", tmp_path / "fit/curve_data.csv")
    correspondence = pipe_surface_correspondence("build", geometry, source, pairs, cfg)
    reference = pipe_pose_reference("build", geometry, correspondence, cfg)
    assert reference["axis"]["bezier"].shape == (6, 3, 1)
    path = tmp_path / "reference.mat"
    save_mat(path, {"reference": reference})
    stored = load_mat(path)["reference"]
    assert stored["axis"]["bezier"].shape == (6, 3)
    loaded = pipe_pose_reference("load", path)
    query = pipe_pose_reference("eval_progress", loaded, [0, 0.5, 1])
    np.testing.assert_allclose(query["position_mm"], [[40, 0, 10], [40, 0, 12], [40, 0, 14]], atol=1e-10, rtol=0)
    assert loaded["axis"]["bezier"].shape == (6, 3, 1)
    assert loaded["orientation"]["bezier"].shape == reference["orientation"]["bezier"].shape
    axis = pipe_axis_smoothing("eval", stored["axis"], [0, 2, 4])
    np.testing.assert_allclose(axis["tool_axis_ref_W"], np.tile([-1, 0, 0], (3, 1)), atol=1e-10, rtol=0)
    controls = pipe_axis_smoothing("controls", stored["axis"], 0, 4)
    np.testing.assert_allclose(controls, reference["axis"]["bezier"][:, :, 0], atol=1e-12, rtol=0)
    frame = pipe_orientation_reference("eval", stored["orientation"], [0, 2, 4])
    np.testing.assert_allclose(frame["R_WT"], query["R_WT"], atol=1e-12, rtol=0)
