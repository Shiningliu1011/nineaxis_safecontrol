from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder, read_stream
from robot_safecontrol_moveit.static_experiment_evaluation import ordered_projection
from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/static_experiment_s0.yaml"


def test_s0_uses_declared_geometry_and_complete_path():
    config = ExperimentConfig.load(CONFIG, resource_root=ROOT)
    geometry, cylinder = config.load_path()
    relative = geometry.positions_m - cylinder["center_m"]
    radial = relative - np.outer(relative @ cylinder["axis"], cylinder["axis"])
    np.testing.assert_allclose(np.linalg.norm(radial, axis=1), 0.2706, atol=1e-12)
    assert config.mode == "sim"
    assert config.workpiece_radius_m == 0.2206
    assert config.path_radius_m - config.workpiece_radius_m == pytest.approx(0.05)
    assert geometry.total_length_m > 1.8
    assert config.initial_position.shape == (9,)
    assert config.plant_parameters()["publish_frequency_hz"] == 500.0
    assert config.controller_parameters()["path_radius_m"] == 0.2706
    assert config.identity == ExperimentConfig.load(CONFIG, resource_root=ROOT).identity


def test_explicit_method_options_are_preserved_in_identity_and_manifest():
    original = ExperimentConfig.load(CONFIG, resource_root=ROOT)
    joint = ExperimentConfig.load(CONFIG, resource_root=ROOT, solver_mode="joint", recovery_mode="improved")
    assert joint.values["control"]["solver_mode"] == "joint"
    assert joint.values["control"]["recovery"]["mode"] == "improved"
    assert joint.identity != original.identity
    assert joint.manifest()["overrides"] == {"solver_mode": "joint", "recovery_mode": "improved"}
    with pytest.raises(ValueError):
        ExperimentConfig.load(CONFIG, resource_root=ROOT, solver_mode="unknown")


def test_methods_period_and_feed_are_explicit_and_conflicting_options_are_rejected():
    config = ExperimentConfig.load(CONFIG, resource_root=ROOT, method="E", period_s=.04, feed_factor=1.5)
    assert config.values["control"]["solver_mode"] == "sns"
    assert config.values["control"]["recovery"]["mode"] == "improved"
    assert config.values["control"]["period_s"] == .04
    baseline = ExperimentConfig.load(CONFIG, resource_root=ROOT)
    assert config.values["control"]["nominal_feed_multiplier"] == pytest.approx(
        baseline.values["control"]["nominal_feed_multiplier"] * 1.5,
    )
    with pytest.raises(ValueError):
        ExperimentConfig.load(CONFIG, resource_root=ROOT, method="E", recovery_mode="basic")
    with pytest.raises(ValueError):
        ExperimentConfig.load(CONFIG, resource_root=ROOT, joint_travel_scale=.4)


def test_s3_available_travel_is_centered_on_the_actual_initial_configuration(tmp_path):
    values = yaml.safe_load((ROOT / "config/static_experiment_s1.yaml").read_text())
    values["scenario"] = "S3"
    path = tmp_path / "s3.yaml"
    path.write_text(yaml.safe_dump(values))
    full = ExperimentConfig.load(path, resource_root=ROOT).execution_limits()
    for scale in (.7, .4):
        config = ExperimentConfig.load(path, resource_root=ROOT, joint_travel_scale=scale)
        reduced = config.execution_limits()
        for name in ("lower", "upper"):
            distance = np.asarray(reduced[name]) - config.initial_position
            expected = np.asarray(full[name]) - config.initial_position
            expected[[0, 3]] *= scale
            np.testing.assert_allclose(distance, expected, atol=1e-15)
        np.testing.assert_array_equal(reduced["max_velocity"], full["max_velocity"])


@pytest.mark.parametrize("change", [
    {"mode": "live"},
    {"path_radius_m": 0.0},
    {"workpiece_radius_m": 0.2706},
    {"initial_position": [0.0] * 8},
    {"unknown_field": True},
    {"obstacles": [{"name": "one", "center_m": [0, 0, 0], "radius_m": .06}]},
    {"scenario": "S1", "obstacles": [{"name": "table", "center_m": [0, 0, 0], "radius_m": .06}]},
    {"scenario": "S1", "obstacles": [{"name": "Link5", "center_m": [0, 0, 0], "radius_m": .06}]},
])
def test_invalid_experiment_is_rejected_before_start(tmp_path, change):
    values = yaml.safe_load(CONFIG.read_text())
    values.update(change)
    path = tmp_path / "experiment.yaml"
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError):
        ExperimentConfig.load(path, resource_root=ROOT)


def test_recording_is_readable_before_writer_shutdown(tmp_path):
    path = tmp_path / "feedback.sqlite3"
    with RunRecorder(path) as recorder:
        recorder.append("feedback", monotonic_s=1.0, simulation_s=0.002,
                        ros_time_ns=100, values={"q": [0.1] * 9})
        sample = read_stream(path, "feedback")[0]
        assert sample["simulation_s"] == 0.002
        assert sample["ros_time_ns"] == 100
        assert sample["values"]["q"] == [0.1] * 9
        with pytest.raises(ValueError, match="monotonic"):
            recorder.append("feedback", monotonic_s=0.9, simulation_s=0.004,
                            ros_time_ns=101, values={"q": [0.2] * 9})
        with pytest.raises(ValueError):
            recorder.append("feedback", monotonic_s=2.0, simulation_s=0.004,
                            ros_time_ns=102, values={"q": [float("nan")] * 9})
    assert len(read_stream(path, "feedback")) == 1


def test_control_reserve_override_is_recorded_and_keeps_required_clearances():
    original = ExperimentConfig.load(CONFIG, resource_root=ROOT)
    reserved = ExperimentConfig.load(CONFIG, resource_root=ROOT, clearance_reserve_m=.0001)
    assert reserved.identity != original.identity
    assert reserved.values["control"]["clearance_reserve_m"] == .0001
    assert reserved.overrides["clearance_reserve_m"] == .0001
    assert reserved.values["external_clearance_m"] == .03
    assert reserved.values["self_clearance_m"] == .01


def test_ordered_projection_follows_both_passes_through_intersection():
    bootstrap_portable(ROOT / "portable_oscbf")
    from work.path_following import PathGeometry

    vertices = np.array([
        [-0.02, -0.02, 0], [0.02, 0.02, 0],
        [0.02, -0.02, 0], [-0.02, 0.02, 0],
    ])
    samples = np.concatenate([
        np.linspace(start, end, 101)[:-1]
        for start, end in zip(vertices[:-1], vertices[1:])
    ] + [vertices[-1:]])
    geometry = PathGeometry.from_samples(
        vertices, np.repeat(np.eye(3)[None], 4, axis=0),
        np.ones(4), np.arange(4, dtype=float),
    )
    progress, distances = ordered_projection(geometry, samples)
    expected = np.r_[0, np.cumsum(np.linalg.norm(np.diff(samples, axis=0), axis=1))]
    np.testing.assert_allclose(progress, expected, atol=1e-12)
    np.testing.assert_allclose(distances, 0.0, atol=1e-12)


@pytest.mark.parametrize("scenario", ["S0", "S1"])
def test_mujoco_scene_contains_configured_workpiece_and_table(tmp_path, scenario):
    import mujoco
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from robot_safecontrol_moveit.mujoco_viewer_with_cylinder import MuJoCoJointStateViewer

    context = Context()
    rclpy.init(context=context, args=[], domain_id=199)
    viewer = None
    config_path = CONFIG
    if scenario == "S1":
        values = yaml.safe_load(CONFIG.read_text())
        values.update(scenario="S1", obstacles=[
            {"name": "test_obstacle", "center_m": [.5, .4, 1.2], "radius_m": .06},
        ])
        config_path = tmp_path / "scene.yaml"
        config_path.write_text(yaml.safe_dump(values))
    try:
        viewer = MuJoCoJointStateViewer(context=context, parameter_overrides=[
            Parameter("static_experiment_config", value=str(config_path)),
            Parameter("static_experiment_resource_root", value=str(ROOT)),
        ])
        index = mujoco.mj_name2id(viewer.model, mujoco.mjtObj.mjOBJ_GEOM, "experiment_workpiece")
        assert index >= 0
        assert viewer.model.geom_size[index, 0] == pytest.approx(0.2206)
        assert mujoco.mj_name2id(viewer.model, mujoco.mjtObj.mjOBJ_GEOM, "experiment_table") >= 0
        if scenario == "S1":
            index = mujoco.mj_name2id(viewer.model, mujoco.mjtObj.mjOBJ_GEOM, "experiment_test_obstacle")
            assert index >= 0
            assert viewer.model.geom_size[index, 0] == pytest.approx(.06)
            box_index = mujoco.mj_name2id(
                viewer.model, mujoco.mjtObj.mjOBJ_GEOM, "experiment_test_obstacle_obb",
            )
            assert box_index >= 0
            np.testing.assert_allclose(viewer.model.geom_size[box_index], [.06] * 3, atol=1e-12)
        config = ExperimentConfig.load(CONFIG, resource_root=ROOT)
        np.testing.assert_allclose(viewer.data.qpos, config.initial_position, atol=1e-12)
    finally:
        if viewer is not None:
            viewer.destroy_node()
        context.shutdown()
