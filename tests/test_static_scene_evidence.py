import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
from robot_safecontrol_moveit.static_experiment_nodes import _resources
from robot_safecontrol_moveit.static_scene_evidence import (
    blocked_path_evidence, initial_configurations, project_fixed_pose,
)
from robot_safecontrol_moveit.static_scenario_evidence import verify_blockage
from robot_safecontrol_moveit.static_reconfiguration_evidence import _position_motion, certify_fixed_pose


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def resources():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    return config, _resources(config)


def test_three_initial_configurations_hold_the_same_five_dimensional_task(resources):
    config, (_, geometry, planner) = resources
    positions = np.asarray(initial_configurations(config))
    assert positions.shape == (3, 9)
    original_position, original_rotation = geometry.kinematics.ee_pose(positions[0])
    for index, q in enumerate(positions):
        assert np.all(q >= planner.lower) and np.all(q <= planner.upper)
        position, rotation = geometry.kinematics.ee_pose(q)
        np.testing.assert_allclose(position, original_position, atol=1e-7)
        np.testing.assert_allclose(rotation[:, 0], original_rotation[:, 0], atol=1e-7)
        assert all(row.distance.valid and row.distance.distance_m >= row.required_m for row in geometry.query(q))
        for other in positions[index + 1:]:
            assert np.linalg.norm(q - other) > .02


def test_s5_blockage_is_recomputed_from_actual_tool_geometry_and_fixed_obstacle(resources, tmp_path):
    config, _ = resources
    evidence = blocked_path_evidence(config)
    values = config.manifest()["config"]
    values["scenario"], values["obstacles"] = "S5", [evidence["obstacle"]]
    configuration = tmp_path / "s5.yaml"
    configuration.write_text(yaml.safe_dump(values))
    fixed = ExperimentConfig.load(configuration, resource_root=ROOT)
    path = tmp_path / "blockage.json"
    path.write_text(json.dumps(evidence))
    result = verify_blockage(fixed, path)
    assert result["status"] == "proven"
    assert result["maximum_distance_bound_m"] < .03
    assert result["blocked_interval_m"][0] < result["blocked_interval_m"][1]
    assert result["position_tolerance_m"] == .002
    evidence["tool_axis_offset_m"] = 1.
    path.write_text(json.dumps(evidence))
    with pytest.raises(ValueError, match="Link9"):
        verify_blockage(fixed, path)


def test_fixed_pose_certificate_checks_continuous_ruckig_motion(resources):
    config, (_, geometry, planner) = resources
    start = config.initial_position
    position, rotation = geometry.kinematics.ee_pose(start)
    transforms, spatial = geometry.kinematics.forward_kinematics_and_jacobian(start)
    endpoint = transforms["ee_link"]
    jacobian = np.vstack([
        spatial[3:] + np.cross(spatial[:3].T, endpoint[:3, 3]).T,
        rotation[:, 1:].T @ spatial[:3],
    ])
    direction = np.linalg.svd(jacobian, full_matrices=True)[2][-1]
    goal = project_fixed_pose(
        geometry, start, start + .0005 * direction, planner.lower, planner.upper, position, rotation,
    )
    motion = _position_motion(planner, start, goal)
    proof = certify_fixed_pose(motion, geometry, planner, position, rotation[:, 0])
    assert proof["motion"]["covered_pairs"] == 56
    assert proof["maximum_position_bound_m"] <= 5e-5
    assert proof["maximum_angle_bound_rad"] <= 1e-4
    changed = start.copy()
    changed[0] += .001
    invalid = _position_motion(planner, start, changed)
    with pytest.raises(ValueError, match="固定位置"):
        certify_fixed_pose(invalid, geometry, planner, position, rotation[:, 0])
