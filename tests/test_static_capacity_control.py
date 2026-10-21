from copy import deepcopy
from pathlib import Path

import pytest

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, METHODS
from robot_safecontrol_moveit.static_experiment_engineering_capacity import capacity_common_control
from robot_safecontrol_moveit.static_formal_experiments import experiment_rows


ROOT = Path(__file__).resolve().parents[1]


def test_capacity_common_control_accepts_real_method_overrides_without_mutation():
    source = ROOT / "config/static_experiment_s0.yaml"
    controls = {
        method: ExperimentConfig.load(source, resource_root=ROOT, method=method).manifest()["config"]["control"]
        for method in METHODS
    }
    original = deepcopy(controls)
    expected = capacity_common_control(controls["D"])
    for method, control in controls.items():
        assert (control["solver_mode"], control["recovery"]["mode"]) == METHODS[method]
        assert capacity_common_control(control) == expected
    assert controls == original
    rows = experiment_rows()
    assert len(rows) == len({row["id"] for row in rows}) == 1080
    assert {method: sum(row["method"] == method for row in rows) for method in METHODS} == dict.fromkeys(METHODS, 216)


@pytest.mark.parametrize("path", (
    ("period_s",), ("cbf_alpha",), ("task_gains",), ("task_weights",),
    ("posture_weight",), ("smooth_weight",), ("feed_weight",),
    ("posture_speed_limit",), ("posture_influence_m",), ("posture_joint_margin_fraction",),
    ("recovery", "pause_position_m"), ("recovery", "resume_position_m"),
    ("recovery", "pause_angle_rad"), ("recovery", "resume_angle_rad"),
    ("recovery", "confirmation_s"), ("recovery", "timeout_s"),
    ("recovery", "feed_acceleration_m_s2"),
))
def test_capacity_common_control_keeps_every_other_public_parameter(path):
    control = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method="D",
    ).manifest()["config"]["control"]
    changed = deepcopy(control)
    parent = changed if len(path) == 1 else changed[path[0]]
    value = parent[path[-1]]
    parent[path[-1]] = [item * 2 for item in value] if isinstance(value, list) else value * 2
    assert capacity_common_control(changed) != capacity_common_control(control)


def test_capacity_common_control_allows_declared_feed_factors_and_rejects_unknown_methods():
    source = ROOT / "config/static_experiment_s0.yaml"
    control = ExperimentConfig.load(source, resource_root=ROOT, method="D").manifest()["config"]["control"]
    for factor in (.5, 1., 1.5):
        actual = ExperimentConfig.load(
            source, resource_root=ROOT, method="A", feed_factor=factor,
        ).manifest()["config"]["control"]
        assert capacity_common_control(actual) == capacity_common_control(control)
    invalid = deepcopy(control)
    invalid["solver_mode"] = "unknown"
    with pytest.raises(ValueError, match="A–E"):
        capacity_common_control(invalid)
