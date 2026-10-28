from dataclasses import replace
from pathlib import Path

import numpy as np
from work.nineaxis_model import NineaxisModel
from work.process_limits import ProcessLimits
from work.static_motion import ActuatorState, MotionPlanner, certify_motion

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
from robot_safecontrol_moveit.static_experiment_nodes import _resources

ROOT = Path(__file__).resolve().parents[1]


def test_real_static_resources_consume_the_recorded_per_joint_jerk():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    _, _, planner = _resources(config)
    np.testing.assert_array_equal(planner.max_jerk, config.execution_limits()["max_jerk"])
    assert planner.max_jerk[0] == 62.5
    assert planner.limits_identity is None


def test_real_scene_certificate_rejects_a_different_bound_configuration():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    _, geometry, _ = _resources(config)
    bound = ProcessLimits.load().bind(NineaxisModel.load(root=ROOT), path_hash="a" * 64,
        workpiece_hash="b" * 64, scene_hash="c" * 64, path_length_m=2., phase="tracking",
        evidence_scope="research_simulation")
    planner = MotionPlanner.from_limits(bound, minimum_duration_s=.08)
    state = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
    motion = planner.propose(state, np.zeros(9), period_s=.02)
    changed = MotionPlanner.from_limits(replace(bound, phase="stopping"), minimum_duration_s=.08)
    certificate = certify_motion(motion, geometry, changed)
    assert not certificate.accepted and certificate.reason == "limits_identity"
    assert certificate.checked_intervals == 0
