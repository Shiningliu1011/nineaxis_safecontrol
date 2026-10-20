from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
from robot_safecontrol_moveit.static_experiment_nodes import _resources
from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")

from work.static_collision import StaticGeometry
from work.static_motion import ActuatorState, certify_motion
from work.static_swept_geometry import transform_derivatives, transform_derivatives_batch
from work.obb_collision_model import OBB_LINK_NAMES


@pytest.fixture(scope="module")
def resources():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    path, geometry, planner = _resources(config)
    return config, geometry, planner


def test_transform_derivatives_match_independent_fk_differences(resources):
    config, geometry, _ = resources
    velocity = np.array([.02, .03, -.04, .02, .01, -.03, .04, -.02, .01])
    acceleration = np.array([.03, -.1, .2, -.1, .1, .15, -.1, .12, -.1])
    state = ActuatorState(config.initial_position, velocity, acceleration)
    derivatives = transform_derivatives(state)
    epsilon = 1e-4
    center = geometry.kinematics.forward_kinematics(state.position)
    plus = geometry.kinematics.forward_kinematics(state.position + epsilon * velocity + epsilon ** 2 * acceleration / 2)
    minus = geometry.kinematics.forward_kinematics(state.position - epsilon * velocity + epsilon ** 2 * acceleration / 2)
    for name in center:
        np.testing.assert_allclose(derivatives[name][0], center[name], atol=1e-12)
        np.testing.assert_allclose(derivatives[name][1], (plus[name] - minus[name]) / (2 * epsilon), atol=2e-9)
        np.testing.assert_allclose(derivatives[name][2], (plus[name] + minus[name] - 2 * center[name]) / epsilon ** 2, atol=3e-7)


def test_projected_continuous_bounds_cover_actual_ruckig_samples(resources):
    config, geometry, planner = resources
    state = ActuatorState(config.initial_position, np.array([.02, .01] + [0.] * 7), np.array([.05, .1] + [0.] * 7))
    motion = planner.propose(state, np.array([-.015, -.02, .03, -.02, .01, -.02, .02, .01, -.01]), period_s=.1)
    for start, end in ((0., .1), (.1, motion.duration_s)):
        midpoint = motion.at_time((start + end) / 2)
        rows = geometry.query(midpoint.position, gradients=False)
        proven = geometry.swept.lower_bounds(rows, midpoint, motion.bounds, (end - start) / 2)
        for value in np.linspace(start, end, 151):
            actual = geometry.query(motion.at_time(float(value)).position, gradients=False)
            margins = np.array([row.distance.distance_m - row.required_m for row in actual])
            assert np.all(margins >= proven - 1e-12)


def test_batched_motion_transforms_match_independent_fk_and_fcl(resources):
    config, geometry, planner = resources
    state = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
    motion = planner.propose(
        state, np.array([.015, -.02, .03, -.02, .01, -.02, .02, .01, -.01]), period_s=.1,
    )
    states = [motion.at_time(float(value)) for value in np.linspace(0., motion.duration_s, 7)]
    derivatives = transform_derivatives_batch(states)
    poses = np.stack([derivatives[name][0] for name in OBB_LINK_NAMES], axis=1)
    batches = geometry.query_transforms(poses)
    epsilon = 1e-4
    for offset, sample in enumerate(states):
        center = geometry.kinematics.forward_kinematics(sample.position)
        plus = geometry.kinematics.forward_kinematics(
            sample.position + epsilon * sample.velocity + epsilon ** 2 * sample.acceleration / 2,
        )
        minus = geometry.kinematics.forward_kinematics(
            sample.position - epsilon * sample.velocity + epsilon ** 2 * sample.acceleration / 2,
        )
        for name in center:
            np.testing.assert_allclose(derivatives[name][0][offset], center[name], atol=1e-12)
            np.testing.assert_allclose(
                derivatives[name][1][offset], (plus[name] - minus[name]) / (2 * epsilon), atol=2e-9,
            )
            np.testing.assert_allclose(
                derivatives[name][2][offset],
                (plus[name] + minus[name] - 2 * center[name]) / epsilon ** 2, atol=3e-7,
            )
        expected = geometry.query(sample.position, gradients=False)
        for actual, reference in zip(batches[offset], expected):
            assert actual.pair == reference.pair
            assert actual.distance.valid == reference.distance.valid
            assert actual.distance.distance_m == pytest.approx(reference.distance.distance_m, abs=1e-12)


def test_batched_interval_proofs_cover_real_motion_samples(resources):
    config, geometry, planner = resources
    initial = ActuatorState(
        config.initial_position, np.array([.01, -.02] + [0.] * 7),
        np.array([.03, .1] + [0.] * 7),
    )
    motion = planner.propose(
        initial, np.array([-.015, .02, .01, -.02, .01, -.02, .02, .01, -.01]), period_s=.1,
    )
    knots = np.linspace(0., motion.duration_s, 10)
    values = motion.values_at_times((knots[:-1] + knots[1:]) / 2)
    transforms = transform_derivatives_batch(values)
    queries = geometry.query_transforms(np.stack([transforms[name][0] for name in OBB_LINK_NAMES], axis=1))
    proven = geometry.swept.lower_bounds_batch(
        queries, motion.bounds, np.diff(knots) / 2, transforms,
    )
    lower, upper = motion.bounds.position_ranges(knots[:-1], knots[1:])
    for index, (start, end) in enumerate(zip(knots[:-1], knots[1:])):
        for time_s in np.linspace(start, end, 13):
            state = motion.at_time(float(time_s))
            assert np.all(state.position >= lower[index] - 1e-14)
            assert np.all(state.position <= upper[index] + 1e-14)
            actual = geometry.query(state.position, gradients=False)
            assert np.all(np.array([
                row.distance.distance_m - row.required_m for row in actual
            ]) >= proven[index] - 1e-12)


def test_internal_turning_point_collision_is_rejected_when_endpoints_are_safe(resources):
    config, original, planner = resources
    state = ActuatorState(config.initial_position, np.array([.02] + [0.] * 8), np.array([.05] + [0.] * 8))
    motion = planner.propose(state, np.array([-.015] + [0.] * 8), period_s=.1)
    peak = state.position.copy()
    peak[0] = motion.bounds.upper[0]
    transform = original.kinematics.forward_kinematics(peak)["Link9"]
    vertices = original.swept.external_vertices["Link9"] @ transform[:3, :3].T + transform[:3, 3]
    obstacle_center = vertices[np.argmax(vertices[:, 2])] + [0., 0., .06 + .0299]
    scene = deepcopy(config.scene())
    scene["obstacles"] = [{"name": "turning_obstacle", "center_m": obstacle_center.tolist(), "radius_m": .06}]
    geometry = StaticGeometry(scene)
    for time_s in (0., motion.duration_s):
        assert all(
            row.distance.valid and row.distance.distance_m >= row.required_m
            for row in geometry.query(motion.at_time(time_s).position)
        )
    certificate = certify_motion(motion, geometry, planner)
    assert not certificate.accepted
    assert certificate.reason == "collision_clearance"


def test_pair_subsets_preserve_exact_geometry_and_validate_indices(resources):
    config, geometry, planner = resources
    rows = geometry.query(config.initial_position)
    selected = np.array([0, 19, 20, 35, 55])
    subset = geometry.query(config.initial_position, pair_indices=selected)
    for actual, index in zip(subset, selected):
        assert actual.to_record() == rows[index].to_record()
    for invalid in ([56], [-1], [1, 1], [.5]):
        with pytest.raises(ValueError):
            geometry.query(config.initial_position, pair_indices=invalid)
    motion = planner.propose(ActuatorState(config.initial_position, np.zeros(9), np.zeros(9)), np.zeros(9), period_s=.1)
    assert certify_motion(motion, geometry, planner).covered_pairs == len(rows)
