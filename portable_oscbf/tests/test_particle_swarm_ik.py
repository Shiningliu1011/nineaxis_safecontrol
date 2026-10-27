from pathlib import Path

import _path_setup  # noqa: F401
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import URDF

from work.nineaxis_kinematics import NineaxisKinematics
from work.particle_swarm_ik import solve_particle_swarm
from work.robot_units import joints_mm_deg_to_si, joints_si_to_mm_deg


URDF_PATH = Path(__file__).resolve().parents[2] / "models/ninezzhou/urdf/ninezzhou.urdf"
ROBOT = URDF.from_xml_string(URDF_PATH.read_bytes())
CHAIN = [ROBOT.joint_map[name] for name in ROBOT.get_chain(ROBOT.get_root(), "tool0", links=False)]


def reference_pose(q):
    values = dict(zip((f"J{index}" for index in range(1, 10)), q, strict=True))
    transform = np.eye(4)
    for joint in CHAIN:
        origin = np.eye(4)
        origin[:3, :3] = Rotation.from_euler("xyz", joint.origin.rpy).as_matrix()
        origin[:3, 3] = joint.origin.xyz
        motion = np.eye(4)
        if joint.type == "prismatic":
            motion[:3, 3] = np.asarray(joint.axis) * values[joint.name]
        elif joint.type == "revolute":
            motion[:3, :3] = Rotation.from_rotvec(np.asarray(joint.axis) * values[joint.name]).as_matrix()
        else:
            assert joint.type == "fixed"
        transform = transform @ origin @ motion
    return transform


@pytest.fixture(scope="module")
def kinematics():
    return NineaxisKinematics()


def assert_accepted(kinematics, result, target, with_rotation=True):
    assert result.success
    assert np.all(result.q >= kinematics.joint_limits.q_min)
    assert np.all(result.q <= kinematics.joint_limits.q_max)
    actual = reference_pose(result.q)
    position_error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
    rotation_error = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()
    assert position_error <= 1e-4
    np.testing.assert_allclose(result.position_error_m, position_error, atol=2e-14, rtol=0)
    if with_rotation:
        assert rotation_error <= 1e-3
        np.testing.assert_allclose(result.rotation_error_rad, rotation_error, atol=2e-14, rtol=0)
    assert np.all(np.diff(result.convergence_curve) <= 0)


def test_batch_poe_matches_independent_urdf_at_limits_and_random_configurations(kinematics):
    lower, upper = kinematics.joint_limits.q_min, kinematics.joint_limits.q_max
    middle = (lower + upper) / 2
    samples = [np.zeros(9), lower, upper, middle]
    for index in range(9):
        for limit in (lower, upper):
            q = middle.copy()
            q[index] = limit[index]
            samples.append(q)
    samples.extend(np.random.default_rng(23145).uniform(lower, upper, (128, 9)))
    expected = np.stack([reference_pose(q) for q in samples])
    actual = kinematics._compute_ee_poses(np.asarray(samples))
    np.testing.assert_allclose(actual, expected, atol=2e-13, rtol=0)
    for q, pose in zip(samples, actual, strict=True):
        np.testing.assert_allclose(kinematics._compute_ee_pose(q), pose, atol=2e-13, rtol=0)


@pytest.mark.parametrize("with_rotation", [False, True])
@pytest.mark.parametrize("sample_index", [0, 1, 2, 3, 4, 5, 6])
def test_pso_reaches_urdf_targets_from_zero(kinematics, sample_index, with_rotation):
    lower, upper = kinematics.joint_limits.q_min, kinematics.joint_limits.q_max
    samples = [lower, upper, (lower + upper) / 2]
    samples.extend(np.random.default_rng(8102).uniform(lower, upper, (1, 9)))
    width = upper - lower
    configurations = np.random.default_rng(20261027).uniform(lower + 0.02 * width, upper - 0.02 * width, (24, 9))
    samples.extend(configurations[[1, 13, 20]])
    target = reference_pose(samples[sample_index])
    result = kinematics.solve_ik(target[:3, 3], target[:3, :3] if with_rotation else None)
    assert_accepted(kinematics, result, target, with_rotation)


def test_every_evaluated_particle_respects_joint_limits(kinematics):
    lower, upper = kinematics.joint_limits.q_min, kinematics.joint_limits.q_max
    q_target = (lower + upper) / 2
    target = reference_pose(q_target)
    evaluated = 0

    def observed_forward(q):
        nonlocal evaluated
        assert np.all(np.isfinite(q))
        assert np.all(q >= lower)
        assert np.all(q <= upper)
        evaluated += len(q)
        return kinematics._compute_ee_poses(q)

    result = solve_particle_swarm(
        observed_forward, target[:3, 3], target[:3, :3], np.zeros(9), lower, upper,
    )
    assert_accepted(kinematics, result, target)
    assert evaluated == result.evaluations


def test_insufficient_budget_reports_failure_for_reachable_target(kinematics):
    target = reference_pose(np.array([0.12, 0.3, -0.2, 0.4, -0.1, 0.2, -0.3, 0.4, -0.5]))
    kwargs = dict(max_iter=1, swarm_size=8, tol_pos=1e-12, tol_rot=1e-12)
    result = kinematics.solve_ik(target[:3, 3], target[:3, :3], **kwargs)
    assert not result.success
    assert result.iterations == 1
    assert np.all(result.q >= kinematics.joint_limits.q_min)
    assert np.all(result.q <= kinematics.joint_limits.q_max)
    assert kinematics.ik(target[:3, 3], target[:3, :3], **kwargs) is None


def test_position_and_orientation_acceptance_are_independent(kinematics):
    target = reference_pose(np.array([0.12, 0.3, -0.2, 0.4, -0.1, 0.2, -0.3, 0.4, -0.5]))
    zero = reference_pose(np.zeros(9))
    tol_pos = np.linalg.norm(zero[:3, 3] - target[:3, 3]) + 1e-4
    accepted = kinematics.solve_ik(target[:3, 3], target[:3, :3], tol_pos=tol_pos, tol_rot=np.pi)
    assert accepted.success and accepted.iterations == 0
    rejected = kinematics.solve_ik(
        target[:3, 3], target[:3, :3], tol_pos=tol_pos, tol_rot=1e-12, max_iter=1, swarm_size=8,
    )
    assert not rejected.success


def test_seed_is_reproducible_and_does_not_change_global_random_state(kinematics):
    target = reference_pose((kinematics.joint_limits.q_min + kinematics.joint_limits.q_max) / 2)
    before = np.random.get_state()
    first = kinematics.solve_ik(target[:3, 3], target[:3, :3], max_iter=10, swarm_size=16, seed=6723)
    second = kinematics.solve_ik(target[:3, 3], target[:3, :3], max_iter=10, swarm_size=16, seed=6723)
    after = np.random.get_state()
    np.testing.assert_array_equal(first.q, second.q)
    assert first.convergence_curve == second.convergence_curve
    assert first.evaluations == second.evaluations
    for left, right in zip(before, after, strict=True):
        np.testing.assert_equal(left, right)


def test_mm_deg_boundaries_and_round_trip(kinematics):
    q = np.array([250.0, 30.0, -20.0, 40.0, -10.0, 20.0, -30.0, 40.0, -50.0])
    original = q.copy()
    si = joints_mm_deg_to_si(q)
    assert si[0] == 0.25
    assert si[1] == pytest.approx(np.pi / 6)
    np.testing.assert_array_equal(q, original)
    np.testing.assert_allclose(joints_si_to_mm_deg(si), q, atol=2e-14, rtol=0)
    np.testing.assert_allclose(joints_mm_deg_to_si(np.stack([q, q])), np.stack([si, si]), atol=0, rtol=0)
    expected = reference_pose(si)
    position_mm, rotation = kinematics.ee_pose_mm_deg(q)
    np.testing.assert_allclose(position_mm, expected[:3, 3] * 1000, atol=2e-10, rtol=0)
    np.testing.assert_allclose(rotation, expected[:3, :3], atol=2e-13, rtol=0)
    result = kinematics.ik_mm_deg(position_mm, rotation, q_init_mm_deg=q)
    np.testing.assert_allclose(result, q, atol=2e-14, rtol=0)


def test_mm_deg_solver_from_different_initial_configuration(kinematics):
    q_mm_deg = np.array([250.0, 30.0, -20.0, 40.0, -10.0, 20.0, -30.0, 40.0, -50.0])
    target = reference_pose(joints_mm_deg_to_si(q_mm_deg))
    initial = q_mm_deg.copy()
    initial[0] -= 10.0
    solution = kinematics.ik_mm_deg(target[:3, 3] * 1000.0, target[:3, :3], q_init_mm_deg=initial)
    assert solution is not None
    actual = reference_pose(joints_mm_deg_to_si(solution))
    assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) * 1000.0 <= 0.1
    assert np.rad2deg(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()) <= np.rad2deg(1e-3)


@pytest.mark.parametrize("kwargs", [
    {"target_pos": [np.nan, 0, 0]},
    {"target_pos": [0, 0]},
    {"target_R": np.zeros((3, 3))},
    {"target_R": np.diag([1, 1, -1])},
    {"q_init": [-0.001, 0, 0, 0, 0, 0, 0, 0, 0]},
    {"q_init": np.zeros(8)},
    {"tol_pos": 0},
    {"tol_rot": np.nan},
    {"max_iter": 0},
    {"swarm_size": 1},
    {"seed": -1},
])
def test_invalid_inputs_fail_before_search(kinematics, kwargs):
    arguments = {"target_pos": kinematics.ee_position(np.zeros(9)), **kwargs}
    with pytest.raises(ValueError):
        kinematics.solve_ik(**arguments)


@pytest.mark.parametrize("values", [np.zeros(8), np.full(9, np.inf), 1.0])
def test_invalid_project_units_rejected(values):
    with pytest.raises(ValueError):
        joints_mm_deg_to_si(values)
