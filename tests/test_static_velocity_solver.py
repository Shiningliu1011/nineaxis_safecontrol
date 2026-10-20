from pathlib import Path

import jax
import jax.numpy as jnp
import numpy as np
import pytest
from scipy.optimize import LinearConstraint, minimize

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")
jax.config.update("jax_enable_x64", True)

from work.jax_path_following import as_jax_path_geometry
from work.moving_reference_task import moving_reference_terms_jax
from work.nineaxis_kinematics import NineaxisKinematics
from work.static_collision import StaticGeometry
from work.static_velocity_solver import (
    SolverFailure, VelocityLimits, build_constraints, common_posture_velocity,
    solve_velocity, _solve_problem,
)


@pytest.fixture(scope="module")
def problem():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    q = config.initial_position
    kin = NineaxisKinematics()
    p, r = kin.ee_pose(q)
    terms = moving_reference_terms_jax(
        jnp.asarray(p), jnp.asarray(r), jnp.asarray(kin.compute_full_jacobian(q)),
        as_jax_path_geometry(config.load_path()[0]), jnp.asarray(.0002),
    )
    measurements = StaticGeometry(config.scene()).query(q)
    limits = VelocityLimits(
        kin.joint_limits.q_min, kin.joint_limits.q_max,
        np.array([.3] + [1.5] * 8),
    )
    return q, terms, measurements, limits


@pytest.mark.parametrize("mode", ["separated", "joint"])
def test_actual_qp_matches_independent_slsqp(mode, tmp_path, problem):
    q, terms, measurements, limits = problem
    constraints = build_constraints(q, limits, measurements, period_s=.02, alpha=5.)
    auxiliary = common_posture_velocity(q, limits, terms.jacobian, measurements)
    result = solve_velocity(
        terms, constraints, nominal_feed=.006, previous_velocity=np.zeros(9),
        posture_velocity=auxiliary, mode=mode,
    )
    problem = result.problem
    np.savez(tmp_path / f"{mode}-qp.npz", **problem, solution=result.solution)
    reference = minimize(
        lambda x: (.5 * x @ problem["P"] @ x + problem["c"] @ x,
                   problem["P"] @ x + problem["c"]),
        result.solution, jac=True, method="SLSQP",
        constraints=[LinearConstraint(problem["G"], -np.inf, problem["h"])],
        options={"ftol": 1e-13, "maxiter": 1000},
    )
    assert reference.success
    assert np.max(problem["G"] @ result.solution - problem["h"]) <= 1e-9
    assert np.max(problem["G"] @ reference.x - problem["h"]) <= 1e-9
    actual_cost = .5 * result.solution @ problem["P"] @ result.solution + problem["c"] @ result.solution
    reference_cost = .5 * reference.x @ problem["P"] @ reference.x + problem["c"] @ reference.x
    assert actual_cost == pytest.approx(reference_cost, abs=1e-8)
    assert result.kkt_residual <= 1e-8
    assert 0 <= result.feed_m_s <= .006
    assert len(constraints.labels) == 92
    assert ("Link9", "workpiece") in constraints.labels


def test_common_posture_preserves_five_dimensional_task(problem):
    q, terms, measurements, limits = problem
    velocity = common_posture_velocity(q, limits, terms.jacobian, measurements)
    assert np.linalg.norm(velocity) > 1e-8
    assert np.all(np.abs(velocity) <= np.minimum(limits.max_velocity, .1))
    np.testing.assert_allclose(np.asarray(terms.jacobian) @ velocity, 0, atol=1e-12)


def test_joint_margin_assistance_remains_finite_at_actual_joint_limits(problem):
    q, _, _, limits = problem
    kin = NineaxisKinematics()
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    path = as_jax_path_geometry(config.load_path()[0])
    for bound in (limits.lower, limits.upper):
        position = q.copy()
        position[0] = bound[0]
        p, r = kin.ee_pose(position)
        terms = moving_reference_terms_jax(
            jnp.asarray(p), jnp.asarray(r), jnp.asarray(kin.compute_full_jacobian(position)),
            path, jnp.asarray(0.),
        )
        jacobian = np.asarray(terms.jacobian)
        velocity = common_posture_velocity(position, limits, jacobian, ())
        assert np.all(np.isfinite(velocity))
        assert np.max(np.abs(velocity)) <= .1
        np.testing.assert_allclose(jacobian @ velocity, 0, atol=1e-10)


def test_clearance_reserve_strengthens_actual_constraints_and_preserves_stopping(problem):
    q, terms, measurements, limits = problem
    original = build_constraints(q, limits, measurements, period_s=.1, alpha=5.)
    reserved = build_constraints(
        q, limits, measurements, period_s=.1, alpha=5., clearance_reserve_m=.001,
    )
    np.testing.assert_array_equal(reserved.G, original.G)
    assert np.all(reserved.h <= original.h)
    assert np.any(reserved.h < original.h)
    assert np.all(reserved.G @ np.zeros(9) <= reserved.h)
    result = solve_velocity(
        terms, reserved, nominal_feed=.003, previous_velocity=np.zeros(9),
        posture_velocity=np.zeros(9), mode="sns", max_velocity=limits.max_velocity,
    )
    assert np.max(original.G @ result.velocity - original.h) <= 1e-9
    for invalid in (-.001, float("nan")):
        with pytest.raises(ValueError):
            build_constraints(q, limits, measurements, period_s=.1, alpha=5., clearance_reserve_m=invalid)


def test_outside_joint_position_is_rejected(problem):
    original_q, _, measurements, limits = problem
    q = original_q.copy()
    q[0] = limits.upper[0] + .001
    with pytest.raises(ValueError, match="关节位置"):
        build_constraints(q, limits, measurements, period_s=.02, alpha=5.)


def test_solver_failure_does_not_return_a_command(problem):
    q, terms, measurements, limits = problem
    constraints = build_constraints(q, limits, measurements, period_s=.02, alpha=5.)
    constraints.h[:2] = -10
    constraints.G[:2] = 0
    constraints.G[0, 0], constraints.G[1, 0] = 1, -1
    with pytest.raises(SolverFailure):
        solve_velocity(
            terms, constraints, nominal_feed=.006, previous_velocity=np.zeros(9),
            posture_velocity=np.zeros(9), mode="joint",
        )


def test_recorded_joint_qp_recomputes_motion_at_feed_boundary():
    # 数据来自 S4 完整仿真 controller.sqlite3 的 sequence=5906。
    with np.load(ROOT / "tests/data/static_qp_feed_boundary.npz") as recorded:
        problem = {key: recorded[key] for key in recorded.files}
    solution, health, _ = _solve_problem(problem, joint=True)
    solution = np.asarray(solution)
    assert bool(health.accepted)
    assert 0. <= solution[9] <= 1.
    assert np.max(problem["G"] @ solution - problem["h"]) <= 1e-9
    reference = minimize(
        lambda x: (.5 * x @ problem["P"] @ x + problem["c"] @ x,
                   problem["P"] @ x + problem["c"]),
        solution, jac=True, method="SLSQP",
        constraints=[LinearConstraint(problem["G"], -np.inf, problem["h"])],
        options={"ftol": 1e-13, "maxiter": 1000},
    )
    assert reference.success
    actual_cost = .5 * solution @ problem["P"] @ solution + problem["c"] @ solution
    assert actual_cost == pytest.approx(reference.fun, abs=1e-8)
