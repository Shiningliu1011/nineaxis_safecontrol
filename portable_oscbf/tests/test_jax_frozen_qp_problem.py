"""Frozen qpax inputs must be the exact matrices used by the JAX kernel."""

import _path_setup  # noqa: F401

import jax
import numpy as np


def _block(tree):
    return jax.tree_util.tree_map(
        lambda value: value.block_until_ready() if hasattr(value, 'block_until_ready') else value,
        tree)


def test_frozen_qp_core_matches_the_production_candidate_with_rate_slacks():
    from actuator_limits import command_delta_limits, load_actuator_limit_profile
    from work.jax_control_facade import JaxControlLoop

    profile = load_actuator_limit_profile()
    loop = JaxControlLoop(
        dt=0.01,
        temporal_lambda=0.2,
        rate_limit_du_max=command_delta_limits(profile, dt_s=0.01),
        rate_limit_penalty=1.0e3,
    )
    loop.init_cbf()

    # This is a recorded, feasible 100 Hz baseline pose rather than a toy QP.
    q = np.array([
        0.37734028163323363, 0.04138857475176939, -1.002034505433217,
        0.4807032155337039, -2.688021155912466, -0.8455241443682718,
        -0.22142819011531317, -1.171850644172406, 1.2498600708370442,
    ])
    u_nom = np.array([
        0.01452483635720892, 0.0417577559797377, 0.002283273834107314,
        -0.02309474534053294, -0.3467380791391691, 0.03855349278356258,
        0.175663268985398, -0.06844287260695076, 0.2314129208570542,
    ])
    u_previous = np.array([
        0.013689217958735895, 0.046874803423999044, 0.0009160765942872808,
        -0.028357904259296785, -0.3457173806366253, 0.04213794287351649,
        0.17498996769356714, -0.07322513348583401, 0.23003730919915463,
    ])

    problem = _block(loop.freeze_qp_problem(q, u_nom, u_safe_prev=u_previous))
    P, _q, A, _b, G, _h = problem
    assert P.shape == (11, 11)
    assert A.shape == (0, 11)
    assert G.shape[1] == 11
    # M2/M7 row topology: 18 joint limits + 14 self-collision pairs + 10 OBB
    # obstacle rows + 1 singularity CBF, then 18 velocity-box rows and the
    # rate-slack augmentation (9 lower + 9 upper + 2 non-negativity).
    assert G.shape[0] == 18 + 14 + 10 + 1 + 18 + 9 + 9 + 2

    core_result = _block(loop.solve_frozen_qp_problem(problem))
    loop.step(q, u_nom, u_safe_prev=u_previous)

    np.testing.assert_allclose(
        np.asarray(core_result[0])[:9], loop.last_qp_candidate,
        rtol=1.0e-10, atol=1.0e-10)
