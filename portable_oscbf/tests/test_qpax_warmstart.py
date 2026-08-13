#!/usr/bin/env python3
"""Warm-started qpax PDIP must preserve the cold-start QP solution."""

import numpy as np
import pytest


def test_warm_started_qpax_matches_cold_solution_and_reuses_converged_state():
    pytest.importorskip("qpax")
    import jax
    import jax.numpy as jnp

    from work.qpax_warmstart import empty_warm_start_state, solve_qp_warm_started

    q_matrix = jnp.array([[2.0, 0.0], [0.0, 4.0]])
    q_vector = jnp.array([-1.0, 0.5])
    a_matrix = jnp.zeros((0, 2))
    b_vector = jnp.zeros(0)
    g_matrix = jnp.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
    h_vector = jnp.ones(4)
    initial = empty_warm_start_state(2, 4, 0, dtype=q_matrix.dtype)

    solve = jax.jit(solve_qp_warm_started)
    cold = solve(q_matrix, q_vector, a_matrix, b_vector, g_matrix, h_vector, initial)
    warm = solve(q_matrix, q_vector, a_matrix, b_vector, g_matrix, h_vector, cold[-1])

    cold_x, _, _, _, cold_ok, cold_iterations, _ = cold
    warm_x, _, _, _, warm_ok, warm_iterations, _ = warm
    np.testing.assert_allclose(warm_x, cold_x, atol=1e-8)
    assert bool(cold_ok)
    assert bool(warm_ok)
    assert int(warm_iterations) < int(cold_iterations)
