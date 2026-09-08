#!/usr/bin/env python3
"""Regression tests for accepting a valid PDIP terminal iterate."""

import numpy as np


def test_terminal_kkt_accepts_feasible_final_iterate_after_solver_limit():
    """A stale upstream convergence flag must not discard a valid safe QP step."""
    import jax.numpy as jnp

    from work.qp_solver_health import terminal_qp_health

    # min 0.5 * (u - 0.2)^2 subject to -1 <= u <= 1.
    # This mimics qpax reaching a valid final Newton iterate immediately after
    # its last pre-update convergence check.
    q_matrix = jnp.array([[1.0]])
    q_vector = jnp.array([-0.2])
    a_matrix = jnp.zeros((0, 1))
    b_vector = jnp.zeros(0)
    g_matrix = jnp.array([[-1.0], [1.0]])
    h_vector = jnp.array([1.0, 1.0])
    solution = jnp.array([0.2])
    slack = h_vector - g_matrix @ solution
    dual = jnp.zeros(2)
    equality_dual = jnp.zeros(0)

    health = terminal_qp_health(
        q_matrix, q_vector, a_matrix, b_vector, g_matrix, h_vector,
        solution, slack, dual, equality_dual, solver_tol=1.0e-3)

    assert bool(health.accepted)
    assert float(health.kkt_residual) < 1.0e-8
    assert float(health.hard_violation) <= 1.0e-8


def test_terminal_kkt_rejects_a_hard_constraint_violation():
    import jax.numpy as jnp

    from work.qp_solver_health import terminal_qp_health

    health = terminal_qp_health(
        jnp.array([[1.0]]), jnp.array([0.0]),
        jnp.zeros((0, 1)), jnp.zeros(0),
        jnp.array([[1.0]]), jnp.array([0.0]),
        jnp.array([0.1]), jnp.array([0.0]),
        jnp.array([0.0]), jnp.zeros(0), solver_tol=1.0e-3)

    assert not bool(health.accepted)
    assert np.isclose(float(health.hard_violation), 0.1)
