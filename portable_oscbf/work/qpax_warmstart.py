#!/usr/bin/env python3
"""Fixed-shape warm-start adapter for qpax's explicit hard-QP PDIP solver.

The upstream ``qpax.solve_qp`` API recreates its primal-dual state on every
call.  This adapter preserves the same predictor-corrector method, but accepts
the preceding converged state as an input to a surrounding JIT control loop.
It is intentionally limited to finite, hard-inequality QPs used by the JAX
OSCBF controller.
"""

from __future__ import annotations

from typing import NamedTuple

import jax
import jax.numpy as jnp

from qpax.explicit.pdip import (
    LinearSolver,
    QPData,
    QPState,
    _all_finite,
    centering_params,
    factorize_kkt,
    initialize,
    ort_linesearch,
    solve_kkt_rhs,
)


class WarmStartState(NamedTuple):
    """Primal-dual state from the preceding QP solve.

    ``valid`` is set only after a finite converged solve.  ``s`` is rebuilt
    from the current constraint right-hand side before reuse, so newly active
    constraints never inherit a stale positive slack.
    """

    x: jax.Array
    s: jax.Array
    z: jax.Array
    y: jax.Array
    valid: jax.Array


def empty_warm_start_state(n: int, p: int, m: int, *, dtype) -> WarmStartState:
    """Create an invalid state; the first control period cold-starts qpax."""
    return WarmStartState(
        x=jnp.zeros(n, dtype=dtype),
        s=jnp.ones(p, dtype=dtype),
        z=jnp.ones(p, dtype=dtype),
        y=jnp.zeros(m, dtype=dtype),
        valid=jnp.asarray(False),
    )


def _reuse_state(state: WarmStartState, g_matrix, h_vector, floor) -> QPState:
    """Map a prior solution to strictly interior current inequality variables."""
    slack = jnp.maximum(h_vector - g_matrix @ state.x, floor)
    dual = jnp.maximum(state.z, floor)
    return QPState(state.x, slack, dual, state.y)


def solve_qp_warm_started(q_matrix, q_vector, a_matrix, b_vector,
                          g_matrix, h_vector, warm_state: WarmStartState,
                          solver_tol: float = 1e-3, max_iter: int = 30):
    """Solve a finite hard QP using qpax PDIP, reusing a valid prior state.

    Returns ``(x, s, z, y, converged, iterations, next_warm_state)``.  The
    implementation mirrors qpax's public explicit solver for finite matrices;
    its only algorithmic addition is the guarded initial-state selection.
    """
    q_matrix = 0.5 * (q_matrix + q_matrix.T)
    qp = QPData(q_matrix, q_vector, a_matrix, b_vector, g_matrix, h_vector)
    floor = jnp.sqrt(jnp.finfo(q_matrix.dtype).eps)
    cold_state = initialize(qp, LinearSolver.CHOLESKY)
    can_reuse = warm_state.valid & _all_finite(
        warm_state.x, warm_state.s, warm_state.z, warm_state.y)
    initial_state = jax.lax.cond(
        can_reuse,
        lambda _: _reuse_state(warm_state, g_matrix, h_vector, floor),
        lambda _: cold_state,
        operand=None,
    )

    def step(inputs):
        state, converged, iterations, bad_step = inputs
        x, s, z, y = state
        s = jnp.maximum(s, floor)
        z = jnp.maximum(z, floor)

        r1 = q_matrix @ x + q_vector + a_matrix.T @ y + g_matrix.T @ z
        r2 = s * z
        r3 = g_matrix @ x + s - h_vector
        r4 = a_matrix @ x - b_vector
        residual = jnp.concatenate((r1, r2, r3, r4))
        converged = jnp.linalg.norm(residual, ord=jnp.inf) < solver_tol

        p_inv_vec, l_h, l_f = factorize_kkt(
            q_matrix, g_matrix, a_matrix, s, z, LinearSolver.CHOLESKY)
        _, ds_affine, dz_affine, _ = solve_kkt_rhs(
            g_matrix, a_matrix, s, z, p_inv_vec, l_h, l_f,
            -r1, -r2, -r3, -r4, LinearSolver.CHOLESKY)
        sigma, mu = centering_params(s, z, ds_affine, dz_affine)
        r2_corrected = r2 - (sigma * mu - ds_affine * dz_affine)
        dx, ds, dz, dy = solve_kkt_rhs(
            g_matrix, a_matrix, s, z, p_inv_vec, l_h, l_f,
            -r1, -r2_corrected, -r3, -r4, LinearSolver.CHOLESKY)
        alpha = 0.99 * jnp.min(jnp.array([
            ort_linesearch(s, ds), ort_linesearch(z, dz)]))
        finite_step = _all_finite(dx, ds, dz, dy, alpha)
        take = jnp.logical_not(converged)
        next_state = QPState(
            jnp.where(take, x + alpha * dx, x),
            jnp.where(take, s + alpha * ds, s),
            jnp.where(take, z + alpha * dz, z),
            jnp.where(take, y + alpha * dy, y),
        )
        return (next_state, converged, iterations + 1,
                jnp.logical_or(bad_step, jnp.logical_not(finite_step)))

    def condition(inputs):
        _, converged, iterations, bad_step = inputs
        return jnp.logical_and(
            iterations < max_iter,
            jnp.logical_and(jnp.logical_not(converged), jnp.logical_not(bad_step)))

    final_state, converged, iterations, bad_step = jax.lax.while_loop(
        condition, step, (initial_state, jnp.asarray(False), jnp.asarray(0), jnp.asarray(False)))
    result_finite = _all_finite(
        final_state.x, final_state.s, final_state.z, final_state.y)
    accepted = converged & result_finite & jnp.logical_not(bad_step)
    next_warm_state = WarmStartState(
        final_state.x, final_state.s, final_state.z, final_state.y, accepted)
    return (
        final_state.x, final_state.s, final_state.z, final_state.y,
        accepted, iterations, next_warm_state)
