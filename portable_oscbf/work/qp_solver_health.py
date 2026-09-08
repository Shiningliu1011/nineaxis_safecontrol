"""JAX-safe terminal health checks for QP solver results.

``qpax`` evaluates its convergence condition before a Newton update.  If the
last permitted update reaches the requested tolerance, the returned iterate
can be valid while its boolean flag still says ``False``.  This module checks
the final primal-dual KKT state directly; it never relaxes an inequality.
"""

from __future__ import annotations

from typing import NamedTuple

import jax.numpy as jnp


class QpTerminalHealth(NamedTuple):
    """Terminal QP diagnostics in solver units.

    ``hard_violation`` is the largest value of ``G @ x - h``.  A positive
    value means at least one original hard inequality is violated.
    """

    kkt_residual: object
    hard_violation: object
    accepted: object


def _max_abs_or_zero(values, *, dtype):
    """Return an infinity norm while supporting zero-row equality systems."""
    if values.size == 0:
        return jnp.asarray(0.0, dtype=dtype)
    return jnp.max(jnp.abs(values))


def _max_or_zero(values, *, dtype):
    """Return a maximum while supporting zero-row equality systems."""
    if values.size == 0:
        return jnp.asarray(0.0, dtype=dtype)
    return jnp.max(values)


def terminal_qp_health(q_matrix, q_vector, a_matrix, b_vector,
                       g_matrix, h_vector, solution, slack, dual,
                       equality_dual, *, solver_tol: float) -> QpTerminalHealth:
    """Validate the final PDIP iterate without weakening any hard constraint.

    Parameters use qpax's convention ``G @ x <= h`` and its non-negative
    slack/dual variables.  The returned ``accepted`` flag is true only when
    finite terminal values satisfy stationarity, complementarity, primal
    residuals, and positivity within ``solver_tol``.
    """
    stationarity = (
        q_matrix @ solution + q_vector + a_matrix.T @ equality_dual
        + g_matrix.T @ dual)
    complementarity = slack * dual
    inequality_residual = g_matrix @ solution + slack - h_vector
    equality_residual = a_matrix @ solution - b_vector
    hard_violation = jnp.maximum(
        _max_or_zero(g_matrix @ solution - h_vector, dtype=solution.dtype),
        jnp.asarray(0.0, dtype=solution.dtype))
    nonnegative_violation = jnp.maximum(
        jnp.maximum(
            _max_or_zero(-slack, dtype=solution.dtype),
            _max_or_zero(-dual, dtype=solution.dtype)),
        jnp.asarray(0.0, dtype=solution.dtype))
    kkt_residual = jnp.max(jnp.stack([
        _max_abs_or_zero(stationarity, dtype=solution.dtype),
        _max_abs_or_zero(complementarity, dtype=solution.dtype),
        _max_abs_or_zero(inequality_residual, dtype=solution.dtype),
        _max_abs_or_zero(equality_residual, dtype=solution.dtype),
        hard_violation,
        nonnegative_violation,
    ]))
    finite = jnp.all(jnp.isfinite(jnp.concatenate([
        jnp.ravel(solution), jnp.ravel(slack), jnp.ravel(dual),
        jnp.ravel(equality_dual),
    ])))
    accepted = finite & (kkt_residual <= solver_tol)
    return QpTerminalHealth(kkt_residual, hard_violation, accepted)
