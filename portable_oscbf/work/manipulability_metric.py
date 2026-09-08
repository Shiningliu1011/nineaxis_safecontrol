#!/usr/bin/env python3
"""Regularised log-determinant manipulability metric (M8).

``phi = 0.5 * log det(J_s J_s^T + epsilon I)`` with the task-scaled Jacobian
``J_s = S_x @ J``, ``S_x = diag(I_3, l_c * I_3)``.  The gradient with respect
to ``q`` is obtained with JAX automatic differentiation (autodiff); the M8
test cross-checks it against central finite differences.
"""

from __future__ import annotations

import jax.numpy as jnp


def scaled_task_jacobian(jacobian: jnp.ndarray,
                         characteristic_length: float = 0.4) -> jnp.ndarray:
    """Return the task-scaled 6-DOF Jacobian ``J_s = S_x @ J``."""

    scale = jnp.diag(jnp.array(
        [1.0, 1.0, 1.0, characteristic_length,
         characteristic_length, characteristic_length]))
    return scale @ jacobian


def regularized_logdet_manipulability(
        jacobian: jnp.ndarray,
        characteristic_length: float = 0.4,
        epsilon: float = 1e-6) -> jnp.ndarray:
    """Compute phi = 0.5 * log det(J_s J_s^T + epsilon I)."""

    jacobian_scaled = scaled_task_jacobian(jacobian, characteristic_length)
    gram = jacobian_scaled @ jacobian_scaled.T
    gram = gram + epsilon * jnp.eye(gram.shape[0])
    return 0.5 * jnp.log(jnp.linalg.det(gram))
