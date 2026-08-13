#!/usr/bin/env python3
"""Null-space policies (M8): pluggable online self-motion generators.

``NullspacePolicy`` is the interface; ``ManipulabilityGradientPolicy``
implements the online manipulability gradient from the porting guide §5.2:

    qdot_N = s_v * k_m * rho * N * W_q^-1 * N^T * g_m
    g_m = grad_q phi,  phi = 0.5 log det(J_s J_s^T + eps I)
    s_v = min(1, v_N,max / ||k_m * rho * d_m||)   (global scaling, never
                                                   per-joint clipping)
    W_q = diag(1 / dq_max^2)

Activation (``rho``) is disabled in v1 (``activation.enabled=false``), so
``rho = 1``.  The optional low-pass filter (``filter_alpha``) is applied by
the caller through ``prev_null_velocity``; when None the filter is off.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import jax
import jax.numpy as jnp

from work.manipulability_metric import regularized_logdet_manipulability


class NullspacePolicy(ABC):
    """Interface for a pure-JAX null-space velocity generator."""

    @abstractmethod
    def nullspace_velocity(self, q, jacobian, jacobian_hash, null_projector,
                           joint_max_velocities, prev_null_velocity=None):
        """Return the null-space joint velocity command ``qdot_N`` (9,)."""


class ManipulabilityGradientPolicy(NullspacePolicy):
    """Online manipulability-gradient null-space policy (M8)."""

    def __init__(self, robot=None, *, gain: float = 0.15,
                 max_weighted_speed: float = 0.25,
                 characteristic_length: float = 0.4,
                 epsilon: float = 1e-6,
                 activation_enabled: bool = False,
                 filter_alpha: float = 0.2):
        self._robot = robot
        self.k_m = float(gain)
        self.v_N_max = float(max_weighted_speed)
        self.l_c = float(characteristic_length)
        self.epsilon = float(epsilon)
        self.activation_enabled = bool(activation_enabled)
        self.filter_alpha = float(filter_alpha)

    def manipulability(self, q, jacobian) -> jnp.ndarray:
        """phi for one configuration (pure function of q and J)."""

        return regularized_logdet_manipulability(
            jacobian, self.l_c, self.epsilon)

    def gradient(self, q, jacobian_fn) -> jnp.ndarray:
        """g_m = grad_q phi via JAX autodiff."""

        return jax.grad(
            lambda qq: self.manipulability(qq, jacobian_fn(qq)))(q)

    def nullspace_velocity(self, q, jacobian, jacobian_hash, null_projector,
                           joint_max_velocities, prev_null_velocity=None):
        """Compute the scaled manipulability-gradient self-motion command."""

        if self._robot is None:
            raise RuntimeError(
                "ManipulabilityGradientPolicy requires a robot to evaluate "
                "the Jacobian during autodiff")
        jacobian_fn = self._robot.ee_jacobian
        g_m = self.gradient(q, jacobian_fn)
        rho = jnp.asarray(1.0)  # activation disabled in v1
        weight_inv = jnp.diag(jnp.asarray(joint_max_velocities) ** 2)
        d_m = null_projector @ weight_inv @ null_projector.T @ g_m
        norm_d = jnp.linalg.norm(self.k_m * rho * d_m)
        s_v = jnp.minimum(
            1.0, self.v_N_max / jnp.maximum(norm_d, 1e-12))
        qdot_null = s_v * self.k_m * rho * d_m
        if prev_null_velocity is not None:
            qdot_null = (
                self.filter_alpha * qdot_null
                + (1.0 - self.filter_alpha) * prev_null_velocity)
        return qdot_null
