"""Geometry for a 5-D position plus tool-axis task.

The task keeps Cartesian position and the direction of the tool X axis.  It
deliberately leaves rotation about that axis free, which is appropriate only
for processes whose nozzle/tool is roll-insensitive.  All vectors are in the
world frame; angular Jacobian rows use radians per second.

This module is pure task geometry.  It has no ROS, QP, collision, or runner
state so its sign conventions can be verified independently by finite
differences before being used by a controller.
"""

from __future__ import annotations

import jax.numpy as jnp
import numpy as np

from work.task_mode_contract import (
    SUPPORTED_TASK_MODES,
    TASK_MODE_POSE_6D,
    TASK_MODE_TOOL_AXIS_5D,
)


TOOL_AXIS_INDEX = 0


def skew_matrix(vector: np.ndarray) -> np.ndarray:
    """Return ``[vector]_x`` such that ``[v]_x @ w == cross(v, w)``."""
    value = np.asarray(vector, dtype=float).reshape(3)
    return np.array([
        [0.0, -value[2], value[1]],
        [value[2], 0.0, -value[0]],
        [-value[1], value[0], 0.0],
    ])


def tool_axis_and_basis(rotation: np.ndarray,
                        tool_axis_index: int = TOOL_AXIS_INDEX) -> tuple[np.ndarray, np.ndarray]:
    """Return one tool axis and its two orthonormal desired tangent directions.

    The two remaining columns of a valid rotation matrix are already an
    orthonormal basis of the plane normal to the chosen tool axis.  They only
    provide coordinates for the two controlled orientation components; no
    roll angle is used as an error signal.
    """
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError(f'rotation must have shape (3, 3), got {matrix.shape}')
    axis_index = int(tool_axis_index)
    if axis_index not in (0, 1, 2):
        raise ValueError('tool_axis_index must be 0, 1, or 2')
    axis = matrix[:, axis_index]
    axis_norm = float(np.linalg.norm(axis))
    if not np.isfinite(axis_norm) or axis_norm <= 1.0e-12:
        raise ValueError('tool axis must be finite and non-zero')
    basis = np.delete(matrix, axis_index, axis=1)
    return axis / axis_norm, basis


def tool_axis_error_2d(current_rotation: np.ndarray, desired_rotation: np.ndarray,
                       tool_axis_index: int = TOOL_AXIS_INDEX) -> np.ndarray:
    """Return the two-component tool-axis error in the desired tangent plane.

    ``a_des x a_cur`` has the useful small-angle convention that a positive
    rotation error produces a positive task error.  The velocity controller
    therefore uses ``-K * error`` and rotates back toward the desired axis.
    A pure roll leaves ``a_cur`` unchanged and yields exactly zero error.
    """
    current_axis, _ = tool_axis_and_basis(current_rotation, tool_axis_index)
    desired_axis, basis = tool_axis_and_basis(desired_rotation, tool_axis_index)
    return basis.T @ np.cross(desired_axis, current_axis)


def tool_axis_jacobian_2d(current_rotation: np.ndarray, desired_rotation: np.ndarray,
                          angular_jacobian: np.ndarray,
                          tool_axis_index: int = TOOL_AXIS_INDEX) -> np.ndarray:
    """Map joint velocity to the local two-component tool-axis error rate.

    For a fixed desired axis, ``a_dot = omega x a`` and
    ``e = a_des x a``.  Thus ``e_dot = B.T [a_des]_x (-[a]_x) J_omega``.
    The row along the tool axis is absent, leaving a 2xN orientation task.
    """
    current_axis, _ = tool_axis_and_basis(current_rotation, tool_axis_index)
    desired_axis, basis = tool_axis_and_basis(desired_rotation, tool_axis_index)
    jacobian = np.asarray(angular_jacobian, dtype=float)
    if jacobian.ndim != 2 or jacobian.shape[0] != 3:
        raise ValueError('angular_jacobian must have shape (3, N)')
    return basis.T @ skew_matrix(desired_axis) @ (-skew_matrix(current_axis)) @ jacobian


def task_jacobian_5d(full_jacobian: np.ndarray, current_rotation: np.ndarray,
                     desired_rotation: np.ndarray,
                     tool_axis_index: int = TOOL_AXIS_INDEX) -> np.ndarray:
    """Return a 5xN Jacobian: Cartesian position plus two tool-axis rows."""
    jacobian = np.asarray(full_jacobian, dtype=float)
    if jacobian.ndim != 2 or jacobian.shape[0] != 6:
        raise ValueError('full_jacobian must have shape (6, N)')
    axis_rows = tool_axis_jacobian_2d(
        current_rotation, desired_rotation, jacobian[3:], tool_axis_index)
    return np.vstack([jacobian[:3], axis_rows])


def tool_axis_angular_velocity_2d(desired_rotation: np.ndarray,
                                  desired_omega: np.ndarray,
                                  tool_axis_index: int = TOOL_AXIS_INDEX) -> np.ndarray:
    """Project desired world angular velocity into the two controlled rows."""
    _, basis = tool_axis_and_basis(desired_rotation, tool_axis_index)
    return basis.T @ np.asarray(desired_omega, dtype=float).reshape(3)


def task_error_5d(current_position: np.ndarray, desired_position: np.ndarray,
                  current_rotation: np.ndarray, desired_rotation: np.ndarray,
                  tool_axis_index: int = TOOL_AXIS_INDEX) -> np.ndarray:
    """Return position error followed by the two controlled axis errors."""
    position_error = (np.asarray(current_position, dtype=float).reshape(3)
                      - np.asarray(desired_position, dtype=float).reshape(3))
    return np.concatenate([
        position_error,
        tool_axis_error_2d(current_rotation, desired_rotation, tool_axis_index),
    ])


def task_error_report_6d(error_5d: np.ndarray) -> np.ndarray:
    """Pad a 5-D error for existing 6-D telemetry consumers.

    The final zero is intentionally not a roll error.  It keeps legacy CSV
    fields fixed-shape while ``norm(err[3:])`` remains the tool-axis error.
    """
    error = np.asarray(error_5d, dtype=float).reshape(5)
    return np.concatenate([error[:3], error[3:], np.zeros(1)])


def skew_matrix_jax(vector: jnp.ndarray) -> jnp.ndarray:
    """JAX version of :func:`skew_matrix`."""
    value = jnp.asarray(vector).reshape(3)
    return jnp.array([
        [0.0, -value[2], value[1]],
        [value[2], 0.0, -value[0]],
        [-value[1], value[0], 0.0],
    ])


def _tool_axis_and_basis_jax(rotation: jnp.ndarray,
                             tool_axis_index: int = TOOL_AXIS_INDEX) -> tuple[jnp.ndarray, jnp.ndarray]:
    """JAX tangent basis for the project's fixed tool-X convention.

    ``tool_axis_index`` is intentionally static.  Supporting another tool
    axis is a model contract change and must compile a separate kernel.
    """
    if int(tool_axis_index) != TOOL_AXIS_INDEX:
        raise ValueError('the JAX controller currently supports tool X (index 0) only')
    matrix = jnp.asarray(rotation).reshape(3, 3)
    return matrix[:, 0], matrix[:, 1:3]


def tool_axis_error_2d_jax(current_rotation: jnp.ndarray,
                           desired_rotation: jnp.ndarray,
                           tool_axis_index: int = TOOL_AXIS_INDEX) -> jnp.ndarray:
    """JAX mirror of :func:`tool_axis_error_2d`."""
    current_axis, _ = _tool_axis_and_basis_jax(current_rotation, tool_axis_index)
    desired_axis, basis = _tool_axis_and_basis_jax(desired_rotation, tool_axis_index)
    return basis.T @ jnp.cross(desired_axis, current_axis)


def tool_axis_jacobian_2d_jax(current_rotation: jnp.ndarray,
                              desired_rotation: jnp.ndarray,
                              angular_jacobian: jnp.ndarray,
                              tool_axis_index: int = TOOL_AXIS_INDEX) -> jnp.ndarray:
    """JAX mirror of :func:`tool_axis_jacobian_2d`."""
    current_axis, _ = _tool_axis_and_basis_jax(current_rotation, tool_axis_index)
    desired_axis, basis = _tool_axis_and_basis_jax(desired_rotation, tool_axis_index)
    return (basis.T @ skew_matrix_jax(desired_axis)
            @ (-skew_matrix_jax(current_axis)) @ jnp.asarray(angular_jacobian))


def task_jacobian_5d_jax(full_jacobian: jnp.ndarray, current_rotation: jnp.ndarray,
                         desired_rotation: jnp.ndarray,
                         tool_axis_index: int = TOOL_AXIS_INDEX) -> jnp.ndarray:
    """JAX mirror of :func:`task_jacobian_5d`."""
    jacobian = jnp.asarray(full_jacobian)
    axis_rows = tool_axis_jacobian_2d_jax(
        current_rotation, desired_rotation, jacobian[3:], tool_axis_index)
    return jnp.concatenate([jacobian[:3], axis_rows], axis=0)


def tool_axis_angular_velocity_2d_jax(desired_rotation: jnp.ndarray,
                                      desired_omega: jnp.ndarray,
                                      tool_axis_index: int = TOOL_AXIS_INDEX) -> jnp.ndarray:
    """JAX mirror of :func:`tool_axis_angular_velocity_2d`."""
    _, basis = _tool_axis_and_basis_jax(desired_rotation, tool_axis_index)
    return basis.T @ jnp.asarray(desired_omega).reshape(3)


def task_error_5d_jax(current_position: jnp.ndarray, desired_position: jnp.ndarray,
                      current_rotation: jnp.ndarray, desired_rotation: jnp.ndarray,
                      tool_axis_index: int = TOOL_AXIS_INDEX) -> jnp.ndarray:
    """JAX mirror of :func:`task_error_5d`."""
    position_error = (jnp.asarray(current_position).reshape(3)
                      - jnp.asarray(desired_position).reshape(3))
    return jnp.concatenate([
        position_error,
        tool_axis_error_2d_jax(current_rotation, desired_rotation, tool_axis_index),
    ])


def task_error_report_6d_jax(error_5d: jnp.ndarray) -> jnp.ndarray:
    """JAX mirror of :func:`task_error_report_6d`."""
    error = jnp.asarray(error_5d).reshape(5)
    return jnp.concatenate([error[:3], error[3:], jnp.zeros(1, dtype=error.dtype)])
