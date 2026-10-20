from __future__ import annotations

from typing import NamedTuple

import jax
import jax.numpy as jnp
import numpy as np

from work.jax_path_following import JaxPathGeometry, sample_path_jax
from work.tool_axis_task import skew_matrix_jax


class MovingReferenceTerms(NamedTuple):
    error: jnp.ndarray
    jacobian: jnp.ndarray
    reference_rate: jnp.ndarray
    position_m: jnp.ndarray
    rotation: jnp.ndarray
    angle_rad: jnp.ndarray
    valid: jnp.ndarray


def true_axis_angle(current: np.ndarray, reference: np.ndarray):
    current, reference = np.asarray(current, dtype=float), np.asarray(reference, dtype=float)
    if (current.shape != reference.shape or current.shape[-1:] != (3,)
            or not np.all(np.isfinite(current)) or not np.all(np.isfinite(reference))
            or np.any(np.linalg.norm(current, axis=-1) <= 1e-12)
            or np.any(np.linalg.norm(reference, axis=-1) <= 1e-12)):
        raise ValueError("工具轴必须是形状相同、有限且非零的三维向量")
    return np.arctan2(np.linalg.norm(np.cross(reference, current), axis=-1),
                     np.sum(reference * current, axis=-1))


def _direction_error(current_axis, reference_rotation):
    desired_axis = reference_rotation[:, 0]
    cross = jnp.cross(desired_axis, current_axis)
    sine_sq = jnp.dot(cross, cross)
    cosine = jnp.dot(desired_axis, current_axis)
    sine = jnp.sqrt(jnp.maximum(sine_sq, 1e-24))
    # 同向附近使用解析展开，使自动微分在零误差处保持有限。
    scale = jnp.where(
        cosine > 1 - 1e-8,
        1 + sine_sq / 6 + 3 * sine_sq ** 2 / 40,
        jnp.arctan2(sine, cosine) / sine,
    )
    return reference_rotation[:, 1:].T @ (cross * scale)


@jax.jit
def moving_reference_terms_jax(
        position: jnp.ndarray, rotation: jnp.ndarray, full_jacobian: jnp.ndarray,
        geometry: JaxPathGeometry, progress_m: jnp.ndarray) -> MovingReferenceTerms:
    sample = sample_path_jax(geometry, progress_m)
    current_axis = rotation[:, 0]

    def reference_error(progress):
        reference = sample_path_jax(geometry, progress)
        return jnp.concatenate([
            position - reference.position_m,
            _direction_error(current_axis, reference.rotation),
        ])

    error = reference_error(progress_m)
    axis_derivative = jax.jacfwd(_direction_error, argnums=0)(current_axis, sample.rotation)
    jacobian = jnp.concatenate([
        full_jacobian[:3],
        axis_derivative @ (-skew_matrix_jax(current_axis)) @ full_jacobian[3:],
    ])
    reference_rate = -jax.jacfwd(reference_error)(progress_m)
    cosine = jnp.dot(current_axis, sample.rotation[:, 0])
    angle = jnp.arctan2(
        jnp.linalg.norm(jnp.cross(sample.rotation[:, 0], current_axis)), cosine,
    )
    finite = jnp.all(jnp.isfinite(jnp.concatenate([
        error, jacobian.reshape(-1), reference_rate, jnp.atleast_1d(angle),
    ])))
    # 完全反向的球面方向没有唯一导数，交由执行边界报告任务无效。
    valid = finite & (cosine > -1 + 1e-9) & (
        jnp.abs(jnp.dot(current_axis, current_axis) - 1) < 1e-8
    )
    return MovingReferenceTerms(
        error, jacobian, reference_rate, sample.position_m, sample.rotation, angle, valid,
    )
