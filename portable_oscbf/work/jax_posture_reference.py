"""JAX mirror of arc-length-indexed joint posture reference sampling.

The values are immutable closure data for a compiled path controller.  They
are not runtime state, so changing a posture plan means creating a new
controller/kernel deliberately rather than changing a JIT input shape during
tracking.
"""

from __future__ import annotations

from typing import NamedTuple

import jax.numpy as jnp

from work.path_posture_reference import PathPostureReference


_EPS = 1.0e-9


class JaxPathPostureReference(NamedTuple):
    """Array-only posture samples captured by the path JIT closure.

    When ``dq_des_dell`` is provided, it stores the derivative of the
    desired joint configuration with respect to arc length for use as
    a feedforward null-space velocity term.
    """

    arc_length_m: jnp.ndarray
    q_waypoints: jnp.ndarray
    dq_des_dell: jnp.ndarray | None = None


def as_jax_path_posture_reference(
        reference: PathPostureReference) -> JaxPathPostureReference:
    """Convert a validated host reference once before JIT warmup."""
    return JaxPathPostureReference(
        arc_length_m=jnp.asarray(reference.arc_length_m),
        q_waypoints=jnp.asarray(reference.q_waypoints),
        dq_des_dell=(jnp.asarray(reference.dq_des_dell)
                     if reference.dq_des_dell is not None else None),
    )


def sample_path_posture_reference_jax(
        reference: JaxPathPostureReference,
        progress_m: jnp.ndarray) -> jnp.ndarray | tuple[jnp.ndarray, jnp.ndarray]:
    """Interpolate a fixed 9-joint target at clamped arc-length progress.

    Returns:
        If ``dq_des_dell`` is None: ``(q_des,)`` as a single jnp.ndarray.
        If ``dq_des_dell`` is provided: ``(q_des, dq_des_dell)`` tuple.
    """
    n_segments = reference.q_waypoints.shape[0] - 1
    total = reference.arc_length_m[-1]
    progress = jnp.clip(progress_m, 0.0, total)
    segment = jnp.clip(
        jnp.searchsorted(reference.arc_length_m, progress, side='right') - 1,
        0,
        n_segments - 1,
    ).astype(jnp.int32)
    start = reference.arc_length_m[segment]
    span = reference.arc_length_m[segment + 1] - start
    fraction = jnp.clip((progress - start) / jnp.maximum(span, _EPS), 0.0, 1.0)
    q_des = ((1.0 - fraction) * reference.q_waypoints[segment]
             + fraction * reference.q_waypoints[segment + 1])
    if reference.dq_des_dell is not None:
        dq_des = ((1.0 - fraction) * reference.dq_des_dell[segment]
                  + fraction * reference.dq_des_dell[segment + 1])
        return q_des, dq_des
    return q_des
