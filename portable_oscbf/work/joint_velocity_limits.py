"""Joint velocity and acceleration limit helpers."""

from __future__ import annotations

import numpy as np


def velocity_box_bounds(prev_velocity, dq_max, ddq_max, dt: float, scale: float = 1.0):
    """Return hard velocity bounds that also limit per-step acceleration.

    The returned bounds enforce:

    ``prev_velocity - scale * ddq_max * dt <= u <= prev_velocity + scale * ddq_max * dt``

    while still respecting the absolute joint velocity limits ``dq_max``.
    """
    prev = np.asarray(prev_velocity, dtype=float).reshape(-1)
    dq = np.asarray(dq_max, dtype=float).reshape(prev.shape)
    ddq = np.asarray(ddq_max, dtype=float).reshape(prev.shape)
    max_delta = np.maximum(ddq * max(float(dt), 0.0) * max(float(scale), 0.0), 0.0)

    lower = np.maximum(-dq, prev - max_delta)
    upper = np.minimum(dq, prev + max_delta)

    invalid = lower > upper
    if np.any(invalid):
        center = np.clip(prev, -dq, dq)
        lower[invalid] = center[invalid]
        upper[invalid] = center[invalid]
    return lower, upper
