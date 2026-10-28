from functools import partial

import jax
import jax.numpy as jnp

from work.continuous_path_reference import (
    _u_at_s,
    arc_at_u,
    curve_jet,
    sample_reference,
)


@partial(jax.custom_jvp, nondiff_argnums=(2,))
def _inverse_progress(data, s, side):
    return _u_at_s(data, s, jnp)[0]


@_inverse_progress.defjvp
def _inverse_progress_jvp(side, primals, tangents):
    data, s = primals
    data_dot, s_dot = tangents
    u = _inverse_progress(data, s, side)
    # 通过隐函数关系计算导数，使节点的精确取值保留真实弧长导数。
    _, arc_dot = jax.jvp(lambda values: arc_at_u(values, u, xp=jnp), (data,), (data_dot,))
    speed = jnp.linalg.norm(curve_jet(data, u, xp=jnp, side=side)[0][1], axis=-1)
    return u, (s_dot * data.arc_m[-1] + s * data_dot.arc_m[-1] - arc_dot) / speed


def query_reference_jax(data, s, *, side="right"):
    if data.coefficients_m.dtype != jnp.float64:
        raise ValueError("continuous reference 需要 float64 数组")
    return sample_reference(data, s, xp=jnp, side=side,
                            inverse=lambda values, value: _inverse_progress(values, value, side))

