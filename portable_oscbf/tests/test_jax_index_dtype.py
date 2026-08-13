#!/usr/bin/env python3
"""Index tensors must remain int32 when the JAX float precision changes."""


def test_self_collision_pair_indices_are_explicit_int32():
    import jax
    import jax.numpy as jnp

    jax.config.update("jax_enable_x64", True)
    from work.oscbf_collision_config import SELF_COLLISION_PAIRS

    assert SELF_COLLISION_PAIRS.dtype == jnp.int32
