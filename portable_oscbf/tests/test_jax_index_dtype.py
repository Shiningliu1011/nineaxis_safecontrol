#!/usr/bin/env python3
"""Index tensors must remain int32 when the JAX float precision changes."""


def test_obb_collision_pair_indices_are_explicit_int32():
    import jax
    import jax.numpy as jnp

    jax.config.update("jax_enable_x64", True)
    from work.obb_collision_model import OBB_COLLISION_PAIRS

    assert OBB_COLLISION_PAIRS.dtype == jnp.int32
