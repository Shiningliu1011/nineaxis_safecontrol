import numpy as np, jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp

I4 = jnp.eye(4); Z3 = jnp.zeros(3); R3 = jnp.eye(3)
FA = jnp.array([0, 0, 1, 1, 2, 2], dtype=jnp.int32)
SIGN = jnp.array([1., -1., 1., -1., 1., -1.])
TA = jnp.array([[1, 2], [1, 2], [0, 2], [0, 2], [0, 1], [0, 1]], dtype=jnp.int32)
half = jnp.array([1., .7, .6])

def check(tag, f, *args):
    e = np.asarray(f(*args)); j = np.asarray(jax.jit(f)(*args))
    ok = np.array_equal(e, j)
    extra = ''
    if not ok:
        extra = '  <-- jit == eager.reshape(6,3): %s' % np.array_equal(j, e.reshape(6, 3))
    print(f'{tag:52s} match={ok}{extra}')

# V1: verbatim structure
def v1(h):
    rotation = I4[:3, :3] @ R3
    center = I4[:3, :3] @ Z3 + I4[:3, 3]
    nd = rotation[:, FA] * SIGN[None, :]
    return center[None, :] + (nd * h[FA][None, :]).T
check('v1 verbatim (const rotation, .T)', v1, half)

# V2: rotation literal (no matmul)
def v2(h):
    nd = R3[:, FA] * SIGN[None, :]
    return Z3[None, :] + (nd * h[FA][None, :]).T
check('v2 literal rotation', v2, half)

# V3: precomputed constant nd, no gather/mul
ND = (R3[:, FA] * SIGN[None, :])
def v3(h):
    return Z3[None, :] + (ND * h[FA][None, :]).T
check('v3 constant nd', v3, half)

# V4: swapaxes instead of .T
def v4(h):
    nd = R3[:, FA] * SIGN[None, :]
    return Z3[None, :] + jnp.swapaxes(nd * h[FA][None, :], 0, 1)
check('v4 swapaxes', v4, half)

# V5: no center add
def v5(h):
    nd = R3[:, FA] * SIGN[None, :]
    return (nd * h[FA][None, :]).T
check('v5 no centre add', v5, half)

# V6: half as a traced (6,) without the gather
def v6(h):
    nd = R3[:, FA] * SIGN[None, :]
    return Z3[None, :] + (nd * h[None, :]).T
check('v6 no gather on half', v6, jnp.array([1., 1., .7, .7, .6, .6]))

# V7: same but the transpose applies to a matrix that is NOT multiplied
def v7(h):
    nd = R3[:, FA] + 0.0 * h[FA][None, :]
    return Z3[None, :] + nd.T
check('v7 add-then-transpose', v7, half)

# V8: tiled constant instead of broadcast product
def v8(h):
    nd = jnp.broadcast_to(SIGN[None, :], (3, 6)) * R3[:, FA]
    return Z3[None, :] + (nd * h[FA][None, :]).T
check('v8 broadcast_to SIGN', v8, half)
