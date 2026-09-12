import numpy as np, jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp

CONST = jnp.array([[1., -1, 0, 0, 0, 0],
                   [0, 0, 1., -1, 0, 0],
                   [0, 0, 0, 0, 1., -1]])
h = jnp.array([1., 1., .7, .7, .6, .6])   # half_extents[FACE_NORMAL_AXIS]
ZERO = jnp.zeros(3)

def f1(h):                      # const-const broadcast (rotation constant, center constant)
    nd = CONST * h[None, :]
    return nd.T + ZERO[None, :]

def f2(h):                      # center as arg instead of closure constant
    nd = CONST * h[None, :]
    return nd.T + jnp.zeros(3)[None, :]

def f3(h):                      # no zero-add
    nd = CONST * h[None, :]
    return nd.T

def f4(h):                      # center as a real traced argument
    return (CONST * h[None, :]).T

def f5(h, c):                   # explicit transposed multiply instead of .T
    nd = CONST * h[None, :]
    return jnp.einsum('ij->ji', nd) + c[None, :]

for name, fn, args in [('f1 const center closure', f1, (h,)),
                       ('f2 zeros(3) fresh const', f2, (h,)),
                       ('f3 transpose only', f3, (h,)),
                       ('f4 transpose only (dup)', f4, (h,)),
                       ('f5 einsum transpose', f5, (h, jnp.zeros(3)))]:
    e = np.asarray(fn(*args)); j = np.asarray(jax.jit(fn)(*args))
    print(f'{name:26s} eager==jit: {np.array_equal(e, j)}')
    if not np.array_equal(e, j):
        print('   eager\n', e, '\n   jit\n', j, '\n   jit is C-order reshape of eager:', np.array_equal(j, e.reshape(6, 3)))

# does it also happen without any closure constants at all?
def g(const, hh, c):
    return (const * hh[None, :]).T + c[None, :]
print('g traced const ==  ', np.array_equal(np.asarray(g(CONST, h, jnp.zeros(3))), np.asarray(jax.jit(g)(CONST, h, jnp.zeros(3)))))
# and via vmap (the production wrapper shape)
def gm(const, hh, c):
    return (const * hh[None, :]).T + c[None, :]
H = jnp.stack([h, h * 2])
print('vmap traced ==      ', np.array_equal(np.asarray(jax.vmap(gm, in_axes=(None, 0, 0))(CONST, H, jnp.zeros((2, 3)))),
                                             np.asarray(jax.jit(jax.vmap(gm, in_axes=(None, 0, 0)))(CONST, H, jnp.zeros((2, 3))))))
