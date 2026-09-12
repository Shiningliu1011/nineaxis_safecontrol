import numpy as np, jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp

R3 = jnp.eye(3); Z3 = jnp.zeros(3)
FA = jnp.array([0, 0, 1, 1, 2, 2], dtype=jnp.int32)
SIGN = jnp.array([1., -1., 1., -1., 1., -1.])
half = jnp.array([1., .7, .6])

def v2(h):
    nd = R3[:, FA] * SIGN[None, :]
    return Z3[None, :] + (nd * h[FA][None, :]).T

e = np.asarray(v2(half)); j = np.asarray(jax.jit(v2)(half))
np.set_printoptions(precision=6, suppress=True)
print('eager (correct transpose):\n', e)
print('jit   (what actually ran):\n', j)
print('jit == eager.reshape(6,3) :', np.array_equal(j, e.reshape(6, 3)))
print('jit == eager.T             :', np.array_equal(j, e.T))
nd = np.asarray(R3[:, FA] * SIGN[None, :]) * np.asarray(half)[np.asarray(FA)][None, :]
print('\npre-transpose nd (3,6):\n', nd)
print('nd.T           :\n', nd.T)
print('nd.reshape(6,3):\n', nd.reshape(6, 3))
print('nd.ravel()->(6,3) same as reshape:', np.array_equal(nd.reshape(6, 3), nd.T))
print('\njit - Z3 == nd.reshape(6,3)?', np.array_equal(j, nd.reshape(6, 3) + np.asarray(Z3)[None, :]))
for order in ('C', 'F'):
    print(f'nd.reshape(6,3,order={order}) == jit:', np.array_equal(j, nd.reshape(6, 3, order=order) + np.asarray(Z3)[None, :]))
print('\nnd.flatten()      :', nd.flatten())
print('eager-0 flatten() :', e.flatten())

# trigger sweep: which elements of the expression matter
def probe(tag, fn, *a):
    ee = np.asarray(fn(*a)); jj = np.asarray(jax.jit(fn)(*a))
    print(f'{tag:46s} match={np.array_equal(ee, jj)}')

probe('const nd, traced h, .T', lambda h: (R3[:, FA] * SIGN[None, :] * h[FA][None, :]).T, half)
probe('traced nd (add 0*h), .T', lambda h: (R3[:, FA] * SIGN[None, :] + 0.0 * h[FA][None, :]).T, half)
probe('no SIGN multiply, .T', lambda h: (R3[:, FA] * h[FA][None, :]).T, half)
probe('SIGN as python list, .T', lambda h: (R3[:, FA] * np.array([1., -1, 1, -1, 1, -1])[None, :] * h[FA][None, :]).T, half)
probe('half literal const, .T', lambda h: (R3[:, FA] * SIGN[None, :] * jnp.ones(6)[None, :]).T, half)
