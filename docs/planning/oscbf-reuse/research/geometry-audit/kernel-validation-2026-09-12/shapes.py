"""Which call shapes are unsafe?  A constant/dynamic matrix over the four OBB
arguments of `_obb_faces_world`, plus the same question asked of the public
production API.

For each of 16 combinations we report max|jit - eager| on the face-centre output
and whether the jitted values are a pure permutation of the eager multiset
(permutation => layout confusion) or genuinely different numbers.
"""
import sys, json, itertools
from pathlib import Path
import numpy as np
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'portable_oscbf/work').is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc
from work import obb_collision_model as m

H = jnp.array([.31, .42, .53])          # half extents (asymmetric on purpose)
CEN = jnp.array([.011, .022, .033])     # local centre
FA = dc._FACE_NORMAL_AXIS
SIGN = dc._FACE_NORMAL_SIGN


def rot(axis, ang):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def make_transform(Rw):
    T = jnp.eye(4).at[:3, :3].set(jnp.asarray(Rw)).at[:3, 3].set(jnp.array([1.3, -.7, .4]))
    return T


TRUTH_R = rot([.3, 1.0, -.4], 0.9)
T_CONST = make_transform(TRUTH_R)
RL_CONST = jnp.asarray(rot([1., 1., 0.], 1.1))
DYNAMIC_R = jnp.asarray(rot([0., 0., 1.], 0.25))


def perm_or_scramble(a, b):
    """Classification of a vs b: identical, a permutation of the multiset, or different numbers."""
    if np.array_equal(a, b):
        return 'identical'
    if np.allclose(np.sort(np.asarray(a).ravel()), np.sort(np.asarray(b).ravel()),
                   atol=0, rtol=0):
        return 'permutation-of-values'
    return 'different-values'


print('Call-shape matrix for dc._obb_faces_world  (T transform, c centre, R local rot, h half-extents)')
print('%-4s %-4s %-4s %-4s  %12s  %12s  %s' % ('T', 'c', 'R', 'h', 'max|jit-eager|', 'max|jit-TRUTH-e|', 'classification'))
print('     (T = the 4x4 link transform, held constant vs passed as a traced argument)')

rows = []
NAMES = ('T', 'c', 'R', 'h')
PY_VALUES = {'T': T_CONST, 'c': CEN, 'R': RL_CONST, 'h': H}
for modes in itertools.product(*[('const', 'dyn')] * 4):
    const = {n: PY_VALUES[n] for n, v in zip(NAMES, modes) if v == 'const'}

    def body(T_, c_, R_, h_):
        return dc._obb_faces_world(T_, c_, R_, h_)

    if const:
        def f(*dyn_args, _c=const, _body=body):
            a = dict(_c)
            a.update(zip([n for n, v in zip(NAMES, modes) if v == 'dyn'], dyn_args))
            return _body(a['T'], a['c'], a['R'], a['h'])
    else:
        f = body
    dyn_args = tuple(PY_VALUES[n] for n, v in zip(NAMES, modes) if v == 'dyn')

    eager = body(T_CONST, CEN, RL_CONST, H)
    jit_out = jax.jit(f)(*dyn_args)

    fc_j = np.asarray(jit_out[0])
    fc_e = np.asarray(eager[0])
    cls = perm_or_scramble(fc_j, fc_e)
    T_OFF = np.array([1.3, -.7, .4])          # the constant translation inside T_CONST
    truth = (np.asarray(TRUTH_R) @ np.asarray(CEN) + T_OFF)[None, :] + (
        (np.asarray(TRUTH_R) @ np.asarray(RL_CONST))[:, np.asarray(FA)]
        * np.asarray(SIGN)[None, :]
        * np.asarray(H)[np.asarray(FA)][None, :]).T
    rec = {'T': modes[0], 'c': modes[1], 'R': modes[2], 'h': modes[3],
           'max_jit_vs_eager': float(np.abs(fc_j - fc_e).max()),
           'max_jit_vs_truth': float(np.abs(fc_j - truth).max()),
           'classification': cls}
    rows.append(rec)
    print('%-6s %-6s %-6s %-6s  %12.3e  %12.3e  %s' % (
        modes[0], modes[1], modes[2], modes[3], rec['max_jit_vs_eager'],
        rec['max_jit_vs_truth'], cls))

out = {'face_centre_call_shapes': rows}

# ---------------------------------------------------------------- public API
print('\n=== public production API: self_collision_distances ===')
import numpy as np
QLO = np.asarray(m.OBB_LOCAL_CENTERS_M, float)
QRO = np.asarray(m.OBB_LOCAL_ROTATIONS, float)   # all identity for this model
QHE = np.asarray(m.OBB_HALF_EXTENTS_M, float)
print('OBB_LOCAL_ROTATIONS all identity: %s' % np.allclose(QRO, np.eye(3)))
Q0 = jnp.zeros(9)
Q1 = jnp.array([0.35, 0.9, -1.2, -0.8, 2.4, -1.1, 0.6, -0.9, 1.3])

api = {}
def record(name, val):
    a = np.asarray(val)
    api[name] = a.tolist()
    return a

base = record('eager_q1', dc.self_collision_distances(Q1))
for name, val in [
    ('self_collision_distances_jit', dc.self_collision_distances_jit(Q1)),
    ('outer_jit', jax.jit(dc.self_collision_distances)(Q1)),
    ('vmap_jit', jax.jit(jax.vmap(dc.self_collision_distances))(jnp.stack([Q0, Q1]))[1]),
    ('constant_closure', jax.jit(lambda: dc.self_collision_distances(Q1))()),
    ('constant_closure_pair', jax.jit(lambda: dc.pair_distance(Q1, 3))()),
    ('grad_jit', dc.self_collision_distance_grad_jit(Q1, 3)),
    ('grad_eager', dc.self_collision_distance_grad(Q1, 3)),
]:
    a = np.asarray(val)
    d = float(np.abs(a - base).max()) if a.shape == base.shape else None
    api[name] = {'values': a.tolist(), 'max_abs_diff_vs_eager': d}
    print('  %-28s max_abs_diff_vs_eager=%s' % (name, ('%.3e' % d) if d is not None else 'shape %s' % (a.shape,)))

# reference for the same q: QP over the declared pairs
sys.path.insert(0, str(Path(__file__).resolve().parent))
from oracles import fk_vmap, obb_poses
from refs import qp_reference
T = np.asarray(fk_vmap(Q1[None, :]))[0]
Rw, Cw = obb_poses(T[None])[0][0], obb_poses(T[None])[1][0]
pairs = [tuple(map(int, p)) for p in np.asarray(m.OBB_COLLISION_PAIRS)]
qp = np.array([qp_reference(Rw[i], Cw[i], QHE[i], Rw[j], Cw[j], QHE[j])[0] for i, j in pairs])
print('  vs qp_reference (14 declared pairs): max|kernel-qp| = %.3e (eager), %.3e (jit)'
      % (np.abs(base - qp).max(), np.abs(np.asarray(api['self_collision_distances_jit']['values']) - qp).max()))
api['qp_reference'] = qp.tolist()

# the audit's constant call shape is covered by rule2x2.py section B

Path(Path(__file__).resolve().parent / 'shapes.json').write_text(json.dumps(out | {'public_api': api}, indent=2))
print('\nwrote shapes.json')
