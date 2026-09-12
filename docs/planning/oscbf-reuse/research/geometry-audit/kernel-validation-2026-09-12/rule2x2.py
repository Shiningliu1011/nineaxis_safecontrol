"""Decisive 2x2: rotation constant/traced x half-extents constant/traced.

World rotation is the LOCAL rotation here (the transform is identity), so the
numpy ground truth uses RL, not RW.
"""
import sys, json, itertools
from pathlib import Path
import numpy as np
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'portable_oscbf/work').is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc
from refs import qp_reference

FA = dc._FACE_NORMAL_AXIS
SIGN = dc._FACE_NORMAL_SIGN
I4 = jnp.eye(4); Z3 = jnp.zeros(3)


def rot(axis, ang):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


RL = rot([1., 1., 0.], 1.1)
H = np.array([.31, .42, .53])
CEN = np.array([.011, .022, .033])
truth = (np.asarray(CEN)[None, :]
         + (RL[:, np.asarray(FA)] * np.asarray(SIGN)[None, :] * H[np.asarray(FA)][None, :]).T)

print('A. face-centre helper, transform = identity (world rotation = local rotation)')
print('%-24s %12s  %s' % ('shape (rotation,half)', 'max|jit-truth|', 'verdict'))
out = {'helper_2x2': []}
for R_mode, h_mode in itertools.product(('const', 'traced'), repeat=2):
    Rj = jnp.asarray(RL); Hj = jnp.asarray(H)
    if R_mode == 'const':
        f = (lambda h_: dc._obb_faces_world(I4, jnp.asarray(CEN), jnp.asarray(RL), h_)) if h_mode == 'traced' \
            else (lambda: dc._obb_faces_world(I4, jnp.asarray(CEN), jnp.asarray(RL), jnp.asarray(H)))
        args = (Hj,) if h_mode == 'traced' else ()
    else:
        f = (lambda R_, h_: dc._obb_faces_world(I4, jnp.asarray(CEN), R_, h_)) if h_mode == 'traced' \
            else (lambda R_: dc._obb_faces_world(I4, jnp.asarray(CEN), R_, jnp.asarray(H)))
        args = (Rj, Hj) if h_mode == 'traced' else (Rj,)
    fc = np.asarray(jax.jit(f)(*args)[0])
    d = float(np.abs(fc - truth).max())
    perm = bool(np.allclose(np.sort(fc.ravel()), np.sort(truth.ravel()), atol=0, rtol=0))
    print('%-24s %12.3e  %s' % ('R=%s h=%s' % (R_mode, h_mode), d,
                                'CORRUPT (values permuted)' if d > 1e-12 else 'exact'))
    out['helper_2x2'].append({'rotation': R_mode, 'half_extents': h_mode, 'max_abs_err_m': d,
                              'values_permuted': perm})

print('\nB. full pair distance vs the QP oracle, one box per pattern')
RA = rot([0, 0, 1], .6); RB = rot([1, 1, 0], .5)
hA = np.array([.1, .07, .06]); hB = np.array([.04, .03, .02])
cA = np.array([.2, .1, .05]); cB = np.array([.31, .017, .023])
qp = qp_reference(RA, cA, hA, RB, cB, hB)
print('   qp reference = %.9f (overlap=%s)' % (qp[0], qp[3]))
# both boxes in the production pattern: all locals constant, the transform carries
# the geometry, and it is traced.
TA = np.eye(4); TA[:3, :3] = RA; TA[:3, 3] = cA
TB = np.eye(4); TB[:3, :3] = RB; TB[:3, 3] = cB
shapes = {
    'production (locals const, transforms traced)':
        (jax.jit(lambda ta, tb: dc._obb_pair_distance(ta, jnp.asarray([0., 0, 0]), jnp.asarray(np.eye(3)),
                                                      jnp.asarray(hA), tb, jnp.asarray([0., 0, 0]),
                                                      jnp.asarray(np.eye(3)), jnp.asarray(hB))),
         (jnp.asarray(TA), jnp.asarray(TB))),
    'audit (locals const, centres traced, transforms const id)':
        (jax.jit(lambda ca, cb: dc._obb_pair_distance(I4, ca, jnp.asarray(RA), jnp.asarray(hA),
                                                      I4, cb, jnp.asarray(RB), jnp.asarray(hB))),
         (jnp.asarray(cA), jnp.asarray(cB))),
    'audit + half-extents traced too':
        (jax.jit(lambda ca, cb, ha, hb: dc._obb_pair_distance(I4, ca, jnp.asarray(RA), ha,
                                                              I4, cb, jnp.asarray(RB), hb)),
         (jnp.asarray(cA), jnp.asarray(cB), jnp.asarray(hA), jnp.asarray(hB))),
}
out['pair_shapes'] = {}
for name, (f, args) in shapes.items():
    v = float(f(*args))
    print('   %-52s %.9f  err=%+.3e' % (name, v, v - qp[0]))
    out['pair_shapes'][name] = {'kernel': v, 'qp': qp[0], 'err': v - qp[0]}

# the real production entry at the same q, for the record
q = jnp.array([.35, .9, -1.2, -.8, 2.4, -1.1, .6, -.9, 1.3])
prod = np.asarray(dc.self_collision_distances(q))
prod_jit = np.asarray(dc.self_collision_distances_jit(q))
print('\nC. real production entry self_collision_distances: max|eager-jit| = %.3e'
      % np.abs(prod - prod_jit).max())
out['production_entry_eager_vs_jit'] = float(np.abs(prod - prod_jit).max())

# ------------------------------------------------------------------ gradients
print('\nD. gradient wrt the varied argument, same two patterns')
# analytic normal for the aa case: unit vector along the separating axis
g_prod = jax.grad(lambda ta, tb: dc._obb_pair_distance(
    ta, jnp.asarray([0., 0, 0]), jnp.asarray(np.eye(3)), jnp.asarray(hA),
    tb, jnp.asarray([0., 0, 0]), jnp.asarray(np.eye(3)), jnp.asarray(hB)),
    argnums=(0, 1))(jnp.asarray(TA), jnp.asarray(TB))
g_audit = jax.grad(lambda ca, cb: dc._obb_pair_distance(
    I4, ca, jnp.asarray(RA), jnp.asarray(hA), I4, cb, jnp.asarray(RB), jnp.asarray(hB)),
    argnums=(0, 1))(jnp.asarray(cA), jnp.asarray(cB))
for nm, g in (('production (transforms traced)', g_prod), ('audit (locals const, transforms const id)', g_audit)):
    print('   %-46s d/dcA=%s' % (nm, np.round(np.asarray(g[1]), 6)))
out['gradients'] = {'production_dcb': np.asarray(g_prod[1]).tolist(),
                    'audit_dcb': np.asarray(g_audit[1]).tolist()}

print('\nwrote rule2x2.json')
