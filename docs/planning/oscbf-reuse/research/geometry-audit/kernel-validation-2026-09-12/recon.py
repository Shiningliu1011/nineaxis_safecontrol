"""Reconcile: which *faithful* call shapes corrupt the kernel, and is it value-dependent?

Uses the real production helpers (dc._obb_faces_world / dc._obb_pair_distance),
not copies.  Every shape is compared against the eager evaluation of the same
expression on the same numbers.
"""
import sys, json
from pathlib import Path
import numpy as np
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'portable_oscbf/work').is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc
from refs import analytic_aa, qp_reference, gjk_reference

I4 = jnp.eye(4); Z3 = jnp.zeros(3); I3 = jnp.eye(3)
out = {}


def rot(axis, ang):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


print('=== faithful audit shape: make_jit_const (transforms and rotations constant) ===')
a_u, b_u = np.array([1., .7, .6]), np.array([.4, .3, .2])
t_u = np.array([3.1, .17, .23])
a_r, b_r = np.array([.1, .07, .06]), np.array([.04, .03, .02])
t_r = np.array([.31, .017, .023])
cases = [('aa_unequal_sep', t_u, I3, a_u, I3, b_u, analytic_aa(t_u, a_u, b_u)),
         ('aa_robot_scale', t_r, I3, a_r, I3, b_r, analytic_aa(t_r, a_r, b_r)),
         ('rot45_sep', [.6, .2, .1], rot([0, 0, 1], np.pi / 4), a_r, rot([1, 1, 0], np.pi / 5), b_r, None),
         ('rot_edge_edge', [.22, .14, .11], rot([0, 1, 1], .7), [.1, .06, .05], rot([1, 0, 1], 1.1), [.05, .04, .03], None)]


def make_jit_const(RA, RB):
    RAj = jnp.asarray(RA); RBj = jnp.asarray(RB)
    return jax.jit(lambda tt, a, b: dc._obb_pair_distance(I4, Z3, RAj, a, I4, tt, RBj, b))


rows = []
for name, cB, RA, hA, RB, hB, analytic in cases:
    eager = float(dc._obb_pair_distance_impl(I4, Z3, jnp.asarray(RA), jnp.asarray(hA),
                                             I4, jnp.asarray(cB), jnp.asarray(RB), jnp.asarray(hB)))
    jt = float(jax.jit(dc._obb_pair_distance_impl)(
        I4, Z3, jnp.asarray(RA), jnp.asarray(hA), I4, jnp.asarray(cB), jnp.asarray(RB), jnp.asarray(hB)))
    jc = float(make_jit_const(RA, RB)(jnp.asarray(cB), jnp.asarray(hA), jnp.asarray(hB)))
    qp = qp_reference(RA, np.zeros(3), hA, RB, np.asarray(cB), hB)[0]
    gj = gjk_reference(RA, np.zeros(3), hA, RB, np.asarray(cB), hB)[0]
    re_ = {'case': name, 'ref_qp': qp, 'ref_gjk': gj, 'ref_analytic': analytic,
           'eager': eager, 'jit_traced': jt, 'jit_const': jc,
           'err_eager': eager - qp, 'err_jit_const': jc - qp}
    rows.append(re_)
    print('%-16s qp=%.9f eager=%.9f jit_traced=%.9f jit_const=%.9f  err_jit_const=%+.3e'
          % (name, qp, eager, jt, jc, jc - qp))
out['audit_shape_pair_distance'] = rows

print('\n=== same numbers, but the varying quantity passed as the TRANSFORM (production shape) ===')
for name, cB, RA, hA, RB, hB, analytic in cases:
    # the transform carries the world rotation and centre, so it is the traced value;
    # the local rotation stays a closure constant.
    Rc = np.asarray(RA); cc = np.asarray(cB)
    Tw = np.eye(4); Tw[:3, :3] = Rc; Tw[:3, 3] = cc
    jf = jax.jit(lambda T_, a, b, _RB=jnp.asarray(RB), _RA=jnp.asarray(RA):
                 dc._obb_pair_distance_impl(I4, Z3, _RA, a, T_, Z3, _RB, b))
    v = float(jf(jnp.asarray(Tw), jnp.asarray(hA), jnp.asarray(hB)))
    e = float(dc._obb_pair_distance_impl(I4, Z3, jnp.asarray(RA), jnp.asarray(hA),
                                         jnp.asarray(Tw), Z3, jnp.asarray(RB), jnp.asarray(hB)))
    print('%-16s eager=%.9f jit(transform traced)=%.9f  diff=%.3e' % (name, e, v, v - e))
    out.setdefault('transform_traced_shape', []).append({'case': name, 'eager': e, 'jit': v, 'diff': v - e})

print('\n=== value dependence of the face-centre helper (constant rotation, traced half-extents) ===')
probe = []
for Rname, Rn in (('identity', np.eye(3)), ('rot_z_0.4', rot([0, 0, 1], .4)), ('rot_xy_1.1', rot([1, 1, 0], 1.1)),
                  ('rot_xyz_gen', rot([.3, 1., -.4], .9))):
    Rj = jnp.asarray(Rn)
    for hname, hn in (('unit_repeat', np.array([1., 1., .7])), ('generic', np.array([.31, .42, .53])),
                      ('robot_like', np.array([.1, .07, .06]))):
        hj = jnp.asarray(hn)
        body = lambda h_, _R=Rj, _c=Z3: dc._obb_faces_world(I4, _c, _R, h_)
        fc_j = np.asarray(jax.jit(body)(hj)[0])
        fc_e = np.asarray(body(hj)[0])
        d = float(np.abs(fc_j - fc_e).max())
        perm = np.allclose(np.sort(fc_j.ravel()), np.sort(fc_e.ravel()), atol=0, rtol=0)
        probe.append({'rotation': Rname, 'half': hname, 'max_abs_diff': d, 'permutation_of_values': bool(perm)})
        print('  %-12s %-12s maxdiff=%.3e permutation=%s' % (Rname, hname, d, perm))
out['face_centre_value_dependence'] = probe

Path(Path(__file__).resolve().parent / 'recon.json').write_text(json.dumps(out, indent=2))
print('\nwrote recon.json')
