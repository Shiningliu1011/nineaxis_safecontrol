"""Compare the JAX OBB distance kernel against independent references.

One convention everywhere: a box is (transform=I4, center_local=c, rotation_local=R,
half_extents=h), so its world rotation is R and its world centre is c.

Regimes
  eager        : _obb_pair_distance_impl called directly on concrete inputs
  jit_traced   : jax.jit, every argument traced
  jit_const    : jax.jit, rotations held as closure constants (the call shape the
                 2026-09-08 audit used, all cases axis-aligned)
  prod         : jax.jit(jax.vmap(_obb_pair_distance)) with every argument traced
                 (the shape self_collision_distances uses)
"""
import sys, json, warnings, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc
from refs import analytic_aa, qp_reference, fcl_reference, gjk_reference

warnings.filterwarnings('ignore', category=RuntimeWarning, module='scipy.*')
I4 = jnp.eye(4); Z3 = jnp.zeros(3); I3 = jnp.eye(3)

def eager(cB, RA, hA, RB, hB, cA=None):
    cA = np.zeros(3) if cA is None else np.asarray(cA, float)
    return float(dc._obb_pair_distance_impl(I4, jnp.asarray(cA), jnp.asarray(RA), jnp.asarray(hA),
                                            I4, jnp.asarray(cB), jnp.asarray(RB), jnp.asarray(hB)))

def jit_traced(cB, RA, hA, RB, hB, cA=None):
    cA = np.zeros(3) if cA is None else np.asarray(cA, float)
    return float(jax.jit(dc._obb_pair_distance_impl)(
        I4, jnp.asarray(cA), jnp.asarray(RA), jnp.asarray(hA),
        I4, jnp.asarray(cB), jnp.asarray(RB), jnp.asarray(hB)))

def make_jit_const(RA, RB):
    RAj = jnp.asarray(RA); RBj = jnp.asarray(RB)
    return jax.jit(lambda tt, a, b: dc._obb_pair_distance(I4, Z3, RAj, a, I4, tt, RBj, b))

def prod_batch(cAs, RAs, hAs, cBs, RBs, hBs):
    n = len(cAs)
    out = dc._pair_distance_vmap(
        jnp.broadcast_to(I4, (n, 4, 4)), jnp.asarray(cAs), jnp.asarray(RAs), jnp.asarray(hAs),
        jnp.broadcast_to(I4, (n, 4, 4)), jnp.asarray(cBs), jnp.asarray(RBs), jnp.asarray(hBs))
    return np.asarray(out)

def rot(axis, angle):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

# harness self-check: with all rotations identity the four regimes must equal the closed form
chk_t = np.array([3.1, .17, .23]); chk_a = np.array([1., .7, .6]); chk_b = np.array([.4, .3, .2])
chk = dict(analytic=analytic_aa(chk_t, chk_a, chk_b), eager=eager(chk_t, I3, chk_a, I3, chk_b),
           jit_traced=jit_traced(chk_t, I3, chk_a, I3, chk_b),
           jit_const=float(make_jit_const(I3, I3)(jnp.asarray(chk_t), jnp.asarray(chk_a), jnp.asarray(chk_b))),
           prod=float(prod_batch([np.zeros(3)], [I3], [chk_a], [chk_t], [I3], [chk_b])[0]))
print('HARNESS SELF-CHECK (axis-aligned unequal boxes, expected 0.170000000):')
for k, v in chk.items():
    print(f'   {k:12s} {v:.9f}')

targeted = [
    ('aa_gap', [3., 0, 0], I3, [1, 1, 1], I3, [1, 1, 1]),
    ('aa_unequal_sep', [3.1, .17, .23], I3, [1, .7, .6], I3, [.4, .3, .2]),
    ('aa_robot_scale', [.31, .017, .023], I3, [.1, .07, .06], I3, [.04, .03, .02]),
    ('aa_touch', [2., 0, 0], I3, [1, 1, 1], I3, [1, 1, 1]),
    ('aa_overlap', [1.5, 0, 0], I3, [1, 1, 1], I3, [1, 1, 1]),
    ('aa_contained', [0, 0, 0], I3, [1, 1, 1], I3, [.2, .2, .2]),
    ('aa_identical', [0, 0, 0], I3, [1, 1, 1], I3, [1, 1, 1]),
    ('rot_face90', [.25, 0, 0], rot([0, 0, 1], np.pi / 2), [.1, .05, .05], I3, [.02, .02, .02]),
    ('rot45_sep', [.6, .2, .1], rot([0, 0, 1], np.pi / 4), [.1, .07, .06], rot([1, 1, 0], np.pi / 5), [.04, .03, .02]),
    ('rot_edge_edge', [.22, .14, .11], rot([0, 1, 1], 0.7), [.1, .06, .05], rot([1, 0, 1], 1.1), [.05, .04, .03]),
    ('rot_near_contact', [.1501, 0, 0], I3, [.1, .07, .06], I3, [.05, .02, .02]),
    ('rot_small_gap', [.1041, 0, 0], rot([0, 0, 1], np.pi / 2), [.1, .05, .05], I3, [.004, .004, .004]),
    ('micro_scale', [.0301, 0, 0], I3, [.02, .01, .01], I3, [.01, .005, .005]),
]

results = {'harness_self_check': chk, 'targeted': [], 'random': {}}
rows = []
for name, cB, RA, hA, RB, hB in targeted:
    cB = np.asarray(cB, float); hA = np.asarray(hA, float); hB = np.asarray(hB, float)
    q = qp_reference(RA, np.zeros(3), hA, RB, cB, hB)
    f = fcl_reference(RA, np.zeros(3), hA, RB, cB, hB)
    g = gjk_reference(RA, np.zeros(3), hA, RB, cB, hB)
    e = eager(cB, RA, hA, RB, hB)
    jt = jit_traced(cB, RA, hA, RB, hB)
    jc = float(make_jit_const(RA, RB)(jnp.asarray(cB), jnp.asarray(hA), jnp.asarray(hB)))
    pr = float(prod_batch([np.zeros(3)], [RA], [hA], [cB], [RB], [hB])[0])
    results['targeted'].append({
        'name': name, 'centre_b': cB.tolist(), 'hA': hA.tolist(), 'hB': hB.tolist(),
        'ref_analytic': analytic_aa(cB, hA, hB), 'ref_qp': q[0], 'ref_fcl': f[0], 'ref_gjk': g[0],
        'ref_overlap': bool(q[3] or g[1]), 'ref_fcl_colliding': f[1],
        'kernel_eager': e, 'kernel_jit_traced': jt, 'kernel_jit_const': jc, 'kernel_prod': pr,
        'err_eager': e - q[0], 'err_prod': pr - q[0], 'err_jit_const': jc - q[0]})
    rows.append((name, q[0], e, jt, jc, pr, bool(q[3] or g[1])))

print('\n%-16s %13s %13s %13s %13s %13s %8s' % ('case', 'ref_qp', 'eager', 'jit_traced', 'jit_const', 'prod', 'overlap'))
for n, r, e, jt, jc, pr, ov in rows:
    print('%-16s %13.9f %13.9f %13.9f %13.9f %13.9f %8s' % (n, r, e, jt, jc, pr, ov))

# ---- random rotated battery ------------------------------------------------
rng = np.random.default_rng(20260911)
def rand_rot(rng):
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

N = 400
cases = []
for k in range(N):
    scale = float(rng.choice([0.02, 0.05, 0.1, 0.3, 1.0]))
    hA = rng.uniform(0.4, 1.0, 3) * scale
    hB = rng.uniform(0.4, 1.0, 3) * scale
    RA = rand_rot(rng); RB = rand_rot(rng)
    reach = float(np.sum(hA) + np.sum(hB))
    cB = rng.normal(size=3); cB = cB / np.linalg.norm(cB) * rng.uniform(0.2, 1.6) * reach
    cases.append((k, cB, RA, hA, RB, hB))

prod_vals = prod_batch([np.zeros(3)] * N, [c[2] for c in cases], [c[3] for c in cases],
                       [c[1] for c in cases], [c[4] for c in cases], [c[5] for c in cases])
eager_vals = np.array([eager(cB, RA, hA, RB, hB) for _, cB, RA, hA, RB, hB in cases])

sep_eager, sep_prod, overlap_rows = [], [], []
for idx, (k, cB, RA, hA, RB, hB) in enumerate(cases):
    q = qp_reference(RA, np.zeros(3), hA, RB, cB, hB)
    f = fcl_reference(RA, np.zeros(3), hA, RB, cB, hB)
    if bool(q[3] or f[1]):
        if prod_vals[idx] > 1e-9:
            overlap_rows.append({'k': k, 'kernel_prod': float(prod_vals[idx]),
                                 'scale': float(max(hA.max(), hB.max()))})
    else:
        sep_eager.append(float(eager_vals[idx]) - q[0])
        sep_prod.append(float(prod_vals[idx]) - q[0])

def stats(v):
    v = np.asarray(v, float)
    return {'n': int(v.size), 'max_abs_err': float(np.abs(v).max()),
            'p95_abs_err': float(np.quantile(np.abs(v), .95)), 'median_abs_err': float(np.median(np.abs(v))),
            'max_overestimate_m': float(v.max()), 'max_underestimate_m': float(v.min()),
            'n_err_gt_1um': int((np.abs(v) > 1e-6).sum()),
            'n_err_gt_1mm': int((np.abs(v) > 1e-3).sum()),
            'n_overestimate_gt_1mm': int((v > 1e-3).sum()), 'n_underestimate_gt_1mm': int((v < -1e-3).sum())}

results['random'] = {'n_cases': N, 'n_separated': len(sep_prod), 'n_overlapping': N - len(sep_prod),
                     'sep_err_eager': stats(sep_eager), 'sep_err_prod': stats(sep_prod),
                     'overlap_kernel_returns_positive_count': len(overlap_rows),
                     'overlap_kernel_returns_positive_max': max([r['kernel_prod'] for r in overlap_rows], default=0.0),
                     'overlap_examples': overlap_rows[:5]}
print()
print(json.dumps(results['random'], indent=2))
Path(Path(__file__).resolve().parent / 'validate_kernel.json').write_text(json.dumps(results, indent=2))
print('\nwrote validate_kernel.json')
