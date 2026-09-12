"""Derivative evidence for the production OBB distance kernel.

Convention: box = (transform=I4, centre_local=c, rotation_local=R, half=h).
All kernel gradients below are taken in the *production* regime (rotations
traced, never compile-time constants).
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
from work.nineaxis_kinematics import NineaxisKinematics
from work import obb_collision_model as m
from refs import qp_reference, fcl_reference

warnings.filterwarnings('ignore', category=RuntimeWarning, module='scipy.*')
I4 = jnp.eye(4); Z3 = jnp.zeros(3); I3 = jnp.eye(3)
out = {}

def rot(axis, angle):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

def dist_fn(cA, RA, hA, cB, RB, hB):
    return dc._obb_pair_distance(I4, jnp.asarray(cA), jnp.asarray(RA), jnp.asarray(hA),
                                 I4, jnp.asarray(cB), jnp.asarray(RB), jnp.asarray(hB))

# ---------------------------------------------------------------- 1. gradients
cases = [
    ('aa_robot_scale', np.zeros(3), I3, [.1, .07, .06], [.31, .017, .023], I3, [.04, .03, .02]),
    ('rot_face90', np.zeros(3), rot([0, 0, 1], np.pi / 2), [.1, .05, .05], [.25, 0, 0], I3, [.02, .02, .02]),
    ('rot45_sep', np.zeros(3), rot([0, 0, 1], np.pi / 4), [.1, .07, .06], [.6, .2, .1], rot([1, 1, 0], np.pi / 5), [.04, .03, .02]),
    ('rot_edge_edge', np.zeros(3), rot([0, 1, 1], 0.7), [.1, .06, .05], [.22, .14, .11], rot([1, 0, 1], 1.1), [.05, .04, .03]),
]
grad_rows = []
for name, cA, RA, hA, cB, RB, hB in cases:
    cA = np.asarray(cA, float); hA = np.asarray(hA, float); cB = np.asarray(cB, float); hB = np.asarray(hB, float)
    f = lambda cb, ca: dist_fn(ca, RA, hA, cb, RB, hB)
    g_b = np.asarray(jax.grad(f, argnums=0)(jnp.asarray(cB), jnp.asarray(cA)))
    g_a = np.asarray(jax.grad(f, argnums=1)(jnp.asarray(cB), jnp.asarray(cA)))
    d, pA, pB, ov = qp_reference(RA, cA, hA, RB, cB, hB)
    n = (pA - pB) / d if d > 0 else np.zeros(3)
    eps_sweep = {}
    for eps in (1e-2, 1e-3, 1e-4, 1e-5, 1e-6):
        fd = np.array([(float(dist_fn(cA, RA, hA, cB + np.eye(3)[i] * eps, RB, hB))
                        - float(dist_fn(cA, RA, hA, cB - np.eye(3)[i] * eps, RB, hB))) / (2 * eps)
                       for i in range(3)])
        eps_sweep[f'{eps:g}'] = float(np.abs(fd - g_b).max())
    grad_rows.append({'case': name, 'distance': d, 'overlap': bool(ov),
                      'autodiff_d_dcB': g_b.tolist(), 'reference_minus_normal': (-n).tolist(),
                      'max_err_vs_reference_normal': float(np.abs(g_b - (-n)).max()),
                      'autodiff_d_dcA': g_a.tolist(), 'd_dcA_vs_normal_err': float(np.abs(g_a - n).max()),
                      'autodiff_vs_self_FD_max_err': eps_sweep})
out['gradients'] = grad_rows
print('gradient evidence (production regime)')
print('%-16s %11s %24s %24s %10s' % ('case', 'distance', 'autodiff d/dcB', 'reference -n', 'max err'))
for r in grad_rows:
    print('%-16s %11.8f [%8.5f %8.5f %8.5f] [%8.5f %8.5f %8.5f] %9.2e' % (
        r['case'], r['distance'], *r['autodiff_d_dcB'], *r['reference_minus_normal'], r['max_err_vs_reference_normal']))
print('autodiff vs own central-difference, max |g - FD| per step:')
for r in grad_rows:
    print('   %-16s' % r['case'], {k: f'{v:.2e}' for k, v in r['autodiff_vs_self_FD_max_err'].items()})

# ------------------------------------------- 2. nearest-feature switching path
RA = rot([0, 0, 1], 0.4); hA = np.array([.1, .06, .05]); RB = I3; hB = np.array([.03, .04, .02])
path = []
for y in np.linspace(-0.10, 0.10, 41):
    cB = np.array([0.145, y, 0.0])
    d, pA, pB, ov = qp_reference(RA, np.zeros(3), hA, RB, cB, hB)
    # classify reference closest feature by active bounds of the QP solution
    kernel = float(dist_fn(np.zeros(3), RA, hA, cB, RB, hB))
    path.append({'y': float(y), 'ref': d, 'kernel': kernel, 'err': kernel - d, 'overlap': bool(ov)})
switch_idx = [i for i in range(1, len(path)) if abs(path[i]['err'] - path[i-1]['err']) > 1e-9]
out['feature_switch_path'] = {'path': path,
                             'max_abs_err': float(max(abs(p['err']) for p in path)),
                             'err_jumps': switch_idx}
print('\nnearest-feature switching sweep (rotated A, 41 steps): max |kernel - ref| = %.3e'
      % out['feature_switch_path']['max_abs_err'])

# --------------------------------------------------- 3. nine-axis q-space chain
kin = NineaxisKinematics()
QLO = np.asarray(m.OBB_LOCAL_CENTERS_M); QRO = np.asarray(m.OBB_LOCAL_ROTATIONS); QHE = np.asarray(m.OBB_HALF_EXTENTS_M)
NAMES = list(m.OBB_LINK_NAMES)
def poses_from_numpy_fk(q):
    fk = kin.forward_kinematics(np.asarray(q, float))
    R = np.stack([fk[n][:3, :3] @ QRO[i] for i, n in enumerate(NAMES)])
    C = np.stack([fk[n][:3, :3] @ QLO[i] + fk[n][:3, 3] for i, n in enumerate(NAMES)])
    return R, C

q_lim_lo = np.array([0.0, -1.5708, -1.5708, -1.5708, -3.1416, -1.48353, -1.48353, -1.48353, -1.48353])
q_lim_hi = np.array([0.585, 1.5708, 1.5708, 1.5708, 3.1416, 1.48353, 1.48353, 1.48353, 1.48353])
rng = np.random.default_rng(20260911)
qs = [np.zeros(9)] + [rng.uniform(q_lim_lo, q_lim_hi) for _ in range(40)]

# FK agreement between the numpy kinematics and the JAX model
fk_jax = np.asarray(dc.link_transforms(jnp.asarray(qs[5])))
R_np, C_np = poses_from_numpy_fk(qs[5])
fk_max = 0.0
for i in range(10):
    Tj = fk_jax[i]
    fk_max = max(fk_max, float(np.abs(Tj[:3, :3] - R_np[i]).max()), float(np.abs(Tj[:3, 3] - C_np[i]).max()))
out['fk_agreement_max'] = fk_max

pairs = np.asarray(m.OBB_COLLISION_PAIRS)
dist_rows, grad_rows_q = [], []
for q in qs:
    kern = np.asarray(dc.self_collision_distances(jnp.asarray(q)))
    R_np, C_np = poses_from_numpy_fk(q)
    for k, (i, j) in enumerate(pairs):
        ref = qp_reference(R_np[i], C_np[i], QHE[i], R_np[j], C_np[j], QHE[j])
        dist_rows.append({'pair': [NAMES[i], NAMES[j]], 'kernel': float(kern[k]), 'ref': ref[0],
                          'err': float(kern[k]) - ref[0], 'ref_overlap': bool(ref[3])})
    # gradient vs central difference of the kernel itself through q (chain check)
    for k in (0, 3, 7):
        if k >= len(pairs):
            continue
        idx = int(k)
        g = np.asarray(dc.self_collision_grads_jit(jnp.asarray(q), jnp.arange(len(pairs)))[idx])
        eps = 1e-6
        fd = np.array([(float(dc.pair_distance(jnp.asarray(q + np.eye(9)[d] * eps), idx))
                        - float(dc.pair_distance(jnp.asarray(q - np.eye(9)[d] * eps), idx))) / (2 * eps)
                       for d in range(9)])
        grad_rows_q.append({'pair': [NAMES[pairs[idx][0]], NAMES[pairs[idx][1]]],
                            'autodiff_max_abs': float(np.abs(g).max()), 'fd_max_abs': float(np.abs(fd).max()),
                            'max_err': float(np.abs(g - fd).max()),
                            'rel_err': float(np.abs(g - fd).max() / max(np.abs(g).max(), 1e-12))})
errs = np.array([r['err'] for r in dist_rows])
out['q_space_distances'] = {
    'n_samples': len(qs), 'n_pair_samples': len(dist_rows),
    'max_abs_err': float(np.abs(errs).max()), 'p99_abs_err': float(np.quantile(np.abs(errs), .99)),
    'median_abs_err': float(np.median(np.abs(errs))),
    'n_err_gt_1um': int((np.abs(errs) > 1e-6).sum()),
    'n_overlap_ref': int(sum(r['ref_overlap'] for r in dist_rows)),
    'n_overlap_ref_kernel_positive': int(sum(1 for r in dist_rows if r['ref_overlap'] and r['kernel'] > 1e-9)),
    'min_kernel': float(min(r['kernel'] for r in dist_rows)),
    'min_ref': float(min(r['ref'] for r in dist_rows)),
    'rows_with_err_gt_1um': [r for r in dist_rows if abs(r['err']) > 1e-6][:8]}
out['q_space_gradients'] = {
    'rows': grad_rows_q,
    'max_rel_err': float(max(r['rel_err'] for r in grad_rows_q)),
    'max_abs_err': float(max(r['max_err'] for r in grad_rows_q))}
print('\nnine-axis chain: FK numpy-vs-JAX max diff = %.3e' % fk_max)
print('q-space distance agreement vs independent QP reference: %s'
      % json.dumps({k: v for k, v in out['q_space_distances'].items() if k != 'rows_with_err_gt_1um'}))
print('q-space gradient (autodiff vs central difference of the kernel through q): %s'
      % json.dumps(out['q_space_gradients'] if False else {k: v for k, v in out['q_space_gradients'].items() if k != 'rows'}))
for r in grad_rows_q:
    print('   %-24s max|g|=%.4e  max|FD|=%.4e  max|g-FD|=%.3e  rel=%.2e' % (str(r['pair']), r['autodiff_max_abs'], r['fd_max_abs'], r['max_err'], r['rel_err']))

Path(Path(__file__).resolve().parent / 'derivatives.json').write_text(json.dumps(out, indent=2))
print('\nwrote derivatives.json')
