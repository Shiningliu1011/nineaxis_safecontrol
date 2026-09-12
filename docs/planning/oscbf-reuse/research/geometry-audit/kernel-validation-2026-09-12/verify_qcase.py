"""Re-derive the two q-space rows where the kernel disagreed with the QP reference.

Row A: base_link/Link7, kernel 0.0166, ref overlap  (overlap blindness on a real pair)
Row B: base_link/Link8, kernel 2.24e-4, ref 7.2e-9 (0.22 mm overestimate, or a bad reference?)

Also fixes the FK-agreement assertion (previous run compared the link origin
against the OBB centre).
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
from refs import qp_reference, fcl_reference, gjk_reference
from scipy.optimize import minimize
warnings.filterwarnings('ignore', category=RuntimeWarning, module='scipy.*')

kin = NineaxisKinematics()
NAMES = list(m.OBB_LINK_NAMES)
QLO = np.asarray(m.OBB_LOCAL_CENTERS_M); QRO = np.asarray(m.OBB_LOCAL_ROTATIONS); QHE = np.asarray(m.OBB_HALF_EXTENTS_M)

def poses(q):
    fk = kin.forward_kinematics(np.asarray(q, float))
    R = np.stack([fk[n][:3, :3] @ QRO[i] for i, n in enumerate(NAMES)])
    C = np.stack([fk[n][:3, :3] @ QLO[i] + fk[n][:3, 3] for i, n in enumerate(NAMES)])
    return R, C

# same sampling as derivatives.py
q_lo = np.array([0.0, -1.5708, -1.5708, -1.5708, -3.1416, -1.48353, -1.48353, -1.48353, -1.48353])
q_hi = np.array([0.585, 1.5708, 1.5708, 1.5708, 3.1416, 1.48353, 1.48353, 1.48353, 1.48353])
rng = np.random.default_rng(20260911)
qs = [np.zeros(9)] + [rng.uniform(q_lo, q_hi) for _ in range(40)]

pairs = np.asarray(m.OBB_COLLISION_PAIRS)

# ---- FK agreement, done properly: link transform vs link transform -------
def fk_max_diff(q):
    T_jax = np.asarray(dc.link_transforms(jnp.asarray(q)))
    fk = kin.forward_kinematics(np.asarray(q, float))
    worst = 0.0; where = None
    for i, n in enumerate(NAMES):
        Tn = np.asarray(fk[n], float)
        d = max(float(np.abs(T_jax[i][:3, :3] - Tn[:3, :3]).max()),
                float(np.abs(T_jax[i][:3, 3] - Tn[:3, 3]).max()))
        if d > worst:
            worst, where = d, n
    return worst, where

print('FK agreement (link transform vs link transform, fixed check):')
worst = 0.0
for q in qs[:6]:
    d, w = fk_max_diff(q)
    worst = max(worst, d)
print('   max over 6 sampled q: %.3e' % worst)

# ---- locate the two rows -------------------------------------------------
bad = []
for qi, q in enumerate(qs):
    kern = np.asarray(dc.self_collision_distances(jnp.asarray(q)))
    R, C = poses(q)
    for k, (i, j) in enumerate(pairs):
        ref = qp_reference(R[i], C[i], QHE[i], R[j], C[j], QHE[j])
        if abs(float(kern[k]) - ref[0]) > 1e-6:
            bad.append((qi, k, int(i), int(j), float(kern[k]), ref, R, C))
print('\nrows with |kernel - ref_qp| > 1um: %d' % len(bad))

out = {'fk_agreement_max': worst, 'rows': []}
for qi, k, i, j, kern, ref, R, C in bad:
    q = qs[qi]
    print('\n--- q index %d, pair %s/%s ---' % (qi, NAMES[i], NAMES[j]))
    print('   q =', np.array2string(q, precision=6))
    print('   kernel      = %.9e' % kern)
    print('   ref_qp      = %.9e  (overlap flag %s)' % (ref[0], ref[3]))
    gj = gjk_reference(R[i], C[i], QHE[i], R[j], C[j], QHE[j])
    fl = fcl_reference(R[i], C[i], QHE[i], R[j], C[j], QHE[j])
    print('   ref_gjk     = %.9e  overlapping=%s' % (gj[0], gj[1]))
    print('   ref_fcl     = %.9e  colliding=%s' % (fl[0], fl[1]))
    # QP with several starts: is the near-zero value a solver artefact?
    vals = []
    for seed in range(5):
        r2 = np.random.default_rng(seed)
        z0 = np.clip(r2.uniform(-1, 1, 6) * np.concatenate([QHE[i], QHE[j]]), -np.concatenate([QHE[i], QHE[j]]), np.concatenate([QHE[i], QHE[j]]))
        def f(z):
            d = (R[i] @ z[:3] + C[i]) - (R[j] @ z[3:] + C[j]); return float(d @ d)
        def g(z):
            d = (R[i] @ z[:3] + C[i]) - (R[j] @ z[3:] + C[j])
            return np.concatenate([2 * R[i].T @ d, -2 * R[j].T @ d])
        res = minimize(f, z0, jac=g, method='SLSQP',
                       bounds=[(-QHE[i][t], QHE[i][t]) for t in range(3)] + [(-QHE[j][t], QHE[j][t]) for t in range(3)],
                       options={'ftol': 1e-16, 'maxiter': 800})
        vals.append(float(np.sqrt(max(res.fun, 0.0))))
    print('   ref_qp multi-start values =', ['%.6e' % v for v in vals])
    # local FD of both sides in q (does each side look smooth here?)
    eps = 1e-6
    fd_kern, fd_ref = [], []
    for d in range(9):
        kp = float(dc.pair_distance(jnp.asarray(q + np.eye(9)[d] * eps), k))
        km = float(dc.pair_distance(jnp.asarray(q - np.eye(9)[d] * eps), k))
        fd_kern.append((kp - km) / (2 * eps))
        Rp, Cp = poses(q + np.eye(9)[d] * eps); Rm, Cm = poses(q - np.eye(9)[d] * eps)
        a = qp_reference(Rp[i], Cp[i], QHE[i], Rp[j], Cp[j], QHE[j])[0]
        b = qp_reference(Rm[i], Cm[i], QHE[i], Rm[j], Cm[j], QHE[j])[0]
        fd_ref.append((a - b) / (2 * eps))
    print('   |FD kernel| max = %.4e   |FD ref_qp| max = %.4e' % (np.abs(fd_kern).max(), np.abs(fd_ref).max()))
    out['rows'].append({'q_index': qi, 'pair': [NAMES[i], NAMES[j]], 'q': q.tolist(),
                        'kernel': kern, 'ref_qp': ref[0], 'ref_qp_overlap': bool(ref[3]),
                        'ref_gjk': gj[0], 'ref_gjk_overlap': gj[1],
                        'ref_fcl': fl[0], 'ref_fcl_colliding': fl[1],
                        'ref_qp_multistart': vals,
                        'fd_kernel_max': float(np.abs(fd_kern).max()),
                        'fd_refqp_max': float(np.abs(fd_ref).max())})

Path(Path(__file__).resolve().parent / 'verify_qcase.json').write_text(json.dumps(out, indent=2))
print('\nwrote verify_qcase.json')
