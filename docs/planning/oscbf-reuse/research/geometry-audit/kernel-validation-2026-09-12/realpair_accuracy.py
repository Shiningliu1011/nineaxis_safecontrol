"""Kernel accuracy on real link pairs (not synthetic boxes).

For uniform in-limits configurations:
  - pairs whose OBBs are SAT-separated: compare the production kernel against the
    independent SLSQP QP reference.  This is the accuracy claim that matters for
    the CBF, on the actual nine-axis geometry.
  - pairs whose OBBs SAT-collide: the kernel cannot represent this at all, so it
    is reported separately (see exclusion_audit.py for the mesh-level truth).

Also reports the per-pair QP minimum clearance and the mesh-level minimum
clearance over the sample, both for declared and omitted pairs.
"""
import sys, json, time, warnings, itertools, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import jax.numpy as jnp
from refs import qp_reference, gjk_reference
from oracles import NAMES, QHE, DECLARED, NONADJ, Q_LO, Q_HI, fk_vmap, obb_poses, sat, kernel_pairs, mesh_collision  # noqa
warnings.filterwarnings('ignore', category=RuntimeWarning, module='scipy.*')



N = 60
rng = np.random.default_rng(777)
Q = np.vstack([np.zeros(9), rng.uniform(Q_LO, Q_HI, size=(N - 1, 9))])
T = np.asarray(fk_vmap(jnp.asarray(Q)))
R, C = obb_poses(T)

ALL = list(DECLARED) + [p for p in NONADJ if p not in DECLARED]
K = kernel_pairs(Q, ALL)
print('kernel distances for %d configs x %d pairs computed' % (len(Q), len(ALL)))

rows = []
for k, (i, j) in enumerate(ALL):
    seps, refs, kers, gjk_ok = [], [], [], 0
    for c in range(len(Q)):
        sc, _, _ = sat(R[c:c + 1, i], QHE[i], C[c:c + 1, i], R[c:c + 1, j], QHE[j], C[c:c + 1, j])
        if sc[0]:
            continue
        ref = qp_reference(R[c, i], C[c, i], QHE[i], R[c, j], C[c, j], QHE[j])
        seps.append(ref[0]); kers.append(float(K[c, k]))
    rows.append({'pair': [NAMES[i], NAMES[j]], 'declared': (i, j) in DECLARED,
                 'n_separated_samples': len(seps),
                 'min_qp_clearance_m': float(min(seps)) if seps else None,
                 'min_kernel_m': float(min(kers)) if kers else None,
                 'max_abs_err_m': float(max(abs(a - b) for a, b in zip(kers, seps))) if seps else None,
                 'max_overestimate_m': float(max(a - b for a, b in zip(kers, seps))) if seps else None,
                 'max_underestimate_m': float(min(a - b for a, b in zip(kers, seps))) if seps else None})
allerr = [r['max_abs_err_m'] for r in rows if r['max_abs_err_m'] is not None]
over = [r['max_overestimate_m'] for r in rows if r['max_overestimate_m'] is not None]
print('\n%-22s %-9s %6s %14s %14s %12s' % ('pair', 'declared', 'n_sep', 'min QP clear', 'min kernel', 'max |err|'))
for r in sorted(rows, key=lambda r: (not r['declared'], r['min_qp_clearance_m'] if r['min_qp_clearance_m'] is not None else 9)):
    print('%-22s %-9s %6d %14s %14s %12s' % ('/'.join(r['pair']), 'yes' if r['declared'] else '',
                                             r['n_separated_samples'],
                                             ('%.6f' % r['min_qp_clearance_m']) if r['min_qp_clearance_m'] is not None else 'n/a',
                                             ('%.6f' % r['min_kernel_m']) if r['min_kernel_m'] is not None else 'n/a',
                                             ('%.3e' % r['max_abs_err_m']) if r['max_abs_err_m'] is not None else 'n/a'))

# independent verification at each pair's closest sampled configuration
print('\nverification of the closest sampled configuration per pair (QP vs GJK vs kernel):')
verify = []
for k, (i, j) in enumerate(ALL):
    best = None
    for c in range(len(Q)):
        sc, _, _ = sat(R[c:c + 1, i], QHE[i], C[c:c + 1, i], R[c:c + 1, j], QHE[j], C[c:c + 1, j])
        if sc[0]:
            continue
        ref = qp_reference(R[c, i], C[c, i], QHE[i], R[c, j], C[c, j], QHE[j])
        if best is None or ref[0] < best[0]:
            best = (ref[0], c)
    if best is None:
        verify.append({'pair': [NAMES[i], NAMES[j]], 'declared': (i, j) in DECLARED,
                       'note': 'every sampled configuration had colliding OBBs'})
        continue
    d0, c = best
    gj = gjk_reference(R[c, i], C[c, i], QHE[i], R[c, j], C[c, j], QHE[j])
    coll, mdist = mesh_collision(i, j, T[c, i], T[c, j])[:2]
    verify.append({'pair': [NAMES[i], NAMES[j]], 'declared': (i, j) in DECLARED,
                   'cfg': c, 'qp_m': d0, 'gjk_m': gj[0], 'gjk_overlap': bool(gj[1]),
                   'kernel_m': float(K[c, k]), 'kernel_minus_qp_m': float(K[c, k]) - d0,
                   'mesh_colliding': bool(coll), 'mesh_clearance_m': mdist})
print('%-22s %-9s %12s %12s %12s %12s %12s' % ('pair', 'declared', 'QP', 'GJK', 'kernel', 'kernel-QP', 'mesh clear'))
for r in verify:
    if 'note' in r:
        print('%-22s %-9s %s' % ('/'.join(r['pair']), 'yes' if r['declared'] else '', r['note']))
        continue
    print('%-22s %-9s %12.6f %12.6f %12.6f %12.2e %12.6f' % ('/'.join(r['pair']), 'yes' if r['declared'] else '',
                                                              r['qp_m'], r['gjk_m'], r['kernel_m'],
                                                              r['kernel_minus_qp_m'], r['mesh_clearance_m']))

out = {'n_configs': len(Q), 'seed': 777, 'pairs': rows, 'closest_config_verification': verify,
       'summary': {'pairs_with_separated_samples': len(allerr),
                   'worst_pair_max_abs_err_m': float(max(allerr)) if allerr else None,
                   'worst_pair_max_overestimate_m': float(max(over)) if over else None}}
Path(Path(__file__).resolve().parent / 'realpair_accuracy.json').write_text(json.dumps(out, indent=2))
print('\nwrote realpair_accuracy.json')
