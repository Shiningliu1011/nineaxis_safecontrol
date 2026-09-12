"""Exclusion-pair / coverage audit for the M2 OBB self-collision model.

Oracles are imported from oracles.py (SAT on the OBBs, FCL BVH on the STL meshes)
and refs.py (SLSQP QP, GJK); none of them is the production kernel.

Phases
  1  per-pair SAT collision statistics over uniform in-limits samples
  2  kernel blind spot: SAT-colliding configs where the kernel reports clearance
  3  mesh-level FCL truth at *every* SAT-colliding event (OBB collision is a
     necessary condition for mesh collision, so this is complete on the samples)
  4  kernel-gradient descent for a per-pair minimum-clearance estimate
"""
import sys, json, time, warnings, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc
from refs import qp_reference, fcl_reference, gjk_reference
from oracles import (NAMES, QLO, QRO, QHE, DECLARED, NONADJ, Q_LO, Q_HI,
                     fk_vmap, obb_poses, sat, kernel_pairs, mesh_collision)
warnings.filterwarnings('ignore', category=RuntimeWarning, module='scipy.*')

out = {'joint_limits': {'lower': Q_LO.tolist(), 'upper': Q_HI.tolist()},
       'declared_pairs': [[NAMES[i], NAMES[j]] for i, j in DECLARED],
       'omitted_nonadjacent_pairs': [[NAMES[i], NAMES[j]] for i, j in NONADJ
                                     if (i, j) not in DECLARED]}


def obb_poses(T):
    """T (N,11,4,4) -> world OBB rotations (N,10,3,3) and centres (N,10,3)."""
    Tl = T[:, :10]
    R = np.einsum('nlij,ljk->nlik', Tl[:, :, :3, :3], QRO)
    C = np.einsum('nlij,lj->nli', Tl[:, :, :3, :3], QLO) + Tl[:, :, :3, 3]
    return R, C


def sat(RA, hA, cA, RB, hB, cB):
    """Vectorised SAT. RA/RB (N,3,3) column-axes, cA/cB (N,3), hA/hB (3,).

    Returns (colliding (N,), penetration_depth (N,), lower_bound (N,)) where the
    lower bound is max_a gap_a over unit axes, a valid lower bound on distance
    when the boxes are separated (0 when colliding).
    """
    d = cB - cA
    cand = [RA[:, :, a] for a in range(3)] + [RB[:, :, a] for a in range(3)]
    for a in range(3):
        for b in range(3):
            cand.append(np.cross(RA[:, :, a], RB[:, :, b]))
    ovs = []
    for ax in cand:
        nrm = np.linalg.norm(ax, axis=1)
        ok = nrm > 1e-8
        axn = ax / np.where(nrm > 1e-12, nrm, 1.0)[:, None]
        pA = np.sum(hA[None, :] * np.abs(np.einsum('ni,nij->nj', axn, RA)), axis=1)
        pB = np.sum(hB[None, :] * np.abs(np.einsum('ni,nij->nj', axn, RB)), axis=1)
        o = pA + pB - np.abs(np.einsum('ni,ni->n', axn, d))
        ovs.append(np.where(ok, o, np.inf))
    ov = np.stack(ovs, axis=1)
    colliding = np.all(ov > 0.0, axis=1)
    depth = np.min(ov, axis=1)
    lb = np.where(colliding, 0.0, np.max(np.where(np.isfinite(ov), -ov, 0.0), axis=1))
    return colliding, depth, lb


def kernel_pairs(Q, pairs):
    """Production kernel distances for arbitrary pairs, batched: (N,P)."""
    Q = np.asarray(Q, float); n = Q.shape[0]; p = len(pairs)
    T = np.asarray(fk_vmap(jnp.asarray(Q)))  # (N,11,4,4)
    idx_i = np.array([q[0] for q in pairs]); idx_j = np.array([q[1] for q in pairs])
    Ti = np.broadcast_to(T[:, idx_i], (n, p, 4, 4)).reshape(n * p, 4, 4)
    Tj = np.broadcast_to(T[:, idx_j], (n, p, 4, 4)).reshape(n * p, 4, 4)
    ci = np.broadcast_to(QLO[idx_i], (n, p, 3)).reshape(n * p, 3)
    cj = np.broadcast_to(QLO[idx_j], (n, p, 3)).reshape(n * p, 3)
    ri = np.broadcast_to(QRO[idx_i], (n, p, 3, 3)).reshape(n * p, 3, 3)
    rj = np.broadcast_to(QRO[idx_j], (n, p, 3, 3)).reshape(n * p, 3, 3)
    hi = np.broadcast_to(QHE[idx_i], (n, p, 3)).reshape(n * p, 3)
    hj = np.broadcast_to(QHE[idx_j], (n, p, 3)).reshape(n * p, 3)
    d = dc._pair_distance_vmap(jnp.asarray(Ti), jnp.asarray(ci), jnp.asarray(ri), jnp.asarray(hi),
                               jnp.asarray(Tj), jnp.asarray(cj), jnp.asarray(rj), jnp.asarray(hj))
    return np.asarray(d).reshape(n, p)


# ---------------------------------------------------------------- mesh oracle
_meshes = {}
def bvh(name):
    if name not in _meshes:
        msh = trimesh.load_mesh(ROOT / f'models/ninezzhou/meshes/{name}.STL')
        b = fcl.BVHModel()
        b.beginModel(len(msh.vertices), len(msh.faces))
        b.addSubModel(np.asarray(msh.vertices, float), np.asarray(msh.faces, np.int32))
        b.endModel()
        _meshes[name] = b
    return _meshes[name]


    """True mesh-mesh state at link transforms Ti, Tj -> (colliding, distance)."""
    o1 = fcl.CollisionObject(bvh(NAMES[i]), fcl.Transform(Ti[:3, :3], Ti[:3, 3]))
    o2 = fcl.CollisionObject(bvh(NAMES[j]), fcl.Transform(Tj[:3, :3], Tj[:3, 3]))
    cr = fcl.CollisionResult()
    fcl.collide(o1, o2, fcl.CollisionRequest(), cr)
    if cr.is_collision:
        return True, 0.0
    dr = fcl.DistanceResult()
    d = float(fcl.distance(o1, o2, fcl.DistanceRequest(), dr))
    return False, d


# ------------------------------------------------------- 1. SAT sample sweep
N = 3000
rng = np.random.default_rng(20260912)
Q = rng.uniform(Q_LO, Q_HI, size=(N, 9))
Q = np.vstack([np.zeros(9), Q])
T = np.asarray(fk_vmap(jnp.asarray(Q)))
R, C = obb_poses(T)
t0 = time.time()
sat_coll = np.zeros((len(Q), len(NONADJ)), bool)
sat_depth = np.zeros((len(Q), len(NONADJ)))
sat_lb = np.zeros((len(Q), len(NONADJ)))
for k, (i, j) in enumerate(NONADJ):
    c, dep, lb = sat(R[:, i], QHE[i], C[:, i], R[:, j], QHE[j], C[:, j])
    sat_coll[:, k] = c; sat_depth[:, k] = dep; sat_lb[:, k] = lb
print('SAT sweep over %d configs x %d non-adjacent pairs (%.1fs)' % (len(Q), len(NONADJ), time.time() - t0))

rows = []
for k, (i, j) in enumerate(NONADJ):
    n_coll = int(sat_coll[:, k].sum())
    sep = ~sat_coll[:, k]
    rows.append({
        'pair': [NAMES[i], NAMES[j]], 'declared': (i, j) in DECLARED,
        'sat_collision_configs': n_coll,
        'sat_collision_rate': n_coll / len(Q),
        'max_sat_penetration_m': float(sat_depth[:, k].max()) if n_coll else 0.0,
        'min_sat_lower_bound_over_separated_m': float(sat_lb[sep, k].min()) if sep.any() else None,
        'sat_lower_bound_p05_m': float(np.quantile(sat_lb[sep, k], .05)) if sep.any() else None,
    })
out['sat_per_pair'] = rows
decl = [r for r in rows if r['declared']]
omt = [r for r in rows if not r['declared']]
print('\n%-22s %9s %9s  %s' % ('omitted non-adjacent pair', 'SAT coll', 'max depth', 'min SAT lower bound (sep)'))
for r in sorted(omt, key=lambda r: -r['sat_collision_configs']):
    lbv = r['min_sat_lower_bound_over_separated_m']
    print('%-22s %9d %9.5f  %s' % ('/'.join(r['pair']), r['sat_collision_configs'], r['max_sat_penetration_m'],
                                   ('%.6f' % lbv) if lbv is not None else 'n/a'))
print('\n%-22s %9s %9s  %s' % ('declared pair', 'SAT coll', 'max depth', 'min SAT lower bound (sep)'))
for r in sorted(decl, key=lambda r: (r['min_sat_lower_bound_over_separated_m'] if r['min_sat_lower_bound_over_separated_m'] is not None else 1e9)):
    lbv = r['min_sat_lower_bound_over_separated_m']
    print('%-22s %9d %9.5f  %s' % ('/'.join(r['pair']), r['sat_collision_configs'], r['max_sat_penetration_m'],
                                   ('%.6f' % lbv) if lbv is not None else 'n/a'))

# --------------------------------------------- 2. kernel blind spot on samples
coll_any = sat_coll.any(axis=1)
sel = np.where(coll_any)[0]
print('\nconfigs with >=1 SAT collision among the 36 pairs: %d / %d' % (len(sel), len(Q)))
sub = np.concatenate([sel, rng.choice(len(Q), size=min(300, len(Q)), replace=False)])
sub = np.unique(sub)
K = kernel_pairs(Q[sub], NONADJ)
blind = []
for a, cfg in enumerate(sub):
    for k, (i, j) in enumerate(NONADJ):
        if sat_coll[cfg, k] and K[a, k] > 1e-9:
            blind.append({'cfg': int(cfg), 'pair': [NAMES[i], NAMES[j]], 'declared': (i, j) in DECLARED,
                          'kernel_m': float(K[a, k]), 'sat_penetration_m': float(sat_depth[cfg, k])})
print('blind events (SAT colliding, kernel > 1e-9): %d' % len(blind))
per_pair_blind = {}
for b in blind:
    per_pair_blind.setdefault('/'.join(b['pair']), 0)
    per_pair_blind['/'.join(b['pair'])] += 1
print('   per pair:', json.dumps(per_pair_blind, indent=None))
# QP cross-check on a bounded subset of blind events
qp_check = []
for b in blind[:12]:
    cfg = b['cfg']; i, j = NAMES.index(b['pair'][0]), NAMES.index(b['pair'][1])
    q = Q[cfg]
    ref = qp_reference(R[cfg, i], C[cfg, i], QHE[i], R[cfg, j], C[cfg, j], QHE[j])
    gj = gjk_reference(R[cfg, i], C[cfg, i], QHE[i], R[cfg, j], C[cfg, j], QHE[j])
    fl = fcl_reference(R[cfg, i], C[cfg, i], QHE[i], R[cfg, j], C[cfg, j], QHE[j])
    qp_check.append({'pair': b['pair'], 'kernel_m': b['kernel_m'], 'ref_qp_m': ref[0],
                     'ref_qp_overlap': bool(ref[3]), 'ref_gjk_m': gj[0], 'ref_gjk_overlap': gj[1],
                     'ref_fcl_obb_m': fl[0], 'ref_fcl_obb_colliding': fl[1]})
out['blind_events_count'] = len(blind)
out['blind_events_per_pair'] = per_pair_blind
out['blind_events_head'] = blind[:20]
out['blind_qp_crosscheck'] = qp_check
print('\nQP/GJK/FCL cross-check of the first blind events (all oracles on the OBBs):')
for r in qp_check:
    print('   %-22s kernel=%.6e qp=%.3e(ov=%s) gjk=%.3e(ov=%s) fcl_box=%s(coll=%s)' % (
        '/'.join(r['pair']), r['kernel_m'], r['ref_qp_m'], r['ref_qp_overlap'],
        r['ref_gjk_m'], r['ref_gjk_overlap'], r['ref_fcl_obb_m'], r['ref_fcl_obb_colliding']))

# --------------------------- 3. mesh-level truth at EVERY OBB-collision event
# OBB collision is a necessary condition for mesh collision, so scanning exactly
# the SAT-colliding (config, pair) events is a complete mesh-level test on the
# sampled configurations.
ev_cfg, ev_k = np.where(sat_coll)
print('\nSAT-colliding (config, pair) events to mesh-check: %d' % len(ev_cfg))
# kernel value at each event, batched per pair
ev_kernel = np.zeros(len(ev_cfg))
for k in np.unique(ev_k):
    msk = ev_k == k
    ev_kernel[msk] = kernel_pairs(Q[ev_cfg[msk]], [NONADJ[k]])[:, 0]
print('kernel values at events: max %.4f  min %.4g  fraction > 1e-9: %.4f'
      % (ev_kernel.max(), ev_kernel.min(), float((ev_kernel > 1e-9).mean())))
t0 = time.time()
mesh_rows = []
for a, (cfg, k) in enumerate(zip(ev_cfg, ev_k)):
    i, j = NONADJ[k]
    coll, dist, _ = mesh_collision(i, j, T[cfg, i], T[cfg, j])
    mesh_rows.append({'cfg': int(cfg), 'pair': [NAMES[i], NAMES[j]], 'declared': (i, j) in DECLARED,
                      'kernel_m': float(ev_kernel[a]),
                      'sat_penetration_m': float(sat_depth[cfg, k]),
                      'mesh_colliding': bool(coll), 'mesh_distance_m': dist})
print('mesh scan done in %.1fs' % (time.time() - t0))
coll_rows = [r for r in mesh_rows if r['mesh_colliding']]
print('real mesh-level collisions among them: %d' % len(coll_rows))
for r in coll_rows[:20]:
    print('   COLLISION cfg=%d %-22s mesh_d=%.6f declared=%s' % (r['cfg'], '/'.join(r['pair']), r['mesh_distance_m'], r['declared']))
nz = [r for r in mesh_rows if not r['mesh_colliding']]
if nz:
    print('min non-colliding mesh clearance among those events: %.6f m' % min(r['mesh_distance_m'] for r in nz))
out['mesh_scan'] = {'n_events': len(mesh_rows), 'n_mesh_collisions': len(coll_rows),
                    'mesh_collisions': coll_rows[:40],
                    'min_mesh_clearance_at_noncolliding_events_m': min([r['mesh_distance_m'] for r in nz], default=None)}

# detail for every real mesh collision: contacts, joint values, kernel reading
detail = []
print('\ndetail of real mesh collisions:')
for r in coll_rows:
    cfg = r['cfg']; i, j = NAMES.index(r['pair'][0]), NAMES.index(r['pair'][1])
    q = Q[cfg]
    d = {'cfg': cfg, 'pair': r['pair'], 'q': q.tolist(),
         'kernel_m': r['kernel_m'], 'sat_penetration_m': r['sat_penetration_m'],
         'joints_at_lower_limit': [int(t) for t in np.where(np.abs(q - Q_LO) < 1e-9)[0]],
         'joints_at_upper_limit': [int(t) for t in np.where(np.abs(q - Q_HI) < 1e-9)[0]],
         'contacts': mesh_collision(i, j, T[cfg, i], T[cfg, j])[2]}
    ref = qp_reference(R[cfg, i], C[cfg, i], QHE[i], R[cfg, j], C[cfg, j], QHE[j])
    d['ref_qp_m'] = ref[0]; d['ref_qp_overlap'] = bool(ref[3])
    d['max_mesh_penetration_m'] = max([c['depth'] for c in d['contacts']], default=0.0)
    print('   cfg=%d %-16s kernel=%.6e  qp=%.3e  sat_depth=%.5f  n_contacts=%d  max_pen=%.5f'
          % (cfg, '/'.join(r['pair']), r['kernel_m'], ref[0], r['sat_penetration_m'],
             len(d['contacts']), d['max_mesh_penetration_m']))
    print('      q =', np.array2string(q, precision=4))
    detail.append(d)
out['mesh_collision_detail'] = detail

# the specific in-limits config from the q-space derivative battery (q index 10),
# where base_link collides with Link7 and Link8 at OBB level
vq_path = Path(__file__).resolve().parent / 'verify_qcase.json'
if vq_path.exists():
    vq = json.loads(vq_path.read_text())
    focus = []
    for row in vq['rows']:
        i, j = NAMES.index(row['pair'][0]), NAMES.index(row['pair'][1])
        qv = np.asarray(row['q'], float)
        Tf = np.asarray(fk_vmap(jnp.asarray(qv)[None, :]))[0]
        coll, dist, _ = mesh_collision(i, j, Tf[i], Tf[j])
        focus.append({'pair': row['pair'], 'q': row['q'], 'kernel_m': row['kernel'],
                      'ref_gjk_overlap': row['ref_gjk_overlap'], 'ref_fcl_obb_colliding': row['ref_fcl_colliding'],
                      'mesh_colliding': bool(coll), 'mesh_distance_m': dist})
        print('   focus q10 %-22s OBB overlap -> mesh_colliding=%s mesh_d=%.6f' % ('/'.join(row['pair']), coll, dist))
    out['focus_config_mesh_truth'] = focus

# -------------------------------- 4. per-pair minimum clearance (kernel descent)
print('\nper-pair clearance refinement (kernel-gradient descent, QP-verified):')

def _clearance(qq, i, j):
    T = dc.link_transforms(qq)
    return dc._obb_pair_distance(T[i], QLO[i], QRO[i], QHE[i],
                                 T[j], QLO[j], QRO[j], QHE[j])

_clr = jax.jit(_clearance, static_argnums=(1, 2))
_clr_grad = jax.jit(jax.grad(_clearance, argnums=0), static_argnums=(1, 2))
all_pairs = DECLARED + [p for p in NONADJ if p not in DECLARED]
desc_rows = []
for (i, j) in all_pairs:
    best = None
    for t in range(6):
        q0 = np.zeros(9) if t == 0 else np.random.default_rng(1000 + t).uniform(Q_LO, Q_HI)
        q = np.clip(q0, Q_LO, Q_HI)
        step = 0.05
        for _ in range(300):
            g = np.asarray(_clr_grad(jnp.asarray(q), i, j))
            gn = np.linalg.norm(g)
            if gn < 1e-6:
                break
            improved = False
            for _bt in range(12):
                qn = np.clip(q - step * g / max(gn, 1e-12), Q_LO, Q_HI)
                if float(_clr(jnp.asarray(qn), i, j)) < float(_clr(jnp.asarray(q), i, j)) - 1e-14:
                    q = qn; improved = True; break
                step *= 0.5
            if not improved:
                step = min(step * 2.0, 0.5)
                if step <= 1e-6:
                    break
        dval = float(_clr(jnp.asarray(q), i, j))
        if best is None or dval < best[0]:
            best = (dval, q.copy())
    dval, q = best
    Tq = np.asarray(fk_vmap(jnp.asarray(q)[None, :]))
    Rq, Cq = obb_poses(Tq)
    ref = qp_reference(Rq[0, i], Cq[0, i], QHE[i], Rq[0, j], Cq[0, j], QHE[j])
    gj = gjk_reference(Rq[0, i], Cq[0, i], QHE[i], Rq[0, j], Cq[0, j], QHE[j])
    satc, _, _ = sat(Rq[:, i], QHE[i], Cq[:, i], Rq[:, j], QHE[j], Cq[:, j])
    mcoll, mdist, _ = mesh_collision(i, j, Tq[0, i], Tq[0, j])
    desc_rows.append({'pair': [NAMES[i], NAMES[j]], 'declared': (i, j) in DECLARED,
                      'kernel_min_m': dval, 'ref_qp_m': ref[0], 'ref_qp_overlap': bool(ref[3]),
                      'ref_gjk_m': gj[0], 'ref_gjk_overlap': gj[1],
                      'sat_colliding': bool(satc[0]), 'mesh_colliding': bool(mcoll),
                      'mesh_distance_m': mdist, 'q': q.tolist()})
    print('   %-22s %-9s kernel=%9.6f qp=%9.6f gjk=%9.6f sat_coll=%-5s mesh_coll=%-5s mesh_d=%.4f' % (
        '/'.join(desc_rows[-1]['pair']), 'declared' if desc_rows[-1]['declared'] else '',
        dval, ref[0], gj[0], str(bool(satc[0])), str(mcoll), mdist))
out['pair_minimum_clearance'] = desc_rows

Path(Path(__file__).resolve().parent / 'exclusion_audit.json').write_text(json.dumps(out, indent=2))
print('\nwrote exclusion_audit.json')
