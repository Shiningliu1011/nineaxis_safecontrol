"""Shared independent oracles for the OBB kernel audit (ticket #8).

None of these is the production kernel:
  SAT      : 15-axis separating-axis test on the OBBs -> collision state, plus a
             guaranteed *lower bound* on the true clearance (max unit-axis gap).
  QP/GJK   : see refs.py (SLSQP constrained QP, and support-mapping GJK).
  FCL mesh : triangle-mesh BVH collide/distance on the real STL geometry, the
             only oracle that says whether the *links* touch, as opposed to
             their bounding boxes.
"""
import sys, itertools
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
ROOT = next(p for p in _HERE.parents if (p / 'portable_oscbf/work').is_dir())
for p in (str(ROOT / 'portable_oscbf'), str(_HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

import jax  # noqa: E402
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp  # noqa: E402
import trimesh  # noqa: E402
import fcl  # noqa: E402
from work import dpax_collision as dc  # noqa: E402
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX  # noqa: E402
from work import obb_collision_model as m  # noqa: E402

NAMES = list(m.OBB_LINK_NAMES)
QLO = np.asarray(m.OBB_LOCAL_CENTERS_M, float)
QRO = np.asarray(m.OBB_LOCAL_ROTATIONS, float)
QHE = np.asarray(m.OBB_HALF_EXTENTS_M, float)
DECLARED = [tuple(map(int, p)) for p in np.asarray(m.OBB_COLLISION_PAIRS)]
NONADJ = [(i, j) for i, j in itertools.combinations(range(10), 2) if j - i >= 2]
MESH_DIR = ROOT / 'models/ninezzhou/meshes'

_robot = NineaxisManipulatorJAX()
Q_LO = np.asarray(_robot.joint_lower_limits, float)
Q_HI = np.asarray(_robot.joint_upper_limits, float)
fk_vmap = jax.jit(jax.vmap(dc.link_transforms))


def obb_poses(T):
    """T (N,11,4,4) -> world OBB rotations (N,10,3,3) and centres (N,10,3)."""
    Tl = T[:, :10]
    R = np.einsum('nlij,ljk->nlik', Tl[:, :, :3, :3], QRO)
    C = np.einsum('nlij,lj->nli', Tl[:, :, :3, :3], QLO) + Tl[:, :, :3, 3]
    return R, C


def sat(RA, hA, cA, RB, hB, cB):
    """Vectorised SAT. RA/RB (N,3,3) column axes, cA/cB (N,3), hA/hB (3,).

    Returns (colliding, penetration_depth, lower_bound).  The lower bound is
    max_a gap_a over unit axes and is a valid lower bound on the true distance
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
    """Production kernel distances for arbitrary pairs, batched: (N, P)."""
    Q = np.asarray(Q, float); n = Q.shape[0]; p = len(pairs)
    T = np.asarray(fk_vmap(jnp.asarray(Q)))
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


_meshes = {}


def bvh(name):
    if name not in _meshes:
        msh = trimesh.load_mesh(MESH_DIR / f'{name}.STL')
        b = fcl.BVHModel()
        b.beginModel(len(msh.vertices), len(msh.faces))
        b.addSubModel(np.asarray(msh.vertices, float), np.asarray(msh.faces, np.int32))
        b.endModel()
        _meshes[name] = b
    return _meshes[name]


def mesh_collision(i, j, Ti, Tj, max_contacts=8):
    """FCL BVH mesh-mesh query -> (colliding, distance, contacts)."""
    o1 = fcl.CollisionObject(bvh(NAMES[i]), fcl.Transform(Ti[:3, :3], Ti[:3, 3]))
    o2 = fcl.CollisionObject(bvh(NAMES[j]), fcl.Transform(Tj[:3, :3], Tj[:3, 3]))
    cr = fcl.CollisionResult()
    fcl.collide(o1, o2, fcl.CollisionRequest(num_max_contacts=max_contacts, enable_contact=True), cr)
    contacts = [{'pos': np.asarray(c.pos, float).tolist(),
                 'depth': float(c.penetration_depth),
                 'normal': np.asarray(c.normal, float).tolist()} for c in cr.contacts]
    if cr.is_collision:
        return True, 0.0, contacts
    dr = fcl.DistanceResult()
    return False, float(fcl.distance(o1, o2, fcl.DistanceRequest(), dr)), contacts
