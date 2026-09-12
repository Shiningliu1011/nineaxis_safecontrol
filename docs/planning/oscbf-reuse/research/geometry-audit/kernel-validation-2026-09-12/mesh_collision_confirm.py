"""Independent confirmation of the three real mesh-level collisions.

FCL's BVH collision traversal is one library opinion.  The second, unrelated
method here is a direct triangle-triangle SAT test (11 axes: the two face
normals plus the 9 edge-edge cross products), run on the local triangle
neighbourhood of each reported contact point using only numpy + scipy.

A control configuration where FCL reports no collision is run through the same
code path and must yield zero intersecting triangle pairs.
"""
import sys, json, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(Path(__file__).resolve().parent))
import trimesh
from scipy.spatial import cKDTree
from oracles import NAMES, MESH_DIR, fk_vmap, mesh_collision  # noqa
import jax.numpy as jnp


def tri_tri_sat(T1, T2, chunk=200000):
    """Vectorised triangle-triangle intersection over pairs (P,3,3) x (P,3,3).

    Returns a boolean per pair.  Axes: the two triangle normals plus the nine
    edge-edge cross products (11 axes in total, complete for simplices in R^3).
    A pair survives only if no axis separates it.
    """
    out = np.zeros(len(T1), bool)
    for start in range(0, len(T1), chunk):
        A = T1[start:start + chunk]; B = T2[start:start + chunk]
        e1 = np.stack([A[:, 1] - A[:, 0], A[:, 2] - A[:, 1], A[:, 0] - A[:, 2]], axis=1)
        e2 = np.stack([B[:, 1] - B[:, 0], B[:, 2] - B[:, 1], B[:, 0] - B[:, 2]], axis=1)
        n1 = np.cross(e1[:, 0], e1[:, 1])
        n2 = np.cross(e2[:, 0], e2[:, 1])
        axes = [n1, n2]
        for a in range(3):
            for b in range(3):
                axes.append(np.cross(e1[:, a], e2[:, b]))
        alive = np.ones(len(A), bool)
        for ax in axes:
            nrm = np.linalg.norm(ax, axis=1, keepdims=True)
            good = nrm[:, 0] > 1e-12
            u = ax / np.where(nrm > 1e-12, nrm, 1.0)
            p1 = np.einsum('pij,pj->pi', A, u)
            p2 = np.einsum('pij,pj->pi', B, u)
            separated = (p1.max(axis=1) < p2.min(axis=1) - 1e-12) | (p2.max(axis=1) < p1.min(axis=1) - 1e-12)
            alive &= ~(good & separated)
            if not alive.any():
                break
        out[start:start + chunk] = alive
        if not out.any() and start + chunk >= len(T1):
            break
    return out


def _selftest():
    """A clearly crossing pair must intersect; a far pair must not."""
    t1 = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    t2 = np.array([[[0.2, 0.2, -1.], [0.2, 0.2, 1.], [0.9, 0.2, 0.]]])   # pierces t1
    t3 = np.array([[[0., 0., 5.], [1., 0., 5.], [0., 1., 5.]]])           # far away
    ok = bool(tri_tri_sat(t1, t2)[0]) and not bool(tri_tri_sat(t1, t3)[0])
    print('SELFTEST triangle-triangle SAT (crossing=1, far=0): %s' % ('PASS' if ok else 'FAIL'))
    return ok


def local_triangles(mesh, world_T, points, radius, max_tris=2000):
    """Triangles whose centroid lies within `radius` of any query point (capped)."""
    verts = np.asarray(mesh.vertices, float) @ world_T[:3, :3].T + world_T[:3, 3]
    tris = verts[np.asarray(mesh.faces)]
    cent = tris.mean(axis=1)
    tree = cKDTree(cent)
    idx = set()
    for p in points:
        idx.update(tree.query_ball_point(p, radius))
    idx = np.asarray(sorted(idx), int)
    if len(idx) > max_tris:
        d = np.linalg.norm(cent[idx] - np.asarray(points[0], float), axis=1)
        idx = idx[np.argsort(d)[:max_tris]]
    return tris[idx], idx


_selftest()
audit = json.loads((Path(__file__).resolve().parent / 'exclusion_audit.json').read_text())
meshes = {n: trimesh.load_mesh(MESH_DIR / f'{n}.STL') for n in NAMES}
out = {'cases': [], 'control': None}


def check(pair, q, label):
    i, j = NAMES.index(pair[0]), NAMES.index(pair[1])
    T = np.asarray(fk_vmap(jnp.asarray(q, float)[None, :]))[0]
    coll, dist, contacts = mesh_collision(i, j, T[i], T[j], max_contacts=8)
    # query points: contact positions, or the mesh-mesh midpoint region for the control
    pts = [np.asarray(c['pos'], float) for c in contacts] if contacts else []
    if not pts:
        pi = np.asarray(meshes[pair[0]].vertices, float) @ T[i][:3, :3].T + T[i][:3, 3]
        pj = np.asarray(meshes[pair[1]].vertices, float) @ T[j][:3, :3].T + T[j][:3, 3]
        pts = [pi.mean(axis=0), pj.mean(axis=0)]
    radius = 0.05 if coll else 0.08
    ti, _ = local_triangles(meshes[pair[0]], T[i], pts, radius)
    tj, _ = local_triangles(meshes[pair[1]], T[j], pts, radius)
    hits = 0
    example = None
    n_pairs = len(ti) * len(tj)
    for s0 in range(0, len(ti), 200):
        A0 = ti[s0:s0 + 200]
        A = np.repeat(A0, len(tj), axis=0)
        B = np.tile(tj, (len(A0), 1, 1))
        m = tri_tri_sat(A, B)
        hits += int(m.sum())
        if hits and example is None:
            k = int(np.nonzero(m)[0][0])
            example = {'tri_i': A[k].tolist(), 'tri_j': B[k].tolist()}
    rec = {'pair': pair, 'label': label, 'fcl_colliding': bool(coll),
           'n_contacts': len(contacts), 'triangles_tested': int(n_pairs),
           'independent_tri_tri_intersections': hits, 'example': example}
    print('%-16s %-9s fcl=%s  tested=%d  independent tri-tri hits=%d'
          % ('/'.join(pair), label, coll, rec['triangles_tested'], hits))
    return rec


for d in audit['mesh_collision_detail']:
    out['cases'].append(check(d['pair'], d['q'], 'cfg=%d' % d['cfg']))
# control: the same pair at the zero configuration, where FCL reports no contact
out['control'] = check(['base_link', 'Link9'], np.zeros(9), 'zero-config control')
out['control_expected_no_collision'] = True

Path(Path(__file__).resolve().parent / 'mesh_collision_confirm.json').write_text(json.dumps(out, indent=2))
print('\nwrote mesh_collision_confirm.json')
