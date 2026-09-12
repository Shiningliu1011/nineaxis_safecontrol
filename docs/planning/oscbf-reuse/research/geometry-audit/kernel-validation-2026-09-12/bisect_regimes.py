import sys, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc
from refs import qp_reference, fcl_reference
I3 = np.eye(3); Z3 = jnp.zeros(3); I4 = jnp.eye(4)
def rot(axis, angle):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

RA = rot([0, 0, 1], np.pi / 2); hA = np.array([.1, .05, .05])
RB = I3;                        hB = np.array([.02, .02, .02])
t  = np.array([.25, 0, 0]);     tA = np.zeros(3)
ref = qp_reference(RA, tA, hA, RB, t, hB)[0]
print('reference (QP/FCL):', ref, fcl_reference(RA, tA, hA, RB, t, hB)[0])

# one rotation convention: the world rotation is the LOCAL rotation and the world
# centre is the LOCAL centre, so an argument appears exactly once.
args = (I4, jnp.asarray(tA), jnp.asarray(RA), jnp.asarray(hA),
        I4, jnp.asarray(t),  jnp.asarray(RB), jnp.asarray(hB))

def show(tag, v):
    print(f'  {tag:46s} {float(v):.12f}   err_vs_ref={float(v)-ref:+.3e}')

print('\n_obb_pair_distance_impl:')
show('eager', dc._obb_pair_distance_impl(*args))
show('jit', jax.jit(dc._obb_pair_distance_impl)(*args))
show('vmap(eager)', jax.vmap(dc._obb_pair_distance_impl)(
    *(jnp.asarray(a)[None] if i % 2 == 0 or i in (2, 3, 6, 7) else jnp.asarray(a)[None]
      for i, a in enumerate(args)))[0])
show('jit(vmap(impl))', dc._pair_distance_vmap(*(jnp.asarray(a)[None] for a in args))[0])

print('\n_obb_pair_distance (custom_jvp):')
show('eager', dc._obb_pair_distance(*args))
show('jit', jax.jit(dc._obb_pair_distance)(*args))
show('jit(vmap)', jax.jit(jax.vmap(dc._obb_pair_distance, in_axes=(0,)*8))(
    *(jnp.asarray(a)[None] for a in args))[0])

print('\nedge term only:')
def edge_only(Ti, ci, Ri, hi, Tj, cj, Rj, hj):
    return dc._edge_edge_minimum(dc._obb_corners_world(Ti, ci, Ri, hi),
                                 dc._obb_corners_world(Tj, cj, Rj, hj))
show('eager', edge_only(*args))
show('jit', jax.jit(edge_only)(*args))
show('jit(vmap)', jax.jit(jax.vmap(edge_only))(*(jnp.asarray(a)[None] for a in args))[0])

print('\nface term only:')
def face_only(Ti, ci, Ri, hi, Tj, cj, Rj, hj):
    return dc._point_face_minimum(Ti, ci, Ri, hi, Tj, cj, Rj, hj)
show('eager', face_only(*args))
show('jit', jax.jit(face_only)(*args))
show('jit(vmap)', jax.jit(jax.vmap(face_only))(*(jnp.asarray(a)[None] for a in args))[0])
