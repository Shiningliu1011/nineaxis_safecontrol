import sys, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc

I = jnp.eye(4); zero = jnp.zeros(3); R = jnp.eye(3)
t = np.array([3.1, .17, .23]); a = np.array([1., .7, .6]); b = np.array([.4, .3, .2])
ta = jnp.asarray(t); aa = jnp.asarray(a); bb = jnp.asarray(b)

def cmp(name, f, *args):
    e = f(*args); j = jax.jit(f)(*args)
    es = jax.tree_util.tree_map(np.asarray, e); js = jax.tree_util.tree_map(np.asarray, j)
    d = max(float(np.abs(x - y).max()) for x, y in zip(jax.tree_util.tree_leaves(es), jax.tree_util.tree_leaves(js)))
    print(f'{name:38s} max|eager-jit| = {d:.3e}')
    return e, j

cmp('corners_world(I,zero,R,a)', lambda A: dc._obb_corners_world(I, zero, R, A), aa)
cmp('corners_world(I,t,R,b)',   lambda A: dc._obb_corners_world(I, A, R, bb), ta)
cmp('faces_world(I,t,R,b)',     lambda A: dc._obb_faces_world(I, A, R, bb), ta)
cmp('faces_world(I,zero,R,a)',  lambda A: dc._obb_faces_world(I, zero, R, A), aa)
cmp('point_face_distances',     lambda c, f: dc._point_face_distances(c, f),
    dc._obb_corners_world(I, zero, R, aa), dc._obb_faces_world(I, ta, R, bb))

# same but transform passed as an argument (not a closure constant)
cmp('corners_world(TR,zero,R,a) arg-I', lambda TR, A: dc._obb_corners_world(TR, zero, R, A), I, aa)
cmp('faces_world(TR,t,R,b) arg-I',      lambda TR, A: dc._obb_faces_world(TR, A, R, bb), I, ta)

# whole pair distance limit
cmp('pair_impl', lambda tt, A, B: dc._obb_pair_distance_impl(I, zero, R, A, I, tt, R, B), ta, aa, bb)
print('jax', jax.__version__, 'x64', jax.config.jax_enable_x64, 'backend', jax.default_backend())
