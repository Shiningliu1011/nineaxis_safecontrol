import sys, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jp
from work import dpax_collision as dc

I = jp.eye(4); zero = jp.zeros(3); R = jp.eye(3)
t = np.array([3.1, .17, .23]); a = np.array([1., .7, .6]); b = np.array([.4, .3, .2])

def split(t, a, b):
    ta = jp.asarray(t); aa = jp.asarray(a); bb = jp.asarray(b)
    ci = dc._obb_corners_world(I, zero, R, aa)
    cj = dc._obb_corners_world(I, ta, R, bb)
    return float(dc._edge_edge_minimum(ci, cj)), float(dc._point_face_minimum(I, zero, R, aa, I, ta, R, bb))

print('eager  impl split      :', split(t, a, b))
print('eager  custom_jvp     :', float(dc._obb_pair_distance(I, zero, R, jp.asarray(a), I, jp.asarray(t), R, jp.asarray(b))))

fn = jax.jit(lambda tt, aa, bb: dc._obb_pair_distance(I, zero, R, aa, I, tt, R, bb))
print('jitted custom_jvp     :', float(fn(t, a, b)))

fn_impl = jax.jit(dc._obb_pair_distance_impl)
print('jitted impl           :', float(fn_impl(I, zero, R, jp.asarray(a), I, jp.asarray(t), R, jp.asarray(b))))

# does jit change the edge term?
edge_jit = jax.jit(lambda tt, aa, bb: dc._edge_edge_minimum(
    dc._obb_corners_world(I, zero, R, aa), dc._obb_corners_world(I, tt, R, bb)))
face_jit = jax.jit(lambda tt, aa, bb: dc._point_face_minimum(I, zero, R, aa, I, tt, R, bb))
print('jitted edge / face    :', float(edge_jit(t, a, b)), float(face_jit(t, a, b)))

# vmap path (production uses _pair_distance_vmap)
print('vmap prod path        :', np.asarray(dc.self_collision_distances(jp.zeros(9)))[:3])
