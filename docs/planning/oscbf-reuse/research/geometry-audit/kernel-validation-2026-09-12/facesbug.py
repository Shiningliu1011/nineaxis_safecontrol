import sys, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc

I = jnp.eye(4); zero = jnp.zeros(3); R = jnp.eye(3)
a = np.array([1., .7, .6]); aa = jnp.asarray(a)

def show(tag, out):
    centers, tangents, widths = out
    print(f'--- {tag}')
    print('  centers\n', np.asarray(centers))
    print('  tangents\n', np.asarray(tangents).reshape(6, 6))
    print('  widths\n', np.asarray(widths))

e = dc._obb_faces_world(I, zero, R, aa)
j = jax.jit(lambda A: dc._obb_faces_world(I, zero, R, A))(aa)
show('eager', e)
show('jit', j)
for nm, x, y in (('centers', e[0], j[0]), ('tangents', e[1], j[1]), ('widths', e[2], j[2])):
    print(nm, 'differing entries:', int((np.asarray(x) != np.asarray(y)).sum()))
print()
print('FACE_NORMAL_AXIS', np.asarray(dc._FACE_NORMAL_AXIS), dc._FACE_NORMAL_AXIS.dtype)
print('FACE_NORMAL_SIGN', np.asarray(dc._FACE_NORMAL_SIGN))
print('FACE_TANGENT_AXES\n', np.asarray(dc._FACE_TANGENT_AXES))
