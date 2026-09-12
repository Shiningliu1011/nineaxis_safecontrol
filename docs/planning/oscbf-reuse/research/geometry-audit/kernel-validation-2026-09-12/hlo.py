import sys, numpy as np, re
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc

I = jnp.eye(4); zero = jnp.zeros(3); R = jnp.eye(3)
aa = jnp.asarray(np.array([1., .7, .6]))

f = jax.jit(lambda A: dc._obb_faces_world(I, zero, R, A))
low = f.lower(aa)
hlo = low.as_text()
print('transpose occurrences:', hlo.count('transpose'), '| broadcast:', hlo.count('broadcast'), '| reshape:', hlo.count('reshape'))
# print the region around the first transpose / the add
lines = hlo.splitlines()
for i, ln in enumerate(lines):
    if 'transpose' in ln or 'ROOT' in ln:
        print(f'--- [{i}] {ln.strip()[:220]}')
print()
print('=== eager jaxpr of the same computation ===')
f2 = lambda A: dc._obb_faces_world(I, zero, R, A)
closed = jax.make_jaxpr(f2)(aa)
print(str(closed)[:1800])
