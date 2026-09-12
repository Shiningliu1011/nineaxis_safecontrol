"""Mechanism probe: what exactly does jit+constant-folding do to the face-centre
offset `(normal_directions * half[FA][None,:]).T`?

Prints the full eager and jitted (6,3) matrices, their difference, and the HLO
op summary for each call shape.  No product code is touched.
"""
import sys, json, re
from pathlib import Path
import numpy as np
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'portable_oscbf/work').is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
from work import dpax_collision as dc

FA = dc._FACE_NORMAL_AXIS
SIGN = dc._FACE_NORMAL_SIGN
h = jnp.array([1., 1., .7, .7, .6, .6])
cen = jnp.array([.01, .02, .03])
I3 = jnp.eye(3)


def body_const_rot(half_, cen_):
    nd = I3[:, FA] * SIGN[None, :]
    return cen_[None, :] + (nd * half_[FA][None, :]).T


def body_traced_rot(rot, half_, cen_):
    nd = rot[:, FA] * SIGN[None, :]
    return cen_[None, :] + (nd * half_[FA][None, :]).T


def show(name, arr, ref):
    a = np.asarray(arr)
    print('  %-26s shape=%s maxdiff-vs-eager=%.3e' % (name, a.shape, np.abs(a - ref).max()))
    return a


out = {}
print('=== identity rotation, constant closed over ===')
eager = np.asarray(body_const_rot(h, cen))
print('eager (6,3):'); print(eager)
jit_const = np.asarray(jax.jit(body_const_rot)(h, cen))
print('jit   (6,3):'); print(jit_const)
show('jit', jit_const, eager)
print('  difference:'); print(jit_const - eager)
out['jit_const'] = jit_const.tolist()
out['eager'] = eager.tolist()

print('\n=== rotation passed as a traced argument ===')
jit_traced = np.asarray(jax.jit(body_traced_rot)(jnp.asarray(I3), h, cen))
show('jit_traced', jit_traced, eager)
out['jit_traced'] = jit_traced.tolist()

print('\n=== eager with a traced rotation (inside an outer jit, q-like) ===')
jt = jax.jit(lambda r: body_traced_rot(r, h, cen))
out['jit_traced_vs_eager_maxdiff'] = float(np.abs(np.asarray(jt(jnp.asarray(I3))) - eager).max())
print('  maxdiff=%.3e' % out['jit_traced_vs_eager_maxdiff'])

print('\n=== is the jitted result a *permutation* of the eager one per column? ===')
for c in range(3):
    col_e = np.sort(np.asarray(eager)[:, c])
    col_j = np.sort(np.asarray(jit_const)[:, c])
    print('  col %d sorted-equal=%s  %s vs %s' % (c, np.allclose(col_e, col_j), np.round(col_e, 3), np.round(col_j, 3)))

print('\n=== FALSIFIABLE HYPOTHESIS ===')
print('  jit result == C-order-reshape(prod, (6,3)) + cen[None,:] ?')
prod3 = np.asarray((I3[:, FA] * SIGN[None, :]) * h[FA][None, :])       # (3,6), C order
recon = prod3.reshape(6, 3) + np.asarray(cen)[None, :]
print('  reconstruction:'); print(recon)
print('  maxdiff vs jit = %.3e   exact = %s' % (np.abs(recon - jit_const).max(),
                                                np.array_equal(recon, jit_const)))
out['reshape_reconstruction_maxdiff'] = float(np.abs(recon - jit_const).max())
out['reshape_reconstruction_exact'] = bool(np.array_equal(recon, jit_const))

print('\n=== does the same reconstruction hold for a non-identity constant rotation? ===')
def rot(axis, ang):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)

for name, Rn in (('rot_z_0.4', rot([0, 0, 1], 0.4)), ('rot_xy_1.1', rot([1, 1, 0], 1.1))):
    Rj = jnp.asarray(Rn)

    def b_const(half_, cen_, _R=Rj):
        nd = _R[:, FA] * SIGN[None, :]
        return cen_[None, :] + (nd * half_[FA][None, :]).T

    jv = np.asarray(jax.jit(b_const)(h, cen))
    p3 = np.asarray((Rn[:, FA] * np.asarray(SIGN)[None, :]) * np.asarray(h)[FA][None, :])
    rec = p3.reshape(6, 3) + np.asarray(cen)[None, :]
    print('  %-10s reshape-reconstruction exact=%s  maxdiff=%.3e' % (
        name, np.array_equal(rec, jv), np.abs(rec - jv).max()))
    out.setdefault('reshape_reconstruction_other_rot', {})[name] = {
        'exact': bool(np.array_equal(rec, jv)), 'maxdiff': float(np.abs(rec - jv).max())}

print('\n=== the same product without the transpose (jit, constant rot) ===')
def body_norot(half_):
    nd = I3[:, FA] * SIGN[None, :]
    return nd * half_[FA][None, :]


e_norot = np.asarray(body_norot(h))
j_norot = np.asarray(jax.jit(body_norot)(h))
print('  eager (3,6):'); print(e_norot)
print('  jit   (3,6):'); print(j_norot)
print('  maxdiff=%.3e' % np.abs(j_norot - e_norot).max())
out['prod_no_transpose_maxdiff'] = float(np.abs(j_norot - e_norot).max())

print('\n=== with the centre term removed (isolate the transpose from the add) ===')
def body_only_T(half_):
    nd = I3[:, FA] * SIGN[None, :]
    return (nd * half_[FA][None, :]).T


e_T = np.asarray(body_only_T(h)); j_T = np.asarray(jax.jit(body_only_T)(h))
print('  maxdiff=%.3e' % np.abs(j_T - e_T).max())
print('  jit:'); print(j_T)
print('  C-order reshape(prod):'); print(np.asarray(body_norot(h)).reshape(6, 3))
out['only_T_maxdiff'] = float(np.abs(j_T - e_T).max())
out['only_T_equals_reshape'] = bool(np.array_equal(j_T, np.asarray(body_norot(h)).reshape(6, 3)))
print('  only_T == C-order reshape(prod): %s' % out['only_T_equals_reshape'])

print('\n=== do alternative transpose spellings avoid it? (constant rotation, traced half) ===')
def mk(mode):
    def f(h_):
        nd = I3[:, FA] * SIGN[None, :]
        prod = nd * h_[FA][None, :]
        off = {'dotT': prod.T, 'swapaxes': jnp.swapaxes(prod, 0, 1),
               'transpose': jnp.transpose(prod), 'einsum': jnp.einsum('ij->ji', prod)}[mode]
        return cen[None, :] + off
    return f


for mode in ('dotT', 'swapaxes', 'transpose', 'einsum'):
    v = np.asarray(jax.jit(mk(mode))(h))
    print('  %-10s max|jit-eager|=%.3e' % (mode, np.abs(v - eager).max()))
    out.setdefault('rewrites_max_jit_vs_eager', {})[mode] = float(np.abs(v - eager).max())

print('\n=== HLO op inventory (identity-rotation constant shape) ===')
low = jax.jit(body_const_rot).lower(h, cen)
txt = low.as_text()
ops = re.findall(r'=\s*(\w+)\(', txt)
from collections import Counter
print('  ', Counter(ops).most_common(12))
out['hlo_ops'] = Counter(ops).most_common(12)
for line in txt.splitlines():
    if any(k in line for k in ('transpose', 'reshape', 'iota', 'gather', 'concatenate')):
        print('   ', line.strip()[:150])

Path(Path(__file__).resolve().parent / 'mech.json').write_text(json.dumps(out, indent=2))
print('\nwrote mech.json')
