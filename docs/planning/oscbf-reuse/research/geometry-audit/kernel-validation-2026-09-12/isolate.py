"""Isolate which candidate term produces the wrong OBB-OBB distance."""
import sys, numpy as np
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jp
from work import dpax_collision as dc

R = jp.eye(3); I = jp.eye(4); z3 = jp.zeros(3)

def parts(t, a, b):
    t = jp.asarray(t, dtype=jp.float64); a = jp.asarray(a, dtype=jp.float64); b = jp.asarray(b, dtype=jp.float64)
    ci = dc._obb_corners_world(I, z3, R, a)
    cj = dc._obb_corners_world(I, t, R, b)
    edge = float(dc._edge_edge_minimum(ci, cj))
    face = float(dc._point_face_minimum(I, z3, R, a, I, t, R, b))
    total = float(dc._obb_pair_distance_impl(I, z3, R, a, I, t, R, b))
    analytic = float(np.linalg.norm(np.maximum(np.abs(np.asarray(t)) - np.asarray(a) - np.asarray(b), 0.0)))
    return edge, face, total, analytic

for name, t, a, b in [
    ('gap', [3.,0,0], [1,1,1], [1,1,1]),
    ('unequal_separated', [3.1,.17,.23], [1,.7,.6], [.4,.3,.2]),
    ('unequal_robot_scale', [.31,.017,.023], [.1,.07,.06], [.04,.03,.02]),
    ('touch', [2.,0,0], [1,1,1], [1,1,1]),
    ('overlap', [1.5,0,0], [1,1,1], [1,1,1]),
    ('contained', [0,0,0], [1,1,1], [.2,.2,.2]),
    ('identical', [0,0,0], [1,1,1], [1,1,1]),
]:
    e, f, tot, an = parts(t, a, b)
    print(f'{name:22s} edge={e:.9f} face={f:.9f} kernel={tot:.9f} analytic={an:.9f} '
          f'{"EDGE-WRONG" if e < an - 1e-9 else ""}{" KERNEL<ANALYTIC" if tot < an - 1e-9 else ""}')
