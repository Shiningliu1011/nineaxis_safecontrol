import numpy as np, sys
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
from refs import analytic_aa, qp_reference, fcl_reference, gjk_reference
I = np.eye(3)
cases = [('gap', [3.,0,0], [1,1,1], [1,1,1]), ('unequal', [3.1,.17,.23], [1,.7,.6], [.4,.3,.2]),
         ('robot_scale', [.31,.017,.023], [.1,.07,.06], [.04,.03,.02]), ('touch', [2.,0,0], [1,1,1], [1,1,1]),
         ('overlap', [1.5,0,0], [1,1,1], [1,1,1]), ('contained', [0,0,0], [1,1,1], [.2,.2,.2]),
         ('identical', [0,0,0], [1,1,1], [1,1,1]), ('corner', [2.5,2.5,2.5], [1,1,1], [.5,.5,.5])]
for n, t, a, b in cases:
    t = np.array(t, float); a = np.array(a, float); b = np.array(b, float)
    an = analytic_aa(t, a, b)
    q = qp_reference(I, np.zeros(3), a, I, t, b)
    f, fc = fcl_reference(I, np.zeros(3), a, I, t, b)
    g, go = gjk_reference(I, np.zeros(3), a, I, t, b)
    print(f'{n:12s} analytic={an:.9f} qp={q[0]:.9f}(ov={q[3]}) fcl={f:.9f}(coll={fc}) gjk={g:.9f}(ov={go})')
