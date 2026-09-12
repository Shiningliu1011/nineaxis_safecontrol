"""Independent OBB-OBB distance references (no JAX kernel, no dpax)."""
import numpy as np
import fcl
from scipy.optimize import minimize

def analytic_aa(t, hA, hB):
    """Solid unsigned distance of two axis-aligned boxes (exact, closed form)."""
    return float(np.linalg.norm(np.maximum(np.abs(t) - hA - hB, 0.0)))

def qp_reference(RA, tA, hA, RB, tB, hB, ftol=1e-14, maxiter=400):
    """min || (RA x + tA) - (RB y + tB) ||^2 s.t. |x|<=hA, |y|<=hB (SLSQP).

    Formulated as a constrained QP, an algorithmically different family from
    the vertex-face / edge-edge enumeration used by the JAX kernel.
    Returns (distance, pointA, pointB, overlap) where overlap=True means the
    minimum is (numerically) zero, i.e. the solids intersect.
    """
    RA = np.asarray(RA, float); RB = np.asarray(RB, float)
    tA = np.asarray(tA, float); tB = np.asarray(tB, float)
    hA = np.asarray(hA, float); hB = np.asarray(hB, float)

    def f(z):
        d = (RA @ z[:3] + tA) - (RB @ z[3:] + tB)
        return float(d @ d)

    def g(z):
        d = (RA @ z[:3] + tA) - (RB @ z[3:] + tB)
        return np.concatenate([2 * RA.T @ d, -2 * RB.T @ d])

    z0 = np.zeros(6)
    bounds = [(-hA[i], hA[i]) for i in range(3)] + [(-hB[i], hB[i]) for i in range(3)]
    res = minimize(f, z0, jac=g, method='SLSQP', bounds=bounds,
                   options={'ftol': ftol, 'maxiter': maxiter})
    z = res.x
    pA = RA @ z[:3] + tA
    pB = RB @ z[3:] + tB
    value = f(z)
    scale = max(np.linalg.norm(hA), np.linalg.norm(hB))
    overlap = value <= (1e-9 * scale) ** 2
    return float(np.sqrt(max(value, 0.0))), pA, pB, bool(overlap)

def fcl_reference(RA, tA, hA, RB, tB, hB):
    """FCL 0.7 Box-Box reference. Returns (distance, colliding)."""
    o1 = fcl.CollisionObject(fcl.Box(*(2 * np.asarray(hA, float))),
                             fcl.Transform(np.asarray(RA, float), np.asarray(tA, float)))
    o2 = fcl.CollisionObject(fcl.Box(*(2 * np.asarray(hB, float))),
                             fcl.Transform(np.asarray(RB, float), np.asarray(tB, float)))
    cr = fcl.CollisionResult()
    fcl.collide(o1, o2, fcl.CollisionRequest(enable_contact=True), cr)
    colliding = bool(cr.is_collision)
    dr = fcl.DistanceResult()
    dist = float(fcl.distance(o1, o2, fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=False), dr))
    return dist, colliding

def support(R, t, h, d):
    m = R.T @ d
    s = np.where(m >= 0.0, 1.0, -1.0)
    return t + R @ (s * h)

def gjk_reference(RA, tA, hA, RB, tB, hB, tol=1e-12, max_iter=200):
    """GJK distance between two OBBs (support-mapping family).

    Independent of the vertex-face / edge-edge enumeration: uses the
    Minkowski-difference support function and a simplex-based closest-point
    iteration.  Minimises the squared norm via an active-set over the simplex.
    Returns (distance, overlapping).
    """
    RA = np.asarray(RA, float); RB = np.asarray(RB, float)
    tA = np.asarray(tA, float); tB = np.asarray(tB, float)
    hA = np.asarray(hA, float); hB = np.asarray(hB, float)

    def sup(d):
        # support of A - B in direction d: sA(d) - sB(-d)
        return support(RA, tA, hA, d) - support(RB, tB, hB, -d)

    v = tA - tB
    if np.linalg.norm(v) < 1e-15:
        v = np.array([1.0, 0.0, 0.0])
    simplex = [sup(v)]
    v = simplex[-1].copy()
    for _ in range(max_iter):
        if np.linalg.norm(v) < tol:
            return 0.0, True
        w = sup(-v)
        if (w @ (-v)) - (v @ (-v)) < tol * max(1.0, np.linalg.norm(v)):
            break
        simplex.append(w)
        # closest point to origin on the simplex hull (up to 4 points)
        P = np.array(simplex)
        if len(P) == 1:
            v = P[0].copy()
            lam = np.array([1.0])
        else:
            # solve min ||P^T lam||^2 s.t. sum lam = 1, lam >= 0
            n = len(P)
            G = P @ P.T
            res = minimize(lambda l: float(l @ G @ l), np.ones(n) / n, jac=lambda l: 2 * G @ l,
                           method='SLSQP',
                           constraints=[{'type': 'eq', 'fun': lambda l: l.sum() - 1.0,
                                         'jac': lambda l: np.ones(n)}],
                           bounds=[(0.0, 1.0)] * n,
                           options={'ftol': 1e-16, 'maxiter': 300})
            lam = res.x
            keep = lam > 1e-12
            simplex = [P[i] for i in np.nonzero(keep)[0]]
            P = P[keep]; lam = lam[keep] / lam[keep].sum()
            v = lam @ P
        if len(simplex) > 4:
            break
    return float(np.linalg.norm(v)), bool(np.linalg.norm(v) < 1e-9)
