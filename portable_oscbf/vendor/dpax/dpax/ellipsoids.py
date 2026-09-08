import jax
from jax import grad, jit, custom_jvp
import jax.numpy as jnp

from dpax.pdip_solver import solve_lp


# ---- Fibonacci 球面采样 (预计算, 固定 K) ----

def fibonacci_sphere(n):
    """Fibonacci 球面均匀采样, 返回 n 个单位法向量 (n, 3)."""
    golden_ratio = (1 + jnp.sqrt(5)) / 2
    i = jnp.arange(n)
    theta = 2 * jnp.pi * i / golden_ratio
    phi = jnp.arccos(1 - 2 * (i + 0.5) / n)
    x = jnp.sin(phi) * jnp.cos(theta)
    y = jnp.sin(phi) * jnp.sin(theta)
    z = jnp.cos(phi)
    return jnp.column_stack([x, y, z])


# 默认 42 面 (精度 vs 速度平衡)
_DEFAULT_K = 42
_DEFAULT_NORMALS = fibonacci_sphere(_DEFAULT_K)


# ---- 椭球体 → 多面体转换 ----

def ellipsoid_to_polytope(P, Q, normals=None):
    """椭球体 x^T P x <= 1 旋转 Q 后转为多面体 Ax <= b.

    Parameters
    ----------
    P : (3, 3)  椭球体形状矩阵 (体坐标系, diag(1/a^2, 1/b^2, 1/c^2))
    Q : (3, 3)  体→世界旋转矩阵 W_Q_B
    normals : (K, 3)  采样方向 (体坐标系, 单位向量), 默认 42 面

    Returns
    -------
    A : (K, 3)  世界坐标系法向量
    b : (K,)    偏移量
    """
    if normals is None:
        normals = _DEFAULT_NORMALS
    P_inv = jnp.linalg.inv(P + 1e-10 * jnp.eye(3))
    b_body = jnp.sqrt(jnp.einsum('ki,ij,kj->k', normals, P_inv, normals))
    A_world = normals @ Q.T
    return A_world, b_body


# ---- 椭球体-椭球体 proximity ----

def _ellipsoid_ee_problem_matrices(P1, r1, Q1, P2, r2, Q2, normals):
    """两个椭球体的 DCOL LP 矩阵.

    注意: ellipsoid_to_polytope 已将法向量从体坐标系旋转到世界坐标系,
    所以这里直接用世界坐标系法向量, 不再传 Q 给 polytope 格式.
    """
    A1, b1 = ellipsoid_to_polytope(P1, Q1, normals)  # A1 已是世界坐标系
    A2, b2 = ellipsoid_to_polytope(P2, Q2, normals)

    c = jnp.array([0., 0., 0., 1.])
    G = jnp.vstack((
        jnp.hstack((A1, -b1[:, None])),
        jnp.hstack((A2, -b2[:, None])),
    ))
    h = jnp.concatenate((A1 @ r1, A2 @ r2))
    return c, G, h


@custom_jvp
@jit
def ellipsoid_proximity(P1, r1, Q1, P2, r2, Q2, normals=None):
    """两个椭球体 DCOL proximity (α 缩放因子).

    α > 1 → 无碰撞, α <= 1 → 碰撞.
    """
    if normals is None:
        normals = _DEFAULT_NORMALS
    c, G, h = _ellipsoid_ee_problem_matrices(P1, r1, Q1, P2, r2, Q2, normals)
    x, s, z = solve_lp(c, G, h)
    return x[3]


@jit
def _ellipsoid_lagrangian(P1, r1, Q1, P2, r2, Q2, normals, x, s, z):
    c, G, h = _ellipsoid_ee_problem_matrices(P1, r1, Q1, P2, r2, Q2, normals)
    return z.dot(G @ x - h)


@jit
def ellipsoid_proximity_grads(P1, r1, Q1, P2, r2, Q2, normals=None):
    """返回 α 和 ∂α/∂(P1, r1, Q1, P2, r2, Q2)."""
    if normals is None:
        normals = _DEFAULT_NORMALS
    c, G, h = _ellipsoid_ee_problem_matrices(P1, r1, Q1, P2, r2, Q2, normals)
    x, s, z = solve_lp(c, G, h)
    alpha = x[3]

    lag_grad = grad(_ellipsoid_lagrangian, argnums=(0, 1, 2, 3, 4, 5))
    gP1, gr1, gQ1, gP2, gr2, gQ2 = lag_grad(
        P1, r1, Q1, P2, r2, Q2, normals, x, s, z)

    return alpha, gP1, gr1, gQ1, gP2, gr2, gQ2


@ellipsoid_proximity.defjvp
@jit
def _ellipsoid_proximity_jvp(primals, targets):
    P1, r1, Q1, P2, r2, Q2, normals = primals
    dP1, dr1, dQ1, dP2, dr2, dQ2, _ = targets

    alpha, gP1, gr1, gQ1, gP2, gr2, gQ2 = ellipsoid_proximity_grads(
        P1, r1, Q1, P2, r2, Q2, normals)

    tangent_out = (jnp.sum(dP1 * gP1) + dr1.dot(gr1) + jnp.sum(dQ1 * gQ1) +
                   jnp.sum(dP2 * gP2) + dr2.dot(gr2) + jnp.sum(dQ2 * gQ2))
    return alpha, tangent_out


# ---- 椭球体-多面体 proximity ----

def _ellipsoid_poly_problem_matrices(P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals):
    """椭球体 vs 多面体 DCOL LP 矩阵."""
    A_ell, b_ell = ellipsoid_to_polytope(P_ell, Q_ell, normals)

    c = jnp.array([0., 0., 0., 1.])
    G = jnp.vstack((
        jnp.hstack((A_ell, -b_ell[:, None])),
        jnp.hstack((A_poly @ Q_poly.T, -b_poly[:, None])),
    ))
    h = jnp.concatenate((A_ell @ r_ell, A_poly @ Q_poly.T @ r_poly))
    return c, G, h


@custom_jvp
@jit
def ellipsoid_polytope_proximity(P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals=None):
    """椭球体 vs 多面体 DCOL proximity."""
    if normals is None:
        normals = _DEFAULT_NORMALS
    c, G, h = _ellipsoid_poly_problem_matrices(
        P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals)
    x, s, z = solve_lp(c, G, h)
    return x[3]


@jit
def _ellipsoid_poly_lagrangian(P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals, x, s, z):
    c, G, h = _ellipsoid_poly_problem_matrices(
        P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals)
    return z.dot(G @ x - h)


@jit
def ellipsoid_polytope_proximity_grads(P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals=None):
    """返回 α 和 ∂α/∂(P_ell, r_ell, Q_ell) (椭球体参数的梯度)."""
    if normals is None:
        normals = _DEFAULT_NORMALS
    c, G, h = _ellipsoid_poly_problem_matrices(
        P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals)
    x, s, z = solve_lp(c, G, h)
    alpha = x[3]

    lag_grad = grad(_ellipsoid_poly_lagrangian, argnums=(0, 1, 2))
    gP, gr, gQ = lag_grad(
        P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals, x, s, z)

    return alpha, gP, gr, gQ


@ellipsoid_polytope_proximity.defjvp
@jit
def _ellipsoid_poly_proximity_jvp(primals, targets):
    P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals = primals
    dP_ell, dr_ell, dQ_ell, _, _, _, _, _ = targets

    alpha, gP, gr, gQ = ellipsoid_polytope_proximity_grads(
        P_ell, r_ell, Q_ell, A_poly, b_poly, r_poly, Q_poly, normals)

    tangent_out = jnp.sum(dP_ell * gP) + dr_ell.dot(gr) + jnp.sum(dQ_ell * gQ)
    return alpha, tangent_out


# ---- 椭球体-胶囊体 proximity ----

def _closest_pt_on_segment(center, a, b):
    """点 center 到线段 [a, b] 的最近点."""
    d = b - a
    t = jnp.clip(jnp.dot(center - a, d) / jnp.maximum(jnp.dot(d, d), 1e-12), 0.0, 1.0)
    return a + t * d


@custom_jvp
@jit
def ellipsoid_capsule_proximity(P_ell, r_ell, Q_ell, R_cap, a_cap, b_cap, normals=None):
    """椭球体 vs 胶囊体 proximity (α 近似).

    策略: 椭球体→多面体, 胶囊体→线段+球膨胀.
    对多面体近似的每个面, 检查面到线段距离 > R_cap + b_k.
    使用简化公式: α ≈ dist(ellipsoid_center, segment) - R_cap - max_semi_axis.

    更精确的实现: 将胶囊体近似为球 (以最近点为中心, R=R_cap),
    然后求椭球体 vs 球的 DCOL proximity.
    """
    if normals is None:
        normals = _DEFAULT_NORMALS

    # 胶囊体 → 最近点球
    closest = _closest_pt_on_segment(r_ell, a_cap, b_cap)

    # 椭球体多面体 vs 球 (R_cap, center=closest)
    A_ell, b_ell = ellipsoid_to_polytope(P_ell, Q_ell, normals)

    # 球的多面体: 用同一组法向量, b = R_cap
    b_sphere = jnp.full(normals.shape[0], R_cap)

    c = jnp.array([0., 0., 0., 1.])
    G = jnp.vstack((
        jnp.hstack((A_ell, -b_ell[:, None])),
        jnp.hstack((normals, -b_sphere[:, None])),
    ))
    h = jnp.concatenate((A_ell @ r_ell, normals @ closest))
    x, s, z = solve_lp(c, G, h)
    return x[3]


@jit
def _ellipsoid_capsule_lagrangian(P_ell, r_ell, Q_ell, R_cap, a_cap, b_cap, normals, x, s, z):
    """Lagrangian for implicit function theorem."""
    # 重新构造 LP (因为 closest 依赖于 r_ell, a_cap, b_cap)
    closest = _closest_pt_on_segment(r_ell, a_cap, b_cap)
    A_ell, b_ell = ellipsoid_to_polytope(P_ell, Q_ell, normals)
    b_sphere = jnp.full(normals.shape[0], R_cap)

    c = jnp.array([0., 0., 0., 1.])
    G = jnp.vstack((
        jnp.hstack((A_ell, -b_ell[:, None])),
        jnp.hstack((normals, -b_sphere[:, None])),
    ))
    h = jnp.concatenate((A_ell @ r_ell, normals @ closest))
    return z.dot(G @ x - h)


@jit
def ellipsoid_capsule_proximity_grads(P_ell, r_ell, Q_ell, R_cap, a_cap, b_cap, normals=None):
    """返回 α 和 ∂α/∂(P_ell, r_ell, Q_ell, R_cap, a_cap, b_cap)."""
    if normals is None:
        normals = _DEFAULT_NORMALS

    closest = _closest_pt_on_segment(r_ell, a_cap, b_cap)
    A_ell, b_ell = ellipsoid_to_polytope(P_ell, Q_ell, normals)
    b_sphere = jnp.full(normals.shape[0], R_cap)

    c = jnp.array([0., 0., 0., 1.])
    G = jnp.vstack((
        jnp.hstack((A_ell, -b_ell[:, None])),
        jnp.hstack((normals, -b_sphere[:, None])),
    ))
    h = jnp.concatenate((A_ell @ r_ell, normals @ closest))
    x, s, z = solve_lp(c, G, h)
    alpha = x[3]

    lag_grad = grad(_ellipsoid_capsule_lagrangian, argnums=(0, 1, 2, 3, 4, 5))
    gP, gr, gQ, gR, ga, gb = lag_grad(
        P_ell, r_ell, Q_ell, R_cap, a_cap, b_cap, normals, x, s, z)

    return alpha, gP, gr, gQ, gR, ga, gb


@ellipsoid_capsule_proximity.defjvp
@jit
def _ellipsoid_capsule_proximity_jvp(primals, targets):
    P_ell, r_ell, Q_ell, R_cap, a_cap, b_cap, normals = primals
    dP_ell, dr_ell, dQ_ell, dR_cap, da_cap, db_cap, _ = targets

    alpha, gP, gr, gQ, gR, ga, gb = ellipsoid_capsule_proximity_grads(
        P_ell, r_ell, Q_ell, R_cap, a_cap, b_cap, normals)

    tangent_out = (jnp.sum(dP_ell * gP) + dr_ell.dot(gr) + jnp.sum(dQ_ell * gQ) +
                   dR_cap * gR + da_cap.dot(ga) + db_cap.dot(gb))
    return alpha, tangent_out
