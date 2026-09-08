#!/usr/bin/env python3
"""
test_cbfpy_migration.py
=======================
cbfpy 迁移测试: 验证 JAX 运动学、OSCBF 配置和统一 QP 求解器。
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import jax.numpy as jnp
import pytest


def test_jax_kinematics_fk():
    """测试 JAX FK 精度"""
    from work.nineaxis_kinematics import NineaxisKinematics
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX

    kin_np = NineaxisKinematics()
    kin_jax = NineaxisManipulatorJAX()

    q_test = np.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.1, -0.1, 0.2, -0.1])

    pos_np = kin_np.ee_position(q_test)
    pos_jax = np.array(kin_jax.ee_position(jnp.array(q_test)))

    assert np.allclose(pos_np, pos_jax, atol=1e-6), f"FK 误差: {np.linalg.norm(pos_np - pos_jax):.2e}"


def test_jax_kinematics_jacobian():
    """测试 JAX Jacobian 精度"""
    from work.nineaxis_kinematics import NineaxisKinematics
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX

    kin_np = NineaxisKinematics()
    kin_jax = NineaxisManipulatorJAX()

    q_test = np.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.1, -0.1, 0.2, -0.1])

    J_np = kin_np.compute_full_jacobian(q_test)
    J_jax = np.array(kin_jax.ee_jacobian(jnp.array(q_test)))

    assert np.allclose(J_np, J_jax, atol=1e-6), f"Jacobian 最大误差: {np.max(np.abs(J_np - J_jax)):.2e}"


def test_velocity_config():
    """测试速度级 OSCBF 配置"""
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    robot = NineaxisManipulatorJAX()
    config = NineaxisOSCBFVelocityConfig(robot)

    # 测试 P 矩阵
    q_test = jnp.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.1, -0.1, 0.2, -0.1])
    u_des = jnp.zeros(9)
    P = config.P(q_test, u_des)

    assert P.shape == (9, 9), f"P 矩阵形状错误: {P.shape}"
    assert np.allclose(np.array(P), np.array(P.T), atol=1e-6), "P 矩阵不对称"
    assert np.all(np.linalg.eigvalsh(np.array(P)) >= -1e-6), "P 矩阵不正定"

    # 测试约束 (无障碍物: obs_enabled 全 0)
    from work.jax_control_facade import MAX_JAX_OBSTACLES
    obs_pos_empty = jnp.zeros((MAX_JAX_OBSTACLES, 3))
    obs_rad_empty = jnp.zeros(MAX_JAX_OBSTACLES)
    obs_en_empty = jnp.zeros(MAX_JAX_OBSTACLES)
    h = config.h_2(q_test, obs_pos_empty, obs_rad_empty, obs_en_empty)
    expected_count = int(config.num_cbf)
    assert len(h) == expected_count, (
        f"约束数量错误: {len(h)} (期望 {expected_count})")

    # 测试约束 (有 1 个障碍物, 其余 disabled)
    obs_pos = jnp.zeros((MAX_JAX_OBSTACLES, 3))
    obs_pos = obs_pos.at[0].set(jnp.array([0.3, 0.2, 0.5]))
    obs_rad = jnp.zeros(MAX_JAX_OBSTACLES)
    obs_rad = obs_rad.at[0].set(0.1)
    obs_en = jnp.zeros(MAX_JAX_OBSTACLES)
    obs_en = obs_en.at[0].set(1.0)
    h_obs = config.h_2(q_test, obs_pos, obs_rad, obs_en)
    # 约束数量不变 (固定 shape), 但第 1 个障碍物的 h 值应 < 1e3
    assert len(h_obs) == expected_count, (
        f"约束数量错误: {len(h_obs)} (期望 {expected_count})")
    # 第一条障碍物约束 (robot body 0 vs obs 0) 应有有限 h 值
    assert float(h_obs[config.obstacle_h_start]) < 1e3, (
        "启用障碍物的 h 值不应为 1e3")


@pytest.mark.skip(
    reason="depends on work.unified_qp_solver, excluded from the portable core "
           "(not in OSCBF_PORTING_GUIDE.md Appendix A); qpax path covers the "
           "same solver contract"
)
def test_unified_qp_solver_cbfpy():
    """测试统一 QP 求解器 (cbfpy 后端)"""
    from work.unified_qp_solver import UnifiedQPSolver
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    robot = NineaxisManipulatorJAX()
    config = NineaxisOSCBFVelocityConfig(robot)
    solver = UnifiedQPSolver(backend='cbfpy', config=config)

    z = np.zeros(9)
    u_nom = np.ones(9) * 0.1

    # 预热 JIT
    _, _, _ = solver.solve(z, u_nom)

    # 正式测试
    u_safe, success, solve_time = solver.solve(z, u_nom)

    assert u_safe.shape == (9,), f"u_safe 形状错误: {u_safe.shape}"
    assert success, "QP 求解失败"
    assert solve_time < 1, f"求解时间过长: {solve_time:.3f}ms"


def test_automatic_differentiation():
    """测试 JAX 自动微分精度"""
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    import jax

    robot = NineaxisManipulatorJAX()
    q_test = jnp.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.1, -0.1, 0.2, -0.1])

    # 碰撞约束
    def collision_h(q):
        ee_pos = robot.ee_position(q)
        return jnp.linalg.norm(ee_pos) - 0.5

    grad_auto = jax.grad(collision_h)(q_test)

    # 有限差分
    def finite_diff_grad(fn, q, eps=1e-6):
        grad = np.zeros(len(q))
        for i in range(len(q)):
            q_plus = q.at[i].add(eps)
            q_minus = q.at[i].add(-eps)
            grad[i] = (fn(q_plus) - fn(q_minus)) / (2 * eps)
        return grad

    grad_fd = finite_diff_grad(collision_h, q_test)

    assert np.allclose(np.array(grad_auto), grad_fd, atol=1e-4), \
        f"自动微分误差: {np.max(np.abs(np.array(grad_auto) - grad_fd)):.2e}"


if __name__ == '__main__':
    tests = [
        test_jax_kinematics_fk,
        test_jax_kinematics_jacobian,
        test_velocity_config,
        test_torque_config,
        test_unified_qp_solver_cbfpy,
        test_automatic_differentiation,
    ]

    for test in tests:
        try:
            test()
            print(f"✅ {test.__name__}")
        except Exception as e:
            print(f"❌ {test.__name__}: {e}")

    print(f"\n总计: {len(tests)} 个测试")
