#!/usr/bin/env python3
"""M1 acceptance: JAX POE kinematics vs the repository URDF.

Checks that ``NineaxisManipulatorJAX`` matches the URDF shipped in this
repository (``models/ninezzhou/urdf/ninezzhou.urdf``) for joint count, joint
types, position limits, forward kinematics and the analytic Jacobian.

Control-point note: the reference implementation defines the control frame as
``ee_link`` = Link9 + 0.235 m on local +X, which is exactly the ``tool0``
frame of the repository URDF.  M1 therefore keeps that reference behaviour
(URDF-tool0 control point) instead of switching to raw Link9.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import jax

# The control pipeline runs with JAX x64 (OSCBF_PORTING_GUIDE.md §6.1).  Enable
# it before any JAX array/jit exists so the model and its outputs are float64,
# which the 1e-9 FK and 1e-5 Jacobian tolerances require.
jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import jax.numpy as jnp
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from work.nineaxis_kinematics import JOINT_CHAIN
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX


URDF_PATH = (
    Path(__file__).resolve().parents[2]
    / "models" / "ninezzhou" / "urdf" / "ninezzhou.urdf"
)
ACTIVE_JOINT_NAMES = tuple(f"J{i}" for i in range(1, 10))
_SEED = 20260806


def _parse_urdf_joints():
    """Parse joint tree (name, type, parent, child, origin, axis, limits)."""

    tree = ET.parse(URDF_PATH)
    joints = {}
    for element in tree.getroot().findall("joint"):
        name = element.get("name")
        origin = element.find("origin")
        xyz = np.zeros(3)
        rpy = np.zeros(3)
        if origin is not None:
            if origin.get("xyz"):
                xyz = np.array([float(v) for v in origin.get("xyz").split()])
            if origin.get("rpy"):
                rpy = np.array([float(v) for v in origin.get("rpy").split()])
        axis = np.array([1.0, 0.0, 0.0])
        axis_element = element.find("axis")
        if axis_element is not None and axis_element.get("xyz"):
            axis = np.array([float(v) for v in axis_element.get("xyz").split()])
        limit = element.find("limit")
        lower = float(limit.get("lower")) if limit is not None else None
        upper = float(limit.get("upper")) if limit is not None else None
        velocity = float(limit.get("velocity")) if limit is not None else None
        joints[name] = {
            "type": element.get("type"),
            "parent": element.find("parent").get("link"),
            "child": element.find("child").get("link"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
            "lower": lower,
            "upper": upper,
            "velocity": velocity,
        }
    return joints


def _joint_transform(joint, q):
    """URDF joint transform T_parent_child (same convention as the model)."""

    T = np.eye(4)
    T[:3, 3] = joint["xyz"]
    T[:3, :3] = Rotation.from_euler("xyz", joint["rpy"]).as_matrix()
    axis = joint["axis"]
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    if joint["type"] == "revolute":
        c, s = np.cos(q), np.sin(q)
        vv = 1.0 - c
        ux, uy, uz = axis
        R_j = np.array([
            [c + ux * ux * vv, ux * uy * vv - uz * s, ux * uz * vv + uy * s],
            [uy * ux * vv + uz * s, c + uy * uy * vv, uy * uz * vv - ux * s],
            [uz * ux * vv - uy * s, uz * uy * vv + ux * s, c + uz * uz * vv],
        ])
        T[:3, :3] = T[:3, :3] @ R_j
    elif joint["type"] == "prismatic":
        T[:3, 3] += T[:3, :3] @ (axis * q)
    return T


def _urdf_fk(joints, q):
    """Forward kinematics to ``tool0`` from the parsed URDF."""

    q_by_joint = dict(zip(ACTIVE_JOINT_NAMES, q))
    transforms = {"base_link": np.eye(4)}
    order = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9",
             "tool0_fixed"]
    for name in order:
        joint = joints[name]
        if joint["type"] in ("revolute", "prismatic"):
            q_value = q_by_joint[name]
        else:
            q_value = 0.0
        transforms[joint["child"]] = (
            transforms[joint["parent"]] @ _joint_transform(joint, q_value)
        )
    return transforms["tool0"]


def _random_qs(robot, count=6):
    rng = np.random.default_rng(_SEED)
    lower = np.asarray(robot.joint_lower_limits)
    upper = np.asarray(robot.joint_upper_limits)
    margin = 0.05 * np.ones(9)
    margin[0] = 0.05  # J1 lower is exactly 0.0
    samples = []
    for _ in range(count):
        samples.append(rng.uniform(lower + margin, upper - margin))
    return samples


@pytest.fixture(scope="module")
def model_and_urdf():
    assert URDF_PATH.is_file(), f"URDF not found: {URDF_PATH}"
    return NineaxisManipulatorJAX(), _parse_urdf_joints()


def test_joint_count_types_and_position_limits_match_urdf(model_and_urdf):
    robot, joints = model_and_urdf
    active = [j for j in ACTIVE_JOINT_NAMES if joints[j]["type"] in
              ("revolute", "prismatic")]
    assert len(active) == 9
    assert robot.num_joints == 9
    assert joints["J1"]["type"] == "prismatic"
    for name in ACTIVE_JOINT_NAMES[1:]:
        assert joints[name]["type"] == "revolute"

    urdf_lower = np.array([joints[name]["lower"] for name in ACTIVE_JOINT_NAMES])
    urdf_upper = np.array([joints[name]["upper"] for name in ACTIVE_JOINT_NAMES])
    np.testing.assert_allclose(
        np.asarray(robot.joint_lower_limits), urdf_lower, rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(
        np.asarray(robot.joint_upper_limits), urdf_upper, rtol=0.0, atol=1e-12)

    # Recorded cross-check (not asserted equal): the runtime velocity limits
    # come from the actuator profile, while the URDF ``velocity`` attributes
    # are conservative documentation values.
    urdf_velocity = np.array(
        [joints[name]["velocity"] for name in ACTIVE_JOINT_NAMES])
    runtime_velocity = np.asarray(robot.joint_max_velocities)
    assert np.all(runtime_velocity > 0.0)
    print(f"URDF velocity    : {urdf_velocity}")
    print(f"Runtime dq_max   : {runtime_velocity}")


def test_ee_fk_matches_urdf_tool0(model_and_urdf):
    robot, joints = model_and_urdf
    samples = [np.zeros(9)] + _random_qs(robot, count=6)
    for q in samples:
        q_jax = jnp.asarray(q)
        T_jax = np.asarray(robot.ee_transform(q_jax))
        T_urdf = _urdf_fk(joints, q)
        np.testing.assert_allclose(
            T_jax[:3, 3], T_urdf[:3, 3], rtol=0.0, atol=1e-9,
            err_msg=f"FK position mismatch at q={q}")
        np.testing.assert_allclose(
            T_jax[:3, :3], T_urdf[:3, :3], rtol=0.0, atol=1e-9,
            err_msg=f"FK rotation mismatch at q={q}")


def test_ee_control_point_is_urdf_tool0(model_and_urdf):
    """The JAX control frame equals URDF tool0 (Link9 + 0.235 m on +X)."""

    robot, joints = model_and_urdf
    tool0_joint = joints["tool0_fixed"]
    assert tool0_joint["type"] == "fixed"
    np.testing.assert_allclose(
        tool0_joint["xyz"], [0.235, 0.0, 0.0], rtol=0.0, atol=1e-12)
    assert any(
        child == "ee_link" and jtype == "fixed"
        for _, child, jtype, *_ in JOINT_CHAIN)
    assert any(
        parent == "Link9" and child == "ee_link"
        for parent, child, *_ in JOINT_CHAIN)

    q = np.zeros(9)
    T_urdf = _urdf_fk(joints, q)
    np.testing.assert_allclose(
        np.asarray(robot.ee_position(jnp.asarray(q))), T_urdf[:3, 3],
        rtol=0.0, atol=1e-9)


def test_jacobian_matches_finite_difference(model_and_urdf):
    robot, _ = model_and_urdf

    def fd_jacobian(q):
        jac = np.zeros((6, 9))
        R0 = np.asarray(robot.ee_rotation(jnp.asarray(q)))
        h = 5e-4
        for i in range(9):
            q2p = q.copy()
            q1p = q.copy()
            q1m = q.copy()
            q2m = q.copy()
            q2p[i] += 2.0 * h
            q1p[i] += h
            q1m[i] -= h
            q2m[i] -= 2.0 * h
            p2p = np.asarray(robot.ee_position(jnp.asarray(q2p)))
            p1p = np.asarray(robot.ee_position(jnp.asarray(q1p)))
            p1m = np.asarray(robot.ee_position(jnp.asarray(q1m)))
            p2m = np.asarray(robot.ee_position(jnp.asarray(q2m)))
            jac[:3, i] = (-p2p + 8.0 * p1p - 8.0 * p1m + p2m) / (12.0 * h)
            w2p = Rotation.from_matrix(
                np.asarray(robot.ee_rotation(jnp.asarray(q2p))) @ R0.T).as_rotvec()
            w1p = Rotation.from_matrix(
                np.asarray(robot.ee_rotation(jnp.asarray(q1p))) @ R0.T).as_rotvec()
            w1m = Rotation.from_matrix(
                np.asarray(robot.ee_rotation(jnp.asarray(q1m))) @ R0.T).as_rotvec()
            w2m = Rotation.from_matrix(
                np.asarray(robot.ee_rotation(jnp.asarray(q2m))) @ R0.T).as_rotvec()
            jac[3:, i] = (-w2p + 8.0 * w1p - 8.0 * w1m + w2m) / (12.0 * h)
        return jac

    for q in _random_qs(robot, count=4):
        J_ana = np.asarray(robot.ee_jacobian(jnp.asarray(q)))
        J_fd = fd_jacobian(q)
        absolute_error = np.max(np.abs(J_ana - J_fd))
        assert absolute_error < 1e-9, (
            f"Jacobian absolute error {absolute_error:.2e} at q={q}")
        significant = np.abs(J_ana) >= 1e-6
        if np.any(significant):
            relative_error = np.max(
                np.abs(J_ana[significant] - J_fd[significant])
                / np.abs(J_ana[significant]))
            assert relative_error < 1e-5, (
                f"Jacobian relative error {relative_error:.2e} at q={q}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
