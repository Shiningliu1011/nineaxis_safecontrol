from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import URDF

from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
from work.nineaxis_kinematics import NineaxisKinematics


REPO_ROOT = Path(__file__).resolve().parents[4]
URDF_PATH = REPO_ROOT / "models/ninezzhou/urdf/ninezzhou.urdf"


def urdf_poses(robot: URDF, q: np.ndarray) -> dict[str, np.ndarray]:
    values = dict(zip(DEFAULT_JOINT_NAMES, q, strict=True))
    poses = {robot.get_root(): np.eye(4)}
    for name in robot.get_chain(robot.get_root(), "tool0", links=False):
        joint = robot.joint_map[name]
        origin = np.eye(4)
        origin[:3, 3] = joint.origin.xyz
        origin[:3, :3] = Rotation.from_euler("xyz", joint.origin.rpy).as_matrix()
        motion = np.eye(4)
        if joint.type == "prismatic":
            motion[:3, 3] = np.asarray(joint.axis) * values[name]
        elif joint.type == "revolute":
            motion[:3, :3] = Rotation.from_rotvec(
                np.asarray(joint.axis) * values[name]
            ).as_matrix()
        elif joint.type != "fixed":
            raise ValueError(f"unsupported joint type: {joint.type}")
        poses[joint.child] = poses[joint.parent] @ origin @ motion
    return poses


def main() -> None:
    robot = URDF.from_xml_string(URDF_PATH.read_bytes())
    kinematics = NineaxisKinematics()
    active_names = tuple(
        joint.name for joint in robot.joints if joint.type != "fixed"
    )
    assert active_names == DEFAULT_JOINT_NAMES
    assert robot.joint_map["J1"].type == "prismatic"
    assert all(robot.joint_map[name].type == "revolute" for name in active_names[1:])

    # 这些数值仅用于检查名义运动学，均未连接实物。
    configurations = (
        np.zeros(9),
        np.array([0.25, 0.3, -0.4, 0.2, -0.6, 0.25, -0.15, 0.35, -0.2]),
        np.array([0.5, -0.7, 0.6, -0.5, 0.8, -0.4, 0.3, -0.2, 0.1]),
    )
    maximum_matrix_error = 0.0
    for q in configurations:
        assert np.all(q >= kinematics.joint_limits.q_min)
        assert np.all(q <= kinematics.joint_limits.q_max)
        parsed_poses = urdf_poses(robot, q)
        portable_poses = kinematics.forward_kinematics(q)
        for name, expected in parsed_poses.items():
            alias = "ee_link" if name == "tool0" else name
            actual = portable_poses[alias]
            maximum_matrix_error = max(
                maximum_matrix_error, float(np.max(np.abs(actual - expected)))
            )
            np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-12)

    j9 = robot.joint_map["J9"]
    tool_joint = robot.joint_map["tool0_fixed"]
    assert j9.child == tool_joint.parent == "Link9"
    axis = np.asarray(j9.axis, dtype=float)
    axis /= np.linalg.norm(axis)
    tool_origin = np.asarray(tool_joint.origin.xyz, dtype=float)
    radius = float(np.linalg.norm(np.cross(axis, tool_origin)))
    np.testing.assert_allclose(radius, 0.235, rtol=0.0, atol=1e-15)

    zero_poses = urdf_poses(robot, configurations[0])
    zero_tool = zero_poses["tool0"][:3, 3]
    zero_axis = zero_poses["Link9"][:3, :3] @ axis
    zero_axis_origin = zero_poses["Link9"][:3, 3]
    j9_derivative = np.cross(zero_axis, zero_tool - zero_axis_origin)
    position_jacobian = kinematics.compute_position_jacobian(configurations[0])
    np.testing.assert_allclose(
        position_jacobian[:, 8], j9_derivative, rtol=0.0, atol=1e-12
    )
    q_j9 = configurations[0].copy()
    q_j9[8] = 0.1
    displacement = float(np.linalg.norm(kinematics.ee_position(q_j9) - zero_tool))
    np.testing.assert_allclose(
        displacement, 2.0 * radius * np.sin(0.05), rtol=0.0, atol=1e-12
    )

    print("active_joint_names:", active_names)
    print("active_joint_types:", tuple(robot.joint_map[name].type for name in active_names))
    print("configuration_count:", len(configurations))
    print("link_transforms_per_configuration:", len(zero_poses))
    print("max_urdf_vs_portable_matrix_abs_error:", maximum_matrix_error)
    print("j9_axis_in_Link9:", axis.tolist())
    print("tool0_origin_in_Link9_m:", tool_origin.tolist())
    print("j9_axis_to_tool0_distance_m:", radius)
    print("zero_tool0_in_base_link_m:", zero_tool.tolist())
    print("j9_position_derivative_norm_m_per_rad:", float(np.linalg.norm(j9_derivative)))
    print("j9_0p1_rad_tool0_displacement_m:", displacement)
    print("nominal_geometry_checks: passed")


if __name__ == "__main__":
    main()
