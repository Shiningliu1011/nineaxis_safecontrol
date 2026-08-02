from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "robot_safecontrol_moveit"


def _glob_models(pattern: str) -> list[str]:
    """Glob files under ``models/`` relative to the project root."""
    return [str(p) for p in Path("models").glob(pattern)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/data/nurbs", ["data/nurbs/ik_input.mat"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/tests", glob("tests/*.py")),
        # Model files: URDF, meshes, and MoveIt configuration.
        (
            f"share/{package_name}/models/ninezzhou/urdf",
            _glob_models("ninezzhou/urdf/*.urdf"),
        ),
        (
            f"share/{package_name}/models/ninezzhou/meshes",
            _glob_models("ninezzhou/meshes/*.STL"),
        ),
        (
            f"share/{package_name}/models/ninezzhou_moveit_config/config",
            _glob_models("ninezzhou_moveit_config/config/*"),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="robot_safecontrol maintainers",
    maintainer_email="maintainer@example.com",
    description="MoveIt 2 / ROS 2 continuous IK, planning, and execution pipeline.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "plan_transition = robot_safecontrol_moveit.plan_transition:main",
            "mujoco_viewer = robot_safecontrol_moveit.mujoco_viewer_with_cylinder:main",
            "mujoco_joint_state_viewer = robot_safecontrol_moveit.mujoco_viewer_with_cylinder:main",
            "static_obstacle_publisher = robot_safecontrol_moveit.obstacle_publisher:main",
            "dynamic_obstacle_probe = robot_safecontrol_moveit.dynamic_obstacle_probe:main",
            "transition_planning_server = robot_safecontrol_moveit.transition_planning_server:main",
        ],
    },
)
