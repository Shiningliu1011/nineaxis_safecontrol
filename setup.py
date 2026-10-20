from glob import glob
from pathlib import Path

from setuptools import Extension, find_packages, setup


package_name = "robot_safecontrol_moveit"

PORTABLE_OSCBF_RUNTIME_REQUIREMENTS = [
    "cbfpy==0.0.1",
    "jax==0.6.2",
    "jaxlib==0.6.2",
    "qpax==0.1.4",
    "jsonschema==4.23.0",
    "ruckig==0.12.2",
    "trimesh==5.1.0",
]


def _glob_models(pattern: str) -> list[str]:
    """Glob files under ``models/`` relative to the project root."""
    return [str(p) for p in Path("models").glob(pattern)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
    ext_modules=[Extension(
        "robot_static_fcl",
        ["portable_oscbf/native/static_fcl.cpp"],
        include_dirs=["/usr/include/eigen3"],
        libraries=["fcl"],
        extra_compile_args=["-std=c++17", "-O2", "-g0"],
        language="c++",
    )],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", [
            "package.xml", "setup.py", "setup.cfg", "requirements-static-report.txt",
            "requirements-static-recording.txt",
        ]),
        (f"share/{package_name}/scripts", ["scripts/static_obstacle_experiment.py"]),
        (f"share/{package_name}/docs", [
            "docs/static_obstacle_experiments.md",
            "docs/static_obstacle_experiments_handoff.md",
            "docs/static_experiment_storage.md",
            "docs/host_recovery_20261020.md",
        ]),
        (f"share/{package_name}/docs/specs", ["docs/specs/static-obstacle-experiments.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config", ["config/lidar_crop_experiment.rviz"]),
        (f"share/{package_name}/data/nurbs", ["data/nurbs/ik_input.mat"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/tests", glob("tests/*.py")),
        (f"share/{package_name}/tests/data", glob("tests/data/*.npz")),
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
            f"share/{package_name}/models/ninezzhou/collision_meshes",
            _glob_models("ninezzhou/collision_meshes/*.stl")
            + _glob_models("ninezzhou/collision_meshes/*.json"),
        ),
        (
            f"share/{package_name}/models/ninezzhou_moveit_config/config",
            _glob_models("ninezzhou_moveit_config/config/*"),
        ),
        # Portable OSCBF control core shipped for the oscbf_controller node.
        (
            f"share/{package_name}/portable_oscbf/work",
            glob("portable_oscbf/work/*.py"),
        ),
        (
            f"share/{package_name}/portable_oscbf/config",
            glob("portable_oscbf/config/*.yaml") + glob("portable_oscbf/config/*.json"),
        ),
        (
            f"share/{package_name}/portable_oscbf/scripts",
            ["portable_oscbf/scripts/generate_closed_collision_meshes.py"],
        ),
        (
            f"share/{package_name}/portable_oscbf/native",
            glob("portable_oscbf/native/*.cpp"),
        ),
        (
            f"share/{package_name}/portable_oscbf/data",
            ["portable_oscbf/data/ik_input.mat"],
        ),
        (
            f"share/{package_name}/portable_oscbf",
            ["portable_oscbf/requirements.txt", "portable_oscbf/requirements-geometry.txt"],
        ),
        (
            f"share/{package_name}/portable_oscbf/vendor/dpax/dpax",
            glob("portable_oscbf/vendor/dpax/dpax/*.py"),
        ),
    ],
    install_requires=["setuptools", "msgpack==1.1.2", *PORTABLE_OSCBF_RUNTIME_REQUIREMENTS],
    extras_require={"hardware": ["python-can==4.6.1", "PyYAML>=5.4.1,<7"]},
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="robot_safecontrol maintainers",
    maintainer_email="maintainer@example.com",
    description="MoveIt 2 / ROS 2 continuous IK, planning, and execution pipeline.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "mujoco_viewer = robot_safecontrol_moveit.mujoco_viewer_with_cylinder:main",
            "mujoco_joint_state_viewer = robot_safecontrol_moveit.mujoco_viewer_with_cylinder:main",
            "transition_planning_server = robot_safecontrol_moveit.transition_planning_server:main",
            "perception_bridge = robot_safecontrol_moveit.perception_bridge:main",
            "perception_demo = robot_safecontrol_moveit.perception_demo:main",
            "lidar_crop_experiment = robot_safecontrol_moveit.lidar_crop_experiment:main",
            "oscbf_controller = robot_safecontrol_moveit.oscbf_controller:main",
            "oscbf_plant = robot_safecontrol_moveit.oscbf_plant:main",
            "static_obstacle_experiment = robot_safecontrol_moveit.static_experiment:main",
            "hardware_bridge = robot_safecontrol_moveit.hardware_bridge:main",
            "livox_mid360_node = robot_safecontrol_moveit.livox_mid360.ros_node:main",
            "livox_mid360_tool = robot_safecontrol_moveit.livox_mid360.cli:main",
        ],
    },
)
