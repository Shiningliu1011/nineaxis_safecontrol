from glob import glob

from setuptools import find_packages, setup


package_name = "robot_safecontrol_moveit"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot_safecontrol maintainers",
    maintainer_email="maintainer@example.com",
    description="MoveIt 2 / ROS 2 continuous IK, planning, and execution pipeline.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "plan_transition = robot_safecontrol_moveit.plan_transition:main",
            "mujoco_viewer = robot_safecontrol_moveit.mujoco_viewer_with_cylinder:main",
        ],
    },
)
