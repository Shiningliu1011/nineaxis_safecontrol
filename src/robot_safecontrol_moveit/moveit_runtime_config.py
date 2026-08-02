"""Centralised MoveIt runtime configuration loader.

Reads URDF, SRDF, kinematics, joint limits, OMPL planning, and controller
configs from the models/ directory shipped inside the ``robot_safecontrol_moveit``
package share folder.  The launch file calls :func:`build_moveit_params`
once and passes the resulting dict to ``move_group``; ``robot_state_publisher``
only needs ``robot_description``.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict

import yaml

# ---------------------------------------------------------------------------
# Low-level file loaders
# ---------------------------------------------------------------------------

_MODEL_ROOT = Path("models")
_URDF_REL = _MODEL_ROOT / "ninezzhou" / "urdf" / "ninezzhou.urdf"
_MESHES_REL = _MODEL_ROOT / "ninezzhou" / "meshes"
_SRDF_REL = _MODEL_ROOT / "ninezzhou_moveit_config" / "config" / "ninezzhou.srdf"
_KINEMATICS_REL = _MODEL_ROOT / "ninezzhou_moveit_config" / "config" / "kinematics.yaml"
_JOINT_LIMITS_REL = _MODEL_ROOT / "ninezzhou_moveit_config" / "config" / "joint_limits.yaml"
_OMPL_REL = _MODEL_ROOT / "ninezzhou_moveit_config" / "config" / "ompl_planning.yaml"
_CONTROLLERS_REL = _MODEL_ROOT / "ninezzhou_moveit_config" / "config" / "moveit_controllers.yaml"


def load_urdf(share_dir: Path) -> str:
    """Read the URDF and rewrite ``package://ninezzhou/meshes/`` to
    ``file://<mesh_directory>/`` so MoveIt can locate STL collision meshes
    without a standalone ``ninezzhou`` ROS package."""
    urdf_path = share_dir / _URDF_REL
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF not found at installed path: {urdf_path}")
    mesh_dir = (share_dir / _MESHES_REL).resolve()
    if not mesh_dir.is_dir():
        raise NotADirectoryError(f"Mesh directory not found: {mesh_dir}")
    urdf_text = urdf_path.read_text(encoding="utf-8")
    urdf_text = urdf_text.replace(
        "package://ninezzhou/meshes/", f"file://{mesh_dir}/"
    )
    return urdf_text


def load_srdf(share_dir: Path) -> str:
    """Read the semantic robot description (planning groups, collisions)."""
    srdf_path = share_dir / _SRDF_REL
    if not srdf_path.is_file():
        raise FileNotFoundError(f"SRDF not found: {srdf_path}")
    return srdf_path.read_text(encoding="utf-8")


def load_kinematics(share_dir: Path) -> dict:
    """Return the kinematics solver configuration as a dict."""
    path = share_dir / _KINEMATICS_REL
    if not path.is_file():
        raise FileNotFoundError(f"kinematics.yaml not found: {path}")
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def load_joint_limits(share_dir: Path) -> dict:
    """Return joint velocity/acceleration limits as a dict."""
    path = share_dir / _JOINT_LIMITS_REL
    if not path.is_file():
        raise FileNotFoundError(f"joint_limits.yaml not found: {path}")
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def load_ompl_planning(share_dir: Path) -> dict:
    """Return the OMPL planning pipeline configuration as a dict."""
    path = share_dir / _OMPL_REL
    if not path.is_file():
        raise FileNotFoundError(f"ompl_planning.yaml not found: {path}")
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def load_moveit_controllers(share_dir: Path) -> dict:
    """Return the MoveIt controller manager configuration as a dict."""
    path = share_dir / _CONTROLLERS_REL
    if not path.is_file():
        raise FileNotFoundError(f"moveit_controllers.yaml not found: {path}")
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

_FRAME_RE = re.compile(r'<link\s+name="(\w+)"')


def validate_urdf_frames(urdf_xml: str) -> None:
    """Raise ``ValueError`` with ``ROBOT_MODEL_FRAME_MISSING`` if *urdf_xml*
    does not contain ``base_link`` and ``tool0``."""
    links = set(_FRAME_RE.findall(urdf_xml))
    missing = []
    for required in ("base_link", "tool0"):
        if required not in links:
            missing.append(required)
    if missing:
        raise ValueError(
            f"ROBOT_MODEL_FRAME_MISSING: URDF is missing required "
            f"frame(s): {', '.join(missing)}"
        )


# ---------------------------------------------------------------------------
# High-level builder
# ---------------------------------------------------------------------------


def build_moveit_params(share_dir: Path) -> Dict[str, object]:
    """Return a ROS parameter dictionary suitable for ``move_group``.

    The returned dict contains the keys expected by the standard MoveIt 2
    launch infrastructure:

    * ``robot_description``            — URDF (string)
    * ``robot_description_semantic``   — SRDF (string)
    * ``robot_description_kinematics`` — kinematics solver config (dict)
    * ``robot_description_planning``   — joint limits (dict, NOT a file path)
    * ``ompl``                         — OMPL planning pipeline config (dict)
    * Controller manager keys          — spread from moveit_controllers.yaml

    Callers should merge this into the node's ``parameters`` list.
    ``robot_state_publisher`` only needs ``robot_description``.
    """
    urdf = load_urdf(share_dir)
    validate_urdf_frames(urdf)

    srdf = load_srdf(share_dir)
    kinematics = load_kinematics(share_dir)
    joint_limits = load_joint_limits(share_dir)
    ompl = load_ompl_planning(share_dir)
    controllers = load_moveit_controllers(share_dir)

    return {
        "robot_description": urdf,
        "robot_description_semantic": srdf,
        "robot_description_kinematics": kinematics,
        "robot_description_planning": joint_limits,
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
        **controllers,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
