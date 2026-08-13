"""Offline contract for a validated 5-D safe null-space posture.

``q_safe_center`` is not the arithmetic midpoint of the joint limits.  It is
an explicitly recorded redundant configuration which has passed the current
simulation checks for one 5-D task pose.  This module intentionally stays out
of the JAX control loop: FCL mesh queries and YAML loading are startup/offline
work, not 100 Hz work.

The validation proves only the configured pose under the configured collision
models.  In particular, it does *not* certify clearance to live obstacles,
the startup transition, a complete process path, or real hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import jax.numpy as jnp
import numpy as np
import yaml

from work.fcl_collision_mesh import FclMeshSelfCollisionChecker, LINK_MESH_FILES
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.oscbf_collision_config import SELF_COLLISION_PAIRS, compute_self_collision_h
from work.task_mode_contract import TASK_MODE_TOOL_AXIS_5D
from work.tool_axis_task import TOOL_AXIS_INDEX, task_jacobian_5d


SAFE_POSTURE_SCHEMA_VERSION = 1
_NUM_JOINTS = 9


@dataclass(frozen=True)
class SafePostureContract:
    """Thresholds for one offline posture validation.

    All distances are metres.  ``minimum_legacy_self_collision_h_m`` is the
    residual of the currently active 17-sphere self-collision CBF after
    subtracting ``legacy_self_collision_d_safe_m``; it is not an FCL distance.
    ``minimum_task_sigma`` is the least singular value of the 5x9 task
    Jacobian, whose mixed units are inherited from position and radians.
    """

    minimum_mesh_clearance_m: float
    legacy_self_collision_d_safe_m: float
    minimum_legacy_self_collision_h_m: float
    minimum_joint_limit_margin_fraction: float
    minimum_task_sigma: float
    reference_position_tolerance_m: float
    reference_tool_axis_tolerance_deg: float
    mesh_max_faces: int

    def __post_init__(self) -> None:
        positive_values = {
            'minimum_mesh_clearance_m': self.minimum_mesh_clearance_m,
            'legacy_self_collision_d_safe_m': self.legacy_self_collision_d_safe_m,
            'minimum_legacy_self_collision_h_m': self.minimum_legacy_self_collision_h_m,
            'minimum_task_sigma': self.minimum_task_sigma,
            'reference_position_tolerance_m': self.reference_position_tolerance_m,
            'reference_tool_axis_tolerance_deg': self.reference_tool_axis_tolerance_deg,
        }
        for name, value in positive_values.items():
            if not np.isfinite(value) or float(value) <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        margin = float(self.minimum_joint_limit_margin_fraction)
        if not np.isfinite(margin) or not 0.0 < margin < 0.5:
            raise ValueError('minimum_joint_limit_margin_fraction must be in (0, 0.5)')
        if int(self.mesh_max_faces) <= 0:
            raise ValueError('mesh_max_faces must be positive')


@dataclass(frozen=True)
class SafePostureProfile:
    """One named, task-specific redundant posture candidate.

    The reference stores only the controlled 5-D task components: Cartesian
    position and the tool X axis in the world frame.  It deliberately does not
    constrain roll about the tool axis.
    """

    name: str
    description: str
    task_mode: str
    frame_id: str
    q_safe_center: np.ndarray
    reference_position_m: np.ndarray
    reference_tool_axis_world: np.ndarray
    contract: SafePostureContract
    source_path: Path

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError('safe posture profile name must not be empty')
        if self.task_mode != TASK_MODE_TOOL_AXIS_5D:
            raise ValueError(
                'safe posture profiles currently require task_mode=tool_axis_5d')
        if not self.frame_id:
            raise ValueError('safe posture profile frame_id must not be empty')

        q = _readonly_vector(self.q_safe_center, 'q_safe_center', _NUM_JOINTS)
        position = _readonly_vector(self.reference_position_m, 'reference_position_m', 3)
        axis = _readonly_vector(self.reference_tool_axis_world, 'reference_tool_axis_world', 3)
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm <= 1.0e-12:
            raise ValueError('reference_tool_axis_world must be non-zero')
        axis = np.array(axis / axis_norm, dtype=float)
        axis.setflags(write=False)
        object.__setattr__(self, 'q_safe_center', q)
        object.__setattr__(self, 'reference_position_m', position)
        object.__setattr__(self, 'reference_tool_axis_world', axis)
        object.__setattr__(self, 'source_path', Path(self.source_path).resolve())


@dataclass(frozen=True)
class SafePostureValidation:
    """Auditable result from :func:`validate_safe_posture_profile`."""

    profile_name: str
    profile_path: str
    passed: bool
    failure_reasons: tuple[str, ...]
    mesh_min_clearance_m: float
    mesh_closest_pair: tuple[str, str]
    legacy_self_collision_h_min_m: float
    legacy_self_collision_closest_pair: tuple[int, int]
    minimum_joint_limit_margin_fraction: float
    task_sigma_min: float
    reference_position_error_m: float
    reference_tool_axis_error_deg: float
    task_jacobian_shape: tuple[int, int]

    def to_dict(self) -> dict[str, Any]:
        """Return JSON-safe validation data with explicit physical units."""
        return {
            'profile_name': self.profile_name,
            'profile_path': self.profile_path,
            'passed': bool(self.passed),
            'failure_reasons': list(self.failure_reasons),
            'mesh_min_clearance_m': float(self.mesh_min_clearance_m),
            'mesh_closest_pair': list(self.mesh_closest_pair),
            'legacy_self_collision_h_min_m': float(self.legacy_self_collision_h_min_m),
            'legacy_self_collision_closest_pair': list(self.legacy_self_collision_closest_pair),
            'minimum_joint_limit_margin_fraction': float(
                self.minimum_joint_limit_margin_fraction),
            'task_sigma_min': float(self.task_sigma_min),
            'reference_position_error_m': float(self.reference_position_error_m),
            'reference_tool_axis_error_deg': float(self.reference_tool_axis_error_deg),
            'task_jacobian_shape': list(self.task_jacobian_shape),
        }


def _readonly_vector(value: Any, name: str, size: int) -> np.ndarray:
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.shape != (size,):
        raise ValueError(f'{name} must contain {size} values, got shape {vector.shape}')
    if not np.all(np.isfinite(vector)):
        raise ValueError(f'{name} must contain only finite values')
    copied = vector.copy()
    copied.setflags(write=False)
    return copied


def _require_mapping(data: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = data.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f'{name} must be a mapping')
    return value


def _require_value(mapping: Mapping[str, Any], name: str) -> Any:
    if name not in mapping:
        raise ValueError(f'missing required profile field: {name}')
    return mapping[name]


def load_safe_posture_profile(path: str | Path) -> SafePostureProfile:
    """Load a versioned safe-posture contract from YAML.

    Loading validates schema and shapes only.  Call
    :func:`validate_safe_posture_profile` to run FCL, CBF, limit and 5-D
    kinematic checks against the current source tree.
    """
    profile_path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise ValueError(f'safe posture profile does not exist: {profile_path}') from exc
    if not isinstance(data, Mapping):
        raise ValueError('safe posture profile must contain a YAML mapping')
    if int(data.get('schema_version', -1)) != SAFE_POSTURE_SCHEMA_VERSION:
        raise ValueError('unsupported safe posture profile schema_version')

    reference = _require_mapping(data, 'reference_task')
    contract_data = _require_mapping(data, 'validation_contract')
    contract = SafePostureContract(
        minimum_mesh_clearance_m=float(
            _require_value(contract_data, 'minimum_mesh_clearance_m')),
        legacy_self_collision_d_safe_m=float(
            _require_value(contract_data, 'legacy_self_collision_d_safe_m')),
        minimum_legacy_self_collision_h_m=float(
            _require_value(contract_data, 'minimum_legacy_self_collision_h_m')),
        minimum_joint_limit_margin_fraction=float(
            _require_value(contract_data, 'minimum_joint_limit_margin_fraction')),
        minimum_task_sigma=float(_require_value(contract_data, 'minimum_task_sigma')),
        reference_position_tolerance_m=float(
            _require_value(contract_data, 'reference_position_tolerance_m')),
        reference_tool_axis_tolerance_deg=float(
            _require_value(contract_data, 'reference_tool_axis_tolerance_deg')),
        mesh_max_faces=int(_require_value(contract_data, 'mesh_max_faces')),
    )
    return SafePostureProfile(
        name=str(_require_value(data, 'name')),
        description=str(data.get('description', '')),
        task_mode=str(_require_value(data, 'task_mode')),
        frame_id=str(_require_value(data, 'frame_id')),
        q_safe_center=np.asarray(_require_value(data, 'q_safe_center'), dtype=float),
        reference_position_m=np.asarray(
            _require_value(reference, 'position_m'), dtype=float),
        reference_tool_axis_world=np.asarray(
            _require_value(reference, 'tool_axis_world'), dtype=float),
        contract=contract,
        source_path=profile_path,
    )


def normalized_joint_limit_margins(q: np.ndarray, q_min: np.ndarray,
                                   q_max: np.ndarray) -> np.ndarray:
    """Return each joint's fractional distance to its closer hard limit.

    A value of ``0.5`` is the geometric midpoint and ``0.0`` is on a limit.
    This is unitless, so J1's metres and J2--J9's radians can be compared
    without mixing them in one absolute-distance threshold.
    """
    value = np.asarray(q, dtype=float).reshape(-1)
    lower = np.asarray(q_min, dtype=float).reshape(-1)
    upper = np.asarray(q_max, dtype=float).reshape(-1)
    if value.shape != lower.shape or value.shape != upper.shape:
        raise ValueError('q, q_min, and q_max must have the same shape')
    span = upper - lower
    if np.any(~np.isfinite(value)) or np.any(~np.isfinite(span)) or np.any(span <= 0.0):
        raise ValueError('joint limits and q must be finite with positive spans')
    return np.minimum((value - lower) / span, (upper - value) / span)


def create_mesh_self_collision_checker(
        mesh_dir: str | Path, max_faces: int) -> FclMeshSelfCollisionChecker:
    """Build one offline FCL checker and verify that all calibrated meshes exist.

    The caller may reuse this object for a whole offline path validation.  It
    must never cross into the JAX/100 Hz control path: FCL mesh distance is a
    verification oracle here, not the runtime CBF geometry.
    """
    checker = FclMeshSelfCollisionChecker(str(mesh_dir), max_faces=max_faces)
    missing = sorted(set(LINK_MESH_FILES) - set(checker._mesh_objs))
    if missing:
        raise RuntimeError(
            'safe posture mesh validation is incomplete; missing mesh links: '
            + ', '.join(missing))
    return checker


def mesh_self_clearance(kin, q: np.ndarray, mesh_dir: str | Path,
                        max_faces: int, *,
                        checker: FclMeshSelfCollisionChecker | None = None
                        ) -> tuple[float, tuple[str, str]]:
    """Return the closest calibrated FCL mesh pair for one offline pose.

    Supplying a pre-built ``checker`` prevents an entire path validator from
    loading and simplifying the same meshes for every sample.  Its mesh
    directory and face limit remain the caller's explicit contract.
    """
    active_checker = (create_mesh_self_collision_checker(mesh_dir, max_faces)
                      if checker is None else checker)
    # A large activation distance requests every calibrated non-adjacent pair.
    # It is intentionally offline and must never be copied into the hot loop.
    pairs = active_checker.check(kin.forward_kinematics(q), activation_dist=10.0)
    if not pairs:
        raise RuntimeError('FCL mesh validation returned no calibrated collision pairs')
    closest = min(pairs, key=lambda item: item.distance)
    return float(closest.distance), (closest.name_i, closest.name_j)


def legacy_self_collision_h(q: np.ndarray, robot: NineaxisManipulatorJAX,
                            d_safe_m: float) -> tuple[float, tuple[int, int]]:
    """Return the active 17-sphere/14-pair CBF minimum for one offline pose.

    This value is the existing CBF residual ``h`` rather than an independent
    mesh distance.  It is kept beside FCL so an offline path report makes the
    distinction visible instead of treating the two collision models as
    interchangeable.
    """
    # Keep the caller's JAX dtype policy.  The full control facade enables
    # x64 before building its kernels; standalone offline validation may
    # intentionally run with JAX's default dtype and must not emit a false
    # promise that x64 was used.
    data = np.asarray(robot.self_collision_data(jnp.asarray(q)))
    pairs = np.asarray(SELF_COLLISION_PAIRS, dtype=int)
    h_values = np.asarray(compute_self_collision_h(
        jnp.asarray(data[:, :3]), jnp.asarray(data[:, 3]),
        jnp.asarray(pairs), float(d_safe_m)))
    if h_values.shape != (len(pairs),):
        raise RuntimeError('legacy self-collision CBF returned an unexpected shape')
    closest_index = int(np.argmin(h_values))
    return float(h_values[closest_index]), tuple(int(v) for v in pairs[closest_index])


def _tool_axis_error_deg(current_axis: np.ndarray, reference_axis: np.ndarray) -> float:
    current_norm = float(np.linalg.norm(current_axis))
    if current_norm <= 1.0e-12:
        raise RuntimeError('computed tool axis is zero')
    cosine = float(np.clip(
        np.dot(current_axis / current_norm, reference_axis), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def validate_safe_posture_profile(profile: SafePostureProfile, kin,
                                  mesh_dir: str | Path,
                                  *, robot: NineaxisManipulatorJAX | None = None
                                  ) -> SafePostureValidation:
    """Validate one profile against current kinematics and collision code.

    ``kin`` must be the same URDF-native kinematics factory used by the RViz
    runner.  The optional JAX robot avoids rebuilding its geometry when a
    caller validates several profiles, but does not change the result.
    """
    q = profile.q_safe_center
    q_limits = kin.joint_limits
    position, rotation = kin.ee_pose(q)
    full_jacobian = kin.compute_full_jacobian(q)
    task_jacobian = task_jacobian_5d(full_jacobian, rotation, rotation,
                                      TOOL_AXIS_INDEX)
    singular_values = np.linalg.svd(task_jacobian, compute_uv=False)
    if singular_values.shape != (5,):
        raise RuntimeError('5-D task Jacobian must have exactly five singular values')

    margins = normalized_joint_limit_margins(q, q_limits.q_min, q_limits.q_max)
    mesh_clearance, mesh_pair = mesh_self_clearance(
        kin, q, mesh_dir, profile.contract.mesh_max_faces)
    jax_robot = NineaxisManipulatorJAX() if robot is None else robot
    legacy_h, legacy_pair = legacy_self_collision_h(
        q, jax_robot, profile.contract.legacy_self_collision_d_safe_m)

    position_error = float(np.linalg.norm(position - profile.reference_position_m))
    axis_error_deg = _tool_axis_error_deg(
        rotation[:, TOOL_AXIS_INDEX], profile.reference_tool_axis_world)
    min_margin = float(np.min(margins))
    min_sigma = float(np.min(singular_values))
    reasons: list[str] = []
    if np.any(q < q_limits.q_min) or np.any(q > q_limits.q_max):
        reasons.append('q_safe_center is outside a hard joint limit')
    if min_margin < profile.contract.minimum_joint_limit_margin_fraction:
        reasons.append(
            'joint_limit_margin_fraction '
            f'{min_margin:.6f} < {profile.contract.minimum_joint_limit_margin_fraction:.6f}')
    if mesh_clearance < profile.contract.minimum_mesh_clearance_m:
        reasons.append(
            f'mesh_clearance_m {mesh_clearance:.6f} < '
            f'{profile.contract.minimum_mesh_clearance_m:.6f}')
    if legacy_h < profile.contract.minimum_legacy_self_collision_h_m:
        reasons.append(
            f'legacy_self_collision_h_m {legacy_h:.6f} < '
            f'{profile.contract.minimum_legacy_self_collision_h_m:.6f}')
    if min_sigma < profile.contract.minimum_task_sigma:
        reasons.append(
            f'task_sigma_min {min_sigma:.6f} < {profile.contract.minimum_task_sigma:.6f}')
    if position_error > profile.contract.reference_position_tolerance_m:
        reasons.append(
            f'reference_position_error_m {position_error:.6e} > '
            f'{profile.contract.reference_position_tolerance_m:.6e}')
    if axis_error_deg > profile.contract.reference_tool_axis_tolerance_deg:
        reasons.append(
            f'reference_tool_axis_error_deg {axis_error_deg:.6f} > '
            f'{profile.contract.reference_tool_axis_tolerance_deg:.6f}')

    return SafePostureValidation(
        profile_name=profile.name,
        profile_path=str(profile.source_path),
        passed=not reasons,
        failure_reasons=tuple(reasons),
        mesh_min_clearance_m=mesh_clearance,
        mesh_closest_pair=mesh_pair,
        legacy_self_collision_h_min_m=legacy_h,
        legacy_self_collision_closest_pair=legacy_pair,
        minimum_joint_limit_margin_fraction=min_margin,
        task_sigma_min=min_sigma,
        reference_position_error_m=position_error,
        reference_tool_axis_error_deg=axis_error_deg,
        task_jacobian_shape=tuple(int(v) for v in task_jacobian.shape),
    )
