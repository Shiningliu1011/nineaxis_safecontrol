"""Single source of truth for simulation and shadow joint-velocity limits.

This module reads the actuator reference YAML once during robot construction.
It deliberately separates a useful simulation limit from a hardware-approved
limit: J1 is linear while its supplied actuator data is rotational, so its
``0.5 m/s`` value cannot authorize a real command path.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


JOINT_NAMES = tuple(f"J{index}" for index in range(1, 10))
_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "actuator_modules.yaml"


@dataclass(frozen=True)
class ActuatorLimitProfile:
    """Derived joint limits with their hardware-validation boundary.

    Units are ``m/s`` and ``m/s^2`` for J1, then ``rad/s`` and ``rad/s^2`` for
    J2-J9.  ``velocity_limits`` and ``acceleration_limits`` are only safe for
    simulation/shadow validation until every joint is hardware approved.
    """

    joint_names: tuple[str, ...]
    velocity_limits: np.ndarray
    acceleration_limits: np.ndarray
    hardware_validated: np.ndarray
    source_path: Path
    safety_factor: float

    def __post_init__(self) -> None:
        velocity = np.asarray(self.velocity_limits, dtype=float).reshape(len(self.joint_names))
        acceleration = np.asarray(self.acceleration_limits, dtype=float).reshape(len(self.joint_names))
        validated = np.asarray(self.hardware_validated, dtype=bool).reshape(len(self.joint_names))
        if np.any(~np.isfinite(velocity)) or np.any(velocity <= 0.0):
            raise ValueError("velocity_limits must be finite and positive")
        if np.any(~np.isfinite(acceleration)) or np.any(acceleration <= 0.0):
            raise ValueError("acceleration_limits must be finite and positive")
        object.__setattr__(self, "velocity_limits", velocity)
        object.__setattr__(self, "acceleration_limits", acceleration)
        object.__setattr__(self, "hardware_validated", validated)

    @property
    def j1_simulation_only(self) -> bool:
        return not bool(self.hardware_validated[0])

    @property
    def speed_source_verified(self) -> np.ndarray:
        """Whether each speed limit has a documented derivation source.

        This is intentionally narrower than full hardware readiness.  J2-J9
        have a rotary rated-speed conversion, while J1 still lacks the linear
        transmission evidence required for any executable hardware path.
        """
        return self.hardware_validated.copy()

    @property
    def hardware_executable(self) -> bool:
        return bool(np.all(self.hardware_validated))

    @property
    def unverified_joint_names(self) -> tuple[str, ...]:
        return tuple(
            joint for joint, validated in zip(self.joint_names, self.hardware_validated)
            if not validated)

    def require_hardware_executable(self) -> None:
        """Reject a future real command path before it reaches a driver."""
        if self.hardware_executable:
            return
        names = ", ".join(self.unverified_joint_names)
        raise RuntimeError(
            "hardware command mode is blocked because actuator limits are not "
            f"validated for: {names}")


@dataclass(frozen=True)
class VelocityLayerAudit:
    """Audit nominal, QP-candidate and actuator commands without clipping."""

    nominal_max_ratio: float
    qp_candidate_max_ratio: float
    actuator_max_ratio: float
    nominal_within_limits: bool
    qp_candidate_within_limits: bool
    actuator_within_limits: bool

    def metrics(self) -> dict[str, float]:
        return {
            "velocity_limit_nominal_max_ratio": float(self.nominal_max_ratio),
            "velocity_limit_qp_candidate_max_ratio": float(self.qp_candidate_max_ratio),
            "velocity_limit_actuator_max_ratio": float(self.actuator_max_ratio),
            "velocity_limit_nominal_ok": float(self.nominal_within_limits),
            "velocity_limit_qp_candidate_ok": float(self.qp_candidate_within_limits),
            "velocity_limit_actuator_ok": float(self.actuator_within_limits),
        }


def load_actuator_limit_profile(config_path: str | Path | None = None) -> ActuatorLimitProfile:
    """Derive the current simulation/shadow limits from actuator reference data."""
    path = _DEFAULT_CONFIG_PATH if config_path is None else Path(config_path).expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"actuator limit config does not exist: {path}") from exc
    if not isinstance(data, dict):
        raise ValueError("actuator limit config must contain a mapping")

    mapping = data.get("joint_module_mapping")
    modules = data.get("modules")
    profile = data.get("control_limit_profile")
    if not isinstance(mapping, dict) or not isinstance(modules, dict) or not isinstance(profile, dict):
        raise ValueError("actuator config is missing joint_module_mapping, modules, or control_limit_profile")
    if int(profile.get("schema_version", -1)) != 1:
        raise ValueError("unsupported control_limit_profile schema_version")

    safety_factor = float(profile.get("rated_speed_safety_factor", 0.0))
    if not 0.0 < safety_factor <= 1.0:
        raise ValueError("rated_speed_safety_factor must be in (0, 1]")
    j1_profile = profile.get("J1")
    if not isinstance(j1_profile, dict):
        raise ValueError("control_limit_profile.J1 must be a mapping")
    j1_velocity = float(j1_profile.get("simulation_velocity_limit_m_s", 0.0))
    j1_acceleration = float(j1_profile.get("simulation_acceleration_limit_m_s2", 0.0))
    rotary_acceleration = float(profile.get("revolute_simulation_acceleration_limit_rad_s2", 0.0))
    if min(j1_velocity, j1_acceleration, rotary_acceleration) <= 0.0:
        raise ValueError("simulation velocity and acceleration limits must be positive")

    velocity = [j1_velocity]
    acceleration = [j1_acceleration]
    validated = [bool(j1_profile.get("hardware_validated", False))]
    for joint in JOINT_NAMES[1:]:
        info = mapping.get(joint)
        if not isinstance(info, dict):
            raise ValueError(f"joint_module_mapping.{joint} is missing")
        module_name = info.get("module")
        module = modules.get(module_name)
        if not isinstance(module, dict) or not isinstance(module.get("derived"), dict):
            raise ValueError(f"module data is missing for {joint}: {module_name}")
        rated_speed = float(module["derived"].get("rated_speed_rad_s", 0.0))
        if rated_speed <= 0.0:
            raise ValueError(f"rated_speed_rad_s must be positive for {module_name}")
        velocity.append(rated_speed * safety_factor)
        acceleration.append(rotary_acceleration)
        # This acknowledges only the rotary speed-source conversion.  It does
        # not make the complete robot hardware-ready, because J1 remains false.
        validated.append(True)

    return ActuatorLimitProfile(
        joint_names=JOINT_NAMES,
        velocity_limits=np.asarray(velocity, dtype=float),
        acceleration_limits=np.asarray(acceleration, dtype=float),
        hardware_validated=np.asarray(validated, dtype=bool),
        source_path=path,
        safety_factor=safety_factor,
    )


def audit_velocity_layers(profile: ActuatorLimitProfile, nominal: Iterable[float],
                          qp_candidate: Iterable[float], actuator: Iterable[float],
                          *, tolerance: float = 1.0e-9) -> VelocityLayerAudit:
    """Measure all three command layers; never alter a command for diagnostics."""
    limits = profile.velocity_limits
    values = [
        np.asarray(layer, dtype=float).reshape(limits.shape)
        for layer in (nominal, qp_candidate, actuator)
    ]
    ratios = [
        (float("inf") if not np.all(np.isfinite(layer))
         else float(np.max(np.abs(layer) / limits)))
        for layer in values
    ]
    within = [ratio <= 1.0 + float(tolerance) for ratio in ratios]
    return VelocityLayerAudit(
        nominal_max_ratio=ratios[0],
        qp_candidate_max_ratio=ratios[1],
        actuator_max_ratio=ratios[2],
        nominal_within_limits=within[0],
        qp_candidate_within_limits=within[1],
        actuator_within_limits=within[2],
    )


def command_delta_limits(profile: ActuatorLimitProfile, *, dt_s: float,
                         acceleration_scale: float = 1.0) -> np.ndarray:
    """Derive per-command velocity changes from the profile acceleration.

    The returned units are ``m/s`` for J1 and ``rad/s`` for J2-J9.  The
    acceleration values remain simulation/shadow assumptions until the motor,
    transmission and drive-controller limits have been measured on hardware.
    """
    dt = float(dt_s)
    scale = float(acceleration_scale)
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError('dt_s must be finite and positive')
    if not np.isfinite(scale) or not 0.0 < scale <= 1.0:
        raise ValueError('acceleration_scale must be in (0, 1]')
    return profile.acceleration_limits * dt * scale
