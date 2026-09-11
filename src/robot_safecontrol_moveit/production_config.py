"""Strict production-configuration contract for the OSCBF ROS adapter.

The ROS parameter file is loaded here, before controller resources or command
publishers are created.  Launch/CLI parameter overrides remain a separate
input so a required YAML value cannot be supplied accidentally by an override.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

import yaml

from .robot_spec import DEFAULT_JOINT_NAMES


REQUIRED_PARAMETERS = frozenset(
    {
        "dt",
        "dt_path",
        "publish_frequency_hz",
        "kp_pos",
        "kp_orient",
        "kp_joint",
        "damping",
        "w_pos",
        "w_orient",
        "w_joint",
        "temporal_lambda",
        "enable_x64",
        "solver_tol",
        "task_mode",
        "use_nullspace_policy",
        "reference_feedrate_scale",
        "nullspace_speed_limit",
        "max_tool_axis_speed_rad_s",
        "reference_lead_m",
        "orientation_mode",
        "cylinder_axis_direction",
        "wait_for_start",
        "enable_perception_obstacles",
        "joint_names",
        "joint_state_topic",
        "publish_joint_state_topic",
        "perception_tracks_topic",
        "trajectory_mat",
        "portable_oscbf_root",
        "portable_config_yaml",
    }
)

OPTIONAL_DEFAULTS: Mapping[str, Any] = {
    "cylinder_center": [],
    "telemetry_period_s": 1.0,
    "perf_report_path": "output/oscbf_m10_perf.md",
    "latency_budget_ms": 20.0,
}

MANAGED_PARAMETERS = REQUIRED_PARAMETERS | frozenset(OPTIONAL_DEFAULTS)

LEGACY_CONTROL_PARAMETERS = frozenset(
    {
        "alpha_joint_limit",
        "alpha_collision",
        "alpha_singularity",
        "cbf_alpha",
        "controller",
        "kp_pos_fixed",
        "kp_pos_non_fixed",
        "kp_orient_fixed",
        "kp_orient_non_fixed",
        "mode",
        "task_tracking",
        "nullspace",
        "cbf",
        "qp",
        "torque_weight",
    }
)


@dataclass(frozen=True)
class ProductionProfile:
    """Validated YAML base values and the bytes that identify their source."""

    path: Path
    content: bytes
    values: Mapping[str, Any]
    yaml_names: frozenset[str]


@dataclass(frozen=True)
class EffectiveConfiguration:
    """Resolved immutable values plus their complete startup provenance."""

    values: Mapping[str, Any]
    sources: Mapping[str, str]
    override_chains: Mapping[str, list[dict[str, Any]]]
    resources: Mapping[str, Mapping[str, str]]

    def diagnostics(self) -> dict[str, Any]:
        """Return a JSON-compatible view of values and provenance."""
        return {
            "values": dict(self.values),
            "sources": dict(self.sources),
            "override_chains": {
                name: [dict(entry) for entry in chain]
                for name, chain in self.override_chains.items()
            },
            "resources": {
                name: dict(resource)
                for name, resource in self.resources.items()
            },
        }


def _reject_unrecognized_parameters(
    names, *, allowed: frozenset[str], source: str
) -> None:
    unknown = sorted(set(names) - allowed)
    if not unknown:
        return
    legacy = sorted(set(unknown) & LEGACY_CONTROL_PARAMETERS)
    if legacy:
        raise ValueError(
            f"{source} contains legacy, unconsumed control parameters: "
            + ", ".join(legacy)
        )
    raise ValueError(
        f"{source} contains unknown parameters: " + ", ".join(unknown)
    )


def load_production_profile(path: Path, *, node_name: str) -> ProductionProfile:
    """Load the ROS parameter block for ``node_name`` and check its schema."""
    source = path.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"production config not found or unreadable: {source}")
    try:
        content = source.read_bytes()
    except OSError as exc:
        raise OSError(f"production config unreadable: {source}: {exc}") from exc
    try:
        document = yaml.safe_load(content)
    except yaml.YAMLError as exc:
        raise ValueError(f"production config cannot be parsed: {source}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"production config root must be a mapping: {source}")

    values: dict[str, Any] = {}
    for selector in ("/**", node_name, f"/{node_name}"):
        section = document.get(selector)
        if section is None:
            continue
        if not isinstance(section, dict) or not isinstance(
            section.get("ros__parameters"), dict
        ):
            raise ValueError(
                "production config section must contain a ros__parameters mapping: "
                f"{selector} in {source}"
            )
        values.update(section["ros__parameters"])
    if not values:
        raise ValueError(
            f"production config has no parameters for node {node_name!r}: {source}"
        )

    missing = sorted(REQUIRED_PARAMETERS - values.keys())
    if missing:
        raise ValueError(
            "production config missing required parameters: " + ", ".join(missing)
        )
    _reject_unrecognized_parameters(
        values, allowed=MANAGED_PARAMETERS, source="production config"
    )

    merged = dict(OPTIONAL_DEFAULTS)
    merged.update(values)
    validate_parameter_values(merged, source="production config")
    return ProductionProfile(
        path=source,
        content=content,
        values=MappingProxyType(merged),
        yaml_names=frozenset(values),
    )


def _number(name: str, value: Any, *, source: str) -> float:
    if type(value) is not float:
        raise ValueError(f"{source}: {name} must be a floating-point number")
    if not math.isfinite(value):
        raise ValueError(f"{source}: {name} must be finite")
    return value


def _number_vector(
    name: str, value: Any, *, lengths: set[int], source: str
) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) not in lengths:
        sizes = " or ".join(str(length) for length in sorted(lengths))
        raise ValueError(f"{source}: {name} must contain {sizes} numbers")
    return [_number(name, item, source=source) for item in value]


def validate_parameter_values(values: Mapping[str, Any], *, source: str) -> None:
    """Validate strict types, finite values and existing consumer domains."""
    for name in (
        "enable_x64",
        "use_nullspace_policy",
        "wait_for_start",
        "enable_perception_obstacles",
    ):
        if type(values[name]) is not bool:
            raise ValueError(f"{source}: {name} must be a boolean")

    numeric = {
        name: _number(name, values[name], source=source)
        for name in (
            "dt",
            "dt_path",
            "publish_frequency_hz",
            "kp_pos",
            "kp_orient",
            "kp_joint",
            "damping",
            "w_pos",
            "w_orient",
            "w_joint",
            "temporal_lambda",
            "solver_tol",
            "reference_feedrate_scale",
            "nullspace_speed_limit",
            "max_tool_axis_speed_rad_s",
            "reference_lead_m",
            "telemetry_period_s",
            "latency_budget_ms",
        )
    }
    if not 0.0 < numeric["dt"] <= 0.1:
        raise ValueError(f"{source}: dt must be in (0, 0.1]")
    if numeric["dt_path"] <= 0.0:
        raise ValueError(f"{source}: dt_path must be positive")
    if not 1.0 <= numeric["publish_frequency_hz"] <= 1000.0:
        raise ValueError(
            f"{source}: publish_frequency_hz must be in [1, 1000]"
        )
    for name in (
        "kp_pos",
        "kp_orient",
        "kp_joint",
        "damping",
        "w_pos",
        "w_orient",
        "w_joint",
        "reference_feedrate_scale",
        "nullspace_speed_limit",
        "max_tool_axis_speed_rad_s",
        "telemetry_period_s",
        "latency_budget_ms",
    ):
        if numeric[name] <= 0.0:
            raise ValueError(f"{source}: {name} must be positive")
    if numeric["temporal_lambda"] < 0.0:
        raise ValueError(f"{source}: temporal_lambda must be non-negative")
    if numeric["reference_lead_m"] <= 0.0:
        raise ValueError(f"{source}: reference_lead_m must be positive")
    if not 0.0 < numeric["solver_tol"] < 1.0:
        raise ValueError(f"{source}: solver_tol must be in (0, 1)")

    if values["task_mode"] not in ("pose6d", "tool_axis_5d"):
        raise ValueError(f"{source}: unsupported task_mode {values['task_mode']!r}")
    if values["orientation_mode"] not in ("fixed", "surface_normal"):
        raise ValueError(
            f"{source}: unsupported orientation_mode {values['orientation_mode']!r}"
        )
    for name in (
        "task_mode",
        "orientation_mode",
        "joint_state_topic",
        "publish_joint_state_topic",
        "perception_tracks_topic",
        "trajectory_mat",
        "portable_oscbf_root",
        "portable_config_yaml",
        "perf_report_path",
    ):
        if not isinstance(values[name], str):
            raise ValueError(f"{source}: {name} must be a string")
    for name in (
        "joint_state_topic",
        "publish_joint_state_topic",
        "perception_tracks_topic",
        "perf_report_path",
    ):
        if not values[name].strip():
            raise ValueError(f"{source}: {name} must not be empty")

    joint_names = values["joint_names"]
    if (
        not isinstance(joint_names, (list, tuple))
        or any(not isinstance(name, str) for name in joint_names)
        or tuple(joint_names) != DEFAULT_JOINT_NAMES
    ):
        raise ValueError(
            f"{source}: joint_names must match the canonical order "
            f"{list(DEFAULT_JOINT_NAMES)!r}"
        )

    axis = _number_vector(
        "cylinder_axis_direction",
        values["cylinder_axis_direction"],
        lengths={3},
        source=source,
    )
    if math.sqrt(sum(component * component for component in axis)) <= 0.0:
        raise ValueError(
            f"{source}: cylinder_axis_direction must be a non-zero 3-vector"
        )
    _number_vector(
        "cylinder_center", values["cylinder_center"], lengths={0, 3}, source=source
    )


def _absolute_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def build_effective_configuration(
    profile: ProductionProfile,
    explicit_overrides: Mapping[str, Any],
    *,
    share_dir: Path,
) -> EffectiveConfiguration:
    """Merge explicit ROS overrides, resolve resources and retain provenance."""
    _reject_unrecognized_parameters(
        explicit_overrides,
        allowed=MANAGED_PARAMETERS,
        source="explicit override",
    )

    values = dict(profile.values)
    values.update(explicit_overrides)
    if explicit_overrides:
        validate_parameter_values(values, source="explicit override")

    sources: dict[str, str] = {}
    chains: dict[str, list[dict[str, Any]]] = {}
    for name in MANAGED_PARAMETERS:
        initial_source = (
            "production_yaml" if name in profile.yaml_names else "optional_default"
        )
        chain = [{"source": initial_source, "value": profile.values[name]}]
        if name in explicit_overrides:
            chain.append(
                {"source": "explicit_override", "value": explicit_overrides[name]}
            )
            sources[name] = "explicit_override"
        else:
            sources[name] = initial_source
        chains[name] = chain

    defaults = {
        "trajectory_mat": share_dir / "data" / "nurbs" / "ik_input.mat",
        "portable_oscbf_root": share_dir / "portable_oscbf",
        "portable_config_yaml": share_dir
        / "portable_oscbf"
        / "config"
        / "nineaxis.yaml",
    }
    resources: dict[str, Mapping[str, str]] = {}
    for name, fallback in defaults.items():
        raw = values[name]
        resolved = fallback.resolve() if raw == "" else _absolute_path(raw)
        kind = "directory" if name == "portable_oscbf_root" else "file"
        exists = resolved.is_dir() if kind == "directory" else resolved.is_file()
        if not exists or not os.access(resolved, os.R_OK):
            raise FileNotFoundError(
                f"{name} {kind} not found or unreadable: {resolved} "
                f"(raw value {raw!r}, source {sources[name]})"
            )
        values[name] = str(resolved)
        resources[name] = MappingProxyType(
            {
                "raw_value": raw,
                "resolved_path": str(resolved),
                "value_source": sources[name],
                "resolution_source": "share_default" if raw == "" else "configured_path",
            }
        )

    return EffectiveConfiguration(
        values=MappingProxyType(values),
        sources=MappingProxyType(sources),
        override_chains=MappingProxyType(chains),
        resources=MappingProxyType(resources),
    )
