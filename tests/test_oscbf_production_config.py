"""Production configuration contract for the OSCBF ROS entry point."""

from __future__ import annotations

import inspect
import sys
import json
from pathlib import Path

import pytest
import rclpy
from rclpy.context import Context
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _parameter(name: str, value):
    return rclpy.parameter.Parameter(name, value=value)


def _construct_with_config(config_path: Path):
    from robot_safecontrol_moveit.oscbf_controller import OscbfController

    context = Context()
    rclpy.init(context=context)
    try:
        return OscbfController(
            node_name="oscbf_config_contract",
            context=context,
            parameter_overrides=[
                _parameter("production_config_yaml", str(config_path)),
                _parameter("trajectory_mat", "/definitely/not/a/trajectory.mat"),
            ],
        )
    finally:
        if context.ok():
            rclpy.shutdown(context=context)


def _write_profile(tmp_path: Path, **changes) -> Path:
    document = yaml.safe_load(
        (REPO_ROOT / "config" / "oscbf_controller.yaml").read_text(
            encoding="utf-8"
        )
    )
    document["/**"]["ros__parameters"].update(changes)
    target = tmp_path / "production.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    return target


def test_missing_production_yaml_is_rejected_before_resource_construction(tmp_path):
    missing = tmp_path / "missing-production.yaml"

    with pytest.raises(FileNotFoundError, match=r"production config.*missing-production"):
        _construct_with_config(missing)


def test_unreadable_production_yaml_reports_its_source(tmp_path, monkeypatch):
    config = _write_profile(tmp_path)
    original_read_bytes = Path.read_bytes

    def _deny_read(path):
        if path.resolve() == config.resolve():
            raise PermissionError("permission denied by test")
        return original_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", _deny_read)
    with pytest.raises(OSError, match=r"production config unreadable.*production.yaml"):
        _construct_with_config(config)


def test_malformed_production_yaml_is_rejected(tmp_path):
    malformed = tmp_path / "malformed.yaml"
    malformed.write_text("/**: [unterminated\n", encoding="utf-8")

    with pytest.raises(ValueError, match=r"production config cannot be parsed"):
        _construct_with_config(malformed)


def test_direct_entry_failure_always_shuts_down_rclpy(tmp_path):
    from robot_safecontrol_moveit.oscbf_controller import main

    missing = tmp_path / "direct-entry-missing.yaml"
    with pytest.raises(FileNotFoundError, match="direct-entry-missing"):
        main(
            args=[
                "--ros-args",
                "-p",
                f"production_config_yaml:={missing}",
            ]
        )

    assert not rclpy.ok()


def test_missing_required_yaml_value_cannot_be_repaired_by_override(tmp_path):
    source = (REPO_ROOT / "config" / "oscbf_controller.yaml").read_text(
        encoding="utf-8"
    )
    incomplete = tmp_path / "incomplete.yaml"
    incomplete.write_text(source.replace("    dt: 0.01\n", ""), encoding="utf-8")

    with pytest.raises(ValueError, match=r"production config.*missing required.*dt"):
        _construct_with_config(incomplete)


def test_current_controller_unknown_field_is_rejected(tmp_path):
    config = _write_profile(tmp_path, kp_ps=160.0)

    with pytest.raises(ValueError, match=r"unknown.*kp_ps"):
        _construct_with_config(config)


def test_legacy_control_field_has_an_explicit_error(tmp_path):
    config = _write_profile(tmp_path, alpha_joint_limit=5.0)

    with pytest.raises(ValueError, match=r"legacy.*alpha_joint_limit"):
        _construct_with_config(config)


@pytest.mark.parametrize(
    ("name", "value", "message"),
    [
        ("dt", "0.01", "dt must be a number"),
        ("enable_x64", 1, "enable_x64 must be a boolean"),
        ("kp_pos", float("nan"), "kp_pos must be finite"),
        ("dt_path", 0.0, "dt_path must be positive"),
        ("temporal_lambda", -0.1, "temporal_lambda must be non-negative"),
        ("cylinder_axis_direction", [0.0, 0.0, 0.0], "non-zero 3-vector"),
    ],
)
def test_invalid_yaml_values_are_rejected_strictly(
    tmp_path, name, value, message
):
    config = _write_profile(tmp_path, **{name: value})

    with pytest.raises(ValueError, match=message):
        _construct_with_config(config)


def test_other_node_parameter_sections_are_outside_controller_validation(tmp_path):
    document = yaml.safe_load(
        (REPO_ROOT / "config" / "oscbf_controller.yaml").read_text(
            encoding="utf-8"
        )
    )
    document["/perception_bridge"] = {
        "ros__parameters": {"legal_perception_field": 42}
    }
    config = tmp_path / "multi-node.yaml"
    config.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(FileNotFoundError, match="trajectory_mat"):
        _construct_with_config(config)


def _fake_share(tmp_path: Path) -> Path:
    share = tmp_path / "share"
    (share / "data" / "nurbs").mkdir(parents=True)
    (share / "data" / "nurbs" / "ik_input.mat").write_bytes(b"trajectory")
    (share / "portable_oscbf" / "config").mkdir(parents=True)
    (share / "portable_oscbf" / "config" / "nineaxis.yaml").write_text(
        "kinematics: {}\n", encoding="utf-8"
    )
    return share


def test_override_cannot_repair_an_invalid_yaml_base(tmp_path):
    config = _write_profile(tmp_path, dt=0.0)
    from robot_safecontrol_moveit.production_config import (
        build_effective_configuration,
        load_production_profile,
    )

    with pytest.raises(ValueError, match=r"production config: dt"):
        profile = load_production_profile(config, node_name="oscbf_controller")
        build_effective_configuration(
            profile, {"dt": 0.01}, share_dir=_fake_share(tmp_path)
        )


def test_invalid_explicit_override_is_rejected_before_resources(tmp_path):
    config = _write_profile(tmp_path)
    from robot_safecontrol_moveit.production_config import (
        build_effective_configuration,
        load_production_profile,
    )

    profile = load_production_profile(config, node_name="oscbf_controller")
    with pytest.raises(ValueError, match=r"explicit override: dt_path must be finite"):
        build_effective_configuration(
            profile,
            {"dt_path": float("nan")},
            share_dir=_fake_share(tmp_path),
        )


@pytest.mark.parametrize(
    ("overrides", "expected_root", "expected_config"),
    [
        ({}, "share", "share"),
        ({"portable_oscbf_root": "custom-root"}, "custom-root", "share"),
        ({"portable_config_yaml": "custom-config"}, "share", "custom-config"),
        (
            {
                "portable_oscbf_root": "custom-root",
                "portable_config_yaml": "custom-config",
            },
            "custom-root",
            "custom-config",
        ),
    ],
)
def test_resource_resolution_preserves_root_only_share_fallback(
    tmp_path, overrides, expected_root, expected_config
):
    from robot_safecontrol_moveit.production_config import (
        build_effective_configuration,
        load_production_profile,
    )

    share = _fake_share(tmp_path)
    custom_root = tmp_path / "custom-root"
    custom_root.mkdir()
    custom_config = tmp_path / "custom-config.yaml"
    custom_config.write_text("kinematics: {}\n", encoding="utf-8")
    resolved_overrides = {
        name: (
            str(custom_root)
            if value == "custom-root"
            else str(custom_config)
        )
        for name, value in overrides.items()
    }
    profile = load_production_profile(
        REPO_ROOT / "config" / "oscbf_controller.yaml",
        node_name="oscbf_controller",
    )

    effective = build_effective_configuration(
        profile, resolved_overrides, share_dir=share
    )

    root = Path(effective.values["portable_oscbf_root"])
    config = Path(effective.values["portable_config_yaml"])
    assert root == (custom_root if expected_root == "custom-root" else share / "portable_oscbf")
    assert config == (
        custom_config
        if expected_config == "custom-config"
        else share / "portable_oscbf" / "config" / "nineaxis.yaml"
    )
    assert effective.resources["portable_config_yaml"]["raw_value"] == (
        str(custom_config) if "portable_config_yaml" in overrides else ""
    )


def test_effective_configuration_records_yaml_and_override_chain(tmp_path):
    from robot_safecontrol_moveit.production_config import (
        build_effective_configuration,
        load_production_profile,
    )

    profile = load_production_profile(
        REPO_ROOT / "config" / "oscbf_controller.yaml",
        node_name="oscbf_controller",
    )
    effective = build_effective_configuration(
        profile, {"kp_pos": 161.0}, share_dir=_fake_share(tmp_path)
    )

    assert effective.values["kp_pos"] == 161.0
    assert effective.sources["kp_pos"] == "explicit_override"
    assert effective.override_chains["kp_pos"] == [
        {"source": "production_yaml", "value": 160.0},
        {"source": "explicit_override", "value": 161.0},
    ]
    assert effective.sources["cylinder_center"] == "optional_default"


def test_portable_kinematics_remains_valid_with_legacy_controller_section():
    from robot_safecontrol_moveit.production_config import (
        build_effective_configuration,
        load_production_profile,
    )

    portable_config = REPO_ROOT / "portable_oscbf" / "config" / "nineaxis.yaml"
    document = yaml.safe_load(portable_config.read_text(encoding="utf-8"))
    assert "kinematics" in document
    assert "controller" in document

    profile = load_production_profile(
        REPO_ROOT / "config" / "oscbf_controller.yaml",
        node_name="oscbf_controller",
    )
    effective = build_effective_configuration(
        profile, {}, share_dir=REPO_ROOT
    )
    assert effective.values["portable_config_yaml"] == str(portable_config)


def test_offline_facade_keeps_its_independent_defaults():
    from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable

    bootstrap_portable(REPO_ROOT / "portable_oscbf")
    from work.jax_control_facade import JaxControlLoop

    signature = inspect.signature(JaxControlLoop)
    assert signature.parameters["dt"].default == 0.002
    assert signature.parameters["dt_path"].default is None
    assert signature.parameters["w_pos"].default == 20.0
    assert signature.parameters["task_mode"].default == "pose6d"


def test_atomic_runtime_snapshots_are_unique_and_leave_no_partial_files(tmp_path):
    from robot_safecontrol_moveit.production_config import persist_runtime_snapshot

    first = persist_runtime_snapshot({"final_values": {"dt": 0.01}}, tmp_path)
    second = persist_runtime_snapshot({"final_values": {"dt": 0.01}}, tmp_path)

    assert first != second
    assert json.loads(first.read_text(encoding="utf-8"))["snapshot_path"] == str(
        first.resolve()
    )
    assert json.loads(second.read_text(encoding="utf-8"))["snapshot_path"] == str(
        second.resolve()
    )
    assert not list(tmp_path.glob("*.tmp"))


def test_snapshot_failure_prevents_command_publisher_creation(monkeypatch):
    from types import SimpleNamespace

    import rclpy
    from rclpy.context import Context

    from robot_safecontrol_moveit.oscbf_controller import OscbfController

    def _fake_build_controller(node, portable_root):
        del portable_root
        node._loop = SimpleNamespace(
            _config=SimpleNamespace(obstacle_h_baseline_alpha=10.0)
        )
        node._surface_axis = None
        node._surface_centre = None
        node._surface_radius = None

    monkeypatch.setattr(OscbfController, "_build_controller", _fake_build_controller)
    timer_calls = []
    original_create_timer = rclpy.node.Node.create_timer

    def _record_timer(*args, **kwargs):
        timer_calls.append((args, kwargs))
        return original_create_timer(*args, **kwargs)

    monkeypatch.setattr(rclpy.node.Node, "create_timer", _record_timer)
    context = Context()
    rclpy.init(context=context, domain_id=171)
    probe = rclpy.create_node("snapshot_gate_probe", context=context)
    parameters = [
        rclpy.parameter.Parameter(
            "production_config_yaml",
            value=str(REPO_ROOT / "config" / "oscbf_controller.yaml"),
        ),
        rclpy.parameter.Parameter(
            "portable_oscbf_root", value=str(REPO_ROOT / "portable_oscbf")
        ),
        rclpy.parameter.Parameter(
            "trajectory_mat", value=str(REPO_ROOT / "data" / "nurbs" / "ik_input.mat")
        ),
        rclpy.parameter.Parameter(
            "portable_config_yaml",
            value=str(REPO_ROOT / "portable_oscbf" / "config" / "nineaxis.yaml"),
        ),
        rclpy.parameter.Parameter(
            "perf_report_path", value="/proc/oscbf-snapshot-denied/perf.md"
        ),
    ]

    try:
        with pytest.raises(RuntimeError, match="snapshot persistence failed"):
            OscbfController(context=context, parameter_overrides=parameters)
        assert probe.count_publishers("/oscbf_command") == 0
        assert not timer_calls
    finally:
        probe.destroy_node()
        rclpy.shutdown(context=context)
