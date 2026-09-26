from __future__ import annotations

from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import jax
import jax.numpy as jnp
import numpy as np
import pytest
from jsonschema import ValidationError

from collision_parameter_inputs import DEVICE, SCENARIO, parameter_payload
from work.collision_parameters import CollisionParameterArtifact, CollisionPolicy
from work.collision_safety import CollisionSafety, CollisionSafetyConfig, CollisionStatus, QueryMode
from test_collision_safety import _config, _identities, _query_batch, _scene, _segment_batch


def _artifact(payload: dict | None = None) -> CollisionParameterArtifact:
    return CollisionParameterArtifact.create(
        parameter_payload() if payload is None else payload, device=DEVICE, scenario=SCENARIO
    )


def _policy(artifact: CollisionParameterArtifact) -> CollisionPolicy:
    return CollisionPolicy(artifact, b"g" * 32, b"k" * 32)


def _load(path: Path) -> CollisionParameterArtifact:
    return CollisionParameterArtifact.load(path, device=DEVICE, scenario=SCENARIO)


def test_json_roundtrip_and_independent_process_identity(tmp_path: Path) -> None:
    artifact = _artifact()
    document = artifact.to_document()
    expected = parameter_payload()
    for name, record in expected["parameters"].items():
        assert document["parameters"][name] == record
        assert type(document["parameters"][name]["value"]) is type(record["value"])
    path = tmp_path / "parameters.json"
    path.write_text(json.dumps(document, indent=2), encoding="utf-8")
    reordered = tmp_path / "reordered.json"
    reordered.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")
    assert _load(path).parameter_hash == _load(reordered).parameter_hash == artifact.parameter_hash

    environment = os.environ.copy()
    project = Path(__file__).resolve().parents[1]
    environment["PYTHONPATH"] = f"{project}:{environment.get('PYTHONPATH', '')}"
    completed = subprocess.run(
        [sys.executable, "-c", (
            "import sys; from work.collision_parameters import CollisionParameterArtifact, CollisionPolicy; "
            "a = CollisionParameterArtifact.load(sys.argv[1], device=sys.argv[2], scenario=sys.argv[3]); "
            "print(a.parameter_hash.hex()); print(CollisionPolicy(a, b'g'*32, b'k'*32).collision_policy_hash.hex()); "
            "assert 'rclpy' not in sys.modules"
        ), str(path), DEVICE, SCENARIO],
        env=environment, check=True, capture_output=True, text=True,
    )
    assert completed.stdout.splitlines() == [artifact.parameter_hash.hex(), _policy(artifact).collision_policy_hash.hex()]


@pytest.mark.parametrize("name", list(parameter_payload()["parameters"]))
def test_each_safety_value_changes_policy_and_rejects_old_identity(name: str) -> None:
    payload = parameter_payload()
    record = payload["parameters"][name]
    if name == "control_deadline_ns":
        record["value"] -= 1
    else:
        record["value"] += 1
    changed = _policy(_artifact(payload))
    config = _config()
    assert changed.artifact.parameter_hash != config.policy.artifact.parameter_hash
    assert changed.collision_policy_hash != config.collision_policy_hash
    module = CollisionSafety(config)
    identities = _identities(config)._replace(
        collision_policy_hash=jnp.asarray(np.frombuffer(changed.collision_policy_hash, dtype=np.uint8))
    )
    assert int(module.prepare_scene(_scene(), identities).status) == CollisionStatus.INVALID_SCENE


@pytest.mark.parametrize("field", [
    "source", "measurement_method", "calculation_method", "devices", "scenarios",
    "confidence_requirement", "validation",
])
def test_provenance_changes_parameter_and_policy_identity(field: str) -> None:
    payload = parameter_payload()
    record = payload["parameters"]["lidar_range_error_mm"]
    if field == "source":
        record[field]["description"] += "；第二组解析输入"
    elif field == "validation":
        record[field]["evidence"] += ":additional-check"
    elif field in ("devices", "scenarios"):
        record[field].append("additional-analytic-domain")
    else:
        record[field] += "；增加检查条件"
    assert _artifact(payload).parameter_hash != _artifact().parameter_hash
    assert _policy(_artifact(payload)).collision_policy_hash != _policy(_artifact()).collision_policy_hash


@pytest.mark.parametrize("field", ["name", "version", "method"])
def test_generation_metadata_changes_identity(field: str) -> None:
    payload = parameter_payload()
    payload["generator"][field] += "-revised"
    assert _policy(_artifact(payload)).collision_policy_hash != _policy(_artifact()).collision_policy_hash


def test_input_data_and_pair_clearance_change_identity() -> None:
    original = _policy(_artifact())
    payload = parameter_payload()
    new_hash = hashlib.sha256(b"another independent analytic input").hexdigest()
    payload["input_data_hashes"].append(new_hash)
    payload["parameters"]["lidar_range_error_mm"]["validation"]["data_hashes"].append(new_hash)
    assert _policy(_artifact(payload)).collision_policy_hash != original.collision_policy_hash
    payload = parameter_payload()
    payload["self_clearance_mm"][0]["clearance"]["value"] += 1
    assert _policy(_artifact(payload)).collision_policy_hash != original.collision_policy_hash


def test_policy_includes_geometry_kernel_and_full_allowed_contact_record() -> None:
    original = _policy(_artifact())
    assert replace(original, geometry_hash=b"x" * 32).collision_policy_hash != original.collision_policy_hash
    assert replace(original, kernel_version=b"x" * 32).collision_policy_hash != original.collision_policy_hash
    contact = {
        "links": ["base_link", "Link1"], "reason": "解析 identity 输入",
        "evidence": "解析接触记录的序列化检查", "geometry_hash": (b"g" * 32).hex(),
    }
    allowed = replace(original, allowed_contacts=(contact,))
    assert allowed.collision_policy_hash != original.collision_policy_hash
    for field in ("reason", "evidence"):
        changed = {**contact, field: contact[field] + "；变更"}
        assert replace(original, allowed_contacts=(changed,)).collision_policy_hash != allowed.collision_policy_hash
    with pytest.raises(ValueError, match="geometry_hash"):
        replace(allowed, geometry_hash=b"x" * 32)
    with pytest.raises(ValueError, match="duplicate"):
        replace(original, allowed_contacts=(contact, contact))
    with pytest.raises(ValueError, match="unknown"):
        replace(original, allowed_contacts=({**contact, "links": ["Link7", "Link8"]},))
    with pytest.raises(ValidationError):
        replace(original, allowed_contacts=({key: value for key, value in contact.items() if key != "evidence"},))
    contact["links"][0] = "modified-by-caller"
    assert allowed.allowed_contacts[0]["links"][0] == "base_link"


@pytest.mark.parametrize("field", list(parameter_payload()["parameters"]["lidar_range_error_mm"]))
def test_incomplete_parameter_record_is_rejected_on_load(field: str, tmp_path: Path) -> None:
    document = _artifact().to_document()
    del document["parameters"]["lidar_range_error_mm"][field]
    path = tmp_path / "incomplete.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValidationError):
        _load(path)


@pytest.mark.parametrize("section,field", [
    ("source", "description"), ("source", "data_hashes"),
    ("validation", "data_hashes"), ("validation", "method"),
    ("validation", "result"), ("validation", "evidence"),
])
def test_incomplete_provenance_is_rejected(section: str, field: str) -> None:
    payload = parameter_payload()
    del payload["parameters"]["lidar_range_error_mm"][section][field]
    with pytest.raises(ValidationError):
        _artifact(payload)


@pytest.mark.parametrize("name,value", [
    ("lidar_range_error_mm", -1), ("lidar_range_error_mm", True),
    ("lidar_range_error_mm", float("nan")), ("lidar_range_error_mm", float("inf")),
    ("lidar_range_error_mm", "1"), ("query_batch_size", 0),
    ("query_batch_size", 1.5), ("query_batch_size", 2.0),
    ("query_batch_size", 2**31), ("point_timestamp_error_ns", 2**63),
    ("self_collision_cbf_rate_s_inv", 0), ("dcol_primal_residual_limit", 0),
    ("bisection_max_depth", -1), ("control_deadline_ns", 10_000_001),
    ("query_deadline_ns", 10_000_000), ("support_merge_radius_limit_mm", 101),
    ("support_radius_limit_mm", 0.1),
])
def test_invalid_numeric_fields_are_rejected(name: str, value: object) -> None:
    payload = parameter_payload()
    payload["parameters"][name]["value"] = value
    with pytest.raises((ValueError, ValidationError)):
        _artifact(payload)


def test_unknown_missing_units_scope_and_validation_are_rejected() -> None:
    payload = parameter_payload()
    changed = deepcopy(payload)
    changed["parameters"]["lidar_range_error_mm"]["unit"] = "m"
    with pytest.raises(ValidationError):
        _artifact(changed)
    changed = deepcopy(payload)
    changed["parameters"]["extra"] = deepcopy(changed["parameters"]["lidar_range_error_mm"])
    with pytest.raises(ValidationError):
        _artifact(changed)
    changed = deepcopy(payload)
    del changed["parameters"]["voxel_size_mm"]
    with pytest.raises(ValidationError):
        _artifact(changed)
    for value in ("failed", "", None):
        changed = deepcopy(payload)
        changed["parameters"]["lidar_range_error_mm"]["validation"]["result"] = value
        with pytest.raises(ValidationError):
            _artifact(changed)
    for field in ("measurement_method", "calculation_method", "confidence_requirement"):
        changed = deepcopy(payload)
        changed["parameters"]["lidar_range_error_mm"][field] = "  "
        with pytest.raises(ValidationError):
            _artifact(changed)
    with pytest.raises(ValueError, match="scope"):
        CollisionParameterArtifact.create(payload, device="production-device", scenario=SCENARIO)
    with pytest.raises(ValueError, match="scope"):
        CollisionParameterArtifact.create(payload, device=DEVICE, scenario="production-scene")


def test_independent_data_and_unique_pair_requirements() -> None:
    payload = parameter_payload()
    record = payload["parameters"]["lidar_range_error_mm"]
    record["validation"]["data_hashes"] = record["source"]["data_hashes"]
    with pytest.raises(ValueError, match="independent"):
        _artifact(payload)
    payload = parameter_payload()
    payload["input_data_hashes"] = payload["input_data_hashes"][:1]
    with pytest.raises(ValueError, match="input_data_hashes"):
        _artifact(payload)
    payload = parameter_payload()
    payload["self_clearance_mm"].append(deepcopy(payload["self_clearance_mm"][0]))
    payload["self_clearance_mm"][-1]["links"].reverse()
    with pytest.raises(ValueError, match="duplicate"):
        _artifact(payload)


def test_version_hash_tampering_and_duplicate_json_fields_fail_loading(tmp_path: Path) -> None:
    path = tmp_path / "parameters.json"
    original = _artifact().to_document()
    for field, value in (("schema_version", 2), ("schema_version", True), ("schema_version", 1.0), ("parameter_hash", "0" * 64)):
        document = {**original, field: value}
        path.write_text(json.dumps(document), encoding="utf-8")
        with pytest.raises((ValueError, ValidationError)):
            _load(path)
    changed = deepcopy(original)
    changed["parameters"]["lidar_range_error_mm"]["value"] += 1
    path.write_text(json.dumps(changed), encoding="utf-8")
    with pytest.raises(ValueError, match="parameter_hash"):
        _load(path)
    path.write_text('{"schema_version":1,"schema_version":1}', encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate JSON field"):
        _load(path)
    del original["parameter_hash"]
    path.write_text(json.dumps(original), encoding="utf-8")
    with pytest.raises(ValidationError):
        _load(path)


def test_artifact_policy_and_runtime_configuration_cannot_be_overridden() -> None:
    payload = parameter_payload()
    artifact = _artifact(payload)
    config = CollisionSafetyConfig.from_policy(_policy(artifact))
    module = CollisionSafety(config)
    payload["parameters"]["environment_clearance_mm"]["value"] = 0
    artifact.to_document()["parameters"]["environment_clearance_mm"]["value"] = 0
    assert artifact.parameters["environment_clearance_mm"]["value"] == 30.0
    with pytest.raises(TypeError):
        artifact.parameters["environment_clearance_mm"]["value"] = 0
    with pytest.raises(TypeError):
        artifact.parameters["lidar_range_error_mm"]["source"]["data_hashes"][0] = "0" * 64
    with pytest.raises(FrozenInstanceError):
        artifact.parameter_hash = b"x" * 32
    with pytest.raises(FrozenInstanceError):
        config.policy.collision_policy_hash = b"x" * 32
    with pytest.raises(FrozenInstanceError):
        config.query_deadline_ns = 1
    with pytest.raises(AttributeError):
        module.config = _config()
    for field in ("max_support_points", "query_deadline_ns", "max_primitive_rows"):
        with pytest.raises(ValueError, match="override"):
            replace(config, **{field: getattr(config, field) + 1})
    with pytest.raises(ValueError, match="override"):
        replace(config, collision_policy_hash=b"x" * 32)
    with pytest.raises(TypeError, match="policy"):
        replace(config, policy=None)


@pytest.mark.parametrize("change", ["method", "geometry", "kernel", "allowed_contacts"])
def test_all_operations_reject_prepared_scene_from_previous_policy(change: str) -> None:
    old_config = _config()
    old_module = CollisionSafety(old_config)
    old_prepared = old_module.prepare_scene(_scene(), _identities(old_config))
    assert int(old_prepared.status) == CollisionStatus.OK
    policy = old_config.policy
    if change == "method":
        payload = parameter_payload()
        payload["generator"]["method"] += "；更新"
        policy = _policy(_artifact(payload))
    elif change == "geometry":
        policy = replace(policy, geometry_hash=b"x" * 32)
    elif change == "kernel":
        policy = replace(policy, kernel_version=b"x" * 32)
    else:
        policy = replace(policy, allowed_contacts=({
            "links": ["base_link", "Link1"], "reason": "解析 identity 输入",
            "evidence": "解析接触记录的序列化检查", "geometry_hash": (b"g" * 32).hex(),
        },))
    module = CollisionSafety(CollisionSafetyConfig.from_policy(policy))
    prepared = module.prepare_scene(_scene(), _identities(old_config))
    assert int(prepared.status) == CollisionStatus.INVALID_SCENE
    query = module.query(_query_batch(), old_prepared, QueryMode.OSCBF_BARRIER)
    certificate = module.certify(_segment_batch(), old_prepared)
    expected = CollisionStatus.DEADLINE_MISSED if bool(query.header.deadline_missed) else CollisionStatus.INVALID_SCENE
    assert int(query.header.status) == expected
    assert int(certificate.header.status) == CollisionStatus.INVALID_SCENE
    assert not np.any(query.valid_mask)
    assert not np.any(certificate.certified_mask)
    current_identities = _identities(module.config)._replace(
        geometry_hash=jnp.asarray(np.frombuffer(policy.geometry_hash, dtype=np.uint8)),
        kernel_version=jnp.asarray(np.frombuffer(policy.kernel_version, dtype=np.uint8)),
    )
    current = module.prepare_scene(_scene(config=module.config, identities=current_identities), current_identities)
    assert int(current.status) == CollisionStatus.OK


def test_scene_cannot_override_artifact_clearance_or_radius_limits() -> None:
    config = _config()
    module = CollisionSafety(config)
    identities = _identities(config)
    for field, values in (
        ("required_clearance_mm", jnp.zeros(3, dtype=jnp.float64)),
        ("support_radii_mm", jnp.full(3, 101.0, dtype=jnp.float64)),
    ):
        scene = _scene()._replace(**{field: values})
        assert int(module.prepare_scene(scene, identities).status) == CollisionStatus.INVALID_SCENE
        prepared = module.prepare_scene(_scene(), identities)._replace(**{field: values})
        assert int(module.certify(_segment_batch(), prepared).header.status) == CollisionStatus.INVALID_SCENE
