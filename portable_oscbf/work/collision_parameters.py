from __future__ import annotations

from collections.abc import Mapping
from dataclasses import InitVar, dataclass, field
import hashlib
import json
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any

from jsonschema import Draft202012Validator


_SCHEMA_PATH = Path(__file__).resolve().parents[1] / "config" / "collision_parameter_schema.json"


def _json_bytes(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def _freeze(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _schema() -> dict[str, Any]:
    schema = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return schema


def _digest(value: object) -> bytes:
    return hashlib.sha256(_json_bytes(value)).digest()


def _identity(value: bytes, name: str) -> None:
    if type(value) is not bytes or len(value) != 32:
        raise ValueError(f"{name} must contain 32 bytes")


@dataclass(frozen=True, slots=True)
class CollisionParameterArtifact:
    document: InitVar[dict[str, Any]]
    device: str
    scenario: str
    parameter_hash: bytes = field(init=False)
    parameters: Mapping[str, Any] = field(init=False, repr=False)
    self_clearance_mm: tuple[Mapping[str, Any], ...] = field(init=False, repr=False)
    _canonical_document: bytes = field(init=False, repr=False)

    def __post_init__(self, document: dict[str, Any]) -> None:
        # JSON 往返隔离调用方持有的容器，并立即拒绝所有非有限数值。
        canonical = _json_bytes(document)
        data = json.loads(canonical)
        schema = _schema()
        Draft202012Validator(schema).validate(data)
        if type(data["schema_version"]) is not int:
            raise ValueError("schema_version must be an integer")
        for name, scope in (("device", self.device), ("scenario", self.scenario)):
            if not isinstance(scope, str) or not scope.strip():
                raise ValueError(f"{name} must be nonempty")

        records = list(data["parameters"].items())
        pairs: set[tuple[str, str]] = set()
        for entry in data["self_clearance_mm"]:
            pair = tuple(sorted(entry["links"]))
            if pair in pairs:
                raise ValueError(f"duplicate self_clearance_mm pair: {pair}")
            pairs.add(pair)
            records.append((f"self_clearance_mm[{pair}]", entry["clearance"]))

        input_hashes = set(data["input_data_hashes"])
        development_hashes: set[str] = set()
        validation_hashes: set[str] = set()
        for name, record in records:
            if self.device not in record["devices"] or self.scenario not in record["scenarios"]:
                raise ValueError(f"{name}: device or scenario outside applicable scope")
            source = set(record["source"]["data_hashes"])
            independent = set(record["validation"]["data_hashes"])
            development_hashes.update(source)
            validation_hashes.update(independent)
        if development_hashes & validation_hashes:
            raise ValueError("validation data must be independent of all development data")
        if input_hashes != development_hashes | validation_hashes:
            raise ValueError("input_data_hashes must equal all source and validation data hashes")

        parameters = data["parameters"]
        for name, definition in schema["properties"]["parameters"]["properties"].items():
            integer = definition["$ref"] in ("#/$defs/count", "#/$defs/ns", "#/$defs/deadline")
            if (integer or name == "bisection_max_depth") and type(parameters[name]["value"]) is not int:
                raise ValueError(f"{name}.value must be an integer")
        def value(name: str) -> int | float:
            return parameters[name]["value"]

        if value("query_deadline_ns") + value("qp_deadline_ns") > value("control_deadline_ns"):
            raise ValueError("query and QP deadlines exceed control_deadline_ns")
        if value("control_deadline_ns") > 10_000_000:
            raise ValueError("control_deadline_ns exceeds the 100 Hz period")
        if value("support_merge_radius_limit_mm") > value("support_radius_limit_mm"):
            raise ValueError("support_merge_radius_limit_mm exceeds support_radius_limit_mm")
        if value("voxel_size_mm") * math.sqrt(3) / 2 > value("support_radius_limit_mm"):
            raise ValueError("support_radius_limit_mm cannot cover the voxel half diagonal")

        payload = {key: item for key, item in data.items() if key != "parameter_hash"}
        digest = _digest(payload)
        if data["parameter_hash"] != digest.hex():
            raise ValueError("parameter_hash does not match artifact content")
        object.__setattr__(self, "parameter_hash", digest)
        object.__setattr__(self, "parameters", _freeze(parameters))
        object.__setattr__(self, "self_clearance_mm", _freeze(data["self_clearance_mm"]))
        object.__setattr__(self, "_canonical_document", canonical)

    @classmethod
    def create(
        cls, payload: dict[str, Any], *, device: str, scenario: str
    ) -> CollisionParameterArtifact:
        if "parameter_hash" in payload:
            raise ValueError("create expects an unsigned payload without parameter_hash")
        return cls({**payload, "parameter_hash": _digest(payload).hex()}, device, scenario)

    @classmethod
    def load(cls, path: str | Path, *, device: str, scenario: str) -> CollisionParameterArtifact:
        data = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
        return cls(data, device, scenario)

    def to_document(self) -> dict[str, Any]:
        return json.loads(self._canonical_document)


@dataclass(frozen=True, slots=True)
class CollisionPolicy:
    artifact: CollisionParameterArtifact
    geometry_hash: bytes
    kernel_version: bytes
    allowed_contacts: tuple[Mapping[str, Any], ...] = ()
    collision_policy_hash: bytes = field(init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.artifact, CollisionParameterArtifact):
            raise TypeError("artifact must be CollisionParameterArtifact")
        _identity(self.geometry_hash, "geometry_hash")
        _identity(self.kernel_version, "kernel_version")
        # 接触记录沿用 geometry artifact 的字段；先复制再冻结。
        contacts = [dict(item) for item in self.allowed_contacts]
        contacts = json.loads(_json_bytes(contacts))
        schema = _schema()
        Draft202012Validator({
            "$ref": "#/$defs/allowed_contacts", "$defs": schema["$defs"]
        }).validate(contacts)
        clearance_pairs = {tuple(sorted(item["links"])) for item in self.artifact.self_clearance_mm}
        seen: set[tuple[str, str]] = set()
        for contact in contacts:
            if contact["geometry_hash"] != self.geometry_hash.hex():
                raise ValueError("allowed_contacts geometry_hash differs from policy geometry")
            pair = tuple(sorted(contact["links"]))
            if pair in seen or pair not in clearance_pairs:
                raise ValueError(f"allowed_contacts contains a duplicate or unknown link pair: {pair}")
            seen.add(pair)
        digest = _digest({
            "schema_version": 1,
            "parameter_hash": self.artifact.parameter_hash.hex(),
            "geometry_hash": self.geometry_hash.hex(),
            "kernel_version": self.kernel_version.hex(),
            "allowed_contacts": contacts,
        })
        object.__setattr__(self, "allowed_contacts", _freeze(contacts))
        object.__setattr__(self, "collision_policy_hash", digest)
