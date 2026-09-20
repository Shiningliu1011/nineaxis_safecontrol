#!/usr/bin/env python3
"""Build a reviewable OFF-02 point assessment and region certificate.

The input is JSON with ``region`` fields matching ``JointRegionSpec`` plus an
explicit ``scope_statement`` and ``unknown_items``.  The output is
deterministic: it has no wall-clock timestamp and binds every listed source
file by SHA-256.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
PORTABLE_ROOT = REPO_ROOT / "portable_oscbf"
for entry in (PORTABLE_ROOT, PORTABLE_ROOT / "work"):
    value = str(entry)
    if value not in sys.path:
        sys.path.insert(0, value)

from work.obb_geometry_admission import (  # noqa: E402
    CertificateStatus,
    JointRegionSpec,
    PointCollisionStatus,
    assess_point_collision,
    build_region_certificate,
)


MODEL_IDENTITY_FILES = (
    "portable_oscbf/work/obb_geometry_admission.py",
    "portable_oscbf/work/obb_collision_model.py",
    "portable_oscbf/work/kinematics_data.py",
    "portable_oscbf/work/nineaxis_kinematics.py",
    "portable_oscbf/work/robot_geometry.py",
    "portable_oscbf/scripts/certify_obb_region.py",
    "models/ninezzhou/meshes/base_link.STL",
    "models/ninezzhou/meshes/Link1.STL",
    "models/ninezzhou/meshes/Link2.STL",
    "models/ninezzhou/meshes/Link3.STL",
    "models/ninezzhou/meshes/Link4.STL",
    "models/ninezzhou/meshes/Link5.STL",
    "models/ninezzhou/meshes/Link6.STL",
    "models/ninezzhou/meshes/Link7.STL",
    "models/ninezzhou/meshes/Link8.STL",
    "models/ninezzhou/meshes/Link9.STL",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_hashes(extra_files: list[str]) -> dict[str, str]:
    root = REPO_ROOT.resolve()
    result = {}
    for relative in (*MODEL_IDENTITY_FILES, *extra_files):
        path = (REPO_ROOT / relative).resolve()
        if root not in path.parents:
            raise ValueError(f"identity file leaves repository: {relative}")
        if not path.is_file():
            raise FileNotFoundError(f"identity file is missing: {relative}")
        result[relative] = _sha256(path)
    return dict(sorted(result.items()))


def _json_bytes(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
        default=_json_default,
    ).encode("utf-8")


def _document_sha256(document: dict) -> str:
    return hashlib.sha256(_json_bytes(document)).hexdigest()


def _region(data: dict) -> JointRegionSpec:
    vector_fields = (
        "center_q",
        "task_half_width_q",
        "state_error_half_width_q",
        "stop_half_width_q",
    )
    values = dict(data)
    for field in vector_fields:
        values[field] = tuple(float(item) for item in values[field])
    return JointRegionSpec(**values)


def build_evidence(
    document: dict,
    *,
    input_sha256: str | None = None,
) -> dict:
    scope = str(document.get("scope_statement", "")).strip()
    unknown_items = document.get("unknown_items")
    if not scope:
        raise ValueError("scope_statement must be explicit")
    if not isinstance(unknown_items, list) or not unknown_items:
        raise ValueError("unknown_items must be a non-empty list")
    if any(not str(item).strip() for item in unknown_items):
        raise ValueError("unknown_items cannot contain blank entries")

    region = _region(document["region"])
    query_id = str(document.get("query_id", "off02-region-center")).strip()
    boundary = str(document.get("boundary", "kernel_candidate")).strip()
    point = assess_point_collision(
        region.center_q,
        query_id=query_id,
        boundary=boundary,
        task_id=region.task_id,
        attachment_id=region.attachment_id,
    )
    certificate = build_region_certificate(region)
    support_assessments = []
    for item in document.get("support_points", []):
        point_id = str(item.get("id", "")).strip()
        q = tuple(float(value) for value in item.get("q", []))
        if not point_id or len(q) != 9:
            raise ValueError("each support point needs a non-blank id and 9-value q")
        support_assessments.append(assess_point_collision(
            q,
            query_id=f"{query_id}:{point_id}",
            boundary=boundary,
            task_id=region.task_id,
            attachment_id=region.attachment_id,
        ))

    if (
        point.status == PointCollisionStatus.SEPARATED.value
        and all(
            item.status == PointCollisionStatus.SEPARATED.value
            for item in support_assessments
        )
        and certificate.status == CertificateStatus.CERTIFIED.value
    ):
        admission_status = "separated_and_certified"
    elif (
        point.status == PointCollisionStatus.OVERLAP.value
        or any(
            item.status == PointCollisionStatus.OVERLAP.value
            for item in support_assessments
        )
        or certificate.status == CertificateStatus.NOT_CERTIFIED.value
    ):
        admission_status = "rejected"
    else:
        admission_status = "indeterminate"

    reason_codes = tuple(dict.fromkeys(
        [
            reason
            for item in support_assessments
            for reason in item.reason_codes
        ]
        + list(point.reason_codes)
        + list(certificate.reason_codes)
    ))
    extra_files = [str(item) for item in document.get("identity_files", [])]
    if input_sha256 is None:
        input_sha256 = _document_sha256(document)
    return {
        "schema_version": 1,
        "input_sha256": input_sha256,
        "model_id": certificate.model_id,
        "pair_policy_id": certificate.pair_policy_id,
        "certificate_id": certificate.certificate_id,
        "scope_statement": scope,
        "admission_status": admission_status,
        "reason_codes": list(reason_codes),
        "unknown_items": [str(item) for item in unknown_items],
        "candidate_context": document.get("candidate_context", {}),
        "source_sha256": _source_hashes(extra_files),
        "point_assessment": point.to_dict(),
        "support_point_assessments": [
            item.to_dict() for item in support_assessments
        ],
        "region_certificate": certificate.to_dict(),
    }


def _json_default(value: object) -> object:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with args.input.open(encoding="utf-8") as stream:
        document = json.load(stream)
    evidence = build_evidence(
        document,
        input_sha256=_sha256(args.input),
    )
    encoded = json.dumps(
        evidence,
        ensure_ascii=False,
        sort_keys=True,
        indent=2,
        allow_nan=False,
        default=_json_default,
    ) + "\n"
    if args.output is None:
        sys.stdout.write(encoded)
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
        print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
