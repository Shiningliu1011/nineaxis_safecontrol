"""Independent OBB overlap facts and conservative joint-box certificates.

The differentiable DCOL distance remains the CBF kernel.  This module is an
independent admission companion: SAT exposes overlap explicitly for every
non-adjacent link pair, while a relative-motion bound certifies omitted pairs
over a bounded joint region.  It performs no ROS I/O and owns no latch state.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import hashlib
import itertools
import json
import math
from typing import Sequence

import numpy as np

from work.kinematics_data import JOINT_CHAIN
from work.nineaxis_kinematics import NineaxisKinematics
from work.obb_collision_model import (
    OBB_COLLISION_PAIRS,
    OBB_HALF_EXTENTS_M,
    OBB_LINK_NAMES,
    OBB_LOCAL_CENTERS_M,
    OBB_LOCAL_ROTATIONS,
)


SCHEMA_VERSION = 1
NUM_JOINTS = 9
JOINT_ORDER = tuple(f"J{index}" for index in range(1, NUM_JOINTS + 1))
POINT_BOUNDARIES = (
    "kernel_candidate",
    "filtered_command",
    "simulated_state",
    "real_feedback",
)

GEOMETRY_OVERLAP = "GEOMETRY_OVERLAP"
GEOMETRY_INPUT_INVALID = "GEOMETRY_INPUT_INVALID"
GEOMETRY_NUMERICAL_INDETERMINATE = "GEOMETRY_NUMERICAL_INDETERMINATE"
PAIR_POLICY_INCOMPLETE = "PAIR_POLICY_INCOMPLETE"
DOMAIN_OUTSIDE_CERTIFICATE = "DOMAIN_OUTSIDE_CERTIFICATE"
CERTIFICATE_MISSING = "CERTIFICATE_MISSING"
CERTIFICATE_IDENTITY_MISMATCH = "CERTIFICATE_IDENTITY_MISMATCH"
REQUIRED_ENVELOPE_INCOMPLETE = "REQUIRED_ENVELOPE_INCOMPLETE"

ALL_NONADJACENT_PAIRS = tuple(
    (i, j) for i, j in itertools.combinations(range(len(OBB_LINK_NAMES)), 2)
    if j - i >= 2
)
ONLINE_CONSTRAINT_PAIRS = tuple(
    (int(i), int(j)) for i, j in np.asarray(OBB_COLLISION_PAIRS)
)
OMITTED_NONADJACENT_PAIRS = tuple(
    pair for pair in ALL_NONADJACENT_PAIRS
    if pair not in frozenset(ONLINE_CONSTRAINT_PAIRS)
)
_ROBOT = NineaxisKinematics()


class PointCollisionStatus(str, Enum):
    SEPARATED = "separated"
    OVERLAP = "overlap"
    INDETERMINATE = "indeterminate"


class CertificateStatus(str, Enum):
    CERTIFIED = "certified"
    NOT_CERTIFIED = "not_certified"
    INDETERMINATE = "indeterminate"


class DomainCoverageStatus(str, Enum):
    COVERED = "covered"
    OUTSIDE = "outside"
    INDETERMINATE = "indeterminate"


def _canonical_id(prefix: str, payload: object) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
        default=_json_default,
    ).encode("utf-8")
    return f"{prefix}:v{SCHEMA_VERSION}:" + hashlib.sha256(encoded).hexdigest()


def _json_default(value: object) -> object:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


def _joint_chain_payload() -> list[list[object]]:
    rows: list[list[object]] = []
    for parent, child, kind, x, y, z, roll, pitch, yaw, axis in JOINT_CHAIN:
        rows.append([
            parent, child, kind, x, y, z, roll, pitch, yaw, list(axis),
        ])
    return rows


def geometry_model_id() -> str:
    """Content identity for the kinematic chain and generated OBB model."""

    return _canonical_id("obb-model", {
        "links": list(OBB_LINK_NAMES),
        "centers_m": np.asarray(OBB_LOCAL_CENTERS_M).tolist(),
        "half_extents_m": np.asarray(OBB_HALF_EXTENTS_M).tolist(),
        "rotations": np.asarray(OBB_LOCAL_ROTATIONS).tolist(),
        "joint_chain": _joint_chain_payload(),
        "joint_position_limits": {
            "q_min": _ROBOT.joint_limits.q_min.tolist(),
            "q_max": _ROBOT.joint_limits.q_max.tolist(),
        },
    })


def _pair_policy_id(pairs: Sequence[tuple[int, int]], name: str) -> str:
    return _canonical_id("obb-pairs", {
        "name": name,
        "links": list(OBB_LINK_NAMES),
        "pairs": [list(pair) for pair in pairs],
        "adjacent_policy": "exclude_direct_chain_neighbours",
    })


ALL_NONADJACENT_POLICY_ID = _pair_policy_id(
    ALL_NONADJACENT_PAIRS, "all_nonadjacent",
)
OMITTED_PAIR_POLICY_ID = _pair_policy_id(
    OMITTED_NONADJACENT_PAIRS, "omitted_from_online_constraint_table",
)
GEOMETRY_MODEL_ID = geometry_model_id()


@dataclass(frozen=True)
class PairPointAssessment:
    pair: tuple[str, str]
    status: str
    max_axis_gap_m: float | None


@dataclass(frozen=True)
class PointCollisionAssessment:
    query_id: str
    boundary: str
    q: tuple[float, ...] | None
    joint_order: tuple[str, ...]
    status: str
    model_id: str
    pair_policy_id: str
    checked_pair_count: int
    task_id: str
    attachment_id: str
    reason_codes: tuple[str, ...]
    pairs: tuple[PairPointAssessment, ...]

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class JointRegionSpec:
    """Joint box split into task, state-error, and stop-envelope widths.

    J1 values are metres; J2--J9 values are radians.  Every width is required
    explicitly so an unavailable physical bound cannot silently become zero.
    """

    center_q: tuple[float, ...]
    task_half_width_q: tuple[float, ...]
    state_error_half_width_q: tuple[float, ...]
    stop_half_width_q: tuple[float, ...]
    geometry_error_m: float
    required_clearance_m: float
    task_id: str
    attachment_id: str
    task_region_source: str
    state_error_source: str
    stop_envelope_source: str
    geometry_error_source: str

    def __post_init__(self) -> None:
        arrays = (
            self.center_q, self.task_half_width_q,
            self.state_error_half_width_q, self.stop_half_width_q,
        )
        if any(len(values) != NUM_JOINTS for values in arrays):
            raise ValueError("joint region fields must contain exactly 9 values")
        if not all(math.isfinite(float(value)) for values in arrays for value in values):
            raise ValueError("joint region values must be finite")
        if any(float(value) < 0.0 for values in arrays[1:] for value in values):
            raise ValueError("joint region half widths must be non-negative")
        if not math.isfinite(self.geometry_error_m) or self.geometry_error_m < 0.0:
            raise ValueError("geometry_error_m must be finite and non-negative")
        if not math.isfinite(self.required_clearance_m) or self.required_clearance_m < 0.0:
            raise ValueError("required_clearance_m must be finite and non-negative")
        identities = (
            self.task_id,
            self.attachment_id,
            self.task_region_source,
            self.state_error_source,
            self.stop_envelope_source,
            self.geometry_error_source,
        )
        if any(
            not isinstance(value, str) or not value.strip()
            for value in identities
        ):
            raise ValueError("region identities and evidence sources must be explicit")

    @property
    def total_half_width_q(self) -> tuple[float, ...]:
        values = (
            np.asarray(self.task_half_width_q)
            + np.asarray(self.state_error_half_width_q)
            + np.asarray(self.stop_half_width_q)
        )
        return tuple(float(value) for value in values)

    def to_dict(self) -> dict:
        return asdict(self) | {"total_half_width_q": list(self.total_half_width_q)}


@dataclass(frozen=True)
class PairRegionEvidence:
    pair: tuple[str, str]
    center_status: str
    center_separation_lower_bound_m: float | None
    relative_motion_bound_m: float | None
    geometry_error_m: float
    numerical_tolerance_m: float
    certified_clearance_lower_bound_m: float | None
    required_clearance_m: float
    certified: bool
    active_joint_indices: tuple[int, ...]
    reason_codes: tuple[str, ...]


@dataclass(frozen=True)
class RegionCertificate:
    certificate_id: str
    status: str
    model_id: str
    pair_policy_id: str
    pair_count: int
    region: JointRegionSpec
    numerical_tolerance_m: float
    reason_codes: tuple[str, ...]
    pair_evidence: tuple[PairRegionEvidence, ...]

    def to_dict(self) -> dict:
        result = asdict(self)
        result["region"]["total_half_width_q"] = list(
            self.region.total_half_width_q
        )
        return result


@dataclass(frozen=True)
class DomainCoverageAssessment:
    query_id: str
    boundary: str
    status: str
    model_id: str
    certificate_id: str
    task_id: str
    attachment_id: str
    reason_codes: tuple[str, ...]

    def to_dict(self) -> dict:
        return asdict(self)


_ACTIVE_JOINTS = tuple(
    row for row in JOINT_CHAIN if row[2] in ("revolute", "prismatic")
)


def _world_obbs(q: np.ndarray) -> tuple[tuple[np.ndarray, np.ndarray, np.ndarray], ...]:
    transforms = _ROBOT.forward_kinematics(q)
    result = []
    for index, name in enumerate(OBB_LINK_NAMES):
        transform = transforms[name]
        rotation = transform[:3, :3] @ OBB_LOCAL_ROTATIONS[index]
        center = (
            transform[:3, :3] @ OBB_LOCAL_CENTERS_M[index]
            + transform[:3, 3]
        )
        result.append((rotation, center, OBB_HALF_EXTENTS_M[index]))
    return tuple(result)


def _sat_assessment(
    obb_a: tuple[np.ndarray, np.ndarray, np.ndarray],
    obb_b: tuple[np.ndarray, np.ndarray, np.ndarray],
    *,
    numerical_tolerance_m: float,
) -> tuple[PointCollisionStatus, float]:
    rotation_a, center_a, half_a = obb_a
    rotation_b, center_b, half_b = obb_b
    axes = np.vstack((
        rotation_a.T,
        rotation_b.T,
        np.asarray([
            np.cross(rotation_a[:, i], rotation_b[:, j])
            for i in range(3)
            for j in range(3)
        ]),
    ))
    norms = np.linalg.norm(axes, axis=1)
    axes = axes[norms > 1e-12] / norms[norms > 1e-12, None]
    delta = center_b - center_a
    gaps = (
        np.abs(axes @ delta)
        - np.abs(axes @ rotation_a) @ half_a
        - np.abs(axes @ rotation_b) @ half_b
    )
    max_gap = float(np.max(gaps))
    if max_gap > numerical_tolerance_m:
        return PointCollisionStatus.SEPARATED, max_gap
    if max_gap < -numerical_tolerance_m:
        return PointCollisionStatus.OVERLAP, max_gap
    return PointCollisionStatus.INDETERMINATE, max_gap


def _invalid_point_assessment(
    query_id: str,
    boundary: str,
    task_id: str,
    attachment_id: str,
    reason: str | Sequence[str],
    q: tuple[float, ...] | None = None,
) -> PointCollisionAssessment:
    reason_codes = (reason,) if isinstance(reason, str) else tuple(reason)
    return PointCollisionAssessment(
        query_id=query_id,
        boundary=boundary,
        q=q,
        joint_order=JOINT_ORDER,
        status=PointCollisionStatus.INDETERMINATE.value,
        model_id=GEOMETRY_MODEL_ID,
        pair_policy_id=ALL_NONADJACENT_POLICY_ID,
        checked_pair_count=0,
        task_id=task_id,
        attachment_id=attachment_id,
        reason_codes=reason_codes,
        pairs=(),
    )


def _is_nonblank_text(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_finite_positive(value: object) -> bool:
    try:
        return math.isfinite(float(value)) and float(value) > 0.0
    except (TypeError, ValueError, OverflowError):
        return False


def assess_point_collision(
    q: Sequence[float],
    *,
    query_id: str,
    boundary: str,
    task_id: str,
    attachment_id: str,
    expected_model_id: str | None = None,
    numerical_tolerance_m: float = 1e-9,
) -> PointCollisionAssessment:
    """Assess all 36 non-adjacent OBB pairs at one bound state."""

    safe_query_id = query_id if isinstance(query_id, str) else ""
    safe_boundary = boundary if isinstance(boundary, str) else ""
    safe_task_id = task_id if isinstance(task_id, str) else ""
    safe_attachment_id = attachment_id if isinstance(attachment_id, str) else ""
    reasons = []
    if (
        not _is_nonblank_text(query_id)
        or not isinstance(boundary, str)
        or boundary not in POINT_BOUNDARIES
        or not _is_nonblank_text(task_id)
        or not _is_nonblank_text(attachment_id)
        or not _is_finite_positive(numerical_tolerance_m)
    ):
        reasons.append(GEOMETRY_INPUT_INVALID)
    if expected_model_id is not None and expected_model_id != GEOMETRY_MODEL_ID:
        reasons.append(CERTIFICATE_IDENTITY_MISMATCH)
    try:
        state = np.asarray(q, dtype=float)
    except (TypeError, ValueError, OverflowError):
        state = None
        reasons.append(GEOMETRY_INPUT_INVALID)
    lower = _ROBOT.joint_limits.q_min
    upper = _ROBOT.joint_limits.q_max
    reported_q = None
    if state is not None and state.shape == (NUM_JOINTS,) \
            and np.all(np.isfinite(state)):
        reported_q = tuple(float(value) for value in state)
    if state is None or (
        state.shape != (NUM_JOINTS,)
        or not np.all(np.isfinite(state))
        or np.any(state < lower)
        or np.any(state > upper)
    ):
        if GEOMETRY_INPUT_INVALID not in reasons:
            reasons.append(GEOMETRY_INPUT_INVALID)
    if reasons:
        return _invalid_point_assessment(
            safe_query_id,
            safe_boundary,
            safe_task_id,
            safe_attachment_id,
            tuple(dict.fromkeys(reasons)),
            q=reported_q,
        )

    numerical_tolerance_m = float(numerical_tolerance_m)
    obbs = _world_obbs(state)
    pair_rows = []
    for i, j in ALL_NONADJACENT_PAIRS:
        status, gap = _sat_assessment(
            obbs[i], obbs[j], numerical_tolerance_m=numerical_tolerance_m,
        )
        pair_rows.append(PairPointAssessment(
            pair=(OBB_LINK_NAMES[i], OBB_LINK_NAMES[j]),
            status=status.value,
            max_axis_gap_m=gap,
        ))
    if any(row.status == PointCollisionStatus.OVERLAP.value for row in pair_rows):
        status = PointCollisionStatus.OVERLAP
        reasons = [GEOMETRY_OVERLAP]
    elif any(row.status == PointCollisionStatus.INDETERMINATE.value for row in pair_rows):
        status = PointCollisionStatus.INDETERMINATE
        reasons = [GEOMETRY_NUMERICAL_INDETERMINATE]
    else:
        status = PointCollisionStatus.SEPARATED
        reasons = []
    if any(row.status == PointCollisionStatus.INDETERMINATE.value for row in pair_rows) \
            and GEOMETRY_NUMERICAL_INDETERMINATE not in reasons:
        reasons.append(GEOMETRY_NUMERICAL_INDETERMINATE)
    return PointCollisionAssessment(
        query_id=query_id,
        boundary=boundary,
        q=tuple(float(value) for value in state),
        joint_order=JOINT_ORDER,
        status=status.value,
        model_id=GEOMETRY_MODEL_ID,
        pair_policy_id=ALL_NONADJACENT_POLICY_ID,
        checked_pair_count=len(pair_rows),
        task_id=task_id,
        attachment_id=attachment_id,
        reason_codes=tuple(reasons),
        pairs=tuple(pair_rows),
    )


def _obb_farthest_radius(link_index: int) -> float:
    center = np.asarray(OBB_LOCAL_CENTERS_M[link_index])
    rotation = np.asarray(OBB_LOCAL_ROTATIONS[link_index])
    half = np.asarray(OBB_HALF_EXTENTS_M[link_index])
    radii = []
    for signs in itertools.product((-1.0, 1.0), repeat=3):
        corner = center + rotation @ (half * np.asarray(signs))
        radii.append(float(np.linalg.norm(corner)))
    return max(radii)


def relative_pair_motion_bound_m(
    pair: tuple[int, int], half_width_q: Sequence[float],
) -> tuple[float, tuple[int, ...]]:
    """Bound Hausdorff motion of the downstream OBB in the upstream frame.

    Common upstream joint motion cancels.  Revolute contributions use the
    maximum downstream chain reach and the exact chord bound; the prismatic J1
    contribution is its interval half width.
    """

    i, j = pair
    if pair not in ALL_NONADJACENT_PAIRS:
        raise ValueError("pair must be a non-adjacent ordered link pair")
    widths = np.asarray(half_width_q, dtype=float)
    if widths.shape != (NUM_JOINTS,) or not np.all(np.isfinite(widths)) \
            or np.any(widths < 0.0):
        raise ValueError("half_width_q must contain 9 finite non-negative values")
    total = 0.0
    active = []
    for zero_index in range(i, j):
        delta = float(widths[zero_index])
        if delta == 0.0:
            continue
        active.append(zero_index + 1)
        joint = _ACTIVE_JOINTS[zero_index]
        if joint[2] == "prismatic":
            total += delta
            continue
        downstream_fixed_reach = sum(
            float(np.linalg.norm(np.asarray(row[3:6], dtype=float)))
            for row in _ACTIVE_JOINTS[zero_index + 1:j]
        )
        radius = downstream_fixed_reach + _obb_farthest_radius(j)
        total += 2.0 * radius * math.sin(min(delta, math.pi) / 2.0)
    return total, tuple(active)


def _certificate_payload(
    status: str,
    region: JointRegionSpec,
    numerical_tolerance_m: float,
    reasons: tuple[str, ...],
    evidence: tuple[PairRegionEvidence, ...],
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "model_id": GEOMETRY_MODEL_ID,
        "pair_policy_id": OMITTED_PAIR_POLICY_ID,
        "region": region.to_dict(),
        "numerical_tolerance_m": numerical_tolerance_m,
        "reason_codes": list(reasons),
        "pair_evidence": [asdict(row) for row in evidence],
    }


def build_region_certificate(
    region: JointRegionSpec,
    *,
    numerical_tolerance_m: float = 1e-9,
) -> RegionCertificate:
    """Build a conservative certificate for all 22 omitted non-adjacent pairs."""

    if not math.isfinite(numerical_tolerance_m) or numerical_tolerance_m <= 0.0:
        raise ValueError("numerical_tolerance_m must be finite and positive")
    center = np.asarray(region.center_q, dtype=float)
    half = np.asarray(region.total_half_width_q, dtype=float)
    lower = _ROBOT.joint_limits.q_min
    upper = _ROBOT.joint_limits.q_max
    if np.any(center - half < lower) or np.any(center + half > upper):
        status = CertificateStatus.INDETERMINATE.value
        reasons = (GEOMETRY_INPUT_INVALID,)
        evidence: tuple[PairRegionEvidence, ...] = ()
    else:
        obbs = _world_obbs(center)
        rows = []
        for pair in OMITTED_NONADJACENT_PAIRS:
            i, j = pair
            center_status, gap = _sat_assessment(
                obbs[i], obbs[j], numerical_tolerance_m=numerical_tolerance_m,
            )
            motion_bound, active = relative_pair_motion_bound_m(pair, half)
            certified_lower = None
            row_reasons: tuple[str, ...] = ()
            certified = False
            if center_status is PointCollisionStatus.OVERLAP:
                row_reasons = (GEOMETRY_OVERLAP,)
            elif center_status is PointCollisionStatus.INDETERMINATE:
                row_reasons = (GEOMETRY_NUMERICAL_INDETERMINATE,)
            else:
                certified_lower = (
                    gap - motion_bound - region.geometry_error_m
                    - numerical_tolerance_m
                )
                certified = certified_lower > region.required_clearance_m
                if not certified:
                    row_reasons = (REQUIRED_ENVELOPE_INCOMPLETE,)
            rows.append(PairRegionEvidence(
                pair=(OBB_LINK_NAMES[i], OBB_LINK_NAMES[j]),
                center_status=center_status.value,
                center_separation_lower_bound_m=gap,
                relative_motion_bound_m=motion_bound,
                geometry_error_m=region.geometry_error_m,
                numerical_tolerance_m=numerical_tolerance_m,
                certified_clearance_lower_bound_m=certified_lower,
                required_clearance_m=region.required_clearance_m,
                certified=certified,
                active_joint_indices=active,
                reason_codes=row_reasons,
            ))
        evidence = tuple(rows)
        evidence_reasons = tuple(dict.fromkeys(
            reason
            for row in evidence
            for reason in row.reason_codes
        ))
        if GEOMETRY_OVERLAP in evidence_reasons:
            status = CertificateStatus.NOT_CERTIFIED.value
            reasons = evidence_reasons
        elif GEOMETRY_NUMERICAL_INDETERMINATE in evidence_reasons:
            status = CertificateStatus.INDETERMINATE.value
            reasons = evidence_reasons
        elif not all(row.certified for row in evidence):
            status = CertificateStatus.NOT_CERTIFIED.value
            reasons = evidence_reasons or (REQUIRED_ENVELOPE_INCOMPLETE,)
        else:
            status = CertificateStatus.CERTIFIED.value
            reasons = ()

    payload = _certificate_payload(
        status, region, numerical_tolerance_m, reasons, evidence,
    )
    certificate_id = _canonical_id("obb-region", payload)
    return RegionCertificate(
        certificate_id=certificate_id,
        status=status,
        model_id=GEOMETRY_MODEL_ID,
        pair_policy_id=OMITTED_PAIR_POLICY_ID,
        pair_count=len(OMITTED_NONADJACENT_PAIRS),
        region=region,
        numerical_tolerance_m=numerical_tolerance_m,
        reason_codes=reasons,
        pair_evidence=evidence,
    )


def assess_domain_coverage(
    required: JointRegionSpec,
    certificate: RegionCertificate | None,
    *,
    query_id: str,
    boundary: str,
    expected_model_id: str | None = None,
    expected_certificate_id: str | None = None,
    expected_pair_policy_id: str | None = None,
) -> DomainCoverageAssessment:
    """Check whether a required joint box is covered by a bound certificate."""

    model_id = GEOMETRY_MODEL_ID
    certificate_id = ""
    reasons = []
    result_query_id = query_id if isinstance(query_id, str) else ""
    result_boundary = boundary if isinstance(boundary, str) else ""
    query_is_valid = isinstance(query_id, str) and bool(query_id.strip())
    required_is_valid = isinstance(required, JointRegionSpec)
    if not query_is_valid \
            or not isinstance(boundary, str) \
            or boundary not in POINT_BOUNDARIES:
        reasons.append(GEOMETRY_INPUT_INVALID)
    if not required_is_valid:
        reasons.append(REQUIRED_ENVELOPE_INCOMPLETE)
    if certificate is None:
        reasons.append(CERTIFICATE_MISSING)
    elif not isinstance(certificate, RegionCertificate):
        reasons.extend((CERTIFICATE_IDENTITY_MISMATCH,
                        REQUIRED_ENVELOPE_INCOMPLETE))
    else:
        certificate_id = certificate.certificate_id
        try:
            certificate_payload = _certificate_payload(
                certificate.status,
                certificate.region,
                certificate.numerical_tolerance_m,
                certificate.reason_codes,
                certificate.pair_evidence,
            )
            certificate_identity_valid = (
                math.isfinite(certificate.numerical_tolerance_m)
                and certificate.numerical_tolerance_m > 0.0
                and certificate.certificate_id == _canonical_id(
                    "obb-region", certificate_payload,
                )
            )
        except (AttributeError, TypeError, ValueError):
            certificate_identity_valid = False
        if not certificate_identity_valid:
            reasons.append(CERTIFICATE_IDENTITY_MISMATCH)
        if certificate.status != CertificateStatus.CERTIFIED.value:
            reasons.append(REQUIRED_ENVELOPE_INCOMPLETE)
        reasons.extend(certificate.reason_codes)
        if certificate.model_id != model_id:
            reasons.append(CERTIFICATE_IDENTITY_MISMATCH)
        if certificate.pair_policy_id != OMITTED_PAIR_POLICY_ID \
                or certificate.pair_count != len(OMITTED_NONADJACENT_PAIRS) \
                or len(certificate.pair_evidence) \
                != len(OMITTED_NONADJACENT_PAIRS) \
                or tuple(row.pair for row in certificate.pair_evidence) \
                != tuple(
                    (OBB_LINK_NAMES[i], OBB_LINK_NAMES[j])
                    for i, j in OMITTED_NONADJACENT_PAIRS
                ):
            reasons.append(PAIR_POLICY_INCOMPLETE)
        if required_is_valid and (
            certificate.region.task_id != required.task_id
            or certificate.region.attachment_id != required.attachment_id
            or certificate.region.task_region_source
            != required.task_region_source
            or certificate.region.state_error_source
            != required.state_error_source
            or certificate.region.stop_envelope_source
            != required.stop_envelope_source
            or certificate.region.geometry_error_source
            != required.geometry_error_source
            or certificate.region.geometry_error_m
            < required.geometry_error_m
        ):
            reasons.append(CERTIFICATE_IDENTITY_MISMATCH)
        if (
            certificate.status == CertificateStatus.CERTIFIED.value
            and (
                certificate.reason_codes
                or any(
                    not row.certified
                    or row.center_status != PointCollisionStatus.SEPARATED.value
                    or row.reason_codes
                    for row in certificate.pair_evidence
                )
            )
        ):
            reasons.append(CERTIFICATE_IDENTITY_MISMATCH)
    if expected_model_id is not None and expected_model_id != model_id:
        reasons.append(CERTIFICATE_IDENTITY_MISMATCH)
    if (
        expected_certificate_id is not None
        and expected_certificate_id != certificate_id
    ):
        reasons.append(CERTIFICATE_IDENTITY_MISMATCH)
    if (
        expected_pair_policy_id is not None
        and expected_pair_policy_id != OMITTED_PAIR_POLICY_ID
    ):
        reasons.append(PAIR_POLICY_INCOMPLETE)
    if reasons:
        return DomainCoverageAssessment(
            query_id=result_query_id,
            boundary=result_boundary,
            status=DomainCoverageStatus.INDETERMINATE.value,
            model_id=model_id,
            certificate_id=certificate_id,
            task_id=(required.task_id if required_is_valid else ""),
            attachment_id=(required.attachment_id if required_is_valid else ""),
            reason_codes=tuple(dict.fromkeys(reasons)),
        )

    assert certificate is not None
    assert required_is_valid
    required_center = np.asarray(required.center_q)
    required_half = np.asarray(required.total_half_width_q)
    certified_center = np.asarray(certificate.region.center_q)
    certified_half = np.asarray(certificate.region.total_half_width_q)
    included = np.all(
        np.abs(required_center - certified_center) + required_half
        <= certified_half + certificate.numerical_tolerance_m
    )
    clearance_covered = all(
        row.certified
        and row.center_status == PointCollisionStatus.SEPARATED.value
        and not row.reason_codes
        and row.certified_clearance_lower_bound_m is not None
        and row.certified_clearance_lower_bound_m
        > required.required_clearance_m
        for row in certificate.pair_evidence
    )
    covered = bool(included and clearance_covered)
    outside_reasons = []
    if not included:
        outside_reasons.append(DOMAIN_OUTSIDE_CERTIFICATE)
    if not clearance_covered:
        outside_reasons.append(REQUIRED_ENVELOPE_INCOMPLETE)
    return DomainCoverageAssessment(
        query_id=result_query_id,
        boundary=result_boundary,
        status=(
            DomainCoverageStatus.COVERED.value
            if covered else DomainCoverageStatus.OUTSIDE.value
        ),
        model_id=model_id,
        certificate_id=certificate.certificate_id,
        task_id=required.task_id,
        attachment_id=required.attachment_id,
        reason_codes=(() if covered else tuple(outside_reasons)),
    )


def dump_certificate(certificate: RegionCertificate) -> str:
    """Deterministic JSON representation for review and later loading."""

    return json.dumps(
        certificate.to_dict(), ensure_ascii=False, sort_keys=True, indent=2,
        allow_nan=False, default=_json_default,
    ) + "\n"
