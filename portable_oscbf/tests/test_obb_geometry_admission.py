#!/usr/bin/env python3
"""OFF-02 acceptance tests for point facts and bounded-region evidence."""

from __future__ import annotations

from dataclasses import replace
import json

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.obb_geometry_admission import (
    ALL_NONADJACENT_PAIRS,
    ALL_NONADJACENT_POLICY_ID,
    CERTIFICATE_MISSING,
    CERTIFICATE_IDENTITY_MISMATCH,
    DOMAIN_OUTSIDE_CERTIFICATE,
    GEOMETRY_INPUT_INVALID,
    GEOMETRY_MODEL_ID,
    GEOMETRY_OVERLAP,
    JOINT_ORDER,
    OMITTED_NONADJACENT_PAIRS,
    OMITTED_PAIR_POLICY_ID,
    ONLINE_CONSTRAINT_PAIRS,
    REQUIRED_ENVELOPE_INCOMPLETE,
    CertificateStatus,
    DomainCoverageStatus,
    JointRegionSpec,
    PointCollisionStatus,
    assess_domain_coverage,
    assess_point_collision,
    build_region_certificate,
    dump_certificate,
    relative_pair_motion_bound_m,
)
from scripts.certify_obb_region import build_evidence


KNOWN_BLIND_OVERLAP_Q = (
    0.14329339328270277,
    -1.4618425887303508,
    -0.9147667179289727,
    -1.5535417564110803,
    -0.42121936335795374,
    -0.002084100515956866,
    -1.3914541748249132,
    -0.9170144472791932,
    -0.5862754227014818,
)


def _region(
    *,
    center_q: tuple[float, ...] = (0.2, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0),
    task_half_width_q: tuple[float, ...] = (0.001,) * 9,
) -> JointRegionSpec:
    return JointRegionSpec(
        center_q=center_q,
        task_half_width_q=task_half_width_q,
        state_error_half_width_q=(0.0,) * 9,
        stop_half_width_q=(0.0,) * 9,
        geometry_error_m=0.001,
        required_clearance_m=0.01,
        task_id="butterfly-current-v1",
        attachment_id="none",
        task_region_source="OFF-02 test fixture",
        state_error_source="model-only zero; not a hardware bound",
        stop_envelope_source="model-only zero; not a hardware bound",
        geometry_error_source="conservative test allowance",
    )


def _point(q) -> object:
    return assess_point_collision(
        q,
        query_id="test-query",
        boundary="kernel_candidate",
        task_id="butterfly-current-v1",
        attachment_id="none",
    )


def test_pair_partition_covers_only_nonadjacent_pairs():
    assert len(ALL_NONADJACENT_PAIRS) == 36
    assert len(ONLINE_CONSTRAINT_PAIRS) == 14
    assert len(OMITTED_NONADJACENT_PAIRS) == 22
    assert set(ONLINE_CONSTRAINT_PAIRS).isdisjoint(OMITTED_NONADJACENT_PAIRS)
    assert set(ONLINE_CONSTRAINT_PAIRS) | set(OMITTED_NONADJACENT_PAIRS) \
        == set(ALL_NONADJACENT_PAIRS)
    assert all(right - left >= 2 for left, right in ALL_NONADJACENT_PAIRS)
    assert (3, 5) in OMITTED_NONADJACENT_PAIRS


def test_point_assessment_checks_all_36_pairs_at_zero_configuration():
    result = _point(np.zeros(9))
    assert result.status == PointCollisionStatus.SEPARATED.value
    assert result.checked_pair_count == 36
    assert len(result.pairs) == 36
    assert result.model_id == GEOMETRY_MODEL_ID
    assert result.pair_policy_id == ALL_NONADJACENT_POLICY_ID
    assert result.reason_codes == ()
    assert result.q == (0.0,) * 9
    assert result.joint_order == JOINT_ORDER


def test_point_assessment_rejects_known_14_pair_blind_overlap():
    result = _point(KNOWN_BLIND_OVERLAP_Q)
    overlapping = {row.pair for row in result.pairs if row.status == "overlap"}
    assert result.status == PointCollisionStatus.OVERLAP.value
    assert GEOMETRY_OVERLAP in result.reason_codes
    assert ("base_link", "Link9") in overlapping
    assert ("Link6", "Link8") in overlapping


@pytest.mark.parametrize(
    "q",
    [
        (0.0,) * 8,
        (float("nan"),) + (0.0,) * 8,
        (0.0, 2.0) + (0.0,) * 7,
    ],
)
def test_invalid_point_input_is_indeterminate(q):
    result = _point(q)
    assert result.status == PointCollisionStatus.INDETERMINATE.value
    assert result.checked_pair_count == 0
    assert result.reason_codes == (GEOMETRY_INPUT_INVALID,)


def test_invalid_point_metadata_fails_closed_without_raising():
    result = assess_point_collision(
        np.zeros(9),
        query_id=None,
        boundary="kernel_candidate",
        task_id="butterfly-current-v1",
        attachment_id="none",
    )
    assert result.status == PointCollisionStatus.INDETERMINATE.value
    assert result.checked_pair_count == 0
    assert result.q == (0.0,) * 9


def test_model_identity_mismatch_is_indeterminate():
    result = assess_point_collision(
        np.zeros(9),
        query_id="test-query",
        boundary="filtered_command",
        task_id="butterfly-current-v1",
        attachment_id="none",
        expected_model_id="wrong-model",
    )
    assert result.status == PointCollisionStatus.INDETERMINATE.value
    assert result.reason_codes == (CERTIFICATE_IDENTITY_MISMATCH,)
    assert result.q == (0.0,) * 9


def test_relative_motion_bound_cancels_common_upstream_motion():
    j1_only = (0.01,) + (0.0,) * 8
    link3_link5_bound, active = relative_pair_motion_bound_m((3, 5), j1_only)
    base_link9_bound, base_active = relative_pair_motion_bound_m((0, 9), j1_only)
    assert link3_link5_bound == 0.0
    assert active == ()
    assert base_link9_bound == pytest.approx(0.01)
    assert base_active == (1,)


def test_region_certificate_covers_all_22_omitted_pairs():
    certificate = build_region_certificate(_region())
    assert certificate.status == CertificateStatus.CERTIFIED.value
    assert certificate.pair_count == 22
    assert certificate.pair_policy_id == OMITTED_PAIR_POLICY_ID
    assert len(certificate.pair_evidence) == 22
    assert all(row.certified for row in certificate.pair_evidence)
    assert json.loads(dump_certificate(certificate))["certificate_id"] \
        == certificate.certificate_id
    assert build_region_certificate(_region()).certificate_id \
        == certificate.certificate_id


def test_region_with_omitted_pair_overlap_is_not_certified():
    center = (0.3, 0.0, 0.8, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0)
    certificate = build_region_certificate(
        _region(center_q=center, task_half_width_q=(0.0,) * 9)
    )
    failed_pairs = {
        row.pair for row in certificate.pair_evidence if not row.certified
    }
    assert certificate.status == CertificateStatus.NOT_CERTIFIED.value
    assert certificate.reason_codes == (GEOMETRY_OVERLAP,)
    assert ("Link3", "Link5") in failed_pairs


def test_domain_coverage_binds_region_identity_and_clearance():
    certificate = build_region_certificate(_region())
    required = replace(_region(), task_half_width_q=(0.0005,) * 9)
    covered = assess_domain_coverage(
        required,
        certificate,
        query_id="coverage-query",
        boundary="filtered_command",
        expected_model_id=GEOMETRY_MODEL_ID,
        expected_certificate_id=certificate.certificate_id,
        expected_pair_policy_id=OMITTED_PAIR_POLICY_ID,
    )
    assert covered.status == DomainCoverageStatus.COVERED.value
    assert covered.reason_codes == ()

    outside = assess_domain_coverage(
        replace(required, center_q=(0.203,) + required.center_q[1:]),
        certificate,
        query_id="coverage-query",
        boundary="filtered_command",
    )
    assert outside.status == DomainCoverageStatus.OUTSIDE.value
    assert outside.reason_codes == (DOMAIN_OUTSIDE_CERTIFICATE,)

    mismatched = assess_domain_coverage(
        replace(required, attachment_id="payload-a"),
        certificate,
        query_id="coverage-query",
        boundary="filtered_command",
    )
    assert mismatched.status == DomainCoverageStatus.INDETERMINATE.value
    assert CERTIFICATE_IDENTITY_MISMATCH in mismatched.reason_codes

    tampered = assess_domain_coverage(
        required,
        replace(certificate, certificate_id="tampered"),
        query_id="coverage-query",
        boundary="filtered_command",
    )
    assert tampered.status == DomainCoverageStatus.INDETERMINATE.value
    assert CERTIFICATE_IDENTITY_MISMATCH in tampered.reason_codes

    insufficient_clearance = assess_domain_coverage(
        replace(required, required_clearance_m=1.0),
        certificate,
        query_id="coverage-query",
        boundary="filtered_command",
    )
    assert insufficient_clearance.status == DomainCoverageStatus.OUTSIDE.value
    assert insufficient_clearance.reason_codes == (REQUIRED_ENVELOPE_INCOMPLETE,)


def test_domain_coverage_preserves_certificate_failure_reasons():
    bad_certificate = build_region_certificate(
        replace(_region(), center_q=(0.3, 0.0, 0.8, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0),
                task_half_width_q=(0.0,) * 9)
    )
    result = assess_domain_coverage(
        _region(),
        bad_certificate,
        query_id="coverage-query",
        boundary="filtered_command",
    )
    assert result.status == DomainCoverageStatus.INDETERMINATE.value
    assert GEOMETRY_OVERLAP in result.reason_codes


def test_domain_coverage_rejects_malformed_request_and_certificate():
    malformed_request = assess_domain_coverage(
        None,
        None,
        query_id=None,
        boundary="kernel_candidate",
    )
    assert malformed_request.status == DomainCoverageStatus.INDETERMINATE.value
    assert GEOMETRY_INPUT_INVALID in malformed_request.reason_codes
    assert REQUIRED_ENVELOPE_INCOMPLETE in malformed_request.reason_codes
    assert CERTIFICATE_MISSING in malformed_request.reason_codes

    malformed_certificate = assess_domain_coverage(
        _region(),
        object(),
        query_id="coverage-query",
        boundary="kernel_candidate",
    )
    assert malformed_certificate.status == DomainCoverageStatus.INDETERMINATE.value
    assert CERTIFICATE_IDENTITY_MISMATCH in malformed_certificate.reason_codes


def test_offline_evidence_exports_input_and_geometry_identities():
    region = _region()
    document = {
        "scope_statement": "OFF-02 test evidence",
        "unknown_items": ["physical error bounds are unavailable"],
        "region": {
            "center_q": list(region.center_q),
            "task_half_width_q": list(region.task_half_width_q),
            "state_error_half_width_q": list(region.state_error_half_width_q),
            "stop_half_width_q": list(region.stop_half_width_q),
            "geometry_error_m": region.geometry_error_m,
            "required_clearance_m": region.required_clearance_m,
            "task_id": region.task_id,
            "attachment_id": region.attachment_id,
            "task_region_source": region.task_region_source,
            "state_error_source": region.state_error_source,
            "stop_envelope_source": region.stop_envelope_source,
            "geometry_error_source": region.geometry_error_source,
        },
    }
    evidence = build_evidence(document)
    assert len(evidence["input_sha256"]) == 64
    assert evidence["model_id"] == evidence["point_assessment"]["model_id"]
    assert evidence["pair_policy_id"] == evidence["region_certificate"]["pair_policy_id"]
    assert evidence["certificate_id"] == evidence["region_certificate"]["certificate_id"]
    assert evidence["point_assessment"]["q"] == region.center_q


def test_region_requires_explicit_evidence_sources():
    with pytest.raises(ValueError, match="evidence sources"):
        replace(_region(), stop_envelope_source="")
