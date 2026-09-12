"""OFF-15 independent geometry, failure, scope and evidence contracts."""
from dataclasses import FrozenInstanceError, replace
import json
import hashlib
import math

import numpy as np
import pytest

from robot_safecontrol_moveit.tracking_evaluator import (
    TrackingEvaluator, TrackingStepData, step_from_result, tool_axis_angle,
)
from robot_safecontrol_moveit.tracking_contract import (
    EvidenceContext, EvaluationScope, Threshold, baseline_thresholds,
)


def _evidence(**overrides):
    data = dict(run_id="analytic-run", kind="analytic", boundary="kernel_candidate",
                model_id="analytic-point", config_id="fixture-v1", trajectory_id="straight-1m",
                data_id="equations-in-test", scenario="straight line without physical hardware",
                measurement="analytic point and same-time reference", time_basis="test seconds")
    data.update(overrides)
    return EvidenceContext(**data)


def _step(progress=0.0, **overrides):
    data = dict(
        ee_pos=np.array([progress, 0., 0.]), reference_position_m=np.array([progress, 0., 0.]),
        reference_tangent=np.array([1., 0., 0.]), ee_rot=np.eye(3), reference_rotation=np.eye(3),
        projected_progress_m=progress, reference_progress_m=progress,
        reference_source_time_s=123., reference_at_endpoint=progress == 1.,
        feedrate_m_s=0.1, actual_tangent_speed_m_s=0.1,
        qp_ok=True, admission_ok=True, overlap=False, step_latency_ms=1.,
        limiting_reason_code=0,
    )
    data.update(overrides)
    return data


def _run(scope=None, evidence=None, **kwargs):
    return TrackingEvaluator(scope=scope or EvaluationScope(1.), evidence=evidence or _evidence(),
                             deadline_ms=20., **kwargs)


def _complete(evaluator, **overrides):
    for index, progress in enumerate(np.linspace(evaluator.scope.start_m, evaluator.scope.end_m, 11)):
        evaluator.update(_step(progress, **overrides), wall_time_s=float(index))
    evaluator.finish("completed")
    return evaluator.report()


def test_analytic_cross_track_removes_tangent_but_endpoint_uses_full_error():
    sample = _step(0., ee_pos=np.array([0.04, 0.003, 0.004]))
    measured = step_from_result(sample)
    assert measured.values["cross_track_m"] == pytest.approx(0.005)
    assert measured.values["pos_error_m"] == pytest.approx(math.sqrt(0.04**2 + 0.005**2))
    endpoint = step_from_result(dict(sample, reference_at_endpoint=True))
    assert endpoint.values["cross_track_m"] == pytest.approx(measured.values["pos_error_m"])


@pytest.mark.parametrize("degrees", [0., 0.01, 45., 90., 179., 180.])
def test_true_tool_axis_angle_handles_large_tilts(degrees):
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    assert tool_axis_angle(rotation, np.eye(3)) == pytest.approx(math.radians(degrees), abs=1e-14)


def test_roll_is_free_and_legacy_sine_cannot_reconstruct_angle():
    c, s = math.cos(1.2), math.sin(1.2)
    roll = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    assert tool_axis_angle(roll, np.eye(3)) == 0.
    legacy = step_from_result(dict(err_6d=np.zeros(6), cross_track_error_m=0.))
    assert legacy.values["legacy_orientation_error"] == 0.
    assert legacy.values["tool_axis_error_rad"] is None
    assert legacy.values["cross_track_m"] is None


def test_rms_and_percentile_use_independent_known_values():
    evaluator = _run()
    for i, error in enumerate((0., 3., 4.)):
        evaluator.update(_step(i / 3., ee_pos=np.array([i / 3., error, 0.])), wall_time_s=float(i))
    stat = evaluator.report().metrics["cross_track_m"]
    assert stat.mean == pytest.approx(7 / 3)
    assert stat.rms == pytest.approx(5 / math.sqrt(3))
    assert stat.p95 == pytest.approx(3.9)
    assert stat.maximum == 4.


def test_full_analytic_path_task_passes_but_provisional_safety_stays_unsettled():
    report = _complete(_run())
    assert report.completed and report.full_path_covered
    assert not report.full_path_verified  # safety/timing criteria remain provisional
    assert report.task_verdict == "pass"
    assert report.online_verdict == "pass"
    assert report.verdict == "insufficient_evidence"
    assert report.completion_fraction == 1.
    assert not hasattr(report, "tracking_score")
    assert "score=" not in report.summary()
    assert "综合评分" not in report.markdown()
    assert "kernel_candidate" in report.markdown()
    assert "未采集边界" in report.markdown()
    json.loads(report.json())


def test_predeclared_subinterval_can_pass_without_full_path_verification():
    report = _complete(_run(scope=EvaluationScope(1., 0.6, 1.)))
    assert report.task_verdict == "pass"
    assert report.completed
    assert report.completion_fraction == 1.  # absolute position, not coverage
    assert report.scope_completion_fraction == 1.
    assert report.initial_progress_m == pytest.approx(0.6)
    assert not report.full_path_verified


def test_outside_tail_cannot_dilute_subinterval_rms_into_a_pass():
    evaluator = _run(scope=EvaluationScope(1., 0., 0.5))
    points = list(np.linspace(0., .5, 11)) + list(np.linspace(.505, 1., 100))
    for i, progress in enumerate(points):
        error = .0004 if progress <= .5 else 0.
        evaluator.update(_step(progress, ee_pos=np.array([progress, error, 0.])), wall_time_s=float(i))
    evaluator.finish("completed")
    report = evaluator.report()
    assert report.metrics["cross_track_m"].rms < .00015  # diagnostic dilution is preserved
    assert report.total_steps == 111
    assert len(json.loads(evaluator.trace_json())["samples"]) == 111
    assert report.task_verdict == "fail"
    assert not report.completed
    assert any("outside predeclared scope" in issue for issue in report.issues)


@pytest.mark.parametrize("progress", [0.499, 0.801])
def test_subinterval_rejects_excursions_even_when_it_returns_to_its_endpoint(progress):
    evaluator = _run(scope=EvaluationScope(1., .5, .8))
    for i, position in enumerate((.5, progress, .8)):
        evaluator.update(_step(position), wall_time_s=float(i))
    evaluator.finish("completed")
    assert not evaluator.report().completed
    assert evaluator.report().task_verdict == "fail"


@pytest.mark.parametrize("before", [.49, .81, float("nan"), float("inf")])
def test_all_supplied_pre_step_boundaries_must_be_valid_and_in_scope(before):
    evaluator = _run(scope=EvaluationScope(1., .5, .8))
    evaluator.update(_step(.5), wall_time_s=0.)
    evaluator.update(_step(.8, projection_before_m=before), wall_time_s=1.)
    evaluator.finish("completed")
    assert not evaluator.report().completed


def test_roundoff_cannot_make_zero_progress_complete_a_tiny_interval():
    evaluator = _run(scope=EvaluationScope(1., 0., 1e-15))
    evaluator.update(_step(0.), wall_time_s=0.)
    evaluator.update(_step(0.), wall_time_s=1.)
    evaluator.finish("completed")
    assert not evaluator.report().completed
    assert evaluator.report().scope_completion_fraction == 0.


@pytest.mark.parametrize("roundoff_factor, completed", [(.5, True), (2., False)])
def test_scope_boundary_allows_only_declared_float_roundoff(roundoff_factor, completed):
    evaluator = _run(scope=EvaluationScope(1., 0., .5))
    end = .5 + roundoff_factor * evaluator.scope.roundoff_m
    evaluator.update(_step(0.), wall_time_s=0.)
    evaluator.update(_step(end), wall_time_s=1.)
    evaluator.finish("completed")
    assert evaluator.report().completed is completed


def test_finished_evaluator_public_metadata_is_readonly_for_background_export():
    evaluator = _run()
    _complete(evaluator)
    with pytest.raises(AttributeError):
        evaluator.trajectory_duration_s = 123.
    assert evaluator.termination == "completed"


def test_entering_full_path_at_sixty_percent_cannot_claim_complete_task():
    evaluator = _run()
    for i, progress in enumerate((0.6, 0.8, 1.)):
        evaluator.update(_step(progress), wall_time_s=float(i))
    evaluator.finish("completed")
    report = evaluator.report()
    assert report.completion_fraction == 1.
    assert report.scope_completion_fraction == pytest.approx(0.4)
    assert not report.completed
    assert report.task_verdict == "fail"


def test_source_time_and_reference_endpoint_do_not_prove_progress():
    evaluator = _run(trajectory_duration_s=0.5)
    for i in range(3):
        evaluator.update(_step(0., reference_source_time_s=10000., reference_at_endpoint=True), wall_time_s=float(i))
    evaluator.finish("completed")
    report = evaluator.report()
    assert report.completion_fraction == 0.
    assert not report.completed
    assert report.task_verdict == "fail"


@pytest.mark.parametrize("reason", ["held", "cancelled", "fault", "task_conflict", "interrupted"])
def test_termination_before_completion_is_never_success(reason):
    evaluator = _run()
    evaluator.update(_step(0.), wall_time_s=0.)
    evaluator.update(_step(0.5, feedrate_m_s=0.), wall_time_s=1.)
    evaluator.finish(reason)
    assert not evaluator.report().completed
    assert evaluator.report().task_verdict == "fail"
    assert evaluator.report().termination == reason


def test_safety_slowdown_is_diagnostic_and_does_not_penalize_task():
    report = _complete(_run(), feedrate_m_s=0.01, limiting_reason_code=2)
    assert report.task_verdict == "pass"
    assert report.limiting_reason_counts == {2: 11}


def test_empty_and_missing_samples_remain_unknown():
    evaluator = _run()
    empty = evaluator.report()
    assert empty.metrics["cross_track_m"].mean is None
    assert empty.qp_success_rate is None
    assert empty.task_verdict != "pass"
    evaluator.update({}, wall_time_s=0.)
    report = evaluator.report()
    assert report.qp_missing_count == 1
    assert report.qp_success_rate is None
    assert report.metrics["cross_track_m"].missing == 1
    assert not report.completed
    assert report.verdict != "pass"
    json.loads(report.json())


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_geometry_cannot_be_erased_by_valid_samples(bad):
    evaluator = _run()
    evaluator.update(_step(0.), wall_time_s=0.)
    evaluator.update(_step(0.5, ee_pos=np.array([0.5, bad, 0.])), wall_time_s=1.)
    evaluator.update(_step(1.), wall_time_s=2.)
    evaluator.finish("completed")
    report = evaluator.report()
    assert report.metrics["cross_track_m"].invalid == 1
    assert report.metrics["cross_track_m"].count == 2
    assert not report.completed
    assert report.task_verdict != "pass"
    text = report.json()
    assert "NaN" not in text and "Infinity" not in text


def test_180_degree_orientation_is_a_failed_criterion_even_when_legacy_is_zero():
    reverse = np.diag([-1., -1., 1.])
    report = _complete(_run(), ee_rot=reverse, err_6d=np.zeros(6))
    assert report.task_verdict == "fail"
    assert report.metrics["tool_axis_error_rad"].maximum == pytest.approx(math.pi)


@pytest.mark.parametrize("qp", [None, float("nan"), "true", 1])
def test_qp_status_requires_explicit_boolean(qp):
    report = _complete(_run(), qp_ok=qp)
    assert report.qp_success_rate is None
    assert report.task_verdict != "pass"


def test_single_admission_rejection_cannot_be_hidden_by_qp_success():
    evaluator = _run()
    evaluator.update(_step(0.), wall_time_s=0.)
    evaluator.update(_step(0.5, admission_ok=False), wall_time_s=1.)
    evaluator.update(_step(1.), wall_time_s=2.)
    evaluator.finish("completed")
    report = evaluator.report()
    assert report.qp_success_rate == 1.
    assert report.online_verdict == report.task_verdict == "fail"
    assert report.admission_counts["rejected"] == 1


def test_fault_event_survives_later_good_samples():
    evaluator = _run()
    evaluator.record_event("command_rejected")
    assert _complete(evaluator).verdict == "fail"


def test_unknown_gate_is_not_inferred_from_solver_or_clearance():
    report = _complete(_run(), admission_ok=None, overlap=None, obstacle_clearance_m=1.)
    assert report.online_verdict == "insufficient_evidence"


@pytest.mark.parametrize("times", [(0., 0., 1.), (0., 2., 1.), (0., None, 2.)])
def test_missing_duplicate_or_reversed_timestamps_prevent_pass(times):
    evaluator = _run()
    for progress, stamp in zip((0., 0.5, 1.), times):
        evaluator.update(_step(progress), wall_time_s=stamp)
    evaluator.finish("completed")
    assert evaluator.report().task_verdict != "pass"


def test_progress_regression_and_nan_cannot_prove_coverage():
    for sequence in ((0., 0.8, 0.4, 1.), (0., float("nan"), 1.)):
        evaluator = _run()
        for i, progress in enumerate(sequence):
            evaluator.update(_step(progress), wall_time_s=float(i))
        evaluator.finish("completed")
        assert not evaluator.report().completed
        json.loads(evaluator.report().json())


def test_scope_and_history_cannot_be_changed_after_sampling():
    evaluator = _run()
    step = _step()
    evaluator.update(step, wall_time_s=0.)
    step["ee_pos"][1] = 100.
    assert evaluator.report().metrics["cross_track_m"].maximum == 0.
    with pytest.raises(FrozenInstanceError):
        evaluator.scope.start_m = 0.6
    with pytest.raises(AttributeError):
        evaluator.scope = EvaluationScope(1., 0.6, 1.)
    evaluator.finish("held")
    with pytest.raises(RuntimeError):
        evaluator.finish("completed")
    with pytest.raises(RuntimeError):
        evaluator.update(_step(1.))


@pytest.mark.parametrize("scope", [(0.,), (1., 1., 1.), (1., -0.1, 1.), (1., 0., 2.), (float("nan"),)])
def test_invalid_scope_is_rejected(scope):
    with pytest.raises(ValueError):
        EvaluationScope(*scope)


def test_evidence_missing_and_wrong_hardware_identity_cannot_pass():
    report = _complete(_run(evidence=EvidenceContext()))
    assert report.task_verdict == "insufficient_evidence"
    with pytest.raises(ValueError):
        _evidence(kind="model", boundary="real_feedback")
    hardware = _evidence(kind="hardware_recording", boundary="real_feedback")
    assert _complete(_run(evidence=hardware)).verdict == "insufficient_evidence"


def test_deadline_uses_declared_budget_and_preserves_provisional_state():
    evaluator = _run()
    for i, latency in enumerate((1., 20., 21.)):
        evaluator.update(_step(i / 2., step_latency_ms=latency), wall_time_s=float(i))
    evaluator.finish("completed")
    report = evaluator.report()
    assert report.deadline_miss_rate == pytest.approx(1 / 3)
    criterion = next(c for c in report.criteria if c.threshold.metric == "deadline_miss_rate")
    assert criterion.threshold.status == "provisional"
    assert criterion.verdict == "insufficient_evidence"


def test_unknown_threshold_can_report_value_without_pass():
    threshold = Threshold("cross_track_m.maximum", None, "<=", "m", "task", "unknown", "awaiting measurement")
    report = _complete(_run(thresholds=(threshold,)))
    criterion = next(c for c in report.criteria if c.threshold.metric == threshold.metric)
    assert criterion.value == 0.
    assert criterion.verdict == "insufficient_evidence"
    with pytest.raises(ValueError):
        replace(threshold, status="accepted")


def test_constraint_classes_keep_units_sources_and_missing_ticks():
    evaluator = _run()
    quantities = {
        "linear.residual": dict(value=0.002, quantity="unrelaxed_residual", unit="m/s", source="analytic-row"),
        "angular.residual": dict(value=0.003, quantity="unrelaxed_residual", unit="rad/s", source="analytic-row"),
    }
    evaluator.update(_step(0., constraint_metrics=quantities), wall_time_s=0.)
    evaluator.update(_step(1., constraint_metrics={"angular.residual": quantities["angular.residual"]}), wall_time_s=1.)
    report = evaluator.report()
    assert report.constraint_metrics["linear.residual"]["statistics"]["missing"] == 1
    assert report.constraint_metrics["angular.residual"]["unit"] == "rad/s"
    assert report.constraint_metrics["linear.residual"]["unit"] == "m/s"
    assert report.constraint_metrics["angular.residual"]["statistics"]["maximum"] == 0.003
    json.loads(report.json())


def test_changed_constraint_unit_invalidates_that_series():
    evaluator = _run()
    for i, unit in enumerate(("m/s", "rad/s")):
        evaluator.update(_step(i, constraint_metrics={"row": dict(value=1., unit=unit, quantity="residual", source="fixture")}), wall_time_s=float(i))
    assert evaluator.report().constraint_metrics["row"]["statistics"]["invalid"] == 2


def test_disabled_obstacle_sentinel_is_not_physical_clearance():
    report = _complete(_run(), min_obs_dist=1.)
    assert report.metrics["obstacle_margin_m"].mean == 1.
    assert report.metrics["obstacle_clearance_m"].mean is None


def test_typed_input_does_not_bypass_validity_checks():
    evaluator = _run()
    evaluator.update(TrackingStepData(
        values={"cross_track_m": -1., "tool_axis_error_rad": 4.},
        qp_ok=float("nan"), projected_progress_m="invalid"), wall_time_s=0.)
    report = evaluator.report()
    assert report.qp_success_rate is None
    assert report.metrics["cross_track_m"].invalid == 1
    assert report.metrics["tool_axis_error_rad"].invalid == 1
    json.loads(report.json())


def test_samples_from_different_boundaries_cannot_be_merged():
    evaluator = _run()
    with pytest.raises(ValueError, match="separate report"):
        evaluator.update(_step(measurement_boundary="real_feedback"), wall_time_s=0.)
    assert _complete(evaluator).verdict == "fail"


def test_independent_boundary_reports_preserve_evidence_kind():
    for boundary, kind in (("kernel_candidate", "model"), ("filtered_command", "model"),
                           ("simulated_state", "simulation"), ("real_feedback", "hardware_recording")):
        report = _complete(_run(evidence=_evidence(boundary=boundary, kind=kind)))
        assert report.evidence.boundary == boundary
        assert report.evidence.kind == kind
        assert report.verdict == "insufficient_evidence"


def test_custom_threshold_cannot_remove_other_mandatory_task_criteria():
    only_qp = next(t for t in baseline_thresholds() if t.metric == "qp_success_rate")
    report = _complete(_run(thresholds=(only_qp,)), ee_rot=np.diag([-1., -1., 1.]))
    assert report.task_verdict == "fail"


def test_wrong_unit_prevents_threshold_pass():
    threshold = Threshold("cross_track_m.maximum", 2., "<=", "mm", "task", "accepted", "fixture unit mismatch")
    report = _complete(_run(thresholds=(threshold,)))
    criterion = next(c for c in report.criteria if c.threshold.metric == threshold.metric)
    assert criterion.verdict == "insufficient_evidence"
    assert "unit" in criterion.reason


def test_sourced_per_class_numerical_threshold_is_separate_from_task_tolerance():
    threshold = Threshold("linear.residual.maximum", 1e-6, "<=", "m/s", "numerical", "accepted", "analytic fixture only")
    quantities = {"linear.residual": dict(value=2e-6, quantity="unrelaxed_residual", unit="m/s", source="analytic-row")}
    report = _complete(_run(thresholds=(threshold,)), constraint_metrics=quantities)
    criterion = next(c for c in report.criteria if c.threshold.metric == threshold.metric)
    assert criterion.value == 2e-6
    assert criterion.verdict == "fail"
    assert report.task_verdict == "pass"
    assert report.verdict == "fail"


def test_analytic_complete_acceptance_never_claims_hardware():
    thresholds = tuple(replace(t, status="accepted", source="analytic fixture only")
                       for t in baseline_thresholds() if t.category != "task")
    report = _complete(_run(thresholds=thresholds), obstacle_clearance_m=0.1)
    assert report.verdict == "pass"
    assert report.full_path_verified
    assert report.evidence.kind == "analytic"
    assert "不授予实机准入" in report.markdown()


def test_trace_hash_binds_report_and_preserves_invalid_sample():
    evaluator = _run()
    evaluator.update(_step(0., tool_axis_error_rad=float("nan"), ee_rot=None), wall_time_s=0.)
    trace = evaluator.trace_json()
    assert evaluator.report().sample_data_sha256 == hashlib.sha256(trace.encode()).hexdigest()
    record = json.loads(trace)["samples"][0]
    assert record["values"]["tool_axis_error_rad"] == {"nonfinite": "nan"}


def test_negative_clearance_fails_accepted_floor_despite_unsettled_30mm_target():
    report = _complete(_run(), obstacle_clearance_m=-0.001)
    floor = next(c for c in report.criteria if c.threshold.metric == "obstacle_clearance_nonnegative")
    assert floor.threshold.status == "accepted"
    assert floor.verdict == "fail"
    assert report.online_verdict == report.task_verdict == report.verdict == "fail"
    target = next(c for c in report.criteria if c.threshold.metric == "obstacle_clearance_m.minimum")
    assert target.threshold.status == "provisional"
    assert target.verdict == "insufficient_evidence"
