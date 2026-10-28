from dataclasses import replace
from pathlib import Path

import jsonschema
import numpy as np
import pytest
import yaml
from work.nineaxis_model import NineaxisModel
from work.process_limits import (
    PhaseEvidence,
    ProcessLimits,
    ProcessObservation,
    ReferenceFeed,
)
from work.robot_units import joints_si_to_mm_deg

ROOT = Path(__file__).resolve().parents[2]
PATH_HASH, WORKPIECE_HASH, SCENE_HASH = (character * 64 for character in "abc")


@pytest.fixture(scope="module")
def model():
    return NineaxisModel.load(root=ROOT)


@pytest.fixture(scope="module")
def limits():
    return ProcessLimits.load()


def observation(model, *, layer="actual", velocity=.02, acceleration=.03, jerk=.04):
    q = (model.position_lower + model.position_upper) / 2
    derivatives = np.zeros((3, 9))
    derivatives[:, 0] = [velocity, acceleration, jerk]
    motion = model.tool_motion(q, *derivatives, sample_source=(
        "simulation_feedback" if layer == "actual" else "generated_motion"), time_s=1., clock_id="sim-v1")
    return ProcessObservation(
        motion=motion, layer=layer, source_identity="d" * 64, path_hash=PATH_HASH,
        workpiece_hash=WORKPIECE_HASH, scene_hash=SCENE_HASH,
        tangent_derivatives=np.array([[0., 0., 1.], [.5, 0., 0.], [0., 0., -.25]]),
        workpiece_normal=np.array([1., 0., 0.]),
        reference_position_m=motion.position_m + [.0002, 0., 0.],
        reference_axis=motion.axis_derivatives[0],
        projected_position_m=motion.position_m + [.0001, 0., 0.],
        work_distances_m=np.array([.05, .05, 0.]), external_clearance_m=.04, self_clearance_m=.02,
        projection_identity="1" * 64, projection_time_s=1., projection_clock_id="sim-v1", projection_valid=True,
    )


def bind(limits, model, phase="tracking"):
    return limits.bind(model, path_hash=PATH_HASH, workpiece_hash=WORKPIECE_HASH,
                       scene_hash=SCENE_HASH, path_length_m=2., phase=phase, evidence_scope="research_simulation")


def test_confirmed_joint_arrays_metadata_and_si_conversion(limits, model):
    expected_velocity = [500.] + [120.] * 5 + [150.] * 3
    expected_acceleration = [5000.] + [120.] * 5 + [150.] * 3
    expected_jerk = [62500.] + [600.] * 5 + [750.] * 3
    bound = bind(limits, model)
    for name, expected in (("max_velocity", expected_velocity), ("max_acceleration", expected_acceleration),
                           ("max_jerk", expected_jerk)):
        np.testing.assert_allclose(joints_si_to_mm_deg(getattr(bound, name)), expected, atol=2e-12, rtol=0)
        with pytest.raises(ValueError):
            getattr(bound, name).setflags(write=True)
    np.testing.assert_array_equal(bound.position_lower, model.position_lower)
    np.testing.assert_array_equal(bound.position_upper, model.position_upper)
    assert len(limits.config_hash) == 64 and bound.identity != limits.config_hash
    assert limits.parameters["minimum_processing_feed"].value_or_bounds is None
    assert "#" in limits.parameters["minimum_processing_feed"].acquisition
    assert limits.parameters["feed_max"].unit == "mm/s"
    for parameter in limits.parameters.values():
        assert parameter.quantity and parameter.object and parameter.frame and parameter.phases
        assert parameter.source and parameter.assumptions and parameter.verification_state
    with pytest.raises(TypeError):
        limits.parameters["unexpected"] = limits.parameters["feed_max"]
    assert bound.cbf_margin == .01 and bound.start_ik_margin == .02
    assert limits.parameters["existing_position_manipulability_min"].value_or_bounds == .005
    assert limits.parameters["existing_position_manipulability_activation"].value_or_bounds == .05
    with pytest.raises(RuntimeError, match="hardware"):
        limits.bind(model, path_hash=PATH_HASH, workpiece_hash=WORKPIECE_HASH,
                    scene_hash=SCENE_HASH, path_length_m=2., phase="tracking", evidence_scope="hardware")


def test_config_schema_units_values_and_expected_identity_reject_changes(limits, tmp_path):
    original = yaml.safe_load((ROOT / "portable_oscbf/config/nineaxis_research_limits.yaml").read_text())
    for change in (lambda data: data.update(schema_version=2),
                   lambda data: data["parameters"]["feed_max"].update(unit="mph"),
                   lambda data: data["parameters"]["feed_max"].update(value_or_bounds=float("nan")),
                   lambda data: data["parameters"]["joint_max_jerk"].update(value_or_bounds=[1.] * 8)):
        values = yaml.safe_load(yaml.safe_dump(original))
        change(values)
        path = tmp_path / "changed.yaml"
        path.write_text(yaml.safe_dump(values))
        with pytest.raises((ValueError, jsonschema.ValidationError)):
            ProcessLimits.load(path)
    original["parameters"]["feed_max"]["source"] += " independent revision"
    path = tmp_path / "source-change.yaml"
    path.write_text(yaml.safe_dump(original))
    changed = ProcessLimits.load(path)
    assert changed.config_hash != limits.config_hash
    with pytest.raises(ValueError, match="expected_hash"):
        ProcessLimits.load(path, expected_hash=limits.config_hash)


def test_actual_quantities_include_tangent_time_change_and_normal_planes(limits, model):
    sample = observation(model)
    report = bind(limits, model).check(sample)
    assert report.accepted and not report.pause_required
    metrics = report.metrics
    assert metrics["tcp_speed_mm_s"] == pytest.approx(20.)
    assert metrics["feed_mm_s"] == pytest.approx(20.)
    assert metrics["feed_acceleration_mm_s2"] == pytest.approx(30.)
    # T_ddot·v 的非零贡献与 Cartesian jerk 投影分别保存。
    assert metrics["feed_jerk_mm_s3"] == pytest.approx(35.)
    assert metrics["cartesian_tangent_jerk_mm_s3"] == pytest.approx(40.)
    assert metrics["normal_plane_acceleration_mm_s2"] == pytest.approx(0., abs=1e-12)
    assert metrics["workpiece_normal_speed_mm_s"] == pytest.approx(0., abs=1e-12)
    assert metrics["reference_position_error_mm"] == pytest.approx(.2)
    assert metrics["cross_track_mm"] == pytest.approx(.1)
    assert metrics["axis_error_deg"] == pytest.approx(0., abs=1e-12)
    assert report.layer == "actual" and report.source_identity == sample.source_identity
    assert "work_distance_tolerance" in report.unconfigured
    required = bind(limits, model).check(sample, required_quantities=("minimum_processing_feed",))
    assert not required.accepted and required.missing_required == ("minimum_processing_feed",)


def test_reference_and_actual_layers_preserve_distinct_source_and_limits(limits, model):
    actual = observation(model)
    reference = replace(observation(model, layer="execution_reference"), reference_feed=ReferenceFeed(
        length_m=2., nu_s_inv=.02, nu_dot_s_inv2=.03, nu_ddot_s_inv3=.04, curvature_m_inv=2.,
        source_identity="e" * 64, path_hash=PATH_HASH, time_s=1., clock_id="sim-v1"))
    nominal = replace(reference, layer="source_nominal", source_identity="f" * 64,
                      reference_feed=replace(reference.reference_feed, nu_s_inv=.04))
    actual_report = bind(limits, model).check(actual)
    reference_report = bind(limits, model).check(reference)
    nominal_report = bind(limits, model).check(nominal)
    assert actual_report.accepted and reference_report.accepted
    assert not nominal_report.accepted and "feed_max" in nominal_report.violations
    assert reference_report.metrics["reference_feed_mm_s"] == pytest.approx(40.)
    assert reference_report.metrics["reference_normal_acceleration_mm_s2"] == pytest.approx(3.2)
    assert reference_report.metrics["reference_curvature_jerk_mm_s3"] == pytest.approx(.256)
    assert {actual_report.layer, reference_report.layer, nominal_report.layer} == {
        "actual", "execution_reference", "source_nominal"}


def test_identity_validity_and_missing_geometry_are_explicit(limits, model):
    bound, sample = bind(limits, model), observation(model)
    for name in ("path_hash", "workpiece_hash", "scene_hash"):
        with pytest.raises(ValueError, match=name):
            bound.check(replace(sample, **{name: "0" * 64}))
    with pytest.raises(ValueError, match="model_hash"):
        bound.check(replace(sample, motion=replace(sample.motion, model_hash="0" * 64)))
    with pytest.raises(ValueError, match="phase"):
        bind(limits, model, "unknown")
    for tangent in (np.ones((2, 3)), np.full((3, 3), np.nan), np.ones((3, 3))):
        with pytest.raises(ValueError, match="tangent"):
            replace(sample, tangent_derivatives=tangent)
    with pytest.raises(ValueError, match="actual"):
        replace(sample, motion=replace(sample.motion, sample_source="generated_motion"))
    for changes in ({"projection_time_s": .5}, {"projection_clock_id": "other"}, {"projection_valid": False}):
        with pytest.raises(ValueError, match="projection"):
            replace(sample, **changes)
    missing = bound.check(replace(sample, external_clearance_m=None))
    assert not missing.accepted and "external_clearance" in missing.missing_required


def test_stopping_motion_violations_are_checked_and_recovery_endpoint_need_evidence(limits, model):
    sample = observation(model, velocity=.07, acceleration=.2, jerk=20.)
    report = bind(limits, model, "stopping").check(sample)
    assert not report.accepted
    assert {"feed_max", "feed_acceleration_max", "feed_jerk_max"}.issubset(report.violations)
    good = observation(model)
    displaced = replace(good, reference_position_m=good.motion.position_m + [.0011, 0., 0.])
    assert bind(limits, model).check(displaced).pause_required
    recovery = bind(limits, model, "recovery")
    assert not recovery.phase_status(good, PhaseEvidence(10., .1, .1, False, False, waiting_elapsed_s=.1)).ready_to_transition
    assert recovery.phase_status(good, PhaseEvidence(10., .3, .3, False, False, True, .3)).ready_to_transition
    endpoint = bind(limits, model, "endpoint_hold")
    good = observation(model, velocity=0., acceleration=0., jerk=0.)
    assert not endpoint.phase_status(good, PhaseEvidence(10., 1., 1., False, True)).ready_to_transition
    assert not endpoint.phase_status(good, PhaseEvidence(10., 1., .2, True, True)).ready_to_transition
    assert endpoint.phase_status(good, PhaseEvidence(10., 1., .6, True, True)).ready_to_transition
    assert not endpoint.phase_status(good, PhaseEvidence(421., 1., .6, True, True)).can_continue
    assert not bind(limits, model, "paused").phase_status(good, PhaseEvidence(41., 31., 0., True, False, waiting_elapsed_s=31.)).can_continue
    assert not recovery.phase_status(good, PhaseEvidence(41., .3, .3, True, False, True, 31.)).can_continue


def test_newly_supplied_optional_numeric_limits_are_enforced(limits, model, tmp_path):
    values = yaml.safe_load((ROOT / "portable_oscbf/config/nineaxis_research_limits.yaml").read_text())
    values["parameters"]["minimum_processing_feed"].update(value_or_bounds=30., verification_state="confirmed_research",
                                                          source="analytic lower-bound acceptance", acquisition="")
    path = tmp_path / "processing-feed.yaml"
    path.write_text(yaml.safe_dump(values))
    configured = ProcessLimits.load(path)
    report = bind(configured, model).check(observation(model))
    assert not report.accepted and "minimum_processing_feed" in report.violations
    assert "minimum_processing_feed" not in report.unconfigured
    assert bind(configured, model, "stopping").check(observation(model, velocity=0., acceleration=0., jerk=0.)).accepted


def test_replacing_a_parameter_builds_a_new_validated_identity(limits, model):
    changed = replace(limits, parameters={**limits.parameters, "feed_max": replace(
        limits.parameters["feed_max"], value_or_bounds=55., source="analytic 55 mm/s check")})
    assert changed.config_hash != limits.config_hash
    assert bind(changed, model).identity != bind(limits, model).identity
    with pytest.raises((ValueError, jsonschema.ValidationError)):
        replace(limits, parameters={**limits.parameters, "feed_max": replace(limits.parameters["feed_max"], unit="mph")})


def test_tracking_quality_uses_actual_errors_and_true_axis_angles(limits, model):
    sample = observation(model)
    report = bind(limits, model).tracking_quality([sample, sample])
    assert report.accepted and report.metrics["cross_track_rms_mm"] == pytest.approx(.1)
    disturbed = replace(sample, reference_axis=-sample.reference_axis)
    report = bind(limits, model).tracking_quality([sample, disturbed])
    assert not report.accepted and report.metrics["axis_max_deg"] == pytest.approx(180.)
    with pytest.raises(ValueError, match="empty"):
        bind(limits, model).tracking_quality([])


@pytest.mark.parametrize("quantity", ["cross_track_max", "axis_max", "external_clearance", "self_clearance"])
def test_configured_sample_phases_apply_each_limit_only_in_the_declared_stage(limits, model, quantity):
    configured = replace(limits, parameters={**limits.parameters, quantity: replace(
        limits.parameters[quantity], phases=("stopping",))})
    sample = observation(model, velocity=0., acceleration=0., jerk=0.)
    changes = {
        "cross_track_max": {"projected_position_m": sample.motion.position_m + [.003, 0., 0.]},
        "axis_max": {"reference_axis": -sample.reference_axis},
        "external_clearance": {"external_clearance_m": .02},
        "self_clearance": {"self_clearance_m": .005},
    }
    sample = replace(sample, **changes[quantity])
    assert quantity in bind(configured, model, "stopping").check(sample).violations
    assert quantity not in bind(configured, model, "tracking").check(sample).violations
    if quantity.endswith("clearance"):
        absent = replace(sample, **{f"{quantity}_m": None})
        assert quantity in bind(configured, model, "stopping").check(absent).missing_required
        assert quantity not in bind(configured, model, "tracking").check(absent).missing_required


@pytest.mark.parametrize("quantity", ["cross_track_rms", "cross_track_p95", "cross_track_max", "axis_rms", "axis_max"])
def test_quality_summary_respects_each_parameter_stage(limits, model, quantity):
    configured = replace(limits, parameters={**limits.parameters, quantity: replace(
        limits.parameters[quantity], phases=("stopping",))})
    sample = observation(model, velocity=0., acceleration=0., jerk=0.)
    sample = replace(sample, projected_position_m=sample.motion.position_m + [.003, 0., 0.],
                     reference_axis=-sample.reference_axis)
    assert quantity in bind(configured, model, "stopping").tracking_quality([sample]).violations
    assert quantity not in bind(configured, model, "tracking").tracking_quality([sample]).violations


@pytest.mark.parametrize("quantity", ["joint_position", "joint_max_velocity", "recovery_position",
                                      "recovery_confirmation", "endpoint_hold", "total_timeout", "source_chord_error"])
def test_phase_rules_with_fixed_roles_reject_unsupported_stage_combinations(limits, quantity):
    with pytest.raises(ValueError, match="phases"):
        replace(limits, parameters={**limits.parameters, quantity: replace(
            limits.parameters[quantity], phases=("stopping",))})


@pytest.mark.parametrize("quantity", ["pause_position", "pause_axis"])
def test_pause_signal_blocks_continuing_the_current_applicable_phase(limits, model, quantity):
    sample = observation(model, velocity=0., acceleration=0., jerk=0.)
    if quantity == "pause_position":
        sample = replace(sample, reference_position_m=sample.motion.position_m + [.0011, 0., 0.])
    else:
        normal = np.cross(sample.reference_axis, [0., 1., 0.])
        normal /= np.linalg.norm(normal)
        angle = np.deg2rad(.25)
        sample = replace(sample, reference_axis=sample.reference_axis * np.cos(angle) + normal * np.sin(angle))
    evidence = PhaseEvidence(10., 2., 0., True, False)
    report = bind(limits, model).check(sample)
    status = bind(limits, model).phase_status(sample, evidence)
    assert report.accepted and report.pause_required
    assert not status.can_continue and not status.ready_to_transition
    assert "pause_required" in status.reasons
    assert bind(limits, model, "stopping").phase_status(sample, evidence).can_continue
    configured = replace(limits, parameters={**limits.parameters, quantity: replace(
        limits.parameters[quantity], phases=("stopping",))})
    assert bind(configured, model, "tracking").phase_status(sample, evidence).can_continue
    stopping = bind(configured, model, "stopping").phase_status(sample, evidence)
    assert not stopping.can_continue and "pause_required" in stopping.reasons


def test_entry_recovery_and_endpoint_conditions_keep_their_declared_roles(limits, model):
    good = observation(model, velocity=0., acceleration=0., jerk=0.)
    assert limits.parameters["recovery_position"].phases == ("entry", "recovery")
    assert bind(limits, model, "entry").phase_status(good, PhaseEvidence(10., 2., 0., True, False)).ready_to_transition
    distant = replace(good, reference_position_m=good.motion.position_m + [.0006, 0., 0.])
    assert not bind(limits, model, "entry").phase_status(distant, PhaseEvidence(10., 2., 0., True, False)).ready_to_transition
    recovery = bind(limits, model, "recovery")
    assert not recovery.phase_status(distant, PhaseEvidence(10., 1., 1., True, False, True, 1.)).ready_to_transition
    assert recovery.phase_status(good, PhaseEvidence(10., 1., 1., True, False, True, 1.)).ready_to_transition
    endpoint = bind(limits, model, "endpoint_hold")
    assert not endpoint.phase_status(distant, PhaseEvidence(10., 1., 1., True, True)).ready_to_transition
    assert endpoint.phase_status(good, PhaseEvidence(10., 1., 1., True, True)).ready_to_transition
