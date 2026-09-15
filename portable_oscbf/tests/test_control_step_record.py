"""Contract between the control node's step record and its OFF-15 reader.

The record the node hands to the evaluator is the facade's
``JaxPathTrackingResult``, and two files name its slots independently: the
facade defines them, and the evaluator lists the ones it reads in
``STEP_RECORD_SLOTS`` (it must stay importable without JAX, so it cannot import
the facade).  These tests are what keeps the second list a subset of the first
instead of a copy that drifts -- the drift this file was written to catch was
``qp_primal_residual``/``constraint_metrics``, which the evaluator reads but the
kernel record spells differently.
"""

from robot_safecontrol_moveit.tracking_evaluator import (
    STEP_RECORD_SLOTS, step_from_record,
)
from work.jax_control_facade import JaxPathTrackingResult

# Slots the evaluator accepts from its caller instead of from the record: they
# are host-side measurements, not step outputs (docs/tracking_evaluation.md).
CALLER_SUPPLIED_SLOTS = frozenset({
    "admission_ok", "overlap", "step_latency_ms", "projection_before_m",
})


def test_evaluator_slots_are_record_fields_or_caller_supplied():
    assert set(STEP_RECORD_SLOTS) - set(JaxPathTrackingResult.__dataclass_fields__) \
        == CALLER_SUPPLIED_SLOTS


def test_step_from_record_reads_every_record_slot_without_raising():
    """The typed entry must tolerate a record whose optional slots are unset."""
    data = step_from_record(JaxPathTrackingResult(
        q_next=None, u_safe=None, u_nom=None, err_6d=None, ee_pos=None,
        ee_rot=None, qp_ok=True, min_obs_dist=None, path_state=None,
        reference_position_m=None, reference_rotation=None,
        posture_reference=None, reference_tangent=None,
        reference_omega_per_m=None, reference_source_time_s=None,
        cross_track_error_m=None, gamma=None, feedrate_nominal_m_s=None,
        feedrate_m_s=None, feedrate_joint_limit_m_s=None,
        feedrate_cbf_limit_m_s=None, feedrate_rate_limit_m_s=None,
        feedrate_tool_axis_limit_m_s=None, feedrate_endpoint_brake_limit_m_s=None,
        limiting_reason_code=None, actual_tangent_speed_m_s=None,
        reference_at_endpoint=None,
    ))
    # Gates survive as booleans; slots the caller did not supply stay unmeasured
    # rather than becoming zeros.
    assert data.qp_ok is True
    assert data.admission_ok is None and data.overlap is None
    assert data.values["step_latency_ms"] is None
    assert data.projection_before_m is None
