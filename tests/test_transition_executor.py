from dataclasses import FrozenInstanceError

import pytest

from robot_safecontrol_moveit.transition_executor import TransitionResult


@pytest.mark.parametrize("code,success", [
    ("TRANSITION_PLANNED", True), ("PLAN_ONLY_SUCCESS", True),
    ("TRANSITION_REPLAYED", True), ("TRANSITION_EXECUTED", True),
    ("START_STATE_UNAVAILABLE", False), ("PLANNER_FAILED", False),
    ("TRANSITION_EXECUTION_FAILED", False), ("UNEXPECTED_ERROR", False),
    ("PLANNING_ALREADY_RUNNING", False),
])
def test_result_success_is_derived_and_immutable(code, success):
    result = TransitionResult(code, 42, 3.14159, (("detail", "diagnostic"),))
    assert result.success is success
    with pytest.raises(FrozenInstanceError):
        result.code = "OTHER"
    with pytest.raises(FrozenInstanceError):
        result.success = not success
    with pytest.raises(TypeError):
        TransitionResult(code, success=success)


@pytest.mark.parametrize("handoff_code", [
    "TRACKING_STARTED", "ALREADY_TRACKING", "START_SERVICE_UNAVAILABLE",
    "START_SERVICE_TIMEOUT", "START_SERVICE_FAILED",
])
def test_handoff_protocol_data_preserves_replay_success(handoff_code):
    result = TransitionResult("TRANSITION_REPLAYED", 9, .5,
                              handoff_requested=True, handoff_code=handoff_code)
    assert result.success
    assert result.handoff_requested and result.handoff_code == handoff_code
    assert TransitionResult("TRANSITION_REPLAYED").handoff_code is None


def test_result_rejects_inconsistent_or_mutable_facts():
    with pytest.raises(ValueError):
        TransitionResult("TRANSITION_REPLAYED", handoff_code="TRACKING_STARTED")
    with pytest.raises(ValueError):
        TransitionResult("TRANSITION_REPLAYED", handoff_requested=True)
    with pytest.raises(TypeError):
        TransitionResult("GOAL_IK_FAILED", context=[("detail", "failed")])
