import pytest

from robot_safecontrol_moveit.transition_executor import TransitionResult, VALID_RESULT_MODES
from robot_safecontrol_moveit.transition_planning_server import format_transition_result


def test_ros_result_format_preserves_field_order_and_precision():
    result = TransitionResult("TRANSITION_PLANNED", 42, 3.14159)
    assert format_transition_result(result) == (
        "error_code=TRANSITION_PLANNED|trajectory_points=42|planning_time=3.142"
    )


def test_ros_diagnostics_keep_order_and_literal_values():
    result = TransitionResult("GOAL_IK_FAILED", 0, 1.5, (
        ("detail", "IK rejected: x=y"), ("moveit_error_code", "-31"),
        ("planning_group", "arm"), ("seed_names", "J1,J2"),
    ))
    assert format_transition_result(result) == (
        "error_code=GOAL_IK_FAILED|trajectory_points=0|planning_time=1.500|"
        "detail=IK rejected: x=y|moveit_error_code=-31|planning_group=arm|seed_names=J1,J2"
    )


@pytest.mark.parametrize("code", [None, "TRACKING_STARTED", "START_SERVICE_TIMEOUT", "START_SERVICE_FAILED"])
def test_handoff_fields_do_not_change_existing_trigger_text(code):
    result = TransitionResult("TRANSITION_REPLAYED", 10, 2.5,
                              handoff_requested=code is not None, handoff_code=code)
    assert result.success
    assert format_transition_result(result) == (
        "error_code=TRANSITION_REPLAYED|trajectory_points=10|planning_time=2.500"
    )


def test_supported_modes_are_unchanged():
    assert VALID_RESULT_MODES == {"plan_only", "joint_state_replay", "moveit_execute"}
