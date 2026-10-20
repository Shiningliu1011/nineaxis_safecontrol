import numpy as np
import pytest

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from pathlib import Path

bootstrap_portable(Path(__file__).resolve().parents[1] / "portable_oscbf")

from work.static_recovery import RecoveryRules, RecoveryState


def _step(state, time_s, **changes):
    values = dict(
        time_s=time_s, position_error_m=.0001, angle_rad=np.deg2rad(.01),
        positive_candidate=True, endpoint=False, nominal_feed=.01,
        executed_feed=0., period_s=.02,
    )
    values.update(changes)
    return state.update(**values)


def test_improved_requires_continuous_recovery_confirmation():
    state = RecoveryState(RecoveryRules("improved"))
    assert _step(state, 0.) == 0
    assert state.phase == "RECOVERING"
    assert _step(state, .19) == 0
    assert _step(state, .2, positive_candidate=False) == 0
    assert _step(state, .21) == 0
    assert _step(state, .4) == 0
    assert _step(state, .42) == pytest.approx(.0006)
    assert state.phase == "RUNNING"
    assert _step(state, .44, position_error_m=.002) == 0
    assert state.phase == "REFERENCE_PAUSED"
    assert _step(state, .46, angle_rad=np.deg2rad(.1)) == 0


def test_basic_recovers_on_positive_candidate_and_ramps_feed():
    state = RecoveryState(RecoveryRules("basic"))
    assert _step(state, 0., positive_candidate=False) == 0
    assert _step(state, .02) == pytest.approx(.0006)
    assert _step(state, .04, executed_feed=.0006) == pytest.approx(.0012)
    assert _step(state, .06, positive_candidate=False) == 0
    assert _step(state, .08) == pytest.approx(.0006)


def test_timeout_and_endpoint_are_explicit_terminal_conditions():
    state = RecoveryState(RecoveryRules("improved"))
    assert _step(state, 0., positive_candidate=False) == 0
    assert _step(state, 30.001, positive_candidate=False) == 0
    assert state.phase == "FAULT" and state.reason == "pause_timeout"
    assert _step(state, 31.) == 0
    state = RecoveryState(RecoveryRules("basic"))
    assert _step(state, 0., endpoint=True) == 0
    assert state.phase == "ENDPOINT_HOLD"


def test_recovery_thresholds_must_be_stricter_than_pause():
    with pytest.raises(ValueError):
        RecoveryRules("improved", resume_position_m=.001)


def test_rejected_admission_restarts_confirmation_and_preserves_pause_timeout():
    state = RecoveryState(RecoveryRules("improved"))
    assert _step(state, 0.) == 0.
    assert _step(state, .2) > 0.
    state.admission_rejected(.21, "late")
    assert state.phase == "REFERENCE_PAUSED"
    assert _step(state, .3) == 0.
    state.admission_rejected(.31, "late")
    assert _step(state, .4) == 0.
    assert _step(state, .6) == 0.
    assert _step(state, .61) > 0.
    state.admission_rejected(.62, "late")
    assert _step(state, .7, positive_candidate=False) == 0.
    state.admission_rejected(30., "late")
    assert _step(state, 30.63, positive_candidate=False) == 0.
    assert state.phase == "FAULT" and state.reason == "pause_timeout"
