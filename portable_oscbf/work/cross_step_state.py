"""Explicit cross-step memory for the JAX control facade.

Exactly one value genuinely crosses a control step into the compiled kernel:
the previous safe joint velocity, which the CBF rows and the temporal
proximity term read as ``u_safe_prev``.  The CBF right-hand side and its
gradient from the previous step are remembered only to report how far they
moved.  All of it lives in one named structure so that "what is remembered
between steps, and has it been measured" is answered in one place instead of
by loose attributes on the loop.

The ``NamedTuple`` names everything remembered between steps.  It expresses
"not measured" with ``None`` and ``NaN`` because this state never crosses the
jit boundary itself: the previous CBF telemetry keeps the ``None`` the facade
has always stored there, and the two delta norms above it are ``NaN`` when the
step carried no telemetry.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np


class CrossStepState(NamedTuple):
    """The control loop's memory between two compiled steps."""

    u_safe_prev: np.ndarray
    cbf_h_prev: np.ndarray | None
    cbf_grad_prev: np.ndarray | None
    cbf_h_delta_norm: float
    cbf_grad_delta_norm: float


def initial_cross_step_state(num_joints: int) -> CrossStepState:
    """State before the first step: nothing measured, nothing remembered."""
    return CrossStepState(
        u_safe_prev=np.zeros(num_joints),
        cbf_h_prev=None,
        cbf_grad_prev=None,
        cbf_h_delta_norm=0.0,
        cbf_grad_delta_norm=0.0,
    )


def advance_cross_step_state(state: CrossStepState, *,
                             u_safe=None, h_vals=None,
                             cbf_grad=None) -> CrossStepState:
    """Fold one step's outputs into the cross-step state.

    ``h_vals``/``cbf_grad`` are ``None`` when the step ran without CBF
    telemetry (the facade's fast path).  The previous telemetry is then
    dropped rather than kept: a stale gradient must never be reported as the
    current step's change, and the delta norms become ``NaN`` -- unmeasured,
    not zero.

    ``u_safe`` defaults to the command already remembered, so a caller folding
    only telemetry does not have to restate it.
    """
    # Keep the kernel's own dtype: converting here would change the avals the
    # compiled step sees on the next call.
    u_safe_prev = (state.u_safe_prev if u_safe is None
                   else np.asarray(u_safe))
    if h_vals is None or cbf_grad is None:
        return state._replace(
            u_safe_prev=u_safe_prev,
            cbf_h_prev=None,
            cbf_grad_prev=None,
            cbf_h_delta_norm=float('nan'),
            cbf_grad_delta_norm=float('nan'),
        )
    h_now = np.asarray(h_vals)
    grad_now = np.asarray(cbf_grad)
    return CrossStepState(
        u_safe_prev=u_safe_prev,
        cbf_h_prev=h_now.copy(),
        cbf_grad_prev=grad_now.copy(),
        cbf_h_delta_norm=(
            0.0 if state.cbf_h_prev is None
            else float(np.linalg.norm(h_now - state.cbf_h_prev))),
        cbf_grad_delta_norm=(
            0.0 if state.cbf_grad_prev is None
            else float(np.linalg.norm(grad_now - state.cbf_grad_prev))),
    )
