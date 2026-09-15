"""Named, fixed-shape output of one compiled path-tracking control step.

The compiled ``path_tracking_step`` kernel returns 44 values whose positional
order was the only thing identifying them.  This module gives that output one
named structure, shared by the kernel that produces it and the facade that
reads it, so adding a slot no longer means editing three files in lockstep.
The two measurement flags are the slots this round added.

Two properties are forced by the ``jax.jit`` boundary between producer and
consumer, and they are the reason this is a ``NamedTuple`` rather than a
dataclass:

* ``NamedTuple`` is a registered pytree, so the record crosses the jit
  boundary at zero cost and positional unpacking keeps working.  An
  unregistered frozen dataclass collapses to a single leaf and fails inside
  jit, and a registered one still cannot carry the strings that host-side
  processing attaches.
* ``None`` cannot mean "not measured" on this boundary: a ``None`` in a jit
  return is a compile-time-static empty subtree, not a per-step value.  Slots
  that may legitimately not apply carry a 0-d boolean measurement flag
  instead -- ``min_obs_dist_measured`` / ``min_esdf_dist_measured`` -- so the
  numeric sentinel the kernel substitutes is never mistaken for a
  measurement.  See ``work.qpax_warmstart.WarmStartState.valid`` for the same
  pattern applied to cross-step state.
"""

from __future__ import annotations

from typing import NamedTuple

import jax


class ControlStepRecord(NamedTuple):
    """Everything one compiled control step produced, by name.

    The names match ``JaxPathTrackingResult`` wherever the two overlap, so the
    facade reads a slot and stores it under the same name; only the host-side
    processing steps (constraint metrics, ``_``-prefixed internals) differ.
    Values inside jit are traced arrays; the facade converts them at the host
    boundary.
    """

    q_next: jax.Array
    u_safe: jax.Array
    u_candidate: jax.Array
    u_nom: jax.Array
    err_6d: jax.Array
    ee_pos: jax.Array
    ee_rot: jax.Array
    qp_ok: jax.Array
    # Distance slots carry their own measurement flag: the kernel substitutes
    # a finite sentinel when the geometry is disabled, and a sentinel is not a
    # measurement (docs/tracking_evaluation.md).
    min_obs_dist: jax.Array
    min_obs_dist_measured: jax.Array
    min_esdf_dist: jax.Array
    min_esdf_dist_measured: jax.Array
    rate_constraint_violation: jax.Array
    rate_solver_slack: jax.Array
    h_vals: jax.Array
    cbf_grad: jax.Array
    active_count: jax.Array
    primal_residual: jax.Array
    terminal_kkt_residual: jax.Array
    terminal_kkt_accepted: jax.Array
    dual_max: jax.Array
    qp_iterations: jax.Array
    delta_slack: jax.Array
    path_state: jax.Array
    reference_position_m: jax.Array
    reference_rotation: jax.Array
    reference_tangent: jax.Array
    reference_omega_per_m: jax.Array
    reference_source_time_s: jax.Array
    cross_track_error_m: jax.Array
    gamma: jax.Array
    feedrate_nominal_m_s: jax.Array
    feedrate_m_s: jax.Array
    feedrate_joint_limit_m_s: jax.Array
    feedrate_cbf_limit_m_s: jax.Array
    feedrate_rate_limit_m_s: jax.Array
    feedrate_tool_axis_limit_m_s: jax.Array
    feedrate_endpoint_brake_limit_m_s: jax.Array
    limiting_reason_code: jax.Array
    actual_tangent_speed_m_s: jax.Array
    reference_at_endpoint: jax.Array
    posture_reference: jax.Array
    # OFF-15 diagnostics only: unrelaxed rows at the solve state, keeping the
    # raw candidate even when qp_ok rejects it.
    constraint_residuals: jax.Array
    ee_pos_before: jax.Array
