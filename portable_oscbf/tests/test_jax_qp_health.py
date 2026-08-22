#!/usr/bin/env python3
"""A persistent JAX QP failure should stop safely without log flooding."""

import numpy as np
import pytest

# This reference test file also exercises the ROS-side ``newaxis`` runtime
# (excluded by OSCBF_PORTING_GUIDE.md §4.7).  The portable QP-health contract
# is covered by M5/M7; skip the file until then and register it in the M0
# baseline.
pytestmark = pytest.mark.skip(
    reason="depends on newaxis (excluded by OSCBF_PORTING_GUIDE.md §4.7); "
           "portable QP-health coverage lands in M5/M7"
)


def test_jax_health_gate_never_integrates_an_unconverged_qp_candidate():
    import jax.numpy as jnp

    from work.jax_barrier_terms import apply_qp_health_gate

    q = jnp.array([0.2, -0.1])
    candidate = jnp.array([3.0, -4.0])
    q_next, applied = apply_qp_health_gate(
        q, candidate, jnp.asarray(False), dt=0.01,
        q_min=jnp.array([-1.0, -1.0]), q_max=jnp.array([1.0, 1.0]))

    np.testing.assert_allclose(q_next, q)
    np.testing.assert_allclose(applied, 0.0)


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


def test_cbfpy_qp_failure_logs_once_until_a_success_resets_it():
    node = object.__new__(OSCBFRvizNewaxis)
    logger = _Logger()
    node.qp_backend = "cbfpy"
    node._consecutive_qp_failures = 0
    node._qp_fail_limit = 5
    node.get_logger = lambda: logger

    assert np.allclose(node._check_qp_health(False, np.ones(N_JOINTS)), 0.0)
    assert np.allclose(node._check_qp_health(False, np.ones(N_JOINTS)), 0.0)
    assert len(logger.errors) == 1

    node._check_qp_health(True, np.ones(N_JOINTS))
    node._check_qp_health(False, np.ones(N_JOINTS))
    assert len(logger.errors) == 2
