#!/usr/bin/env python3
"""JAX QP solver tolerance must be explicit and preserve the safe default."""

import pytest


def test_jax_solver_tolerance_is_forwarded_to_cbf_config():
    pytest.importorskip("cbfpy")
    from work.jax_control_loop import JaxControlLoop
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    loop = JaxControlLoop(solver_tol=2.0e-3)
    config = NineaxisOSCBFVelocityConfig(
        NineaxisManipulatorJAX(), solver_tol=loop.solver_tol)

    assert loop.solver_tol == pytest.approx(2.0e-3)
    assert config.solver_tol == pytest.approx(2.0e-3)


def test_jax_solver_tolerance_must_be_positive():
    from work.jax_control_loop import JaxControlLoop

    with pytest.raises(ValueError, match="solver_tol"):
        JaxControlLoop(solver_tol=0.0)
