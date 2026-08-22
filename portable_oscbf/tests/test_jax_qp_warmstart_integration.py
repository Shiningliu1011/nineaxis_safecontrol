#!/usr/bin/env python3
"""Production JAX control must not use the experimental custom PDIP path."""

import pytest


def test_jax_tracking_rejects_the_archived_custom_warm_start_flag():
    from work.jax_control_facade import JaxControlLoop

    with pytest.raises(ValueError, match='archived'):
        JaxControlLoop(dt=0.002, temporal_lambda=0.2, qp_warm_start=True)
