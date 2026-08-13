"""Compatibility facade for the modular fixed-shape JAX OSCBF controller.

New implementation modules:

* ``jax_control_facade``: public state, input normalisation and diagnostics;
* ``jax_kernel_factory``: compiled FK/OSC/CBF/QP/integration kernels;
* ``jax_barrier_terms``: pure fixed-shape geometry and CBF RHS operations.

Existing callers may continue importing ``JaxControlLoop`` and barrier helpers
from this module while the runtime code stays split by responsibility.
"""

from work.jax_barrier_terms import (
    MAX_JAX_OBSTACLES,
    aggregate_dynamic_obstacle_terms,
    apply_aggregated_dynamic_cbf_terms,
    apply_dynamic_obstacle_cbf_terms,
    apply_qp_health_gate,
    compute_obstacle_clearance,
    compute_obstacle_time_terms,
)
from work.jax_control_facade import JaxControlLoop


__all__ = [
    'JaxControlLoop',
    'MAX_JAX_OBSTACLES',
    'aggregate_dynamic_obstacle_terms',
    'apply_aggregated_dynamic_cbf_terms',
    'apply_dynamic_obstacle_cbf_terms',
    'apply_qp_health_gate',
    'compute_obstacle_clearance',
    'compute_obstacle_time_terms',
]
