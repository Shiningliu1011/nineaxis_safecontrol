"""Cold-process precision and endpoint regression, without solver/hardware I/O."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize("x64", [True, False])
def test_configured_precision_precedes_path_conversion_in_a_fresh_process(x64):
    root = Path(__file__).resolve().parents[2]
    script = '''
import numpy as np
import jax
import jax.numpy as jnp
from work.jax_control_facade import JaxControlLoop
from work.path_following import PathGeometry, PathFollowingConfig
from work.jax_path_following import reconcile_path_state_after_motion_jax
from robot_safecontrol_moveit.tracking_contract import EvaluationScope, EvidenceContext
from robot_safecontrol_moveit.tracking_evaluator import TrackingEvaluator

# Start with the opposite mode, independent of test collection/global JAX state.
enabled = ENABLED
jax.config.update('jax_enable_x64', not enabled)
geometry = PathGeometry.from_samples(np.array([[0.,0.,0.],[1.3,0.,0.]]),
    np.repeat(np.eye(3)[None,:,:],2,axis=0),np.array([.1,.1]),np.array([0.,13.]))
loop = JaxControlLoop(enable_x64=enabled)
loop.configure_path(geometry, PathFollowingConfig())
arrays = loop.path_geometry_arrays()
expected = np.dtype('float64' if enabled else 'float32')
assert all(a.dtype == expected for a in arrays.values())
assert loop.q_min.dtype == expected
total = float(arrays['arc_length_m'][-1])
assert total == (1.3 if enabled else float(np.float32(1.3)))
arrays['arc_length_m'][-1] = 123.
assert float(loop.path_geometry_arrays()['arc_length_m'][-1]) == total

state = reconcile_path_state_after_motion_jax(loop._path_geometry,loop._path_config,
    jnp.array([total,total-.01,0.,0.,0.]),jnp.array([1.31,0.,0.]),dt_s=.01)
assert float(state[1]) == total
e = TrackingEvaluator(scope=EvaluationScope(total),evidence=EvidenceContext(
    run_id='precision-test',kind='model',boundary='kernel_candidate',model_id='line',
    config_id=str(enabled),trajectory_id='actual-kernel-arrays',data_id='analytic-endpoint',
    scenario='projection-only',measurement='point on model line',time_basis='test seconds'))
for index, progress in enumerate((0.,float(state[1]))):
    e.update(dict(projected_progress_m=progress,measured_cross_track_error_m=0.,
        tool_axis_error_rad=0.,qp_ok=True),wall_time_s=float(index))
e.finish('completed')
assert e.report().completed
assert e.report().task_verdict == 'pass'
'''.replace("ENABLED", repr(x64))
    env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(root / "src"), str(root / "portable_oscbf"))))
    result = subprocess.run([sys.executable, "-c", script], env=env,
                            capture_output=True, text=True, timeout=40)
    assert result.returncode == 0, result.stdout + result.stderr
