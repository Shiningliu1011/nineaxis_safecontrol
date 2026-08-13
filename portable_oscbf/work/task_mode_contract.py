"""Public task-mode contract shared by the runner and JAX controller.

The names in this module are intentionally independent of ROS and JAX.  They
make an unsupported controller/task combination fail at startup instead of
silently using the legacy 6-D orientation controller.
"""

from __future__ import annotations


TASK_MODE_POSE_6D = 'pose6d'
TASK_MODE_TOOL_AXIS_5D = 'tool_axis_5d'
SUPPORTED_TASK_MODES = (TASK_MODE_POSE_6D, TASK_MODE_TOOL_AXIS_5D)


def validate_task_mode_configuration(task_mode: str, *, qp_backend: str,
                                     control_mode: str) -> str:
    """Validate a task mode before a ROS runner starts moving the simulator.

    ``tool_axis_5d`` has only been implemented and verified in the fixed-shape
    JAX velocity kernel.  The NumPy/OSQP and torque paths still implement the
    legacy 6-D pose task, so accepting the flag there would falsely advertise
    roll freedom.
    """
    mode = str(task_mode)
    if mode not in SUPPORTED_TASK_MODES:
        raise ValueError(
            f'task_mode must be one of {SUPPORTED_TASK_MODES}, got {mode!r}')
    if mode == TASK_MODE_TOOL_AXIS_5D and str(qp_backend) != 'cbfpy':
        raise ValueError(
            'task_mode=tool_axis_5d currently requires qp_backend=cbfpy; '
            'the NumPy/OSQP and qpax runner paths remain 6-D pose controllers')
    if mode == TASK_MODE_TOOL_AXIS_5D and str(control_mode) != 'velocity':
        raise ValueError(
            'task_mode=tool_axis_5d currently requires control_mode=velocity; '
            'the torque path remains a 6-D pose controller')
    return mode
