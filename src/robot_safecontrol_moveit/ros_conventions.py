"""Shared ROS transport conventions (QoS profiles and topic names).

The plant, controller, and viewer exchange joint state on one stream and
safe commands on another; keeping the names and QoS in one place prevents
publisher/subscriber mismatches across nodes.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

JOINT_STATE_TOPIC = "/mujoco_joint_states"
OSCBF_COMMAND_TOPIC = "/oscbf_command"


def state_stream_qos() -> QoSProfile:
    """BEST_EFFORT with depth 20 for the 100 Hz joint-state stream.

    ``qos_profile_sensor_data`` (depth 5) overflows under plant bursts, which
    showed up as long-run p95 latency > 10 ms.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
