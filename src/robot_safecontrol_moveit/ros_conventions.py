"""Shared ROS transport conventions (QoS profiles and topic names).

The plant, controller, and viewer exchange joint state on one stream and
safe commands on another; keeping the names and QoS in one place prevents
publisher/subscriber mismatches across nodes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.qos import QoSProfile

JOINT_STATE_TOPIC = "/mujoco_joint_states"
OSCBF_COMMAND_TOPIC = "/oscbf_command"
PERCEPTION_TRACKS_TOPIC = "/perception/tracks"
STATE_STREAM_QOS_DEPTH = 20
COMMAND_STREAM_QOS_DEPTH = 5


def _stream_qos(depth: int) -> QoSProfile:
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def state_stream_qos() -> QoSProfile:
    """关节反馈状态流使用的 QoS。"""
    return _stream_qos(STATE_STREAM_QOS_DEPTH)


def command_stream_qos() -> QoSProfile:
    """关节位置指令流使用的 QoS。"""
    return _stream_qos(COMMAND_STREAM_QOS_DEPTH)
