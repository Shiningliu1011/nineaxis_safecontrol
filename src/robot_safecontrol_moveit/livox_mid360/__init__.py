"""Pure-Python Livox MID-360 / MID-360S support for robot_safecontrol.

Adapted from the user-supplied ``tmbs-main`` archive (see
``docs/planning/oscbf-reuse/research/tmbs-mid360-python-driver-reuse.md`` for the
full reuse/adaptation record).

Layers
------
``protocol``   Livox Ethernet protocol: control frames, CRC, point cloud decode.
``device``     Per-device command channel (SN, IP, FOV, work mode, reboot).
``discovery``  Broadcast discovery plus unicast SN probe.
``stream``     Continuous UDP reception and frame assembly.
``ros_node``   ROS 2 adapter publishing the upstream-compatible PointCloud2.
``cli``        ``livox_mid360_tool`` bring-up and diagnostics.

The official C++ driver (``livox_ros_driver2`` + ``Livox-SDK2``) stays the
primary path per ticket #31; this package covers device control and gives a
Python fallback that needs neither the vendor SDK nor a colcon workspace.
"""

from robot_safecontrol_moveit.livox_mid360.device import Mid360Device
from robot_safecontrol_moveit.livox_mid360.discovery import discover_device, discover_subnet, query_sn
from robot_safecontrol_moveit.livox_mid360.protocol import (
    CloudPacket,
    build_ctrl_frame,
    parse_cloud_packet,
    parse_ctrl_frame,
)
from robot_safecontrol_moveit.livox_mid360.stream import (
    CloudFrame,
    CloudReceiver,
    FrameAssembler,
    StateMonitor,
)

__all__ = [
    "CloudFrame",
    "CloudPacket",
    "CloudReceiver",
    "FrameAssembler",
    "Mid360Device",
    "StateMonitor",
    "build_ctrl_frame",
    "discover_device",
    "discover_subnet",
    "parse_cloud_packet",
    "parse_ctrl_frame",
    "query_sn",
]
