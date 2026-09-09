"""Pure-Python Livox MID-360 / MID-360S driver.

Adapted from the user-supplied ``tmbs-main`` archive; the full reuse and
adaptation record is kept in the robot_safecontrol planning docs
(``docs/planning/oscbf-reuse/research/tmbs-mid360-python-driver-reuse.md``).

Layers
------
``protocol``   Livox Ethernet protocol: control frames, CRC, point cloud decode.
``device``     Per-device command channel (SN, IP, FOV, work mode, reboot).
``discovery``  Broadcast discovery plus unicast SN probe.
``stream``     Continuous UDP reception and frame assembly.
``ros_node``   ROS 2 adapter publishing the upstream-compatible PointCloud2.
``cli``        ``xy-mid-360-s`` bring-up and diagnostics.

The official C++ driver (``livox_ros_driver2`` + ``Livox-SDK2``) stays the
primary path per ticket #31; this package covers device control and gives a
Python fallback that needs neither the vendor SDK nor a colcon workspace.
"""

from xy_mid_360_s.device import Mid360Device
from xy_mid_360_s.discovery import discover_device, discover_subnet, query_sn
from xy_mid_360_s.protocol import (
    CloudPacket,
    build_ctrl_frame,
    parse_cloud_packet,
    parse_ctrl_frame,
)
from xy_mid_360_s.stream import (
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
