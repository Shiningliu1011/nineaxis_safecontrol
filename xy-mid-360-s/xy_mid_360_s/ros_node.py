"""ROS 2 node: MID-360/MID-360S → ``sensor_msgs/PointCloud2`` on ``/livox/lidar``.

Adapted from the user-supplied ``tmbs-main`` archive; the
protocol/decoding comes from :mod:`~xy_mid_360_s.protocol`
and :mod:`~xy_mid_360_s.stream`, and this module is the
project-specific ROS adapter.

Why this exists next to the official ``livox_ros_driver2`` (ticket #31)
-----------------------------------------------------------------------
The official C++ driver remains the primary path.  This node covers the gaps it
does not: it is pure Python (no SDK build, no colcon workspace), it exposes the
device control channel (SN/IP/FOV/work mode/reboot via the CLI), and it can
publish the *same* PointCloud2 layout so ``perception_bridge`` needs no change:

====================  ========  ======  ==================================
field                 datatype  offset  meaning
====================  ========  ======  ==================================
x, y, z               FLOAT32   0/4/8   metres, sensor-local frame
intensity             FLOAT32   12      reflectivity (0..255 as float)
tag                   UINT8     16      device tag, bits 0..1 = confidence
line                  UINT8     17      index % 4 (MID-360 family)
timestamp             FLOAT64   18      absolute point time in ns
====================  ========  ======  ==================================

``point_step`` is 26 and ``header.stamp`` is the first point's time, exactly as
``livox_ros_driver2`` 1.2.6 ``Lddc::InitPointcloud2Msg`` produces it.  The time
base follows the upstream rule: device time when the packet reports PTP/GPS
sync, otherwise host receive time (``GetEthPacketTimestamp``).  Which clock the
device actually reports is a ticket #7 (perception time model) question — this
node does not claim hardware synchronisation.

Usage::

    xy-mid-360-s-node
    # or, without installing:
    PYTHONPATH=. python3 -m xy_mid_360_s.ros_node
    # parameters, e.g. an intentional bring-up:
    xy-mid-360-s-node --ros-args \
        -p host_ip:=192.168.1.5 -p lidar_ips:="[192.168.1.115]" -p configure_on_start:=true

``configure_on_start`` is **false** by default: merely running the node must not
change device state.  Enable it only for an intentional bring-up, or configure
the device once with ``xy-mid-360-s mode set sampling --ip <addr>``.
"""

from __future__ import annotations

import array
import logging
from typing import List

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from xy_mid_360_s.device import Mid360Device
from xy_mid_360_s.protocol import (
    MAX_FRAME_POINTS,
    PORT_PUSH_PCL,
    PORT_PUSH_STATE,
    WORK_SAMPLING,
)
from xy_mid_360_s.stream import CloudFrame, CloudReceiver, StateMonitor

logger = logging.getLogger(__name__)

# Packed layout, matching LivoxPointXyzrtlt under #pragma pack(1).
_POINT_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("intensity", "<f4"),
    ("tag", "u1"), ("line", "u1"),
    ("timestamp", "<f8"),
])
assert _POINT_DTYPE.itemsize == 26, _POINT_DTYPE.itemsize

_POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]


def frame_to_pointcloud2(frame: CloudFrame, *, frame_id: str,
                         stamp_ns: int) -> PointCloud2:
    """Build a PointCloud2 in the upstream driver's layout (see module docstring)."""
    n = frame.size
    payload = np.empty(n, dtype=_POINT_DTYPE)
    payload["x"] = frame.points[:, 0]
    payload["y"] = frame.points[:, 1]
    payload["z"] = frame.points[:, 2]
    payload["intensity"] = frame.points[:, 3]
    payload["tag"] = frame.tag
    payload["line"] = frame.line
    payload["timestamp"] = frame.timestamp_ns

    msg = PointCloud2()
    msg.header.stamp = rclpy.time.Time(nanoseconds=int(stamp_ns)).to_msg()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.fields = _POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_DTYPE.itemsize
    msg.row_step = msg.point_step * n
    msg.is_dense = True
    # Assign an array.array, not bytes: rclpy's generated setter converts a
    # bytes payload element by element (~33 ms for a 20k-point frame, measured
    # on hardware), which stalls the receive thread long enough to overflow the
    # kernel UDP buffer.  array.array copies the buffer in C (~0.02 ms).
    msg.data = array.array("B", payload.tobytes())
    return msg


class LivoxMid360Node(Node):
    """Bridges the Python MID-360 UDP path onto ROS topics."""

    def __init__(self) -> None:
        super().__init__("xy_mid_360_s_node")
        self.declare_parameter("topic", "/livox/lidar")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("host_ip", "")
        self.declare_parameter(
            "lidar_ips", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_parameter("pcl_port", PORT_PUSH_PCL)
        self.declare_parameter("state_port", PORT_PUSH_STATE)
        self.declare_parameter("publish_freq", 10.0)
        self.declare_parameter("keep_all_points", True)
        self.declare_parameter("configure_on_start", False)
        self.declare_parameter("qos_reliable", False)

        self._frame_id = str(self.get_parameter("frame_id").value)
        host_ip = str(self.get_parameter("host_ip").value)
        lidar_ips = [ip for ip in self.get_parameter("lidar_ips").value if ip]
        publish_freq = float(self.get_parameter("publish_freq").value)
        keep_all_points = bool(self.get_parameter("keep_all_points").value)
        self._configure_on_start = bool(self.get_parameter("configure_on_start").value)
        reliable = bool(self.get_parameter("qos_reliable").value)

        if publish_freq <= 0.0:
            raise ValueError(f"publish_freq must be positive, got {publish_freq}")
        # The MID-360/MID-360S streams 200,000 points/s, so a frame longer than
        # MAX_FRAME_POINTS/200,000 s cannot stay inside the RMW's 512 KiB
        # shared-memory limit; the cap, not publish_freq, then sets the rate.
        if publish_freq < 10.0:
            self.get_logger().warning(
                f"publish_freq={publish_freq:.1f} Hz needs ~{200_000 / publish_freq:.0f} "
                f"points per frame, above the {MAX_FRAME_POINTS}-point cap imposed by "
                f"the RMW's 512 KiB shared-memory limit; frames will flush at the cap "
                f"(~10 Hz) instead")

        if reliable:
            qos = QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
        else:
            qos = QoSProfile(
                depth=5,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
        self._pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("topic").value), qos)

        self._receiver = CloudReceiver(
            host_ip=host_ip,
            port=int(self.get_parameter("pcl_port").value),
            frame_interval_s=1.0 / publish_freq,
            keep_all_points=keep_all_points,
            on_frame=self._on_frame,
        )
        self._state_monitor = StateMonitor(
            host_ip=host_ip,
            port=int(self.get_parameter("state_port").value),
            on_state=self._on_state,
        )
        self._devices: List[Mid360Device] = []
        self._published_frames = 0
        self._published_points = 0

        try:
            self._receiver.start()
            self._state_monitor.start()
        except OSError as exc:
            raise RuntimeError(
                f"cannot bind LiDAR UDP ports (pcl={self._receiver.port}, "
                f"state={self._state_monitor.port}): {exc}") from exc

        if self._configure_on_start and lidar_ips:
            self._configure_devices(lidar_ips, host_ip)
        elif self._configure_on_start:
            self.get_logger().warning(
                "configure_on_start=true but lidar_ips is empty — nothing configured")
        else:
            self.get_logger().info(
                "listen-only mode: run 'xy-mid-360-s mode set sampling --ip <addr>' "
                "to start the device pushing to this host")

        self.get_logger().info(
            f"publishing {self.get_parameter('topic').value} as {self._frame_id} "
            f"at {publish_freq:.1f} Hz (pcl port {self._receiver.port})")

    # ── callbacks ────────────────────────────────────────────────────────────

    def _configure_devices(self, lidar_ips: List[str], host_ip: str) -> None:
        for ip in lidar_ips:
            device = Mid360Device(ip, host_ip)
            self._devices.append(device)
            self.get_logger().info(f"configuring device {ip} for push to {host_ip}")
            device.configure_push_destinations()
            if not device.set_work_mode(WORK_SAMPLING):
                self.get_logger().warning(f"device {ip}: could not enter sampling mode")

    def _on_frame(self, frame: CloudFrame) -> None:
        msg = frame_to_pointcloud2(
            frame, frame_id=self._frame_id, stamp_ns=frame.base_time_ns)
        self._pub.publish(msg)
        self._published_frames += 1
        self._published_points += frame.size
        if self._published_frames % 100 == 0:
            self.get_logger().debug(
                f"published {self._published_frames} frames, "
                f"{self._published_points} points")

    def _on_state(self, src_ip: str, state: dict) -> None:
        self.get_logger().info(
            f"device {src_ip} state: {state.get('status')} "
            f"(temp {state.get('temperature_c')} °C)")

    # ── lifecycle ────────────────────────────────────────────────────────────

    def destroy_node(self) -> bool:
        self._receiver.stop()
        self._state_monitor.stop()
        for device in self._devices:
            device.close()
        self._devices.clear()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxMid360Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
