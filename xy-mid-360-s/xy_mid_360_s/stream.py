"""Continuous MID-360 point cloud reception and frame assembly.

Adapted from the user-supplied ``tmbs-main`` archive
(``backend/app/drivers/mid360_driver.py``, ``_pcl_recv_loop``/``_state_recv_loop``
and ``_parse_pcl_packet``).

Adaptation summary
------------------
The source driver buffers packets into a *single-shot scan batch* driven by its
platform's task API (``create_task``/``start_batch``/``stop_current_batch``,
1e8-point buffer, batch callbacks).  This project needs a continuous ROS stream
instead, so the batch machinery is replaced by:

* :class:`FrameAssembler` — accumulates decoded packets and emits a frame every
  ``frame_interval_s`` of point time (default 0.1 s ≈ the upstream driver's
  ``publish_freq=10``), with the same per-point time base the upstream C++
  driver produces: ``offset_time = packet_time + index * point_interval``,
  frame base time = first point's ``offset_time``.
* :class:`CloudReceiver` — one UDP socket bound to the push port; all devices
  send to it and are told apart by source address, matching the vendor port
  convention (device source 56300, host destination 56301).
* :class:`StateMonitor` — decodes device-pushed state frames (source port
  56200) into a status string plus temperature, for health reporting.

Neither class changes device state; they only listen.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from xy_mid_360_s.protocol import (
    KEY_LIDAR_TEMP,
    KEY_WORK_STATE,
    MAX_FRAME_POINTS,
    PORT_PUSH_PCL,
    PORT_PUSH_STATE,
    WORK_STATE_NAMES,
    CloudPacket,
    parse_cloud_packet,
    parse_ctrl_frame,
    parse_key_value_list,
    packet_time_base_ns,
    point_interval_ns,
)

logger = logging.getLogger(__name__)

FrameCallback = Callable[["CloudFrame"], None]
StateCallback = Callable[[str, Dict[str, Any]], None]


@dataclass
class CloudFrame:
    """One assembled point cloud frame, ready to become a PointCloud2."""

    points: np.ndarray        # (N, 4) float32 [x, y, z, intensity], metres
    tag: np.ndarray           # (N,) uint8
    line: np.ndarray          # (N,) uint8
    timestamp_ns: np.ndarray  # (N,) float64, absolute per-point time
    base_time_ns: int         # first point's timestamp (header.stamp source)
    src_ip: str = ""

    @property
    def size(self) -> int:
        return int(self.points.shape[0])


class FrameAssembler:
    """Accumulate decoded packets into fixed-duration frames.

    Pure logic (no sockets), so it is unit-testable with synthetic packets.
    """

    def __init__(self, *, frame_interval_s: float = 0.1,
                 max_points: int = MAX_FRAME_POINTS) -> None:
        if frame_interval_s <= 0:
            raise ValueError("frame_interval_s must be positive")
        self.frame_interval_ns = int(frame_interval_s * 1e9)
        self.max_points = int(max_points)
        self._points: List[np.ndarray] = []
        self._tag: List[np.ndarray] = []
        self._line: List[np.ndarray] = []
        self._time: List[np.ndarray] = []
        self._base_time_ns: Optional[int] = None
        self._count = 0
        self._src_ip = ""

    @property
    def pending_points(self) -> int:
        """Points accumulated but not yet emitted as a frame."""
        return self._count

    def add_packet(self, packet: CloudPacket, *, host_now_ns: Optional[int] = None,
                   src_ip: str = "") -> List[CloudFrame]:
        """Decode one packet into the pending frame; returns completed frames."""
        if packet.empty:
            return []
        if src_ip:
            self._src_ip = src_ip
        now_ns = int(time.time_ns() if host_now_ns is None else host_now_ns)
        base = packet_time_base_ns(packet.time_type, packet.timestamp_ns, now_ns)
        step = point_interval_ns(packet.time_interval_raw, packet.dot_num)
        offsets = base + np.arange(packet.points.shape[0], dtype=np.int64) * step

        completed: List[CloudFrame] = []
        if self._base_time_ns is None:
            self._base_time_ns = int(offsets[0])
        elif offsets[0] < self._base_time_ns:
            # Device clock stepped backwards (reboot / time sync): close the
            # pending frame instead of waiting for a span that may never come.
            logger.warning("point time went backwards (%d < %d) — flushing frame",
                           offsets[0], self._base_time_ns)
            frame = self.flush(src_ip=src_ip)
            if frame is not None:
                completed.append(frame)
            self._base_time_ns = int(offsets[0])

        self._points.append(packet.points)
        self._tag.append(packet.tag)
        self._line.append(packet.line)
        self._time.append(offsets)
        self._count += packet.points.shape[0]

        span_ns = int(offsets[-1]) - self._base_time_ns
        if span_ns >= self.frame_interval_ns or self._count >= self.max_points:
            frame = self.flush(src_ip=src_ip)
            if frame is not None:
                completed.append(frame)
        return completed

    def flush(self, *, src_ip: str = "") -> Optional[CloudFrame]:
        """Emit the pending points as one frame (or ``None`` when empty)."""
        if self._count == 0:
            return None
        points = np.concatenate(self._points, axis=0) if len(self._points) > 1 else self._points[0]
        tag = np.concatenate(self._tag, axis=0) if len(self._tag) > 1 else self._tag[0]
        line = np.concatenate(self._line, axis=0) if len(self._line) > 1 else self._line[0]
        times = np.concatenate(self._time, axis=0) if len(self._time) > 1 else self._time[0]
        frame = CloudFrame(
            points=points,
            tag=tag,
            line=line,
            timestamp_ns=times.astype(np.float64),
            base_time_ns=int(times[0]),
            src_ip=src_ip or self._src_ip,
        )
        self._points.clear()
        self._tag.clear()
        self._line.clear()
        self._time.clear()
        self._base_time_ns = None
        self._count = 0
        return frame


class CloudReceiver:
    """UDP point cloud listener → :class:`FrameAssembler` → ``on_frame``.

    Listen-only: it never configures the device.  Use
    :class:`~xy_mid_360_s.device.Mid360Device` (or the
    CLI) to point the device's push destination at this socket.
    """

    def __init__(self, *, host_ip: str = "", port: int = PORT_PUSH_PCL,
                 frame_interval_s: float = 0.1, keep_all_points: bool = True,
                 on_frame: Optional[FrameCallback] = None,
                 recv_timeout: float = 0.5) -> None:
        self.host_ip = host_ip
        self.port = port
        self.keep_all_points = keep_all_points
        self.on_frame = on_frame
        self._recv_timeout = recv_timeout
        self._frame_interval_s = frame_interval_s
        self._assembler = FrameAssembler(frame_interval_s=frame_interval_s)
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self.stats: Dict[str, Any] = {
            "packets": 0, "points": 0, "dropped_packets": 0,
            "frames": 0, "last_packet_monotonic": None,
        }

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._running.is_set():
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self._recv_timeout)
        sock.bind((self.host_ip, self.port))
        self._sock = sock
        self.port = sock.getsockname()[1]
        self._running.set()
        self._thread = threading.Thread(
            target=self._recv_loop, name="livox_mid360_pcl", daemon=True)
        self._thread.start()
        logger.info("point cloud receiver listening on %s:%d",
                    self.host_ip or "0.0.0.0", self.port)

    def stop(self) -> None:
        self._running.clear()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._sock = None

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def last_packet_age_s(self) -> Optional[float]:
        with self._lock:
            last = self.stats["last_packet_monotonic"]
        return None if last is None else time.monotonic() - last

    # ── receive loop ─────────────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        while self._running.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                raw, addr = sock.recvfrom(65535)
            except socket.timeout:
                self._flush_if_stalled()
                continue
            except OSError:
                if self._running.is_set():
                    logger.warning("point cloud socket error", exc_info=True)
                break

            packet = parse_cloud_packet(raw, keep_all_points=self.keep_all_points)
            if packet is None:
                with self._lock:
                    self.stats["dropped_packets"] += 1
                continue
            if packet.empty:
                continue
            with self._lock:
                self.stats["packets"] += 1
                self.stats["points"] += int(packet.points.shape[0])
                self.stats["last_packet_monotonic"] = time.monotonic()

            for frame in self._assembler.add_packet(packet, src_ip=addr[0]):
                self._deliver(frame)

    def _flush_if_stalled(self) -> None:
        """Emit buffered points when the stream stops mid-frame.

        Without this, a device that stops pushing right after a few packets would
        leave those points stuck in the assembler until the next packet arrives —
        the health path would see stale data with no frame ever published.
        """
        if self._assembler.pending_points == 0:
            return
        with self._lock:
            last = self.stats["last_packet_monotonic"]
        if last is None or time.monotonic() - last >= self._frame_interval_s:
            frame = self._assembler.flush()
            if frame is not None:
                self._deliver(frame)

    def _deliver(self, frame: CloudFrame) -> None:
        with self._lock:
            self.stats["frames"] += 1
        if self.on_frame is not None:
            try:
                self.on_frame(frame)
            except Exception:
                logger.warning("on_frame callback failed", exc_info=True)


class StateMonitor:
    """Listen to device-pushed state frames (source port 56200).

    Emits ``on_state(src_ip, {"status": str, "temperature_c": float|None})``
    whenever the work state changes; always refreshes ``last_seen``.
    """

    def __init__(self, *, host_ip: str = "", port: int = PORT_PUSH_STATE,
                 on_state: Optional[StateCallback] = None,
                 recv_timeout: float = 0.5) -> None:
        self.host_ip = host_ip
        self.port = port
        self.on_state = on_state
        self._recv_timeout = recv_timeout
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self.last_seen: Dict[str, float] = {}
        self.status: Dict[str, str] = {}
        self.temperature_c: Dict[str, Optional[float]] = {}

    def start(self) -> None:
        if self._running.is_set():
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self._recv_timeout)
        sock.bind((self.host_ip, self.port))
        self._sock = sock
        self.port = sock.getsockname()[1]
        self._running.set()
        self._thread = threading.Thread(
            target=self._recv_loop, name="livox_mid360_state", daemon=True)
        self._thread.start()
        logger.info("state monitor listening on %s:%d",
                    self.host_ip or "0.0.0.0", self.port)

    def stop(self) -> None:
        self._running.clear()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._sock = None

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def _recv_loop(self) -> None:
        while self._running.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                raw, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._running.is_set():
                    logger.warning("state socket error", exc_info=True)
                break
            frame = parse_ctrl_frame(raw)
            if frame is None or frame["sender"] != 1 or frame["cmd_type"] != 0:
                continue  # only device-initiated pushes
            values = parse_key_value_list(frame["data"])
            src_ip = addr[0]
            status = None
            work_state_raw = values.get(KEY_WORK_STATE)
            if work_state_raw:
                status = WORK_STATE_NAMES.get(
                    work_state_raw[0], f"unknown(0x{work_state_raw[0]:02x})")
            temp_raw = values.get(KEY_LIDAR_TEMP)
            temperature = None
            if temp_raw and len(temp_raw) >= 4:
                temperature = round(struct.unpack_from("<i", temp_raw)[0] / 100.0, 1)

            with self._lock:
                self.last_seen[src_ip] = time.time()
                if temperature is not None:
                    self.temperature_c[src_ip] = temperature
                changed = status is not None and self.status.get(src_ip) != status
                if status is not None:
                    self.status[src_ip] = status
            if changed and self.on_state is not None:
                try:
                    self.on_state(src_ip, {"status": status, "temperature_c": temperature})
                except Exception:
                    logger.warning("on_state callback failed", exc_info=True)
