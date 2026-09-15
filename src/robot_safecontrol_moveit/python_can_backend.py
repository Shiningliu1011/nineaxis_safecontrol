"""Thin, synchronous python-can adapter; no interface setup or automatic enable.

One owner must serialize transactions. Received timestamps remain the original
transport wall-clock timestamps; callers must validate them before using them
as feedback freshness.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class ReceivedFrame:
    frame_id: int
    data: bytes
    timestamp_s: float

    def __iter__(self):
        """Keep the legacy ``frame_id, data = recv(...)`` unpacking seam."""
        yield self.frame_id
        yield self.data

    def __eq__(self, other):
        if isinstance(other, ReceivedFrame):
            return (
                self.frame_id == other.frame_id
                and self.data == other.data
                and self.timestamp_s == other.timestamp_s
            )
        if isinstance(other, tuple) and len(other) == 2:
            return (self.frame_id, self.data) == other
        return NotImplemented


class PythonCANBackend:
    """SocketCAN in production; explicit virtual interface for offline tests."""

    def __init__(self, channel: str, *, interface: str = "socketcan",
                 send_timeout_s: float = 0.01):
        if interface not in ("socketcan", "virtual") or not channel:
            raise ValueError("explicit socketcan/virtual channel required")
        if not math.isfinite(send_timeout_s) or send_timeout_s <= 0:
            raise ValueError("send timeout must be finite and positive")
        try:
            import can
        except ImportError as exc:
            raise RuntimeError("Install hardware extra: python-can==4.6.1") from exc
        self._can = can
        self._timeout = send_timeout_s
        self._closed = False
        self.last_error = ""
        # ignore_config prevents ~/.can or CAN_* environment changing the bus.
        try:
            self._bus = can.Bus(channel=channel, interface=interface,
                                ignore_config=True, receive_own_messages=False)
        except (can.CanError, OSError) as exc:
            raise RuntimeError(f"Cannot open {interface} channel {channel}: {exc}") from exc

    def send(self, frame_id: int, data: bytes) -> bool:
        if self._closed:
            self.last_error = "backend closed"
            return False
        if not 0 <= frame_id <= 0x7FF or len(data) != 8:
            raise ValueError("DrEmpower requires an 11-bit, 8-byte classic CAN frame")
        try:
            self._bus.send(self._can.Message(arbitration_id=frame_id, data=data,
                                            is_extended_id=False, check=True),
                           timeout=self._timeout)
            return True
        except (self._can.CanError, OSError) as exc:
            self.last_error = str(exc)
            return False

    def receive(self, timeout_s: float = 0.01) -> ReceivedFrame | None:
        if self._closed:
            raise RuntimeError("backend closed")
        if not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("receive timeout must be finite and nonnegative")
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                msg = self._bus.recv(max(0.0, deadline - time.monotonic()))
            except (self._can.CanError, OSError) as exc:
                self.last_error = str(exc)
                raise RuntimeError(f"CAN receive failed: {exc}") from exc
            if msg is None:
                return None
            if msg.is_error_frame:
                self.last_error = "CAN error frame"
                raise RuntimeError(self.last_error)
            if (not msg.is_extended_id and not msg.is_remote_frame and not msg.is_fd
                    and msg.is_rx and msg.dlc == 8 and len(msg.data) == 8):
                return ReceivedFrame(msg.arbitration_id, bytes(msg.data), msg.timestamp)
            if time.monotonic() >= deadline:
                return None

    def recv(self, node_id: int, timeout_s: float = 0.01) -> ReceivedFrame | None:
        """Return the node frame without discarding its transport timestamp."""
        if not 1 <= node_id <= 63 or not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("invalid node or timeout")
        deadline = time.monotonic() + timeout_s
        while True:
            frame = self.receive(max(0.0, deadline - time.monotonic()))
            if frame is None:
                return None
            if frame.frame_id >> 5 == node_id:
                return frame
            if time.monotonic() >= deadline:
                return None

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
