"""Per-device MID-360 control channel (UDP port 56100).

Adapted from the user-supplied ``tmbs-main`` archive
(``backend/app/drivers/mid360_driver.py``, class ``Mid360Device``).  The command
semantics, retry/sequence handling and read-modify-write FOV behaviour are the
source driver's; the platform coupling (``app.platform.logging_runtime``,
device-id registry, status callbacks) is removed so this class can be used from
the ROS package, a CLI, or tests without the tmbs backend.

Scope: device control only.  Point cloud reception lives in
:mod:`xy_mid_360_s.stream`.

.. warning::

   ``set_work_mode``/``start_sampling``/``enter_ready``/``set_lidar_ip``/``reboot``
   change device state.  The upstream Livox ROS driver also configures the
   device on start; here that is always an explicit, opt-in action, never a
   side effect of merely subscribing to a topic.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Any, Dict, Optional

from xy_mid_360_s.protocol import (
    CMD_PARAM_CONFIG,
    CMD_PARAM_INQUIRE,
    CMD_REBOOT,
    CMD_TYPE_ACK,
    KEY_FOV_CFG0,
    KEY_FOV_CFG1,
    KEY_FOV_CFG_EN,
    KEY_IMU_HOST_CFG,
    KEY_LIDAR_IP,
    KEY_LIDAR_TEMP,
    KEY_PATTERN_MODE,
    KEY_PCL_DATA_TYPE,
    KEY_PCL_HOST_CFG,
    KEY_SN,
    KEY_STATE_HOST_CFG,
    KEY_WORK_MODE,
    KEY_WORK_STATE,
    PORT_CTRL,
    PORT_PUSH_IMU,
    PORT_PUSH_PCL,
    PORT_PUSH_STATE,
    PORT_SRC_IMU,
    PORT_SRC_PCL,
    PORT_SRC_STATE,
    WORK_READY,
    WORK_SAMPLING,
    WORK_STANDBY,
    WORK_STATE_NAMES,
    build_ctrl_frame,
    build_param_config,
    build_param_inquire,
    ip_config_value,
    parse_ctrl_frame,
    parse_key_value_list,
)

logger = logging.getLogger(__name__)

CMD_LOCK_ACQUIRE_TIMEOUT_SEC = 2.0


class Mid360Device:
    """Command channel to one MID-360/MID-360S device."""

    def __init__(
        self,
        ip: str,
        host_ip: str,
        *,
        ctrl_port: int = PORT_CTRL,
        local_ctrl_port: int = 0,
        pcl_host_port: int = PORT_PUSH_PCL,
        state_host_port: int = PORT_PUSH_STATE,
        imu_host_port: int = PORT_PUSH_IMU,
        cmd_timeout: float = 2.0,
        cmd_retries: int = 2,
        cmd_lock_timeout: float = CMD_LOCK_ACQUIRE_TIMEOUT_SEC,
    ) -> None:
        self.ip = ip
        self.host_ip = host_ip
        self.ctrl_port = ctrl_port
        self.pcl_host_port = pcl_host_port
        self.state_host_port = state_host_port
        self.imu_host_port = imu_host_port
        self.cmd_timeout = cmd_timeout
        self.cmd_retries = cmd_retries
        self._cmd_lock_timeout = cmd_lock_timeout

        self.status = "offline"
        self.sn = ""
        self.temperature: Optional[float] = None

        self._seq = 0
        self._seq_lock = threading.Lock()
        self._cmd_lock = threading.Lock()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(cmd_timeout)
        # A fixed local control port matches the vendor port convention
        # (56101...) but is optional: the device answers the request source
        # port, so an ephemeral port works too and avoids clashing with the
        # official driver's host ports when both are running.
        if local_ctrl_port:
            try:
                self._sock.bind(("", local_ctrl_port))
            except OSError as exc:
                logger.warning("%s: cannot bind local control port %d (%s); "
                               "falling back to an ephemeral port",
                               self.ip, local_ctrl_port, exc)

    # ── command plumbing ─────────────────────────────────────────────────────

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            return self._seq

    def send_command(self, cmd_id: int, data: bytes = b"",
                     *, retries: Optional[int] = None,
                     timeout: Optional[float] = None) -> Optional[bytes]:
        """Send one command and return the matching ACK data field.

        The reply must match ``cmd_id``, be an ACK and echo the request
        ``seq_num``; anything else is ignored.  Returns ``None`` on timeout or
        when the command lock cannot be acquired in time.
        """
        retries = self.cmd_retries if retries is None else retries
        timeout = self.cmd_timeout if timeout is None else timeout
        if not self._cmd_lock.acquire(timeout=self._cmd_lock_timeout):
            logger.debug("[%s] cmd 0x%04x: command lock acquire timeout", self.ip, cmd_id)
            return None
        try:
            for attempt in range(retries):
                seq = self._next_seq()
                frame = build_ctrl_frame(cmd_id, data, seq=seq)
                self._drain_socket()
                self._sock.settimeout(timeout)
                try:
                    self._sock.sendto(frame, (self.ip, self.ctrl_port))
                    deadline = time.time() + timeout
                    while time.time() < deadline:
                        resp_raw, addr = self._sock.recvfrom(4096)
                        if addr[0] != self.ip:
                            continue
                        resp = parse_ctrl_frame(resp_raw)
                        if resp is None:
                            continue
                        if (resp["cmd_id"] == cmd_id
                                and resp["cmd_type"] == CMD_TYPE_ACK
                                and resp["seq_num"] == seq):
                            return resp["data"]
                except socket.timeout:
                    logger.debug("[%s] cmd 0x%04x attempt %d timeout",
                                 self.ip, cmd_id, attempt + 1)
            return None
        finally:
            self._cmd_lock.release()

    def _drain_socket(self) -> None:
        """Drop stale datagrams so a previous reply is never matched."""
        self._sock.setblocking(False)
        try:
            while True:
                self._sock.recvfrom(4096)
        except OSError:
            pass
        finally:
            self._sock.setblocking(True)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            logger.debug("[%s] control socket close failed", self.ip, exc_info=True)

    # ── configuration / queries ──────────────────────────────────────────────

    def configure_push_destinations(self, *, timeout: Optional[float] = None,
                                    retries: Optional[int] = None) -> bool:
        """Point cloud/state/IMU push destinations → this host, Cartesian high format."""
        kv = [
            (KEY_PCL_DATA_TYPE, struct.pack("<B", 0x01)),
            (KEY_PCL_HOST_CFG, ip_config_value(self.host_ip, self.pcl_host_port,
                                               PORT_SRC_PCL)),
            (KEY_STATE_HOST_CFG, ip_config_value(self.host_ip, self.state_host_port,
                                                 PORT_SRC_STATE)),
            (KEY_IMU_HOST_CFG, ip_config_value(self.host_ip, self.imu_host_port,
                                               PORT_SRC_IMU)),
        ]
        resp = self.send_command(CMD_PARAM_CONFIG, build_param_config(kv),
                                 timeout=timeout, retries=retries)
        if resp and resp[0] == 0:
            logger.info("[%s] push destinations configured → %s pcl:%d state:%d imu:%d",
                        self.ip, self.host_ip, self.pcl_host_port,
                        self.state_host_port, self.imu_host_port)
            return True
        logger.warning("[%s] push destination configuration failed: resp=%s",
                       self.ip, _hex(resp))
        return False

    def query_sn(self, *, timeout: Optional[float] = None,
                 retries: Optional[int] = None) -> str:
        """Read the hardware serial number; returns ``""`` on failure."""
        resp = self.send_command(CMD_PARAM_INQUIRE, build_param_inquire([KEY_SN]),
                                 timeout=timeout, retries=retries)
        if resp:
            raw = parse_key_value_list(resp).get(KEY_SN)
            if raw:
                self.sn = raw.split(b"\x00")[0].decode("ascii", errors="replace").strip()
                return self.sn
        return ""

    def query_state(self, *, timeout: Optional[float] = None,
                    retries: Optional[int] = None) -> Optional[str]:
        """Read ``KEY_WORK_STATE``; updates and returns the status string."""
        resp = self.send_command(CMD_PARAM_INQUIRE,
                                 build_param_inquire([KEY_WORK_STATE]),
                                 timeout=timeout, retries=retries)
        raw = parse_key_value_list(resp).get(KEY_WORK_STATE) if resp else None
        if raw:
            work_state = raw[0]
            self.status = WORK_STATE_NAMES.get(
                work_state, f"unknown(0x{work_state:02x})")
            return self.status
        return None

    def query_temperature(self) -> Optional[float]:
        """Read the core temperature in °C (device reports 0.01 °C units)."""
        resp = self.send_command(CMD_PARAM_INQUIRE,
                                 build_param_inquire([KEY_LIDAR_TEMP]))
        raw = parse_key_value_list(resp).get(KEY_LIDAR_TEMP) if resp else None
        if raw and len(raw) >= 4:
            self.temperature = round(struct.unpack_from("<i", raw)[0] / 100.0, 1)
            return self.temperature
        return None

    def set_work_mode(self, mode: int, *, timeout: Optional[float] = None,
                      retries: Optional[int] = None) -> bool:
        """Write ``KEY_WORK_MODE`` (0x01 sampling / 0x02 standby / 0x09 ready)."""
        kv = [(KEY_WORK_MODE, struct.pack("<B", mode))]
        resp = self.send_command(CMD_PARAM_CONFIG, build_param_config(kv),
                                 timeout=timeout, retries=retries)
        if resp and resp[0] == 0:
            return True
        logger.warning("[%s] set work mode %d failed: resp=%s", self.ip, mode, _hex(resp))
        return False

    def start_sampling(self) -> bool:
        """Configure push destinations, then switch to SAMPLING."""
        if not self.configure_push_destinations():
            logger.warning("[%s] push configuration failed; attempting sampling anyway",
                           self.ip)
        return self.set_work_mode(WORK_SAMPLING)

    def enter_ready(self, *, timeout: Optional[float] = None,
                    retries: Optional[int] = None) -> bool:
        return self.set_work_mode(WORK_READY, timeout=timeout, retries=retries)

    def enter_standby(self, *, timeout: Optional[float] = None,
                      retries: Optional[int] = None) -> bool:
        return self.set_work_mode(WORK_STANDBY, timeout=timeout, retries=retries)

    # ── FOV (KEY_FOV_CFG0/1 + KEY_FOV_CFG_EN) ────────────────────────────────

    def query_fov_enable(self) -> int:
        """Current FOV enable bitmask (bit0 = profile 0, bit1 = profile 1)."""
        resp = self.send_command(CMD_PARAM_INQUIRE,
                                 build_param_inquire([KEY_FOV_CFG_EN]))
        raw = parse_key_value_list(resp).get(KEY_FOV_CFG_EN) if resp else None
        return raw[0] if raw else 0

    def set_fov(self, profile: int, yaw_start: float, yaw_stop: float,
                pitch_start: float, pitch_stop: float, enable: bool) -> bool:
        """Write one FOV profile; read-modify-write preserves the other profile.

        Angles are whole degrees: yaw in [0, 360), pitch in (-10, 60).
        """
        if profile not in (0, 1):
            raise ValueError(f"profile must be 0 or 1, got {profile}")
        key = KEY_FOV_CFG0 if profile == 0 else KEY_FOV_CFG1
        fov_value = struct.pack(
            "<iiiiI",
            int(round(yaw_start)), int(round(yaw_stop)),
            int(round(pitch_start)), int(round(pitch_stop)),
            0)  # reserved
        current = self.query_fov_enable()
        en_byte = (current | (1 << profile)) if enable else (current & ~(1 << profile) & 0xFF)
        kv = [(key, fov_value), (KEY_FOV_CFG_EN, struct.pack("<B", en_byte))]
        resp = self.send_command(CMD_PARAM_CONFIG, build_param_config(kv))
        if resp and resp[0] == 0:
            logger.info("[%s] FOV profile %d set: yaw=[%.0f,%.0f] pitch=[%.0f,%.0f] enable=%s",
                        self.ip, profile, yaw_start, yaw_stop,
                        pitch_start, pitch_stop, enable)
            return True
        logger.warning("[%s] FOV profile %d write failed: resp=%s", self.ip, profile, _hex(resp))
        return False

    def query_fov(self, profile: int) -> Optional[Dict[str, Any]]:
        """Read one FOV profile: yaw/pitch bounds in degrees plus enable flag."""
        if profile not in (0, 1):
            raise ValueError(f"profile must be 0 or 1, got {profile}")
        key = KEY_FOV_CFG0 if profile == 0 else KEY_FOV_CFG1
        resp = self.send_command(CMD_PARAM_INQUIRE, build_param_inquire([key, KEY_FOV_CFG_EN]))
        if not resp:
            return None
        values = parse_key_value_list(resp)
        raw = values.get(key)
        if raw is None or len(raw) < 16:
            logger.warning("[%s] FOV profile %d query returned no value", self.ip, profile)
            return None
        yaw_start, yaw_stop, pitch_start, pitch_stop = struct.unpack_from("<iiii", raw)
        en_raw = values.get(KEY_FOV_CFG_EN)
        enable = bool(en_raw[0] & (1 << profile)) if en_raw else False
        return {
            "yaw_start": float(yaw_start),
            "yaw_stop": float(yaw_stop),
            "pitch_start": float(pitch_start),
            "pitch_stop": float(pitch_stop),
            "enable": enable,
        }

    def set_fov_enable_byte(self, enable: int) -> bool:
        """Write the FOV enable bitmask directly (bit0/bit1 = profile 0/1)."""
        kv = [(KEY_FOV_CFG_EN, struct.pack("<B", enable & 0x03))]
        resp = self.send_command(CMD_PARAM_CONFIG, build_param_config(kv))
        if resp and resp[0] == 0:
            return True
        logger.warning("[%s] FOV enable byte write failed: resp=%s", self.ip, _hex(resp))
        return False

    # ── device IP ────────────────────────────────────────────────────────────

    def set_lidar_ip(self, ip: str, subnet: str = "255.255.255.0",
                     gateway: str = "0.0.0.0") -> bool:
        """Write the device IP configuration (takes effect after a reboot)."""
        try:
            value = socket.inet_aton(ip) + socket.inet_aton(subnet) \
                + socket.inet_aton(gateway)
        except OSError as exc:
            logger.warning("[%s] invalid IP configuration: %s", self.ip, exc)
            return False
        kv = [(KEY_LIDAR_IP, value)]
        resp = self.send_command(CMD_PARAM_CONFIG, build_param_config(kv))
        if resp and resp[0] == 0:
            logger.info("[%s] device IP set to %s/%s gw=%s (reboot to apply)",
                        self.ip, ip, subnet, gateway)
            return True
        logger.warning("[%s] device IP write failed: resp=%s", self.ip, _hex(resp))
        return False

    def query_lidar_ip(self) -> Optional[Dict[str, str]]:
        resp = self.send_command(CMD_PARAM_INQUIRE, build_param_inquire([KEY_LIDAR_IP]))
        raw = parse_key_value_list(resp).get(KEY_LIDAR_IP) if resp else None
        if raw and len(raw) >= 12:
            try:
                return {
                    "ip": socket.inet_ntoa(raw[0:4]),
                    "subnet": socket.inet_ntoa(raw[4:8]),
                    "gateway": socket.inet_ntoa(raw[8:12]),
                }
            except OSError:
                logger.warning("[%s] malformed IP configuration response", self.ip)
        return None

    # ── scan pattern / reboot ────────────────────────────────────────────────

    def set_pattern_mode(self, mode: int) -> bool:
        """0 = non-repetitive, 1 = repetitive, 2 = low-frame-rate repetitive."""
        if mode not in (0, 1, 2):
            logger.warning("[%s] invalid pattern mode %d", self.ip, mode)
            return False
        kv = [(KEY_PATTERN_MODE, struct.pack("<B", mode))]
        resp = self.send_command(CMD_PARAM_CONFIG, build_param_config(kv))
        if resp and resp[0] == 0:
            return True
        logger.warning("[%s] pattern mode write failed: resp=%s", self.ip, _hex(resp))
        return False

    def query_pattern_mode(self) -> Optional[int]:
        resp = self.send_command(CMD_PARAM_INQUIRE,
                                 build_param_inquire([KEY_PATTERN_MODE]))
        raw = parse_key_value_list(resp).get(KEY_PATTERN_MODE) if resp else None
        return raw[0] if raw else None

    def reboot(self, delay_ms: int = 2000) -> bool:
        """Request a device reboot after ``delay_ms`` milliseconds."""
        resp = self.send_command(CMD_REBOOT, struct.pack("<H", delay_ms))
        if resp and resp[0] == 0:
            logger.info("[%s] reboot accepted, delay=%d ms", self.ip, delay_ms)
            return True
        logger.warning("[%s] reboot request failed: resp=%s", self.ip, _hex(resp))
        return False


def _hex(data: Optional[bytes]) -> Optional[str]:
    return data.hex() if data is not None else None
