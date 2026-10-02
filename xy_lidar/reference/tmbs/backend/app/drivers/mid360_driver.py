"""
Livox MID-360 LiDAR driver — 基于官方 Livox LiDAR Communication Protocol v1.4.x
https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html

官方协议端口说明
----------------
  56000   设备监听端口（仅支持广播发现命令 0x0000）
  56100   控制命令端口（双向，设备监听）
  56200   设备状态推送（device→host, 源端口 56200, 目标可配，默认 56201）
  56300   点云数据推送（device→host, 源端口 56300, 目标可配，默认 56301）
  56400   IMU 数据推送（device→host）

控制帧格式（24 字节头）
  Offset  Size  字段
  0       1     sof          = 0xAA
  1       1     version      = 0x00
  2       2     length       (整帧总字节数，含头和 data)
  4       4     seq_num
  8       2     cmd_id
  10      1     cmd_type     (0x00=REQ, 0x01=ACK)
  11      1     sender_type  (0x00=主机, 0x01=设备)
  12      6     resv
  18      2     crc16        CRC-16/CCITT-FALSE, 校验字节 0..17
  20      4     crc32        CRC-32, 校验 data 字段（无 data 时填 0）
  24      n     data

点云数据帧头（36 字节，无控制帧头）
  0       1     version
  1       2     length
  3       2     time_interval   (0.1µs)
  5       2     dot_num         (本包点数)
  7       2     udp_cnt
  9       1     frame_cnt
  10      1     data_type       (1=CartXYZ32)
  11      1     time_type
  12      12    reserved
  24      4     crc32
  28      8     timestamp       (ns)
  36      n     data

data_type=1 每点 14 字节: x(i32mm) y(i32mm) z(i32mm) refl(u8) tag(u8)
"""

import asyncio
import os
import socket
import struct
import threading
import time
import uuid
from typing import Dict, Any, List, Optional, Callable, Set

import numpy as np

from app.drivers.base import LidarDriver
from app.drivers.livox_tag_filter import DEFAULT_LIVOX_TAG_FILTER, livox_tag_keep_mask
from app.drivers.scan_admission import scan_admission
from app.platform.logging_runtime import get_platform_logger

logger = get_platform_logger(__name__, component="driver.mid360")


def _device_log_context(device_ids: List[int] | tuple[int, ...] | None) -> dict[str, int]:
    """Keep driver correlation scalar and allowlisted for multi-device tasks."""

    values = tuple(device_ids or ())
    if len(values) == 1:
        return {"deviceId": int(values[0])}
    if values:
        return {"count": len(values)}
    return {}

# ────────────────────────────────────────────────────────────
# 协议常量
# ────────────────────────────────────────────────────────────
SOF = 0xAA
PROTO_VER = 0x00                # 固定 0x00

CMD_TYPE_REQ = 0x00
CMD_TYPE_ACK = 0x01

SENDER_HOST  = 0x00
SENDER_LIDAR = 0x01

# 端口
PORT_BCAST_CMD   = 56000     # 仅广播发现
PORT_CTRL        = 56100     # 控制命令
PORT_PUSH_STATE  = 56201     # 状态推送目标 (默认)
PORT_PUSH_PCL    = 56301     # 点云推送目标 (默认)
PORT_PUSH_IMU    = 56401     # IMU 推送目标

# 命令 ID
CMD_DISCOVERY     = 0x0000
CMD_PARAM_CONFIG  = 0x0100
CMD_PARAM_INQUIRE = 0x0101
CMD_REBOOT        = 0x0200   # 请求设备重启，data=uint16 delay_ms

# 设备类型（广播发现 ACK data[1]，详见 Livox MID360 协议 v1.4.x §1.2.1.5.1.1）
DEV_TYPE_MID360   = 9
DEV_TYPE_MID360S  = 35
ACCEPTED_DEV_TYPES = {DEV_TYPE_MID360, DEV_TYPE_MID360S}
DEV_TYPE_NAMES = {DEV_TYPE_MID360: "MID-360", DEV_TYPE_MID360S: "MID-360S"}

# 参数 key
KEY_PCL_DATA_TYPE   = 0x0000   # 点云格式
KEY_WORK_MODE       = 0x001A   # 工作模式(可写): 0x01=SAMPLING, 0x02=STANDBY, 0x09=READY(MID-360专用)
KEY_STATE_HOST_CFG  = 0x0005   # 状态推送目标
KEY_PCL_HOST_CFG    = 0x0006   # 点云推送目标
KEY_IMU_HOST_CFG    = 0x0007   # IMU 推送目标
KEY_LIDAR_IP        = 0x0004   # 设备IP配置 (可写/可读): IP(4B)+子网掩码(4B)+网关(4B), 共12字节
KEY_FOV_CFG0        = 0x0015   # FOV 配置文件 0 (可写/可读): yaw_start/stop + pitch_start/stop, 20字节
KEY_FOV_CFG1        = 0x0016   # FOV 配置文件 1 (可写/可读); 两套配置文件可同时使能
KEY_FOV_CFG_EN      = 0x0017   # FOV 使能 (可写/可读): uint8, Bit0=cfg0启用, Bit1=cfg1启用（可同时置位）
KEY_DETECT_MODE     = 0x0018   # 探测模式: 0=Normal, 1=Sensitive
KEY_PATTERN_MODE    = 0x0001   # 扫描模式 (可写): 0=非重复, 1=重复(暂不支持), 2=低帧率重复(暂不支持)
KEY_SN              = 0x8000   # 序列号 (只读)
KEY_WORK_STATE      = 0x8006   # 当前工作状态 (只读)
KEY_LIDAR_TEMP      = 0x8007   # 内核温度 (只读, int32_t, 单位 0.01°C，由设备状态推送帧携带)

# 可写工作模式（KEY_WORK_MODE = 0x001A，写命令用）
WORK_SAMPLING       = 0x01   # Normal ：点云采集，激光器与电机均开启
WORK_STANDBY        = 0x02   # WakeUp/Standby：冷启动态，扫描模块与激光器均关闭，切换到SAMPLING需较长时间
WORK_SLEEP          = 0x03   # Sleep  ：深度休眠（低功耗），恢复慢
WORK_READY          = 0x09   # Ready  ：就绪态（MID-360/HAP专用），扫描模块运行，激光器关闭，可快速切换到SAMPLING
WORK_IDLE           = WORK_STANDBY  # 向后兼容别名（实际为Standby冷启动态，避免使用）

# 只读工作状态（KEY_WORK_STATE = 0x8006，设备上报）
# 对应 SDK2 LivoxLidarWorkMode 枚举 + MID-360 扩展
WORK_STATE_NORMAL          = 0x01   # Normal       ：正常采集中
WORK_STATE_WAKEUP          = 0x02   # WakeUp       ：冷启动/唤醒态
WORK_STATE_SLEEP           = 0x03   # Sleep        ：深度休眠
WORK_STATE_ERROR           = 0x04   # Error/Fault  ：设备故障
WORK_STATE_SELFTEST        = 0x05   # PowerOnSelfTest：上电自检
WORK_STATE_MOTOR_STARTING  = 0x06   # MotorStarting ：电机启动中
WORK_STATE_MOTOR_STOPPING  = 0x07   # MotorStopping ：电机停止中
WORK_STATE_UPGRADE         = 0x08   # Upgrade       ：固件升级中
WORK_STATE_READY           = 0x09   # Ready         ：就绪（MID-360/HAP）

# 点云缓冲区上限（点数），超出后停止采集。
# 2 千万点 × 28 字节/点（N×7 float32 运行时缓冲）≈ 534 MiB。
MAX_BUFFER_POINTS = 20_000_000

# 设备注册不会等待网络握手；worker 在后台完成。关闭/移除时所有握手共用同一
# deadline，避免设备数量放大停机等待时间。
HANDSHAKE_START_DELAY_SEC = 0.2
HANDSHAKE_SHUTDOWN_TIMEOUT_SEC = 5.0
HANDSHAKE_FORCE_CLOSE_TIMEOUT_SEC = 1.0

# 命令锁获取超时（秒）：握手 worker 与关闭清扫/手动切换并发时，避免对
# _cmd_lock 无限等待（获取失败即返回 None——全链路已把 None 当命令失败处理）。
CMD_LOCK_ACQUIRE_TIMEOUT_SEC = 2.0

# 批次等待设备进入采集状态的超时（秒）与轮询间隔。
BATCH_READY_WAIT_TIMEOUT_SEC = 30.0
BATCH_READY_POLL_INTERVAL_SEC = 0.5

# 控制帧头长度
CTRL_HDR_LEN = 24   # sof+ver+len+seq+cmd_id+cmd_type+sender+resv+crc16+crc32
CTRL_HDR_FMT = "<BBH IH BB 6s H I"  # 24 bytes total


# ────────────────────────────────────────────────────────────
# CRC 计算
# ────────────────────────────────────────────────────────────

def _crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refin=false, refout=false. 查表法."""
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


# 预计算 CRC-16 查找表
def _build_crc16_table() -> list:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
        table.append(crc)
    return table

_CRC16_TABLE = _build_crc16_table()


def _crc32(data: bytes) -> int:
    """CRC-32 (IEEE 802.3, refin=true, refout=true)."""
    import binascii
    return binascii.crc32(data) & 0xFFFFFFFF


# ────────────────────────────────────────────────────────────
# 帧构建 / 解析
# ────────────────────────────────────────────────────────────

def _build_ctrl_frame(cmd_id: int, data: bytes, seq: int = 0,
                      cmd_type: int = CMD_TYPE_REQ) -> bytes:
    """构建控制命令帧."""
    total_len = CTRL_HDR_LEN + len(data)
    # 先用 0 占位 crc16/crc32
    hdr_nochek = struct.pack(
        "<BBH IH BB 6s",
        SOF, PROTO_VER, total_len,
        seq, cmd_id,
        cmd_type, SENDER_HOST,
        b'\x00' * 6,
    )  # 18 bytes
    crc16 = _crc16_ccitt(hdr_nochek)
    crc32 = _crc32(data) if data else 0
    header = hdr_nochek + struct.pack("<HI", crc16, crc32)  # +6 → 24 bytes
    return header + data


def _parse_ctrl_frame(raw: bytes) -> Optional[Dict[str, Any]]:
    """解析控制命令帧，返回字段字典；失败返回 None."""
    if len(raw) < CTRL_HDR_LEN:
        return None
    if raw[0] != SOF:
        return None
    try:
        sof, ver, length, seq_num, cmd_id, cmd_type, sender, resv, crc16, crc32 = \
            struct.unpack_from("<BBH IH BB 6s H I", raw, 0)
    except struct.error:
        return None
    if length > len(raw):
        return None
    data_field = raw[CTRL_HDR_LEN: length]
    # CRC16 验证 — 校验失败则丢弃帧
    expected_crc16 = _crc16_ccitt(raw[:18])
    if expected_crc16 != crc16:
        logger.debug("CRC16 mismatch: recv=%04x calc=%04x — frame discarded", crc16, expected_crc16)
        return None
    # CRC32 验证 — 校验数据段完整性
    if data_field:
        expected_crc32 = _crc32(data_field)
        if expected_crc32 != crc32:
            logger.debug("CRC32 mismatch: recv=%08x calc=%08x — frame discarded", crc32, expected_crc32)
            return None
    return {
        "version": ver, "length": length, "seq_num": seq_num,
        "cmd_id": cmd_id, "cmd_type": cmd_type, "sender": sender,
        "crc16": crc16, "crc32": crc32,
        "data": data_field,
    }


# ────────────────────────────────────────────────────────────
# key_value_list 辅助
# ────────────────────────────────────────────────────────────

def _quick_query_sn(ip: str, port: int = PORT_CTRL) -> str:
    """向指定 IP 发送 PARAM_INQUIRE(KEY_SN) 并返回 SN 字符串；超时/失败返回空串。"""
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.5)
        kv_data = struct.pack("<HH H", 1, 0, KEY_SN)
        frame = _build_ctrl_frame(CMD_PARAM_INQUIRE, kv_data, seq=1)
        sock.sendto(frame, (ip, port))
        raw, _ = sock.recvfrom(4096)
        resp_frame = _parse_ctrl_frame(raw)
        if resp_frame and resp_frame["cmd_id"] == CMD_PARAM_INQUIRE \
                and resp_frame["cmd_type"] == CMD_TYPE_ACK:
            resp = resp_frame["data"]
            # 响应格式: ret(1B)+num(2B)+rsvd(2B)+key(2B)+raw_value...
            if resp and len(resp) > 7 and resp[0] == 0:
                return resp[7:].split(b'\x00')[0].decode('ascii', errors='replace').strip()
    except Exception:
        logger.log_event(
            "driver.mid360.serial.query.failed",
            "MID-360 序列号查询失败",
            level="WARNING",
            context={"deviceIp": ip, "stage": "query", "status": "degraded"},
        )
    finally:
        try:
            if sock is not None:
                sock.close()
        except Exception:
            logger.log_event(
                "driver.mid360.socket.close.failed",
                "MID-360 查询套接字关闭失败",
                level="WARNING",
                context={"deviceIp": ip, "stage": "cleanup", "status": "degraded"},
            )
    return ""


def _build_key_value(key: int, value: bytes) -> bytes:
    """构建单个 key_value 条目."""
    return struct.pack("<HH", key, len(value)) + value


def _build_param_config(kv_pairs: List[tuple]) -> bytes:
    """构建 cmd_id=0x0100 的 data 字段 (key_num + rsvd + key_value_list)."""
    kv_bytes = b"".join(_build_key_value(k, v) for k, v in kv_pairs)
    return struct.pack("<HH", len(kv_pairs), 0) + kv_bytes


def _ip_to_bytes(ip: str) -> bytes:
    return socket.inet_aton(ip)


# ────────────────────────────────────────────────────────────
# 点云帧解析（独立格式，非控制帧）
# ────────────────────────────────────────────────────────────

PCL_HDR_FMT = "<B H HHH BB B 12s I Q"  # 36 bytes
PCL_HDR_LEN = struct.calcsize(PCL_HDR_FMT)  # should be 36


def _parse_pcl_packet(raw: bytes) -> Optional[np.ndarray]:
    """
    解析点云 UDP 包（data_type=1, Cartesian 32bit）.
    返回 (N,5) float32 [x_m, y_m, z_m, refl, tag], or None。

    解析层不丢弃任何 tag；设备级策略在写入采集缓冲前统一应用。
    """
    if len(raw) < PCL_HDR_LEN:
        return None
    try:
        fields = struct.unpack_from(PCL_HDR_FMT, raw, 0)
    except struct.error:
        return None
    ver, length, time_interval, dot_num, udp_cnt, frame_cnt, data_type, time_type, resv, crc32, timestamp = fields
    if data_type != 1:
        # 仅处理 Cartesian 32bit (data_type=1)
        return None
    point_size = 14  # x(4)+y(4)+z(4)+refl(1)+tag(1)
    pts_raw = raw[PCL_HDR_LEN: PCL_HDR_LEN + dot_num * point_size]
    if len(pts_raw) < dot_num * point_size:
        return None
    # CRC-32: MID-360 固件在点云推送帧中 CRC32 字段始终为 0（不计算），
    # 因此仅当字段非零时才校验（覆盖 timestamp+data，即 offset 28 起）
    if crc32 != 0:
        crc_payload = raw[28: PCL_HDR_LEN + dot_num * point_size]
        expected_crc32 = _crc32(crc_payload)
        if expected_crc32 != crc32:
            return None
    pts_list = []
    for i in range(dot_num):
        off = i * point_size
        x, y, z, refl, tag = struct.unpack_from("<iiiBB", pts_raw, off)
        pts_list.append([x / 1000.0, y / 1000.0, z / 1000.0, float(refl), float(tag)])
    if not pts_list:
        return np.empty((0, 5), dtype=np.float32)
    return np.array(pts_list, dtype=np.float32)


def _apply_tag_filter(points: np.ndarray, policy: Dict[str, bool]) -> np.ndarray:
    """按 Livox tag 分组过滤，保留原始 tag 列。

    MID-360 协议定义 bit0..1=相邻物体间胶着点、bit2..3=雨雾尘、
    bit4..5=其他探测异常；bit6..7 为保留位，不参与用户过滤策略。
    """
    if points.size == 0:
        return points
    return points[livox_tag_keep_mask(points[:, 4], policy)]


def _filter_finite_point_rows(points: np.ndarray, *, task_id: str, label: str) -> np.ndarray:
    """过滤点数组中的非有限 XYZ。"""
    if points.size == 0:
        return points

    finite_mask = np.isfinite(points[:, :3]).all(axis=1)
    removed = int(np.sum(~finite_mask))
    if removed == 0:
        return points

    logger.warning("任务 %s %s：过滤掉 %d 个非有限点", task_id, label, removed)
    return points[finite_mask]


def _needs_warmup(prev_statuses: Dict[int, Optional[str]], started: List[int]) -> bool:
    """判断采集前是否需要预热丢弃不稳定数据（P-3）。

    设备在采集前均已处于 ``scanning``（采集间保持活跃，未发生模式切换）时
    无需预热；任一台设备从其他状态（standby/ready/未知）进入 SAMPLING 时
    保留预热。``prev_statuses`` 为批次开始前的锁内状态快照。
    """
    return any(prev_statuses.get(did) != "scanning" for did in started)


# ────────────────────────────────────────────────────────────
# 单设备句柄
# ────────────────────────────────────────────────────────────

class Mid360Device:
    """管理单台 MID-360 的命令通道（端口 56100）."""

    def __init__(self, device_id: int, ip: str, host_ip: str,
                 ctrl_port: int = PORT_CTRL,
                 host_ctrl_port: int = 56101,
                 pcl_host_port: int = PORT_PUSH_PCL,
                 state_host_port: int = PORT_PUSH_STATE,
                 imu_host_port: int = PORT_PUSH_IMU):
        self.device_id = device_id
        self.ip = ip
        self.host_ip = host_ip
        self.ctrl_port = ctrl_port
        self.host_ctrl_port = host_ctrl_port
        self.pcl_host_port = pcl_host_port
        self.state_host_port = state_host_port
        self.imu_host_port = imu_host_port
        self.status = "offline"
        # 工作模式命令纪元：每次下发 standby/ready/SAMPLING 命令（set_work_mode）
        # 自增。批次在启动时记下"自己那次 SAMPLING 之后"的纪元，失败/超时回退前
        # 若发现纪元已被推进，说明设备已被其它批次或会话（如会话 teardown 的
        # READY、手工 API）重新指挥过——此时回退必须跳过，否则会改写别人的设备
        # 模式（真机证据：孤儿批次把平台刚置好的 ready 改回 standby）。
        self.mode_epoch = 0
        self.sn: str = ""
        self.temperature: Optional[float] = None  # 激光雷达温度 °C，由状态推送帧更新
        self._seq = 0
        self._lock = threading.Lock()
        self._cmd_lock = threading.Lock()  # 串行化控制命令，防止同一 socket 并发读写
        # 状态变化回调，由驱动注册: (device_id, new_status) -> None
        self.on_status_change: Optional[Callable[[int, str], None]] = None

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(2.0)
        # 每台设备绑定独立本机端口，避免多设备端口冲突
        try:
            self._sock.bind(("", host_ctrl_port))
            logger.debug("MID360 device %d: ctrl socket bound to :%d", device_id, host_ctrl_port)
        except OSError as e:
            logger.warning("MID360 设备 %d：控制套接字绑定端口 :%d 失败：%s，将使用临时端口",
                           device_id, host_ctrl_port, e)

    def _next_seq(self) -> int:
        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            return self._seq

    def _send_recv(self, cmd_id: int, data: bytes = b"",
                   retries: int = 2, timeout: float = 2.0) -> Optional[bytes]:
        """发送命令并等待应答；校验 seq_num；返回 data 字段，超时/错误返回 None.

        命令锁获取有界（``CMD_LOCK_ACQUIRE_TIMEOUT_SEC``）：后台握手 worker 与
        runtime 冷却清扫/手动切换并发时，避免无限等待锁（None 失败语义全链路通用）。
        """
        if not self._cmd_lock.acquire(timeout=CMD_LOCK_ACQUIRE_TIMEOUT_SEC):
            logger.debug("[MID360 %s] cmd 0x%04x 命令锁获取超时", self.ip, cmd_id)
            return None
        try:
          for attempt in range(retries):
            seq = self._next_seq()
            frame = _build_ctrl_frame(cmd_id, data, seq=seq)
            # 发送前先清空接收缓冲区（防止接受旧的缓存响应）
            self._sock.setblocking(False)
            try:
                while True:
                    self._sock.recvfrom(4096)
            except Exception:
                # intentional-ignore: non-blocking receive drain ends on EAGAIN.
                pass
            self._sock.setblocking(True)
            self._sock.settimeout(timeout)
            try:
                self._sock.sendto(frame, (self.ip, self.ctrl_port))
                deadline = time.time() + timeout
                while time.time() < deadline:
                    resp_raw, addr = self._sock.recvfrom(4096)
                    if addr[0] != self.ip:
                        continue
                    resp = _parse_ctrl_frame(resp_raw)
                    if resp is None:
                        continue
                    # 必须匹配 cmd_id、ACK 以及 seq_num
                    if (resp["cmd_id"] == cmd_id
                            and resp["cmd_type"] == CMD_TYPE_ACK
                            and resp["seq_num"] == seq):
                        return resp["data"]
            except socket.timeout:
                logger.debug("[MID360 %s] cmd 0x%04x attempt %d timeout",
                             self.ip, cmd_id, attempt + 1)
          return None
        finally:
            self._cmd_lock.release()

    # ── 具体操作 ──────────────────────────────────────────────

    def configure_push_destinations(self, timeout: float = 2.0,
                                    retries: int = 2) -> bool:
        """
        配置点云/状态/IMU 推送目标为本机 IP+端口，
        并请求 Cartesian 32bit 点云格式.
        """
        host_ip_b = _ip_to_bytes(self.host_ip)

        def ipc_value(dest_port: int, src_port: int) -> bytes:
            return host_ip_b + struct.pack("<HH", dest_port, src_port)

        kv = [
            (KEY_PCL_DATA_TYPE, struct.pack("<B", 0x01)),  # Cartesian 32bit
            (KEY_PCL_HOST_CFG,   ipc_value(self.pcl_host_port, PORT_PUSH_PCL)),
            (KEY_STATE_HOST_CFG, ipc_value(self.state_host_port, PORT_PUSH_STATE)),
            (KEY_IMU_HOST_CFG,   ipc_value(self.imu_host_port, PORT_PUSH_IMU)),
        ]
        resp = self._send_recv(CMD_PARAM_CONFIG, _build_param_config(kv),
                               timeout=timeout, retries=retries)
        if resp is not None and len(resp) >= 1 and resp[0] == 0:
            logger.info("[MID360 %s] 推送目标已配置 → %s:%d",
                        self.ip, self.host_ip, self.pcl_host_port)
            return True
        logger.warning("[MID360 %s] 推送目标配置失败：resp=%s",
                       self.ip, resp.hex() if resp else None)
        return False

    def query_sn(self) -> bool:
        """查询硬件序列号，成功则更新 self.sn."""
        kv_data = struct.pack("<HH H", 1, 0, KEY_SN)
        resp = self._send_recv(CMD_PARAM_INQUIRE, kv_data)
        if resp and len(resp) > 7 and resp[0] == 0:
            self.sn = resp[7:].split(b'\x00')[0].decode('ascii', errors='replace').strip()
            return True
        return False

    def query_state(self, timeout: float = 2.0, retries: int = 2) -> bool:
        """查询当前工作状态（KEY_WORK_STATE=0x8006），成功则更新 self.status."""
        kv_data = struct.pack("<HH H", 1, 0, KEY_WORK_STATE)
        resp = self._send_recv(CMD_PARAM_INQUIRE, kv_data,
                               timeout=timeout, retries=retries)
        if resp is not None and len(resp) >= 4:
            ret = resp[0]
            if ret == 0:
                if len(resp) > 7:
                    work_state = resp[7]
                    # 完整映射：涵盖 SDK2 LivoxLidarWorkMode 所有枚举值
                    # 以及 MID-360/HAP 专用的 READY 状态(0x09)
                    state_map = {
                        WORK_STATE_NORMAL:         "scanning",       # 0x01 正常采集中
                        WORK_STATE_WAKEUP:         "standby",        # 0x02 冷启动/唤醒态
                        WORK_STATE_SLEEP:          "sleep",           # 0x03 深度休眠
                        WORK_STATE_ERROR:          "error",           # 0x04 设备故障
                        WORK_STATE_SELFTEST:       "selftest",        # 0x05 上电自检
                        WORK_STATE_MOTOR_STARTING: "motor_starting",  # 0x06 电机启动中
                        WORK_STATE_MOTOR_STOPPING: "motor_stopping",  # 0x07 电机停止中
                        WORK_STATE_UPGRADE:        "upgrade",         # 0x08 固件升级中
                        WORK_STATE_READY:          "ready",           # 0x09 就绪(快速启动)
                    }
                    self.status = state_map.get(work_state, f"unknown(0x{work_state:02x})")
                return True
        return False

    def set_work_mode(self, mode: int, *, timeout: float | None = None,
                      retries: int | None = None) -> bool:
        """设置工作模式: WORK_SAMPLING=0x01, WORK_IDLE=0x02.

        ``timeout``/``retries`` 覆盖命令级超时与重试（RT 快速失败清扫
        用短超时单次尝试）；缺省沿用模块默认（2s × 2）。

        本方法是设备工作模式的唯一命令入口，因此**命令纪元在此自增**（含失败
        尝试：下发过就说明有人指挥过这台设备）。
        """
        self.mode_epoch += 1
        kv = [(KEY_WORK_MODE, struct.pack("<B", mode))]
        resp = self._send_recv(
            CMD_PARAM_CONFIG, _build_param_config(kv),
            retries=retries if retries is not None else 2,
            timeout=timeout if timeout is not None else 2.0,
        )
        if resp is not None and len(resp) >= 1 and resp[0] == 0:
            return True
        logger.warning("[MID360 %s] 工作模式设置失败 mode=%d：resp=%s",
                       self.ip, mode, resp.hex() if resp else None)
        return False

    def start_sampling(self) -> bool:
        """
        配置推流目标并切换到 SAMPLING。
        发送命令后不主动设置状态，状态仅通过 query_state() 轮询获取。
        """
        ok = self.configure_push_destinations()
        if not ok:
            logger.warning("[MID360 %s] 推送配置失败，仍将尝试启动", self.ip)
        ok2 = self.set_work_mode(WORK_SAMPLING)
        if ok2:
            logger.info("[MID360 %s] SAMPLING 命令已接受（收到 ACK）", self.ip)
        return ok2

    def enter_ready(self, *, timeout: float | None = None,
                    retries: int | None = None) -> bool:
        """切换到 READY 就绪模式（扫描模块保持运行，激光器关闭，可快速切换回 SAMPLING）."""
        ok = self.set_work_mode(WORK_READY, timeout=timeout, retries=retries)
        if ok:
            logger.info("[MID360 %s] READY 命令已接受（收到 ACK）", self.ip)
        return ok

    def enter_standby(self, *, timeout: float | None = None,
                      retries: int | None = None) -> bool:
        """切换到 STANDBY 冷启动状态（扫描模块+激光器均关闭，唤醒慢，尽量避免使用）."""
        ok = self.set_work_mode(WORK_STANDBY, timeout=timeout, retries=retries)
        if ok:
            logger.info("[MID360 %s] STANDBY 命令已接受（收到 ACK）", self.ip)
        return ok

    # ── FOV 配置 (KEY_FOV_CFG0=0x0015, KEY_FOV_CFG1=0x0016, KEY_FOV_CFG_EN=0x0017) ──

    def _query_fov_enable(self) -> int:
        """查询 KEY_FOV_CFG_EN 当前值，失败返回 0。"""
        kv_data = struct.pack("<HH H", 1, 0, KEY_FOV_CFG_EN)
        resp = self._send_recv(CMD_PARAM_INQUIRE, kv_data)
        if resp and len(resp) >= 8 and resp[0] == 0:
            return resp[7]
        return 0

    def set_fov_config(self, profile: int, yaw_start: float, yaw_stop: float,
                       pitch_start: float, pitch_stop: float, enable: bool) -> bool:
        """
        设置 FOV 限制配置文件（profile 0 或 1）。
        角度单位：整数度，yaw[0,360) pitch(-10,60)。
        两套配置文件可独立使能，互不影响：使用读-改-写保留另一路的使能状态。
        """
        key = KEY_FOV_CFG0 if profile == 0 else KEY_FOV_CFG1
        fov_value = struct.pack("<iiiiI",
                                int(round(yaw_start)),
                                int(round(yaw_stop)),
                                int(round(pitch_start)),
                                int(round(pitch_stop)),
                                0)  # rsvd
        # 读-改-写：读取当前使能状态，仅修改本 profile 对应的 bit
        current_en = self._query_fov_enable()
        if enable:
            en_byte = current_en | (1 << profile)
        else:
            en_byte = current_en & ~(1 << profile) & 0xFF
        kv = [(key, fov_value), (KEY_FOV_CFG_EN, struct.pack("<B", en_byte))]
        resp = self._send_recv(CMD_PARAM_CONFIG, _build_param_config(kv))
        if resp is not None and len(resp) >= 1 and resp[0] == 0:
            logger.info("[MID360 %s] FOV 配置 %d 已设置：yaw=[%.1f,%.1f] pitch=[%.1f,%.1f] en=%s",
                        self.ip, profile, yaw_start, yaw_stop, pitch_start, pitch_stop, enable)
            return True
        logger.warning("[MID360 %s] FOV 配置设置失败 profile=%d：resp=%s",
                       self.ip, profile, resp.hex() if resp else None)
        return False

    def query_fov_config(self, profile: int) -> Optional[Dict[str, Any]]:
        """
        查询 FOV 配置文件（profile 0 或 1）。
        返回 {yaw_start, yaw_stop, pitch_start, pitch_stop, enable} 或 None（查询失败）。
        角度以度为单位返回。
        """
        key = KEY_FOV_CFG0 if profile == 0 else KEY_FOV_CFG1
        # 同时查询 FOV region 和 FOV enable 两个 key
        kv_data = struct.pack("<HH HH", 2, 0, key, KEY_FOV_CFG_EN)
        resp = self._send_recv(CMD_PARAM_INQUIRE, kv_data)
        # 响应格式: ret(1B)+key_num(2B)+[key(2B)+val_len(2B)+value]*
        if not resp or len(resp) < 3 or resp[0] != 0:
            logger.warning("[MID360 %s] FOV 配置查询失败 profile=%d：resp=%s",
                           self.ip, profile, resp.hex() if resp else None)
            return None
        try:
            key_num = struct.unpack_from("<H", resp, 1)[0]
            pos = 3
            fov_tuple = None
            en_value: Optional[bool] = None
            for _ in range(key_num):
                if pos + 4 > len(resp):
                    break
                rkey, rlen = struct.unpack_from("<HH", resp, pos)
                pos += 4
                if pos + rlen > len(resp):
                    break
                val = resp[pos: pos + rlen]
                pos += rlen
                if rkey == key and rlen >= 16:
                    y_start, y_stop, p_start, p_stop = struct.unpack_from("<iiii", val)
                    fov_tuple = (y_start, y_stop, p_start, p_stop)
                elif rkey == KEY_FOV_CFG_EN and rlen >= 1:
                    en_value = bool(val[0] & (1 << profile))
            if fov_tuple is None:
                logger.warning("[MID360 %s] FOV 配置查询响应中没有 FOV 值 profile=%d",
                               self.ip, profile)
                return None
            return {
                "yaw_start":   float(fov_tuple[0]),
                "yaw_stop":    float(fov_tuple[1]),
                "pitch_start": float(fov_tuple[2]),
                "pitch_stop":  float(fov_tuple[3]),
                "enable":      en_value if en_value is not None else False,
            }
        except Exception as e:
            logger.warning("[MID360 %s] FOV 配置响应解析失败 profile=%d：%s",
                           self.ip, profile, e)
            return None

    def set_fov_enable_byte(self, enable: int) -> bool:
        """
        直接写入 KEY_FOV_CFG_EN 字节（0-3 bitmask: Bit0=cfg0, Bit1=cfg1）。
        enable=0: 关闭所有; 1: 仅cfg0; 2: 仅cfg1; 3: 两者都启用。
        """
        kv = [(KEY_FOV_CFG_EN, struct.pack("<B", enable & 0x03))]
        resp = self._send_recv(CMD_PARAM_CONFIG, _build_param_config(kv))
        if resp is not None and len(resp) >= 1 and resp[0] == 0:
            logger.info("[MID360 %s] FOV 启用字节已设为 0x%02x", self.ip, enable)
            return True
        logger.warning("[MID360 %s] FOV 启用字节设置失败：resp=%s",
                       self.ip, resp.hex() if resp else None)
        return False

    # ── 设备 IP 配置 (KEY_LIDAR_IP=0x0004) ──────────────────────

    def set_lidar_ip(self, ip: str, subnet: str = "255.255.255.0", gateway: str = "0.0.0.0") -> bool:
        """
        配置设备自身的 IP 地址、子网掩码和网关（12字节）。
        注意：修改后设备需重启才能使用新 IP 连接，当前连接将中断。
        """
        try:
            value = socket.inet_aton(ip) + socket.inet_aton(subnet) + socket.inet_aton(gateway)
        except OSError as e:
            logger.warning("[MID360 %s] 雷达 IP 设置失败：地址无效：%s", self.ip, e)
            return False
        kv = [(KEY_LIDAR_IP, value)]
        resp = self._send_recv(CMD_PARAM_CONFIG, _build_param_config(kv))
        if resp is not None and len(resp) >= 1 and resp[0] == 0:
            logger.info("[MID360 %s] IP 已配置：ip=%s subnet=%s gw=%s", self.ip, ip, subnet, gateway)
            return True
        logger.warning("[MID360 %s] 雷达 IP 设置失败：resp=%s", self.ip, resp.hex() if resp else None)
        return False

    def query_lidar_ip(self) -> Optional[Dict[str, str]]:
        """
        查询设备当前 IP 配置（IP + 子网掩码 + 网关）。
        返回 {ip, subnet, gateway} 或 None。
        """
        kv_data = struct.pack("<HH H", 1, 0, KEY_LIDAR_IP)
        resp = self._send_recv(CMD_PARAM_INQUIRE, kv_data)
        # 响应: ret(1B)+key_num(2B)+key(2B)+val_len(2B)+value(12B) = 19B total
        if resp and len(resp) >= 19 and resp[0] == 0:
            try:
                return {
                    "ip":      socket.inet_ntoa(resp[7:11]),
                    "subnet":  socket.inet_ntoa(resp[11:15]),
                    "gateway": socket.inet_ntoa(resp[15:19]),
                }
            except OSError:
                # intentional-ignore: malformed device response is reported by
                # the stable warning below.
                pass
        logger.warning("[MID360 %s] 雷达 IP 查询失败：resp=%s", self.ip, resp.hex() if resp else None)
        return None

    def reboot(self, delay_ms: int = 2000) -> bool:
        """发送设备重启命令 (0x0200)，delay_ms 为重启延迟（毫秒）。"""
        data = struct.pack("<H", delay_ms)
        resp = self._send_recv(CMD_REBOOT, data)
        if resp and len(resp) >= 1 and resp[0] == 0:
            logger.info("[MID360 %s] 重启命令已接受，delay=%dms", self.ip, delay_ms)
            return True
        logger.warning("[MID360 %s] 重启命令失败：resp=%s", self.ip, resp.hex() if resp else None)
        return False

    # ── 扫描模式 (KEY_PATTERN_MODE=0x0001) ───────────────────

    # 模式常量
    PATTERN_NON_REPETITIVE = 0  # 非重复扫描：逐帧覆盖不同区域，积累可填满 FOV（推荐用于静态隧道场景）
    PATTERN_REPETITIVE     = 1  # 重复扫描：固定扫描线（当前固件暂不支持）
    PATTERN_LOW_FRAME_RATE = 2  # 低帧率重复扫描：低转速模式（当前固件暂不支持）

    def set_pattern_mode(self, mode: int) -> bool:
        """
        设置扫描模式（0=非重复, 1=重复[暂不支持], 2=低帧率重复[暂不支持]）。
        建议在设备处于 READY 或 STANDBY 状态时调用，生效后在下次扫描时应用。
        """
        if mode not in (0, 1, 2):
            logger.warning("[MID360 %s] 扫描模式设置失败：无效模式 %d", self.ip, mode)
            return False
        kv = [(KEY_PATTERN_MODE, struct.pack("<B", mode))]
        resp = self._send_recv(CMD_PARAM_CONFIG, _build_param_config(kv))
        if resp is not None and len(resp) >= 1 and resp[0] == 0:
            logger.info("[MID360 %s] 扫描模式已设为 %d", self.ip, mode)
            return True
        logger.warning("[MID360 %s] 扫描模式设置失败 mode=%d：resp=%s",
                       self.ip, mode, resp.hex() if resp else None)
        return False

    def query_pattern_mode(self) -> Optional[int]:
        """查询当前扫描模式（0/1/2），失败返回 None。"""
        kv_data = struct.pack("<HH H", 1, 0, KEY_PATTERN_MODE)
        resp = self._send_recv(CMD_PARAM_INQUIRE, kv_data)
        if resp and len(resp) >= 8 and resp[0] == 0:
            return resp[7]
        return None

    def set_detect_mode(self, mode: str) -> bool:
        """设置探测模式（normal/sensitive）。"""
        values = {"normal": 0, "sensitive": 1}
        if mode not in values:
            return False
        kv = [(KEY_DETECT_MODE, struct.pack("<B", values[mode]))]
        resp = self._send_recv(CMD_PARAM_CONFIG, _build_param_config(kv))
        return bool(resp is not None and len(resp) >= 1 and resp[0] == 0)

    def close(self):
        try:
            self._sock.close()
        except Exception:
            logger.log_event(
                "driver.mid360.socket.close.failed",
                "MID-360 设备套接字关闭失败",
                level="WARNING",
                context={"deviceId": self.device_id, "stage": "cleanup", "status": "degraded"},
            )


# ────────────────────────────────────────────────────────────
# 主驱动类
# ────────────────────────────────────────────────────────────

class Mid360LidarDriver(LidarDriver):
    """
    Livox MID-360 多设备驱动.

    特性：
    - 广播发现 (端口 56000)
    - 命令通信 (端口 56100)
    - 点云接收 (端口 56301, 默认)
    - 采集交付设备原始坐标点云（raw 帧，坐标帧变换由平台门面统一应用）
    """

    def __init__(self, host_ip: str = "192.168.1.50",
                 pcl_port: int = PORT_PUSH_PCL):
        self.host_ip = host_ip
        self.pcl_port = pcl_port

        self._devices: Dict[int, Mid360Device] = {}
        self._tasks: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()

        # 点云接收套接字
        self._pcl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pcl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._pcl_sock.settimeout(0.05)
        try:
            self._pcl_sock.bind(("", pcl_port))
            logger.info("MID360Driver：PCL 套接字已绑定端口 :%d", pcl_port)
        except OSError as e:
            logger.warning("MID360Driver：PCL 套接字绑定端口 :%d 失败：%s", pcl_port, e)

        # 设备状态推送接收套接字（device → host, 源端口 56200, 目标端口 56201）
        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._state_sock.settimeout(0.05)
        try:
            self._state_sock.bind(("", PORT_PUSH_STATE))
            logger.info("MID360Driver：状态套接字已绑定端口 :%d", PORT_PUSH_STATE)
        except OSError as e:
            logger.warning("MID360Driver：状态套接字绑定端口 :%d 失败：%s", PORT_PUSH_STATE, e)

        # 各设备最近一次收到状态推送的时间戳（ip → float)
        self._last_state_seen: Dict[str, float] = {}

        # 广播发现套接字（接收 0x0000 的 ACK）
        self._bcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._bcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._bcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._bcast_sock.settimeout(0.05)
        try:
            self._bcast_sock.bind(("", 0))  # 随机主机端端口
        except OSError as e:
            logger.warning("MID360Driver：广播套接字绑定失败：%s", e)

        self._running = True
        self._point_buffers: Dict[str, List[np.ndarray]] = {}
        self._buffer_pt_totals: Dict[str, int] = {}  # buf_key → 缓冲区已有总点数
        self._device_pt_counts: Dict[str, Dict[str, int]] = {}  # buf_key → {"device_id": count}
        self._ip_id_cache: Dict[str, int] = {}  # ip → device_id 快速查找缓存
        self._recv_thread_alive = True  # 接收线程存活标记
        self._task_cancel_events: Dict[str, threading.Event] = {}  # task_id → 取消事件
        self._silent_stop_tasks: Set[str] = set()  # 用户触发 stop-scan 的任务集合，capture 循环遇到时静默退出
        # 网卡切换后重配推流目标失败/探测不可达的设备 id 集合，由心跳循环周期重试
        self._reconfig_retry: Set[int] = set()
        self._handshake_threads: Dict[int, threading.Thread] = {}
        self._handshake_cancel_events: Dict[int, threading.Event] = {}
        # 设备级质量策略由平台持久化，driver 只保留运行时快照。
        self._device_parameter_settings: Dict[int, Dict[str, Any]] = {}

        # 外部设备状态变化回调（可由服务层注册，用于同步数据库等）
        # 签名: (device_id: int, new_status: str) -> None
        self.on_device_status_change: Optional[Callable[[int, str], None]] = None
        # Hardware initialization fact. Runtime installs the observer before
        # devices are registered; the driver never decides READY/STANDBY policy.
        self.on_device_initialized: Optional[Callable[[int], None]] = None

        # ── 事件化回调（由服务层注册）─────────────────────────
        # 设计：驱动只发"硬件事件"，不传递业务状态字串。

        # 采集成功
        # 签名: (task_id, capture_info: dict) -> None
        # capture_info: {deviceIds, duration, devicePointCounts, pointCount,
        #                points (N×7 float32 只读数组，末列为 tag), bufferTruncated}
        self.on_batch_collected: Optional[Callable[[str, Dict[str, Any]], None]] = None

        # 采集失败（设备启动失败、采集超时、点云为空、写文件失败等）
        # 签名: (task_id, error_msg, devices_attempted) -> None
        self.on_batch_failed: Optional[Callable[[str, str, List[int]], None]] = None

        # 采集被用户主动取消（非错误，silent_stop 语义）
        # 签名: (task_id) -> None
        self.on_batch_cancelled: Optional[Callable[[str], None]] = None

        # 点云接收线程
        self._pcl_thread = threading.Thread(
            target=self._pcl_recv_loop, daemon=True, name="mid360-pcl")
        self._pcl_thread.start()

        # 设备状态被动接收线程
        self._state_thread = threading.Thread(
            target=self._state_recv_loop, daemon=True, name="mid360-state")
        self._state_thread.start()

        # 心跳线程（仅检测超时离线，不主动查询）
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="mid360-hb")
        self._hb_thread.start()

    # ── 设备注册 & 握手 ───────────────────────────────────────

    def _state_recv_loop(self):
        """被动接收设备推送的状态帧，解析 KEY_WORK_STATE 并更新设备状态。"""
        _state_map = {
            WORK_STATE_NORMAL:         "scanning",
            WORK_STATE_WAKEUP:         "standby",
            WORK_STATE_SLEEP:          "sleep",
            WORK_STATE_ERROR:          "error",
            WORK_STATE_SELFTEST:       "selftest",
            WORK_STATE_MOTOR_STARTING: "motor_starting",
            WORK_STATE_MOTOR_STOPPING: "motor_stopping",
            WORK_STATE_UPGRADE:        "upgrade",
            WORK_STATE_READY:          "ready",
        }
        while self._running:
            try:
                raw, addr = self._state_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception:
                logger.log_event(
                    "driver.mid360.state.receive.failed",
                    "MID-360 状态接收失败",
                    level="WARNING",
                    context={"stage": "receive", "status": "degraded"},
                )
                time.sleep(0.02)
                continue

            src_ip = addr[0]
            frame = _parse_ctrl_frame(raw)
            if frame is None:
                continue
            # 只处理设备主动推送的帧（sender=LIDAR, cmd_type=REQ）
            if frame["sender"] != SENDER_LIDAR or frame["cmd_type"] != CMD_TYPE_REQ:
                continue

            data = frame["data"]
            if len(data) < 4:
                continue

            # 解析 key_value_list: key_num(2B) + rsvd(2B) + entries
            try:
                key_num = struct.unpack_from("<H", data, 0)[0]
                pos = 4
                work_state_val = None
                temp_val: Optional[float] = None
                for _ in range(key_num):
                    if pos + 4 > len(data):
                        break
                    key, val_len = struct.unpack_from("<HH", data, pos)
                    pos += 4
                    if pos + val_len > len(data):
                        break
                    val_bytes = data[pos: pos + val_len]
                    pos += val_len
                    if key == KEY_WORK_STATE and val_len >= 1:
                        work_state_val = val_bytes[0]
                    elif key == KEY_LIDAR_TEMP and val_len >= 4:
                        # 温度：int32_t，单位 0.01°C
                        temp_raw = struct.unpack_from("<i", val_bytes)[0]
                        temp_val = round(temp_raw / 100.0, 1)
            except Exception as _pe:
                logger.log_event(
                    "driver.mid360.state.parse.failed",
                    "MID-360 状态帧解析失败",
                    level="WARNING",
                    context={"deviceIp": src_ip, "stage": "parse", "status": "degraded"},
                )
                continue

            if work_state_val is None:
                # 即使没有工作状态，也要更新温度和在线时间戳
                did = self._ip_to_devid(src_ip)
                if did is not None:
                    with self._lock:
                        dev = self._devices.get(did)
                        if dev is not None and temp_val is not None:
                            dev.temperature = temp_val
                self._last_state_seen[src_ip] = time.time()
                continue

            new_status = _state_map.get(work_state_val, f"unknown(0x{work_state_val:02x})")

            did = self._ip_to_devid(src_ip)
            if did is None:
                self._last_state_seen[src_ip] = time.time()
                continue

            with self._lock:
                dev = self._devices.get(did)
                if dev is None:
                    continue
                prev_status = dev.status
                dev.status = new_status
                if temp_val is not None:
                    dev.temperature = temp_val
            self._last_state_seen[src_ip] = time.time()

            if new_status != prev_status:
                logger.info("MID360：设备 %d（%s）状态推送：%s → %s",
                            did, src_ip, prev_status, new_status)
                self._on_device_status_change(did, new_status)

    def _on_device_status_change(self, device_id: int, new_status: str) -> None:
        """设备状态变化时触发外部回调（例如同步数据库）。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return
        # 调用外部注册的回调（例如同步数据库）
        if self.on_device_status_change:
            try:
                self.on_device_status_change(device_id, new_status)
            except Exception as _cb_err:
                logger.log_event(
                    "driver.mid360.status_callback.failed",
                    "MID-360 设备状态回调失败",
                    level="ERROR",
                    context={
                        "deviceId": device_id,
                        "stage": "publish",
                        "status": "failed",
                    },
                    exc_info=True,
                )

    def add_device(self, device_id: int, ip: str, port: int = PORT_CTRL) -> None:
        thread: threading.Thread | None = None
        start_error: Exception | None = None
        with self._lock:
            if device_id in self._devices:
                return
            host_ctrl_port = 56100 + device_id
            try:
                dev = Mid360Device(device_id, ip, self.host_ip,
                                   ctrl_port=port,
                                   host_ctrl_port=host_ctrl_port,
                                   pcl_host_port=self.pcl_port)
            except Exception as e:
                logger.log_event(
                    "driver.mid360.device.create.failed",
                    "MID-360 设备创建失败",
                    level="ERROR",
                    context={
                        "deviceId": device_id,
                        "deviceIp": ip,
                        "stage": "startup",
                        "status": "failed",
                    },
                    exc_info=True,
                )
                return
            dev.on_status_change = self._on_device_status_change
            self._devices[device_id] = dev
            self._ip_id_cache[ip] = device_id
            cancel_event = threading.Event()
            thread = threading.Thread(
                target=self._handshake,
                args=(device_id, dev, cancel_event),
                daemon=True,
                name=f"mid360-handshake-{device_id}",
            )
            self._handshake_threads[device_id] = thread
            self._handshake_cancel_events[device_id] = cancel_event
            try:
                thread.start()
            except Exception as error:
                start_error = error
                self._handshake_threads.pop(device_id, None)
                self._handshake_cancel_events.pop(device_id, None)
        logger.info("MID360：正在注册设备 %d @ %s:%d", device_id, ip, port)
        if start_error is not None:
            self._settle_handshake_offline(device_id, dev)
            self._log_handshake_failure(device_id, dev.ip, "MID360_HANDSHAKE_FAILED")

    def remove_device(self, device_id: int) -> None:
        """从驱动中移除设备，关闭其 socket 资源。"""
        self._cancel_handshake_workers(
            {device_id},
            timeout_sec=HANDSHAKE_SHUTDOWN_TIMEOUT_SEC,
        )
        # Connection reset is implemented as remove + add. Publish an offline
        # boundary before the old device disappears so the runtime scheduler
        # closes this initialization generation; otherwise an early status
        # frame from the replacement device could run idle/heat policy before
        # its new handshake publishes on_device_initialized.
        with self._lock:
            current = self._devices.get(device_id)
            if current is not None:
                current.status = "offline"
                current.temperature = None
        if current is not None:
            self._on_device_status_change(device_id, "offline")
        with self._lock:
            dev = self._devices.pop(device_id, None)
            if dev:
                self._ip_id_cache.pop(dev.ip, None)
            self._reconfig_retry.discard(device_id)
            self._device_parameter_settings.pop(device_id, None)
        if dev:
            dev.close()
        logger.info("MID360：设备 %d 已移除", device_id)

    def _handshake(
        self,
        device_id: int,
        dev: Mid360Device,
        cancel_event: threading.Event,
    ) -> None:
        """上线时：配置推流目标；工作模式（冷/热）由 runtime 状态策略接管。"""
        try:
            # Event.wait 让 remove/shutdown 能取消尚未开始网络 I/O 的握手。
            if cancel_event.wait(HANDSHAKE_START_DELAY_SEC):
                return
            if not self._handshake_is_current(device_id, dev, cancel_event):
                return

            if not dev.configure_push_destinations():
                self._handle_handshake_failure(
                    device_id,
                    dev,
                    cancel_event,
                    code="MID360_HANDSHAKE_FAILED",
                )
                return
            if cancel_event.is_set():
                return

            # 查询并缓存设备序列号（best effort）。
            try:
                dev.query_sn()
            except Exception:
                logger.log_event(
                    "driver.mid360.serial.query.failed",
                    "MID-360 序列号查询失败，初始化继续",
                    level="WARNING",
                    context={"deviceId": device_id, "stage": "startup", "status": "degraded"},
                )
            if cancel_event.is_set():
                return
            with self._lock:
                settings = dict(self._device_parameter_settings.get(device_id, {}))
            detect_mode = settings.get("detectMode")
            if detect_mode in {"normal", "sensitive"}:
                try:
                    restored = dev.set_detect_mode(str(detect_mode))
                except Exception:
                    restored = False
                if not restored:
                    logger.log_event(
                        "driver.mid360.detect-mode.restore.failed",
                        "MID-360 持久化探测模式恢复失败",
                        level="WARNING",
                        context={"deviceId": device_id, "stage": "startup", "status": "degraded"},
                    )
            if cancel_event.is_set():
                return
            observer = self.on_device_initialized
            if observer is not None:
                try:
                    observer(device_id)
                except Exception:
                    logger.log_event(
                        "driver.mid360.initialized.publish.failed",
                        "MID-360 初始化完成事件处理失败",
                        level="WARNING",
                        context={"deviceId": device_id, "stage": "startup", "status": "degraded"},
                        exc_info=True,
                    )
            logger.info("MID360：设备 %d 握手完成，正在等待状态推送", device_id)
        except OSError:
            self._handle_handshake_failure(
                device_id,
                dev,
                cancel_event,
                code="MID360_DEVICE_UNREACHABLE",
            )
        except Exception:
            self._handle_handshake_failure(
                device_id,
                dev,
                cancel_event,
                code="MID360_HANDSHAKE_FAILED",
                exc_info=True,
            )
        finally:
            current = threading.current_thread()
            with self._lock:
                if self._handshake_threads.get(device_id) is current:
                    self._handshake_threads.pop(device_id, None)
                    self._handshake_cancel_events.pop(device_id, None)

    def _handshake_is_current(
        self,
        device_id: int,
        dev: Mid360Device,
        cancel_event: threading.Event,
    ) -> bool:
        with self._lock:
            return (
                not cancel_event.is_set()
                and self._running
                and self._devices.get(device_id) is dev
                and self._handshake_cancel_events.get(device_id) is cancel_event
            )

    def _handle_handshake_failure(
        self,
        device_id: int,
        dev: Mid360Device,
        cancel_event: threading.Event,
        *,
        code: str,
        exc_info: bool = False,
    ) -> None:
        """把真实握手失败收敛为一次安全事件；取消不属于失败。"""
        # 失败与 remove/shutdown 的取消共用此锁作为线性化点：先取得锁的一方
        # 决定该结果是“真实失败”还是“主动取消”，避免检查通过后取消又误报 ERROR。
        with self._lock:
            if (
                cancel_event.is_set()
                or not self._running
                or self._devices.get(device_id) is not dev
                or self._handshake_cancel_events.get(device_id) is not cancel_event
            ):
                return
            dev.status = "offline"
            dev.temperature = None
        self._on_device_status_change(device_id, "offline")
        self._log_handshake_failure(device_id, dev.ip, code, exc_info=exc_info)

    def _settle_handshake_offline(self, device_id: int, dev: Mid360Device) -> None:
        with self._lock:
            if self._devices.get(device_id) is not dev:
                return
            dev.status = "offline"
            dev.temperature = None
        # 即使设备初始值本来就是 offline 也主动投影一次；EventHub 接线后的
        # initial sync 会覆盖接线前已经结束的握手。
        self._on_device_status_change(device_id, "offline")

    @staticmethod
    def _log_handshake_failure(
        device_id: int,
        device_ip: str,
        code: str,
        *,
        exc_info: bool = False,
    ) -> None:
        logger.log_event(
            "driver.mid360.handshake.failed",
            "MID-360 设备启动握手失败，设备保持离线",
            level="ERROR",
            context={
                "deviceId": device_id,
                "deviceIp": device_ip,
                "driver": "mid360",
                "stage": "startup",
                "status": "failed",
                "code": code,
            },
            exc_info=exc_info,
        )

    def _cancel_handshake_workers(
        self,
        device_ids: Set[int] | None = None,
        *,
        timeout_sec: float,
    ) -> Set[int]:
        """取消并在共享 deadline 内回收握手 worker。

        先在共享 deadline 内协作取消；仅对仍阻塞的 worker 关闭对应命令
        socket，以打断 ``recvfrom``，再进行一次短暂的最终等待。返回因此提前
        关闭 socket 的设备 id，供 shutdown 跳过后续 standby 命令。
        """
        with self._lock:
            selected = set(self._handshake_threads)
            if device_ids is not None:
                selected.intersection_update(device_ids)
            workers = [
                (device_id, self._handshake_threads[device_id])
                for device_id in sorted(selected)
            ]
            for device_id in selected:
                cancel_event = self._handshake_cancel_events.get(device_id)
                if cancel_event is not None:
                    cancel_event.set()

        deadline = time.monotonic() + max(0.0, timeout_sec)
        self._join_handshake_workers(workers, deadline)

        current = threading.current_thread()
        blocked = [
            (device_id, thread)
            for device_id, thread in workers
            if thread is not current and thread.is_alive()
        ]
        closed_device_ids: Set[int] = set()
        for device_id, _thread in blocked:
            with self._lock:
                dev = self._devices.get(device_id)
            if dev is not None:
                dev.close()
                closed_device_ids.add(device_id)

        force_close_deadline = time.monotonic() + HANDSHAKE_FORCE_CLOSE_TIMEOUT_SEC
        self._join_handshake_workers(blocked, force_close_deadline)
        for device_id, thread in blocked:
            if thread is current or not thread.is_alive():
                continue
            logger.log_event(
                "driver.mid360.handshake.cleanup.incomplete",
                "MID-360 握手线程未能在关闭期限内退出",
                level="WARNING",
                context={
                    "deviceId": device_id,
                    "stage": "cleanup",
                    "status": "degraded",
                    "code": "MID360_HANDSHAKE_CLEANUP_TIMEOUT",
                },
            )
        return closed_device_ids

    @staticmethod
    def _join_handshake_workers(
        workers: List[tuple[int, threading.Thread]],
        deadline: float,
    ) -> None:
        current = threading.current_thread()
        for _device_id, thread in workers:
            if thread is current or not thread.is_alive():
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            thread.join(timeout=remaining)

    # ── 点云接收循环 ──────────────────────────────────────────

    def _pcl_recv_loop(self):
        _pkt_cnt: Dict[str, int] = {}   # ip → 收到包数
        _pt_cnt:  Dict[str, int] = {}   # ip → 有效点数
        _crc_fail_cnt: int = 0          # CRC 校验失败帧计数
        _log_interval = 5.0
        _last_log = time.time()

        try:
            while self._running:
                try:
                    raw, addr = self._pcl_sock.recvfrom(65535)
                except socket.timeout:
                    # 周期性输出每设备统计（仅当本周期内有新数据时才打印）
                    now = time.time()
                    if _pkt_cnt and now - _last_log >= _log_interval:
                        for ip, cnt in _pkt_cnt.items():
                            logger.info("PCL 接收 [%s]：%d 包、%d 点（最近 %.0fs）",
                                        ip, cnt, _pt_cnt.get(ip, 0), _log_interval)
                        if _crc_fail_cnt > 0:
                            total_pkts = sum(_pkt_cnt.values())
                            fail_rate = _crc_fail_cnt / max(total_pkts, 1) * 100
                            logger.warning("PCL 接收：%d 次 CRC 失败（%.1f%%，最近 %.0fs）",
                                           _crc_fail_cnt, fail_rate, _log_interval)
                        _pkt_cnt.clear()
                        _pt_cnt.clear()
                        _crc_fail_cnt = 0
                        _last_log = now
                    continue
                except OSError:
                    if not self._running:
                        break
                    time.sleep(0.02)
                    continue
                except Exception:
                    time.sleep(0.02)
                    continue

                src_ip = addr[0]
                _pkt_cnt[src_ip] = _pkt_cnt.get(src_ip, 0) + 1

                pts = _parse_pcl_packet(raw)
                if pts is None:
                    _crc_fail_cnt += 1
                    continue
                if pts.shape[0] == 0:
                    continue

                _pt_cnt[src_ip] = _pt_cnt.get(src_ip, 0) + pts.shape[0]

                did = self._ip_to_devid(src_ip)
                with self._lock:
                    settings = dict(self._device_parameter_settings.get(did or 0, {}))
                task_pts = _apply_tag_filter(
                    pts,
                    dict(settings.get("tagFilter") or DEFAULT_LIVOX_TAG_FILTER),
                )
                if task_pts.shape[0] == 0:
                    continue

                # 写入活跃采集任务缓冲
                dev_id_val = float(did) if did is not None else 0.0
                with self._lock:
                    for tid, task in self._tasks.items():
                        # 活跃 = 有正在进行的采集（capturing is True）
                        if not task.get("capturing"):
                            continue
                        buf_key = task.get("buf_key")
                        if not buf_key:
                            continue
                        # 检查任务是否已取消
                        cancel_ev = self._task_cancel_events.get(tid)
                        if cancel_ev and cancel_ev.is_set():
                            continue
                        cur_total = self._buffer_pt_totals.get(buf_key, 0)
                        if cur_total >= MAX_BUFFER_POINTS:
                            continue
                        dev_col = np.full((task_pts.shape[0], 1), dev_id_val, dtype=np.float32)
                        # PCD 保留 batch 列（恒为 1）：旧 wire 格式兼容，runtime 不消费
                        batch_col = np.ones((task_pts.shape[0], 1), dtype=np.float32)
                        # 保持既有 devId/batch 列位置，tag 追加为第 7 列。
                        pts_ext = np.hstack([
                            task_pts[:, :4],
                            dev_col,
                            batch_col,
                            task_pts[:, 4:5],
                        ])
                        if buf_key not in self._point_buffers:
                            self._point_buffers[buf_key] = []
                        self._point_buffers[buf_key].append(pts_ext)
                        self._buffer_pt_totals[buf_key] = cur_total + pts_ext.shape[0]
                        if did is not None:
                            did_s = str(did)
                            dpc = self._device_pt_counts.setdefault(buf_key, {})
                            dpc[did_s] = dpc.get(did_s, 0) + task_pts.shape[0]
                        if self._buffer_pt_totals[buf_key] >= MAX_BUFFER_POINTS:
                            task["buffer_full"] = True
                            logger.error("任务 %s：缓冲区达到上限（%d 点 >= %d），"
                                         "正在停止采集（交付结果将被截断）",
                                         tid, self._buffer_pt_totals[buf_key], MAX_BUFFER_POINTS)
        except Exception:
            logger.log_event(
                "driver.mid360.receive.loop.failed",
                "PCL 接收循环发生致命错误",
                level="ERROR",
                context={"stage": "receive", "status": "failed"},
                exc_info=True,
            )
        finally:
            self._recv_thread_alive = False
            if self._running:
                logger.warning("PCL 接收循环意外退出")
            else:
                logger.info("PCL 接收循环已退出")

        # 线程退出时打印最后一个周期的未输出数据
        if _pkt_cnt:
            for ip, cnt in _pkt_cnt.items():
                logger.info("PCL 最后接收区间 [%s]：%d 包、%d 点",
                            ip, cnt, _pt_cnt.get(ip, 0))

    def _ip_to_devid(self, ip: str) -> Optional[int]:
        return self._ip_id_cache.get(ip)

    # ── 心跳 ─────────────────────────────────────────────────

    def _heartbeat_loop(self):
        """定期检查各设备是否超时未推送状态，超时则标记离线（不主动查询设备）。

        另消费网卡切换后遗留的 ``_reconfig_retry``：周期重试探测+重配推流
        目标，设备恢复后移出集合，状态经回调回流（ADR-0018 自愈）。
        """
        OFFLINE_TIMEOUT = 15.0   # 超过此秒数未收到状态推送 → 标记为 offline
        INTERVAL = 5.0           # 检查间隔（秒）

        while self._running:
            time.sleep(INTERVAL)
            now = time.time()
            with self._lock:
                device_ids = list(self._devices.keys())
            for did in device_ids:
                with self._lock:
                    dev = self._devices.get(did)
                if dev is None:
                    continue

                last_seen = self._last_state_seen.get(dev.ip, 0)
                if now - last_seen > OFFLINE_TIMEOUT and dev.status != "offline":
                    dev.status = "offline"
                    dev.temperature = None
                    logger.warning(
                        "MID360：设备 %d（%s）已标记离线（%.0fs 内未收到状态推送）",
                        did, dev.ip, now - last_seen,
                    )
                    self._on_device_status_change(did, "offline")

            # 网卡切换遗留设备自愈重试（限速：每 INTERVAL 一轮，每台一次探测）
            with self._lock:
                retry_ids = list(self._reconfig_retry)
            for did in retry_ids:
                with self._lock:
                    dev = self._devices.get(did)
                if dev is None:
                    with self._lock:
                        self._reconfig_retry.discard(did)
                    continue
                if self._verify_and_reconfigure(did, dev):
                    with self._lock:
                        self._reconfig_retry.discard(did)
                    logger.info("MID360：设备 %d 在网卡切换后已重新连接", did)

    # ── 广播发现 ─────────────────────────────────────────────

    def discover_subnet(self, subnet: str = "192.168.1",
                        timeout: float = 2.0,
                        targets: List[str] | None = None) -> List[Dict[str, str]]:
        """
        向子网广播发送发现命令（cmd_id=0x0000），收集响应.
        返回响应设备的 IP 列表.

        :param subnet:  子网前缀，如 "192.168.1"，用于构造定向广播地址
        :param timeout: 超时秒数
        :param targets: 额外的 unicast 目标 IP 列表。在 WSL / NAT 环境下
                        广播无法穿越到物理网卡，通过 56100 端口 SN 查询探测。

        关键：必须将套接字绑定到 host_ip 所在网卡，否则 Windows 多网卡
        环境下系统按默认路由选出口，广播会从错误的网卡发出，LiDAR 收不到。
        """
        ping = _build_ctrl_frame(CMD_DISCOVERY, b"", seq=0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 绑定到 host_ip，确保广播从正确的网卡（LiDAR 所在子网）发出
        try:
            sock.bind((self.host_ip, 0))
            logger.info("MID360 发现：套接字已绑定到 %s", self.host_ip)
        except OSError as e:
            logger.warning("MID360 发现：绑定到 %s 失败：%s，将使用默认路由（可能无法连接雷达）", self.host_ip, e)
        sock.settimeout(1.0)   # 每次 recvfrom 最多等 1 秒，超时后立即重发

        subnet_bcast = f"{subnet}.255"   # 定向子网广播（更可靠）

        def _send_ping():
            for dst in (subnet_bcast, "255.255.255.255"):
                try:
                    sock.sendto(ping, (dst, PORT_BCAST_CMD))
                    logger.debug("MID360 discover: sent broadcast → %s:%d", dst, PORT_BCAST_CMD)
                except Exception as _e:
                    logger.warning("MID360 发现：向 %s 广播失败：%s", dst, _e)

        _send_ping()
        found = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(4096)
                frame = _parse_ctrl_frame(data)
                if frame and frame["cmd_id"] == CMD_DISCOVERY and \
                        frame["cmd_type"] == CMD_TYPE_ACK:
                    ip = addr[0]
                    if ip == self.host_ip:
                        continue
                    if any(d["ip"] == ip for d in found):
                        continue
                    payload = frame["data"]
                    if len(payload) < 2:
                        logger.debug("MID360 discover: ACK from %s too short (%d bytes), skip",
                                     ip, len(payload))
                        continue
                    ret_code = payload[0]
                    if ret_code != 0:
                        logger.debug("MID360 discover: %s ret_code=%d, skip", ip, ret_code)
                        continue
                    dev_type = payload[1]
                    if dev_type not in ACCEPTED_DEV_TYPES:
                        logger.info("MID360 发现：%s 的 dev_type=%d 不属于 {MID-360(9), "
                                    "MID-360S(35)}，已跳过", ip, dev_type)
                        continue
                    model_name = DEV_TYPE_NAMES[dev_type]
                    found.append({"ip": ip, "deviceType": dev_type, "modelName": model_name})
                    logger.info("MID360：在 %s 发现设备 %s", ip, model_name)
            except socket.timeout:
                # 每次超时后重发，持续轰炸直到 deadline
                _send_ping()
        sock.close()

        # 对广播发现的设备查询 SN
        for d in found:
            d["sn"] = _quick_query_sn(d["ip"])

        # targets 走 56100 端口 unicast 探测（WSL / NAT 下广播无效的兜底）
        found_ips = {d["ip"] for d in found}
        for target_ip in (targets or []):
            if target_ip in found_ips:
                continue
            logger.info("MID360 发现：正在通过 56100 端口 SN 查询探测 %s", target_ip)
            sn = _quick_query_sn(target_ip)
            if sn:
                dev_info = {
                    "ip": target_ip,
                    "deviceType": DEV_TYPE_MID360,
                    "modelName": "MID-360",
                    "sn": sn,
                }
                found.append(dev_info)
                found_ips.add(target_ip)
                logger.info("MID360：通过 56100 端口探测在 %s 发现 %s", target_ip, "MID-360")

        logger.info("MID360 发现结果：%s",
                    [(d["ip"], d.get("modelName")) for d in found])
        return found

    def discover_device(self, ip: str, timeout: float = 2.0) -> Dict[str, str] | None:
        """通过 56100 端口 SN 查询探测指定 IP 的设备（适用于 WSL / NAT 环境）。

        :param ip:      目标设备 IP
        :param timeout: 超时秒数（保留参数，实际由 _quick_query_sn 内部控制）
        :return:        设备信息字典，或 None（无响应）
        """
        sn = _quick_query_sn(ip)
        if sn:
            result = {"ip": ip, "deviceType": DEV_TYPE_MID360, "modelName": "MID-360", "sn": sn}
            logger.info("MID360 指定设备发现：通过 56100 端口在 %s 发现 MID-360", ip)
            return result
        logger.info("MID360 指定设备发现：%s 未响应", ip)
        return None

    # ── 采集流程 ─────────────────────────────────────────────

    def create_task(self) -> str:
        """创建采集任务（不启动采集），返回 task_id。

        本方法仅初始化驱动侧的硬件层任务结构（缓冲/取消事件等），**单次采集
        原语**：任务生命周期内只存在一次采集，无批次/合并概念。业务状态机由
        服务层独立维护，本结构**不再保留** status / batches / batch_files
        字段。
        """
        task_id = str(uuid.uuid4())
        with self._lock:
            self._tasks[task_id] = {
                "progress": 0,
                "capturing": False,
                "buf_key": None,
                "error": None,
            }
            self._task_cancel_events[task_id] = threading.Event()
        logger.info("任务 %s 已创建", task_id)
        return task_id

    def start_batch(self, task_id: str, device_ids: List[int],
                    duration: float) -> None:
        """触发一次采集。在 daemon 线程中执行。

        准入条件：任务未处于采集状态且设备可用。设备准入判据由
        ``app.drivers.scan_admission`` 唯一表达——**稳态不可采集**才在这里拒绝，
        ``motor_starting``/``motor_stopping`` 等瞬态交给 ``_run_batch`` 的等待窗口
        （冷唤醒实测约 6 s），避免冷态首次采集在任何 I/O 之前被瞬时拒绝。
        Raises ValueError on validation failure.
        """
        blocked: list[tuple[int, str, str, str]] = []
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                raise ValueError(f"任务 {task_id[:8]}… 不存在")
            if task.get("capturing"):
                raise ValueError(f"任务 {task_id[:8]}… 已有正在进行的采集，无法触发新采集")
            # 检查设备
            for did in device_ids:
                dev = self._devices.get(did)
                if dev is None:
                    raise ValueError(f"设备 {did} 未注册到驱动")
                admission = scan_admission(dev.status)
                if admission.blocked:
                    blocked.append((did, dev.ip, str(dev.status), admission.status))
            if not blocked:
                # 占用任务：进入采集
                task["capturing"] = True
                task["progress"] = 0
                task["error"] = None
                self._silent_stop_tasks.discard(task_id)
                buf_key = task_id
                task["buf_key"] = buf_key
                self._point_buffers[buf_key] = []
                self._buffer_pt_totals[buf_key] = 0
                self._device_pt_counts[buf_key] = {}

        if blocked:
            # 拒绝原因必须可见（IO 不进锁：日志与抛出都在锁外）
            for did, ip, raw_status, status_key in blocked:
                logger.log_event(
                    "driver.mid360.batch.admission.rejected",
                    f"MID-360 采集准入被拒绝：设备 {did}（{ip}）状态 '{raw_status}'，无法扫描",
                    level="WARNING",
                    context={
                        "deviceId": did,
                        "deviceIp": ip,
                        "stage": "admission",
                        "status": "rejected",
                        "code": f"DEVICE_{status_key.upper()}",
                    },
                )
            did, ip, raw_status, _ = blocked[0]
            raise ValueError(
                f"设备 {did}（{ip}）当前状态为 '{raw_status}'，无法扫描"
            )

        th = threading.Thread(
            target=self._run_batch,
            args=(task_id, device_ids, duration), daemon=True,
            name=f"capture-{task_id[:8]}")
        th.start()

    def get_task_progress(self, task_id: str) -> Optional[int]:
        """返回采集线程维护的进度快照，不暴露可变任务字典。"""

        with self._lock:
            task = self._tasks.get(task_id)
            value = task.get("progress") if task is not None else None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return max(0, min(100, int(value)))

    def _run_batch(self, task_id: str, device_ids: List[int],
                   duration: float):
        """执行一次采集：启动设备 → 采集 → 写临时文件 → 停止设备."""
        logger.info("任务 %s：采集 %.1fs，devices=%s",
                    task_id, duration, device_ids)
        buf_key = task_id

        # 批次开始前的设备状态快照（状态由推送线程异步更新；用于判断本批次
        # 是否实际发生 standby/ready→SAMPLING 切换，决定是否需要预热丢弃数据）
        with self._lock:
            prev_statuses = {
                did: (self._devices[did].status if did in self._devices else None)
                for did in device_ids
            }

        def _fail(msg: str):
            """统一失败处理：清理 buffer + 释放任务占用 + 通知服务层。

            错误记录由服务层根据回调写入 DB，驱动不维护成功/失败清单。
            """
            with self._lock:
                self._point_buffers.pop(buf_key, None)
                self._buffer_pt_totals.pop(buf_key, None)
                self._device_pt_counts.pop(buf_key, None)
                task = self._tasks.get(task_id)
                if task:
                    task["buf_key"] = None
                    task["capturing"] = False
                    task["progress"] = 0
                    task["error"] = msg
            if self.on_batch_failed:
                try:
                    self.on_batch_failed(task_id, msg, list(device_ids))
                except Exception:
                    logger.log_event(
                        "driver.mid360.batch_failed_callback.failed",
                        "MID-360 批次失败回调执行失败",
                        level="ERROR",
                        context={
                            "taskId": task_id,
                            "stage": "publish",
                            "status": "failed",
                            **_device_log_context(device_ids),
                        },
                        exc_info=True,
                    )

        def _deliver_silent_stop(label: str) -> None:
            """静默停止收尾（等待窗口 / 等待结束 / 自然结束竞态共用）。

            契约（与 ``stop_current_batch`` 一致）：取消只交付
            ``on_batch_cancelled``，**不发出任何设备模式命令**——设备保持当前
            工作模式，可立即接受下一次采集。回退只属于失败路径，且受命令纪元
            判据约束（见 ``_revert_device_mode``）。
            """
            with self._lock:
                self._silent_stop_tasks.discard(task_id)
                self._point_buffers.pop(buf_key, None)
                self._buffer_pt_totals.pop(buf_key, None)
                self._device_pt_counts.pop(buf_key, None)
                task = self._tasks.get(task_id)
                if task is not None:
                    task["buf_key"] = None
                    task["capturing"] = False
                    task["progress"] = 0
            logger.log_event(
                "driver.mid360.batch.cancelled",
                label,
                level="INFO",
                context={
                    "taskId": task_id,
                    "stage": "cancel",
                    "status": "cancelled",
                    "code": "BATCH_CANCELLED",
                    **_device_log_context(started),
                },
            )
            if self.on_batch_cancelled:
                try:
                    self.on_batch_cancelled(task_id)
                except Exception:
                    logger.log_event(
                        "driver.mid360.batch_cancelled_callback.failed",
                        "MID-360 批次取消回调执行失败",
                        level="ERROR",
                        context={
                            "taskId": task_id,
                            "stage": "publish",
                            "status": "failed",
                            **_device_log_context(started),
                        },
                        exc_info=True,
                    )

        # ── 1. 并行启动设备 ──────────────────────────────────
        from concurrent.futures import ThreadPoolExecutor, as_completed

        # 本批次对每台设备的"命令纪元"所有权：读取 SAMPLING 命令下发前的纪元，
        # 命令成功即本批次拥有 before+1（set_work_mode 每次命令自增 1）。回退前
        # 纪元不相等 = 已被他人接管，回退必须跳过（见 _revert_device_mode）。
        expected_epochs: Dict[int, int] = {}

        def _start_one(did):
            with self._lock:
                dev = self._devices.get(did)
            if dev is None:
                logger.warning("任务 %s：设备 %d 不存在", task_id, did)
                return None
            before = dev.mode_epoch
            if dev.start_sampling():
                expected_epochs[did] = before + 1
                return did
            logger.warning("任务 %s：设备 %d 启动失败",
                           task_id, did)
            return None

        started = []
        with ThreadPoolExecutor(max_workers=len(device_ids)) as pool:
            futs = {pool.submit(_start_one, did): did for did in device_ids}
            for fut in as_completed(futs):
                r = fut.result()
                if r is not None:
                    started.append(r)

        if not started:
            _fail("所有设备均无法切换到 SAMPLING")
            return

        # ── 2. 等待设备进入采集状态 ──────────────────────────
        # 等待窗口必须消费取消语义：stop_current_batch 只置 _silent_stop_tasks
        # 与取消事件，不在此处读取就会让批次跑满 30s 窗口、并在超时后按陈旧快照
        # 回退设备模式（真机：取消后 28.7s 发 STANDBY，撤销了下一个任务的
        # SAMPLING）。命中取消即走与"计时采集期间取消"相同的收尾。
        _t_wait = time.time()
        while time.time() - _t_wait < BATCH_READY_WAIT_TIMEOUT_SEC:
            if self._batch_abandoned(task_id):
                _deliver_silent_stop("进入采集状态前静默停止")
                return
            with self._lock:
                all_ok = all(
                    (d := self._devices.get(did)) is not None and d.status == "scanning"
                    for did in started
                )
            if all_ok:
                break
            time.sleep(BATCH_READY_POLL_INTERVAL_SEC)
        else:
            # 边界竞态：窗口刚结束就被取消/放弃时，按取消收尾而不是记失败并回退模式。
            if self._batch_abandoned(task_id):
                _deliver_silent_stop("进入采集状态前静默停止")
                return
            # 列出未就绪的设备及状态
            not_ready = []
            with self._lock:
                for did in started:
                    dev = self._devices.get(did)
                    if dev is None or dev.status != "scanning":
                        st = dev.status if dev else "missing"
                        not_ready.append(f"{did}({st})")
            logger.error("任务 %s：设备未能在 %.0fs 内进入采集状态，"
                         "未就绪: %s", task_id, BATCH_READY_WAIT_TIMEOUT_SEC,
                         ", ".join(not_ready))
            with self._lock:
                devs_to_revert = [self._devices.get(did) for did in started]
            for dev in devs_to_revert:
                if dev:
                    self._revert_device_mode(
                        dev, prev_statuses,
                        expected_epoch=expected_epochs.get(dev.device_id),
                    )
            _fail(f"设备未能在{int(BATCH_READY_WAIT_TIMEOUT_SEC)}秒内进入采集状态")
            return

        waited = time.time() - _t_wait
        if waited >= BATCH_READY_POLL_INTERVAL_SEC:
            # 冷唤醒（standby → motor_starting → ready → scanning）需要显式等待，
            # 耗时必须可见，否则"准入放行瞬态"无法从日志验证。
            logger.info("任务 %s：等待设备进入采集状态 %.1fs", task_id, waited)

        # 等待结束（设备已 scanning）后才发现取消：同样走共用收尾，不回退模式。
        if self._batch_abandoned(task_id):
            _deliver_silent_stop("进入采集状态前静默停止")
            return
        logger.info("任务 %s：采集已开始", task_id)

        # ── 2.5 预热等待（仅实际发生 SAMPLING 切换时丢弃前 0.5s 不稳定数据）──
        # 设备在采集前已处于 scanning（采集间保持活跃，未发生模式切换）时跳过
        # 预热，避免无条件 sleep + 丢弃有效数据。
        _WARMUP_SECS = 0.5
        entered_sampling = _needs_warmup(prev_statuses, started)
        if entered_sampling:
            logger.info("任务 %s：预热 %.1fs（丢弃初始数据）",
                        task_id, _WARMUP_SECS)
            time.sleep(_WARMUP_SECS)
            # 清空预热期间积累的不可靠数据
            with self._lock:
                self._point_buffers[buf_key] = []
                self._buffer_pt_totals[buf_key] = 0
                self._device_pt_counts[buf_key] = {}
        else:
            logger.info("任务 %s：设备已在采集中，跳过预热",
                        task_id)

        # ── 3. 计时采集 ──────────────────────────────────────
        cancel_ev = self._task_cancel_events.get(task_id)
        t0 = time.time()
        buffer_full = False
        cancelled = False
        while time.time() - t0 < duration:
            # 检查取消事件（兼顾静默停止旗标，避免事件被替换后漏检）
            if (cancel_ev and cancel_ev.is_set()) or task_id in self._silent_stop_tasks:
                cancelled = True
                logger.info("任务 %s：采集期间已取消", task_id)
                break
            # 检查接收线程是否存活
            if not self._recv_thread_alive:
                logger.error("任务 %s：PCL 接收线程已退出", task_id)
                _fail("点云接收线程异常退出")
                return
            pct = int((time.time() - t0) / duration * 100)
            with self._lock:
                task_meta = self._tasks.get(task_id)
                if task_meta is None:
                    # 任务已被 drop_task 移除（取消竞态）：停止推进进度
                    break
                if task_meta.get("buffer_full"):
                    buffer_full = True
                    break
                task_meta["progress"] = pct
            time.sleep(0.2)

        if cancelled:
            with self._lock:
                silent = task_id in self._silent_stop_tasks
            if silent:
                # 与等待窗口/自然结束竞态共用同一份收尾（取消不下发任何模式命令）。
                _deliver_silent_stop("用户静默停止（无错误）")
                return
            with self._lock:
                self._silent_stop_tasks.discard(task_id)
                self._point_buffers.pop(buf_key, None)
                self._buffer_pt_totals.pop(buf_key, None)
                self._device_pt_counts.pop(buf_key, None)
                task = self._tasks.get(task_id)
                if task is not None:
                    task["buf_key"] = None
                    task["capturing"] = False
                    task["progress"] = 0
            _fail("扫描已取消")
            return

        if buffer_full:
            logger.error("任务 %s：缓冲区达到上限，交付结果将被截断 "
                         "(bufferTruncated=true in on_batch_collected)", task_id)

        # ── silent_stop 与自然结束的竞态保护 ────────────────────
        # loop 自然退出后到取出 buffer 之前，若用户已触发停止，应静默退出
        # 而不是走 _finalize_batch 的"未收到点云数据"错误路径。
        if self._batch_abandoned(task_id):
            _deliver_silent_stop("采集循环自然退出后静默停止")
            return
        with self._lock:
            task_meta = self._tasks.get(task_id)
            if task_meta is not None:
                task_meta["progress"] = 100

        # ── 4. 取出缓冲（采集间保持设备工作状态）────────────────
        with self._lock:
            buf_total = self._buffer_pt_totals.get(buf_key, 0)
            buf_chunks = len(self._point_buffers.get(buf_key, []))
            bufs = self._point_buffers.pop(buf_key, [])
            self._buffer_pt_totals.pop(buf_key, None)
            task_meta = self._tasks.get(task_id)
            if task_meta is not None:
                task_meta.pop("buffer_full", None)
                task_meta["buf_key"] = None

        logger.info("任务 %s：采集完成，缓冲区包含 %d 个分块 / %d 个点 "
                     "（recv_thread alive=%s）",
                     task_id, buf_chunks, buf_total, self._recv_thread_alive)

        # ── 5. 装配并交付点云（内存数组，无中间文件）────────────
        self._finalize_batch(task_id, device_ids, duration, bufs, buffer_full=buffer_full)

    def _finalize_batch(self, task_id: str,
                        device_ids: List[int], duration: float,
                        bufs: List[np.ndarray], *, buffer_full: bool = False):
        """装配采集点云（过滤/统计）并通过回调以内存数组交付。

        扫描产物无持久化意义（ADR-0018 决策）：不写中间文件，交付
        ``points``（N×7 float32：x y z intensity devId batch tag，只读数组，
        runtime 经 ``NativePointCloudBatch.from_owned`` 零拷贝接管）。
        缓冲超限截断时（``buffer_full``）在回调元数据中携带
        ``bufferTruncated=true``，截断数据不以"成功交付"掩盖。
        """
        buf_key = task_id

        def _fail(err_msg: str):
            with self._lock:
                task = self._tasks.get(task_id)
                if task is not None:
                    task["capturing"] = False
                    task["buf_key"] = None
                    task["progress"] = 0
                    task["error"] = err_msg
                self._device_pt_counts.pop(buf_key, None)
            if self.on_batch_failed:
                try:
                    self.on_batch_failed(task_id, err_msg, list(device_ids))
                except Exception:
                    logger.log_event(
                        "driver.mid360.batch_failed_callback.failed",
                        "MID-360 批次失败回调执行失败",
                        level="ERROR",
                        context={
                            "taskId": task_id,
                            "stage": "publish",
                            "status": "failed",
                            **_device_log_context(device_ids),
                        },
                        exc_info=True,
                    )

        if not bufs:
            err_msg = "未收到点云数据"
            logger.error("任务 %s：未收到点云数据 "
                         "(请检查: 1.防火墙是否放行 UDP %d 端口 "
                         "2.设备推流目标是否为 %s:%d "
                         "3.PCL 接收线程 alive=%s)",
                         task_id, self.pcl_port,
                         self.host_ip, self.pcl_port,
                         self._recv_thread_alive)
            _fail(err_msg)
            return

        all_pts = np.vstack(bufs)  # shape (N, 7): x y z intensity devId batch tag
        logger.info("任务 %s：已采集 %d 个点",
                    task_id, all_pts.shape[0])
        all_pts = _filter_finite_point_rows(
            all_pts,
            task_id=task_id,
            label="capture raw",
        )

        # 保证列数 >= 7（历史测试缓冲缺 tag 时补 0）
        if all_pts.shape[1] < 7:
            pad = np.zeros((all_pts.shape[0], 7 - all_pts.shape[1]), dtype=np.float32)
            all_pts = np.hstack([all_pts, pad])

        # 零点过滤
        if all_pts.shape[0] > 0:
            mask = ~np.all(all_pts[:, :3] == 0, axis=1)
            if not mask.all():
                all_pts = all_pts[mask]
                logger.info("任务 %s：零点过滤后剩余 %d 个点",
                            task_id, all_pts.shape[0])

        if all_pts.shape[0] == 0:
            err_msg = "过滤后无有效点云"
            logger.error("任务 %s：过滤后无有效点云", task_id)
            _fail(err_msg)
            return

        pcd_n = all_pts.shape[0]

        # 计算每设备点数
        capture_dpc = {}
        dev_col = all_pts[:, 4].astype(np.int32)
        uids, ucounts = np.unique(dev_col, return_counts=True)
        capture_dpc = {str(int(u)): int(c) for u, c in zip(uids, ucounts)}

        # 更新任务（采集成功）；任务已被 drop_task 移除（取消竞态）时跳过交付
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                logger.info("任务 %s：交付前已被移除，跳过交付", task_id)
                return
            task["capturing"] = False
            task["buf_key"] = None
            task["progress"] = 100
            task["error"] = None
            self._device_pt_counts.pop(buf_key, None)

        # 只读化后交付：runtime 经 from_owned 零拷贝接管，驱动不再引用
        all_pts.setflags(write=False)

        if self.on_batch_collected:
            try:
                self.on_batch_collected(task_id, {
                    "deviceIds": device_ids,
                    "duration": duration, "devicePointCounts": capture_dpc,
                    "pointCount": pcd_n, "points": all_pts,
                    "bufferTruncated": bool(buffer_full),
                })
            except Exception:
                logger.log_event(
                    "driver.mid360.batch_collected_callback.failed",
                    "MID-360 批次采集完成回调执行失败",
                    level="ERROR",
                    context={
                        "taskId": task_id,
                        "stage": "publish",
                        "status": "failed",
                        **_device_log_context(device_ids),
                    },
                    exc_info=True,
                )

    def _batch_abandoned(self, task_id: str) -> bool:
        """批次是否已被放弃（取消 / 静默停止 / 调用方丢弃任务）。

        等待窗口与各收尾点共用同一判据，避免"取消后仍按失败路径走"的语义分裂。
        判据只读既有状态，不引入新记账：

        * 任务记录已消失 —— 调用方 ``drop_task``（runtime 取消后立刻调用，会一并
          清掉静默停止标志，因此不能只看标志）；
        * ``capturing`` 已为 False —— ``stop_current_batch`` 的静默停止；
        * 静默停止标志或当前取消事件已置位。
        """
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return True
            if not task.get("capturing"):
                return True
            if task_id in self._silent_stop_tasks:
                return True
            cancel_event = self._task_cancel_events.get(task_id)
        return cancel_event is not None and cancel_event.is_set()

    def stop_current_batch(self, task_id: str) -> None:
        """停止当前正在进行的采集：中断采集线程并清空当前采集缓存，
        但保留设备工作状态。

        * 不调用 ``enter_standby``：设备保持当前工作模式，可立即接受下一次采集。
        * 不触发错误回调（silent_stop 语义，取消由 ``on_batch_cancelled`` 通知）。
        """

        cancel_ev = self._task_cancel_events.get(task_id)
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return
            self._silent_stop_tasks.add(task_id)
            buf_key = task.get("buf_key")
            if buf_key:
                self._point_buffers.pop(buf_key, None)
                self._buffer_pt_totals.pop(buf_key, None)
                self._device_pt_counts.pop(buf_key, None)
                task["buf_key"] = None
            task["capturing"] = False
            task["progress"] = 0
            task["error"] = None
            # 重置取消事件，让下一次采集能重新使用
            self._task_cancel_events[task_id] = threading.Event()
        if cancel_ev:
            cancel_ev.set()
        logger.info("任务 %s：当前采集已静默停止（设备保持活动）", task_id)

    def drop_task(self, task_id: str) -> None:
        """从驱动内存中彻底移除任务记录。"""

        with self._lock:
            self._tasks.pop(task_id, None)
            self._task_cancel_events.pop(task_id, None)
            self._silent_stop_tasks.discard(task_id)

    def get_device_status(self, device_id: int) -> Dict[str, Any]:
        with self._lock:
            dev = self._devices.get(device_id)
            if not dev:
                return {"status": "unknown", "deviceId": device_id}
            last_seen = self._last_state_seen.get(dev.ip)
            return {"deviceId": device_id, "ip": dev.ip,
                    "status": dev.status,
                    "lastSeen": last_seen,
                    "temperature": dev.temperature}

    def list_devices(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [{"deviceId": did, "ip": dev.ip, "sn": dev.sn,
                     "port": dev.ctrl_port,
                     "status": dev.status,
                     "lastSeen": self._last_state_seen.get(dev.ip),
                     "temperature": dev.temperature}
                    for did, dev in self._devices.items()]

    def set_device_work_mode(self, device_id: int, mode: str,
                             *, timeout_sec: float | None = None,
                             retries: int | None = None) -> bool:
        """切换设备工作模式（standby / ready）。SAMPLING 通过 start_batch() 触发。

        ``timeout_sec``/``retries`` 覆盖命令级超时（RT 快速失败清扫用
        0.5s × 1 次）；缺省沿用模块默认（2s × 2）。
        """
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return False
        if mode == "standby":
            return dev.enter_standby(timeout=timeout_sec, retries=retries)
        elif mode == "ready":
            return dev.enter_ready(timeout=timeout_sec, retries=retries)
        return False

    def _revert_device_mode(
        self,
        dev: Mid360Device,
        prev_statuses: Dict[int, Optional[str]],
        *,
        expected_epoch: Optional[int] = None,
    ) -> None:
        """采集失败回退：按会话前状态快照恢复工作模式（受命令纪元判据约束）。

        R1（冷热状态上收）后，driver 不再有策略性待机：回退只恢复快照中的
        会话前状态（ready→enter_ready / standby→enter_standby）；快照缺失或
        异常态（offline/scanning/未知）兜底 enter_standby（安全默认，激光必停）。
        失败仅记 ``driver.mid360.device.mode.revert.failed`` WARNING，不抛出。

        ``expected_epoch`` 是本批次拥有的命令纪元（其 SAMPLING 之后的值）。纪元
        不等 = 设备已被其它批次/会话重新指挥过（例如会话 teardown 的 READY、
        手工 API、平台关闭清扫），此时**跳过回退**并记
        ``driver.mid360.device.mode.revert.skipped``：旧批次的清理不得改写别人的
        设备模式（真机证据：取消后的孤儿批次在 28.7s 后把平台刚置好的 ready 改回
        standby，撤销了下一个任务已下发的 SAMPLING）。调用方未提供纪元（历史路径/
        直接调用）时保持原有语义。
        """
        if expected_epoch is not None and dev.mode_epoch != expected_epoch:
            logger.log_event(
                "driver.mid360.device.mode.revert.skipped",
                "MID-360 设备模式回退被跳过（设备已被其它批次或会话接管）",
                level="WARNING",
                context={
                    "deviceId": dev.device_id,
                    "stage": "cleanup",
                    "status": "skipped",
                    "expectedEpoch": expected_epoch,
                    "actualEpoch": dev.mode_epoch,
                },
            )
            return
        prev = prev_statuses.get(dev.device_id)
        try:
            if prev == "ready":
                dev.enter_ready()
            else:
                dev.enter_standby()
        except Exception:
            logger.log_event(
                "driver.mid360.device.mode.revert.failed",
                "MID-360 设备回退工作模式失败",
                level="WARNING",
                context={"deviceId": dev.device_id, "stage": "cleanup", "status": "degraded"},
            )

    def set_device_ip_config(self, device_id: int, ip: str, subnet: str, gateway: str) -> bool:
        """配置指定设备的 IP 地址、子网掩码和网关。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return False
        return dev.set_lidar_ip(ip, subnet, gateway)

    def get_device_ip_config(self, device_id: int) -> Optional[Dict[str, str]]:
        """查询指定设备的 IP 配置。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return None
        return dev.query_lidar_ip()

    def reboot_device(self, device_id: int, delay_ms: int = 2000) -> bool:
        """向指定设备发送重启命令。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return False
        return dev.reboot(delay_ms)

    def set_device_fov_enable(self, device_id: int, enable: int) -> bool:
        """设置指定设备的 FOV 使能字节（Bit0=cfg0, Bit1=cfg1，0-3）。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return False
        return dev.set_fov_enable_byte(enable)

    def set_device_fov(self, device_id: int, profile: int,
                       yaw_start: float, yaw_stop: float,
                       pitch_start: float, pitch_stop: float,
                       enable: bool) -> bool:
        """设置指定设备的 FOV 配置文件（0 或 1）。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return False
        return dev.set_fov_config(profile, yaw_start, yaw_stop, pitch_start, pitch_stop, enable)

    def get_device_fov(self, device_id: int, profile: int) -> Optional[Dict[str, Any]]:
        """查询指定设备的 FOV 配置文件（0 或 1）。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return None
        return dev.query_fov_config(profile)

    def set_device_pattern_mode(self, device_id: int, mode: int) -> bool:
        """设置指定设备的扫描模式（0=非重复, 1=重复, 2=低帧率重复）。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return False
        return dev.set_pattern_mode(mode)

    def get_device_pattern_mode(self, device_id: int) -> Optional[int]:
        """查询指定设备的当前扫描模式。"""
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is None:
            return None
        return dev.query_pattern_mode()

    def configure_device_parameters(
        self,
        device_id: int,
        *,
        detect_mode: str,
        tag_filter: Dict[str, bool],
    ) -> bool:
        if detect_mode not in {"normal", "sensitive"}:
            return False
        with self._lock:
            dev = self._devices.get(device_id)
        if dev is not None and not dev.set_detect_mode(detect_mode):
            return False
        with self._lock:
            self._device_parameter_settings[device_id] = {
                "detectMode": detect_mode,
                "tagFilter": dict(tag_filter),
            }
        return True

    def get_device_parameter_capabilities(self, device_id: int) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            if device_id not in self._devices:
                return {}
        unavailable = {"status": "unavailable", "value": None}
        return {
            # 推流握手强制配置为 Cartesian 32-bit，因此可靠投影。
            "pointCloudFormat": {"status": "available", "value": "cartesian32"},
            "imuEnabled": dict(unavailable),
            "lidarSpeed": dict(unavailable),
            "timeFilter": dict(unavailable),
            "imuSensorConfig": dict(unavailable),
        }

    def get_device_tag_filter(self, device_id: int) -> Dict[str, bool]:
        with self._lock:
            settings = self._device_parameter_settings.get(int(device_id), {})
            policy = settings.get("tagFilter")
            if not isinstance(policy, dict):
                return dict(DEFAULT_LIVOX_TAG_FILTER)
            return {
                key: bool(policy.get(key, default))
                for key, default in DEFAULT_LIVOX_TAG_FILTER.items()
            }

    # ── 运行时切换网卡 ───────────────────────────────────────

    _SWITCH_PROBE_TIMEOUT = 1.0   # 切换路径探测/重配的单次命令超时（秒）
    _SWITCH_PROBE_RETRIES = 1     # 切换路径探测/重配的重试次数

    def switch_host_ip(self, new_ip: str) -> Dict[str, Any]:
        """运行时切换主机网卡（同步收口，ADR-0018）：

        1. 更新本机 host_ip（所有 socket 均为通配符绑定，无需重绑）；
        2. 并行探测既有设备连通性（query_state，短超时）；
        3. 探测成功者同步重配推流目标并等 ACK，状态变化经回调上报；
        4. 探测/重配失败者进入 ``_reconfig_retry``，由心跳循环周期重试，
           恢复后状态自然回流（SSE 设备事件）。

        返回 ``{"ip": new_ip, "reconfigured": [...], "unreachable": [...]}``。目标 IP 的
        本机归属校验由服务层（``DeviceOperationService.switch_nic``）完成。
        """
        logger.info("MID360Driver：正在切换 host_ip %s → %s", self.host_ip, new_ip)
        self.host_ip = new_ip

        with self._lock:
            # 丢弃旧网卡路径的观测时间，必须发生在探测前；否则成功探测写入的
            # 新鲜时间会被本次切换末尾的 clear 抹掉（ADR-0018）。
            self._last_state_seen.clear()
            devs = list(self._devices.items())

        reconfigured: List[int] = []
        unreachable: List[int] = []

        if devs:
            from concurrent.futures import ThreadPoolExecutor

            def _verify_one(item: tuple) -> bool:
                did, dev = item
                return self._verify_and_reconfigure(did, dev)

            with ThreadPoolExecutor(max_workers=len(devs)) as pool:
                futures = {pool.submit(_verify_one, (did, dev)): did for did, dev in devs}
                for fut in futures:
                    did = futures[fut]
                    if fut.result():
                        reconfigured.append(did)
                    else:
                        unreachable.append(did)

        with self._lock:
            self._reconfig_retry = set(unreachable)

        logger.info(
            "MID360Driver：host_ip 已切换到 %s（reconfigured=%s, unreachable=%s）",
            new_ip, reconfigured, unreachable,
        )
        return {"ip": new_ip, "reconfigured": reconfigured, "unreachable": unreachable}

    def _verify_and_reconfigure(self, device_id: int, dev: "Mid360Device") -> bool:
        """探测单台设备在新网卡路径下的连通性，成功则重配推流目标。

        探测成功会更新 ``dev.status``（与状态推送同词汇）；状态实际变化时
        经 ``_on_device_status_change`` 上报，驱动侧不再区分推送/探测来源。
        """
        dev.host_ip = self.host_ip
        with self._lock:
            prev_status = dev.status
        try:
            probed = dev.query_state(
                timeout=self._SWITCH_PROBE_TIMEOUT,
                retries=self._SWITCH_PROBE_RETRIES,
            )
        except OSError:
            probed = False
        if not probed:
            return False
        with self._lock:
            new_status = dev.status
        if new_status != prev_status:
            self._on_device_status_change(device_id, new_status)
        try:
            ok = dev.configure_push_destinations(
                timeout=self._SWITCH_PROBE_TIMEOUT,
                retries=self._SWITCH_PROBE_RETRIES,
            )
        except OSError:
            ok = False
        if ok:
            with self._lock:
                self._last_state_seen[dev.ip] = time.time()
        return ok

    # ── 关闭 ─────────────────────────────────────────────────

    def shutdown(self):
        self._running = False
        # R1（冷热状态上收）：关闭前的设备冷却由 runtime 状态调度器
        # （cooldown_all 快速失败清扫）在 driver.shutdown 之前执行；
        # 此处只做握手取消、线程回收与 socket 关闭。
        self._cancel_handshake_workers(
            timeout_sec=HANDSHAKE_SHUTDOWN_TIMEOUT_SEC,
        )
        # 等待后台线程退出
        for th in [self._pcl_thread, self._state_thread, self._hb_thread]:
            if th is not None and th.is_alive():
                th.join(timeout=2)
        try:
            self._pcl_sock.close()
        except Exception:
            logger.log_event(
                "driver.mid360.socket.close.failed",
                "MID-360 点云套接字关闭失败",
                level="WARNING",
                context={"stage": "shutdown", "status": "degraded"},
            )
        try:
            self._state_sock.close()
        except Exception:
            logger.log_event(
                "driver.mid360.socket.close.failed",
                "MID-360 状态套接字关闭失败",
                level="WARNING",
                context={"stage": "shutdown", "status": "degraded"},
            )
        try:
            self._bcast_sock.close()
        except Exception:
            logger.log_event(
                "driver.mid360.socket.close.failed",
                "MID-360 发现套接字关闭失败",
                level="WARNING",
                context={"stage": "shutdown", "status": "degraded"},
            )
        with self._lock:
            for dev in self._devices.values():
                dev.close()
        logger.info("MID360LidarDriver 已关闭")
