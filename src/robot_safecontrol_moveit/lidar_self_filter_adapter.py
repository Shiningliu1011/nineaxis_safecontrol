from __future__ import annotations

from copy import deepcopy

import numpy as np
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from work.lidar_occupancy import LidarOccupancyScene
from work.lidar_self_filter import (
    ClockMapping,
    FilterCalibration,
    JointStateHistory,
    MeshRaySelfFilter,
    SelfFilterResult,
    TimedLidarScan,
)

from robot_safecontrol_moveit.calibration_record import CalibrationRecord
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES


def joint_state_sample(message: JointState) -> tuple[int, np.ndarray]:
    if not isinstance(message, JointState):
        raise TypeError("反馈历史只能接收真实 JointState")
    names = list(message.name)
    if (len(names) != len(message.position) or len(set(names)) != len(names)
            or any(name not in names for name in DEFAULT_JOINT_NAMES)):
        raise ValueError("JointState 关节名称、重复项或 position 数量无效")
    by_name = dict(zip(names, message.position))
    positions = np.array([by_name[name] for name in DEFAULT_JOINT_NAMES], dtype=np.float64)
    if not np.all(np.isfinite(positions)):
        raise ValueError("JointState position 必须有限")
    stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
    return stamp, positions


def pointcloud_samples(message: PointCloud2) -> tuple[np.ndarray, np.ndarray]:
    if not isinstance(message, PointCloud2):
        raise TypeError("LiDAR 输入必须是 PointCloud2")
    names = [field.name for field in message.fields]
    required = ("x", "y", "z", "timestamp")
    if any(names.count(name) != 1 for name in required):
        raise ValueError("PointCloud2 必须保留唯一 XYZ 与逐点 timestamp 字段")
    fields = {field.name: field for field in message.fields}
    if (fields["timestamp"].datatype != PointField.FLOAT64
            or any(fields[name].datatype not in (PointField.FLOAT32, PointField.FLOAT64) for name in required[:3])
            or any(fields[name].count != 1 for name in required)):
        raise ValueError("PointCloud2 XYZ 或 timestamp 类型无效")
    if message.row_step != message.width * message.point_step:
        raise ValueError("当前 sensor_msgs_py 版本要求连续 PointCloud2 行")
    if len(message.data) != message.row_step * message.height:
        raise ValueError("PointCloud2 data 长度与行信息不一致")
    # read_points 在处理不同 byte order 时会修改 buffer，使用独立消息副本。
    points = point_cloud2.read_points(deepcopy(message), field_names=list(required), skip_nans=False)
    raw_stamps = np.array(points["timestamp"], copy=True)
    xyz = np.column_stack([points[name] for name in required[:3]])
    if (not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(raw_stamps))
            or np.any(raw_stamps <= 0) or np.any(np.diff(raw_stamps) < 0)):
        raise ValueError("LiDAR XYZ 和逐点时间必须有限，逐点时间必须为正数且顺序有效")
    return xyz, raw_stamps


def scan_from_pointcloud(
        message: PointCloud2, *, mapping: ClockMapping, scan_id: int,
        received_monotonic_ns: int, max_range_m: float) -> TimedLidarScan:
    if not isinstance(mapping, ClockMapping):
        raise TypeError("LiDAR 需要显式 ClockMapping")
    xyz, raw_stamps = pointcloud_samples(message)
    stamps, errors = mapping.map(raw_stamps)
    return TimedLidarScan(
        xyz, stamps, errors, raw_stamps, np.arange(len(xyz), dtype=np.int64), scan_id,
        mapping.source_clock_id, mapping.identity, message.header.frame_id, received_monotonic_ns, max_range_m,
        mapping.run_id, mapping.target_clock_id,
    )


def history_from_joint_states(
        messages: list[JointState], *, mapping: ClockMapping, feedback_source: str,
        model_hash: str, zero_calibration_id: str) -> JointStateHistory:
    if not messages or not isinstance(mapping, ClockMapping):
        raise ValueError("反馈历史需要真实 JointState 与显式 ClockMapping")
    times, positions = [], []
    for message in messages:
        stamp, position = joint_state_sample(message)
        positions.append(position)
        times.append(stamp)
    stamps, errors = mapping.map(np.array(times, dtype=np.int64))
    return JointStateHistory(
        stamps, np.array(positions), errors, feedback_source, model_hash, zero_calibration_id, mapping.identity,
        mapping.run_id, mapping.target_clock_id,
    )


def calibration_for_self_filter(record: CalibrationRecord) -> FilterCalibration:
    if not isinstance(record, CalibrationRecord):
        raise TypeError("标定需要已加载的 CalibrationRecord")
    source = record.source("lidar")
    if (source is None or source.matrix is None or source.calibration_id is None
            or source.frame_from is None or source.frame_to is None or record.file_sha256 is None):
        raise ValueError("LiDAR 标定记录缺少矩阵或身份")
    return FilterCalibration(
        source.matrix, source.frame_from, source.frame_to,
        record.file_sha256 + ":" + source.calibration_id,
        bool(record.read_ok and source.admission_ready),
    )


def update_scene_from_pointcloud(
        message: PointCloud2, *, mapping: ClockMapping, scan_id: int,
        received_monotonic_ns: int, now_ns: int, max_range_m: float,
        filter_module: MeshRaySelfFilter, history: JointStateHistory,
        producer: LidarOccupancyScene) -> SelfFilterResult:
    try:
        scan = scan_from_pointcloud(message, mapping=mapping, scan_id=scan_id,
                                    received_monotonic_ns=received_monotonic_ns, max_range_m=max_range_m)
        result = filter_module.classify(scan, history)
        producer.update(result.to_ray_batch(), now_ns=now_ns)
    except Exception as error:
        # 第三方解码和整条处理链的异常都撤销旧覆盖，并继续抛出原始异常。
        producer.invalidate(now_ns=now_ns, reason="self_filter_source_error:" + type(error).__name__)
        raise
    return result
