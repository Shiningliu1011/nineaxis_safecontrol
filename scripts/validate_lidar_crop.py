import argparse
import hashlib
import json
from pathlib import Path
from time import monotonic, perf_counter

import numpy as np
import rclpy
import rosbag2_py
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from robot_safecontrol_moveit.lidar_crop_experiment import LidarCropExperiment
from robot_safecontrol_moveit.point_cloud_crop import CropRegion, crop_cloud
from robot_safecontrol_moveit.ros_conventions import state_stream_qos


def expected_records(cloud, region):
    points = point_cloud2.read_points(cloud)
    keep = np.ones(len(points), dtype=bool)
    nonzero = np.zeros(len(points), dtype=bool)
    for axis, name in enumerate(("x", "y", "z")):
        value = points[name]
        lower = region.center[axis] - region.size[axis] / 2
        upper = region.center[axis] + region.size[axis] / 2
        keep &= np.isfinite(value) & (value >= lower) & (value <= upper)
        nonzero |= value != 0
    selected = np.flatnonzero(keep & nonzero)
    data = bytes(cloud.data)
    expected = b"".join(data[i * cloud.point_step:(i + 1) * cloud.point_step] for i in selected)
    return selected, expected


def verify(cloud, output, region):
    assert cloud.header.frame_id == "livox_frame", "验证坐标系必须为 livox_frame"
    selected, expected = expected_records(cloud, region)
    assert bytes(output.data) == expected, "点记录与独立筛选结果不同"
    assert output.header == cloud.header, "采样时间或坐标系发生变化"
    assert output.fields == cloud.fields, "字段发生变化"
    assert output.point_step == cloud.point_step
    assert output.width == len(selected)
    assert output.height == 1 and output.row_step == output.width * output.point_step
    return len(selected)


def summarize(rows):
    assert rows, "未取得可验证的真实点云"
    counts = np.array([[r["input"], r["output"]] for r in rows])
    duration = np.array([r["processing_ms"] for r in rows])
    total = counts.sum(axis=0)
    return {
        "frames": len(rows), "input_points": int(total[0]), "output_points": int(total[1]),
        "retained_fraction": float(total[1] / total[0]),
        "output_points_per_frame_min": int(counts[:, 1].min()),
        "output_points_per_frame_max": int(counts[:, 1].max()),
        "crop_function_ms_p50": float(np.percentile(duration, 50)),
        "crop_function_ms_p95": float(np.percentile(duration, 95)),
        "crop_function_ms_max": float(duration.max()),
        "all_records_verified": True,
    }


def evaluate_bag(args, region):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        cloud = deserialize_message(data, PointCloud2)
        original = bytes(cloud.data)
        start = perf_counter()
        output = crop_cloud(cloud, region)
        elapsed = (perf_counter() - start) * 1000
        count = verify(cloud, output, region)
        assert bytes(cloud.data) == original, "原始记录发生变化"
        rows.append({"input": cloud.width * cloud.height, "output": count, "processing_ms": elapsed})
    return {
        "source": str(args.bag.resolve()),
        "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(args.bag.glob("*.db3"))},
        **summarize(rows),
    }


def evaluate_live(args, region):
    rclpy.init()
    probe = Node("lidar_crop_validator", namespace="crop_validation")
    crop = LidarCropExperiment(namespace="crop_validation", parameter_overrides=[
        Parameter("input_topic", value=args.topic),
        Parameter("roi_center", value=list(region.center)),
        Parameter("roi_size", value=list(region.size)),
        Parameter("enabled", value=True),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(crop)
    raw, output, rows = {}, {}, []

    def key(message):
        return message.header.stamp.sec, message.header.stamp.nanosec

    def match():
        for stamp in list(raw.keys() & output.keys()):
            cloud = raw.pop(stamp)
            result = output.pop(stamp)
            count = verify(cloud, result, region)
            start = perf_counter()
            crop_cloud(cloud, region)
            elapsed = (perf_counter() - start) * 1000
            rows.append({"input": cloud.width * cloud.height, "output": count, "processing_ms": elapsed})
        for cache in (raw, output):
            while len(cache) > 100:
                del cache[next(iter(cache))]

    def receive_raw(message):
        raw[key(message)] = message
        match()

    def receive_output(message):
        output[key(message)] = message
        match()

    probe.create_subscription(PointCloud2, args.topic, receive_raw, state_stream_qos())
    probe.create_subscription(PointCloud2, "lidar_crop/points", receive_output, state_stream_qos())
    start = monotonic()
    try:
        while monotonic() - start < args.seconds:
            executor.spin_once(timeout_sec=0.05)
        report = {"source": args.topic, "duration_s": monotonic() - start, **summarize(rows)}
        assert report["frames"] >= args.seconds * 5, "实时验证帧率不足 5 Hz"
        return report
    finally:
        executor.shutdown()
        crop.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="验证真实 rosbag 或实时 LiDAR 的区域裁剪")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path)
    source.add_argument("--live", action="store_true")
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--center", nargs=3, type=float, required=True)
    parser.add_argument("--size", nargs=3, type=float, required=True)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("seconds 必须为有限正数")
    region = CropRegion(args.center, args.size)
    result = evaluate_bag(args, region) if args.bag else evaluate_live(args, region)
    result.update(roi_center=region.center, roi_size=region.size, frame_id="livox_frame",
                  scope="坐标区域筛选、字段保存与算法耗时；未验证机械臂覆盖或碰撞安全")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
