from __future__ import annotations

import hashlib


# 数值仅用于解析软件接口测试，适用范围与生产设备分离。
DEVICE = "analytic-interface-test"
SCENARIO = "json-and-identity-validation"
DEVELOPMENT_DATA = b"analytic development domain: scalar, count, time, link pair"
VALIDATION_DATA = b"independent validation domain: serialization, identity, immutable values"


def parameter_payload() -> dict:
    source_hash = hashlib.sha256(DEVELOPMENT_DATA).hexdigest()
    validation_hash = hashlib.sha256(VALIDATION_DATA).hexdigest()

    def record(value: int | float, unit: str) -> dict:
        return {
            "value": value,
            "unit": unit,
            "source": {"description": "解析接口输入", "data_hashes": [source_hash]},
            "measurement_method": "按明确单位构造解析接口输入",
            "calculation_method": "精确复制输入值",
            "devices": [DEVICE],
            "scenarios": [SCENARIO],
            "confidence_requirement": "序列化往返精确相等",
            "validation": {
                "data_hashes": [validation_hash],
                "method": "独立检查 JSON 往返的值与类型",
                "result": "passed",
                "evidence": "portable_oscbf/tests/test_collision_parameters.py",
            },
        }

    values = {
        "lidar_range_error_mm": (1.0, "mm"),
        "lidar_angular_error_rad": (0.001, "rad"),
        "lidar_incidence_error_mm": (1.0, "mm"),
        "extrinsic_translation_error_mm": (1.0, "mm"),
        "extrinsic_rotation_error_rad": (0.001, "rad"),
        "point_timestamp_error_ns": (100, "ns"),
        "transport_delay_ns": (60_000_000_000, "ns"),
        "scene_preparation_delay_ns": (60_000_000_000, "ns"),
        "control_delay_ns": (60_000_000_000, "ns"),
        "joint_interpolation_prismatic_error_mm": (1.0, "mm"),
        "joint_interpolation_revolute_error_rad": (0.001, "rad"),
        "voxel_size_mm": (1.0, "mm"),
        "support_radius_limit_mm": (100.0, "mm"),
        "support_merge_radius_limit_mm": (50.0, "mm"),
        "support_merge_max_points": (3, "1"),
        "untracked_speed_mm_s": (1.0, "mm/s"),
        "untracked_acceleration_mm_s2": (1.0, "mm/s^2"),
        "command_position_error_mm": (1.0, "mm"),
        "execution_feedback_error_mm": (1.0, "mm"),
        "stopping_distance_mm": (1.0, "mm"),
        "environment_clearance_mm": (30.0, "mm"),
        "self_collision_cbf_rate_s_inv": (1.0, "s^-1"),
        "environment_cbf_rate_s_inv": (1.0, "s^-1"),
        "max_support_points": (3, "1"),
        "max_tracks": (3, "1"),
        "max_environment_pairs_per_ellipsoid": (2, "1"),
        "max_ellipsoids_per_link": (4, "1"),
        "max_primitive_rows": (4, "1"),
        "query_batch_size": (2, "1"),
        "segment_batch_size": (2, "1"),
        "dcol_max_iterations": (20, "1"),
        "dcol_primal_residual_limit": (0.001, "1"),
        "dcol_dual_residual_limit": (0.001, "1"),
        "qp_max_iterations": (20, "1"),
        "qp_residual_limit": (0.001, "1"),
        "distance_max_iterations": (20, "1"),
        "distance_tolerance_mm": (0.001, "mm"),
        "bisection_max_depth": (8, "1"),
        "scene_preparation_deadline_ns": (60_000_000_000, "ns"),
        "device_transfer_deadline_ns": (60_000_000_000, "ns"),
        "query_deadline_ns": (9_000_000, "ns"),
        "qp_deadline_ns": (500_000, "ns"),
        "control_deadline_ns": (10_000_000, "ns"),
        "certify_deadline_ns": (60_000_000_000, "ns"),
        "trajectory_validation_deadline_ns": (60_000_000_000, "ns"),
        "replanning_deadline_ns": (60_000_000_000, "ns"),
    }
    return {
        "schema_version": 1,
        "generator": {"name": "analytic-interface-inputs", "version": "1", "method": "精确数值输入"},
        "input_data_hashes": [source_hash, validation_hash],
        "parameters": {name: record(value, unit) for name, (value, unit) in values.items()},
        "self_clearance_mm": [
            {"links": ["base_link", "Link1"], "clearance": record(1.0, "mm")},
            {"links": ["Link1", "Link2"], "clearance": record(2.0, "mm")},
        ],
    }
