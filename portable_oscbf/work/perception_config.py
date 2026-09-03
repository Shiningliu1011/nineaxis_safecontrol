#!/usr/bin/env python3
"""感知配置加载器: 让 obstacle_params.yaml 的 point_cloud_collision 段变成活配置。

portable_oscbf 是纯 Python (无 ROS)。本模块只做 yaml 读取 → 冻结 dataclass,
桥接节点 (src/robot_safecontrol_moveit/perception_bridge.py) 与仿真 demo 共享
同一份真源, 避免参数在两处漂移。
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Callable, List, NamedTuple, Optional, Tuple

import numpy as np
import yaml

from work.safety_snapshot import SafetyGridSpec

_DEFAULT_YAML = Path(__file__).resolve().parents[1] / "config" / "obstacle_params.yaml"


class ConfigField(NamedTuple):
    """配置字段元数据: 名称、类型构造函数、默认值。"""
    name: str
    type_constructor: Callable
    default: object


@dataclass(frozen=True)
class PointCloudCollisionConfig:
    """point_cloud_collision + workspace 段的解析结果。"""

    enabled: bool
    input_frame: str
    world_frame: str
    source_topic: str
    voxel_size: float
    max_points: int
    point_radius: float
    safety_margin: float
    workspace_min: np.ndarray
    workspace_max: np.ndarray
    # 以下为感知管线补充参数 (可被桥接节点 ROS 参数覆盖)
    sdf_far_distance: float = 10.0
    static_occupancy_frames: int = 8  # 旧帧计数模式 (保留兼容)
    # --- LiDAR + 融合 (issue 02) ---
    source_topic_lidar: str = ""
    input_frame_lidar: str = ""
    source_voxel_camera_m: float = 0.02
    source_voxel_lidar_m: float = 0.03
    fusion_voxel_m: float = 0.03
    max_inter_sensor_dt_s: float = 0.1
    camera_max_age_s: float = 0.5
    lidar_max_age_s: float = 0.5
    occupancy_timeout_s: float = 0.3
    static_confirm_s: float = 0.5
    perception_timeout_s: float = 1.0
    # --- 聚类 ---
    cluster_max_tracks: int = 8
    cluster_min_points: int = 4
    cluster_association_max_dist_m: float = 0.5

    @classmethod
    def config_fields(cls) -> List[ConfigField]:
        """返回所有可从 YAML 加载的字段元数据 (名称、类型、默认值)。

        这是配置字段的唯一权威声明。感知桥接节点用它自动声明/读取 ROS 参数,
        消除 perception_bridge.py 中的重复 declare/get 样板代码。

        排除: enabled (布尔开关), workspace_min/max (ndarray 特殊处理)。
        """
        result: List[ConfigField] = []
        for f in fields(cls):
            if f.name in ("enabled", "workspace_min", "workspace_max"):
                continue  # 这些字段由特殊逻辑处理
            if f.default is not f.default_factory:
                # 有默认值的字段
                t = type(f.default) if f.default is not None else str
                result.append(ConfigField(f.name, t, f.default))
            else:
                # 无默认值: 根据类型注解推断构造函数
                type_map = {"str": str, "float": float, "int": int, "bool": bool}
                type_name = getattr(f.type, "__name__", "str")
                result.append(ConfigField(
                    f.name, type_map.get(type_name, str), None))
        return result


def load_point_cloud_collision(
    config_yaml_path: Optional[Path] = None,
) -> PointCloudCollisionConfig:
    """读取 obstacle_params.yaml 的 point_cloud_collision + workspace 两节。

    参数
    ----
    config_yaml_path:
        缺省时用 portable_oscbf/config/obstacle_params.yaml。

    返回
    ----
    PointCloudCollisionConfig
    """
    path = Path(config_yaml_path) if config_yaml_path is not None else _DEFAULT_YAML
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    section = data.get("point_cloud_collision") or {}
    workspace = data.get("workspace") or {}
    w_min = np.asarray(workspace.get("min", [-0.6, -0.5, 0.4]), dtype=np.float64).reshape(3)
    w_max = np.asarray(workspace.get("max", [0.6, 1.0, 2.0]), dtype=np.float64).reshape(3)
    if np.any(w_max <= w_min):
        raise ValueError(f"workspace.max must exceed workspace.min: {w_min} vs {w_max}")
    return PointCloudCollisionConfig(
        enabled=bool(section.get("enabled", True)),
        input_frame=str(section.get("input_frame", "camera_color_optical_frame")),
        world_frame=str(section.get("world_frame", "camera_color_optical_frame")),
        source_topic=str(
            section.get("source_topic", "/camera/depth_registered/points")),
        voxel_size=float(section.get("voxel_size", 0.03)),
        max_points=int(section.get("max_points", 5000)),
        point_radius=float(section.get("point_radius", 0.01)),
        safety_margin=float(section.get("safety_margin", 0.08)),
        workspace_min=w_min,
        workspace_max=w_max,
        sdf_far_distance=float(section.get("sdf_far_distance", 10.0)),
        static_occupancy_frames=int(section.get("static_occupancy_frames", 8)),
        source_topic_lidar=str(section.get("source_topic_lidar", "")),
        input_frame_lidar=str(section.get("input_frame_lidar", "")),
        source_voxel_camera_m=float(section.get("source_voxel_camera_m", 0.02)),
        source_voxel_lidar_m=float(section.get("source_voxel_lidar_m", 0.03)),
        fusion_voxel_m=float(section.get("fusion_voxel_m", 0.03)),
        max_inter_sensor_dt_s=float(section.get("max_inter_sensor_dt_s", 0.1)),
        camera_max_age_s=float(section.get("camera_max_age_s", 0.5)),
        lidar_max_age_s=float(section.get("lidar_max_age_s", 0.5)),
        occupancy_timeout_s=float(section.get("occupancy_timeout_s", 0.3)),
        static_confirm_s=float(section.get("static_confirm_s", 0.5)),
        perception_timeout_s=float(section.get("perception_timeout_s", 1.0)),
        cluster_max_tracks=int(section.get("cluster_max_tracks", 8)),
        cluster_min_points=int(section.get("cluster_min_points", 4)),
        cluster_association_max_dist_m=float(
            section.get("cluster_association_max_dist_m", 0.5)),
    )


def spec_of(cfg: PointCloudCollisionConfig) -> SafetyGridSpec:
    """由配置构造 SafetyGridSpec (ESDF 栅格几何)。"""
    return SafetyGridSpec(
        workspace_min=cfg.workspace_min,
        workspace_max=cfg.workspace_max,
        voxel_size=cfg.voxel_size,
    )
