#!/usr/bin/env python3
"""感知配置加载器: 让 obstacle_params.yaml 的 point_cloud_collision 段变成活配置。

portable_oscbf 是纯 Python (无 ROS)。本模块只做 yaml 读取 → 冻结 dataclass,
桥接节点 (src/robot_safecontrol_moveit/perception_bridge.py) 与仿真 demo 共享
同一份真源, 避免参数在两处漂移。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

try:
    from safety_snapshot import SafetyGridSpec
except ImportError:  # 包式导入 (from work.perception_config import ...) 时的回退
    from work.safety_snapshot import SafetyGridSpec

_DEFAULT_YAML = Path(__file__).resolve().parents[1] / "config" / "obstacle_params.yaml"


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
    static_occupancy_frames: int = 8
    cluster_max_tracks: int = 8
    cluster_min_points: int = 4
    cluster_association_max_dist_m: float = 0.5


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
