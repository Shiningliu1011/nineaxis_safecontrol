"""路径跟踪评价指标——纯逻辑，无 ROS / 无 I/O。

累积 path_tracking_step 的逐步输出，生成跟踪质量综合报告。
报告包含位置/姿态/横向误差统计、进给率、QP 成功率、障碍物裕度、
完成度与综合评分（0-1）。

用法：
    evaluator = TrackingEvaluator(trajectory_duration_s=30.0)
    for step in steps:
        evaluator.update(step)  # step = JaxPathTrackingResult 或 dict
    report = evaluator.report()
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class TrackingStepData:
    """单步跟踪数据（从 JaxPathTrackingResult 或 dict 提取）。"""

    pos_error_m: float
    orient_error_rad: float
    cross_track_m: float
    feedrate_m_s: float
    qp_ok: bool
    min_obs_dist: float
    delta_slack: float
    source_time_s: float
    at_endpoint: bool
    limiting_reason_code: int


def step_from_result(result: Any) -> TrackingStepData:
    """从 JaxPathTrackingResult 或 dict 构造 TrackingStepData。"""
    if isinstance(result, dict):
        err = np.asarray(result.get("err_6d", np.zeros(6)), dtype=float)
        return TrackingStepData(
            pos_error_m=float(np.linalg.norm(err[:3])),
            orient_error_rad=float(np.linalg.norm(err[3:])),
            cross_track_m=float(result.get("cross_track_error_m", 0.0)),
            feedrate_m_s=float(result.get("feedrate_m_s", 0.0)),
            qp_ok=bool(result.get("qp_ok", True)),
            min_obs_dist=float(result.get("min_obs_dist", float("inf"))),
            delta_slack=float(result.get("delta_slack", 0.0)),
            source_time_s=float(result.get("reference_source_time_s", 0.0)),
            at_endpoint=bool(result.get("reference_at_endpoint", False)),
            limiting_reason_code=int(result.get("limiting_reason_code", 0)),
        )
    # JaxPathTrackingResult (dataclass)
    err = np.asarray(result.err_6d, dtype=float)
    return TrackingStepData(
        pos_error_m=float(np.linalg.norm(err[:3])),
        orient_error_rad=float(np.linalg.norm(err[3:])),
        cross_track_m=float(result.cross_track_error_m),
        feedrate_m_s=float(result.feedrate_m_s),
        qp_ok=bool(result.qp_ok),
        min_obs_dist=float(result.min_obs_dist),
        delta_slack=float(result.delta_slack),
        source_time_s=float(result.reference_source_time_s),
        at_endpoint=bool(result.reference_at_endpoint),
        limiting_reason_code=int(result.limiting_reason_code),
    )


@dataclass(frozen=True)
class TrackingReport:
    """跟踪质量综合报告。"""

    # 步数与完成度
    total_steps: int
    completed: bool
    completion_fraction: float  # 弧长完成比例 (0-1)

    # 位置误差 (m)
    max_pos_error_m: float
    mean_pos_error_m: float
    p95_pos_error_m: float

    # 姿态误差 (rad)
    max_orient_error_rad: float
    mean_orient_error_rad: float
    p95_orient_error_rad: float

    # 横向误差 (m)
    max_cross_track_m: float
    mean_cross_track_m: float
    p95_cross_track_m: float

    # 进给率 (m/s)
    mean_feedrate_m_s: float
    max_feedrate_m_s: float
    min_feedrate_m_s: float

    # QP 成功率
    qp_success_rate: float  # 0-1
    qp_fail_count: int

    # 障碍物裕度
    min_obstacle_distance_m: float
    mean_obstacle_distance_m: float

    # CBF 松弛
    max_delta_slack: float
    mean_delta_slack: float

    # 时间
    wall_time_s: float
    trajectory_duration_s: float

    # 综合评分 (0-1)
    tracking_score: float

    # 限制因素统计
    limiting_reason_counts: dict[int, int]

    def summary(self) -> str:
        """人类可读的一行摘要。"""
        return (
            f"score={self.tracking_score:.3f} "
            f"pos={self.mean_pos_error_m*1000:.2f}/{self.max_pos_error_m*1000:.2f}mm "
            f"cross={self.mean_cross_track_m*1000:.2f}/{self.max_cross_track_m*1000:.2f}mm "
            f"qp={self.qp_success_rate*100:.1f}% "
            f"obs={self.min_obstacle_distance_m*1000:.1f}mm "
            f"rate={self.mean_feedrate_m_s:.4f}m/s "
            f"done={self.completion_fraction*100:.1f}%"
        )

    def markdown(self) -> str:
        """Markdown 格式的完整报告。"""
        lines = [
            "# 路径跟踪评价报告",
            "",
            "## 概要",
            f"- 综合评分: **{self.tracking_score:.3f}** / 1.000",
            f"- 总步数: {self.total_steps}",
            f"- 完成度: {self.completion_fraction*100:.1f}%",
            f"- 耗时: {self.wall_time_s:.1f}s / {self.trajectory_duration_s:.1f}s",
            f"- QP 成功率: {self.qp_success_rate*100:.1f}% ({self.qp_fail_count} 次失败)",
            "",
            "## 误差统计",
            "",
            "| 指标 | 均值 | 最大值 | P95 |",
            "|------|------|--------|-----|",
            f"| 位置误差 (mm) | {self.mean_pos_error_m*1000:.3f} | {self.max_pos_error_m*1000:.3f} | {self.p95_pos_error_m*1000:.3f} |",
            f"| 姿态误差 (deg) | {math.degrees(self.mean_orient_error_rad):.4f} | {math.degrees(self.max_orient_error_rad):.4f} | {math.degrees(self.p95_orient_error_rad):.4f} |",
            f"| 横向误差 (mm) | {self.mean_cross_track_m*1000:.3f} | {self.max_cross_track_m*1000:.3f} | {self.p95_cross_track_m*1000:.3f} |",
            "",
            "## 进给率",
            f"- 均值: {self.mean_feedrate_m_s:.4f} m/s",
            f"- 最大: {self.max_feedrate_m_s:.4f} m/s",
            f"- 最小: {self.min_feedrate_m_s:.4f} m/s",
            "",
            "## 障碍物裕度",
            f"- 最小距离: {self.min_obstacle_distance_m*1000:.1f} mm",
            f"- 均值距离: {self.mean_obstacle_distance_m*1000:.1f} mm",
            f"- CBF 最大松弛: {self.max_delta_slack:.2e}",
            "",
        ]
        if self.limiting_reason_counts:
            lines.append("## 限制因素分布")
            lines.append("")
            lines.append("| 代码 | 次数 |")
            lines.append("|------|------|")
            for code, count in sorted(self.limiting_reason_counts.items()):
                lines.append(f"| {code} | {count} |")
            lines.append("")
        return "\n".join(lines)


class TrackingEvaluator:
    """累积跟踪指标并生成综合报告。"""

    def __init__(self, trajectory_duration_s: float = 30.0) -> None:
        self.trajectory_duration_s = trajectory_duration_s
        self._pos_errors: list[float] = []
        self._orient_errors: list[float] = []
        self._cross_tracks: list[float] = []
        self._feedrates: list[float] = []
        self._qp_ok_count = 0
        self._qp_fail_count = 0
        self._min_obs_dists: list[float] = []
        self._delta_slacks: list[float] = []
        self._source_times: list[float] = []
        self._at_endpoint = False
        self._limiting_reasons: dict[int, int] = {}
        self._start_time: float | None = None
        self._end_time: float | None = None

    def update(self, step: Any, *, wall_time_s: float | None = None) -> None:
        """累积一步数据。step 可以是 JaxPathTrackingResult 或 dict。"""
        data = step_from_result(step)
        self._pos_errors.append(data.pos_error_m)
        self._orient_errors.append(data.orient_error_rad)
        self._cross_tracks.append(data.cross_track_m)
        self._feedrates.append(data.feedrate_m_s)
        if data.qp_ok:
            self._qp_ok_count += 1
        else:
            self._qp_fail_count += 1
        self._min_obs_dists.append(data.min_obs_dist)
        self._delta_slacks.append(data.delta_slack)
        self._source_times.append(data.source_time_s)
        if data.at_endpoint:
            self._at_endpoint = True
        code = data.limiting_reason_code
        self._limiting_reasons[code] = self._limiting_reasons.get(code, 0) + 1
        if wall_time_s is not None:
            if self._start_time is None:
                self._start_time = wall_time_s
            self._end_time = wall_time_s

    def update_from_controller_result(self, result: dict) -> None:
        """从 oscbf_controller.step_once() 的返回值累积。"""
        self.update(result)

    @property
    def step_count(self) -> int:
        return len(self._pos_errors)

    def report(self) -> TrackingReport:
        """生成综合报告。"""
        n = self.step_count
        if n == 0:
            return TrackingReport(
                total_steps=0, completed=False, completion_fraction=0.0,
                max_pos_error_m=0.0, mean_pos_error_m=0.0, p95_pos_error_m=0.0,
                max_orient_error_rad=0.0, mean_orient_error_rad=0.0,
                p95_orient_error_rad=0.0,
                max_cross_track_m=0.0, mean_cross_track_m=0.0,
                p95_cross_track_m=0.0,
                mean_feedrate_m_s=0.0, max_feedrate_m_s=0.0, min_feedrate_m_s=0.0,
                qp_success_rate=0.0, qp_fail_count=0,
                min_obstacle_distance_m=0.0, mean_obstacle_distance_m=0.0,
                max_delta_slack=0.0, mean_delta_slack=0.0,
                wall_time_s=0.0, trajectory_duration_s=self.trajectory_duration_s,
                tracking_score=0.0, limiting_reason_counts={},
            )

        pos = np.array(self._pos_errors)
        orient = np.array(self._orient_errors)
        cross = np.array(self._cross_tracks)
        feed = np.array(self._feedrates)
        obs = np.array(self._min_obs_dists)
        slack = np.array(self._delta_slacks)

        # 完成度：最后一个 source_time / trajectory_duration
        completion = min(self._source_times[-1] / max(self.trajectory_duration_s, 1.0), 1.0) \
            if self._source_times else 0.0

        # 耗时
        wall = (self._end_time - self._start_time) if (
            self._start_time is not None and self._end_time is not None) else 0.0

        # 综合评分 (0-1)：加权组合
        # 权重：位置精度 0.30 + 横向精度 0.20 + QP 成功率 0.20 + 完成度 0.15 + 障碍物裕度 0.15
        pos_score = max(0.0, 1.0 - float(np.mean(pos)) / 0.005)  # 5mm 满分阈值
        cross_score = max(0.0, 1.0 - float(np.mean(cross)) / 0.003)  # 3mm 满分阈值
        qp_score = self._qp_ok_count / max(n, 1)
        completion_score = completion
        obs_score = max(0.0, 1.0 - max(0.0, 0.02 - float(np.min(obs))) / 0.02) \
            if np.isfinite(obs).any() else 1.0  # 20mm 安全阈值

        score = (
            0.30 * pos_score
            + 0.20 * cross_score
            + 0.20 * qp_score
            + 0.15 * completion_score
            + 0.15 * obs_score
        )

        return TrackingReport(
            total_steps=n,
            completed=self._at_endpoint,
            completion_fraction=float(completion),
            max_pos_error_m=float(np.max(pos)),
            mean_pos_error_m=float(np.mean(pos)),
            p95_pos_error_m=float(np.percentile(pos, 95)),
            max_orient_error_rad=float(np.max(orient)),
            mean_orient_error_rad=float(np.mean(orient)),
            p95_orient_error_rad=float(np.percentile(orient, 95)),
            max_cross_track_m=float(np.max(cross)),
            mean_cross_track_m=float(np.mean(cross)),
            p95_cross_track_m=float(np.percentile(cross, 95)),
            mean_feedrate_m_s=float(np.mean(feed)),
            max_feedrate_m_s=float(np.max(feed)),
            min_feedrate_m_s=float(np.min(feed)),
            qp_success_rate=float(qp_score),
            qp_fail_count=self._qp_fail_count,
            min_obstacle_distance_m=float(np.min(obs[np.isfinite(obs)]))
            if np.isfinite(obs).any() else float("inf"),
            mean_obstacle_distance_m=float(np.mean(obs[np.isfinite(obs)]))
            if np.isfinite(obs).any() else float("inf"),
            max_delta_slack=float(np.max(slack)),
            mean_delta_slack=float(np.mean(slack)),
            wall_time_s=float(wall),
            trajectory_duration_s=self.trajectory_duration_s,
            tracking_score=float(score),
            limiting_reason_counts=dict(self._limiting_reasons),
        )
