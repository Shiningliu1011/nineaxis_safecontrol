from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from time import perf_counter
from typing import Callable

import numpy as np

from .tracking_contract import EvidenceContext, EvaluationScope
from .tracking_evaluator import TrackingEvaluator, step_from_result
from .tracking_report_writer import TrackingReportWriter, write_tracking_bundle


class TrackingRun:
    """拥有一次跟踪的状态与报告；publish 接收已经平滑的最终命令。

    调用方串行调用状态变更方法，停止调度后调用 close。publish 在命令
    判定之后、终止报告提交之前执行，保持 ROS adapter 的发布顺序。
    """

    def __init__(self, *, loop, geometry, config, trajectory_duration_s: float,
                 evidence: EvidenceContext, log, scope: EvaluationScope | None = None,
                 surface=None):
        self._loop = loop
        self._geometry = geometry
        self._config = dict(config)
        self._trajectory_duration_s = trajectory_duration_s
        self._evidence = evidence
        self._scope = scope or EvaluationScope(geometry.total_length_m)
        self._log = log
        self._surface = deepcopy(surface)
        self._state = "waiting" if config["wait_for_start"] else "tracking"
        self._started = not config["wait_for_start"]
        self._termination = None
        self._path_state = loop.initial_path_state()
        self._latest_q = None
        self._last_result = None
        self._projection_before_m = None
        self._hold_q = None
        self._q_cmd_smooth = None
        self._stall_since = None
        self._src_hist = deque()
        self._last_pos_err = None
        self._last_state0 = None
        self._step_durations = []
        self._qp_fail_count = 0
        self._evaluator = None
        self._writer = TrackingReportWriter()
        self._limits = (np.asarray(loop.robot.joint_lower_limits, dtype=float),
                        np.asarray(loop.robot.joint_upper_limits, dtype=float))

    @property
    def execution_state(self) -> str:
        return self._state

    def start(self) -> str:
        if self._state == "closed":
            raise RuntimeError("tracking run is closed")
        if self._started:
            return "ALREADY_TRACKING"
        self._started = True
        self._state = "tracking"
        self._path_state = self._loop.initial_path_state()
        return "TRACKING_STARTED"

    def receive_state(self, q: np.ndarray) -> None:
        if self._state == "closed":
            raise RuntimeError("tracking run is closed")
        q = np.asarray(q, dtype=float)
        if q.shape != self._limits[0].shape or not np.all(np.isfinite(q)):
            raise ValueError("joint state must have finite positions in configured joint order")
        self._latest_q = q.copy()

    def step_once(self, q: np.ndarray, *, obs_kwargs: dict | None = None):
        """执行控制步；保持直接调用的轨迹推进行为，不增加评价样本。"""
        return deepcopy(self._step_once(q, obs_kwargs=obs_kwargs))

    def _step_once(self, q: np.ndarray, *, obs_kwargs: dict | None = None):
        if self._state == "closed":
            raise RuntimeError("tracking run is closed")
        kwargs = dict(
            q=np.asarray(q, dtype=float), path_state=self._path_state,
            q_des=np.asarray(q, dtype=float),
            **{name: float(self._config[name]) for name in (
                "kp_pos", "kp_orient", "kp_joint", "nullspace_speed_limit", "damping")},
        )
        if obs_kwargs:
            kwargs.update(obs_kwargs)
        result = self._loop.path_tracking_step(**kwargs)
        projection_before, _ = self._geometry.project_local(
            result.ee_pos_before, anchor_segment=int(self._path_state[2]),
            half_window_segments=self._geometry.num_segments,
        ) if self._evaluator is None or self._evaluator.step_count == 0 else (float(self._path_state[1]), 0)
        self._path_state = np.array(result.path_state, dtype=float, copy=True)
        self._projection_before_m = projection_before
        if not result.min_obs_dist_measured:
            result = replace(result, min_obs_dist=None)
        self._last_result = result
        return result

    def tick(self, publish: Callable[[np.ndarray], None], *, obs_kwargs=None) -> None:
        if self._state in ("waiting", "closed") or self._latest_q is None:
            return
        if self._hold_q is not None:
            self._publish(self._hold_q, publish)
            return
        if self._evaluator is None:
            self._evaluator = TrackingEvaluator(
                self._trajectory_duration_s, scope=self._scope, evidence=self._evidence,
                deadline_ms=float(self._config["latency_budget_ms"]),
            )
        start = perf_counter()
        q_now = np.asarray(self._latest_q, dtype=float)
        selected_obs = dict(obs_kwargs) if self._config["enable_perception_obstacles"] and obs_kwargs else None
        record = self._step_once(q_now, obs_kwargs=selected_obs)
        duration_ms = (perf_counter() - start) * 1000.0
        self._step_durations.append(duration_ms)
        self._latest_q = None
        self._evaluator.update_from_step_record(
            record, wall_time_s=start, step_latency_ms=duration_ms,
            projection_before_m=self._projection_before_m,
        )
        lower, upper = self._limits
        pos_err = float(np.linalg.norm(record.err_6d[:3]))
        if self._last_pos_err is not None and abs(pos_err - self._last_pos_err) > 0.25:
            self._log.warning(
                f"POS_JUMP d_err={pos_err - self._last_pos_err:.3f}m "
                f"u_safe=[{', '.join(f'{v:.3f}' for v in record.u_safe)}] "
                f"q_next-qmax={np.max(np.abs(record.q_next - q_now)):.4f} "
                f"ee=[{', '.join(f'{v:.3f}' for v in record.ee_pos)}] "
                f"ref=[{', '.join(f'{v:.3f}' for v in record.reference_position_m)}]"
            )
        self._last_pos_err = pos_err
        if len(self._step_durations) % 300 == 0:
            state0, state1 = float(self._path_state[0]), float(self._path_state[1])
            delta0 = state0 - self._last_state0 if self._last_state0 is not None else float("nan")
            self._last_state0 = state0
            self._log.info(
                f"DETAIL steps={len(self._step_durations)} prog={state0:.6f} "
                f"proj={state1:.6f} lead={state0 - state1:.6f} dprog300={delta0:.6f} "
                f"feed={record.feedrate_m_s:.5f} nom={record.feedrate_nominal_m_s:.5f} "
                f"gamma={record.gamma:.3f} lim={record.limiting_reason_code} "
                f"cap_j={record.feedrate_joint_limit_m_s if record.feedrate_joint_limit_m_s < 1e9 else 9.99:.4f} "
                f"cap_cbf={record.feedrate_cbf_limit_m_s if record.feedrate_cbf_limit_m_s < 1e9 else 9.99:.4f} "
                f"cap_rate={record.feedrate_rate_limit_m_s if record.feedrate_rate_limit_m_s < 1e9 else 9.99:.4f} "
                f"cap_tool={record.feedrate_tool_axis_limit_m_s if record.feedrate_tool_axis_limit_m_s < 1e9 else 9.99:.4f} "
                f"cap_brake={record.feedrate_endpoint_brake_limit_m_s:.4f} "
                f"u_max={float(np.max(np.abs(record.u_safe))):.5f} "
                f"dq_max={float(np.max(np.abs(record.q_next - q_now))):.6f} "
                f"radial={self._radial_error_m(record.ee_pos)*1e3:.2f}mm "
                f"ref_radial={self._radial_error_m(record.reference_position_m)*1e3:.2f}mm "
                f"src={record.reference_source_time_s:.4f}"
            )
        if (not bool(record.reference_at_endpoint)
                and float(record.feedrate_m_s) <= 1e-3
                and float(record.cross_track_error_m) > 5e-3):
            if self._stall_since is None:
                self._stall_since = start
            elif start - self._stall_since > 1.0:
                self._log.warning(
                    "TRACKING_STALLED: reference feedrate=0 with cross-track "
                    f"={float(record.cross_track_error_m)*1e3:.1f}mm for "
                    f"{start - self._stall_since:.2f}s; holding current pose"
                )
                self._hold(q_now, "held", publish)
                return
        else:
            self._stall_since = None
        self._src_hist.append((start, float(record.reference_source_time_s)))
        while self._src_hist and start - self._src_hist[0][0] > 5.0:
            self._src_hist.popleft()
        if (len(self._src_hist) >= 2
                and float(record.feedrate_m_s) < 0.05
                and float(record.reference_source_time_s) - self._src_hist[0][1] < 0.01
                and pos_err > 0.005):
            self._log.warning(
                "TRACKING_STALLED: reference source-time frozen for 5s "
                f"(feed={float(record.feedrate_m_s):.4f}m/s, "
                f"pos_err={pos_err*1e3:.1f}mm); holding current pose"
            )
            self._hold(q_now, "held", publish)
            return
        if bool(record.reference_at_endpoint):
            self._log.info(f"END_OF_TRACKING: holding final pose pos_err={pos_err*1e3:.1f}mm")
            self._hold(q_now, "completed", publish)
            return
        q_next = record.q_next
        if not (np.all(np.isfinite(q_next)) and np.all(q_next >= lower - 1e-9)
                and np.all(q_next <= upper + 1e-9)):
            self._evaluator.record_event("command_rejected")
            self._log.error(f"discarding invalid safe state: {q_next.tolist()}")
            return
        if not record.qp_ok:
            self._evaluator.record_event("qp_failure")
            self._qp_fail_count += 1
            self._log.warning(f"QP failed at step {len(self._step_durations)}; holding current state")
        self._publish(q_next, publish)

    def _publish(self, positions, publish) -> None:
        dt = 1.0 / float(self._config["publish_frequency_hz"])
        alpha = dt / (dt + 0.02)
        target = np.asarray(positions, dtype=float)
        if self._q_cmd_smooth is None:
            self._q_cmd_smooth = target.copy()
        else:
            self._q_cmd_smooth = self._q_cmd_smooth + alpha * (target - self._q_cmd_smooth)
        publish(self._q_cmd_smooth.copy())

    def _radial_error_m(self, ee_pos) -> float:
        if self._surface is None:
            return float("nan")
        axis, centre, radius = self._surface
        rel = np.asarray(ee_pos, dtype=float) - centre
        radial = rel - axis * float(np.dot(rel, axis))
        return float(np.linalg.norm(radial) - radius)

    def _hold(self, q, reason, publish) -> None:
        self._hold_q = np.clip(q, *self._limits)
        self._state = "holding"
        self._publish(self._hold_q, publish)
        if reason == "completed":
            self._evaluator.record_event("reference_endpoint_hold")
        self.finish(reason)

    def finish(self, reason: str = "interrupted") -> None:
        if self._termination is not None:
            return
        if reason not in ("held", "completed", "interrupted"):
            raise ValueError(f"invalid termination: {reason}")
        self._termination = reason
        if reason == "interrupted" or self._hold_q is None:
            self._state = "closed"
        if self._evaluator is not None:
            self._evaluator.finish(reason)
            self._writer.submit(self._evaluator, self._report_path())

    def close(self) -> None:
        self.finish("interrupted")
        self._state = "closed"
        self._writer.close()

    def _report_path(self) -> str:
        return str(Path(self._config["perf_report_path"]).parent / "tracking_report.md")

    def report_status(self) -> dict:
        return self._writer.status()

    def tracking_report(self):
        return deepcopy(self._evaluator.report()) if self._evaluator is not None else None

    def write_tracking_report(self, path: str | None = None) -> str:
        if self._evaluator is None:
            raise ValueError("tracking has not started; no report to write")
        if self._writer.status()["state"] == "writing":
            raise RuntimeError("background tracking report is still writing")
        return write_tracking_bundle(self._evaluator, path or self._report_path())

    def performance_snapshot(self) -> dict:
        durations = self._step_durations
        budget = float(self._config["latency_budget_ms"])
        misses = int(np.count_nonzero(np.asarray(durations) > budget))
        return {
            "steps": len(durations), "qp_fail_count": self._qp_fail_count,
            "latency_p50_ms": float(np.percentile(durations, 50)) if durations else float("nan"),
            "latency_p95_ms": float(np.percentile(durations, 95)) if durations else float("nan"),
            "latency_max_ms": float(np.max(durations)) if durations else float("nan"),
            "budget_ms": budget, "miss_count": misses,
            "miss_rate": misses / len(durations) if durations else 0.0,
        }

    def progress_snapshot(self) -> dict:
        result = self._last_result
        snapshot = {
            "tracking_started": self._started, "execution_state": self._state,
            "termination": self._termination, "ready": result is not None,
            "report_status": self.report_status(), **self.performance_snapshot(),
        }
        if result is None:
            return snapshot
        measured = step_from_result(result)
        snapshot.update({
            "err_6d": np.array(result.err_6d, dtype=float, copy=True),
            "pos_error_m": float(np.linalg.norm(result.err_6d[:3])),
            "path_progress_m": float(result.path_state[0]),
            "orient_error_rad": measured.values["tool_axis_error_rad"],
            "source_time_s": float(result.reference_source_time_s),
            "trajectory_duration_s": self._trajectory_duration_s,
            "arc_fraction": min(max(float(result.path_state[1]) / self._geometry.total_length_m, 0.0), 1.0),
            "cross_track_error_m": measured.values["cross_track_m"],
            "online_cross_track_error_m": float(result.cross_track_error_m),
            "measurement_boundary": "kernel_candidate", "feedrate_m_s": float(result.feedrate_m_s),
            "limiting_reason_code": int(result.limiting_reason_code),
            "at_endpoint": bool(result.reference_at_endpoint), "qp_ok": bool(result.qp_ok),
            "delta_slack": float(result.delta_slack),
        })
        return snapshot
