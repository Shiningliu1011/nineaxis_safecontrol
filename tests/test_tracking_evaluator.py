"""跟踪评价指标测试。"""

import math

import numpy as np
import pytest

from robot_safecontrol_moveit.tracking_evaluator import (
    TrackingEvaluator,
    TrackingReport,
    TrackingStepData,
    step_from_result,
)


def _step(**overrides):
    defaults = dict(
        err_6d=np.zeros(6), cross_track_error_m=0.0, feedrate_m_s=0.1,
        qp_ok=True, min_obs_dist=0.5, delta_slack=0.0,
        reference_source_time_s=0.0, reference_at_endpoint=False,
        limiting_reason_code=0,
    )
    defaults.update(overrides)
    return defaults


class TestStepFromResult:
    def test_dict_input(self) -> None:
        d = step_from_result(_step(pos_error_m=0.0))
        assert isinstance(d, TrackingStepData)
        assert d.pos_error_m == 0.0
        assert d.qp_ok is True

    def test_err_6d_decomposed(self) -> None:
        err = np.array([0.001, 0.002, 0.003, 0.01, 0.02, 0.03])
        d = step_from_result(_step(err_6d=err))
        assert d.pos_error_m == pytest.approx(np.linalg.norm(err[:3]), abs=1e-12)
        assert d.orient_error_rad == pytest.approx(np.linalg.norm(err[3:]), abs=1e-12)

    def test_qp_fail_flag(self) -> None:
        d = step_from_result(_step(qp_ok=False))
        assert d.qp_ok is False


class TestTrackingEvaluator:
    def test_empty_report(self) -> None:
        ev = TrackingEvaluator(trajectory_duration_s=30.0)
        r = ev.report()
        assert r.total_steps == 0
        assert r.tracking_score == 0.0
        assert r.completed is False

    def test_single_step(self) -> None:
        ev = TrackingEvaluator(trajectory_duration_s=30.0)
        ev.update(_step(
            err_6d=np.array([0.001, 0.0, 0.0, 0.0, 0.0, 0.0]),
            cross_track_error_m=0.0005,
            feedrate_m_s=0.1,
            reference_source_time_s=15.0,
        ))
        r = ev.report()
        assert r.total_steps == 1
        assert r.max_pos_error_m == pytest.approx(0.001, abs=1e-9)
        assert r.mean_cross_track_m == pytest.approx(0.0005, abs=1e-9)
        assert r.qp_success_rate == 1.0
        assert r.completion_fraction == pytest.approx(0.5, abs=1e-3)

    def test_qp_fail_counting(self) -> None:
        ev = TrackingEvaluator()
        for i in range(10):
            ev.update(_step(qp_ok=(i % 3 != 0)))
        r = ev.report()
        assert r.qp_fail_count == 4  # steps 0, 3, 6, 9
        assert r.qp_success_rate == pytest.approx(0.6, abs=1e-9)

    def test_completion_at_endpoint(self) -> None:
        ev = TrackingEvaluator(trajectory_duration_s=30.0)
        ev.update(_step(reference_source_time_s=30.0, reference_at_endpoint=True))
        r = ev.report()
        assert r.completed is True
        assert r.completion_fraction == pytest.approx(1.0, abs=1e-3)

    def test_obstacle_distance_tracking(self) -> None:
        ev = TrackingEvaluator()
        ev.update(_step(min_obs_dist=0.5))
        ev.update(_step(min_obs_dist=0.01))
        ev.update(_step(min_obs_dist=0.1))
        r = ev.report()
        assert r.min_obstacle_distance_m == pytest.approx(0.01, abs=1e-9)
        assert r.mean_obstacle_distance_m == pytest.approx(
            (0.5 + 0.01 + 0.1) / 3, abs=1e-6)

    def test_limiting_reason_counts(self) -> None:
        ev = TrackingEvaluator()
        for _ in range(5):
            ev.update(_step(limiting_reason_code=1))
        for _ in range(3):
            ev.update(_step(limiting_reason_code=2))
        ev.update(_step(limiting_reason_code=0))
        r = ev.report()
        assert r.limiting_reason_counts == {1: 5, 2: 3, 0: 1}

    def test_wall_time(self) -> None:
        ev = TrackingEvaluator()
        ev.update(_step(), wall_time_s=10.0)
        ev.update(_step(), wall_time_s=10.5)
        ev.update(_step(), wall_time_s=11.0)
        r = ev.report()
        assert r.wall_time_s == pytest.approx(1.0, abs=1e-9)

    def test_score_perfect_tracking(self) -> None:
        """零误差 + 全部 qp_ok + 完成 = 高分。"""
        ev = TrackingEvaluator(trajectory_duration_s=30.0)
        for i in range(100):
            ev.update(_step(
                err_6d=np.zeros(6), cross_track_error_m=0.0,
                feedrate_m_s=0.1, qp_ok=True, min_obs_dist=0.5,
                reference_source_time_s=30.0 * i / 99,
                reference_at_endpoint=(i == 99),
            ))
        r = ev.report()
        assert r.tracking_score == pytest.approx(1.0, abs=0.01)
        assert r.completed is True
        assert r.qp_success_rate == 1.0

    def test_score_poor_tracking(self) -> None:
        """大误差 + QP 失败 + 未完成 = 低分。"""
        ev = TrackingEvaluator(trajectory_duration_s=30.0)
        for i in range(100):
            ev.update(_step(
                err_6d=np.array([0.01, 0.01, 0.01, 0.1, 0.1, 0.1]),
                cross_track_error_m=0.01,
                feedrate_m_s=0.01, qp_ok=(i > 50),
                min_obs_dist=0.001,
                reference_source_time_s=15.0,
            ))
        r = ev.report()
        assert r.tracking_score < 0.3
        assert r.completed is False

    def test_summary_string(self) -> None:
        ev = TrackingEvaluator(trajectory_duration_s=30.0)
        ev.update(_step(reference_source_time_s=15.0))
        r = ev.report()
        s = r.summary()
        assert "score=" in s
        assert "pos=" in s
        assert "qp=" in s

    def test_markdown_report(self) -> None:
        ev = TrackingEvaluator(trajectory_duration_s=30.0)
        ev.update(_step(reference_source_time_s=15.0))
        r = ev.report()
        md = r.markdown()
        assert "# 路径跟踪评价报告" in md
        assert "综合评分" in md
        assert "误差统计" in md


class TestP95:
    def test_p95_matches_numpy(self) -> None:
        ev = TrackingEvaluator()
        rng = np.random.default_rng(42)
        values = rng.normal(0.001, 0.0002, size=200).tolist()
        for v in values:
            ev.update(_step(
                err_6d=np.array([v, 0, 0, 0, 0, 0]),
            ))
        r = ev.report()
        assert r.p95_pos_error_m == pytest.approx(
            np.percentile(values, 95), rel=1e-6)


def test_no_global_state() -> None:
    e1 = TrackingEvaluator()
    e2 = TrackingEvaluator()
    e1.update(_step(feedrate_m_s=0.5))
    assert e2.step_count == 0
