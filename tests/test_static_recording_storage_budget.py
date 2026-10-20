from pathlib import Path
import shutil
import struct

import msgpack
import numpy as np
import pytest

from robot_safecontrol_moveit.static_experiment_capacity import (
    RecordingBudget, RecordingBudgetExhausted, RecordingLimits,
)
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder, read_stream


ROOT = Path(__file__).resolve().parents[1]


def _problem():
    with np.load(ROOT / "tests/data/static_qp_feed_boundary.npz") as data:
        return {key: data[key].tolist() for key in data.files}


def _limits(**changes):
    return {
        "database_bytes": 64 * 1024 * 1024,
        "wal_bytes": 64 * 1024,
        "other_bytes": 1024 * 1024,
        "minimum_free_bytes": 1024 * 1024,
        "completion_bytes": 512 * 1024,
        **changes,
    }


def test_wal_limit_stops_new_work_and_retains_triggering_task_and_terminal(tmp_path):
    database = tmp_path / "controller.sqlite3"
    values = {"problem": _problem(), "signed_zero": -0.0}
    with RunRecorder(database, storage_limits=_limits()) as recorder:
        committed = 0
        with pytest.raises(RecordingBudgetExhausted, match="wal_bytes"):
            for number in range(1000):
                recorder.check_capacity()
                recorder.append("candidate", monotonic_s=float(number), values=values)
                committed += 1
        assert 0 < committed < 1000
        recorder.append("cycle", monotonic_s=1001., values={"accepted": False, "reason": "budget_exhausted"})
        recorder.append("event", monotonic_s=1002., values={"reason": "budget_exhausted"})
        snapshot = recorder.capacity_snapshot()
        assert snapshot["reason"] == "wal_bytes"
        assert snapshot["completion_within_reserve"]
    records = read_stream(database, "candidate")
    assert len(records) == committed
    assert records[-1]["values"]["problem"] == values["problem"]
    assert struct.pack("!d", records[-1]["values"]["signed_zero"]) == struct.pack("!d", -0.0)
    assert read_stream(database, "event")[0]["values"]["reason"] == "budget_exhausted"


def test_actual_free_space_check_prevents_new_work_and_preserves_existing_records(tmp_path):
    database = tmp_path / "plant.sqlite3"
    limits = _limits(minimum_free_bytes=shutil.disk_usage(tmp_path).free + 1024 * 1024)
    with RunRecorder(database, storage_limits=limits) as recorder:
        with pytest.raises(RecordingBudgetExhausted, match="free_bytes"):
            recorder.check_capacity()
        recorder.append("event", monotonic_s=1., values={"reason": "budget_exhausted"})
    assert read_stream(database, "event")[0]["values"]["reason"] == "budget_exhausted"


def test_task_budget_checks_real_storage_before_incrementing_task_count(tmp_path):
    with RunRecorder(tmp_path / "controller.sqlite3", storage_limits=_limits(wal_bytes=1)) as recorder:
        budget = RecordingBudget(
            RecordingLimits(420., 2., .1), capacity_check=recorder.check_capacity,
        )
        with pytest.raises(RecordingBudgetExhausted, match="wal_bytes"):
            budget.begin("calculation")
        assert budget.snapshot()["started_tasks"] == {}


def test_database_size_and_new_record_shape_stop_after_complete_record(tmp_path):
    for name, limits, expected in (
        ("database", _limits(database_bytes=1), "database_bytes"),
        ("schema", _limits(record_bytes={"candidate": 1, "event": 1024}), "record_bytes/candidate"),
    ):
        directory = tmp_path / name
        directory.mkdir()
        with RunRecorder(directory / "controller.sqlite3", storage_limits=limits) as recorder:
            recorder.append("candidate", monotonic_s=1., values={"problem": _problem()})
            with pytest.raises(RecordingBudgetExhausted, match=expected):
                recorder.check_capacity()
            recorder.append("event", monotonic_s=2., values={"reason": "budget_exhausted"})
        assert read_stream(recorder.path, "candidate")[0]["values"]["problem"] == _problem()


def test_pending_cycles_require_space_for_every_existing_cycle_and_new_task(tmp_path):
    limits = _limits(
        wal_bytes=1024 * 1024, completion_bytes=8192,
        completion_base_bytes=4096, pending_cycle_bytes=4096,
    )
    with RunRecorder(tmp_path / "controller.sqlite3", storage_limits=limits) as recorder:
        recorder.check_capacity(pending_cycles=1)
        with pytest.raises(RecordingBudgetExhausted, match="completion_bytes"):
            recorder.check_capacity(pending_cycles=2)


def test_fixed_schema_covers_all_nine_real_trials_and_maximum_solver_dimensions():
    from robot_safecontrol_moveit.static_recording_capacity import schema_record_bounds

    fixture = msgpack.unpackb(
        (ROOT / "tests/data/static_recording_nine_trials.msgpack").read_bytes(), raw=False,
    )
    candidate = fixture["record"]["values"]
    assert len(candidate["trials"]) == 9
    bounds = schema_record_bounds(76)
    assert bounds["maximum_trials"] == 9
    assert bounds["qp_shapes"]["G"] == [114, 10]
    assert bounds["sns_shapes"]["G"] == [112, 9]
    assert bounds["candidate_bytes"] > len(msgpack.packb(
        candidate, use_bin_type=True, use_single_float=False,
    ))


def test_terminal_event_and_worker_result_share_capacity_termination(tmp_path):
    import json
    from robot_safecontrol_moveit.static_experiment import finish_worker_recording

    with RunRecorder(tmp_path / "controller.sqlite3", storage_limits=_limits(wal_bytes=1)) as recorder:
        finish_worker_recording(
            None, recorder, tmp_path, role="controller", stop_reason=None,
            terminal={"reason": "worker_stopped"},
        )
    event = read_stream(recorder.path, "event")
    result = json.loads((tmp_path / "controller_result.json").read_text())
    assert len(event) == 1
    assert event[0]["values"]["reason"] == result["termination"] == "budget_exhausted"


def test_actual_plant_checks_storage_before_recording_feedback(tmp_path):
    import rclpy
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from robot_safecontrol_moveit.static_experiment_nodes import StaticExperimentPlant

    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    rclpy.init(domain_id=230)
    try:
        with RunRecorder(tmp_path / "plant.sqlite3", storage_limits=_limits(wal_bytes=1)) as recorder:
            plant = StaticExperimentPlant(config, recorder)
            try:
                with pytest.raises(RecordingBudgetExhausted, match="wal_bytes"):
                    rclpy.spin_once(plant, timeout_sec=.1)
                assert read_stream(recorder.path, "feedback") == []
            finally:
                plant.destroy_node()
    finally:
        rclpy.shutdown()
