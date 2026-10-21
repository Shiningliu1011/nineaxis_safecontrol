import time
import json
import multiprocessing
from pathlib import Path

import pytest

from robot_safecontrol_moveit.static_cycle_timing import (
    CYCLE_TIMING_BOUNDARY, CycleTimings, load_cycle_timing,
)
from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder, write_json


def _write_completed_cycle_run(tmp_path):
    timing = CycleTimings(maximum_records=1)
    started = time.monotonic()
    with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
        recorder.append("event", monotonic_s=time.monotonic(), values={"reason": "started"})
        base_completed = time.monotonic()
        recorder.append("cycle", monotonic_s=base_completed, values={
            "duration_s": base_completed - started, "command_id": None,
            "accepted": False, "reason": "late_command", "period_s": .1,
        })
        sequence = recorder.last_sequence
        timing.record(sequence, started)
        recorder.append("transition", monotonic_s=time.monotonic(), values={
            "from": "TRACKING", "to": "PAUSED", "reason": "late_command",
        })
        after_transition = time.monotonic()
        timing.complete(sequence)
    write_json(tmp_path / "manifest.json", {"cycle_timing_boundary": CYCLE_TIMING_BOUNDARY})
    write_json(tmp_path / "controller.recording.json", {"cycle_timing": timing.snapshot()})
    write_json(tmp_path / "result.json", {"recording_statistics_sha256": {
        "controller": content_hash(tmp_path / "controller.recording.json"),
    }})
    return sequence, started, base_completed, after_transition


def test_complete_cycle_includes_its_sqlite_commit_and_following_transition(tmp_path):
    sequence, started, base_completed, after_transition = _write_completed_cycle_run(tmp_path)
    measured = load_cycle_timing(tmp_path)
    assert measured["boundary"] == CYCLE_TIMING_BOUNDARY
    assert len(measured["cycles"]) == 1
    cycle = measured["cycles"][0]
    assert cycle["sequence"] == sequence == 2
    assert cycle["completed_monotonic_s"] >= after_transition
    assert cycle["duration_s"] >= after_transition - started > base_completed - started


@pytest.mark.parametrize("damage", (
    "missing", "duplicate", "unmatched", "nonfinite", "short", "version", "boundary", "hash", "missing_file",
))
def test_declared_complete_timing_rejects_invalid_sidecar(tmp_path, damage):
    _write_completed_cycle_run(tmp_path)
    path = tmp_path / "controller.recording.json"
    statistics = json.loads(path.read_text())
    timing = statistics["cycle_timing"]
    if damage == "missing":
        timing["cycles"].clear()
    elif damage == "duplicate":
        timing["cycles"].append(dict(timing["cycles"][0]))
    elif damage == "unmatched":
        timing["cycles"][0]["sequence"] = 100
    elif damage == "nonfinite":
        timing["cycles"][0]["duration_s"] = float("nan")
    elif damage == "short":
        timing["cycles"][0]["duration_s"] = 0.
        timing["cycles"][0]["completed_monotonic_s"] = timing["cycles"][0]["started_monotonic_s"]
    elif damage == "version":
        timing["schema_version"] = 2
    elif damage == "boundary":
        timing["boundary"] = "before_commit"
    elif damage == "hash":
        statistics["changed"] = True
    if damage == "missing_file":
        path.rename(tmp_path / "incomplete.recording.json.pending")
    else:
        path.write_text(json.dumps(statistics))
        if damage != "hash":
            write_json(tmp_path / "result.json", {"recording_statistics_sha256": {
                "controller": content_hash(path),
            }})
    with pytest.raises(ValueError, match="完整周期"):
        load_cycle_timing(tmp_path)


def test_pending_cycle_cannot_be_published_or_completed_twice(tmp_path):
    timing = CycleTimings(maximum_records=1)
    with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
        started = time.monotonic()
        recorder.append("cycle", monotonic_s=time.monotonic(), values={"duration_s": 0.})
        timing.record(recorder.last_sequence, started)
        with pytest.raises(ValueError, match="未完成"):
            timing.snapshot()
        timing.complete_pending()
        with pytest.raises(ValueError, match="重复完成"):
            timing.complete(recorder.last_sequence)
        with pytest.raises(ValueError, match="上限"):
            timing.record(recorder.last_sequence + 1, time.monotonic())
        with pytest.raises(AttributeError):
            recorder.last_sequence = 42


def test_worker_preserves_original_exception_when_statistics_cannot_be_published(tmp_path):
    from robot_safecontrol_moveit.static_experiment import finish_worker_recording

    path = tmp_path / "plant.recording.json"
    path.write_text("already published")
    with RunRecorder(tmp_path / "plant.sqlite3") as recorder:
        with pytest.raises(ValueError, match="original failure"):
            try:
                raise ValueError("original failure")
            finally:
                finish_worker_recording(None, recorder, tmp_path, role="plant", stop_reason="fault",
                                        terminal={"reason": "fault"})
    assert path.read_text() == "already published"


def _record_cycle_until_killed(path, connection):
    recorder = RunRecorder(path)
    started = time.monotonic()
    completed = time.monotonic()
    recorder.append("cycle", monotonic_s=completed, values={
        "command_id": None, "accepted": False, "reason": "terminated_before_admission",
        "duration_s": completed - started, "period_s": .1,
    })
    connection.send(recorder.last_sequence)
    connection.recv()


def test_sigkill_preserves_committed_cycle_and_cannot_pass_complete_timing(tmp_path):
    from robot_safecontrol_moveit.static_experiment_recording import finalize_recording, read_stream

    parent, child = multiprocessing.get_context("spawn").Pipe()
    database = tmp_path / "controller.sqlite3"
    process = multiprocessing.get_context("spawn").Process(
        target=_record_cycle_until_killed, args=(database, child),
    )
    process.start()
    try:
        assert parent.poll(20.)
        assert parent.recv() == 1
    finally:
        process.kill()
        process.join(timeout=20.)
        parent.close()
        child.close()
    assert process.exitcode == -9
    finalize_recording(database)
    assert len(read_stream(database, "cycle")) == 1
    write_json(tmp_path / "manifest.json", {"cycle_timing_boundary": CYCLE_TIMING_BOUNDARY})
    with pytest.raises(ValueError, match="sidecar"):
        load_cycle_timing(tmp_path)


def test_failed_calculation_finishes_timing_after_fault_transition_and_terminal_event(tmp_path):
    import numpy as np
    import rclpy

    from robot_safecontrol_moveit.static_experiment import finish_worker_recording
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from robot_safecontrol_moveit.static_experiment_nodes import StaticExperimentController
    from robot_safecontrol_moveit.static_experiment_recording import iter_records, read_stream

    root = Path(__file__).resolve().parents[1]
    config = ExperimentConfig.load(root / "config/static_experiment_s0.yaml", resource_root=root)
    rclpy.init(domain_id=231)
    with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
        controller = StaticExperimentController(config, recorder)
        from work.static_execution import ExecutionSchedule
        from work.static_motion import ActuatorState

        initial = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
        schedule = ExecutionSchedule(initial, controller.planner, .1)
        started = time.monotonic()
        future = controller._executor.submit(
            controller.control.propose, initial, -.1, time_s=.1, executed_feed=0.,
        )
        controller._calculation = future, {
            "snapshot": schedule.snapshot(), "initial": initial, "start_step": 50,
            "progress": -.1, "started": started,
        }
        with pytest.raises(ValueError, match="参考进度"):
            finish_worker_recording(controller, recorder, tmp_path, role="controller",
                                    stop_reason=None, terminal={"reason": "worker_stopped"})
        assert not rclpy.ok()
        statistics = json.loads((tmp_path / "controller.recording.json").read_text())
        measured = statistics["cycle_timing"]["cycles"][0]
        base = next(iter_records(recorder.path, "cycle"))
        terminal = read_stream(recorder.path, "event")[-1]
        transition = read_stream(recorder.path, "transition")[-1]
        assert measured["sequence"] == base["sequence"]
        assert measured["completed_monotonic_s"] > terminal["monotonic_s"] > transition["monotonic_s"]
        assert measured["duration_s"] > base["values"]["duration_s"]
        assert terminal["values"]["reason"] == "fault"


@pytest.mark.parametrize("stage", ("calculation", "admission"))
def test_actual_ros_shutdown_completes_every_started_cycle_after_terminal(tmp_path, stage):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from robot_safecontrol_moveit.static_experiment import finish_worker_recording
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from robot_safecontrol_moveit.static_experiment_nodes import StaticExperimentController, StaticExperimentPlant
    from robot_safecontrol_moveit.static_experiment_recording import iter_records, read_stream

    root = Path(__file__).resolve().parents[1]
    config = ExperimentConfig.load(root / "config/static_experiment_s0.yaml", resource_root=root)
    rclpy.init(domain_id=231)
    executor = SingleThreadedExecutor()
    try:
        with RunRecorder(tmp_path / "controller.sqlite3") as recorder, \
                RunRecorder(tmp_path / "plant.sqlite3") as plant_recorder:
            controller = StaticExperimentController(config, recorder)
            plant = StaticExperimentPlant(config, plant_recorder)
            executor.add_node(controller)
            executor.add_node(plant)
            deadline = time.monotonic() + 10.
            while True:
                executor.spin_once(timeout_sec=.01)
                if stage == "calculation":
                    ready = controller.recording_budget.snapshot()["started_tasks"].get("calculation")
                else:
                    ready = read_stream(recorder.path, "command")
                if ready:
                    break
                assert time.monotonic() < deadline
            executor.remove_node(controller)
            executor.remove_node(plant)
            finish_worker_recording(controller, recorder, tmp_path, role="controller",
                                    stop_reason="budget_exhausted", terminal={"reason": "budget_exhausted"})
            finish_worker_recording(plant, plant_recorder, tmp_path, role="plant",
                                    stop_reason="budget_exhausted", terminal={"reason": "budget_exhausted"})
            cycles = list(iter_records(recorder.path, "cycle"))
            statistics = json.loads((tmp_path / "controller.recording.json").read_text())
            completed = statistics["cycle_timing"]["cycles"]
            assert len(cycles) == len(completed) == 1
            assert [row["sequence"] for row in cycles] == [row["sequence"] for row in completed]
            assert completed[0]["completed_monotonic_s"] > read_stream(recorder.path, "event")[-1]["monotonic_s"]
            assert cycles[0]["values"]["reason"] == "budget_exhausted"
            assert statistics["statistics_storage_bytes"] >= (tmp_path / "controller.recording.json").stat().st_blocks * 512
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def test_real_plant_late_admission_includes_recovery_transition_commit(tmp_path):
    import numpy as np
    import rclpy
    from std_msgs.msg import String
    from robot_safecontrol_moveit.static_experiment import finish_worker_recording
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from robot_safecontrol_moveit.static_experiment_nodes import StaticExperimentController, StaticExperimentPlant
    from robot_safecontrol_moveit.static_experiment_recording import read_stream
    from work.static_motion import ActuatorState

    root = Path(__file__).resolve().parents[1]
    config = ExperimentConfig.load(root / "config/static_experiment_s0.yaml", resource_root=root)
    rclpy.init(domain_id=231)
    with RunRecorder(tmp_path / "controller.sqlite3") as recorder, \
            RunRecorder(tmp_path / "plant.sqlite3") as plant_recorder:
        controller = StaticExperimentController(config, recorder)
        plant = StaticExperimentPlant(config, plant_recorder)
        try:
            initial = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
            started = time.monotonic()
            future = controller._executor.submit(
                controller.control.propose, initial, 0., time_s=.1, executed_feed=0.,
            )
            controller._finish_calculation(
                future, snapshot=plant.schedule.snapshot(), initial=initial,
                start_step=50, progress=0., started=started,
            )
            command = read_stream(recorder.path, "command")[0]["values"]["scheduled"]
            for _ in range(51):
                plant.schedule.advance()
            plant._on_command(String(data=json.dumps({"config_sha256": config.identity, "command": command})))
            plant._pending_check[1].result()
            plant._tick()
            admission = read_stream(plant_recorder.path, "admission")[0]["values"]
            assert admission["reason"] == "late_command"
            controller._on_admission(String(data=json.dumps(admission)))
            after_callback = time.monotonic()
            measured = controller.cycle_timings.snapshot()["cycles"][0]
            transition = read_stream(recorder.path, "transition")[-1]
            assert transition["values"]["reason"] == "late_command"
            assert transition["monotonic_s"] < measured["completed_monotonic_s"] <= after_callback
            assert measured["duration_s"] > read_stream(recorder.path, "cycle")[0]["values"]["duration_s"]
        finally:
            finish_worker_recording(controller, recorder, tmp_path, role="controller",
                                    stop_reason=None, terminal={"reason": "worker_stopped"})
            finish_worker_recording(plant, plant_recorder, tmp_path, role="plant",
                                    stop_reason=None, terminal={"reason": "worker_stopped"})
