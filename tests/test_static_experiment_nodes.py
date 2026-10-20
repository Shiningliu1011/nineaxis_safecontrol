import os
import json
from pathlib import Path
import time

import numpy as np
import pytest
import rclpy

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
from robot_safecontrol_moveit.static_experiment_capacity import RecordingBudgetExhausted
from robot_safecontrol_moveit.static_experiment_nodes import StaticExperimentController, StaticExperimentPlant
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder, read_stream


ROOT = Path(__file__).resolve().parents[1]


def test_actual_plant_checks_local_budget_before_next_feedback(tmp_path):
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    rclpy.init(domain_id=230)
    try:
        with RunRecorder(tmp_path / "plant.sqlite3") as recorder:
            plant = StaticExperimentPlant(config, recorder)
            try:
                plant.recording_budget.start(time.monotonic() - 500.)
                with pytest.raises(RecordingBudgetExhausted, match="deadline"):
                    rclpy.spin_once(plant, timeout_sec=.1)
                assert read_stream(recorder.path, "feedback") == []
            finally:
                plant.destroy_node()
    finally:
        rclpy.shutdown()


def test_actual_controller_shutdown_preserves_started_candidate_and_cycle(tmp_path):
    from rclpy.executors import SingleThreadedExecutor

    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method="D",
    )
    rclpy.init(domain_id=230)
    executor = SingleThreadedExecutor()
    try:
        with RunRecorder(tmp_path / "controller.sqlite3") as controller_recorder, \
                RunRecorder(tmp_path / "plant.sqlite3") as plant_recorder:
            controller = StaticExperimentController(config, controller_recorder)
            plant = StaticExperimentPlant(config, plant_recorder)
            executor.add_node(controller)
            executor.add_node(plant)
            try:
                deadline = time.monotonic() + 5.
                while not controller.recording_budget.snapshot()["started_tasks"].get("calculation"):
                    assert time.monotonic() < deadline
                    executor.spin_once(timeout_sec=.01)
                executor.remove_node(controller)
                executor.remove_node(plant)
                controller.finish_recording("budget_exhausted")
                assert len(read_stream(controller_recorder.path, "candidate")) == 1
                assert len(read_stream(controller_recorder.path, "cycle")) == 1
                assert read_stream(controller_recorder.path, "command") == []
                assert read_stream(controller_recorder.path, "rejection")[0]["values"]["reason"] == "budget_exhausted"
            finally:
                controller.destroy_node()
                plant.destroy_node()
    finally:
        executor.shutdown()
        rclpy.shutdown()


def test_worker_cleanup_preserves_failed_future_event_statistics_and_exception(tmp_path):
    from robot_safecontrol_moveit.static_experiment import finish_worker_recording
    from work.static_motion import ActuatorState
    from work.static_execution import ExecutionSchedule

    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method="D",
    )
    rclpy.init(domain_id=230)
    with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
        controller = StaticExperimentController(config, recorder)
        initial = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
        schedule = ExecutionSchedule(initial, controller.planner, .1)
        future = controller._executor.submit(
            controller.control.propose, initial, -.1, time_s=.1, executed_feed=0.,
        )
        controller._calculation = future, {
            "snapshot": schedule.snapshot(), "initial": initial, "start_step": 50,
            "progress": -.1, "started": time.monotonic(),
        }
        with pytest.raises(ValueError, match="参考进度"):
            finish_worker_recording(
                controller, recorder, tmp_path, role="controller",
                stop_reason=None, terminal={"reason": "worker_stopped"},
            )
        assert not rclpy.ok()
        assert read_stream(recorder.path, "cycle")[0]["values"]["reason"] == "calculation_failure"
        assert read_stream(recorder.path, "event")[0]["values"]["reason"] == "fault"
        assert read_stream(recorder.path, "transition")[-1]["values"]["to"] == "FAULT"
        statistics = json.loads((tmp_path / "controller.recording.json").read_text())
        assert statistics["stream_counts"]["cycle"] == 1
        assert statistics["stream_counts"]["event"] == 1


def test_plant_cleanup_preserves_failed_admission_and_propagates_exception(tmp_path):
    from robot_safecontrol_moveit.static_experiment import finish_worker_recording
    from work.static_motion import certify_motion

    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    rclpy.init(domain_id=230)
    with RunRecorder(tmp_path / "plant.sqlite3") as recorder:
        plant = StaticExperimentPlant(config, recorder)
        command = plant.schedule.active
        received = time.monotonic()
        future = plant._executor.submit(
            certify_motion, command.motion, plant.geometry, plant.planner, minimum_interval_s=-1.,
        )
        plant._pending_check = command, future, received
        with pytest.raises(ValueError, match="区间检查参数"):
            finish_worker_recording(
                plant, recorder, tmp_path, role="plant",
                stop_reason=None, terminal={"reason": "worker_stopped"},
            )
        assert not rclpy.ok()
        assert read_stream(recorder.path, "admission")[0]["values"]["reason"] == "admission_failure"
        assert read_stream(recorder.path, "event")[0]["values"]["reason"] == "fault"
        assert json.loads((tmp_path / "plant.recording.json").read_text())["stream_counts"]["admission"] == 1


def test_actual_controller_records_failed_calculation_cycle(tmp_path):
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method="D",
    )
    previous_localhost = os.environ.get("ROS_LOCALHOST_ONLY")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init(domain_id=229)
    try:
        with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
            controller = StaticExperimentController(config, recorder)
            try:
                from work.static_execution import ExecutionSchedule
                from work.static_motion import ActuatorState

                initial = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
                schedule = ExecutionSchedule(initial, controller.planner, .1)
                started = time.monotonic()
                future = controller._executor.submit(
                    controller.control.propose, initial, -.1, time_s=.1, executed_feed=0.,
                )
                with pytest.raises(ValueError, match="参考进度"):
                    controller._finish_calculation(
                        future, snapshot=schedule.snapshot(), initial=initial,
                        start_step=50, progress=-.1, started=started,
                    )
                cycles = read_stream(recorder.path, "cycle")
                assert len(cycles) == 1
                cycle = cycles[0]["values"]
                assert cycle["accepted"] is False
                assert cycle["reason"] == "calculation_failure"
                assert cycle["command_id"] is None
                assert cycle["duration_s"] > 0.
                assert not read_stream(recorder.path, "command")
            finally:
                controller.destroy_node()
    finally:
        rclpy.shutdown()
        if previous_localhost is None:
            del os.environ["ROS_LOCALHOST_ONLY"]
        else:
            os.environ["ROS_LOCALHOST_ONLY"] = previous_localhost
