import os
from pathlib import Path
import time

import numpy as np
import pytest
import rclpy

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
from robot_safecontrol_moveit.static_experiment_nodes import StaticExperimentController
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder, read_stream


ROOT = Path(__file__).resolve().parents[1]


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
