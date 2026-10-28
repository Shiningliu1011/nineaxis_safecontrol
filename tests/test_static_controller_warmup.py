import json
from pathlib import Path

import jax
import numpy as np
import pytest
import rclpy
from scipy.optimize import LinearConstraint, minimize

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
from robot_safecontrol_moveit.static_experiment_nodes import StaticExperimentController
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("method", ["A", "B", "C", "D", "E"])
def test_first_controller_candidate_starts_with_its_own_recorded_history(tmp_path, method):
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT,
        method=method, clearance_reserve_m=.0001,
    )
    rclpy.init(domain_id=231)
    try:
        with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
            controller = StaticExperimentController(config, recorder)
            try:
                from work.static_motion import ActuatorState

                assert controller.control.recovery.events == []
                state = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
                candidate = controller.control.propose(state, 0., time_s=.1, executed_feed=0.)
                assert candidate is not None and candidate.certificate.accepted
                trials = controller.control.last_trials
                assert trials
                assert all(trial["proposal_sequence"] == 1 for trial in trials)
                assert all(trial["qp_reused_from_proposal"] is None for trial in trials)
                assert all(trial["motion_reused_from_proposal"] is None for trial in trials)
            finally:
                controller.destroy_node()
    finally:
        rclpy.shutdown()


def test_controller_initialization_compiles_recorded_terminal_solver(tmp_path, capfd):
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT,
        method="B", clearance_reserve_m=.0001,
    )
    recorded = json.loads((ROOT / "tests/data/static_b_s0_terminal.json").read_text())
    jax.clear_caches()
    rclpy.init(domain_id=231)
    try:
        with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
            with jax.log_compiles(True):
                controller = StaticExperimentController(config, recorder)
            try:
                from work.static_motion import ActuatorState

                assert "Compiling " in capfd.readouterr().err
                with jax.log_compiles(True):
                    candidate = controller.control.propose(
                        ActuatorState.from_record(recorded["initial"]),
                        recorded["reference_progress_m"],
                        time_s=recorded["start_step"] * .002, executed_feed=0.,
                    )
                assert candidate is not None and candidate.certificate.accepted
                assert candidate.solution.solution.shape == (9,)
                assert controller.control.last_trials[0]["terminal_interval"]
                assert "Compiling " not in capfd.readouterr().err
                problem = candidate.solution.problem
                matrix, vector = np.asarray(problem["P"]), np.asarray(problem["c"])
                constraints, bounds = np.asarray(problem["G"]), np.asarray(problem["h"])
                solution, dual = candidate.solution.solution, candidate.solution.dual
                independent = minimize(
                    lambda value: (.5 * value @ matrix @ value + vector @ value,
                                   matrix @ value + vector),
                    np.zeros_like(solution), jac=True, method="SLSQP",
                    constraints=[LinearConstraint(constraints, -np.inf, bounds)],
                    options={"ftol": 1e-13, "maxiter": 1000},
                )
                assert independent.success
                assert np.max(constraints @ solution - bounds) <= 1e-9
                assert np.min(dual) >= -1e-8
                assert np.max(abs(matrix @ solution + vector + constraints.T @ dual)) <= 1e-8
                assert np.max(abs((bounds - constraints @ solution) * dual)) <= 1e-8
                actual_cost = .5 * solution @ matrix @ solution + vector @ solution
                assert actual_cost == pytest.approx(independent.fun, abs=1e-8)
                remaining = controller.control.path.total_length_m - recorded["reference_progress_m"]
                assert candidate.solution.feed_m_s == pytest.approx(
                    remaining / config.values["control"]["period_s"], abs=1e-13,
                )
            finally:
                controller.destroy_node()
    finally:
        rclpy.shutdown()
