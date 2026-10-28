import json
import logging
from hashlib import sha256
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


def test_controller_initialization_compiles_current_terminal_solver(tmp_path, caplog):
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT,
        method="B", clearance_reserve_m=.0001,
    )
    source = ROOT / "tests/data/static_b_s0_terminal.json"
    recorded = json.loads(source.read_text())
    reference = json.loads((ROOT / "tests/data/static_terminal_configuration_j1_origin85mm.json").read_text())
    assert reference["source_record_sha256"] == sha256(source.read_bytes()).hexdigest()
    # 历史配置的原 J1 小于 85 mm，当前终点验证采用有效的仿真初始配置。
    assert recorded["initial"]["position"][0] - .085 < 0
    progress = reference["reference_progress_m"]
    assert progress == recorded["reference_progress_m"]
    jax.clear_caches()
    rclpy.init(domain_id=231)
    compile_loggers = [logging.getLogger(name) for name in (
        "jax._src.interpreters.pxla", "jax._src.compiler",
    )]
    for logger in compile_loggers:
        logger.addHandler(caplog.handler)
    try:
        with RunRecorder(tmp_path / "controller.sqlite3") as recorder:
            with jax.log_compiles(True):
                controller = StaticExperimentController(config, recorder)
            try:
                from work.static_motion import ActuatorState
                from work.nineaxis_model import NineaxisModel

                model = NineaxisModel.load(root=ROOT)
                q = np.asarray(reference["position_si"])
                assert reference["kind"] == "simulation_initial_configuration"
                assert reference["model_hash"] == model.model_hash
                assert reference["reference_identity"] == controller.control.path.reference_identity
                assert np.all(q >= model.position_lower + reference["start_joint_margin_si"])
                assert np.all(q <= model.position_upper - reference["start_joint_margin_si"])
                target = controller.control.path.sample(progress)
                pose = model.fk(q)
                np.testing.assert_allclose(pose[:3, 3], target.position_m, atol=1e-8, rtol=0)
                np.testing.assert_allclose(pose[:3, 0], target.rotation[:, 0], atol=1e-7, rtol=0)

                assert "Compiling " in caplog.text
                caplog.clear()
                with jax.log_compiles(True):
                    candidate = controller.control.propose(
                        ActuatorState(q, np.zeros(9), np.zeros(9)), progress,
                        time_s=.1, executed_feed=0.,
                    )
                assert candidate is not None and candidate.certificate.accepted
                assert candidate.solution.solution.shape == (9,)
                assert controller.control.last_trials[0]["terminal_interval"]
                assert "Compiling " not in caplog.text
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
                remaining = controller.control.path.total_length_m - progress
                assert candidate.solution.feed_m_s == pytest.approx(
                    remaining / config.values["control"]["period_s"], abs=1e-13,
                )
            finally:
                controller.destroy_node()
    finally:
        for logger in compile_loggers:
            logger.removeHandler(caplog.handler)
        rclpy.shutdown()
