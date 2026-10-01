import hashlib
import json
import logging
import time
from pathlib import Path

import numpy as np
import pytest

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.production_config import build_effective_configuration, load_production_profile
from robot_safecontrol_moveit.tracking_contract import EvidenceContext, EvaluationScope
from robot_safecontrol_moveit.tracking_run import TrackingRun


ROOT = Path(__file__).resolve().parents[1]
START = np.array([
    .027276550053036794, -.03528867367370082, .5059981669275009,
    -.7240440926105509, 1.4877532069125114, -.2066365806668871,
    .3853811558742877, .22248106729961423, -.12343186955927415,
])
DISTANT = np.array([
    .2303562, .1112539, 1.0167209, -.6810303, -1.8294025,
    -.4664294, .4743473, -1.0429228, .0289233,
])


@pytest.fixture(scope="module")
def kernel():
    bootstrap_portable(ROOT / "portable_oscbf")
    from work.ik_data_loader import load_repository_trajectory
    from work.jax_control_facade import JaxControlLoop
    from work.path_following import PathFollowingConfig

    config = dict(build_effective_configuration(
        load_production_profile(ROOT / "config/oscbf_controller.yaml", node_name="oscbf_controller"),
        {}, share_dir=ROOT,
    ).values)
    trajectory = load_repository_trajectory(
        config["trajectory_mat"], config_yaml_path=config["portable_config_yaml"],
        feedrate_scale=config["reference_feedrate_scale"],
    )
    trajectory.set_surface_normal_orientation(config["cylinder_axis_direction"])
    geometry = trajectory.path_geometry()
    loop = JaxControlLoop(**{name: config[name] for name in (
        "dt", "dt_path", "w_pos", "w_orient", "w_joint", "temporal_lambda",
        "enable_x64", "solver_tol", "task_mode",
    )})
    loop.configure_path(geometry, PathFollowingConfig(
        reference_lead_m=config["reference_lead_m"],
        maximum_tool_axis_speed_rad_s=config["max_tool_axis_speed_rad_s"],
    ))
    loop.init_cbf()
    return loop, geometry, config, trajectory.num_points * trajectory.Ts


@pytest.fixture
def run_factory(kernel, tmp_path):
    runs = []

    def create(*, wait=True, destination=None, resources=None, obstacles=False, scope=None):
        loop, geometry, config, duration = resources or kernel
        config = dict(config, wait_for_start=wait, enable_perception_obstacles=obstacles,
                      perf_report_path=str(destination or tmp_path / f"run-{len(runs)}" / "perf.md"))
        run = TrackingRun(
            loop=loop, geometry=geometry, config=config, trajectory_duration_s=duration,
            evidence=EvidenceContext(
                run_id="tracking-run-test", kind="model", boundary="kernel_candidate",
                model_id="portable_oscbf/work", config_id="config/oscbf_controller.yaml",
                trajectory_id="sha256:" + hashlib.sha256(geometry.positions_m.tobytes() + geometry.rotations.tobytes()).hexdigest(),
                data_id="test_tracking_run.py:START,DISTANT", scenario="repository path",
                measurement="kernel candidate before command smoothing", time_basis="perf_counter",
            ), log=logging.getLogger("tracking_run_test"), scope=scope,
        )
        runs.append(run)
        return run

    yield create
    for run in runs:
        run.close()


@pytest.mark.parametrize("wait", [True, False])
def test_start_gate_consumption_and_interruption(run_factory, wait):
    run = run_factory(wait=wait)
    commands = []
    run.tick(commands.append)
    assert run.progress_snapshot()["steps"] == 0
    run.receive_state(START)
    if wait:
        run.tick(commands.append)
        assert commands == []
        assert run.start() == "TRACKING_STARTED"
    assert run.start() == "ALREADY_TRACKING"
    run.tick(commands.append)
    assert len(commands) == 1
    for _ in range(3):
        run.tick(commands.append)
    assert len(commands) == 1
    assert run.tracking_report().total_steps == 1
    run.close()
    run.close()
    snapshot = run.progress_snapshot()
    assert snapshot["execution_state"] == "closed"
    assert snapshot["termination"] == "interrupted"
    assert snapshot["report_status"]["state"] == "saved"


def test_direct_steps_have_no_lifecycle_samples_and_own_results(run_factory):
    run = run_factory()
    q = START.copy()
    for _ in range(4):
        record = run.step_once(q)
        q = record.q_next
        assert record.qp_ok and record.min_obs_dist is None
    assert run.tracking_report() is None
    assert run.progress_snapshot()["steps"] == 0
    record.err_6d[:] = 100
    record.constraint_metrics.clear()
    snapshot = run.progress_snapshot()
    snapshot["err_6d"][:] = 100
    assert np.max(np.abs(run.progress_snapshot()["err_6d"])) < 1
    run.close()
    assert run.report_status()["state"] == "idle"


@pytest.mark.parametrize("enabled", [True, False])
def test_real_obstacle_measurement_and_declared_scope(run_factory, kernel, enabled):
    length = kernel[1].total_length_m
    scope = EvaluationScope(length, length * .5, length)
    run = run_factory(wait=False, obstacles=enabled, scope=scope)
    commands = []
    obstacles = {
        "obs_pos": np.full((8, 3), 5.), "obs_radii": np.full(8, .1),
        "obs_enabled": np.ones(8), "obs_d_safe": np.full(8, .03),
        "obs_vel": np.zeros((8, 3)), "obs_alpha": np.ones(8),
    }
    run.receive_state(START)
    run.tick(commands.append, obs_kwargs=obstacles)
    run.close()
    report = run.tracking_report()
    assert report.scope == scope
    assert report.total_steps == 1 and not report.full_path_verified
    assert not report.completed
    assert any("outside predeclared scope" in issue for issue in report.issues)
    assert report.metrics["obstacle_clearance_m"].count == 0
    metric = report.metrics["obstacle_margin_m"]
    assert metric.count == int(enabled)
    if enabled:
        assert metric.minimum > 0.
    else:
        assert report.constraint_metrics["obstacle.residual"]["inactive_count"] == 1


@pytest.mark.parametrize("interval,log_fragment", [
    (0.0, "source-time frozen"), (1.05, "reference feedrate=0"),
])
def test_both_stall_rules_hold_without_further_sampling(run_factory, interval, log_fragment, capsys):
    run = run_factory(wait=False)
    commands = []
    run.receive_state(DISTANT)
    run.tick(commands.append)
    before = run.progress_snapshot()
    assert before["execution_state"] == "tracking"
    assert before["feedrate_m_s"] <= .001
    assert before["online_cross_track_error_m"] > .005
    if interval:
        time.sleep(interval)
    run.receive_state(DISTANT)
    run.tick(commands.append)
    assert log_fragment in capsys.readouterr().err
    assert run.execution_state == "holding"
    assert run.progress_snapshot()["termination"] == "held"
    count = len(commands)
    run.tick(commands.append)
    assert len(commands) == count
    run.receive_state(DISTANT)
    for _ in range(4):
        run.tick(commands.append)
    assert len(commands) == count + 4
    assert run.tracking_report().total_steps == 2
    assert run.start() == "ALREADY_TRACKING"
    run.finish("interrupted")
    run.close()
    assert run.tracking_report().termination == "held"
    assert run.report_status()["state"] == "saved"
    alpha = .01 / (.01 + .02)
    np.testing.assert_allclose(commands[1], commands[0] + alpha * (DISTANT - commands[0]))


def test_filesystem_failure_preserves_terminal_reason(run_factory, tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("existing content")
    run = run_factory(wait=False, destination=blocker / "perf.md")
    commands = []
    run.receive_state(START)
    run.tick(commands.append)
    run.close()
    assert run.report_status()["state"] == "failed"
    assert run.report_status()["error"]
    assert run.progress_snapshot()["termination"] == "interrupted"
    assert blocker.read_text() == "existing content"


def test_report_identity_and_explicit_export(run_factory, tmp_path):
    run = run_factory(wait=False)
    commands = []
    q = START.copy()
    for _ in range(5):
        run.receive_state(q)
        run.tick(commands.append)
        q = commands[-1]
    report = run.tracking_report()
    assert report.total_steps == 5
    assert report.evidence.boundary == "kernel_candidate"
    assert report.constraint_metrics["obstacle.residual"]["inactive_count"] == 5
    assert report.metrics["tool_axis_error_rad"].count == 5
    path = Path(run.write_tracking_report(str(tmp_path / "explicit.md")))
    summary = json.loads(path.with_suffix(".json").read_text())
    data = path.with_suffix(".samples.json").read_bytes()
    assert summary["sample_data_sha256"] == hashlib.sha256(data).hexdigest()
    assert len(json.loads(data)["samples"]) == 5


@pytest.fixture(scope="module")
def endpoint_kernel(kernel):
    from work.jax_control_facade import JaxControlLoop
    from work.nineaxis_kinematics import NineaxisKinematics
    from work.path_following import PathFollowingConfig, PathGeometry

    config = kernel[2]
    start_q = START.copy()
    end_q = START.copy()
    end_q[0] += .002
    kinematics = NineaxisKinematics()
    poses = [kinematics.ee_pose(q) for q in (start_q, end_q)]
    geometry = PathGeometry.from_samples(
        np.stack([pose[0] for pose in poses]), np.stack([pose[1] for pose in poses]),
        np.array([.03, .03]), np.array([0., .1]),
    )
    loop = JaxControlLoop(**{name: config[name] for name in (
        "dt", "dt_path", "w_pos", "w_orient", "w_joint", "temporal_lambda",
        "enable_x64", "solver_tol", "task_mode",
    )})
    loop.configure_path(geometry, PathFollowingConfig(
        reference_lead_m=config["reference_lead_m"],
        maximum_tool_axis_speed_rad_s=config["max_tool_axis_speed_rad_s"],
    ))
    loop.init_cbf()
    return (loop, geometry, config, .1), end_q


def test_real_short_path_endpoint_and_hold_report(run_factory, endpoint_kernel):
    resources, end_q = endpoint_kernel
    run = run_factory(wait=False, resources=resources)
    commands = []
    for _ in range(100):
        run.receive_state(end_q)
        run.tick(commands.append)
        if run.execution_state == "holding":
            break
    assert run.execution_state == "holding"
    assert run.progress_snapshot()["termination"] == "completed"
    samples = run.tracking_report().total_steps
    run.receive_state(end_q)
    for _ in range(10):
        run.tick(commands.append)
    assert run.tracking_report().total_steps == samples
    run.close()
    assert run.report_status()["state"] == "saved"
    assert run.tracking_report().termination == "completed"
