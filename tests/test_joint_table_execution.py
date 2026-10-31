import csv
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
import pytest
from work.nineaxis_model import NineaxisModel

from robot_safecontrol_moveit.drempower_joint_control import JointFeedback
from robot_safecontrol_moveit.drempower_serial import DrEmpowerSerial
from robot_safecontrol_moveit.joint_table import TABLE_COLUMNS, JointTable
from robot_safecontrol_moveit.joint_table_execution import JointTableExecutor
from robot_safecontrol_moveit.joint_table_runtime import run_joint_table
from robot_safecontrol_moveit.measurement_files import prepare_measurement_files
from robot_safecontrol_moveit.mujoco_viewer_with_cylinder import MuJoCoJointStateViewer
from robot_safecontrol_moveit.single_joint_execution import (
    MotionRejected,
    ObservedFeedback,
    SingleJointMonitor,
    VelocityWindow,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def model():
    return NineaxisModel.load(root=ROOT)


class MuJoCoObservations:
    def __init__(self, executor):
        self.executor = executor
        scene = ET.fromstring(MuJoCoJointStateViewer._urdf_to_mjcf(
            ROOT / "models/ninezzhou/urdf/ninezzhou.urdf", ROOT / "models/ninezzhou/meshes"))
        actuators = ET.SubElement(scene, "actuator")
        for name in model_names():
            ET.SubElement(actuators, "position", name=f"act_{name}", joint=name, kp="60", kv="10")
        self.physics = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
        self.physics.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.physics.opt.gravity[:] = 0
        self.physics.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        self.data = mujoco.MjData(self.physics)
        self.qpos = [self.physics.joint(name).qposadr[0] for name in model_names()]
        self.qvel = [self.physics.joint(name).dofadr[0] for name in model_names()]
        self.actuators = [self.physics.actuator(f"act_{name}").id for name in model_names()]
        self.max_motion = np.zeros(9)

    def step(self, target=None, trajectory_done=True):
        if target is not None:
            self.data.ctrl[self.actuators] = target
        for _ in range(10):
            mujoco.mj_step(self.physics, self.data)
        q, v = self.data.qpos[self.qpos], self.data.qvel[self.qvel]
        self.max_motion = np.maximum(self.max_motion, np.abs(q))
        return {joint: ObservedFeedback(
            self.executor.control._node(joint), JointFeedback(
                self.executor.control.target_deg(joint, q[index]),
                self.executor.control.target_deg(joint, v[index]) / 6),
            self.data.time - .0009, self.data.time, trajectory_done, False,
        ) for index, joint in enumerate(model_names())}

    def observe(self, **kwargs):
        return self.executor.observe(self.step(**kwargs), now_s=self.data.time)


def model_names():
    return tuple(f"J{index}" for index in range(1, 10))


def session(model, path, kind="camera", rows=None):
    rows = rows if rows is not None else [["a", *([0.] * 9)], ["b", .1, *([.4] * 8)]]
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerows(rows)
    executor = JointTableExecutor(JointTable.load(path, model), model, measurement_kind=kind,
                                  task_id=f"calibration-{kind}", feedback_source="simulation_feedback")
    return executor, MuJoCoObservations(executor)


def ready(executor, plant):
    for _ in range(110):
        plant.observe()
    assert executor.phase == "ready"


def capture(executor, plant, path):
    with path.open("w", newline="") as stream:
        csv.writer(stream).writerows([["simulation_time_s", plant.data.time], plant.data.qpos[plant.qpos]])
    return executor.record_measurement(
        [path], now_s=plant.data.time, session_id=executor.session_id,
        task_id=executor.task_id, pose_id=executor.table.rows[executor.index].pose_id,
        measurement_kind=executor.measurement_kind)


@pytest.mark.parametrize("kind", ["camera", "laser_tracker"])
def test_nine_axis_mujoco_motion_stop_measurement_and_next_row(model, tmp_path, kind):
    executor, plant = session(model, tmp_path / "targets.csv", kind)
    ready(executor, plant)
    first = executor.begin(now_s=plant.data.time)
    assert [entry["arguments"]["id_num"] for entry in first] == list(range(1, 10))
    assert first[0]["arguments"]["speed"] == 3
    assert first[1]["arguments"]["speed"] == .41
    for _ in range(35):
        plant.observe()
    assert executor.phase == "awaiting_measurement"
    for _ in range(10):
        plant.observe()
    capture(executor, plant, tmp_path / "capture-a.csv")
    assert executor.phase == "ready"
    executor.begin(now_s=plant.data.time)
    target = np.asarray(executor.table.rows[1].q_si)
    for step in range(1000):
        # 位置伺服作用于真实 MuJoCo 动力学；反馈读取 qpos/qvel。
        fraction = min((step + 1) / 400, 1.)
        plant.observe(target=target * fraction, trajectory_done=fraction == 1.)
        if executor.phase == "awaiting_measurement":
            break
    assert executor.phase == "awaiting_measurement"
    assert np.all(plant.max_motion > 0)
    capture(executor, plant, tmp_path / "capture-b.csv")
    assert executor.phase == "completed"
    report = executor.report()
    assert len(report["records"]) == 2
    assert all(attempt["status"] == "completed" for attempt in report["attempts"])
    assert all("completed_s" in attempt for attempt in report["attempts"])
    assert all(attempt["feedback_stopped"] for attempt in report["attempts"])
    assert report["attempts"][1]["arrival_elapsed_s"] > 8
    assert report["feedback_source"] == "simulation_feedback"
    assert report["hardware_qualified"] is False
    assert report["physical_stop_independently_verified"] is False
    assert report["acceleration_feedback_validated"] is False
    assert all(abs(value["error_mm_deg"]) <= executor.control.project_position(joint, .1)
               for joint, value in report["records"][-1]["feedback"].items())
    executor.write_report(tmp_path / "report.json")
    assert json.loads((tmp_path / "report.json").read_text())["measurement_kind"] == kind


@pytest.mark.parametrize("failure", ["missing", "identity", "time", "overspeed", "nonfinite"])
def test_failure_aborts_entire_group_and_disallows_next(model, tmp_path, failure):
    executor, plant = session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    samples = plant.step()
    if failure == "missing":
        del samples["J9"]
    elif failure == "identity":
        samples["J9"] = replace(samples["J9"], node_id=8)
    elif failure == "time":
        samples["J9"] = replace(samples["J9"], request_s=0)
    else:
        samples["J9"] = replace(samples["J9"], state=replace(
            samples["J9"].state, angle_deg=float("nan") if failure == "nonfinite" else 50))
    with pytest.raises(MotionRejected):
        executor.observe(samples, now_s=plant.data.time)
    assert executor.phase == "aborted"
    assert executor.report()["attempts"][-1]["stop_result"] == "external_stop_required"
    with pytest.raises(MotionRejected):
        executor.begin(now_s=plant.data.time)


def test_deadline_without_new_feedback_and_no_automatic_retry(model, tmp_path):
    executor, plant = session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    with pytest.raises(MotionRejected, match="feedback_timeout"):
        executor.check_time(plant.data.time + .201)
    with pytest.raises(MotionRejected, match="feedback_timeout"):
        executor.observe(plant.step(), now_s=plant.data.time)


def test_motion_timeout_while_all_axes_continue_reporting(model, tmp_path):
    executor, plant = session(model, tmp_path / "targets.csv", rows=[["unreached", 1., *([1.] * 8)]])
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    with pytest.raises(MotionRejected, match="motion_timeout"):
        for _ in range(1200):
            plant.observe(trajectory_done=False)
    assert executor.phase == "aborted"
    assert executor.index == 0


@pytest.mark.parametrize("failure", ["wrong_session", "wrong_kind", "wrong_task", "missing_file", "stale"])
def test_measurement_requires_own_session_and_fresh_stationary_observation(model, tmp_path, failure):
    executor, plant = session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    for _ in range(35):
        plant.observe()
    fields = {"files": [tmp_path / "capture.csv"], "now_s": plant.data.time,
              "session_id": executor.session_id, "task_id": executor.task_id, "pose_id": "a",
              "measurement_kind": "camera"}
    fields["files"][0].write_text("simulation measurement\n")
    if failure == "wrong_session":
        fields["session_id"] = "other"
    elif failure == "wrong_kind":
        fields["measurement_kind"] = "laser_tracker"
    elif failure == "wrong_task":
        fields["task_id"] = "other"
    elif failure == "missing_file":
        fields["files"] = [tmp_path / "missing"]
    else:
        fields["now_s"] += .201
    with pytest.raises(MotionRejected):
        executor.record_measurement(**fields)
    assert executor.phase == "aborted"
    assert not executor.records


def test_runtime_rejects_before_creating_files_or_transport(model, tmp_path):
    executor, _ = session(model, tmp_path / "targets.csv")
    with pytest.raises(RuntimeError, match="锁定|joint_table_live_disabled"):
        run_joint_table(executor, port="absent", log_path=tmp_path / "run.jsonl", measurement_dir=tmp_path)
    assert not (tmp_path / "run.jsonl").exists()


def test_measurement_timeout_with_continuing_real_physics_feedback(model, tmp_path):
    executor, plant = session(model, tmp_path / "targets.csv")
    executor.measurement_timeout_s = .1
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    with pytest.raises(MotionRejected, match="measurement_timeout"):
        for _ in range(45):
            plant.observe()
    assert not executor.records
    assert executor.index == 0


def test_previous_measurement_file_cannot_advance_next_group(model, tmp_path):
    executor, plant = session(model, tmp_path / "targets.csv",
                              rows=[["a", *([0.] * 9)], ["b", *([0.] * 9)]])
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    for _ in range(35):
        plant.observe()
    capture(executor, plant, tmp_path / "capture.csv")
    executor.begin(now_s=plant.data.time)
    for _ in range(35):
        plant.observe()
    with pytest.raises(MotionRejected, match="measurement_reused"):
        executor.record_measurement(
            [tmp_path / "capture.csv"], now_s=plant.data.time, session_id=executor.session_id,
            task_id=executor.task_id, pose_id="b", measurement_kind="camera")
    assert len(executor.records) == 1
    assert executor.phase == "aborted"


def test_next_group_rejection_preserves_completed_group(model, tmp_path):
    executor, plant = session(model, tmp_path / "targets.csv",
                              rows=[["a", *([0.] * 9)], ["b", 51., *([0.] * 8)]])
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    for _ in range(35):
        plant.observe()
    capture(executor, plant, tmp_path / "capture.csv")
    saved = dict(executor.attempts[0])
    with pytest.raises(MotionRejected, match="step_limit"):
        executor.begin(now_s=plant.data.time)
    assert len(executor.attempts) == 2
    assert executor.attempts[0] == saved
    assert saved["status"] == "completed"
    assert executor.records[0]["execution"]["status"] == "completed"
    assert executor.attempts[1]["pose_id"] == "b"
    assert executor.attempts[1]["status"] == "aborted"
    assert executor.attempts[1]["feedback"]["J1"]["error_mm_deg"] == pytest.approx(-51.)


@pytest.mark.parametrize("invalid_time", [float("nan"), float("inf"), -1.])
def test_invalid_begin_time_retains_serializable_rejection_report(model, tmp_path, invalid_time):
    executor, plant = session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    with pytest.raises(MotionRejected, match="joint_table_clock_invalid"):
        executor.begin(now_s=invalid_time)
    executor.write_report(tmp_path / "report.json")
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["attempts"][0]["started_s"] is None
    assert report["attempts"][0]["status"] == "aborted"
    assert report["attempts"][0]["rejection_reason"] == "joint_table_clock_invalid"


@pytest.mark.parametrize("failure", ["expired_feedback", "file_changed"])
def test_prepared_file_acceptance_rechecks_feedback_and_file_identity(model, tmp_path, failure):
    executor, plant = session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    for _ in range(35):
        plant.observe()
    source = tmp_path / "capture.csv"
    source.write_text(f"模拟测量时间,{plant.data.time}\n")
    prepared = prepare_measurement_files([source])
    if failure == "file_changed":
        source.write_text(f"模拟测量时间,{plant.data.time},文件更新\n")
    now_s = plant.data.time + (.201 if failure == "expired_feedback" else 0)
    with pytest.raises(MotionRejected, match="feedback_timeout|files_changed"):
        executor.record_prepared_measurement(
            prepared, now_s=now_s, session_id=executor.session_id, task_id=executor.task_id,
            pose_id="a", measurement_kind="camera")
    assert executor.phase == "aborted"
    assert not executor.records


def test_offline_cli_does_not_load_hardware_extras():
    environment = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
    subprocess.run([
        sys.executable, "-c",
        ("import sys; import robot_safecontrol_moveit.joint_table_cli; "
         "assert 'serial' not in sys.modules; assert 'construct' not in sys.modules"),
    ], env=environment, check=True, capture_output=True, text=True)


def test_long_stationary_estimator_rejects_slow_physical_oscillation(model, tmp_path):
    executor, plant = session(model, tmp_path / "oscillating.csv")
    executor.limits = {joint: replace(limit, stationary_window_s=5., stationary_velocity_noise_rpm=.1)
                       for joint, limit in executor.limits.items()}
    executor.windows = {joint: VelocityWindow(limit.velocity_window_s, limit.stationary_window_s)
                        for joint, limit in executor.limits.items()}
    executor.monitors = {joint: SingleJointMonitor(executor.control._node(joint), limit)
                         for joint, limit in executor.limits.items()}
    with pytest.raises(MotionRejected, match="initial_stationary_timeout"):
        for _ in range(400):
            target = np.zeros(9)
            target[-1] = np.deg2rad(.03) * np.sin(plant.data.time * np.pi / 2)
            plant.observe(target=target)
    assert executor.phase == "aborted"
    assert np.rad2deg(plant.max_motion[-1]) > .02
    assert not executor.attempts


@pytest.mark.parametrize("nodes", [{}, {1: "prismatic"}, {**{i: "revolute" for i in range(1, 10)}},
                                    {1: "prismatic", **{i: "revolute" for i in range(2, 11)}}])
def test_multi_node_permission_is_checked_before_serial_open(tmp_path, nodes):
    with pytest.raises(ValueError, match="九轴位置连接"):
        DrEmpowerSerial(str(tmp_path), position_nodes=nodes)
