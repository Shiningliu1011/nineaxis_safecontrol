import csv
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import struct
import sys

import mujoco
import numpy as np
import pytest
from work.nineaxis_model import NineaxisModel

from robot_safecontrol_moveit.hardware_contract import HardwareState
from robot_safecontrol_moveit.joint_table import JointTable, JointTableProgress, TABLE_COLUMNS
from robot_safecontrol_moveit.mujoco_viewer_with_cylinder import MuJoCoJointStateViewer
from robot_safecontrol_moveit.single_joint_execution import MotionRejected


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def model():
    return NineaxisModel.load(root=ROOT)


def write_table(path, values):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerows(values)
    return path


def test_joint_table_units_identity_and_nominal_fk(model):
    source = ROOT / "config/calibration_joint_table.example.csv"
    table = JointTable.load(source, model)
    assert table.source_sha256 == sha256(source.read_bytes()).hexdigest()
    np.testing.assert_allclose(table.rows[1].q_si, [.01, np.deg2rad(5), *([0] * 7)])
    camera = table.preview(model, measurement_kind="camera")
    tracker = table.preview(model, measurement_kind="laser_tracker")
    assert camera["rows"] == tracker["rows"]
    assert camera["measurement_kind"] != tracker["measurement_kind"]
    assert camera["hardware_executable"] is False
    assert camera["collision_checked"] is False
    assert camera["position_mode"] == "absolute_trapezoid"
    np.testing.assert_allclose(camera["rows"][1]["nominal_tcp_matrix_m"], model.fk(table.rows[1].q_si))
    with pytest.raises(ValueError, match="model_hash"):
        replace(table, model_hash="0" * 64).preview(model, measurement_kind="camera")


def test_table_device_targets_use_existing_absolute_position_protocol(model):
    from robot_safecontrol_moveit.j1_execution import J1ScrewControl
    from robot_safecontrol_moveit.drempower_joint_control import DrEmpowerJointControl

    table = JointTable.load(ROOT / "config/calibration_joint_table.example.csv", model)
    report = table.preview(model, measurement_kind="camera")
    linear = J1ScrewControl.from_root(ROOT)
    rotary = DrEmpowerJointControl(ROOT / "config/hardware_joint_zero.yaml", linear.converter.node_map)
    for row, source in zip(report["rows"], table.rows):
        assert len(row["device_commands"]) == 9
        for index, command in enumerate(row["device_commands"]):
            joint = model.joint_names[index]
            converter = linear.converter if index == 0 else rotary.converter
            expected = converter.joint_to_encoder(joint, source.q_si[index])
            angle, speed_code, accel_code = struct.unpack("<fhh", bytes.fromhex(command["payload_hex"]))
            assert command["joint"] == joint
            assert command["frame_id"] == ((index + 1) << 5) | 0x1A
            assert command["arguments"]["mode"] == 1
            assert angle == pytest.approx(expected, abs=1e-4)
            assert speed_code == (300 if index == 0 else 41)
            assert accel_code == (60 if index == 0 else 5)
    assert report["rows"][1]["device_commands"][0]["arguments"]["angle"] == -720.
    assert report["zero_calibration_sha256"] == sha256((ROOT / "config/hardware_joint_zero.yaml").read_bytes()).hexdigest()


@pytest.mark.parametrize("row", [
    ["bad", 501, *([0] * 8)], ["bad", -1, *([0] * 8)],
    ["bad", 0, 1000, *([0] * 7)], ["bad", float("nan"), *([0] * 8)],
    ["", *([0] * 9)], ["bad", *([0] * 8)], ["bad", *([0] * 10)],
])
def test_invalid_table_refused_at_input(model, tmp_path, row):
    path = write_table(tmp_path / "invalid.csv", [row])
    with pytest.raises(ValueError):
        JointTable.load(path, model)


def test_empty_duplicate_or_unitless_table_refused(model, tmp_path):
    path = write_table(tmp_path / "empty.csv", [])
    with pytest.raises(ValueError, match="至少"):
        JointTable.load(path, model)
    write_table(path, [["same", *([0] * 9)]] * 2)
    with pytest.raises(ValueError, match="重复"):
        JointTable.load(path, model)
    path.write_text("pose_id,J1,J2,J3,J4,J5,J6,J7,J8,J9\na,0,0,0,0,0,0,0,0,0\n")
    with pytest.raises(ValueError, match="列顺序"):
        JointTable.load(path, model)


@pytest.mark.parametrize("measurement_kind", ["camera", "laser_tracker"])
def test_real_mujoco_feedback_sequences_separate_measurement_sessions(model, tmp_path, measurement_kind):
    table_path = write_table(tmp_path / "repeated-pose.csv", [
        ["capture_1", *([0] * 9)], ["capture_2", *([0] * 9)],
    ])
    table = JointTable.load(table_path, model)
    session = JointTableProgress(table, model, ROOT, measurement_kind=measurement_kind,
                                 feedback_source="simulation_feedback")
    physics = mujoco.MjModel.from_xml_string(MuJoCoJointStateViewer._urdf_to_mjcf(
        ROOT / "models/ninezzhou/urdf/ninezzhou.urdf", ROOT / "models/ninezzhou/meshes"))
    physics.opt.gravity[:] = 0
    physics.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    data = mujoco.MjData(physics)
    qpos = [physics.joint(name).qposadr[0] for name in model.joint_names]
    qvel = [physics.joint(name).dofadr[0] for name in model.joint_names]
    assert session.begin().pose_id == "capture_1"
    with pytest.raises(RuntimeError, match="下一组"):
        session.begin()
    with pytest.raises(RuntimeError, match="持续到位"):
        session.record_measurement([table_path], now_s=0)
    for index in range(2):
        for _ in range(300):
            mujoco.mj_step(physics, data)
            state = HardwareState(data.qpos[qpos], data.qvel[qvel], data.time,
                                  feedback_ok=True, mode="sim")
            session.observe(state, now_s=data.time, model_hash=model.model_hash)
        assert session.phase == "awaiting_measurement"
        record = session.record_measurement([table_path], now_s=data.time)
        assert record["pose_id"] == f"capture_{index + 1}"
        assert record["measurement_kind"] == measurement_kind
        assert record["feedback_source"] == "simulation_feedback"
        assert record["files"][0]["sha256"] == table.source_sha256
        if index == 0:
            assert session.phase == "ready"
            for _ in range(300):
                mujoco.mj_step(physics, data)
                state = HardwareState(data.qpos[qpos], data.qvel[qvel], data.time,
                                      feedback_ok=True, mode="sim")
                assert not session.observe(state, now_s=data.time, model_hash=model.model_hash)
            assert session.phase == "ready"
            assert session.begin().pose_id == "capture_2"
    assert session.phase == "completed"
    report_path = tmp_path / f"{measurement_kind}.json"
    session.write_report(report_path)
    report = json.loads(report_path.read_text())
    assert report["measurement_kind"] == measurement_kind
    assert report["phase"] == "completed"
    assert len(report["records"]) == 2
    assert report["hardware_executable"] is False
    with pytest.raises(FileExistsError):
        session.write_report(report_path)
    with pytest.raises(RuntimeError, match="下一组"):
        session.begin()


def test_live_table_execution_rejected_before_any_transport(model, tmp_path):
    table = JointTable.load(ROOT / "config/calibration_joint_table.example.csv", model)
    with pytest.raises(RuntimeError, match="joint_table_live_disabled"):
        JointTableProgress(table, model, ROOT, measurement_kind="camera",
                           feedback_source="measured_feedback")
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.joint_table_cli",
        "--root", str(tmp_path), "--input", "absent.csv", "--output", str(tmp_path / "out.json"),
        "--measurement-kind", "laser_tracker", "--execute",
    ], capture_output=True, text=True)
    assert result.returncode != 0
    assert "joint_table_live_disabled" in result.stderr
    assert not (tmp_path / "out.json").exists()


def test_table_cli_preflight_exports_without_hardware(tmp_path):
    output = tmp_path / "preflight.json"
    subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.joint_table_cli", "--root", str(ROOT),
        "--input", str(ROOT / "config/calibration_joint_table.example.csv"),
        "--measurement-kind", "camera", "--output", str(output),
    ], check=True, capture_output=True, text=True)
    report = json.loads(output.read_text())
    assert report["status"] == "targets_validated"
    assert report["execution"] == "not_started"
    assert len(report["rows"]) == 2


@pytest.mark.parametrize("change,reason", [
    ({"mode": "live"}, "feedback_untrusted"),
    ({"feedback_ok": False}, "feedback_untrusted"),
    ({"watchdog_ok": False}, "feedback_untrusted"),
    ({"q": np.full(9, np.nan)}, "feedback_invalid"),
    ({"qdot": np.ones(9)}, "velocity_limit"),
    ({"stamp_s": -1.0}, "time_invalid"),
])
def test_table_feedback_failure_latches_abort(model, tmp_path, change, reason):
    source = write_table(tmp_path / "origin.csv", [["origin", *([0] * 9)]])
    session = JointTableProgress(JointTable.load(source, model), model, ROOT,
                                 measurement_kind="camera", feedback_source="simulation_feedback")
    session.begin()
    state = HardwareState(np.zeros(9), np.zeros(9), 1, feedback_ok=True, mode="sim")
    with pytest.raises(MotionRejected, match=reason):
        session.observe(replace(state, **change), now_s=1, model_hash=model.model_hash)
    assert session.phase == "aborted"
    with pytest.raises(RuntimeError, match="下一组"):
        session.begin()


def test_measurement_requires_continuous_fresh_stationary_feedback(model, tmp_path):
    source = write_table(tmp_path / "origin.csv", [["origin", *([0] * 9)]])
    table = JointTable.load(source, model)
    for failure in ("feedback_gap", "pose_changed", "measurement_timeout"):
        session = JointTableProgress(table, model, ROOT, measurement_kind="laser_tracker",
                                     feedback_source="simulation_feedback")
        session.begin()
        for stamp in np.arange(1, 1.62, .02):
            session.observe(HardwareState(np.zeros(9), np.zeros(9), stamp, feedback_ok=True, mode="sim"),
                            now_s=stamp, model_hash=model.model_hash)
        assert session.phase == "awaiting_measurement"
        with pytest.raises(MotionRejected):
            if failure == "measurement_timeout":
                session.record_measurement([source], now_s=2)
            else:
                q = np.zeros(9)
                if failure == "pose_changed":
                    q[1] = np.deg2rad(.11)
                stamp = 2 if failure == "feedback_gap" else 1.62
                session.observe(HardwareState(q, np.zeros(9), stamp, feedback_ok=True, mode="sim"),
                                now_s=stamp, model_hash=model.model_hash)
        assert session.phase == "aborted"
