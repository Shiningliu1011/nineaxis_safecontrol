import subprocess
import sys
from dataclasses import replace
from math import factorial
from pathlib import Path

import jax
import jax.numpy as jnp
import jsonschema
import mujoco
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import URDF
from work.nineaxis_model import (
    ModelCalibration,
    NineaxisModel,
    RigidTransform,
    tool_motion_jax,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def model():
    return NineaxisModel.load(root=ROOT)


@pytest.fixture(scope="module")
def independent_model():
    path = ROOT / "models/ninezzhou/urdf/ninezzhou.urdf"
    assets = {entry.name: entry.read_bytes() for entry in (path.parents[1] / "meshes").glob("*.STL")}
    spec = mujoco.MjSpec.from_string(path.read_text(encoding="utf-8"), assets=assets)
    spec.compiler.fusestatic = False
    return spec.compile()


def independent_pose(model, q, body="tool0", correction=None):
    data = mujoco.MjData(model)
    addresses = [model.joint(f"J{index}").qposadr[0] for index in range(1, 10)]
    data.qpos[addresses] = q
    mujoco.mj_forward(model, data)
    body_id = model.body(body).id
    transform = np.eye(4)
    transform[:3, :3] = data.xmat[body_id].reshape(3, 3)
    transform[:3, 3] = data.xpos[body_id]
    if correction is not None:
        transform = transform @ correction
    return transform, data


def test_model_sources_joint_meaning_and_raw_position_limits(model):
    description = URDF.from_xml_string((ROOT / "models/ninezzhou/urdf/ninezzhou.urdf").read_bytes())
    assert model.joint_names == tuple(f"J{index}" for index in range(1, 10))
    assert model.joint_types == ("prismatic",) + ("revolute",) * 8
    assert model.joint_units == ("m",) + ("rad",) * 8
    assert model.base_frame == "base_link"
    assert model.tcp_frame == "tool0"
    np.testing.assert_array_equal(model.tool_axis_local, [1, 0, 0])
    for index, name in enumerate(model.joint_names):
        joint = description.joint_map[name]
        assert joint.joint_type == model.joint_types[index]
        assert model.position_lower[index] == joint.limit.lower
        assert model.position_upper[index] == joint.limit.upper
    assert len(model.mesh_sources) == 10
    assert len(model.self_collision_exclusions) == 9
    assert model.missing_calibrations == ("workpiece", "joint_mapping", "physical_tcp")
    assert not model.hardware_validated
    with pytest.raises(RuntimeError, match="physical_tcp"):
        model.require_hardware_validated()


@pytest.mark.parametrize("axis", range(9))
def test_each_axis_fk_link_poses_and_jacobian_match_mujoco(model, independent_model, axis):
    midpoint = (model.position_lower + model.position_upper) / 2
    for value in (model.position_lower[axis] + 1e-6, midpoint[axis], model.position_upper[axis] - 1e-6):
        q = midpoint.copy()
        q[axis] = value
        expected, data = independent_pose(independent_model, q)
        np.testing.assert_allclose(model.fk(q), expected, atol=2e-12, rtol=0)
        links = model.link_poses(q)
        for name in ("base_link",) + tuple(f"Link{index}" for index in range(1, 10)) + ("tool0",):
            expected_link, _ = independent_pose(independent_model, q, name)
            np.testing.assert_allclose(links[name], expected_link, atol=2e-12, rtol=0)
        jacp, jacr = np.zeros((3, 9)), np.zeros((3, 9))
        mujoco.mj_jacBody(independent_model, data, jacp, jacr, independent_model.body("tool0").id)
        np.testing.assert_allclose(model.jacobian(q), np.vstack([jacp, jacr]), atol=3e-12, rtol=0)
        epsilon = 1e-6
        delta = np.eye(9)[axis] * epsilon
        positive, _ = independent_pose(independent_model, q + delta)
        negative, _ = independent_pose(independent_model, q - delta)
        axis_derivative = (positive[:3, 0] - negative[:3, 0]) / (2 * epsilon)
        np.testing.assert_allclose(model.tool_axis_jacobian(q)[:, axis], axis_derivative, atol=5e-10, rtol=0)


def test_model_rejects_dimensions_units_nonfinite_and_stale_identity(model):
    for q in (np.zeros(8), np.zeros((1, 9)), np.full(9, np.nan), np.full(9, np.inf)):
        with pytest.raises(ValueError):
            model.fk(q)
    with pytest.raises(ValueError, match="units"):
        model.fk(np.zeros(9), units="mm_deg")
    with pytest.raises(ValueError, match="model_hash"):
        model.fk(np.zeros(9), model_hash="a" * 64)
    with pytest.raises(ValueError, match="joint_names"):
        model.fk(np.zeros(9), joint_names=tuple(reversed(model.joint_names)))
    with pytest.raises(ValueError, match="expected_hash"):
        NineaxisModel.load(root=ROOT, expected_hash="a" * 64)


def test_tcp_workpiece_source_axis_and_calibration_identity(model, independent_model):
    tcp_matrix = np.eye(4)
    tcp_matrix[:3, :3] = Rotation.from_euler("xyz", [.1, -.2, .3]).as_matrix()
    tcp_matrix[:3, 3] = [.01, -.02, .03]
    workpiece_matrix = np.eye(4)
    workpiece_matrix[:3, :3] = Rotation.from_euler("xyz", [-.2, .3, -.1]).as_matrix()
    workpiece_matrix[:3, 3] = [.2, -.3, .4]
    tcp = RigidTransform("tcp", "tool0", tcp_matrix, "analytic-tcp-v1", "analytic SE(3) check", "simulation",
                         applicable_model_hash=model.asset_model_hash)
    workpiece = RigidTransform("workpiece", "base_link", workpiece_matrix, "analytic-workpiece-v1", "analytic SE(3) check", "simulation",
                              applicable_model_hash=model.asset_model_hash)
    calibrated = model.with_calibration(ModelCalibration("analytic-v1", tcp, workpiece, None,
        applicable_model_hash=model.asset_model_hash))
    assert calibrated.model_hash != model.model_hash
    q = (model.position_lower + model.position_upper) / 2
    expected, _ = independent_pose(independent_model, q, correction=tcp_matrix)
    np.testing.assert_allclose(calibrated.fk(q), expected, atol=2e-12, rtol=0)
    source_rotation = Rotation.from_euler("xyz", [.3, -.4, .2]).as_matrix()
    source_position = np.array([.1, .2, .3])
    result = calibrated.source_pose_to_base(source_position, source_rotation)
    np.testing.assert_allclose(result[:3, 3], workpiece_matrix[:3, :3] @ source_position + workpiece_matrix[:3, 3], atol=1e-14)
    np.testing.assert_allclose(result[:3, 0], workpiece_matrix[:3, :3] @ source_rotation[:, 2], atol=1e-14)
    with pytest.raises(RuntimeError, match="workpiece"):
        model.source_pose_to_base(source_position, source_rotation)
    tcp_matrix[:] = 0
    np.testing.assert_allclose(calibrated.fk(q), expected, atol=2e-12, rtol=0)
    with pytest.raises(ValueError):
        calibrated.calibration.tcp_correction.matrix.setflags(write=True)
    bad = np.eye(4)
    bad[0, 0] = 2
    with pytest.raises(ValueError, match=r"SE\(3\)"):
        replace(workpiece, matrix=bad)


@pytest.mark.parametrize("active_axis", [None, *range(9)])
def test_actual_tool_motion_three_derivatives_match_independent_time_samples(model, independent_model, active_axis):
    correction = np.eye(4)
    correction[:3, :3] = Rotation.from_euler("xyz", [.2, -.1, .3]).as_matrix()
    correction[:3, 3] = [.012, -.024, .036]
    calibrated = model.with_calibration(ModelCalibration("derivatives-v1", RigidTransform(
        "tcp", "tool0", correction, "derivatives-tcp-v1", "analytic SE(3) check", "simulation",
        applicable_model_hash=model.asset_model_hash,
    ), None, None, applicable_model_hash=model.asset_model_hash))
    q = (model.position_lower + model.position_upper) / 2
    q += [.01, .2, -.3, .1, -.2, .15, .22, -.18, .12]
    velocity = np.array([.02, .13, -.12, .11, -.15, .09, -.07, .08, -.06])
    acceleration = np.array([.03, -.11, .12, -.08, .07, .1, -.09, .05, .06])
    jerk = np.array([-.04, .12, .08, -.09, .11, -.13, .06, -.07, .05])
    if active_axis is not None:
        selector = np.eye(9)[active_axis]
        velocity, acceleration, jerk = (value * selector for value in (velocity, acceleration, jerk))
    motion = calibrated.tool_motion(q, velocity, acceleration, jerk, sample_source="simulation_feedback",
                                    time_s=1.2, clock_id="simulation-clock-v1")
    # MuJoCo 的位姿样本与多项式拟合独立提供前三阶导数。
    times = np.arange(-4, 5) * .003
    samples = [independent_pose(independent_model, q + velocity * time + acceleration * time**2 / 2
                               + jerk * time**3 / 6, correction=correction)[0] for time in times]
    coefficients = np.polynomial.polynomial.polyfit(times, np.asarray(samples).reshape(9, -1), 7)
    expected = np.asarray([coefficients[index].reshape(4, 4) * factorial(index) for index in range(4)])
    for order, tolerance in enumerate((3e-12, 3e-11, 3e-8, 3e-6)):
        np.testing.assert_allclose(motion.transform_derivatives[order], expected[order], atol=tolerance, rtol=0)
    np.testing.assert_allclose(motion.position_m, expected[0, :3, 3], atol=3e-12)
    np.testing.assert_allclose(motion.velocity_m_s, expected[1, :3, 3], atol=3e-11)
    np.testing.assert_allclose(motion.acceleration_m_s2, expected[2, :3, 3], atol=3e-8)
    np.testing.assert_allclose(motion.jerk_m_s3, expected[3, :3, 3], atol=3e-6)
    for order in range(4):
        np.testing.assert_allclose(motion.axis_derivatives[order], expected[order, :3, 0], atol=3e-6, rtol=0)
    rotation, first, second, third = expected[:, :3, :3]
    angular_matrices = (first @ rotation.T, second @ rotation.T + first @ first.T,
                        third @ rotation.T + 2 * second @ first.T + first @ second.T)
    angular = np.asarray([[(matrix[2, 1] - matrix[1, 2]) / 2,
                           (matrix[0, 2] - matrix[2, 0]) / 2,
                           (matrix[1, 0] - matrix[0, 1]) / 2] for matrix in angular_matrices])
    np.testing.assert_allclose(motion.angular_derivatives, angular, atol=3e-6, rtol=0)
    assert motion.sample_source == "simulation_feedback"
    assert motion.time_s == 1.2 and motion.model_hash == calibrated.model_hash
    assert motion.clock_id == "simulation-clock-v1"
    assert np.linalg.norm(motion.axis_derivatives[1]) <= np.linalg.norm(motion.angular_derivatives[0])
    with pytest.raises(ValueError):
        motion.jerk_m_s3.setflags(write=True)


def test_actual_tool_motion_requires_valid_identity_units_and_derivatives(model):
    for bad in (np.zeros(8), np.full(9, np.nan)):
        with pytest.raises(ValueError):
            model.tool_motion(np.zeros(9), bad, np.zeros(9), np.zeros(9),
                              sample_source="simulation_feedback", time_s=0., clock_id="simulation-v1")
    with pytest.raises(ValueError, match="sample_source"):
        model.tool_motion(*([np.zeros(9)] * 4), sample_source="command_echo", time_s=0., clock_id="clock")
    with pytest.raises(ValueError, match="time_s"):
        model.tool_motion(*([np.zeros(9)] * 4), sample_source="measured_feedback", time_s=np.inf, clock_id="clock")
    with pytest.raises(ValueError, match="units"):
        model.tool_motion(*([np.zeros(9)] * 4), sample_source="simulation_feedback", time_s=0., clock_id="clock", units="mm_deg")


def test_model_config_and_transform_units_are_explicit(model, tmp_path):
    with pytest.raises(ValueError, match="translation_unit"):
        replace(model.calibration.tcp_correction, translation_unit="mm")
    original = yaml.safe_load((ROOT / "portable_oscbf/config/nineaxis_model.yaml").read_text())
    for key, value in (("schema_version", 2), ("joint_units", ["mm"] + ["deg"] * 8)):
        changed = {**original, key: value}
        path = tmp_path / "model.yaml"
        path.write_text(yaml.safe_dump(changed))
        with pytest.raises((ValueError, jsonschema.ValidationError)):
            NineaxisModel.load(root=ROOT, config_path=path)
    original["sources"]["urdf"]["sha256"] = "0" * 64
    path = tmp_path / "changed-source.yaml"
    path.write_text(yaml.safe_dump(original))
    with pytest.raises(ValueError, match="source hash"):
        NineaxisModel.load(root=ROOT, config_path=path)


def test_predictive_jax_parameters_use_the_common_tcp_and_explicit_finiteness(model):
    q = jnp.asarray((model.position_lower + model.position_upper) / 2)
    zero = jnp.zeros(9)
    matrices, finite = tool_motion_jax(q, zero, zero, zero, *model.jax_parameters)
    assert bool(finite) and matrices.shape == (4, 4, 4)
    np.testing.assert_allclose(matrices[0], model.fk(q), atol=1e-14)
    _, finite = tool_motion_jax(q.at[0].set(jnp.nan), zero, zero, zero, *model.jax_parameters)
    assert not bool(finite)
    _, finite = tool_motion_jax(q.astype(jnp.float32), zero.astype(jnp.float32), zero.astype(jnp.float32),
                              zero.astype(jnp.float32), *model.jax_parameters)
    assert not bool(finite)


def test_independent_model_entry_establishes_float64_and_rejects_later_precision_changes(model):
    result = subprocess.run([sys.executable, str(ROOT / "tests/data/nineaxis_precision_entry.py")],
                            capture_output=True, text=True, check=True)
    assert '"float64"' in result.stdout
    previous = jax.config.x64_enabled
    jax.config.update("jax_enable_x64", False)
    try:
        with pytest.raises(RuntimeError, match="float64"):
            model.fk(np.zeros(9))
    finally:
        jax.config.update("jax_enable_x64", previous)


def test_calibration_records_alone_do_not_supply_additional_tool_geometry(model):
    correction = replace(model.calibration.tcp_correction, verification_state="measured_verified",
                         position_residual_mm=0., angle_residual_deg=0.)
    workpiece = RigidTransform("workpiece", "base_link", np.eye(4), "schema-check-v1", "SE(3) schema check",
                               "measured_verified", 0., 0., applicable_model_hash=model.asset_model_hash)
    calibrated = model.with_calibration(ModelCalibration("schema-check-v1", correction, workpiece, "mapping schema check",
        applicable_model_hash=model.asset_model_hash))
    assert calibrated.missing_calibrations == ()
    assert not calibrated.hardware_validated
    with pytest.raises(RuntimeError, match="tool_collision_geometry"):
        calibrated.require_hardware_validated()


@pytest.mark.parametrize("changed_field", ["revision", "nominal_tcp"])
def test_calibration_records_bind_to_a_stable_model_asset_identity(model, tmp_path, changed_field):
    applicable = model.asset_model_hash
    calibration = replace(model.calibration, tcp_correction=replace(model.calibration.tcp_correction,
        verification_state="measured_verified", position_residual_mm=.1, angle_residual_deg=.01,
        source="measurement-record identity schema check"))
    calibrated = model.with_calibration(calibration)
    assert calibrated.asset_model_hash == applicable
    assert calibrated.model_hash != model.model_hash
    assert calibration.applicable_model_hash == applicable
    record = calibration.to_record()
    assert record["applicable_model_hash"] == applicable
    assert record["tcp_correction"]["applicable_model_hash"] == applicable
    assert record["tcp_correction"]["verification_state"] == "measured_verified"
    assert record["tcp_correction"]["position_residual_mm"] == .1
    assert NineaxisModel.load(root=ROOT, calibration=calibration).model_hash == calibrated.model_hash
    assert calibrated.with_calibration(calibration).model_hash == calibrated.model_hash
    values = yaml.safe_load((ROOT / "portable_oscbf/config/nineaxis_model.yaml").read_text())
    if changed_field == "revision":
        values["revision"] += "-different"
    else:
        values["nominal_tcp"]["matrix"][0][3] = .001
        values["nominal_tcp"]["source"] = "analytic 1 mm TCP variant for identity validation"
        values["nominal_tcp"]["revision"] = "analytic-1mm-v1"
    path = tmp_path / "model-variant.yaml"
    path.write_text(yaml.safe_dump(values))
    variant = NineaxisModel.load(root=ROOT, config_path=path)
    assert variant.asset_model_hash != applicable
    with pytest.raises(ValueError, match="applicable_model_hash"):
        variant.with_calibration(calibration)
    with pytest.raises(ValueError, match="applicable_model_hash"):
        NineaxisModel.load(root=ROOT, config_path=path, calibration=calibration)
    with pytest.raises(ValueError, match="applicable_model_hash"):
        replace(model, revision=model.revision + "-different")


def test_calibration_container_rejects_transform_records_from_another_model(model):
    applicable = model.asset_model_hash
    assert model.calibration.applicable_model_hash == applicable
    with pytest.raises(ValueError, match="applicable_model_hash"):
        replace(model.calibration, tcp_correction=replace(model.calibration.tcp_correction, applicable_model_hash="0" * 64))
    mismatched_workpiece = replace(model.calibration.tcp_correction, source_frame="workpiece", target_frame="base_link",
                                  applicable_model_hash="0" * 64)
    with pytest.raises(ValueError, match="applicable_model_hash"):
        replace(model.calibration, workpiece=mismatched_workpiece)
    with pytest.raises(TypeError, match="applicable_model_hash"):
        RigidTransform("tcp", "tool0", np.eye(4), "unbound-v1", "SE(3) record check", "simulation")
    wrong_scope = replace(model.calibration, applicable_model_hash=model.model_hash,
        tcp_correction=replace(model.calibration.tcp_correction, applicable_model_hash=model.model_hash))
    with pytest.raises(ValueError, match="applicable_model_hash"):
        model.with_calibration(wrong_scope)
