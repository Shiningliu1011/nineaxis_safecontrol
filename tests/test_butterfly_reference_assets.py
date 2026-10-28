import json
import os
import shutil
import subprocess
import sys
from copy import deepcopy
from hashlib import sha256
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.io import loadmat, savemat
from work.continuous_path_reference import load_reference_bundle, reference_identity
from work.ik_data_loader import load_repository_trajectory

from robot_safecontrol_moveit import robot_spec
from robot_safecontrol_moveit.pipe_assets.butterfly_reference import generate_reference

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path(os.environ.get("BUTTERFLY_SOURCE_DIR", ROOT / "data/nurbs/butterfly_reference_v1/production/source"))


@pytest.fixture(scope="module")
def reference_variants(tmp_path_factory):
    inputs = tmp_path_factory.mktemp("reference-variants")
    shutil.copyfile(SOURCE / "ik_input.mat", inputs / "ik_input.mat")
    default_config = ROOT / "portable_oscbf/config/nineaxis.yaml"
    shifted_config = inputs / "shifted-kinematics.yaml"
    config = yaml.safe_load(default_config.read_text())
    config["kinematics"]["ee_center"] = [0., .344, 1.387]
    shifted_config.write_text(yaml.safe_dump(config))
    bundles = inputs / "butterfly_reference_v1"
    manifests = {}
    for name, kinematics in (("a-default", default_config), ("b-shifted", shifted_config)):
        manifests[name] = generate_reference(
            SOURCE, kinematics, ROOT / "config/oscbf_controller.yaml", bundles / name,
        )
    return inputs, shifted_config, manifests


def test_reference_discovery_selects_the_matching_transform(reference_variants):
    inputs, shifted_config, manifests = reference_variants
    np.testing.assert_allclose(
        manifests["a-default"]["description"]["projection_radii_m"],
        manifests["b-shifted"]["description"]["projection_radii_m"], atol=2e-14, rtol=0,
    )
    assert all(manifest["validation"]["source_points"] == 14992 for manifest in manifests.values())
    trajectory = load_repository_trajectory(str(inputs / "ik_input.mat"), str(shifted_config), 3.5)
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    with pytest.raises(ValueError, match="变换或圆柱身份不匹配"):
        trajectory.attach_continuous_reference(inputs / "butterfly_reference_v1/a-default")
    geometry = trajectory.path_geometry(require_continuous=True)
    assert geometry.reference_identity == manifests["b-shifted"]["identity"]
    expected = load_reference_bundle(inputs / "butterfly_reference_v1/b-shifted").reference
    for fraction in (0., .1734, .5, 1.):
        np.testing.assert_allclose(geometry.sample(fraction * geometry.total_length_m).position_m,
                                   expected.query(fraction).position_m, atol=2e-14, rtol=0)


def test_reference_discovery_checks_model_and_selected_asset_integrity(reference_variants, tmp_path):
    inputs, shifted_config, manifests = reference_variants
    shutil.copyfile(inputs / "ik_input.mat", tmp_path / "ik_input.mat")
    bundles = tmp_path / "butterfly_reference_v1"
    for name in ("a-other-model", "b-matching"):
        shutil.copytree(inputs / "butterfly_reference_v1/b-shifted", bundles / name)
    manifest_path = bundles / "a-other-model/manifest.json"
    other_model = json.loads(manifest_path.read_text())
    other_model["description"]["model_identity"]["kinematics_data_sha256"] = "0" * 64
    other_model["identity"] = reference_identity(other_model["description"])
    manifest_path.write_text(json.dumps(other_model))
    trajectory = load_repository_trajectory(str(tmp_path / "ik_input.mat"), str(shifted_config), 3.5)
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    with pytest.raises(ValueError, match="model 身份不匹配"):
        trajectory.attach_continuous_reference(bundles / "a-other-model")
    assert trajectory.path_geometry(require_continuous=True).reference_identity == manifests["b-shifted"]["identity"]
    (bundles / "b-matching/source/control_points.txt").write_text("invalid\n")
    trajectory = load_repository_trajectory(str(tmp_path / "ik_input.mat"), str(shifted_config), 3.5)
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    with pytest.raises(ValueError, match="SHA256"):
        trajectory.path_geometry(require_continuous=True)


def test_generate_and_load_real_butterfly_files(tmp_path):
    output = tmp_path / "generated"
    manifest = generate_reference(
        SOURCE, ROOT / "portable_oscbf/config/nineaxis.yaml",
        ROOT / "config/oscbf_controller.yaml", output,
    )
    assert manifest["validation"]["source_points"] == 14992
    assert manifest["validation"]["source_position_max_error_m"] < 1e-12
    assert manifest["validation"]["source_axis_max_error"] < 2e-14
    assert manifest["description"]["model_identity"]["controlled_joints"] == list(robot_spec.DEFAULT_JOINT_NAMES)
    identity_source = Path(robot_spec.__file__)
    assert manifest["generator"]["source_code_sha256"][str(identity_source)] == sha256(identity_source.read_bytes()).hexdigest()
    bundle = load_reference_bundle(output)
    assert bundle.manifest["identity"] == manifest["identity"]
    result = bundle.reference.query(np.linspace(0, 1, 99))
    assert result.valid.all()
    trajectory = load_repository_trajectory(str(SOURCE / "ik_input.mat"),
                                            str(ROOT / "portable_oscbf/config/nineaxis.yaml"), 3.5)
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    trajectory.attach_continuous_reference(output)
    geometry = trajectory.path_geometry()
    assert geometry.continuous_reference is not None
    np.testing.assert_allclose(geometry.sample(.123 * geometry.total_length_m).position_m,
                               bundle.reference.query(.123).position_m, atol=2e-14)
    with pytest.raises(FileExistsError):
        generate_reference(SOURCE, ROOT / "portable_oscbf/config/nineaxis.yaml",
                           ROOT / "config/oscbf_controller.yaml", output)
    with pytest.raises(ValueError, match="source"):
        load_reference_bundle(output, expected_source_sha256="0" * 64)
    (output / "source/control_points.txt").write_text("invalid\n")
    with pytest.raises(ValueError, match="SHA256"):
        load_reference_bundle(output)


def test_s0_work_distance_tool_axis_and_nominal_fields_keep_their_sources(tmp_path):
    output = tmp_path / "s0"
    manifest = generate_reference(SOURCE, ROOT / "portable_oscbf/config/nineaxis.yaml",
                                  ROOT / "config/oscbf_controller.yaml", output,
                                  scenario_config=ROOT / "config/static_experiment_s0.yaml")
    bundle = load_reference_bundle(output)
    assert bundle.reference.total_length_m == pytest.approx(1.8167619852376167, abs=3e-10)
    description = manifest["description"]
    assert description["work_distance"]["applied"]
    assert description["work_distance"]["distance_m"] == pytest.approx(.05)
    sample = bundle.reference.query(np.linspace(0, 1, 89))
    centre = np.array(description["projection_centres_m"][-1])
    axis = np.array(description["projection_axes"][-1])
    radial = sample.position_m - centre
    radial -= (radial @ axis)[:, None] * axis
    distance = np.linalg.norm(radial, axis=1) - description["work_distance"]["workpiece_radius_m"]
    np.testing.assert_allclose(distance, .05, atol=4e-16, rtol=0)
    conversion = np.asarray(description["C_source_tcp"])
    source_rotation = sample.rotation @ conversion.T
    np.testing.assert_allclose(source_rotation[:, :, 2], sample.tool_axis, atol=2e-15)
    np.testing.assert_allclose(source_rotation @ conversion, sample.rotation, atol=2e-15)
    assert np.count_nonzero(~bundle.source_samples["valid__speed_series"]) == 1
    assert np.count_nonzero(~np.isfinite(bundle.source_samples["speed_series"])) == 1
    assert float(bundle.source_samples["plan__F"]) == 80
    assert description["calibration_status"] == "nominal_simulation_not_measured"
    description["transform"][0][3] += .001
    manifest["identity"] = reference_identity(description)
    (output / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="source"):
        load_reference_bundle(output)


def test_installed_assets_are_required_by_the_real_butterfly_entry(tmp_path):
    from distutils.core import run_setup

    distribution = run_setup(str(ROOT / "setup.py"), stop_after="config")
    installed = tmp_path / "share/robot_safecontrol_moveit"
    for destination, files in distribution.data_files:
        if "/data/nurbs" in destination:
            folder = tmp_path / destination
            folder.mkdir(parents=True, exist_ok=True)
            for filename in files:
                shutil.copyfile(ROOT / filename, folder / Path(filename).name)
    shutil.copytree(ROOT / "portable_oscbf/work", installed / "portable_oscbf/work",
                    ignore=shutil.ignore_patterns("__pycache__"))
    shutil.copytree(ROOT / "portable_oscbf/config", installed / "portable_oscbf/config")
    environment = dict(os.environ, PYTHONPATH=str(installed / "portable_oscbf"))
    probe = [sys.executable, str(ROOT / "tests/butterfly_asset_admission.py"), str(installed)]
    accepted = subprocess.run(probe, check=False, env=environment, capture_output=True, text=True, timeout=30)
    assert accepted.returncode == 0, accepted.stdout + accepted.stderr
    bundle_root = installed / "data/nurbs/butterfly_reference_v1"
    saved_root = tmp_path / "saved-bundles"
    bundle_root.rename(saved_root)
    missing = subprocess.run(probe, check=False, env=environment, capture_output=True, text=True, timeout=30)
    assert missing.returncode != 0
    assert "FileNotFoundError: continuous reference" in missing.stderr
    saved_root.rename(bundle_root)
    mat_path = installed / "data/nurbs/ik_input.mat"
    raw = loadmat(mat_path, simplify_cells=True)["ik_input"]
    raw["position_series"][0, 0] += .01
    savemat(mat_path, {"ik_input": raw}, do_compression=True)
    changed = subprocess.run(probe, check=False, env=environment, capture_output=True, text=True, timeout=30)
    assert changed.returncode != 0
    assert "continuous reference source" in changed.stderr


def test_orientation_mode_change_rebuilds_the_actual_path_reference():
    trajectory = load_repository_trajectory(
        str(SOURCE / "ik_input.mat"), str(ROOT / "portable_oscbf/config/nineaxis.yaml"), 3.5,
    )
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    assert trajectory.path_geometry().continuous_reference is not None
    trajectory.set_orientation_mode("fixed", fixed_R=np.eye(3))
    geometry = trajectory.path_geometry()
    assert geometry.continuous_reference is None
    for fraction in (0., .5, 1.):
        np.testing.assert_allclose(geometry.sample(fraction * geometry.total_length_m).rotation,
                                   np.eye(3), atol=1e-15, rtol=0.)
    with pytest.raises(ValueError, match="surface_normal"):
        trajectory.path_geometry(require_continuous=True)


@pytest.mark.parametrize("bundle_name", ["production", "s0"])
@pytest.mark.parametrize("via_fixed", [False, True], ids=["repeat", "fixed"])
@pytest.mark.parametrize("explicit_cylinder", [False, True], ids=["fitted", "explicit"])
def test_surface_normal_reset_preserves_the_actual_reference(bundle_name, via_fixed, explicit_cylinder):
    manifest = json.loads((ROOT / f"data/nurbs/butterfly_reference_v1/{bundle_name}/manifest.json").read_text())
    radius = None if bundle_name == "production" else manifest["description"]["projection_radii_m"][-1]
    trajectory = load_repository_trajectory(
        str(SOURCE / "ik_input.mat"), str(ROOT / "portable_oscbf/config/nineaxis.yaml"), 3.5,
    )
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=radius)
    geometry = trajectory.path_geometry(require_continuous=True)
    assert geometry.reference_identity == manifest["identity"]
    original = trajectory.task_reference_sequence()
    times = np.arange(trajectory.num_points) * trajectory.Ts
    original_jerk = np.array([trajectory.jerk_world_at(time).copy() for time in times])
    options = {"radius_m": radius}
    if explicit_cylinder:
        options.update(axis_point=trajectory.surface_centre.copy(), radius_m=trajectory.surface_radius)
    for _ in range(3):
        if via_fixed:
            trajectory.set_orientation_mode("fixed", fixed_R=np.eye(3))
            fixed = trajectory.path_geometry()
            assert fixed.continuous_reference is None
            np.testing.assert_allclose(fixed.sample(.5 * fixed.total_length_m).rotation, np.eye(3), atol=1e-15, rtol=0)
        trajectory.set_surface_normal_orientation([0., 1., 0.], **options)
        current = trajectory.path_geometry(require_continuous=True)
        assert current.reference_identity == manifest["identity"]
        assert current.total_length_m == geometry.total_length_m
    restored = trajectory.task_reference_sequence()
    assert len(restored) == len(original) == 14992
    for field in ("pos", "vel", "accel", "R_des", "omega"):
        np.testing.assert_allclose([getattr(sample, field) for sample in restored],
                                   [getattr(sample, field) for sample in original], atol=2e-14, rtol=0)
    np.testing.assert_allclose([trajectory.jerk_world_at(time) for time in times], original_jerk, atol=2e-14, rtol=0)


@pytest.mark.parametrize("bundle_name", ["production", "s0"])
def test_changing_cylinder_rebuilds_the_reference_from_the_loaded_path(bundle_name):
    manifest = json.loads((ROOT / f"data/nurbs/butterfly_reference_v1/{bundle_name}/manifest.json").read_text())
    radius = None if bundle_name == "production" else manifest["description"]["projection_radii_m"][-1]
    trajectory = load_repository_trajectory(
        str(SOURCE / "ik_input.mat"), str(ROOT / "portable_oscbf/config/nineaxis.yaml"), 3.5,
    )
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    trajectory.set_surface_normal_orientation(
        [.02, 1., .01], axis_point=trajectory.surface_centre + [.01, 0., -.02], radius_m=.31,
    )
    with pytest.raises(ValueError, match="缺少匹配的 continuous reference"):
        trajectory.path_geometry(require_continuous=True)
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=radius)
    geometry = trajectory.path_geometry(require_continuous=True)
    assert geometry.reference_identity == manifest["identity"]
    fresh = load_repository_trajectory(
        str(SOURCE / "ik_input.mat"), str(ROOT / "portable_oscbf/config/nineaxis.yaml"), 3.5,
    )
    fresh.set_surface_normal_orientation([0., 1., 0.], radius_m=radius)
    for accessor in ("pos_world_at", "vel_world_at", "accel_world_at", "jerk_world_at", "angular_velocity_at"):
        times = np.arange(trajectory.num_points) * trajectory.Ts
        np.testing.assert_array_equal([getattr(trajectory, accessor)(time) for time in times],
                                      [getattr(fresh, accessor)(time) for time in times])
    np.testing.assert_array_equal(geometry.rotations, fresh.path_geometry(require_continuous=True).rotations)


def test_invalid_cylinder_parameters_preserve_the_loaded_reference():
    trajectory = load_repository_trajectory(
        str(SOURCE / "ik_input.mat"), str(ROOT / "portable_oscbf/config/nineaxis.yaml"), 3.5,
    )
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=.2706)
    geometry = trajectory.path_geometry(require_continuous=True)
    original = trajectory.task_reference_sequence()
    times = np.arange(trajectory.num_points) * trajectory.Ts
    original_jerk = np.array([trajectory.jerk_world_at(time).copy() for time in times])
    invalid_parameters = (
        {"axis_direction": [0., 1., 0., 0.]},
        {"axis_direction": [0., 1.]},
        {"axis_direction": [0., 0., 0.]},
        {"axis_direction": [0., np.nan, 0.]},
        {"axis_direction": [0., np.inf, 0.]},
        {"axis_direction": [0., 1., 0.], "axis_point": [0., 1.]},
        {"axis_direction": [0., 1., 0.], "axis_point": [0., np.nan, 0.]},
        {"axis_direction": [0., 1., 0.], "axis_point": [0., np.inf, 0.]},
        {"axis_direction": [0., 1., 0.], "radius_m": 0.},
        {"axis_direction": [0., 1., 0.], "radius_m": np.nan},
    )
    for parameters in invalid_parameters:
        with pytest.raises(ValueError):
            trajectory.set_surface_normal_orientation(**parameters)
        current = trajectory.path_geometry(require_continuous=True)
        assert current.reference_identity == geometry.reference_identity
        np.testing.assert_array_equal(current.positions_m, geometry.positions_m)
        restored = trajectory.task_reference_sequence()
        assert len(restored) == len(original) == 14992
        for field in ("pos", "vel", "accel", "R_des", "omega"):
            np.testing.assert_array_equal([getattr(sample, field) for sample in restored],
                                          [getattr(sample, field) for sample in original])
        np.testing.assert_array_equal([trajectory.jerk_world_at(time) for time in times], original_jerk)
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=.2706)
    assert trajectory.path_geometry(require_continuous=True).reference_identity == geometry.reference_identity


@pytest.mark.parametrize("bundle_name", ["production", "s0"])
@pytest.mark.parametrize("fit_failure,expected_error,message", [
    pytest.param("empty_points", ValueError, "zero-size array", id="empty-points"),
    pytest.param("overflow", FloatingPointError, "overflow", id="overflow"),
    pytest.param("zero_radius", ValueError, "cylinder fit", id="zero-radius"),
])
def test_cylinder_fit_failure_preserves_loaded_state(bundle_name, fit_failure, expected_error, message):
    bundle_path = ROOT / "data/nurbs/butterfly_reference_v1" / bundle_name
    manifest = json.loads((bundle_path / "manifest.json").read_text())
    radius = None if bundle_name == "production" else manifest["description"]["projection_radii_m"][-1]
    trajectory = load_repository_trajectory(
        str(SOURCE / "ik_input.mat"), str(ROOT / "portable_oscbf/config/nineaxis.yaml"), 3.5,
    )
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=radius)
    trajectory.attach_continuous_reference(bundle_path)
    geometry = trajectory.path_geometry(require_continuous=True)
    bundle = trajectory._continuous_reference_bundle
    assert bundle is not None
    assert geometry.reference_identity == manifest["identity"]
    geometry_arrays = {field: getattr(geometry, field).copy() for field in
                       ("positions_m", "rotations", "arc_length_m", "feedrate_m_s", "source_time_s")}
    array_objects = {name: getattr(trajectory, name) for name in (
        "_pos_world", "_vel_world", "_acc_world", "_jerk_world", "_R_des_series", "_omega_series",
        "surface_axis", "surface_centre",
    )}
    array_values = {name: array.copy() for name, array in array_objects.items()}
    original_radius = trajectory.surface_radius
    explicit_path = trajectory._explicit_reference_path
    stages = trajectory.projection_stages
    original_stages = deepcopy(stages)
    original = trajectory.task_reference_sequence()
    times = np.arange(trajectory.num_points) * trajectory.Ts
    original_jerk = np.array([trajectory.jerk_world_at(time).copy() for time in times])

    # 从真实首次投影点构造拟合失败条件，调用真实 NumPy 拟合。
    source_points = trajectory._surface_projection_base["_pos_world"]
    if fit_failure == "empty_points":
        fit_points = source_points[:0].copy()
    elif fit_failure == "overflow":
        fit_points = source_points * 1e160
    else:
        axis = np.array([0., 1., 0.])
        fit_points = np.outer(source_points @ axis, axis)
    assert np.isfinite(fit_points).all()
    trajectory._surface_projection_base["_pos_world"] = fit_points
    with pytest.raises(expected_error, match=message), np.errstate(over="raise", invalid="raise"):
        trajectory.set_surface_normal_orientation([0., 1., 0.])

    assert trajectory.orientation_mode == "surface_normal"
    assert trajectory.surface_radius == original_radius
    assert trajectory._explicit_reference_path == explicit_path
    for name, array in array_objects.items():
        assert getattr(trajectory, name) is array
        np.testing.assert_array_equal(getattr(trajectory, name), array_values[name])
    assert trajectory.projection_stages is stages
    assert len(stages) == len(original_stages) == 2
    for current, previous in zip(stages, original_stages):
        assert current.keys() == previous.keys()
        for key in previous:
            np.testing.assert_array_equal(current[key], previous[key])
    assert trajectory._continuous_reference_bundle is bundle
    assert trajectory._path_geometry is geometry
    assert trajectory.path_geometry(require_continuous=True) is geometry
    assert geometry.reference_identity == manifest["identity"]
    for field, values in geometry_arrays.items():
        np.testing.assert_array_equal(getattr(geometry, field), values)
    restored = trajectory.task_reference_sequence()
    assert len(restored) == len(original) == 14992
    for field in ("pos", "vel", "accel", "R_des", "omega"):
        np.testing.assert_array_equal([getattr(sample, field) for sample in restored],
                                      [getattr(sample, field) for sample in original])
    np.testing.assert_array_equal([trajectory.jerk_world_at(time) for time in times], original_jerk)

    assert trajectory._surface_projection_base["_pos_world"] is fit_points
    trajectory._surface_projection_base["_pos_world"] = source_points
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=radius)
    recovered = trajectory.path_geometry(require_continuous=True)
    assert recovered.reference_identity == manifest["identity"]
    for field, values in geometry_arrays.items():
        np.testing.assert_array_equal(getattr(recovered, field), values)


@pytest.mark.parametrize("via_fixed", [False, True], ids=["repeat", "fixed"])
def test_explicit_reference_remains_available_after_orientation_reset(reference_variants, tmp_path, via_fixed):
    inputs, shifted_config, manifests = reference_variants
    shutil.copyfile(inputs / "ik_input.mat", tmp_path / "ik_input.mat")
    trajectory = load_repository_trajectory(str(tmp_path / "ik_input.mat"), str(shifted_config), 3.5)
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    centre, radius = trajectory.surface_centre.copy(), trajectory.surface_radius
    trajectory.attach_continuous_reference(inputs / "butterfly_reference_v1/b-shifted")
    original_geometry = trajectory.path_geometry(require_continuous=True)
    assert original_geometry.reference_identity == manifests["b-shifted"]["identity"]
    original = trajectory.task_reference_sequence()
    times = np.arange(trajectory.num_points) * trajectory.Ts
    original_jerk = np.array([trajectory.jerk_world_at(time).copy() for time in times])
    if via_fixed:
        trajectory.set_orientation_mode("fixed", fixed_R=np.eye(3))
        assert trajectory.path_geometry().continuous_reference is None
    trajectory.set_surface_normal_orientation([0., 1., 0.], axis_point=centre, radius_m=radius)
    geometry = trajectory.path_geometry(require_continuous=True)
    assert geometry.reference_identity == original_geometry.reference_identity
    for field in ("positions_m", "rotations", "arc_length_m", "feedrate_m_s", "source_time_s"):
        np.testing.assert_array_equal(getattr(geometry, field), getattr(original_geometry, field))
    restored = trajectory.task_reference_sequence()
    assert len(restored) == len(original) == 14992
    for field in ("pos", "vel", "accel", "R_des", "omega"):
        np.testing.assert_array_equal([getattr(sample, field) for sample in restored],
                                      [getattr(sample, field) for sample in original])
    np.testing.assert_array_equal([trajectory.jerk_world_at(time) for time in times], original_jerk)


def test_explicit_reference_is_preserved_across_other_cylinder_versions(reference_variants, tmp_path):
    inputs, shifted_config, manifests = reference_variants
    shutil.copyfile(inputs / "ik_input.mat", tmp_path / "ik_input.mat")
    s0_manifest = generate_reference(
        SOURCE, shifted_config, ROOT / "config/oscbf_controller.yaml",
        tmp_path / "butterfly_reference_v1/s0", scenario_config=ROOT / "config/static_experiment_s0.yaml",
    )
    trajectory = load_repository_trajectory(str(tmp_path / "ik_input.mat"), str(shifted_config), 3.5)
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    trajectory.attach_continuous_reference(inputs / "butterfly_reference_v1/b-shifted")
    original = trajectory.path_geometry(require_continuous=True)
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=.31)
    with pytest.raises(ValueError, match="缺少匹配的 continuous reference"):
        trajectory.path_geometry(require_continuous=True)
    trajectory.set_surface_normal_orientation([0., 1., 0.], radius_m=.2706)
    assert trajectory.path_geometry(require_continuous=True).reference_identity == s0_manifest["identity"]
    trajectory.set_orientation_mode("fixed", fixed_R=np.eye(3))
    assert trajectory.path_geometry().continuous_reference is None
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    restored = trajectory.path_geometry(require_continuous=True)
    assert restored.reference_identity == original.reference_identity == manifests["b-shifted"]["identity"]
    for field in ("positions_m", "rotations", "arc_length_m", "feedrate_m_s", "source_time_s"):
        np.testing.assert_array_equal(getattr(restored, field), getattr(original, field))


@pytest.mark.parametrize("damage", ["source_hash", "missing_manifest"])
def test_explicit_reference_is_revalidated_after_orientation_reset(reference_variants, tmp_path, damage):
    inputs, shifted_config, _ = reference_variants
    shutil.copyfile(inputs / "ik_input.mat", tmp_path / "ik_input.mat")
    bundle = tmp_path / "external-reference"
    shutil.copytree(inputs / "butterfly_reference_v1/b-shifted", bundle)
    trajectory = load_repository_trajectory(str(tmp_path / "ik_input.mat"), str(shifted_config), 3.5)
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    trajectory.attach_continuous_reference(bundle)
    assert trajectory.path_geometry(require_continuous=True).continuous_reference is not None
    if damage == "source_hash":
        (bundle / "source/control_points.txt").write_text("invalid\n")
        expected_error, message = ValueError, "SHA256"
    else:
        (bundle / "manifest.json").rename(bundle / "saved-manifest.json")
        expected_error, message = FileNotFoundError, "manifest.json"
    trajectory.set_surface_normal_orientation([0., 1., 0.])
    with pytest.raises(expected_error, match=message):
        trajectory.path_geometry(require_continuous=True)
