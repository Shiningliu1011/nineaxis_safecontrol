import json
import os
import shutil
import subprocess
import sys
from hashlib import sha256
from pathlib import Path

import numpy as np
import pytest
from scipy.io import loadmat, savemat
from work.continuous_path_reference import load_reference_bundle, reference_identity
from work.ik_data_loader import load_repository_trajectory

from robot_safecontrol_moveit import robot_spec
from robot_safecontrol_moveit.pipe_assets.butterfly_reference import generate_reference

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path(os.environ.get("BUTTERFLY_SOURCE_DIR", ROOT / "data/nurbs/butterfly_reference_v1/production/source"))


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
