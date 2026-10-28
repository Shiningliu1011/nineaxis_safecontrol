from dataclasses import replace
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from work.nineaxis_model import NineaxisModel
from work.nineaxis_kinematics import NineaxisKinematics
from work.process_limits import ProcessLimits
from robot_safecontrol_moveit.moveit_runtime_config import load_urdf
from aeb_rrtstar.robot_model import JOINT_LIMITS, forward_kinematics


ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "tests/data/j1_model_reference_4c98a64.json"


def test_confirmed_origin_travel_and_moveit_model():
    description = ET.fromstring(load_urdf(ROOT))
    joint = description.find("joint[@name='J1']")
    assert joint.find("origin").attrib == dict(xyz="0 0 0.085", rpy="0 0 0")
    assert joint.find("axis").get("xyz") == "0 0 1"
    assert float(joint.find("limit").get("lower")) == 0
    assert float(joint.find("limit").get("upper")) == 0.5


@pytest.mark.parametrize("case_index", range(6))
def test_reference_link_tcp_and_collision_geometry(case_index):
    reference = json.loads(REFERENCE.read_text())
    case = reference["cases"][case_index]
    model = NineaxisModel.load(root=ROOT)
    assert model.model_hash != reference["model_hash"]
    assert model.asset_model_hash != reference["asset_model_hash"]
    q = np.asarray(case["old_q_si"])
    q[0] = case["new_j1_m"]
    poses = model.link_poses(q)
    for name, expected in case["link_poses"].items():
        np.testing.assert_allclose(poses[name], expected, rtol=0, atol=1e-12)
    for mesh in model.mesh_sources:
        assert mesh.closed_hash == reference["closed_mesh_hashes"][mesh.link]
        np.testing.assert_allclose(poses[mesh.link] @ mesh.local_transform,
                                   case["collision_poses"][mesh.link], rtol=0, atol=1e-12)
    kinematics = NineaxisKinematics()
    position, rotation = kinematics.ee_pose(q)
    expected_tcp = np.asarray(case["link_poses"]["tcp"])
    np.testing.assert_allclose(position, expected_tcp[:3, 3], rtol=0, atol=1e-12)
    np.testing.assert_allclose(rotation, expected_tcp[:3, :3], rtol=0, atol=1e-12)


def test_old_calibration_identity_is_rejected_and_limits_use_new_travel():
    reference = json.loads(REFERENCE.read_text())
    model = NineaxisModel.load(root=ROOT)
    with pytest.raises(ValueError, match="applicable_model_hash"):
        model.with_calibration(replace(
            model.calibration,
            applicable_model_hash=reference["asset_model_hash"],
            tcp_correction=replace(model.calibration.tcp_correction,
                                   applicable_model_hash=reference["asset_model_hash"]),
        ))
    bound = ProcessLimits.load().bind(
        model, path_hash="a" * 64, workpiece_hash="b" * 64, scene_hash="c" * 64,
        path_length_m=1, phase="tracking", evidence_scope="research_simulation",
    )
    assert bound.position_lower[0] == 0
    assert bound.position_upper[0] == 0.5


@pytest.mark.parametrize("position,world_z", [(0, .085), (.25, .335), (.5, .585)])
def test_standalone_planner_uses_current_j1_coordinates(position, world_z):
    assert JOINT_LIMITS[0] == (0, .5)
    transforms = forward_kinematics([position, *([0] * 8)])
    assert transforms[0][2] == pytest.approx(world_z)


def test_tracking_start_matches_current_physical_path_and_joint_margins():
    from tracking_configurations import START
    from work.ik_data_loader import load_repository_trajectory
    from work.joint_limit_contract import START_IK_MIN_JOINT_MARGIN

    model = NineaxisModel.load(root=ROOT)
    trajectory = load_repository_trajectory(str(ROOT / "data/nurbs/ik_input.mat"))
    trajectory.set_surface_normal_orientation([0, 1, 0])
    geometry = trajectory.path_geometry(require_continuous=True)
    target = geometry.sample(0)
    record = json.loads((ROOT / "tests/data/tracking_configurations_j1_origin85mm.json").read_text())
    assert record["model_hash"] == model.model_hash
    assert record["reference_identity"] == geometry.reference_identity
    assert np.all(START >= model.position_lower + START_IK_MIN_JOINT_MARGIN)
    assert np.all(START <= model.position_upper - START_IK_MIN_JOINT_MARGIN)
    pose = model.fk(START)
    assert np.linalg.norm(pose[:3, 3] - target.position_m) <= 1e-6
    np.testing.assert_allclose(pose[:3, 0], target.rotation[:, 0], atol=1e-5, rtol=0)
