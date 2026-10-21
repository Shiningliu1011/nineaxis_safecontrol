from copy import deepcopy
import json
from pathlib import Path
import shutil

import pytest

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, content_hash
from robot_safecontrol_moveit.static_experiment_provenance import relocate_manifest
from robot_safecontrol_moveit.static_formal_experiments import _physical_identity


ROOT = Path(__file__).resolve().parents[1]
RECORDED = ROOT / "tests/data/static_s1_manifest.json"


@pytest.fixture
def relocated_resources(tmp_path):
    # manifest 与内容校验值来自完整 S1 运行，测试目录保存对应资源的实际副本。
    recorded = json.loads(RECORDED.read_text())
    original_root = Path(recorded["resource_root"])
    collision = recorded["scene"]["self_collision"]
    for record in [collision["manifest"], *collision["auxiliary"], *collision["meshes"]]:
        relative = Path(record["path"]).relative_to(original_root)
        source, target = ROOT / relative, tmp_path / relative
        assert content_hash(source) == record["sha256"]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    for name in ("trajectory_mat", "kinematics_config"):
        relative = Path(recorded["config"][name])
        source, target = ROOT / relative, tmp_path / relative
        assert content_hash(source) == recorded["resources"][name]["sha256"]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    return recorded, relocate_manifest(recorded, tmp_path), tmp_path


def test_actual_s1_physical_identity_matches_relocated_resources(relocated_resources):
    recorded, relocated, destination = relocated_resources
    before = deepcopy(recorded)
    assert recorded["scene"] != relocated["scene"]
    identity = _physical_identity(recorded, destination)
    assert identity == _physical_identity(relocated, destination)
    assert identity["scene"]["self_collision"]["pairs"] == recorded["scene"]["self_collision"]["pairs"]
    assert len(identity["scene"]["self_collision"]["meshes"]) == 10
    assert len(identity["scene"]["self_collision"]["pairs"]) == 36
    assert recorded == before
    assert content_hash(RECORDED) == "0983163905f0c4e5303ed688ee60dba683b9928fa558bbfe9bdb92894612a86e"


@pytest.mark.parametrize("resource", ["collision_manifest", "mesh", "trajectory_mat", "kinematics_config"])
def test_relocated_physical_identity_rejects_changed_resource_content(relocated_resources, resource):
    recorded, relocated, destination = relocated_resources
    collision = relocated["scene"]["self_collision"]
    paths = {
        "collision_manifest": Path(collision["manifest"]["path"]),
        "mesh": Path(collision["meshes"][0]["path"]),
        "trajectory_mat": destination / recorded["config"]["trajectory_mat"],
        "kinematics_config": destination / recorded["config"]["kinematics_config"],
    }
    with paths[resource].open("ab") as stream:
        stream.write(b"\nchanged")
    with pytest.raises(ValueError, match="资源身份变化"):
        _physical_identity(recorded, destination)


def test_distinct_actual_obstacle_scenes_keep_distinct_physical_identity():
    recorded = json.loads(RECORDED.read_text())
    baseline = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method="D",
    ).manifest()
    assert _physical_identity(recorded, ROOT) != _physical_identity(baseline, ROOT)
