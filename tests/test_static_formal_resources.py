from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
from zipfile import ZipFile

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
    resources = [(Path(row["path"]).relative_to(original_root), row["sha256"])
                 for row in [collision["manifest"], *collision["auxiliary"], *collision["meshes"]]]
    resources += [(Path(recorded["config"][name]), recorded["resources"][name]["sha256"])
                  for name in ("trajectory_mat", "kinematics_config")]
    # 保存原始资源字节，历史 manifest 的模型身份保持原值。
    with ZipFile(ROOT / "tests/data/static_s1_original_resources.zip") as archive:
        for relative, expected in resources:
            data = (archive.read(relative.as_posix()) if relative.as_posix() in archive.namelist()
                    else (ROOT / relative).read_bytes())
            assert sha256(data).hexdigest() == expected
            target = tmp_path / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
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


def test_distinct_actual_obstacle_scenes_keep_distinct_physical_identity(relocated_resources):
    recorded, _, destination = relocated_resources
    baseline = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method="D",
    ).manifest()
    assert _physical_identity(recorded, destination) != _physical_identity(baseline, ROOT)


def test_recorded_model_identity_is_rejected_by_current_resources():
    recorded = json.loads(RECORDED.read_text())
    with pytest.raises(ValueError, match="资源身份变化"):
        _physical_identity(recorded, ROOT)
