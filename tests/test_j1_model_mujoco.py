import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from robot_safecontrol_moveit.mujoco_viewer_with_cylinder import MuJoCoJointStateViewer


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def models(tmp_path_factory):
    path = ROOT / "models/ninezzhou/urdf/ninezzhou.urdf"
    mesh_directory = ROOT / "models/ninezzhou/meshes"
    reference = ET.parse(path)
    joint = reference.getroot().find("joint[@name='J1']")
    # 使用基点记录的 J1 定义，通过同一实际导入器求值。
    joint.find("origin").set("xyz", "0 0 0")
    joint.find("limit").set("upper", "0.585")
    old_path = tmp_path_factory.mktemp("j1-reference") / "reference.urdf"
    reference.write(old_path, encoding="utf-8")
    return tuple(mujoco.MjModel.from_xml_string(
        MuJoCoJointStateViewer._urdf_to_mjcf(source, mesh_directory),
    ) for source in (old_path, path))


@pytest.mark.parametrize("case_index", range(6))
def test_same_physical_link_tcp_and_collision_geometry(models, case_index):
    reference = json.loads((ROOT / "tests/data/j1_model_reference_4c98a64.json").read_text())
    case = reference["cases"][case_index]
    old, current = models
    previous_data, current_data = mujoco.MjData(old), mujoco.MjData(current)
    for model, data, j1 in ((old, previous_data, case["old_q_si"][0]),
                            (current, current_data, case["new_j1_m"])):
        for index in range(9):
            joint = model.joint(f"J{index + 1}")
            data.qpos[joint.qposadr[0]] = j1 if index == 0 else case["old_q_si"][index]
        mujoco.mj_forward(model, data)
    assert current.joint("J1").range[0] == 0
    assert current.joint("J1").range[1] == 0.5
    for index in range(1, 10):
        name = f"Link{index}"
        first, second = previous_data.body(name), current_data.body(name)
        np.testing.assert_allclose(second.xpos, first.xpos, rtol=0, atol=1e-12)
        np.testing.assert_allclose(second.xmat, first.xmat, rtol=0, atol=1e-12)
    np.testing.assert_allclose(current_data.geom_xpos, previous_data.geom_xpos, rtol=0, atol=1e-12)
    np.testing.assert_allclose(current_data.geom_xmat, previous_data.geom_xmat, rtol=0, atol=1e-12)
    for data in (previous_data, current_data):
        body = data.body("Link9")
        tcp = body.xpos + body.xmat.reshape(3, 3) @ np.array([.235, 0, 0])
        np.testing.assert_allclose(tcp, np.asarray(case["link_poses"]["tcp"])[:3, 3],
                                   rtol=0, atol=2e-6)
