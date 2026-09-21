from pathlib import Path

import mujoco
import numpy as np

from robot_safecontrol_moveit.mujoco_viewer_with_cylinder import (
    MuJoCoJointStateViewer,
)


_LINK_NAMES = (
    "base_link",
    "Link1",
    "Link2",
    "Link3",
    "Link4",
    "Link5",
    "Link6",
    "Link7",
    "Link8",
    "Link9",
)
_REPO_ROOT = Path(__file__).resolve().parents[1]


def _empty_link_model() -> str:
    bodies = "\n".join(
        f'    <body name="{link_name}"></body>'
        for link_name in _LINK_NAMES
    )
    return f"<mujoco>\n  <worldbody>\n{bodies}\n  </worldbody>\n</mujoco>"


def test_mujoco_injection_uses_generated_obbs_and_32_sample_spheres() -> None:
    xml = MuJoCoJointStateViewer._inject_obb_geometry(
        _empty_link_model(),
        show_boxes=True,
        show_sample_spheres=True,
    )
    model = mujoco.MjModel.from_xml_string(xml)
    (
        link_names,
        _,
        half_extents,
        _,
        _,
        sphere_centers,
        sphere_radii,
    ) = MuJoCoJointStateViewer._obb_visualization_data()

    assert link_names == _LINK_NAMES
    assert model.ngeom == 42
    for link_index, link_name in enumerate(link_names):
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"obb_{link_name}_box"
        )
        assert geom_id >= 0
        np.testing.assert_allclose(
            model.geom_size[geom_id], half_extents[link_index],
            rtol=0.0, atol=1.0e-12,
        )
        assert model.geom_contype[geom_id] == 0
        assert model.geom_conaffinity[geom_id] == 0
        assert model.geom_group[geom_id] == 2

    assert sphere_centers.shape == (32, 3)
    assert sphere_radii.shape == (32,)
    for sphere_index, radius in enumerate(sphere_radii):
        geom_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            f"obb_sample_sphere_{sphere_index:02d}",
        )
        assert geom_id >= 0
        np.testing.assert_allclose(
            model.geom_size[geom_id, 0], radius,
            rtol=0.0, atol=1.0e-12,
        )
        np.testing.assert_allclose(
            model.geom_pos[geom_id], sphere_centers[sphere_index],
            rtol=0.0, atol=1.0e-12,
        )
        assert model.geom_contype[geom_id] == 0
        assert model.geom_conaffinity[geom_id] == 0
        assert model.geom_group[geom_id] == 2


def test_mujoco_obb_switches_are_independent() -> None:
    boxes_only = MuJoCoJointStateViewer._inject_obb_geometry(
        _empty_link_model(),
        show_boxes=True,
        show_sample_spheres=False,
    )
    spheres_only = MuJoCoJointStateViewer._inject_obb_geometry(
        _empty_link_model(),
        show_boxes=False,
        show_sample_spheres=True,
    )

    assert mujoco.MjModel.from_xml_string(boxes_only).ngeom == 10
    assert mujoco.MjModel.from_xml_string(spheres_only).ngeom == 32


def test_project_urdf_compiles_with_obbs_and_sample_spheres() -> None:
    raw_mjcf = MuJoCoJointStateViewer._urdf_to_mjcf(
        _REPO_ROOT / "models" / "ninezzhou" / "urdf" / "ninezzhou.urdf",
        _REPO_ROOT / "models" / "ninezzhou" / "meshes",
    )
    visual_mjcf = MuJoCoJointStateViewer._inject_obb_geometry(
        raw_mjcf,
        show_boxes=True,
        show_sample_spheres=True,
    )
    model = mujoco.MjModel.from_xml_string(visual_mjcf)

    for link_name in _LINK_NAMES:
        assert mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"obb_{link_name}_box"
        ) >= 0
    for sphere_index in range(32):
        assert mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            f"obb_sample_sphere_{sphere_index:02d}",
        ) >= 0
