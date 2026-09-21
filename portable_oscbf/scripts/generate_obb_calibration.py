#!/usr/bin/env python3
"""Generate the OBB collision model for the ninezzhou arm (M2).

For every link STL the script computes a tight oriented bounding box aligned
with the link-frame principal axes (PCA), registers the environment-sampling
grid for that link, then writes:

- ``work/obb_collision_model.py`` (numpy constants consumed by the control
  core and by M3's DCOL collision path);
- ``config/obb_model.yaml`` (human-readable calibration data).

The online CBF table inherits the 14-pair reference topology
(``oscbf_collision_config.py``) mapped to link indices.  It is only the
online constraint subset; OFF-02 evaluates all other non-adjacent pairs with
point checks and bounded-region evidence.

Usage::

    python3 portable_oscbf/scripts/generate_obb_calibration.py \
        [--mesh-dir models/ninezzhou/meshes] \
        [--output-py portable_oscbf/work/obb_collision_model.py] \
        [--output-yaml portable_oscbf/config/obb_model.yaml]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MESH_DIR = REPO_ROOT / "models" / "ninezzhou" / "meshes"
DEFAULT_OUTPUT_PY = REPO_ROOT / "portable_oscbf" / "work" / "obb_collision_model.py"
DEFAULT_OUTPUT_YAML = REPO_ROOT / "portable_oscbf" / "config" / "obb_model.yaml"

LINK_NAMES = (
    "base_link", "Link1", "Link2", "Link3", "Link4",
    "Link5", "Link6", "Link7", "Link8", "Link9",
)

# Reference 14-pair topology (oscbf_collision_config.py SELF_COLLISION_PAIRS)
# mapped from 17-sphere indices to OBB link indices (0=base_link .. 9=Link9).
# Link6 and Link3-Link5 are absent from this historical online subset.  Their
# absence is not a geometric exemption; they remain part of the 36-pair
# non-adjacent point policy and the 22-pair omitted-region certificate.
COLLISION_PAIRS = (
    (0, 4), (0, 5), (0, 7), (0, 8), (0, 9),
    (1, 5), (1, 7), (1, 8), (1, 9),
    (2, 7), (2, 8), (2, 9),
    (3, 8), (3, 9),
)

# 每个连杆的 OBB 分格数；轴顺序是 OBB 的 x、y、z。
SAMPLE_GRID_SHAPES = {
    "base_link": (1, 1, 6),
    "Link1": (1, 3, 2),
    "Link2": (3, 1, 1),
    "Link3": (3, 1, 1),
    "Link4": (1, 2, 1),
    "Link5": (1, 1, 3),
    "Link6": (2, 1, 1),
    "Link7": (2, 1, 1),
    "Link8": (2, 1, 1),
    "Link9": (3, 1, 1),
}
SAMPLE_SPHERE_PADDING_M = 0.002


def _principal_obb(vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (center, half_extents, R_link_obb) from the mesh vertices.

    ``R_link_obb`` maps OBB axes to the link frame: a link-frame point ``p``
    has OBB coordinates ``R.T @ (p - center)``.  Eigenvector signs are
    normalised so each principal axis points toward positive unit-axis
    alignment, which makes the output deterministic and readable.
    """

    center_init = vertices.mean(axis=0)
    centered = vertices - center_init
    covariance = centered.T @ centered / max(len(vertices), 1)
    _, eigenvectors = np.linalg.eigh(covariance)
    rotation = eigenvectors[:, ::-1].copy()  # descending eigenvalues
    for axis in range(3):
        if rotation[0, axis] < 0.0:
            rotation[:, axis] *= -1.0
        elif abs(rotation[0, axis]) < 1e-12 and rotation[1, axis] < 0.0:
            rotation[:, axis] *= -1.0

    projected = centered @ rotation
    lower, upper = projected.min(axis=0), projected.max(axis=0)
    center_obb = 0.5 * (lower + upper)
    half_extents = 0.5 * (upper - lower)
    center = center_init + rotation @ center_obb
    return center, half_extents, rotation


def _select_obb(vertices: np.ndarray):
    """Pick the tighter of the link-frame AABB and the PCA OBB.

    The reference topology expects OBBs aligned with the link-frame xyz axes
    (``R = I``) whenever possible.  PCA is only used when it is strictly
    tighter; for near-circular cross-sections PCA axes are unstable and can
    produce a *looser* box, so the link-frame AABB is preferred there.

    Returns (center, half_extents, rotation, method).
    """

    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    aabb_center = 0.5 * (lower + upper)
    aabb_half = 0.5 * (upper - lower)
    aabb_volume = float(np.prod(upper - lower))

    pca_center, pca_half, pca_rotation = _principal_obb(vertices)
    pca_volume = float(np.prod(2.0 * pca_half))

    if pca_volume < aabb_volume - 1e-9:
        return pca_center, pca_half, pca_rotation, "pca"
    return aabb_center, aabb_half, np.eye(3), "aabb"


def compute_obb_data(mesh_dir: Path) -> dict:
    """Compute OBB data for every link and return volume ratios."""

    entries = []
    volume_ratios = {}
    for index, link_name in enumerate(LINK_NAMES):
        stl_path = mesh_dir / f"{link_name}.STL"
        if not stl_path.is_file():
            raise FileNotFoundError(f"Missing STL for {link_name}: {stl_path}")
        mesh = trimesh.load_mesh(str(stl_path))
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        center, half_extents, rotation, method = _select_obb(vertices)
        aabb_volume = float(np.prod(vertices.max(axis=0) - vertices.min(axis=0)))
        obb_volume = float(np.prod(2.0 * half_extents))
        volume_ratios[link_name] = obb_volume / max(aabb_volume, 1e-12)
        entries.append({
            "link": link_name,
            "index": index,
            "center_m": center,
            "half_extents_m": half_extents,
            "rotation": rotation,
            "method": method,
        })
    return {
        "entries": entries,
        "volume_ratios": volume_ratios,
        "methods": {e["link"]: e["method"] for e in entries},
    }


def _format_array(value) -> str:
    return np.array2string(
        np.asarray(value, dtype=np.float64), separator=", ",
        formatter={"float_kind": lambda v: f"{v:.12g}"})


def render_python_module(data: dict) -> str:
    entries = data["entries"]
    centers = np.stack([e["center_m"] for e in entries])
    half_extents = np.stack([e["half_extents_m"] for e in entries])
    rotations = np.stack([e["rotation"] for e in entries])
    sample_grid_shapes = np.asarray(
        [SAMPLE_GRID_SHAPES[e["link"]] for e in entries], dtype=np.int32)
    pairs = np.asarray(COLLISION_PAIRS, dtype=np.int32)
    lines = [
        '"""Auto-generated OBB collision model (M2).',
        "",
        "Generated by portable_oscbf/scripts/generate_obb_calibration.py.",
        "Do not edit by hand; re-run the generator after STL changes.",
        '"""',
        "",
        "import numpy as np",
        "",
        "OBB_LINK_NAMES = " + repr(LINK_NAMES),
        "",
        "OBB_LINK_INDICES = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9], dtype=np.int32)",
        "",
        "# OBB centre in the link frame (m).",
        "OBB_LOCAL_CENTERS_M = np.array(",
        _format_array(centers) + ")",
        "",
        "# OBB half extents along the OBB axes (m).",
        "OBB_HALF_EXTENTS_M = np.array(",
        _format_array(half_extents) + ")",
        "",
        "# R_link_obb: OBB axes -> link frame; OBB coords = R.T @ (p - center).",
        "OBB_LOCAL_ROTATIONS = np.array(",
        _format_array(rotations) + ")",
        "",
        "# 每个 OBB 沿自身 x、y、z 轴的环境采样分格数。",
        "OBB_SAMPLE_GRID_SHAPES = np.array(",
        _format_array(sample_grid_shapes) + ", dtype=np.int32)",
        "OBB_SAMPLE_SPHERE_PADDING_M = " + repr(SAMPLE_SPHERE_PADDING_M),
        "",
        "",
        "def _generate_sample_spheres():",
        "    link_indices = []",
        "    local_centers = []",
        "    radii = []",
        "    for link_index, grid_shape in enumerate(OBB_SAMPLE_GRID_SHAPES):",
        "        half_extents = OBB_HALF_EXTENTS_M[link_index]",
        "        cell_half_extents = half_extents / grid_shape",
        "        for cell_index in np.ndindex(*grid_shape):",
        "            center_obb = (-half_extents +",
        "                          (np.asarray(cell_index) + 0.5) *",
        "                          (2.0 * cell_half_extents))",
        "            center_link = (OBB_LOCAL_CENTERS_M[link_index] +",
        "                           OBB_LOCAL_ROTATIONS[link_index] @ center_obb)",
        "            link_indices.append(link_index)",
        "            local_centers.append(center_link)",
        "            radii.append(np.linalg.norm(cell_half_extents) +",
        "                         OBB_SAMPLE_SPHERE_PADDING_M)",
        "    return (np.asarray(link_indices, dtype=np.int32),",
        "            np.asarray(local_centers, dtype=np.float64),",
        "            np.asarray(radii, dtype=np.float64))",
        "",
        "",
        "(OBB_SAMPLE_SPHERE_LINK_INDICES,",
        " OBB_SAMPLE_SPHERE_LOCAL_CENTERS_M,",
        " OBB_SAMPLE_SPHERE_RADII_M) = _generate_sample_spheres()",
        "NUM_OBB_SAMPLE_SPHERES = int(len(OBB_SAMPLE_SPHERE_RADII_M))",
        "",
        "if not (",
        "        NUM_OBB_SAMPLE_SPHERES == 32",
        "        and OBB_SAMPLE_SPHERE_LINK_INDICES.shape == (32,)",
        "        and OBB_SAMPLE_SPHERE_LOCAL_CENTERS_M.shape == (32, 3)",
        "        and np.all(OBB_SAMPLE_SPHERE_RADII_M > 0.0)):",
        "    raise RuntimeError(\"OBB 采样球几何无效\")",
        "",
        "# Online CBF subset; omitted non-adjacent pairs are not exemptions.",
        "OBB_COLLISION_PAIRS = np.array(",
        _format_array(pairs.astype(np.float64)) + ", dtype=np.int32)",
        "",
    ]
    return "\n".join(lines)


def write_python_module(data: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_python_module(data), encoding="utf-8")


def render_yaml(data: dict) -> str:
    entries = data["entries"]
    document = {
        "schema_version": 1,
        "generated_by": "portable_oscbf/scripts/generate_obb_calibration.py",
        "control_point": "ee_link (URDF tool0 equivalent)",
        "sample_sphere_padding_m": SAMPLE_SPHERE_PADDING_M,
        "sample_sphere_count": sum(
            int(np.prod(SAMPLE_GRID_SHAPES[e["link"]])) for e in entries),
        "links": [
            {
                "link": e["link"],
                "index": e["index"],
                "center_m": [float(v) for v in e["center_m"]],
                "half_extents_m": [float(v) for v in e["half_extents_m"]],
                "sample_grid_shape": list(SAMPLE_GRID_SHAPES[e["link"]]),
                "rotation": [[float(v) for v in row] for row in e["rotation"]],
                "method": e["method"],
            }
            for e in entries
        ],
        "collision_pairs": [list(pair) for pair in COLLISION_PAIRS],
        "collision_pairs_role": "online_cbf_subset",
        "exclusions": [],
        "pair_policy": {
            "adjacent_pairs": "excluded",
            "all_nonadjacent_pair_count": 36,
            "online_pair_count": len(COLLISION_PAIRS),
            "omitted_nonadjacent_pair_count": 36 - len(COLLISION_PAIRS),
            "omitted_pair_evidence": "OFF-02 bounded-region certificate",
        },
    }
    return yaml.safe_dump(document, sort_keys=False)


def write_yaml(data: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_yaml(data), encoding="utf-8")


def output_matches(path: Path, expected: str) -> bool:
    if not path.is_file():
        print(f"缺少生成文件：{path}")
        return False
    if path.read_text(encoding="utf-8") != expected:
        print(f"生成文件需要更新：{path}")
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh-dir", type=Path, default=DEFAULT_MESH_DIR)
    parser.add_argument("--output-py", type=Path, default=DEFAULT_OUTPUT_PY)
    parser.add_argument("--output-yaml", type=Path, default=DEFAULT_OUTPUT_YAML)
    parser.add_argument(
        "--check", action="store_true",
        help="检查仓库里的生成文件是否与 mesh 和分格规则一致")
    args = parser.parse_args()

    data = compute_obb_data(args.mesh_dir)
    if args.check:
        python_matches = output_matches(
            args.output_py, render_python_module(data))
        yaml_matches = output_matches(args.output_yaml, render_yaml(data))
        if not (python_matches and yaml_matches):
            return 1
        print("OBB 生成文件与 mesh 和分格规则一致")
        return 0

    write_python_module(data, args.output_py)
    write_yaml(data, args.output_yaml)

    print(f"Wrote {args.output_py}")
    print(f"Wrote {args.output_yaml}")
    print("OBB volume / link-frame AABB volume:")
    for link, ratio in data["volume_ratios"].items():
        print(f"  {link:10s} {ratio:.4f}  ({data['methods'][link]})")
    print(f"Online CBF pairs: {len(COLLISION_PAIRS)} of 36 non-adjacent pairs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
