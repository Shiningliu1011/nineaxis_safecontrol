from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from .oscbf_trajectory import bootstrap_portable
from .static_experiment_recording import read_stream


def pose_terms(kinematics, q, position, rotation):
    transforms, spatial = kinematics.forward_kinematics_and_jacobian(q)
    actual = transforms["ee_link"]
    axis, basis = actual[:3, 0], rotation[:, 1:]
    error = np.r_[actual[:3, 3] - position, basis.T @ axis]
    jacobian = np.vstack([
        spatial[3:] + np.cross(spatial[:3].T, actual[:3, 3]).T,
        basis.T @ np.cross(spatial[:3].T, axis).T,
    ])
    return error, jacobian, float(axis @ rotation[:, 0])


def project_fixed_pose(geometry, q, target, lower, upper, position, rotation):
    span = np.asarray(upper) - np.asarray(lower)

    def equality(value):
        return pose_terms(geometry.kinematics, value, position, rotation)[0]

    def equality_jacobian(value):
        return pose_terms(geometry.kinematics, value, position, rotation)[1]

    def clearance(value):
        return np.array([row.distance.distance_m - row.required_m for row in geometry.query(value)])

    def clearance_jacobian(value):
        rows = geometry.query(value)
        if any(not row.distance.valid or row.gradient is None for row in rows):
            raise ValueError("独立构型搜索遇到无效几何查询")
        return np.array([row.gradient for row in rows])

    result = minimize(
        lambda value: float(np.sum(((value - target) / span) ** 2)), q,
        jac=lambda value: 2 * (value - target) / span ** 2,
        method="SLSQP", bounds=list(zip(lower, upper)),
        constraints=[
            {"type": "eq", "fun": equality, "jac": equality_jacobian},
            {"type": "ineq", "fun": clearance, "jac": clearance_jacobian},
        ], options={"ftol": 1e-13, "maxiter": 300},
    )
    error, _, alignment = pose_terms(geometry.kinematics, result.x, position, rotation)
    if (not result.success or np.linalg.norm(error[:3]) > 1e-7
            or np.linalg.norm(error[3:]) > 1e-7 or alignment <= 0
            or np.any(clearance(result.x) < 0)):
        raise ValueError(f"独立构型搜索没有取得有效解：{result.message}")
    return result.x


def initial_configurations(config):
    bootstrap_portable(config.resource_root / "portable_oscbf")
    from work.static_collision import StaticGeometry

    geometry = StaticGeometry(config.scene())
    limits = config.execution_limits()
    q = config.initial_position
    position, rotation = geometry.kinematics.ee_pose(q)
    _, jacobian, _ = pose_terms(geometry.kinematics, q, position, rotation)
    _, _, Vh = np.linalg.svd(jacobian, full_matrices=True)
    direction = Vh[5]
    result = [q.tolist()]
    for sign in (-1., 1.):
        target = q + sign * .12 * direction
        value = project_fixed_pose(
            geometry, q, target, limits["lower"], limits["upper"], position, rotation,
        )
        if np.linalg.norm(value - q) < .02:
            raise ValueError("独立初态搜索未取得足够构型变化")
        result.append(value.tolist())
    return result


def obstacle_candidates(config, baseline: Path) -> dict:
    bootstrap_portable(config.resource_root / "portable_oscbf")
    from work.nineaxis_kinematics import NineaxisKinematics
    from work.obb_collision_model import OBB_HALF_EXTENTS_M, OBB_LOCAL_CENTERS_M, OBB_LOCAL_ROTATIONS
    from work.static_collision import StaticGeometry

    report = json.loads((baseline / "feedback_report.json").read_text())
    manifest = json.loads((baseline / "manifest.json").read_text())
    if (manifest["config"]["scenario"] != "S0" or report["task_verdict"] != "pass"
            or report["external_clearance"]["status"] != "pass"
            or report["self_clearance"]["status"] != "pass"):
        raise ValueError("候选障碍要求完整且通过几何检查的 S0 反馈")
    feedback = read_stream(baseline / "plant.sqlite3", "feedback")
    progress = np.array([row["values"]["reference_progress_m"] for row in feedback])
    path, _ = config.load_path()
    kinematics = NineaxisKinematics()
    initial = kinematics.forward_kinematics(config.initial_position)
    link = initial["Link9"]
    corners = np.array(np.meshgrid(*[[-value, value] for value in OBB_HALF_EXTENTS_M[9]])).T.reshape(-1, 3)
    world_corners = (
        corners @ (link[:3, :3] @ OBB_LOCAL_ROTATIONS[9]).T
        + link[:3, :3] @ OBB_LOCAL_CENTERS_M[9] + link[:3, 3]
    )
    tool_radius = float(np.max(np.linalg.norm(world_corners - initial["ee_link"][:3, 3], axis=1)))
    maximum_spacing = float(np.max(np.diff(path.arc_length_m)))
    base_scene = config.scene()
    candidates = []
    for fraction in (.25, .5, .75):
        index = int(np.argmin(abs(progress - path.total_length_m * fraction)))
        q = np.asarray(feedback[index]["values"]["q"])
        transforms = kinematics.forward_kinematics(q)
        for link_index in range(3, 8):
            transform = transforms[f"Link{link_index}"]
            center = transform[:3, :3] @ OBB_LOCAL_CENTERS_M[link_index] + transform[:3, 3]
            rotation = transform[:3, :3] @ OBB_LOCAL_ROTATIONS[link_index]
            for axis in range(3):
                for sign in (-1., 1.):
                    normal = sign * rotation[:, axis]
                    obstacle_center = center + normal * (
                        OBB_HALF_EXTENTS_M[link_index, axis] + .06 * np.sum(abs(normal)) + .015
                    )
                    point_distances = np.linalg.norm(
                        np.maximum(abs(path.positions_m - obstacle_center) - .06, 0.), axis=1,
                    )
                    corridor = float(point_distances.min() - maximum_spacing / 2 - tool_radius)
                    if corridor < .03:
                        continue
                    scene = deepcopy(base_scene)
                    obstacle = {"name": "obstacle_1", "center_m": obstacle_center.tolist(), "radius_m": .06}
                    scene["obstacles"] = [obstacle]
                    geometry = StaticGeometry(scene)
                    initial_rows = geometry.query(config.initial_position)
                    if not all(row.distance.valid and row.distance.distance_m >= row.required_m for row in initial_rows):
                        continue
                    target = next(
                        row for row in geometry.query(q)
                        if row.pair == (f"Link{link_index}", "obstacle_1")
                    )
                    if target.distance.distance_m >= .03:
                        continue
                    candidates.append({
                        "fraction": fraction, "link": f"Link{link_index}", "obstacle": obstacle,
                        "initial_minimum_margin_m": min(row.distance.distance_m - row.required_m for row in initial_rows),
                        "tool_corridor_lower_bound_m": corridor, "tool_enclosing_radius_m": tool_radius,
                        "counterfactual_q": q.tolist(), "reference_progress_m": float(progress[index]),
                        "counterfactual_measurement": target.to_record(),
                    })
    return {"baseline": str(baseline.resolve()), "candidates": candidates}


def blocked_path_evidence(config, initial_positions=None) -> dict:
    bootstrap_portable(config.resource_root / "portable_oscbf")
    from work.nineaxis_kinematics import NineaxisKinematics
    from work.obb_collision_model import OBB_HALF_EXTENTS_M, OBB_LOCAL_CENTERS_M, OBB_LOCAL_ROTATIONS
    from work.static_collision import StaticGeometry

    transforms = NineaxisKinematics().forward_kinematics(config.initial_position)
    ee, link = transforms["ee_link"], transforms["Link9"]
    rotation = link[:3, :3] @ OBB_LOCAL_ROTATIONS[9]
    center = link[:3, :3] @ OBB_LOCAL_CENTERS_M[9] + link[:3, 3]
    offset, direction = rotation.T @ (ee[:3, 3] - center), rotation.T @ ee[:3, 0]
    low, high = -np.inf, np.inf
    for value, slope, half in zip(offset, direction, OBB_HALF_EXTENTS_M[9]):
        if abs(slope) < 1e-12:
            if abs(value) > half:
                raise ValueError("工具轴未通过 Link9 OBB，无法形成 roll 无关的阻塞见证")
            continue
        first, last = sorted(((-half - value) / slope, (half - value) / slope))
        low, high = max(low, first), min(high, last)
    if not np.isfinite(low + high) or low >= high:
        raise ValueError("工具轴与 Link9 OBB 无有效内部区间")
    path, _ = config.load_path()
    base_scene = config.scene()
    rotations = Rotation.from_matrix(path.rotations)
    angular_steps = (rotations[:-1].inv() * rotations[1:]).magnitude()
    for portion in (.9, .5, .1):
        tool_offset = low + portion * (high - low)
        invariant_points = path.positions_m + tool_offset * path.rotations[:, :, 0]
        uncertainty = .0005 + 2 * abs(tool_offset) * np.sin(np.deg2rad(.5) / 2)
        for fraction in (.25, .5, .75, *np.linspace(.05, .95, 19)):
            index = int(np.argmin(abs(path.arc_length_m - fraction * path.total_length_m)))
            initial_point_distance = np.linalg.norm(np.maximum(
                abs(ee[:3, 3] + tool_offset * ee[:3, 0] - invariant_points[index]) - .06, 0.,
            ))
            if initial_point_distance < .03:
                continue
            obstacle = {"name": "path_block", "center_m": invariant_points[index].tolist(), "radius_m": .06}
            scene = deepcopy(base_scene)
            scene["obstacles"] = [obstacle]
            geometry = StaticGeometry(scene)
            initial = [
                row for q in (initial_positions if initial_positions is not None else [config.initial_position])
                for row in geometry.query(np.asarray(q))
            ]
            if not all(row.distance.valid and row.distance.distance_m >= row.required_m for row in initial):
                continue
            distance = np.linalg.norm(np.maximum(abs(invariant_points - invariant_points[index]) - .06, 0.), axis=1)
            change = np.diff(path.arc_length_m) + abs(tool_offset) * angular_steps
            blocked = distance[:-1] + change + uncertainty < .03
            start, end = index, index
            while start > 0 and blocked[start - 1]:
                start -= 1
            while end < len(blocked) and blocked[end]:
                end += 1
            if start == end:
                continue
            return {
                "status": "proven", "obstacle": obstacle,
                "tool_axis_offset_m": float(tool_offset), "tool_point_inside_link": "Link9",
                "pose_tolerance_displacement_m": float(uncertainty),
                "blocked_interval_m": [float(path.arc_length_m[start]), float(path.arc_length_m[end])],
                "witness_progress_m": float(path.arc_length_m[index]),
                "initial_minimum_margin_m": min(row.distance.distance_m - row.required_m for row in initial),
            }
    raise ValueError("候选路径位置没有同时满足合法初态和阻塞证明")
