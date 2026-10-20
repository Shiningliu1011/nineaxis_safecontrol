from __future__ import annotations

from pathlib import Path

import jax
import numpy as np
import robot_static_fcl

from .static_experiment_config import content_hash


def evaluate_clearances(directory: Path, scene: dict, measured: np.ndarray) -> dict:
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.obb_collision_model import OBB_LINK_NAMES
    from work.static_collision import StaticGeometry

    geometry = StaticGeometry(scene)
    robot = NineaxisManipulatorJAX()
    forward = jax.jit(jax.vmap(lambda q: robot._compute_all_link_transforms(q)[:10]))
    distances, statuses, labels = [], [], None
    for offset in range(0, len(measured), 512):
        poses = np.asarray(forward(measured[offset:offset + 512]))
        values, names = geometry.batch_distances(poses)
        if labels is not None and names != labels:
            raise ValueError("反馈几何对象组合发生变化")
        labels = names
        distances.append(values[:, :, 1].copy())
        statuses.append(values[:, :, 0].astype(np.uint8))
    distance = np.concatenate(distances)
    status = np.concatenate(statuses)
    kinds = np.array(["self" if second in OBB_LINK_NAMES else "external" for _, second in labels])
    required = np.where(kinds == "self", geometry.self_clearance_m, geometry.external_clearance_m)
    if not np.all(np.isfinite(distance)):
        raise ValueError("反馈几何测量包含非有限距离")
    np.savez_compressed(
        directory / "clearance_metrics.npz", distance_m=distance, status=status,
        pairs=np.array(labels), kind=kinds, required_m=required,
    )
    result = {
        "measurement_boundary": "independent_feedback_fk_and_fcl",
        "fcl_version": robot_static_fcl.fcl_version,
        "extension_sha256": content_hash(Path(robot_static_fcl.__file__)),
        "evaluator_sha256": content_hash(Path(__file__)),
        "pairs": [{
            "pair": list(pair), "kind": kinds[index], "required_m": float(required[index]),
            "minimum_m": float(distance[:, index].min()),
            "collision_samples": int(np.sum(status[:, index] == 1)),
            "below_clearance_samples": int(np.sum(distance[:, index] < required[index])),
        } for index, pair in enumerate(labels)],
    }
    for kind, expected in (("external", 10 * (2 + len(scene["obstacles"]))), ("self", 36)):
        selected = kinds == kind
        count = int(np.sum(selected))
        if count != expected:
            result[kind] = {"status": "insufficient_evidence", "measured_pairs": count, "required_pairs": expected}
            continue
        unsafe = (status[:, selected] == 1) | (distance[:, selected] < required[selected])
        result[kind] = {
            "status": "fail" if np.any(unsafe) else "pass",
            "measured_pairs": count, "required_pairs": expected, "samples": len(measured),
            "minimum_m": float(distance[:, selected].min()),
            "unsafe_samples": int(np.sum(np.any(unsafe, axis=1))),
        }
    return result
