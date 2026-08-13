#!/usr/bin/env python3
"""FCL-assisted placement for dynamic obstacle test scenes."""

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import fcl
import numpy as np

from point_cloud_obstacles import FCLPointCloudCollision


DEFAULT_DYNAMIC_TARGET_H = 0.04
DEFAULT_DYNAMIC_MIN_H = 0.01
DEFAULT_DYNAMIC_SAFETY_MIN_H = 0.025
DEFAULT_DYNAMIC_N_CANDIDATES = 6000
DEFAULT_DYNAMIC_PLACEMENT_SEED = 23


@dataclass
class PlacedObstacle:
    """A dynamic obstacle candidate selected near the robot surface."""

    center: np.ndarray
    radius: float
    h_value: float
    body_name: str
    center_distance: float
    active_sample_count: int = 1
    transition_min_h: float = float("inf")


def _distance_to_point(
    collision: FCLPointCloudCollision,
    point: np.ndarray,
) -> Tuple[float, str]:
    """Return the nearest FCL robot-body distance to a point center."""
    req = fcl.DistanceRequest()
    req.enable_nearest_points = True
    pt_obj = fcl.CollisionObject(
        fcl.Sphere(0.0), fcl.Transform(point.astype(np.float64))
    )

    best_dist = float("inf")
    best_body = ""
    for body_name, obj, _ in collision.get_all_bodies():
        res = fcl.DistanceResult()
        dist = fcl.distance(obj, pt_obj, req, res)
        if dist < -0.5:
            continue
        if dist < best_dist:
            best_dist = float(dist)
            best_body = body_name
    return best_dist, best_body


def fcl_center_distance_to_robot(kin, q: np.ndarray, center: np.ndarray) -> Tuple[float, str]:
    """Compute the nearest FCL robot-body surface distance to a point center."""
    collision = FCLPointCloudCollision()
    collision.update_poses(kin.forward_kinematics(np.asarray(q, dtype=float)))
    return _distance_to_point(collision, np.asarray(center, dtype=float))


def _body_priority_penalty(body_name: str) -> float:
    if body_name.startswith(("joint_", "capsule_")):
        return 0.0
    if body_name == "link1_box":
        return 0.02
    return 0.25


def _as_array3(value: Sequence[float], name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {arr.shape}")
    return arr


def auto_place_dynamic_obstacles_fcl(
    kin,
    q_samples: Iterable[np.ndarray],
    radii: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
    d_safe: float,
    activation: float,
    target_h: float = 0.05,
    n_candidates: int = 2000,
    seed: int = 11,
    min_h: float = 0.0,
    min_center_separation: float = 0.12,
    safety_q_samples: Iterable[np.ndarray] = None,
    safety_min_h: float = None,
    trajectory_q_samples: Iterable[np.ndarray] = None,
    score_weights: dict = None,
    body_preference: Sequence[str] = None,
) -> List[PlacedObstacle]:
    """Select dynamic-obstacle centers with FCL inside the CBF activation band.

    The score uses ``h = distance(robot_body, center) - radius - d_safe``. A
    useful test obstacle should be outside penetration (``h >= min_h``), inside
    the CBF activation band, and close enough to ``target_h`` to trigger a
    visible avoidance response.
    """
    q_list = [np.asarray(q, dtype=float) for q in q_samples]
    if not q_list:
        raise ValueError("q_samples must contain at least one configuration")

    radius_list = [float(r) for r in radii]
    if not radius_list:
        return []
    if any(r <= 0.0 for r in radius_list):
        raise ValueError("all radii must be positive")

    lower_arr = _as_array3(lower, "lower")
    upper_arr = _as_array3(upper, "upper")
    if np.any(upper_arr <= lower_arr):
        raise ValueError("upper bounds must be greater than lower bounds")
    if n_candidates <= 0:
        raise ValueError("n_candidates must be positive")

    collisions = []
    for q in q_list:
        collision = FCLPointCloudCollision()
        collision.update_poses(kin.forward_kinematics(q))
        collisions.append(collision)

    trajectory_collisions = []
    if trajectory_q_samples is not None:
        for q in trajectory_q_samples:
            collision = FCLPointCloudCollision()
            collision.update_poses(kin.forward_kinematics(np.asarray(q, dtype=float)))
            trajectory_collisions.append(collision)
    else:
        trajectory_collisions = collisions

    safety_collisions = []
    if safety_q_samples is not None and safety_min_h is not None:
        for q in safety_q_samples:
            collision = FCLPointCloudCollision()
            collision.update_poses(kin.forward_kinematics(np.asarray(q, dtype=float)))
            safety_collisions.append(collision)

    rng = np.random.default_rng(seed)
    candidates = rng.uniform(lower_arr, upper_arr, size=(int(n_candidates), 3))
    weights = {
        "target": 1.0,
        "active": 0.006,
        "body": 1.0,
        "transition": 0.5,
    }
    if score_weights:
        weights.update(score_weights)
    preferred = tuple(body_preference or ())

    placements: List[PlacedObstacle] = []
    for radius in radius_list:
        scored = []
        for center in candidates:
            min_dist = float("inf")
            min_body = ""
            active_sample_count = 0
            for collision in trajectory_collisions:
                dist, body_name = _distance_to_point(collision, center)
                h_sample = dist - radius - float(d_safe)
                if h_sample <= float(activation):
                    active_sample_count += 1
                if dist < min_dist:
                    min_dist = dist
                    min_body = body_name

            h_value = min_dist - radius - float(d_safe)
            if h_value < float(min_h) or h_value > float(activation):
                continue

            safe = True
            transition_min_h = h_value
            if safety_collisions:
                transition_min_h = float("inf")
                for collision in safety_collisions:
                    dist, _ = _distance_to_point(collision, center)
                    h_safe = dist - radius - float(d_safe)
                    transition_min_h = min(transition_min_h, h_safe)
                    if h_safe < float(safety_min_h):
                        safe = False
                        break
            if not safe:
                continue

            body_penalty = _body_priority_penalty(min_body)
            if preferred and any(token in min_body for token in preferred):
                body_penalty *= 0.25
            transition_penalty = 0.0
            if safety_min_h is not None:
                transition_penalty = max(0.0, float(safety_min_h) - transition_min_h)
            score = (
                weights["target"] * abs(h_value - float(target_h))
                + weights["body"] * body_penalty
                + weights["transition"] * transition_penalty
                - weights["active"] * active_sample_count
            )
            scored.append((
                score,
                PlacedObstacle(
                    center=np.asarray(center, dtype=float).copy(),
                    radius=radius,
                    h_value=float(h_value),
                    body_name=min_body,
                    center_distance=float(min_dist),
                    active_sample_count=int(active_sample_count),
                    transition_min_h=float(transition_min_h),
                ),
            ))

        if not scored:
            raise RuntimeError(
                "FCL obstacle placement found no candidates inside the activation band"
            )

        scored.sort(key=lambda item: item[0])
        selected = None
        for _, candidate in scored:
            if all(
                np.linalg.norm(candidate.center - prev.center) >= min_center_separation
                for prev in placements
            ):
                selected = candidate
                break
        if selected is None:
            selected = scored[0][1]
        placements.append(selected)

    return placements
