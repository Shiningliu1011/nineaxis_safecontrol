from __future__ import annotations

import hashlib
import itertools
import json
import math
from dataclasses import dataclass

import jax
import jax.numpy as jnp
import numpy as np

from work.collision_parameters import CollisionPolicy
from work.kinematics_data import JOINT_CHAIN


@dataclass(frozen=True)
class _SelfGeometry:
    centers: jax.Array
    radii: jax.Array
    link_indices: jax.Array
    pairs: jax.Array
    margins: jax.Array
    active_ids: jax.Array
    slot_capacity: int

    @classmethod
    def from_artifact(cls, document: dict, policy: CollisionPolicy) -> _SelfGeometry:
        # 内容复制与 hash 核验发生在构造阶段；查询期间只读取固定 device arrays。
        data = json.loads(json.dumps(document, allow_nan=False))
        geometry = data["geometry"]
        encoded = json.dumps(
            geometry, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
        ).encode("utf-8")
        digest = hashlib.sha256(encoded).digest()
        if data["geometry_hash"] != digest.hex() or digest != policy.geometry_hash:
            raise ValueError("geometry_hash does not match geometry and policy")
        if type(geometry["schema_version"]) is not int or geometry["schema_version"] != 1:
            raise ValueError("unsupported geometry schema_version")
        capacity = geometry["slot_capacity"]
        if type(capacity) is not int or capacity <= 0:
            raise ValueError("geometry slot_capacity must be a positive integer")
        if capacity != policy.artifact.parameters["max_ellipsoids_per_link"]["value"]:
            raise ValueError("geometry slot_capacity differs from parameter artifact")
        links = geometry["links"]
        names = [entry["link"] for entry in links]
        chain_names = [entry[1] for entry in JOINT_CHAIN]
        if len(names) < 2 or len(set(names)) != len(names) or any(n not in chain_names for n in names):
            raise ValueError("geometry links must be distinct known kinematic links")
        expected_pairs = {tuple(sorted(pair)) for pair in itertools.combinations(names, 2)}
        candidates = data["self_collision"]["candidate_pairs"]
        if any(len(pair) != 2 for pair in candidates):
            raise ValueError("candidate_pairs must contain two links per pair")
        actual_pairs = [tuple(sorted(pair)) for pair in candidates]
        if len(actual_pairs) != len(expected_pairs) or set(actual_pairs) != expected_pairs:
            raise ValueError("candidate_pairs must cover every link pair exactly once")
        contacts = data["self_collision"]["allowed_contacts"]
        if contacts != [dict(entry) | {"links": list(entry["links"])} for entry in policy.allowed_contacts]:
            raise ValueError("geometry allowed_contacts differ from policy")
        excluded = {tuple(sorted(entry["links"])) for entry in contacts}
        if not excluded <= expected_pairs:
            raise ValueError("allowed_contacts contain unknown geometry links")
        clearances = {
            tuple(sorted(entry["links"])): entry["clearance"]["value"]
            for entry in policy.artifact.self_clearance_mm
        }
        if set(clearances) != expected_pairs:
            raise ValueError("self_clearance_mm must cover exactly the geometry link pairs")

        centers, radii, link_indices, active = [], [], [], []
        for entry in links:
            if len(entry["slots"]) != capacity:
                raise ValueError("geometry slots differ from slot_capacity")
            for slot in entry["slots"]:
                center, radius = np.asarray(slot["center_m"]), np.asarray(slot["radii_m"])
                if (center.shape != (3,) or radius.shape != (3,)
                        or not np.all(np.isfinite(center)) or not np.all(np.isfinite(radius))
                        or np.any(radius <= 0.0) or type(slot["active_mask"]) is not bool):
                    raise ValueError("geometry slot requires finite center, positive radii and boolean mask")
                centers.append(center)
                radii.append(radius)
                active.append(slot["active_mask"])
                link_indices.append(chain_names.index(entry["link"]))
            if not any(active[-capacity:]):
                raise ValueError("each geometry link must have an active ellipsoid")

        pairs, margins = [], []
        for first, second in itertools.combinations(range(len(names)), 2):
            key = tuple(sorted((names[first], names[second])))
            if key in excluded:
                continue
            for sa, sb in itertools.product(range(capacity), repeat=2):
                ia, ib = first * capacity + sa, second * capacity + sb
                if active[ia] and active[ib]:
                    shortest_sum = float(min(radii[ia])) + float(min(radii[ib]))
                    margin = 1.0 + clearances[key] * 1e-3 / shortest_sum
                    if not math.isfinite(shortest_sum) or not math.isfinite(margin):
                        raise ValueError("geometry and self_clearance_mm must produce a finite scale_margin")
                    pairs.append((ia, ib))
                    margins.append(margin)
        if len(pairs) > policy.artifact.parameters["max_primitive_rows"]["value"]:
            raise ValueError("max_primitive_rows cannot hold every active self pair")
        return cls(
            jnp.asarray(np.asarray(centers), dtype=jnp.float64),
            jnp.asarray(np.asarray(radii), dtype=jnp.float64),
            jnp.asarray(link_indices, dtype=jnp.int32),
            jnp.asarray(pairs, dtype=jnp.int32).reshape((-1, 2)),
            jnp.asarray(margins, dtype=jnp.float64),
            jnp.asarray(np.flatnonzero(active), dtype=jnp.int32),
            capacity,
        )


def _build_world_geometry(geometry: _SelfGeometry):
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX

    robot = NineaxisManipulatorJAX()

    def world_geometry(q: jax.Array) -> tuple[jax.Array, jax.Array]:
        poses = robot._compute_all_link_transforms(q)[geometry.link_indices]
        rotation = poses[:, :3, :3]
        centers = jnp.einsum("nij,nj->ni", rotation, geometry.centers) + poses[:, :3, 3]
        factors = rotation * geometry.radii[:, None, :]
        return centers, factors

    return world_geometry
