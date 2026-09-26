from dataclasses import replace
import time

import numpy as np

from work.collision_safety import CollisionScene, CollisionStatus, SceneMetadata, SceneTracks


def signed_scene(scene):
    return scene._replace(metadata=replace(scene.metadata, checksum=scene.compute_checksum()))


def scene_input(config, identities, status=CollisionStatus.OK, *, points=None, radii=None,
                mask=None, tracks=None, velocity=None):
    count = config.max_support_points
    capacity = config.policy.artifact.parameters["max_tracks"]["value"]
    stamp = time.monotonic_ns()
    active = np.asarray(mask if mask is not None else [True, True, False], dtype=bool)
    tracking = np.asarray(tracks if tracks is not None else [0, 1, 0], dtype=np.int32)
    speed = np.zeros((count, 3)) if velocity is None else np.asarray(velocity, dtype=np.float64)
    track_indices = np.flatnonzero(active & (tracking == 1))
    track_ids = np.full(capacity, -1, dtype=np.int32)
    track_active = np.arange(capacity) < len(track_indices)
    track_velocity = np.zeros((capacity, 3))
    associations = np.full(count, -1, dtype=np.int32)
    for slot, support in enumerate(track_indices):
        track_ids[slot] = support + 1000
        associations[support] = track_ids[slot]
        track_velocity[slot] = speed[support]
    scene = CollisionScene(
        support_points_m=(np.asarray(points) if points is not None else
                          np.asarray([[.1, .2, .3], [.4, .5, .6], [0., 0., 0.]], dtype=np.float32)),
        support_radii_mm=np.asarray(radii if radii is not None else [10., 20., 0.]),
        required_clearance_mm=np.full(count, config.policy.artifact.parameters["environment_clearance_mm"]["value"]),
        support_velocity_m_s=speed,
        support_ids=np.asarray([11, 12, 0], dtype=np.int32),
        support_mask=active, source_stamp_ns=np.int64(stamp), prepared_stamp_ns=np.int64(stamp),
        status=np.int32(status), support_track_status=tracking, support_valid_mask=active.copy(),
        support_track_ids=associations,
        tracks=SceneTracks(
            track_ids, track_active, track_active.copy(), track_velocity, np.zeros(capacity), np.zeros(capacity),
            np.full(capacity, stamp, dtype=np.int64), np.full(capacity, stamp + 180_000_000_000, dtype=np.int64),
        ),
        identities=identities, coverage_space_hash=np.array(identities.required_space_hash),
        coverage_valid_until_ns=np.int64(stamp + 180_000_000_000),
        metadata=SceneMetadata(1, "base_link", "m", "mm", "m/s", "ns", "monotonic",
                               int(np.sum(active)), len(track_indices), True, b""),
    )
    return signed_scene(scene)
