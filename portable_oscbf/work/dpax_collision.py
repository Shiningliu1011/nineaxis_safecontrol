#!/usr/bin/env python3
"""DCOL (dpax) differentiable collision kernel for the portable OSCBF core.

The self-collision geometry is the M2 OBB model (one OBB per link).  Each OBB
is represented by its 12 edges; the OBB-OBB distance is the minimum over the
12x12 edge-pair distances computed by ``dpax.endpoints.proximity`` (capsule
proximity with zero radius, i.e. segment-segment distance).  The whole graph
is JAX-native, so ``jax.grad`` of a pair distance with respect to ``q`` gives
the CBF gradient row directly.

Reference implementation note: DCOLuse/dpax_collision.py used polytope
proximity for boxes but never converted its scale factor ``alpha`` into a
metric distance.  The edge-based kernel here returns a true distance in
metres, which is what the CBF rows and the FCL validation baseline require.
"""

from __future__ import annotations

import jax
import jax.numpy as jnp
import numpy as np

from dpax.endpoints import proximity

from work.nineaxis_manipulator_jax import JOINT_CHAIN, _twist_exp_jax
from work.obb_collision_model import (
    OBB_COLLISION_PAIRS,
    OBB_HALF_EXTENTS_M,
    OBB_LINK_NAMES,
    OBB_LOCAL_CENTERS_M,
    OBB_LOCAL_ROTATIONS,
)


# OBB corner/edge bookkeeping in local coordinates (half extents applied later).
_CORNERS = jnp.array(
    [[x, y, z] for x in (-1.0, 1.0) for y in (-1.0, 1.0) for z in (-1.0, 1.0)],
    dtype=jnp.float64,
)
_EDGES = jnp.array(
    [
        [0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3],
        [2, 6], [3, 7], [4, 5], [4, 6], [5, 7], [6, 7],
    ],
    dtype=jnp.int32,
)

# Face bookkeeping: for each of the 6 OBB faces, the local face normal axis,
# the two local tangent axes, and the sign of the normal (+1/-1).
_FACE_NORMAL_AXIS = jnp.array([0, 0, 1, 1, 2, 2], dtype=jnp.int32)
_FACE_NORMAL_SIGN = jnp.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
_FACE_TANGENT_AXES = jnp.array(
    [
        [1, 2], [1, 2],
        [0, 2], [0, 2],
        [0, 1], [0, 1],
    ],
    dtype=jnp.int32,
)

def _build_robot():
    """Build the shared JAX model once at import time (never inside a trace)."""

    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX

    robot = NineaxisManipulatorJAX()
    return robot.S_axes, robot._M_links


_SCREW_AXES, _M_LINKS = _build_robot()

# Static (10 x 8) OBB-obstacle grid used by the fixed 8-slot obstacle kernel.
# Plain numpy indices keep the grid constant during JIT tracing.
_OBB_OBS_GRID = np.asarray(
    [(link, slot) for link in range(10) for slot in range(8)],
    dtype=np.int32,
)


def link_transforms(q: jnp.ndarray) -> jnp.ndarray:
    """World transforms of all links in ``JOINT_CHAIN`` order (11, 4, 4)."""

    transforms = []
    accumulator = jnp.eye(4)
    joint_index = 0
    for parent, child, jtype, *_ in JOINT_CHAIN:
        if jtype in ("revolute", "prismatic"):
            accumulator = accumulator @ _twist_exp_jax(
                _SCREW_AXES[joint_index], q[joint_index])
            joint_index += 1
        transforms.append(accumulator @ _M_LINKS.get(child, jnp.eye(4)))
    return jnp.stack(transforms)


def _obb_corners_world(transform, center_local, rotation_local, half_extents):
    """World-space 8 corners of one OBB from its link transform."""

    rotation = transform[:3, :3] @ rotation_local
    center = transform[:3, :3] @ center_local + transform[:3, 3]
    scaled = _CORNERS * half_extents[None, :]
    return center[None, :] + (rotation @ scaled.T).T


def _edge_edge_minimum(corners_i, corners_j) -> jnp.ndarray:
    """Minimum over the 12x12 edge pairs (via dpax segment proximity)."""

    a1 = jnp.repeat(corners_i[_EDGES[:, 0]], 12, axis=0)
    b1 = jnp.repeat(corners_i[_EDGES[:, 1]], 12, axis=0)
    a2 = jnp.tile(corners_j[_EDGES[:, 0]], (12, 1))
    b2 = jnp.tile(corners_j[_EDGES[:, 1]], (12, 1))
    phi = _proximity_batch(0.0, a1, b1, 0.0, a2, b2)
    return jnp.sqrt(jnp.maximum(jnp.min(phi), 0.0))


def _point_face_minimum(transform_i, center_i, rotation_i, half_i,
                        transform_j, center_j, rotation_j, half_j) -> jnp.ndarray:
    """Minimum point-to-face distance between two OBBs (scalar).

    For every vertex of box A vs every face of box B and vice versa, the point
    is projected onto the face plane and clipped to the face rectangle.  The
    minimum of edge-edge (segment pairs) and point-face exactly equals the
    OBB-OBB distance for disjoint convex boxes.
    """

    corners_i = _obb_corners_world(transform_i, center_i, rotation_i, half_i)
    corners_j = _obb_corners_world(transform_j, center_j, rotation_j, half_j)
    faces_i = _obb_faces_world(transform_i, center_i, rotation_i, half_i)
    faces_j = _obb_faces_world(transform_j, center_j, rotation_j, half_j)
    distances_ij = _point_face_distances(corners_i, faces_j)
    distances_ji = _point_face_distances(corners_j, faces_i)
    return jnp.minimum(jnp.min(distances_ij), jnp.min(distances_ji))


def _obb_faces_world(transform, center_local, rotation_local, half_extents):
    """World-space face parameters: centers (6,3), tangents (6,2,3), widths (6,2)."""

    rotation = transform[:3, :3] @ rotation_local
    center = transform[:3, :3] @ center_local + transform[:3, 3]
    normal_directions = rotation[:, _FACE_NORMAL_AXIS] * _FACE_NORMAL_SIGN[None, :]
    face_centers = center[None, :] + (normal_directions * half_extents[_FACE_NORMAL_AXIS][None, :]).T
    tangent_a = rotation[:, _FACE_TANGENT_AXES[:, 0]].transpose(1, 0)
    tangent_b = rotation[:, _FACE_TANGENT_AXES[:, 1]].transpose(1, 0)
    tangents = jnp.stack([tangent_a, tangent_b], axis=1)
    widths = jnp.stack([
        half_extents[_FACE_TANGENT_AXES[:, 0]],
        half_extents[_FACE_TANGENT_AXES[:, 1]],
    ], axis=1)
    return face_centers, tangents, widths


def _point_face_distances(corners, faces):
    """Point-to-rectangle distances: corners (8,3) x faces (6 centers/tangents/widths)."""

    centers, tangents, widths = faces
    deltas = corners[:, None, :] - centers[None, :, :]  # (8,6,3)
    s = jnp.sum(deltas * tangents[None, :, 0, :], axis=-1)  # (8,6)
    t = jnp.sum(deltas * tangents[None, :, 1, :], axis=-1)
    s_clipped = jnp.clip(s, -widths[None, :, 0], widths[None, :, 0])
    t_clipped = jnp.clip(t, -widths[None, :, 1], widths[None, :, 1])
    closest = (
        centers[None, :, :]
        + tangents[None, :, 0, :] * s_clipped[:, :, None]
        + tangents[None, :, 1, :] * t_clipped[:, :, None])
    return jnp.linalg.norm(corners[:, None, :] - closest, axis=-1)


def _obb_pair_distance(transform_i, center_i, rotation_i, half_i,
                       transform_j, center_j, rotation_j, half_j) -> jnp.ndarray:
    """Exact distance between two disjoint OBBs (scalar)."""

    corners_i = _obb_corners_world(
        transform_i, center_i, rotation_i, half_i)
    corners_j = _obb_corners_world(
        transform_j, center_j, rotation_j, half_j)
    edge_min = _edge_edge_minimum(corners_i, corners_j)
    face_min = _point_face_minimum(
        transform_i, center_i, rotation_i, half_i,
        transform_j, center_j, rotation_j, half_j)
    return jnp.minimum(edge_min, face_min)


_proximity_batch = jax.jit(
    jax.vmap(proximity, in_axes=(None, 0, 0, None, 0, 0)))
_pair_distance_vmap = jax.jit(
    jax.vmap(_obb_pair_distance,
             in_axes=(0, 0, 0, 0, 0, 0, 0, 0)))


def self_collision_distances(q: jnp.ndarray) -> jnp.ndarray:
    """Per-pair OBB self-collision distances (14,) for the M2 topology."""

    transforms = link_transforms(q)
    pair_indices = OBB_COLLISION_PAIRS
    transforms_i = transforms[pair_indices[:, 0]]
    transforms_j = transforms[pair_indices[:, 1]]
    centers_i = OBB_LOCAL_CENTERS_M[pair_indices[:, 0]]
    centers_j = OBB_LOCAL_CENTERS_M[pair_indices[:, 1]]
    rotations_i = OBB_LOCAL_ROTATIONS[pair_indices[:, 0]]
    rotations_j = OBB_LOCAL_ROTATIONS[pair_indices[:, 1]]
    half_i = OBB_HALF_EXTENTS_M[pair_indices[:, 0]]
    half_j = OBB_HALF_EXTENTS_M[pair_indices[:, 1]]
    return _pair_distance_vmap(
        transforms_i, centers_i, rotations_i, half_i,
        transforms_j, centers_j, rotations_j, half_j)


def pair_distance(q: jnp.ndarray, pair_index: int) -> jnp.ndarray:
    """Distance of a single topology pair (scalar), JIT-stable shape."""

    return self_collision_distances(q)[pair_index]


self_collision_distances_jit = jax.jit(self_collision_distances)


def self_collision_distance_grad(q: jnp.ndarray, pair_index: int) -> jnp.ndarray:
    """CBF distance gradient d(distance)/dq for one pair (9,)."""

    return jax.grad(lambda qq: pair_distance(qq, pair_index))(q)


def _pair_grad_wrapper(q: jnp.ndarray, pair_index: int) -> jnp.ndarray:
    return jax.grad(lambda x: self_collision_distances(x)[pair_index])(q)


self_collision_grads_jit = jax.jit(
    jax.vmap(_pair_grad_wrapper, in_axes=(None, 0)))


def self_collision_distance_grad_jit(q: jnp.ndarray,
                                     pair_index: int) -> jnp.ndarray:
    """Jitted gradient wrapper (single compilation for all pair indices)."""

    indices = jnp.arange(self_collision_distances(q).shape[0])
    return self_collision_grads_jit(q, indices)[pair_index]


def _obb_sphere_pair_distance(transform, center_local, rotation_local, half,
                              sphere_position, sphere_radius) -> jnp.ndarray:
    """Surface distance between one OBB and one sphere (scalar, metres)."""

    corners = _obb_corners_world(transform, center_local, rotation_local, half)
    edges_a = corners[_EDGES[:, 0]]
    edges_b = corners[_EDGES[:, 1]]
    # A sphere is a zero-length capsule.  dpax's active-set QP returns NaN for
    # degenerate endpoints on some segments, so the equivalent analytic
    # segment-to-point distance is used (same min_dist^2 - r^2 geometry).
    edge_surface = jnp.min(
        _segment_point_surface_vmap(
            edges_a, edges_b, sphere_position, sphere_radius))

    faces = _obb_faces_world(transform, center_local, rotation_local, half)
    point_face = _point_face_distances(
        sphere_position[None, :], faces)[0]
    face_surface = jnp.min(point_face) - sphere_radius
    return jnp.maximum(jnp.minimum(edge_surface, face_surface), 0.0)


def _segment_point_surface(a, b, point, radius) -> jnp.ndarray:
    direction = b - a
    denominator = jnp.sum(direction * direction)
    t = jnp.clip(
        jnp.sum((point - a) * direction) / jnp.maximum(denominator, 1e-12),
        0.0, 1.0)
    segment_distance = jnp.linalg.norm((a + t * direction) - point)
    return jnp.maximum(segment_distance - radius, 0.0)


_segment_point_surface_vmap = jax.jit(
    jax.vmap(_segment_point_surface, in_axes=(0, 0, None, None)))


_obb_sphere_pair_vmap = jax.jit(
    jax.vmap(_obb_sphere_pair_distance,
             in_axes=(0, 0, 0, 0, 0, 0)))


def obb_sphere_distances(q: jnp.ndarray,
                         obs_pos: jnp.ndarray,
                         obs_radii: jnp.ndarray) -> jnp.ndarray:
    """OBB-to-sphere surface distances (N_obb, N_obs) for the fixed 8 slots."""

    transforms = link_transforms(q)
    link_indices = _OBB_OBS_GRID[:, 0]
    obs_indices = _OBB_OBS_GRID[:, 1]
    distances = _obb_sphere_pair_vmap(
        transforms[link_indices],
        OBB_LOCAL_CENTERS_M[link_indices],
        OBB_LOCAL_ROTATIONS[link_indices],
        OBB_HALF_EXTENTS_M[link_indices],
        obs_pos[obs_indices],
        obs_radii[obs_indices],
    )
    return distances.reshape(len(OBB_LINK_NAMES), obs_pos.shape[0])


def obb_sphere_clearance(q: jnp.ndarray,
                         obs_pos: jnp.ndarray,
                         obs_radii: jnp.ndarray,
                         obs_vel: jnp.ndarray,
                         obs_radius_dot: jnp.ndarray,
                         obs_d_safe: jnp.ndarray) -> tuple:
    """CBF clearance and time terms for every OBB x obstacle slot.

    Returns ``(h_obs, h_dot_obs)`` both shaped (N_obb, N_obs):
    ``h_obs = d - d_safe`` and ``h_dot_obs = -n·v_obs - r_dot`` obtained from
    the DCOL distance gradient (time terms vanish for static obstacles).
    """

    distances = obb_sphere_distances(q, obs_pos, obs_radii)
    h_obs = distances - obs_d_safe[None, :]

    # Gradient of each pair distance wrt that obstacle's position/radius.
    transforms = link_transforms(q)
    link_indices = _OBB_OBS_GRID[:, 0]
    obs_indices = _OBB_OBS_GRID[:, 1]

    def _pair_grad(position, radius, transform, center_local, rotation_local,
                   half):
        return jax.grad(
            lambda p, r: _obb_sphere_pair_distance(
                transform, center_local, rotation_local, half, p, r),
            argnums=(0, 1),
        )(position, radius)

    grad_p, grad_r = jax.vmap(
        _pair_grad, in_axes=(0, 0, 0, 0, 0, 0))(
            obs_pos[obs_indices],
            obs_radii[obs_indices],
            transforms[link_indices],
            OBB_LOCAL_CENTERS_M[link_indices],
            OBB_LOCAL_ROTATIONS[link_indices],
            OBB_HALF_EXTENTS_M[link_indices],
        )
    grad_p = grad_p.reshape(len(OBB_LINK_NAMES), obs_pos.shape[0], 3)
    grad_r = grad_r.reshape(len(OBB_LINK_NAMES), obs_pos.shape[0])
    h_dot_obs = (
        jnp.sum(grad_p * obs_vel[None, :, :], axis=-1)
        + grad_r * obs_radius_dot[None, :])
    return h_obs, h_dot_obs
