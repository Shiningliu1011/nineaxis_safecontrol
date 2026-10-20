from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from work.kinematics_data import JOINT_CHAIN
from work.obb_collision_model import OBB_HALF_EXTENTS_M, OBB_LINK_NAMES, OBB_LOCAL_CENTERS_M, OBB_LOCAL_ROTATIONS


def _skew(axis):
    x, y, z = axis
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


_ORIGINS = []
for _row in JOINT_CHAIN:
    _matrix = np.eye(4)
    _matrix[:3, :3] = Rotation.from_euler("xyz", _row[6:9]).as_matrix()
    _matrix[:3, 3] = _row[3:6]
    _ORIGINS.append(_matrix)


def transform_derivatives(state):
    batch = transform_derivatives_batch([state])
    return {name: tuple(matrix[0] for matrix in values) for name, values in batch.items()}


def transform_derivatives_batch(states):
    count = len(states)
    if not count:
        raise ValueError("运动变换批次不能为空")
    values = (np.asarray(states) if isinstance(states, np.ndarray) else np.asarray([
        (state.position, state.velocity, state.acceleration) for state in states
    ]))
    if values.shape != (count, 3, 9) or not np.all(np.isfinite(values)):
        raise ValueError("运动变换需要有限的九轴状态批次")
    positions, velocities, accelerations = values[:, 0], values[:, 1], values[:, 2]
    identity = np.broadcast_to(np.eye(4), (count, 4, 4))
    zeros = np.zeros((count, 4, 4))
    transforms = {"world": (identity, zeros, zeros)}
    joint = 0
    for row, origin in zip(JOINT_CHAIN, _ORIGINS):
        parent, child, kind = row[:3]
        motion, first, second = identity.copy(), zeros.copy(), zeros.copy()
        if kind != "fixed":
            q, velocity, acceleration = positions[:, joint], velocities[:, joint], accelerations[:, joint]
            axis = np.asarray(row[9])
            if kind == "prismatic":
                motion[:, :3, 3] = q[:, None] * axis
                first[:, :3, 3] = velocity[:, None] * axis
                second[:, :3, 3] = acceleration[:, None] * axis
            else:
                skew = _skew(axis)
                motion[:, :3, :3] = Rotation.from_rotvec(q[:, None] * axis).as_matrix()
                first[:, :3, :3] = motion[:, :3, :3] @ skew * velocity[:, None, None]
                second[:, :3, :3] = motion[:, :3, :3] @ (
                    skew @ skew * velocity[:, None, None] ** 2 + skew * acceleration[:, None, None]
                )
            joint += 1
        fixed, derivative, curvature = origin @ motion, origin @ first, origin @ second
        prior, prior_first, prior_second = transforms[parent]
        transforms[child] = (
            prior @ fixed,
            prior_first @ fixed + prior @ derivative,
            prior_second @ fixed + 2 * prior_first @ derivative + prior @ curvature,
        )
    return transforms


class SweptGeometry:
    def __init__(self, scene, meshes):
        self.external_vertices = {}
        for index, name in enumerate(OBB_LINK_NAMES):
            corners = np.array(np.meshgrid(*[[-value, value] for value in OBB_HALF_EXTENTS_M[index]])).T.reshape(-1, 3)
            self.external_vertices[name] = corners @ OBB_LOCAL_ROTATIONS[index].T + OBB_LOCAL_CENTERS_M[index]
        self.self_vertices = {mesh.link: mesh.vertices for mesh in meshes}
        self.scene = scene
        self.boxes = {
            "table": (np.asarray(scene["table"]["center_m"]), np.asarray(scene["table"]["half_extents_m"])),
            **{row["name"]: (np.asarray(row["center_m"]), np.full(3, row["radius_m"]))
               for row in scene["obstacles"]},
        }

    @staticmethod
    def _third_bound(link, radius, bounds):
        speed, acceleration, jerk = 0., 0., 0.
        translation, joint = 0., 0
        for row in JOINT_CHAIN:
            translation += np.linalg.norm(row[3:6]) * (speed ** 3 + 3 * speed * acceleration + jerk)
            if row[2] == "prismatic":
                translation += bounds.max_jerk[joint]
                joint += 1
            elif row[2] == "revolute":
                speed += bounds.max_velocity[joint]
                acceleration += bounds.max_acceleration[joint]
                jerk += bounds.max_jerk[joint]
                joint += 1
            if row[1] == link:
                break
        return translation + radius * (speed ** 3 + 3 * speed * acceleration + jerk)

    def lower_bounds(self, rows, state, bounds, radius, *, transforms=None, third_bounds=None):
        transforms = transform_derivatives(state) if transforms is None else transforms
        return self.lower_bounds_batch(
            [rows], bounds, np.array([radius]),
            {name: tuple(matrix[None] for matrix in values) for name, values in transforms.items()},
            third_bounds=third_bounds,
        )[0]

    def lower_bounds_batch(self, batches, bounds, radii, transforms, *, third_bounds=None):
        third = {} if third_bounds is None else third_bounds
        radii = np.asarray(radii)[:, None]
        result = np.empty((len(batches), len(batches[0])))
        for column, row in enumerate(batches[0]):
            selected = [batch[column] for batch in batches]
            if any(not item.distance.valid or item.pair != row.pair for item in selected):
                raise ValueError("分离平面检查要求相同对象及有效距离")
            delta = np.array([item.distance.second_point - item.distance.first_point for item in selected])
            normals = delta / np.linalg.norm(delta, axis=1)[:, None]
            extrema = []
            for index, link in enumerate(row.pair):
                if link not in OBB_LINK_NAMES:
                    if link == "workpiece":
                        cylinder = self.scene["workpiece"]
                        extrema.append(
                            normals @ cylinder["center_m"]
                            - cylinder["radius_m"] * np.linalg.norm(normals[:, [0, 2]], axis=1)
                            - cylinder["height_m"] / 2 * abs(normals[:, 1])
                        )
                    else:
                        center, half = self.boxes[link]
                        extrema.append(normals @ center - abs(normals) @ half)
                    continue
                key = row.kind, link
                vertices = (self.self_vertices if row.kind == "self" else self.external_vertices)[link]
                homogeneous = np.column_stack([vertices, np.ones(len(vertices))])
                if key not in third:
                    third[key] = self._third_bound(link, np.linalg.norm(vertices, axis=1).max(), bounds)
                projection, velocity, acceleration = (
                    np.einsum("ni,nij->nj", normals, matrix[:, :3]) @ homogeneous.T
                    for matrix in transforms[link]
                )
                error = (np.abs(velocity) * radii + .5 * np.abs(acceleration) * radii ** 2
                         + third[key] * radii ** 3 / 6)
                extrema.append(
                    np.max(projection + error, axis=1) if index == 0
                    else np.min(projection - error, axis=1),
                )
            result[:, column] = extrema[1] - extrema[0] - row.required_m - 1e-12
        return result
