from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

import numpy as np
from robot_static_fcl import Body, measure, measure_poses

from work.nineaxis_kinematics import NineaxisKinematics
from work.kinematics_data import JOINT_CHAIN
from work.static_collision_meshes import load_meshes
from work.static_swept_geometry import SweptGeometry
from work.obb_collision_model import (
    OBB_HALF_EXTENTS_M, OBB_LINK_NAMES, OBB_LOCAL_CENTERS_M, OBB_LOCAL_ROTATIONS,
)


class GeometryQueryError(RuntimeError):
    pass


@dataclass(frozen=True)
class GeometryDistance:
    status: str
    distance_m: float
    first_point: np.ndarray | None
    second_point: np.ndarray | None
    valid: bool


@dataclass(frozen=True)
class CollisionMeasurement:
    pair: tuple[str, str]
    kind: str
    required_m: float
    distance: GeometryDistance
    gradient: np.ndarray | None

    def to_record(self) -> dict:
        return {
            "pair": list(self.pair), "kind": self.kind, "required_m": self.required_m,
            "distance_m": self.distance.distance_m,
            "margin_m": self.distance.distance_m - self.required_m,
            "status": self.distance.status, "valid": self.distance.valid,
            "first_point": None if self.distance.first_point is None else self.distance.first_point.tolist(),
            "second_point": None if self.distance.second_point is None else self.distance.second_point.tolist(),
            "gradient": None if self.gradient is None else self.gradient.tolist(),
        }


def distance_between(first: Body, second: Body) -> GeometryDistance:
    values = np.asarray(measure(first, second)).reshape(8)
    if values[0] == 1:
        return GeometryDistance("collision", 0.0, None, None, False)
    distance = float(values[1])
    if not np.isfinite(distance) or distance < 0:
        raise GeometryQueryError(f"FCL 距离查询无效：{distance}")
    if distance <= 1e-12:
        return GeometryDistance("collision", 0.0, None, None, False)
    points = values[2:].reshape(2, 3)
    if (points.shape != (2, 3) or not np.all(np.isfinite(points))
            or not np.isclose(np.linalg.norm(points[1] - points[0]), distance, atol=1e-7, rtol=1e-7)):
        raise GeometryQueryError("FCL 最近点与距离不一致")
    points.setflags(write=False)
    return GeometryDistance("separated", distance, points[0], points[1], True)


def _placed(body: Body, rotation: np.ndarray, translation: np.ndarray) -> Body:
    body.set_transform(rotation, translation)
    return body


def _vector(value, name: str, *, positive: bool = False) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)) or (positive and np.any(array <= 0)):
        raise ValueError(f"{name} 必须为有限的三维向量" + ("，各分量大于零" if positive else ""))
    return array


def _positive(value, name: str) -> float:
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise ValueError(f"{name} 必须为有限正数")
    return number


class StaticGeometry:
    def __init__(self, scene: dict):
        self.scene = deepcopy(scene)
        if self.scene["frame_id"] != "base_link":
            raise ValueError("静态几何要求 base_link 坐标")
        self.kinematics = NineaxisKinematics()
        self._boxes = [
            Body.box(2 * half) for half in OBB_HALF_EXTENTS_M
        ]
        source = self.scene.get("self_collision")
        meshes = () if source is None else load_meshes(source)
        self.swept = SweptGeometry(self.scene, meshes)
        self._meshes = [Body.convex(mesh.vertices, mesh.faces) for mesh in meshes]
        self._self_pairs = () if source is None else tuple(tuple(pair) for pair in source["pairs"])
        clearances = self.scene.get("clearances", {"external_m": .03, "self_m": .01})
        self.external_clearance_m = _positive(clearances["external_m"], "external_clearance_m")
        self.self_clearance_m = _positive(clearances["self_m"], "self_clearance_m")
        offsets = np.array([
            np.linalg.norm(row[3:6]) for row in JOINT_CHAIN
            if row[2] in ("prismatic", "revolute")
        ])
        self._motion_coefficients = {}
        for index, name in enumerate(OBB_LINK_NAMES):
            radius = np.linalg.norm(OBB_LOCAL_CENTERS_M[index]) + np.linalg.norm(OBB_HALF_EXTENTS_M[index])
            if meshes:
                radius = max(radius, meshes[index].radius_m)
            coefficients = np.zeros(9)
            if index:
                coefficients[0] = 1
            for joint in range(1, index):
                coefficients[joint] = np.sum(offsets[joint + 1:index]) + radius
            self._motion_coefficients[name] = coefficients
        workpiece, table = self.scene["workpiece"], self.scene["table"]
        if not np.allclose(_vector(workpiece["axis"], "workpiece.axis"), [0, 1, 0], atol=1e-12):
            raise ValueError("工件圆柱轴必须为 +Y")
        self._external = [
            ("workpiece", _placed(
                Body.cylinder(
                    _positive(workpiece["radius_m"], "workpiece.radius_m"),
                    _positive(workpiece["height_m"], "workpiece.height_m"),
                ),
                np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float),
                _vector(workpiece["center_m"], "workpiece.center_m"),
            )),
            ("table", _placed(
                Body.box(2 * _vector(table["half_extents_m"], "table.half_extents_m", positive=True)),
                np.eye(3), _vector(table["center_m"], "table.center_m"),
            )),
        ]
        names = {"workpiece", "table", *OBB_LINK_NAMES}
        for obstacle in self.scene["obstacles"]:
            name = obstacle["name"]
            if not isinstance(name, str) or not name or name in names:
                raise ValueError("障碍物名称必须非空且唯一")
            names.add(name)
            radius = _positive(obstacle["radius_m"], f"{name}.radius_m")
            self._external.append((name, _placed(
                Body.box(np.full(3, radius * 2)),
                np.eye(3), _vector(obstacle["center_m"], f"{name}.center_m"),
            )))

    @staticmethod
    def _point_jacobian(spatial: np.ndarray, point: np.ndarray, link_index: int) -> np.ndarray:
        value = spatial[3:] + np.cross(spatial[:3].T, point).T
        value[:, link_index:] = 0
        return value

    def displacement_coefficients(self, pair: tuple[str, str]) -> np.ndarray:
        first, second = pair
        if second not in self._motion_coefficients:
            return self._motion_coefficients[first].copy()
        # 两个连杆共同经历的上游刚体运动保持它们之间的距离。
        coefficients = self._motion_coefficients[second].copy()
        coefficients[:OBB_LINK_NAMES.index(first)] = 0
        return coefficients

    def batch_distances(self, link_transforms: np.ndarray, *, pair_indices=None) -> tuple[np.ndarray, tuple[tuple[str, str], ...]]:
        transforms = np.asarray(link_transforms, dtype=float)
        if transforms.ndim != 4 or transforms.shape[1:] != (10, 4, 4) or not np.all(np.isfinite(transforms)):
            raise ValueError("批量几何需要有限的 (N, 10, 4, 4) 连杆变换")
        poses = transforms.copy()
        poses[:, :, :3, :3] = transforms[:, :, :3, :3] @ OBB_LOCAL_ROTATIONS
        poses[:, :, :3, 3] += np.einsum("nbij,bj->nbi", transforms[:, :, :3, :3], OBB_LOCAL_CENTERS_M)
        if self._meshes:
            poses = np.concatenate([poses, transforms], axis=1)
        pairs = np.array([
            (robot, poses.shape[1] + external)
            for robot in range(10) for external in range(len(self._external))
        ] + [(10 + first, 10 + second) for first, second in self._self_pairs], dtype=np.int32)
        labels = (
            tuple((name, external[0]) for name in OBB_LINK_NAMES for external in self._external)
            + tuple((OBB_LINK_NAMES[first], OBB_LINK_NAMES[second]) for first, second in self._self_pairs)
        )
        if pair_indices is not None:
            selected = np.asarray(pair_indices)
            if (selected.ndim != 1 or selected.dtype.kind not in "iu" or not len(selected)
                    or np.any(selected < 0) or np.any(selected >= len(pairs))
                    or len(np.unique(selected)) != len(selected)):
                raise ValueError("几何组合索引必须为有效且不重复的整数")
            pairs = pairs[selected]
            labels = tuple(labels[index] for index in selected)
        values = np.asarray(measure_poses(
            self._boxes + self._meshes, [body for _, body in self._external], poses, pairs,
        ))
        separated = values[:, :, 0] == 0
        point_distance = np.linalg.norm(values[:, :, 5:8] - values[:, :, 2:5], axis=2)
        if (not np.all(np.isfinite(values))
                or not np.all(np.isclose(point_distance[separated], values[:, :, 1][separated], atol=1e-7, rtol=1e-7))):
            raise GeometryQueryError("批量 FCL 最近点与距离不一致")
        return values, labels

    def query(self, q: np.ndarray, *, gradients: bool = True, pair_indices=None) -> tuple[CollisionMeasurement, ...]:
        q = np.asarray(q, dtype=float)
        if q.shape != (9,) or not np.all(np.isfinite(q)):
            raise ValueError("几何查询需要九维有限关节位置")
        if gradients:
            transforms, spatial = self.kinematics.forward_kinematics_and_jacobian(q)
        else:
            transforms, spatial = self.kinematics.forward_kinematics(q), None
        values, labels = self.batch_distances(
            np.array([[transforms[name] for name in OBB_LINK_NAMES]]), pair_indices=pair_indices,
        )
        return self._measurements(values[0], labels, spatial=spatial)

    def query_transforms(self, transforms, *, pair_indices=None):
        values, labels = self.batch_distances(transforms, pair_indices=pair_indices)
        return tuple(self._measurements(batch, labels) for batch in values)

    def _measurements(self, values, labels, *, spatial=None):
        measurements = []
        for value, pair in zip(values, labels):
            first, second = pair
            self_pair = second in OBB_LINK_NAMES
            if value[0] == 1:
                distance = GeometryDistance("collision", 0., None, None, False)
            else:
                points = value[2:].reshape(2, 3)
                points.setflags(write=False)
                distance = GeometryDistance("separated", float(value[1]), points[0], points[1], True)
            gradient = None
            if distance.valid and spatial is not None:
                normal = (distance.second_point - distance.first_point) / distance.distance_m
                first_index = OBB_LINK_NAMES.index(first)
                gradient = -normal @ self._point_jacobian(spatial, distance.first_point, first_index)
                if self_pair:
                    gradient += normal @ self._point_jacobian(
                        spatial, distance.second_point, OBB_LINK_NAMES.index(second),
                    )
                    gradient[:first_index] = 0.
                if not np.all(np.isfinite(gradient)):
                    raise GeometryQueryError(f"{pair}: 关节梯度无效")
                gradient.setflags(write=False)
            measurements.append(CollisionMeasurement(
                pair, "self" if self_pair else "external",
                self.self_clearance_m if self_pair else self.external_clearance_m, distance, gradient,
            ))
        return tuple(measurements)
