from __future__ import annotations

import json
from dataclasses import dataclass
from hashlib import sha256
from io import BytesIO
from pathlib import Path
from urllib.parse import unquote, urlparse

import numpy as np
import trimesh
from rtree import __version__ as RTREE_VERSION
from scipy.spatial.transform import Rotation
from trimesh.ray.ray_triangle import RayMeshIntersector
from urdf_parser_py.urdf import URDF, Mesh

from work.actuator_limits import JOINT_NAMES
from work.kinematics_data import JOINT_CHAIN, JOINT_POSITION_LOWER, JOINT_POSITION_UPPER
from work.lidar_occupancy import LidarRayBatch, RayReturn, _readonly
from work.nineaxis_kinematics import NineaxisKinematics


@dataclass(frozen=True)
class GeometryValidation:
    geometry_hash: str
    surface_error_mm: float
    evidence_id: str
    model_complete: bool

    def __post_init__(self):
        if (len(self.geometry_hash) != 64 or not self.evidence_id.strip()
                or not np.isfinite(self.surface_error_mm) or self.surface_error_mm < 0
                or type(self.model_complete) is not bool):
            raise ValueError("geometry validation 的身份、误差或完整性无效")


@dataclass(frozen=True)
class CollisionMeshPart:
    link_name: str
    triangles_m: np.ndarray
    source_sha256: str
    source_path: Path | None


@dataclass(frozen=True)
class FirstRobotHits:
    measured: np.ndarray
    distance_m: np.ndarray
    link_name: tuple[str, ...]
    triangle_index: np.ndarray
    origin_inside: np.ndarray
    part_index: np.ndarray


class RobotRayGeometry:
    def __init__(self, parts, meshes, *, source_id, urdf_path=None, urdf_sha256="", validation=None):
        if not parts or not source_id.strip():
            raise ValueError("原始 collision mesh 与 source_id 必须完整")
        if trimesh.__version__ != "5.1.0" or RTREE_VERSION != "1.4.1":
            raise RuntimeError("mesh ray 后端需要 trimesh 5.1.0 与 rtree 1.4.1")
        self._kinematics = NineaxisKinematics()
        self._parts = tuple(parts)
        self._meshes = tuple(meshes)
        self._rays = tuple(RayMeshIntersector(mesh) for mesh in meshes)
        self._urdf_path = urdf_path
        self._urdf_sha256 = urdf_sha256
        self._source_id = source_id
        chain_path = Path(__file__).with_name("kinematics_data.py")
        fk_path = Path(__file__).with_name("nineaxis_kinematics.py")
        self._kinematic_sources = (chain_path, fk_path)
        self._kinematic_hashes = tuple(sha256(path.read_bytes()).hexdigest() for path in self._kinematic_sources)
        digest = sha256((source_id + self.urdf_sha256 + "".join(self._kinematic_hashes)
                         + trimesh.__version__ + RTREE_VERSION).encode())
        known = self._kinematics.link_poses(np.zeros(9))
        for part in self.parts:
            if part.link_name not in known:
                raise ValueError(f"collision link 没有共享 FK: {part.link_name}")
            digest.update(part.link_name.encode())
            digest.update(part.source_sha256.encode())
            digest.update(part.triangles_m.tobytes())
        self._geometry_hash = digest.hexdigest()
        if validation is not None and validation.geometry_hash != self.geometry_hash:
            raise ValueError("geometry validation 与原始 mesh 身份不一致")
        self._validation = validation
        self.verify_identity()

    @property
    def parts(self):
        return self._parts

    @property
    def geometry_hash(self):
        return self._geometry_hash

    @property
    def validation(self):
        return self._validation

    @property
    def urdf_sha256(self):
        return self._urdf_sha256

    @property
    def source_id(self):
        return self._source_id

    @classmethod
    def from_link_meshes(cls, meshes, *, source_id, validation=None):
        parts, snapshots = [], []
        for link_name, mesh in meshes:
            if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces):
                raise ValueError("collision mesh 必须包含真实三角面")
            snapshot = trimesh.Trimesh(vertices=np.array(mesh.vertices), faces=np.array(mesh.faces), process=False)
            triangles = _readonly(snapshot.triangles, np.float64, (len(snapshot.faces), 3, 3), "triangles_m")
            parts.append(CollisionMeshPart(link_name, triangles, sha256(triangles.tobytes()).hexdigest(), None))
            snapshots.append(snapshot)
        return cls(parts, snapshots, source_id=source_id, validation=validation)

    @classmethod
    def from_urdf(cls, path, *, package_roots, validation=None):
        path = Path(path).resolve(strict=True)
        urdf_bytes = path.read_bytes()
        robot = URDF.from_xml_string(urdf_bytes)
        joints_by_child = {joint.child: joint for joint in robot.joints}
        if robot.get_root() != "base_link" or len(robot.joints) != len(JOINT_CHAIN) - 1:
            raise ValueError("原始 URDF joint 数量与共享 FK 不一致")
        active_index = 0
        for parent, child, kind, *values in JOINT_CHAIN[1:]:
            # 共享 FK 的末端名称由现有生成器规定为 ee_link。
            source_child = "tool0" if child == "ee_link" else child
            joint = joints_by_child.get(source_child)
            if joint is None or joint.parent != parent or joint.type != kind:
                raise ValueError("原始 URDF 运动链与共享 FK 不一致")
            if kind in ("prismatic", "revolute"):
                if joint.name != JOINT_NAMES[active_index]:
                    raise ValueError("原始 URDF 关节名称与共享 FK 不一致")
                active_index += 1
            xyz = np.zeros(3) if joint.origin is None or joint.origin.xyz is None else joint.origin.xyz
            rpy = np.zeros(3) if joint.origin is None or joint.origin.rpy is None else joint.origin.rpy
            default_axis = [1., 0., 0.] if kind in ("prismatic", "revolute") else [0., 0., 1.]
            axis = default_axis if joint.axis is None else joint.axis
            if not np.allclose(np.r_[xyz, rpy, axis], np.r_[values[:6], values[6]], atol=1e-12, rtol=0):
                raise ValueError("原始 URDF joint origin/axis 与共享 FK 不一致")
        parts, meshes = [], []
        for link in robot.links:
            for collision in link.collisions:
                if not isinstance(collision.geometry, Mesh):
                    raise TypeError(f"collision {link.name} 必须使用原始 mesh")
                uri = urlparse(collision.geometry.filename)
                if uri.scheme == "package":
                    if uri.netloc not in package_roots:
                        raise ValueError(f"collision package 未登记: {uri.netloc}")
                    root = Path(package_roots[uri.netloc]).resolve(strict=True)
                    source = (root / unquote(uri.path).lstrip("/")).resolve(strict=True)
                    if not source.is_relative_to(root):
                        raise ValueError("collision URI 超出登记的 package")
                elif not uri.scheme:
                    source = (path.parent / unquote(uri.path)).resolve(strict=True)
                else:
                    raise ValueError(f"collision URI scheme 不支持: {uri.scheme}")
                mesh_bytes = source.read_bytes()
                mesh = trimesh.load_mesh(BytesIO(mesh_bytes), file_type=source.suffix.lstrip("."), process=False)
                if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces):
                    raise ValueError(f"collision 文件未提供三角面: {source}")
                scale = np.ones(3) if collision.geometry.scale is None else np.array(collision.geometry.scale)
                if scale.shape != (3,) or not np.all(np.isfinite(scale)) or np.any(scale <= 0):
                    raise ValueError("collision mesh scale 无效")
                transform = np.eye(4)
                if collision.origin is not None:
                    xyz = np.zeros(3) if collision.origin.xyz is None else np.array(collision.origin.xyz)
                    rpy = np.zeros(3) if collision.origin.rpy is None else np.array(collision.origin.rpy)
                    if not np.all(np.isfinite(np.r_[xyz, rpy])):
                        raise ValueError("collision origin 必须有限")
                    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
                    transform[:3, 3] = xyz
                mesh.vertices = (np.array(mesh.vertices) * scale) @ transform[:3, :3].T + transform[:3, 3]
                triangles = _readonly(mesh.triangles, np.float64, (len(mesh.faces), 3, 3), "triangles_m")
                link_name = "ee_link" if link.name == "tool0" else link.name
                parts.append(CollisionMeshPart(link_name, triangles, sha256(mesh_bytes).hexdigest(), source))
                meshes.append(mesh)
        return cls(parts, meshes, source_id="original_urdf_collision_m", urdf_path=path,
                   urdf_sha256=sha256(urdf_bytes).hexdigest(), validation=validation)

    @property
    def triangle_count(self):
        return sum(len(part.triangles_m) for part in self.parts)

    @property
    def admission_ready(self):
        return (self.validation is not None and self.validation.model_complete
                and all(mesh.is_watertight and mesh.is_volume for mesh in self._meshes))

    @property
    def validation_reason(self):
        if self.validation is None:
            return "geometry_evidence_missing"
        if not self.validation.model_complete:
            return "geometry_model_incomplete"
        return "ok" if self.admission_ready else "geometry_surface_not_closed"

    def with_validation(self, validation):
        return type(self)(self.parts, self._meshes, source_id=self.source_id,
                          urdf_path=self._urdf_path, urdf_sha256=self.urdf_sha256, validation=validation)

    def verify_identity(self):
        if self._urdf_path is not None and sha256(self._urdf_path.read_bytes()).hexdigest() != self.urdf_sha256:
            raise ValueError("原始 URDF 内容改变")
        for path, digest in zip(self._kinematic_sources, self._kinematic_hashes):
            if sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError("共享 FK 来源改变")
        for part, mesh in zip(self.parts, self._meshes):
            if not np.array_equal(mesh.triangles, part.triangles_m):
                raise ValueError("collision mesh 内容改变")
            if part.source_path is not None and sha256(part.source_path.read_bytes()).hexdigest() != part.source_sha256:
                raise ValueError("原始 collision 文件内容改变")

    def _poses_for_points(self, q_points):
        unique, inverse = np.unique(q_points, axis=0, return_inverse=True)
        poses = [self._kinematics.link_poses(q) for q in unique]
        return {name: np.array([pose[name] for pose in poses])[inverse]
                for name in {part.link_name for part in self.parts}}

    def _local_rays(self, origins, directions, poses):
        rotation = poses[:, :3, :3]
        return (np.einsum("nji,nj->ni", rotation, origins - poses[:, :3, 3]),
                np.einsum("nji,nj->ni", rotation, directions))

    def first_hits(self, origins_m, directions, q_points):
        self.verify_identity()
        count = len(origins_m)
        origins = _readonly(origins_m, np.float64, (count, 3), "origins_m")
        vectors = _readonly(directions, np.float64, (count, 3), "directions")
        q_points = _readonly(q_points, np.float64, (count, 9), "q_points")
        norms = np.linalg.norm(vectors, axis=1)
        if np.any(norms == 0):
            raise ValueError("射线方向不能为零")
        vectors = vectors / norms[:, None]
        distances = np.full(count, np.inf)
        links, triangles = [""] * count, np.full(count, -1, dtype=np.int64)
        part_indices = np.full(count, -1, dtype=np.int64)
        inside = np.zeros(count, dtype=bool)
        poses = self._poses_for_points(q_points)
        for part_index, (part, mesh, ray) in enumerate(zip(self.parts, self._meshes, self._rays)):
            local_origins, local_vectors = self._local_rays(origins, vectors, poses[part.link_name])
            if mesh.is_watertight:
                candidates = np.all(local_origins >= mesh.bounds[0], axis=1) & np.all(local_origins <= mesh.bounds[1], axis=1)
                if candidates.any():
                    inside[candidates] |= ray.contains_points(local_origins[candidates])
            points, rays, faces = ray.intersects_location(local_origins, local_vectors, multiple_hits=False)
            if not len(rays):
                continue
            lengths = np.linalg.norm(points - local_origins[rays], axis=1)
            for index, face, length in zip(rays, faces, lengths):
                if length < distances[index]:
                    distances[index], triangles[index], links[index] = length, face, part.link_name
                    part_indices[index] = part_index
        measured = np.isfinite(distances)
        distances[~measured] = 0.
        return FirstRobotHits(measured, distances, tuple(links), triangles, inside, part_indices)

    def conservative_intervals(self, origins, directions, q_points, errors_m, max_range_m, hits):
        count = len(origins)
        lower, upper = np.full(count, max_range_m), np.full(count, max_range_m)
        certified = np.zeros(count, dtype=bool)
        exact = (errors_m == 0) & hits.measured
        lower[exact] = upper[exact] = hits.distance_m[exact]
        certified[exact] = True
        uncertain_indices = np.flatnonzero(errors_m > 0)
        if not len(uncertain_indices):
            return lower, upper, certified
        error = float(errors_m[uncertain_indices].max())
        poses = self._poses_for_points(q_points)
        for part, mesh, ray in zip(self.parts, self._meshes, self._rays):
            local_origins, local_directions = self._local_rays(
                origins[uncertain_indices], directions[uncertain_indices], poses[part.link_name][uncertain_indices],
            )
            bounds = mesh.bounds + np.array([[-error] * 3, [error] * 3])
            box = trimesh.creation.box(extents=bounds[1] - bounds[0])
            box.apply_translation(bounds.mean(axis=0))
            contained = np.all(local_origins >= bounds[0], axis=1) & np.all(local_origins <= bounds[1], axis=1)
            lower[uncertain_indices[contained]] = 0.
            positions, rays, _ = RayMeshIntersector(box).intersects_location(local_origins, local_directions, multiple_hits=False)
            if len(rays):
                distances = np.linalg.norm(positions - local_origins[rays], axis=1)
                np.minimum.at(lower, uncertain_indices[rays], distances)
            if not mesh.is_watertight:
                continue
            positions, rays, _ = ray.intersects_location(local_origins, local_directions, multiple_hits=True)
            if len(rays) < 2:
                continue
            distances = np.linalg.norm(positions - local_origins[rays], axis=1)
            candidate_rays, candidate_distances = [], []
            for ray_index in np.unique(rays):
                values = np.sort(distances[rays == ray_index])
                midpoints = (values[:-1] + values[1:]) / 2
                midpoints = midpoints[midpoints <= max_range_m]
                candidate_rays.extend([ray_index] * len(midpoints))
                candidate_distances.extend(midpoints)
            if not candidate_rays:
                continue
            ray_indices, distances = np.array(candidate_rays, dtype=int), np.array(candidate_distances)
            witnesses = local_origins[ray_indices] + distances[:, None] * local_directions[ray_indices]
            interior = ray.contains_points(witnesses)
            if not interior.any():
                continue
            _, separation, _ = trimesh.proximity.closest_point(mesh, witnesses[interior])
            selected = separation > errors_m[uncertain_indices[ray_indices[interior]]]
            # 内部点与全部表面的间距超过完整位移界，证明射线必经机器人实体。
            selected_rays = uncertain_indices[ray_indices[interior][selected]]
            np.minimum.at(upper, selected_rays, distances[interior][selected])
            certified[selected_rays] = True
        uncertain_lower = (errors_m > 0) & (lower > 0) & (lower < max_range_m)
        lower[uncertain_lower] = np.nextafter(lower[uncertain_lower], -np.inf)
        upper[certified & (errors_m > 0)] = np.nextafter(upper[certified & (errors_m > 0)], np.inf)
        return lower, upper, certified

    @property
    def joint_lever_bound_m(self):
        chain_length = sum(np.linalg.norm(item[3:6]) for item in JOINT_CHAIN)
        mesh_radius = max(np.linalg.norm(part.triangles_m.reshape(-1, 3), axis=1).max() for part in self.parts)
        return chain_length + mesh_radius + max(abs(JOINT_POSITION_LOWER[0]), abs(JOINT_POSITION_UPPER[0]))


@dataclass(frozen=True)
class ClockMapping:
    source_clock_id: str
    mapping_id: str
    source_anchor_ns: int
    monotonic_anchor_ns: int
    valid_source_start_ns: int
    valid_source_end_ns: int
    offset_error_ns: int
    rate_error_ppb: float
    evidence_id: str
    run_id: str
    target_clock_id: str

    def __post_init__(self):
        if (not self.source_clock_id.strip() or not self.mapping_id.strip() or not self.evidence_id.strip()
                or not self.run_id.strip() or not self.target_clock_id.strip()
                or any(type(value) is not int for value in (self.source_anchor_ns, self.monotonic_anchor_ns,
                           self.valid_source_start_ns, self.valid_source_end_ns, self.offset_error_ns))
                or self.monotonic_anchor_ns <= 0 or self.offset_error_ns < 0
                or self.valid_source_start_ns > self.valid_source_end_ns
                or not self.valid_source_start_ns <= self.source_anchor_ns <= self.valid_source_end_ns
                or not np.isfinite(self.rate_error_ppb) or self.rate_error_ppb < 0):
            raise ValueError("ClockMapping 的身份、锚点或误差无效")

    def map(self, raw_stamp_ns):
        original = np.asarray(raw_stamp_ns)
        if original.ndim != 1 or original.dtype.kind not in "iuf" or not np.isfinite(original).all():
            raise ValueError("逐点原始时间必须是有限数字数组")
        values = original.astype(np.longdouble)
        if (np.any(values < self.valid_source_start_ns) or np.any(values > self.valid_source_end_ns)
                or np.any(np.diff(values) < 0)):
            raise ValueError("逐点时间超出映射有效范围或发生回跳")
        delta = values - self.source_anchor_ns
        mapped = np.rint(delta + self.monotonic_anchor_ns)
        if np.any(mapped <= 0) or np.any(mapped > np.iinfo(np.int64).max):
            raise ValueError("monotonic 时间超出 int64 有效范围")
        quantization = np.zeros(len(values), dtype=np.longdouble)
        if original.dtype.kind == "f":
            quantization = np.abs(np.spacing(original)).astype(np.longdouble) / 2
        error = np.ceil(self.offset_error_ns + abs(delta) * self.rate_error_ppb * 1e-9 + quantization
                        + abs(mapped - delta - self.monotonic_anchor_ns))
        if np.any(error > np.iinfo(np.int64).max):
            raise ValueError("时间误差超出 int64 有效范围")
        return mapped.astype(np.int64), error.astype(np.int64)

    @property
    def identity(self):
        return sha256(json.dumps(vars(self), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


@dataclass(frozen=True)
class TimedLidarScan:
    points_lidar_m: np.ndarray
    point_stamp_ns: np.ndarray
    point_time_error_ns: np.ndarray
    original_stamp_ns: np.ndarray
    row_indices: np.ndarray
    scan_id: int
    source_clock_id: str
    clock_mapping_id: str
    frame_id: str
    received_monotonic_ns: int
    max_range_m: float
    run_id: str
    target_clock_id: str

    def __post_init__(self):
        count = len(self.points_lidar_m)
        if count == 0:
            raise ValueError("扫描必须保留至少一个真实点")
        for name, dtype, shape in (("points_lidar_m", float, (count, 3)), ("point_stamp_ns", np.int64, (count,)),
                                   ("point_time_error_ns", np.int64, (count,)), ("row_indices", np.int64, (count,))):
            object.__setattr__(self, name, _readonly(getattr(self, name), dtype, shape, name))
        original = np.array(self.original_stamp_ns, copy=True)
        if original.shape != (count,) or original.dtype.kind not in "iuf" or not np.isfinite(original).all():
            raise ValueError("original_stamp_ns 必须保留每个点的原始时间")
        original.setflags(write=False)
        object.__setattr__(self, "original_stamp_ns", original)
        if (type(self.scan_id) is not int or self.scan_id < 0 or type(self.received_monotonic_ns) is not int
                or self.received_monotonic_ns <= 0 or np.any(self.point_stamp_ns <= 0)
                or np.any(self.point_time_error_ns < 0) or np.any(np.diff(self.point_stamp_ns) < 0)
                or len(np.unique(self.row_indices)) != count or np.any(self.row_indices < 0)
                or not self.source_clock_id.strip() or not self.clock_mapping_id.strip() or not self.frame_id.strip()
                or not self.run_id.strip() or not self.target_clock_id.strip()
                or not np.isfinite(self.max_range_m) or self.max_range_m <= 0
                or np.any(np.linalg.norm(self.points_lidar_m, axis=1) == 0)
                or np.any(np.linalg.norm(self.points_lidar_m, axis=1) > self.max_range_m)):
            raise ValueError("TimedLidarScan 的来源、时间、距离或原始索引无效")

    @property
    def identity(self):
        header = [self.source_clock_id, self.clock_mapping_id, self.run_id, self.target_clock_id,
                  self.frame_id, self.scan_id, self.max_range_m, self.received_monotonic_ns]
        digest = sha256(json.dumps(header, separators=(",", ":")).encode())
        for value in (self.points_lidar_m, self.original_stamp_ns, self.point_stamp_ns,
                      self.point_time_error_ns, self.row_indices):
            digest.update(value.dtype.str.encode())
            digest.update(value.tobytes())
        return digest.hexdigest()


@dataclass(frozen=True)
class JointStateHistory:
    stamp_ns: np.ndarray
    q_si: np.ndarray
    timestamp_error_ns: np.ndarray
    feedback_source: str
    model_hash: str
    zero_calibration_id: str
    clock_mapping_id: str
    run_id: str
    target_clock_id: str

    def __post_init__(self):
        count = len(self.stamp_ns)
        for name, dtype, shape in (("stamp_ns", np.int64, (count,)), ("q_si", float, (count, 9)),
                                   ("timestamp_error_ns", np.int64, (count,))):
            object.__setattr__(self, name, _readonly(getattr(self, name), dtype, shape, name))
        if (count < 2 or np.any(self.stamp_ns <= 0) or np.any(np.diff(self.stamp_ns) <= 0)
                or np.any(self.timestamp_error_ns < 0)
                or self.feedback_source not in ("simulation", "encoder") or len(self.model_hash) != 64
                or not self.zero_calibration_id.strip() or not self.clock_mapping_id.strip()
                or not self.run_id.strip() or not self.target_clock_id.strip()
                or np.any(self.q_si < np.array(JOINT_POSITION_LOWER))
                or np.any(self.q_si > np.array(JOINT_POSITION_UPPER))):
            raise ValueError("反馈历史的来源、时间、关节坐标或身份无效")

    @property
    def identity(self):
        header = [self.model_hash, self.zero_calibration_id, self.clock_mapping_id,
                  self.run_id, self.target_clock_id, self.feedback_source]
        digest = sha256(json.dumps(header, separators=(",", ":")).encode())
        for value in (self.stamp_ns, self.q_si, self.timestamp_error_ns):
            digest.update(value.tobytes())
        return digest.hexdigest()

    def interpolate(self, stamps, errors, *, maximum_gap_ns, speed_bound):
        lower, upper = stamps.astype(object) - errors, stamps.astype(object) + errors
        history_start = int(self.stamp_ns[0]) + int(self.timestamp_error_ns[0])
        history_end = int(self.stamp_ns[-1]) - int(self.timestamp_error_ns[-1])
        if np.any(lower < history_start) or np.any(upper > history_end):
            return None, "history_missing"
        first = max(0, int(np.searchsorted(self.stamp_ns, lower.min(), side="right")) - 1)
        last = min(len(self.stamp_ns) - 1, int(np.searchsorted(self.stamp_ns, upper.max(), side="left")))
        gaps = np.diff(self.stamp_ns[first:last + 1])
        if np.any(gaps > maximum_gap_ns):
            return None, "history_gap"
        actual_speed = abs(np.diff(self.q_si[first:last + 1], axis=0)) / gaps[:, None] * 1e9
        if np.any(actual_speed > speed_bound + np.finfo(float).eps):
            return None, "feedback_speed_bound_exceeded"
        left = np.clip(np.searchsorted(self.stamp_ns, stamps, side="right") - 1, 0, len(self.stamp_ns) - 2)
        right = left + 1
        fraction = (stamps - self.stamp_ns[left]).astype(np.longdouble) / (self.stamp_ns[right] - self.stamp_ns[left])
        q = np.asarray(self.q_si[left].astype(np.longdouble)
                       + fraction[:, None] * (self.q_si[right].astype(np.longdouble) - self.q_si[left]), dtype=float)
        error_ns = errors.astype(object) + int(self.timestamp_error_ns[first:last + 1].max())
        if np.any(error_ns > np.iinfo(np.int64).max):
            return None, "history_timestamp_error_exceeded"
        error_ns = error_ns.astype(np.int64)
        return (q, error_ns), "ok"


@dataclass(frozen=True)
class FilterCalibration:
    T_base_lidar: np.ndarray
    frame_from: str
    frame_to: str
    calibration_id: str
    admission_ready: bool

    def __post_init__(self):
        matrix = _readonly(self.T_base_lidar, float, (4, 4), "T_base_lidar")
        if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-10, rtol=0)
                or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-10, rtol=0)
                or self.frame_to != "base_link" or not self.frame_from.strip() or not self.calibration_id.strip()
                or type(self.admission_ready) is not bool):
            raise ValueError("LiDAR 标定变换、frame 或身份无效")
        object.__setattr__(self, "T_base_lidar", matrix)


@dataclass(frozen=True)
class SelfFilterBounds:
    range_error_mm: float
    incidence_error_mm: float
    angular_error_rad: float
    translation_error_mm: float
    rotation_error_rad: float
    interpolation_error_si: np.ndarray
    joint_speed_bound_si: np.ndarray
    max_history_gap_ns: int
    timestamp_error_bound_ns: int
    source_id: str

    def __post_init__(self):
        for name in ("range_error_mm", "incidence_error_mm", "angular_error_rad", "translation_error_mm", "rotation_error_rad"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} 必须有来源且有限非负")
        for name in ("interpolation_error_si", "joint_speed_bound_si"):
            array = _readonly(getattr(self, name), float, (9,), name)
            if np.any(array < 0):
                raise ValueError(f"{name} 必须非负")
            object.__setattr__(self, name, array)
        if (type(self.max_history_gap_ns) is not int or self.max_history_gap_ns <= 0
                or type(self.timestamp_error_bound_ns) is not int or self.timestamp_error_bound_ns < 0
                or not self.source_id.strip()):
            raise ValueError("self-filter 插值范围、时间误差或来源无效")

    @classmethod
    def from_collision_policy(cls, policy, *, joint_speed_bound_si, maximum_gap_ns, history_evidence_id):
        if not history_evidence_id.strip():
            raise ValueError("关节插值和速度界需要独立来源")
        value = lambda name: policy.artifact.parameters[name]["value"]
        return cls(value("lidar_range_error_mm"), value("lidar_incidence_error_mm"), value("lidar_angular_error_rad"),
                   value("extrinsic_translation_error_mm"), value("extrinsic_rotation_error_rad"),
                   np.r_[value("joint_interpolation_prismatic_error_mm") * .001,
                         np.full(8, value("joint_interpolation_revolute_error_rad"))],
                   joint_speed_bound_si, maximum_gap_ns, value("point_timestamp_error_ns"),
                   policy.collision_policy_hash.hex() + ":" + history_evidence_id)

    @property
    def identity(self):
        fields = {name: value.tolist() if isinstance(value, np.ndarray) else value for name, value in vars(self).items()}
        return sha256(json.dumps(fields, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


@dataclass(frozen=True)
class SelfFilterResult:
    scan: TimedLidarScan
    origins_m: np.ndarray
    endpoints_m: np.ndarray
    classification: tuple[str, ...]
    first_hit_measured: np.ndarray
    first_hit_distance_m: np.ndarray
    first_hit_link: tuple[str, ...]
    first_hit_triangle: np.ndarray
    first_hit_lower_m: np.ndarray
    first_hit_upper_m: np.ndarray
    first_hit_upper_valid: np.ndarray
    visible_free_until_m: np.ndarray
    unknown_start_m: np.ndarray
    unknown_end_m: np.ndarray
    unknown_radius_mm: np.ndarray
    input_valid: bool
    reason: str
    filter_identity: str
    error_sources: tuple[str, ...]
    first_hit_part_index: np.ndarray

    def __post_init__(self):
        count = len(self.scan.points_lidar_m)
        for name, dtype, shape in (
            ("origins_m", float, (count, 3)), ("endpoints_m", float, (count, 3)),
            ("first_hit_measured", bool, (count,)), ("first_hit_distance_m", float, (count,)),
            ("first_hit_triangle", np.int64, (count,)), ("first_hit_lower_m", float, (count,)),
            ("first_hit_part_index", np.int64, (count,)),
            ("first_hit_upper_m", float, (count,)), ("first_hit_upper_valid", bool, (count,)),
            ("visible_free_until_m", float, (count,)), ("unknown_start_m", float, (count,)),
            ("unknown_end_m", float, (count,)), ("unknown_radius_mm", float, (count,)),
        ):
            object.__setattr__(self, name, _readonly(getattr(self, name), dtype, shape, name))

    @property
    def row_indices(self):
        return self.scan.row_indices

    def to_ray_batch(self):
        kinds = np.array([RayReturn.HIT if kind == "environment" else
                          RayReturn.AMBIGUOUS if kind == "ambiguous" else RayReturn.OCCLUDED
                          for kind in self.classification], dtype=np.int32)
        return LidarRayBatch(
            self.origins_m, self.endpoints_m, self.scan.point_stamp_ns, kinds, self.scan.scan_id,
            visible_free_until_m=self.visible_free_until_m, unknown_start_m=self.unknown_start_m,
            unknown_end_m=self.unknown_end_m, unknown_radius_mm=self.unknown_radius_mm,
            input_valid=self.input_valid, filter_identity=self.filter_identity, invalid_reason=self.reason,
        )


class MeshRaySelfFilter:
    def __init__(self, geometry: RobotRayGeometry, calibration: FilterCalibration, bounds: SelfFilterBounds):
        if (not isinstance(geometry, RobotRayGeometry) or not isinstance(calibration, FilterCalibration)
                or not isinstance(bounds, SelfFilterBounds)):
            raise TypeError("MeshRaySelfFilter 需要完整几何、标定与误差对象")
        self.geometry, self.calibration, self.bounds = geometry, calibration, bounds

    def classify(self, scan: TimedLidarScan, history: JointStateHistory):
        if not isinstance(scan, TimedLidarScan) or not isinstance(history, JointStateHistory):
            raise TypeError("classify 需要 TimedLidarScan 与 JointStateHistory")
        self.geometry.verify_identity()
        count = len(scan.points_lidar_m)
        transform = self.calibration.T_base_lidar
        origins = np.broadcast_to(transform[:3, 3], (count, 3)).copy()
        endpoints = scan.points_lidar_m @ transform[:3, :3].T + transform[:3, 3]
        vectors = endpoints - origins
        distance = np.linalg.norm(vectors, axis=1)
        directions = vectors / distance[:, None]
        validation_identity = "" if self.geometry.validation is None else sha256(
            json.dumps(vars(self.geometry.validation), sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        stable_identity = (self.geometry.geometry_hash, validation_identity, self.calibration.calibration_id,
                           sha256(transform.tobytes()).hexdigest(), self.bounds.identity,
                           scan.clock_mapping_id, history.model_hash, history.zero_calibration_id,
                           history.clock_mapping_id, history.feedback_source, scan.run_id, scan.target_clock_id)
        identity = sha256(json.dumps(stable_identity, separators=(",", ":")).encode()).hexdigest()
        sources = (self.geometry.geometry_hash, self.calibration.calibration_id, self.bounds.source_id,
                   scan.clock_mapping_id, history.clock_mapping_id, history.zero_calibration_id,
                   scan.identity, history.identity)
        reason = "ok"
        if not self.geometry.admission_ready:
            reason = self.geometry.validation_reason
        elif not self.calibration.admission_ready:
            reason = "calibration_not_admitted"
        elif scan.frame_id != self.calibration.frame_from:
            reason = "lidar_frame_mismatch"
        elif history.model_hash != self.geometry.geometry_hash:
            reason = "model_identity_mismatch"
        elif history.run_id != scan.run_id or history.target_clock_id != scan.target_clock_id:
            reason = "clock_run_identity_mismatch"
        elif np.any(scan.point_stamp_ns.astype(object) + scan.point_time_error_ns > scan.received_monotonic_ns):
            reason = "future_point"
        elif np.any(scan.point_time_error_ns > self.bounds.timestamp_error_bound_ns):
            reason = "point_timestamp_error_exceeded"
        interpolated = None
        if reason == "ok":
            interpolated, reason = history.interpolate(
                scan.point_stamp_ns, scan.point_time_error_ns,
                maximum_gap_ns=self.bounds.max_history_gap_ns, speed_bound=self.bounds.joint_speed_bound_si,
            )
        if reason != "ok":
            zeros = np.zeros(count)
            return SelfFilterResult(
                scan, origins, endpoints, ("ambiguous",) * count, np.zeros(count, dtype=bool), zeros,
                ("",) * count, np.full(count, -1, dtype=np.int64), zeros, zeros, np.zeros(count, dtype=bool),
                zeros, zeros, np.full(count, scan.max_range_m), zeros, False, reason, identity, sources,
                np.full(count, -1, dtype=np.int64),
            )
        if interpolated is None:
            raise RuntimeError("反馈插值没有生成有效坐标")
        q, time_errors = interpolated
        q_error = self.bounds.interpolation_error_si[None, :] + time_errors[:, None] * 1e-9 * self.bounds.joint_speed_bound_si
        surface_error = self.geometry.validation.surface_error_mm * .001
        geometric_error = (surface_error + self.bounds.translation_error_mm * .001
                           + scan.max_range_m * (self.bounds.rotation_error_rad + self.bounds.angular_error_rad)
                           + q_error[:, 0] + self.geometry.joint_lever_bound_m * q_error[:, 1:].sum(axis=1))
        measurement_error = (self.bounds.range_error_mm + self.bounds.incidence_error_mm) * .001
        geometric_error[geometric_error > 0] = np.nextafter(geometric_error[geometric_error > 0], np.inf)
        if measurement_error > 0:
            measurement_error = np.nextafter(measurement_error, np.inf)
        hits = self.geometry.first_hits(origins, directions, q)
        lower, upper, upper_valid = self.geometry.conservative_intervals(
            origins, directions, q, geometric_error, scan.max_range_m, hits,
        )
        kinds, visible, unknown_start, unknown_end = [], np.zeros(count), np.zeros(count), np.full(count, scan.max_range_m)
        for index in range(count):
            observed = distance[index]
            observed_lower = np.nextafter(observed - measurement_error, -np.inf) if measurement_error > 0 else observed
            if hits.origin_inside[index] or lower[index] <= np.finfo(float).eps:
                kinds.append("ambiguous")
                continue
            if observed + measurement_error < lower[index] or (not hits.measured[index] and lower[index] == scan.max_range_m):
                kinds.append("environment")
                visible[index] = max(0., min(observed_lower, lower[index]))
                if lower[index] < scan.max_range_m:
                    unknown_start[index] = lower[index]
                else:
                    unknown_start[index] = unknown_end[index] = 0.
            elif upper_valid[index] and observed_lower > upper[index]:
                kinds.append("occluded")
                visible[index] = unknown_start[index] = max(0., lower[index])
            elif (geometric_error[index] == 0 and measurement_error == 0 and hits.measured[index]
                  and abs(observed - hits.distance_m[index]) <= 8 * np.finfo(float).eps * max(1., observed)):
                kinds.append("self")
                visible[index] = unknown_start[index] = hits.distance_m[index]
            else:
                kinds.append("ambiguous")
                visible[index] = unknown_start[index] = max(0., min(observed_lower, lower[index]))
        return SelfFilterResult(
            scan, origins, endpoints, tuple(kinds), hits.measured, hits.distance_m, hits.link_name,
            hits.triangle_index, lower, upper, upper_valid, visible, unknown_start, unknown_end,
            (geometric_error + measurement_error) * 1000, True, "ok", identity, sources, hits.part_index,
        )
