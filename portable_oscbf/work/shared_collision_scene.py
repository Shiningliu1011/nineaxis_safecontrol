from __future__ import annotations

import ctypes
import fcntl
import hashlib
import json
import mmap
import os
import select
import stat
import threading
import time
import uuid
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from work._collision_scene import (
    CollisionIdentities, CollisionScene, CollisionStatus, PreparedScene, SceneMetadata, SceneTracks,
    _admission_status, _copy_arrays, _identity_specs, _snapshot, _support_specs, _track_specs,
)
from work.dynamic_support import SupportTimeModel, run_clock_hash, time_model_specs
from work.fixed_environment import FixedEnvironmentModel


TRANSPORT_VERSION = 1
MAGIC = b"collision_scene1"
EVIDENCE_CAPACITY = 4096
HEADER_BYTES = 256
HEADER_DTYPE = np.dtype([
    ("magic", "S16"), ("version", "<i8"), ("header_bytes", "<i8"), ("slot_bytes", "<i8"),
    ("layout_hash", "u1", (32,)), ("resource_hash", "u1", (32,)), ("source_identity", "u1", (32,)),
    ("epoch", "u1", (16,)), ("writer_pid", "<i8"),
    ("created_ns", "<i8"), ("support_capacity", "<i8"), ("track_capacity", "<i8"),
    ("member_capacity", "<i8"), ("checksum", "u1", (32,)),
], align=True)
METADATA_DTYPE = np.dtype([
    ("layout_version", "<i8"), ("frame_id", "S32"), ("point_unit", "S32"), ("radius_unit", "S32"),
    ("velocity_unit", "S32"), ("time_unit", "S32"), ("clock_id", "S32"),
    ("support_count", "<i8"), ("track_count", "<i8"), ("required_space_covered", "?"),
    ("checksum", "u1", (32,)),
], align=True)


def _array_dtype(specs):
    return np.dtype([(name, "<f8" if dtype == "float" else np.dtype(dtype), shape)
                     for name, (shape, dtype) in specs.items()], align=True)


def _digest(record):
    return hashlib.sha256(record.tobytes()[:record.dtype.fields["checksum"][1]]).digest()


def _array_record(value, dtype):
    result = np.zeros((), dtype=dtype)
    for name in dtype.names:
        result[name] = getattr(value, name)
    return result


def _arrays(record, cls):
    return cls(*(np.array(record[name], copy=True) for name in cls._fields))


def _resource_hash(path):
    return hashlib.sha256(str(path.parent.resolve(strict=True) / path.name).encode("utf-8")).digest()


def _source_identity(value):
    if value is None:
        return bytes(32)
    if not isinstance(value, bytes) or len(value) != 32 or not any(value):
        raise ValueError("来源身份必须为 32-byte 非零 SHA-256")
    return value


def _checked_header(payload, layout, resource_hash):
    header = np.frombuffer(payload, dtype=HEADER_DTYPE, count=1)[0]
    if (bytes(header["magic"]) != MAGIC or int(header["version"]) != TRANSPORT_VERSION
            or int(header["header_bytes"]) != HEADER_BYTES or int(header["slot_bytes"]) != layout.slot_dtype.itemsize
            or bytes(header["layout_hash"]) != layout.layout_hash or bytes(header["resource_hash"]) != resource_hash
            or any(int(header[name]) != value for name, value in asdict(layout).items())
            or bytes(header["checksum"]) != _digest(header)
            or int(header["created_ns"]) <= 0 or int(header["created_ns"]) > time.monotonic_ns()
            or int(header["writer_pid"]) <= 0 or not any(header["epoch"])):
        raise ValueError("共享场景 header、checksum、资源身份或 layout 无效")
    return header


@dataclass(frozen=True)
class SharedSceneLayout:
    support_capacity: int
    track_capacity: int
    member_capacity: int

    def __post_init__(self):
        if any(type(value) is not int or value < 1 for value in asdict(self).values()):
            raise ValueError("共享场景容量必须为正整数")

    @classmethod
    def from_config(cls, config):
        parameters = config.policy.artifact.parameters
        return cls(config.max_support_points, parameters["max_tracks"]["value"],
                   parameters["support_merge_max_points"]["value"])

    @property
    def fixed_capacity(self):
        return self.support_capacity * self.member_capacity

    @property
    def slot_dtype(self):
        fixed = np.dtype([
            ("model_revision", "<i8"), ("source_hash", "u1", (32,)),
            ("validation_evidence", f"S{EVIDENCE_CAPACITY}"), ("policy_hash", "u1", (32,)),
            ("support_points_m", "<f8", (self.fixed_capacity, 3)),
            ("support_radii_mm", "<f8", (self.fixed_capacity,)), ("support_count", "<i8"),
            ("support_demand", "<i8"), ("geometry_hash", "u1", (32,)), ("model_hash", "u1", (32,)),
        ], align=True)
        return np.dtype([
            ("publication", "<u8"), ("supports", _array_dtype(_support_specs(self.support_capacity))),
            ("identities", _array_dtype(_identity_specs())),
            ("tracks", _array_dtype(_track_specs(self.track_capacity))),
            ("coverage_space_hash", "u1", (32,)), ("coverage_valid_until_ns", "<i8"),
            ("metadata", METADATA_DTYPE), ("has_time_model", "?"),
            ("time_model", _array_dtype(time_model_specs(self.support_capacity, self.member_capacity))),
            ("has_fixed_model", "?"), ("fixed_model", fixed), ("checksum", "u1", (32,)),
        ], align=True)

    @property
    def layout_hash(self):
        return hashlib.sha256(json.dumps({
            "version": TRANSPORT_VERSION, "header_bytes": HEADER_BYTES,
            "header": HEADER_DTYPE.descr, "slot": self.slot_dtype.descr,
        }, sort_keys=True, separators=(",", ":")).encode()).digest()

    @property
    def size_bytes(self):
        return HEADER_BYTES + 2 * self.slot_dtype.itemsize

    def slot_offset(self, index):
        if index not in (0, 1):
            raise ValueError("active slot 必须为 0 或 1")
        return HEADER_BYTES + index * self.slot_dtype.itemsize


class _AtomicPublication:
    def __init__(self, mapping):
        # ACCESS_READ 的 NumPy view 仅用于取得地址；原子 load 不需要可写 buffer。
        self.address = np.frombuffer(mapping, dtype=np.uint64, count=1).__array_interface__["data"][0]
        self.library = ctypes.CDLL("libatomic.so.1")
        self.load_function = getattr(self.library, "__atomic_load_8")
        self.load_function.argtypes = (ctypes.c_void_p, ctypes.c_int)
        self.load_function.restype = ctypes.c_uint64
        self.store_function = getattr(self.library, "__atomic_store_8")
        self.store_function.argtypes = (ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int)
        self.store_function.restype = None
        lock_free = getattr(self.library, "__atomic_is_lock_free")
        lock_free.argtypes = (ctypes.c_size_t, ctypes.c_void_p)
        lock_free.restype = ctypes.c_bool
        if self.address % 8 or not lock_free(8, self.address):
            raise RuntimeError("共享发布字要求 8-byte 对齐及 lock-free libatomic")

    def load(self):
        return int(self.load_function(self.address, 2))

    def store(self, value):
        self.store_function(self.address, value, 3)


@dataclass(frozen=True)
class SceneRevision:
    epoch: bytes
    revision: int
    active_slot: int
    layout_hash: bytes
    source_identity: bytes
    source_stamp_ns: int
    prepared_stamp_ns: int
    status: CollisionStatus

    def to_dict(self):
        return {**asdict(self), "epoch": self.epoch.hex(), "layout_hash": self.layout_hash.hex(),
                "source_identity": self.source_identity.hex(),
                "status": int(self.status)}

    @classmethod
    def from_dict(cls, document):
        if set(document) != set(cls.__dataclass_fields__):
            raise ValueError("scene revision 消息字段无效")
        result = cls(**{**document, "epoch": bytes.fromhex(document["epoch"]),
                        "layout_hash": bytes.fromhex(document["layout_hash"]),
                        "source_identity": bytes.fromhex(document["source_identity"]),
                        "status": CollisionStatus(document["status"])})
        if (len(result.epoch) != 16 or not any(result.epoch) or len(result.layout_hash) != 32
                or len(result.source_identity) != 32
                or any(type(value) is not int or value < 0 for value in (
                    result.revision, result.source_stamp_ns, result.prepared_stamp_ns))
                or type(result.active_slot) is not int or result.active_slot not in (0, 1)):
            raise ValueError("scene revision 消息身份、时间或 slot 无效")
        return result


@dataclass(frozen=True)
class SceneReadResult:
    status: CollisionStatus
    reason: str
    revision: SceneRevision | None = None
    prepared_scene: PreparedScene | None = None


class ScenePublicationError(ValueError):
    pass


class SharedSceneWriter:
    def __init__(self, path, safety, *, source_identity=None):
        self.path = Path(path)
        self.safety = safety
        self.layout = SharedSceneLayout.from_config(safety.config)
        self.source_identity = _source_identity(source_identity)
        self.epoch = uuid.uuid4().bytes
        self.owner_pid = os.getpid()
        self.owner_thread = threading.get_ident()
        self._revision = 0
        self._slot = 0
        self._closed = False
        self._notice = SceneRevision(self.epoch, 0, 0, self.layout.layout_hash, self.source_identity, 0, 0,
                                     CollisionStatus.REVISION_PENDING)
        self._lock_fd = os.open(str(self.path) + ".writer.lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(self._lock_fd)
            raise RuntimeError(f"共享场景已有 writer: {self.path}") from None
        staging = self.path.with_name(self.path.name + "." + self.epoch.hex())
        staging_created = False
        try:
            previous = self._validate_target()
            self._fd = os.open(staging, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            staging_created = True
            os.ftruncate(self._fd, self.layout.size_bytes)
            self._mapping = mmap.mmap(self._fd, self.layout.size_bytes, access=mmap.ACCESS_WRITE)
            self._atomic = _AtomicPublication(self._mapping)
            self.created_ns = time.monotonic_ns()
            header = np.zeros((), dtype=HEADER_DTYPE)
            for name, value in {
                "magic": MAGIC, "version": TRANSPORT_VERSION, "header_bytes": HEADER_BYTES,
                "slot_bytes": self.layout.slot_dtype.itemsize, "layout_hash": np.frombuffer(self.layout.layout_hash, "u1"),
                "resource_hash": np.frombuffer(_resource_hash(self.path), "u1"),
                "source_identity": np.frombuffer(self.source_identity, "u1"),
                "epoch": np.frombuffer(self.epoch, "u1"), "writer_pid": self.owner_pid,
                "created_ns": self.created_ns, **asdict(self.layout),
            }.items():
                header[name] = value
            header["checksum"] = np.frombuffer(_digest(header), "u1")
            self._mapping[8:8 + HEADER_DTYPE.itemsize] = header.tobytes()
            self._atomic.store(int(CollisionStatus.REVISION_PENDING) << 1)
            if self._validate_target() != previous:
                raise ValueError("共享场景公共路径在创建期间发生改变")
            if previous is None:
                # link 在目标已存在时失败，保留并发创建的文件。
                os.link(staging, self.path, follow_symlinks=False)
                staging.unlink()
            else:
                os.replace(staging, self.path)
        except BaseException:
            if hasattr(self, "_mapping"):
                self._mapping.close()
            if hasattr(self, "_fd"):
                os.close(self._fd)
            if staging_created:
                staging.unlink(missing_ok=True)
            os.close(self._lock_fd)
            raise

    def _validate_target(self):
        try:
            target = self.path.lstat()
        except FileNotFoundError:
            return None
        if (not stat.S_ISREG(target.st_mode) or target.st_uid != os.geteuid()
                or target.st_nlink != 1 or target.st_size != self.layout.size_bytes):
            raise ValueError(f"共享场景目标需要本用户的完整协议文件: {self.path}")
        fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        try:
            opened = os.fstat(fd)
            if (opened.st_dev, opened.st_ino) != (target.st_dev, target.st_ino):
                raise ValueError("共享场景目标在身份检查期间发生改变")
            _checked_header(os.pread(fd, HEADER_DTYPE.itemsize, 8), self.layout, _resource_hash(self.path))
            word = int(np.frombuffer(os.pread(fd, 8, 0), dtype=np.uint64, count=1)[0])
            CollisionStatus((word >> 1) & 127)
        finally:
            os.close(fd)
        return target.st_dev, target.st_ino, target.st_ctime_ns, target.st_size

    @property
    def revision(self):
        return self._notice

    def _check_owner(self):
        if self._closed or os.getpid() != self.owner_pid or threading.get_ident() != self.owner_thread:
            raise RuntimeError("仅创建共享场景的运行中 writer 进程可以写入")
        if self._revision >= (1 << 56) - 1:
            raise RuntimeError("scene revision 超出发布字容量")

    def invalidate(self, status=CollisionStatus.INVALID_SCENE):
        self._check_owner()
        status = CollisionStatus(status)
        if status is CollisionStatus.OK:
            raise ValueError("invalidate 需要非 OK 状态")
        self._revision += 1
        self._atomic.store((self._revision << 8) | (int(status) << 1) | self._slot)
        self._notice = replace(self._notice, revision=self._revision, status=status)
        return self._notice

    def publish(self, scene):
        self._check_owner()
        try:
            snapshot, _ = _snapshot(scene, scene.identities, self.safety.config.policy.artifact.parameters)
            if snapshot.metadata.checksum != snapshot.compute_checksum():
                raise ScenePublicationError("scene checksum 无效")
            if int(snapshot.prepared_stamp_ns) < self.created_ns:
                raise ScenePublicationError("scene preparation predates writer epoch")
            original_status, _ = _admission_status(
                snapshot, snapshot.identities, self.safety.config.policy, time.monotonic_ns(), True)
            if original_status is not CollisionStatus.OK:
                self.invalidate(original_status)
                raise ScenePublicationError(f"scene admission: {original_status.name}")
            next_revision = self._revision + 1
            identities = snapshot.identities._replace(
                scene_epoch=np.frombuffer(self.epoch, "u1"), scene_revision=np.int64(next_revision))
            values = {name: np.asarray(getattr(snapshot, name), dtype=np.float64)
                      for name, (_, dtype) in _support_specs(self.layout.support_capacity).items() if dtype == "float"}
            tracks = snapshot.tracks._replace(**{
                name: np.asarray(getattr(snapshot.tracks, name), dtype=np.float64)
                for name, (_, dtype) in _track_specs(self.layout.track_capacity).items() if dtype == "float"})
            model = snapshot.support_time_model
            if model is not None:
                model = model._replace(**{
                    name: np.asarray(getattr(model, name), dtype=np.float64)
                    for name, (_, dtype) in time_model_specs(self.layout.support_capacity, self.layout.member_capacity).items()
                    if dtype == "float"}, scene_epoch=identities.scene_epoch,
                    run_clock_hash=np.frombuffer(run_clock_hash(identities.scene_epoch, snapshot.metadata.clock_id), "u1"))
            snapshot = snapshot._replace(**values, identities=identities, tracks=tracks, support_time_model=model)
            snapshot = snapshot._replace(metadata=replace(snapshot.metadata, checksum=snapshot.compute_checksum()))
            status, _ = _admission_status(snapshot, identities, self.safety.config.policy, time.monotonic_ns(), True)
            if status is not CollisionStatus.OK:
                self.invalidate(status)
                raise ScenePublicationError(f"scene admission: {status.name}")
            slot = 1 - self._slot
            word = (next_revision << 8) | slot
            record = self._encode(snapshot, word)
            offset = self.layout.slot_offset(slot)
            self._mapping[offset:offset + record.nbytes] = record.tobytes()
            final_status, _ = _admission_status(
                snapshot, identities, self.safety.config.policy, time.monotonic_ns(), True)
            if final_status is not CollisionStatus.OK:
                self.invalidate(final_status)
                raise ScenePublicationError(f"scene admission: {final_status.name}")
            # 所有 slot 写入先于唯一 release store；启用 slot 不进行任何修改。
            self._atomic.store(word)
            self._revision, self._slot = next_revision, slot
            self._notice = SceneRevision(self.epoch, next_revision, slot, self.layout.layout_hash, self.source_identity,
                                         int(snapshot.source_stamp_ns), int(snapshot.prepared_stamp_ns), status)
            return self._notice
        except BaseException:
            if self._notice.status is CollisionStatus.OK or self._revision == 0:
                self.invalidate()
            raise

    def _encode(self, scene, word):
        record = np.zeros((), dtype=self.layout.slot_dtype)
        record["publication"] = word
        for name in ("supports", "identities", "tracks"):
            source = scene if name == "supports" else getattr(scene, name)
            record[name] = _array_record(source, record[name].dtype)
        for name in ("coverage_space_hash", "coverage_valid_until_ns"):
            record[name] = getattr(scene, name)
        for name, value in asdict(scene.metadata).items():
            if isinstance(value, str):
                value = value.encode("utf-8")
                if len(value) > record["metadata"].dtype.fields[name][0].itemsize:
                    raise ScenePublicationError(f"metadata.{name} 超出布局容量")
            if name == "checksum":
                value = np.frombuffer(value, "u1")
            record["metadata"][name] = value
        if scene.support_time_model is not None:
            record["has_time_model"] = True
            record["time_model"] = _array_record(scene.support_time_model, record["time_model"].dtype)
        fixed = scene.fixed_environment_model
        if fixed is not None:
            fixed.verify_identity()
            count = len(fixed.support_points_m)
            evidence = fixed.validation_evidence.encode("utf-8")
            if count > self.layout.fixed_capacity or len(evidence) > EVIDENCE_CAPACITY or b"\0" in evidence:
                raise ScenePublicationError("fixed model 来源或 support 超出布局容量")
            record["has_fixed_model"] = True
            for name in fixed.__dataclass_fields__:
                value = getattr(fixed, name)
                if isinstance(value, bytes):
                    value = np.frombuffer(value, "u1")
                if name in ("support_points_m", "support_radii_mm"):
                    record["fixed_model"][name][:count] = value
                else:
                    record["fixed_model"][name] = evidence if name == "validation_evidence" else value
            record["fixed_model"]["support_count"] = count
        record["checksum"] = np.frombuffer(_digest(record), "u1")
        return record

    def close(self):
        if not self._closed:
            self.invalidate()
            self._mapping.close()
            os.close(self._fd)
            os.close(self._lock_fd)
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class SharedSceneReader:
    def __init__(self, path, safety, identities, *, slot_timeout_ns, fixed_environment_model_hash=None,
                 source_identity=None):
        if type(slot_timeout_ns) is not int or slot_timeout_ns <= 0:
            raise ValueError("slot_timeout_ns 必须为正整数")
        self.path = Path(path)
        self.safety = safety
        if not isinstance(identities, CollisionIdentities):
            raise TypeError("reader identities 必须是 CollisionIdentities")
        self.expected = CollisionIdentities(**_copy_arrays(identities, _identity_specs(), "reader.identities"))
        for array in self.expected:
            array.setflags(write=False)
        self.layout = SharedSceneLayout.from_config(safety.config)
        self.source_identity = _source_identity(source_identity)
        self.slot_timeout_ns = slot_timeout_ns
        if fixed_environment_model_hash is not None and (
                not isinstance(fixed_environment_model_hash, bytes) or len(fixed_environment_model_hash) != 32
                or not any(fixed_environment_model_hash)):
            raise ValueError("固定环境预期身份必须是 32-byte 非零 model_hash")
        self.fixed_environment_model_hash = fixed_environment_model_hash
        self.device_transfer_count = 0
        self._epoch = None
        self._last_revision = None
        self._cached_key = None
        self._cached_scene = None
        self._mapping = None
        self._pidfd = None
        self._fault = None
        self._result = SceneReadResult(CollisionStatus.REVISION_PENDING, "revision_pending")
        self._open()

    def _open(self):
        fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        try:
            self._stat = os.fstat(fd)
            if not stat.S_ISREG(self._stat.st_mode):
                raise ValueError("共享场景需要普通文件")
            if self._stat.st_size != self.layout.size_bytes:
                raise ValueError("共享场景文件大小与 layout 不一致")
            self._mapping = mmap.mmap(fd, self.layout.size_bytes, access=mmap.ACCESS_READ)
            self._atomic = _AtomicPublication(self._mapping)
            self._header_identity = None
        finally:
            os.close(fd)

    @property
    def prepared_scene(self):
        # 控制查询只读取本地引用；共享检查与 device 准备仅在 refresh 路径执行。
        return self._result.prepared_scene

    @property
    def result(self):
        return self._result

    @property
    def readonly(self):
        with memoryview(self._mapping) as view:
            return view.readonly

    def reset_fault(self):
        self._fault = None

    def accept_epoch(self, epoch):
        if not isinstance(epoch, bytes) or len(epoch) != 16 or not any(epoch):
            raise ValueError("epoch 必须是 16-byte 非零身份")
        self._epoch = epoch
        self._last_revision = None
        self._cached_key = None
        self._cached_scene = None
        self._fault = None
        self._result = SceneReadResult(CollisionStatus.REVISION_PENDING, "epoch_revalidation_pending")

    def _reject(self, status, reason, notice=None, *, latch=True):
        if latch:
            self._fault = reason
        self._result = SceneReadResult(status, reason, notice)
        return self._result

    def _header(self):
        header = _checked_header(self._mapping[8:8 + HEADER_DTYPE.itemsize], self.layout, _resource_hash(self.path))
        identity = header.tobytes()
        if self._header_identity is not None and identity != self._header_identity:
            raise ValueError("同一共享映射的不可变 header 已发生改变")
        self._header_identity = identity
        return header

    def refresh(self, revision=None):
        try:
            current_stat = self.path.stat()
            if current_stat.st_size != self.layout.size_bytes:
                return self._reject(CollisionStatus.INVALID_SCENE, "shared_file_size_changed")
            if self._mapping is None or (current_stat.st_dev, current_stat.st_ino) != (self._stat.st_dev, self._stat.st_ino):
                self.close()
                self._open()
            before = self._atomic.load()
            previous = self._result.revision
            if previous is None or before != ((previous.revision << 8) | (int(previous.status) << 1) | previous.active_slot):
                self._result = SceneReadResult(CollisionStatus.REVISION_PENDING, "revision_pending")
            header = self._header()
            if bytes(header["source_identity"]) != self.source_identity:
                return self._reject(CollisionStatus.INVALID_SCENE, "source_identity_mismatch")
            epoch = bytes(header["epoch"])
            if self._pidfd is None:
                self._pidfd = os.pidfd_open(int(header["writer_pid"]))
            if select.select([self._pidfd], [], [], 0)[0]:
                return self._reject(CollisionStatus.INVALID_SCENE, "writer_exited")
            if self._epoch is None:
                self._epoch = epoch
            elif epoch != self._epoch:
                return self._reject(CollisionStatus.REVISION_PENDING, "epoch_changed", latch=False)
            number, slot = before >> 8, before & 1
            health = CollisionStatus((before >> 1) & 127)
            if health is not CollisionStatus.OK:
                self._last_revision = number
                return self._reject(health, "writer_health:" + health.name, latch=health is not CollisionStatus.REVISION_PENDING)
            if self._last_revision is not None and number not in (self._last_revision, self._last_revision + 1):
                self._last_revision = number
                return self._reject(CollisionStatus.INVALID_SCENE, "revision_jump")
            self._last_revision = number
            offset = self.layout.slot_offset(slot)
            record = np.frombuffer(self._mapping[offset:offset + self.layout.slot_dtype.itemsize],
                                   dtype=self.layout.slot_dtype, count=1)[0]
            if before != self._atomic.load():
                return self._reject(CollisionStatus.INVALID_SCENE, "publication_changed_during_copy")
            if int(record["publication"]) != before or bytes(record["checksum"]) != _digest(record):
                return self._reject(CollisionStatus.INVALID_SCENE, "slot_checksum_or_publication")
            scene = self._decode(record)
            if self.fixed_environment_model_hash is not None and (
                    scene.fixed_environment_model is None
                    or scene.fixed_environment_model.model_hash != self.fixed_environment_model_hash):
                return self._reject(CollisionStatus.INVALID_SCENE, "fixed_environment_source_mismatch")
            notice = SceneRevision(epoch, number, slot, self.layout.layout_hash, self.source_identity,
                                   int(scene.source_stamp_ns), int(scene.prepared_stamp_ns), health)
            if revision is not None and revision != notice:
                return self._reject(CollisionStatus.INVALID_SCENE, "revision_notice_mismatch", notice)
            if (bytes(scene.identities.scene_epoch) != epoch or int(scene.identities.scene_revision) != number
                    or int(scene.prepared_stamp_ns) < int(header["created_ns"])):
                return self._reject(CollisionStatus.INVALID_SCENE, "slot_identity_mismatch", notice)
            expected = self.expected._replace(scene_epoch=np.frombuffer(epoch, "u1"), scene_revision=np.int64(number))
            now = time.monotonic_ns()
            status, _ = _admission_status(scene, expected, self.safety.config.policy, now,
                                         scene.metadata.checksum == scene.compute_checksum())
            if status is not CollisionStatus.OK:
                return self._reject(status, "scene_admission:" + status.name, notice)
            if now - int(scene.prepared_stamp_ns) > self.slot_timeout_ns:
                return self._reject(CollisionStatus.INVALID_SCENE, "slot_timeout", notice)
            if self._fault is not None:
                return self._reject(CollisionStatus.INVALID_SCENE, "fault_latched:" + self._fault, notice, latch=False)
            key = (epoch, number)
            if key != self._cached_key:
                self._result = SceneReadResult(CollisionStatus.REVISION_PENDING, "device_preparation_pending", notice)
                self._cached_scene = self.safety.prepare_scene(scene, expected)
                self._cached_key = key
                self.device_transfer_count += 1
            prepared = self._cached_scene
            completed = time.monotonic_ns()
            final_status, _ = _admission_status(scene, expected, self.safety.config.policy, completed, True)
            if final_status is not CollisionStatus.OK:
                return self._reject(final_status, "scene_expired_during_prepare:" + final_status.name, notice)
            if (before != self._atomic.load() or select.select([self._pidfd], [], [], 0)[0]
                    or (self.path.stat().st_dev, self.path.stat().st_ino) != (self._stat.st_dev, self._stat.st_ino)):
                return self._reject(CollisionStatus.INVALID_SCENE, "publication_changed_during_prepare", notice)
            if (completed - int(scene.prepared_stamp_ns) > self.slot_timeout_ns
                    or completed > int(prepared.valid_until_ns)):
                return self._reject(CollisionStatus.INVALID_SCENE, "slot_timeout", notice)
            if int(prepared.status) != CollisionStatus.OK:
                return self._reject(CollisionStatus(int(prepared.status)), "device_preparation_rejected", notice)
            self._result = SceneReadResult(CollisionStatus.OK, "ok", notice, prepared)
            return self._result
        except (OSError, ValueError, TypeError, OverflowError) as error:
            return self._reject(CollisionStatus.INVALID_SCENE, type(error).__name__ + ":" + str(error))

    def _decode(self, record):
        metadata = {}
        for name in METADATA_DTYPE.names:
            value = record["metadata"][name]
            if name == "checksum":
                value = bytes(value)
            elif value.dtype.kind == "S":
                value = bytes(value).decode("utf-8")
            else:
                value = value.item()
            metadata[name] = value
        values = {name: np.array(record["supports"][name], copy=True) for name in record["supports"].dtype.names}
        values.update(identities=_arrays(record["identities"], CollisionIdentities),
                      tracks=_arrays(record["tracks"], SceneTracks), metadata=SceneMetadata(**metadata),
                      coverage_space_hash=np.array(record["coverage_space_hash"], copy=True),
                      coverage_valid_until_ns=np.array(record["coverage_valid_until_ns"], copy=True))
        if record["has_time_model"]:
            values["support_time_model"] = _arrays(record["time_model"], SupportTimeModel)
        if record["has_fixed_model"]:
            source = record["fixed_model"]
            count = int(source["support_count"])
            if not 0 <= count <= self.layout.fixed_capacity:
                raise ValueError("fixed model 数量超出共享布局容量")
            fixed = {}
            for name in FixedEnvironmentModel.__dataclass_fields__:
                value = source[name]
                if name in ("support_points_m", "support_radii_mm"):
                    value = np.array(value[:count], copy=True)
                elif name == "validation_evidence":
                    value = bytes(value).decode("utf-8")
                elif name.endswith("hash"):
                    value = bytes(value)
                else:
                    value = value.item()
                fixed[name] = value
            values["fixed_environment_model"] = FixedEnvironmentModel(**fixed)
        return CollisionScene(**values)

    def close(self):
        if self._mapping is not None:
            self._mapping.close()
            self._mapping = None
        if self._pidfd is not None:
            os.close(self._pidfd)
            self._pidfd = None
        self._result = SceneReadResult(CollisionStatus.INVALID_SCENE, "reader_closed")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
