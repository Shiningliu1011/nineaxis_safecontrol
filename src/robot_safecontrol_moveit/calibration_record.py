"""标定记录（calibration record）的加载、身份与准入判据。

OFF-01 的实现，契约面见 ADR 0004（谁是真源）与 ADR 0009（记录长什么样、身份怎么算、
未标定记录能不能进入避障）。本模块**刻意不依赖 rclpy**：路径由调用方注入，
因此加载、身份计算与启动分级都能离线跑测试；ROS 节点只负责提供路径、启用源集合
与发布诊断。

两个身份（ADR 0009 决定 2）：

* ``file_sha256`` —— 实际加载的原始字节的 sha256，证明「节点读到的就是被检查的那份文件」。
* ``calibration_id`` —— ``"v1:" + sha256(canonical)[:16]``，canonical 是对显式白名单字段
  做 UTF-8 + 排序键 + 最短往返浮点 + 拒绝 NaN/Inf/重复键的规范化结果（见
  :func:`canonical_bytes`）。标定工具（OFF-16）必须复用同一个函数。

准入判据（ADR 0009 决定 3，实施期补充了第 4 个因子）：

    admission_ready = math_valid ∧ calibrated ∧ provenance_valid ∧ identity_consistent

启动分级（ADR 0009 决定 4）：记录读不到、schema 不支持、矩阵数学非法、帧绑定对不上、
检测到旧外参参数覆盖 —— 一律拒绝启动；``calibrated: false`` 且该源被部署 profile
豁免时启动 + WARN + 不可准入。**没有 identity 兜底**。
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

SCHEMA_VERSION = 1
SOURCES = ("camera", "lidar")
DEFAULT_PACKAGE = "robot_safecontrol_moveit"
DEFAULT_FILENAME = "sensor_extrinsics.yaml"

# 顶层键白名单。多一个未声明的块就报错，避免把拼错的小节（例如 ``Camera:``）
# 静默当成「该源不存在」。新增块时在此处加一行即可。
TOP_LEVEL_KEYS = frozenset({"schema_version", "camera", "lidar", "orbbec_launch"})

# 参与 calibration_id 的字段（ADR 0009 决定 2 的白名单）。
CANONICAL_FIELDS = (
    "schema_version",
    "source",
    "frame_from",
    "frame_to",
    "matrix",
    "calibrated",
    "method",
    "operator",
    "timestamp",
    "sensor_serial",
    "residual",
    "error_estimate",
)

# provenance 完整性判据。任何记录都必须说清「这份几何怎么来的」；
# 声称 ``calibrated: true`` 还必须说清「什么时候、对哪台设备、误差多少」。
PROVENANCE_REQUIRED_ALWAYS = ("method", "operator")
PROVENANCE_REQUIRED_CALIBRATED = ("timestamp", "sensor_serial")
PROVENANCE_QUALITY_FIELDS = ("residual", "error_estimate")

# 软件合法性容差，不是标定精度（ADR 0009 决定 4）。
MATH_TOL = 1e-6
TRANSLATION_ENVELOPE_M = 10.0

# --- 错误码（诊断与测试都按字面量断言） -------------------------------------
CALIB_RECORD_MISSING = "CALIB_RECORD_MISSING"
CALIB_RECORD_UNREADABLE = "CALIB_RECORD_UNREADABLE"
CALIB_RECORD_YAML_INVALID = "CALIB_RECORD_YAML_INVALID"
CALIB_RECORD_NOT_MAPPING = "CALIB_RECORD_NOT_MAPPING"
CALIB_SCHEMA_UNSUPPORTED = "CALIB_SCHEMA_UNSUPPORTED"
CALIB_SECTION_UNKNOWN = "CALIB_SECTION_UNKNOWN"
CALIB_SOURCE_MISSING = "CALIB_SOURCE_MISSING"
CALIB_FIELD_MISSING = "CALIB_FIELD_MISSING"
CALIB_FRAME_MISMATCH = "CALIB_FRAME_MISMATCH"
CALIB_MATRIX_SHAPE = "CALIB_MATRIX_SHAPE"
CALIB_MATRIX_NOT_FINITE = "CALIB_MATRIX_NOT_FINITE"
CALIB_MATRIX_NOT_ORTHONORMAL = "CALIB_MATRIX_NOT_ORTHONORMAL"
CALIB_MATRIX_DETERMINANT = "CALIB_MATRIX_DETERMINANT"
CALIB_MATRIX_BOTTOM_ROW = "CALIB_MATRIX_BOTTOM_ROW"
CALIB_TRANSLATION_OUT_OF_ENVELOPE = "CALIB_TRANSLATION_OUT_OF_ENVELOPE"
CALIB_PROVENANCE_MISSING = "CALIB_PROVENANCE_MISSING"
CALIB_NOT_CALIBRATED = "CALIB_NOT_CALIBRATED"
CALIB_ID_UNAVAILABLE = "CALIB_ID_UNAVAILABLE"
CALIB_ID_MISMATCH = "CALIB_ID_MISMATCH"
CALIB_SOURCE_NOT_EXEMPT = "CALIB_SOURCE_NOT_EXEMPT"
CALIB_LEGACY_PARAM_PRESENT = "CALIB_LEGACY_PARAM_PRESENT"
CALIB_ACCEPTANCE_NOT_EVALUATED = "CALIB_ACCEPTANCE_NOT_EVALUATED"

LEVEL_OK = "OK"
LEVEL_WARN = "WARN"
LEVEL_ERROR = "ERROR"

# DiagnosticStatus level codes (diagnostic_msgs/msg/DiagnosticStatus).
DIAGNOSTIC_LEVELS = {LEVEL_OK: 0, LEVEL_WARN: 1, LEVEL_ERROR: 2}


class CalibrationRecordError(RuntimeError):
    """标定记录无法建立可用快照（路径解析失败、读不到、YAML 坏了）。"""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


class CalibrationStartupRefused(RuntimeError):
    """启动被判据拒绝。``decision`` 携带完整分级结果供调用方记录。"""

    def __init__(self, decision: "StartupDecision") -> None:
        super().__init__(
            "calibration startup refused: " + ", ".join(decision.errors))
        self.decision = decision


# ---------------------------------------------------------------------------
# 快照与身份
# ---------------------------------------------------------------------------
@dataclass(frozen=True, eq=False)
class SourceRecord:
    """一个传感器的标定记录分节及其判据结果。"""

    source: str
    present: bool
    frame_from: str | None
    frame_to: str | None
    matrix: np.ndarray | None
    calibrated: bool
    provenance: Mapping[str, Any]
    declared_calibration_id: str | None
    calibration_id: str | None
    math_valid: bool
    math_errors: tuple[str, ...]
    provenance_valid: bool
    provenance_errors: tuple[str, ...]
    identity_consistent: bool

    @property
    def admission_ready(self) -> bool:
        return bool(
            self.math_valid
            and self.calibrated
            and self.provenance_valid
            and self.identity_consistent
        )

    @property
    def uncalibrated(self) -> bool:
        return not self.calibrated


@dataclass(frozen=True, eq=False)
class CalibrationRecord:
    """一次加载得到的不可变快照（启动期读一次，运行期不再热重载）。"""

    lookup_path: str
    resolved_path: str
    file_sha256: str | None
    read_ok: bool
    schema_version: int | None
    sources: Mapping[str, SourceRecord]
    errors: tuple[str, ...] = ()
    acceptance_evaluated: bool = False

    def source(self, name: str) -> SourceRecord | None:
        return self.sources.get(name)


def _canonical_value(value: Any) -> Any:
    """规范化一个叶子/容器值：数值统一按 float，容器递归，其余原样。

    数值叶子统一成 float，是为了让 ``1`` 与 ``1.0`` 这类写法差异不改变身份；
    ``bool`` 必须在 ``int`` 之前判断（Python 里 ``bool`` 是 ``int`` 的子类）。
    """
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise TypeError("canonical: mapping keys must be strings")
        return {k: _canonical_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_canonical_value(v) for v in value]
    raise TypeError(f"canonical: unsupported value type {type(value).__name__}")


def _canonical_object(
    source: str, entry: Mapping[str, Any], schema_version: int
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": schema_version,
        "source": source,
    }
    for name in CANONICAL_FIELDS:
        if name in ("schema_version", "source"):
            continue
        # 缺席与显式 null 等价：都规范化为 null，避免写不写该字段就改变身份。
        payload[name] = _canonical_value(entry.get(name))
    return payload


def canonical_bytes(
    source: str, entry: Mapping[str, Any], *, schema_version: int = SCHEMA_VERSION
) -> bytes:
    """返回单源记录的规范化字节（身份计算的唯一输入）。"""
    payload = _canonical_object(source, entry, schema_version)
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def compute_calibration_id(
    source: str, entry: Mapping[str, Any], *, schema_version: int = SCHEMA_VERSION
) -> str:
    """``"v1:" + sha256(canonical)[:16]``（ADR 0009 决定 2）。

    矩阵含 NaN/Inf 或字段不可序列化时抛 :class:`CalibrationRecordError`；调用方
    应把它降级成 ``CALIB_ID_UNAVAILABLE``，而不是拿一个假身份继续。
    """
    try:
        canonical = canonical_bytes(source, entry, schema_version=schema_version)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CalibrationRecordError(
            CALIB_ID_UNAVAILABLE, f"{source} record cannot be canonicalised: {exc}"
        ) from exc
    return "v1:" + hashlib.sha256(canonical).hexdigest()[:16]


# ---------------------------------------------------------------------------
# 路径解析
# ---------------------------------------------------------------------------
def _ament_share_dir(package: str) -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package))


def resolve_record_path(
    *,
    explicit: str | Path | None = None,
    share_dir: str | Path | None = None,
    package: str = DEFAULT_PACKAGE,
    filename: str = DEFAULT_FILENAME,
) -> tuple[str, str]:
    """解析出唯一的运行时记录路径，返回 ``(lookup_path, resolved_path)``。

    ADR 0004「运行时文件身份」：生产路径必须经 ament share 解析，源码树副本不算
    运行时真源。``lookup_path`` 是解析前的构造路径（便于看出 source/install 或
    symlink-install 的分叉），``resolved_path`` 是 fully resolved 绝对路径。
    显式路径与 ``share_dir`` 注入供测试和开发使用。
    """
    if explicit is not None:
        path = Path(explicit)
    elif share_dir is not None:
        path = Path(share_dir) / "config" / filename
    else:
        try:
            base = _ament_share_dir(package)
        except Exception as exc:  # PackageNotFoundError 及其他索引失败
            raise CalibrationRecordError(
                CALIB_RECORD_MISSING,
                f"cannot resolve {package}/config/{filename} via ament index: {exc}",
            ) from exc
        path = base / "config" / filename
    try:
        resolved = path.resolve()
    except (OSError, RuntimeError) as exc:
        raise CalibrationRecordError(
            CALIB_RECORD_UNREADABLE, f"cannot resolve {path}: {exc}",
        ) from exc
    return str(path), str(resolved)


# ---------------------------------------------------------------------------
# YAML 解析（拒绝重复键）
# ---------------------------------------------------------------------------
class _StrictLoader(yaml.SafeLoader):
    """SafeLoader + 重复键报错（ADR 0009 决定 2：规范化拒绝重复键）。"""


def _no_duplicate_keys(loader: yaml.Loader, node: yaml.Node, deep: bool = False) -> dict:
    mapping: dict = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as exc:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"unhashable key {key!r}",
                key_node.start_mark,
            ) from exc
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"found duplicate key {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_StrictLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_duplicate_keys
)


def _parse_yaml(data: bytes) -> Mapping[str, Any]:
    try:
        document = yaml.load(data, Loader=_StrictLoader)
    except (yaml.YAMLError, ValueError, OverflowError) as exc:
        raise CalibrationRecordError(CALIB_RECORD_YAML_INVALID, str(exc)) from exc
    if document is None:
        raise CalibrationRecordError(CALIB_RECORD_YAML_INVALID, "document is empty")
    if not isinstance(document, Mapping):
        raise CalibrationRecordError(
            CALIB_RECORD_NOT_MAPPING,
            f"top level must be a mapping, got {type(document).__name__}",
        )
    return document


# ---------------------------------------------------------------------------
# 数学合法性
# ---------------------------------------------------------------------------
def _matrix_from_entry(source: str, raw: Any) -> tuple[np.ndarray | None, list[str]]:
    errors: list[str] = []
    if not isinstance(raw, (list, tuple)) or len(raw) != 16:
        return None, [f"{CALIB_MATRIX_SHAPE}: {source} matrix must be 16 row-major floats"]
    flat: list[float] = []
    for index, value in enumerate(raw):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            errors.append(
                f"{CALIB_MATRIX_SHAPE}: {source} matrix element {index} is not a number")
            return None, errors
        try:
            number = float(value)
        except OverflowError:
            return None, [
                f"{CALIB_MATRIX_NOT_FINITE}: {source} matrix element {index}"
                " exceeds float range"]
        if not math.isfinite(number):
            errors.append(
                f"{CALIB_MATRIX_NOT_FINITE}: {source} matrix element {index} = {value!r}")
            return None, errors
        flat.append(number)
    return np.asarray(flat, dtype=np.float64).reshape(4, 4), errors


def validate_matrix(source: str, matrix: np.ndarray) -> list[str]:
    """检查 4x4 是否为合法 SE(3)（软件合法性，不是标定精度）。"""
    errors: list[str] = []
    rotation = matrix[:3, :3]
    orthonormal = np.max(np.abs(rotation.T @ rotation - np.eye(3)))
    if not math.isfinite(orthonormal) or orthonormal > MATH_TOL:
        errors.append(
            f"{CALIB_MATRIX_NOT_ORTHONORMAL}: {source} |RᵀR-I|max={orthonormal:.3e}"
            f" > {MATH_TOL:g}")
    determinant = float(np.linalg.det(rotation))
    if not math.isfinite(determinant) or abs(determinant - 1.0) > MATH_TOL:
        # 不取绝对值：det = -1 是镜像，会让点云左右手性反转。
        errors.append(
            f"{CALIB_MATRIX_DETERMINANT}: {source} det(R)={determinant:.6f} != +1"
            f" (tolerance {MATH_TOL:g})")
    bottom_row = matrix[3, :]
    if not np.allclose(bottom_row, [0.0, 0.0, 0.0, 1.0], atol=MATH_TOL, rtol=0.0):
        errors.append(
            f"{CALIB_MATRIX_BOTTOM_ROW}: {source} bottom row={bottom_row.tolist()}"
            f" != [0, 0, 0, 1]")
    translation = matrix[:3, 3]
    if np.any(np.abs(translation) > TRANSLATION_ENVELOPE_M):
        errors.append(
            f"{CALIB_TRANSLATION_OUT_OF_ENVELOPE}: {source} |t|max="
            f"{float(np.max(np.abs(translation))):.3f} m exceeds "
            f"{TRANSLATION_ENVELOPE_M:g} m mechanical envelope")
    return errors


# ---------------------------------------------------------------------------
# 单源记录
# ---------------------------------------------------------------------------
def _provenance_errors(source: str, entry: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    calibrated = entry.get("calibrated")
    if not isinstance(calibrated, bool):
        errors.append(
            f"{CALIB_FIELD_MISSING}: {source}.calibrated must be an explicit bool,"
            f" got {calibrated!r}")
        return errors
    required = set(PROVENANCE_REQUIRED_ALWAYS)
    if calibrated:
        required.update(PROVENANCE_REQUIRED_CALIBRATED)
    for name in (*PROVENANCE_REQUIRED_ALWAYS, *PROVENANCE_REQUIRED_CALIBRATED):
        value = entry.get(name)
        if name in required and (
            not isinstance(value, str) or not value.strip()
        ):
            errors.append(
                f"{CALIB_PROVENANCE_MISSING}: {source}.{name} must be a non-empty"
                f" string{' for a calibrated record' if calibrated else ''}")
        elif name not in required and value is not None and (
            not isinstance(value, str) or not value.strip()
        ):
            errors.append(
                f"{CALIB_PROVENANCE_MISSING}: {source}.{name} must be null or a"
                f" non-empty string for an uncalibrated record")
    if calibrated and all(entry.get(n) is None for n in PROVENANCE_QUALITY_FIELDS):
        errors.append(
            f"{CALIB_PROVENANCE_MISSING}: {source} claims calibrated: true but has"
            f" neither residual nor error_estimate")
    return errors


def build_source_record(
    source: str,
    entry: Any,
    *,
    schema_version: int = SCHEMA_VERSION,
) -> SourceRecord:
    """把一个分节的原始映射变成带判据的 :class:`SourceRecord`。"""
    if entry is None:
        return SourceRecord(
            source=source, present=False, frame_from=None, frame_to=None, matrix=None,
            calibrated=False, provenance={}, declared_calibration_id=None,
            calibration_id=None, math_valid=False,
            math_errors=(f"{CALIB_SOURCE_MISSING}: record has no '{source}' section",),
            provenance_valid=False,
            provenance_errors=(f"{CALIB_SOURCE_MISSING}: record has no '{source}' section",),
            identity_consistent=False,
        )
    if not isinstance(entry, Mapping):
        message = f"{CALIB_SOURCE_MISSING}: '{source}' section must be a mapping"
        return SourceRecord(
            source=source, present=True, frame_from=None, frame_to=None, matrix=None,
            calibrated=False, provenance={}, declared_calibration_id=None,
            calibration_id=None, math_valid=False, math_errors=(message,),
            provenance_valid=False, provenance_errors=(message,),
            identity_consistent=False,
        )

    frame_from = entry.get("frame_from")
    frame_to = entry.get("frame_to")
    matrix, math_errors = _matrix_from_entry(source, entry.get("matrix"))
    if matrix is not None:
        math_errors = math_errors + validate_matrix(source, matrix)
    calibrated = entry.get("calibrated") is True
    provenance = {
        name: entry.get(name)
        for name in (
            "calibrated",
            "method",
            "operator",
            "timestamp",
            "sensor_serial",
            "residual",
            "error_estimate",
        )
    }
    provenance_errors = _provenance_errors(source, entry)

    raw_declared = entry.get("calibration_id")
    if raw_declared is None:
        declared = None
    elif isinstance(raw_declared, str):
        declared = raw_declared
    else:
        # Preserve the malformed declaration in diagnostics instead of silently
        # treating it as "not declared" and admitting the record.
        declared = f"<invalid {type(raw_declared).__name__}: {raw_declared!r}>"
    try:
        computed = compute_calibration_id(source, entry, schema_version=schema_version)
    except CalibrationRecordError as exc:
        computed = None
        math_errors = math_errors + [str(exc)]
    identity_consistent = raw_declared is None or (
        isinstance(raw_declared, str) and raw_declared == computed
    )

    return SourceRecord(
        source=source,
        present=True,
        frame_from=frame_from if isinstance(frame_from, str) else None,
        frame_to=frame_to if isinstance(frame_to, str) else None,
        matrix=matrix,
        calibrated=calibrated,
        provenance=provenance,
        declared_calibration_id=declared,
        calibration_id=computed,
        math_valid=not math_errors,
        math_errors=tuple(math_errors),
        provenance_valid=not provenance_errors,
        provenance_errors=tuple(provenance_errors),
        identity_consistent=identity_consistent,
    )


# ---------------------------------------------------------------------------
# 加载
# ---------------------------------------------------------------------------
def load_calibration_record(
    *,
    explicit: str | Path | None = None,
    share_dir: str | Path | None = None,
    package: str = DEFAULT_PACKAGE,
    filename: str = DEFAULT_FILENAME,
    data: bytes | None = None,
    acceptance_evaluated: bool = False,
) -> CalibrationRecord:
    """读取一次字节快照，用它同时完成 hash、解析与校验（ADR 0009 决定 2）。

    ``data`` 直接注入字节内容供测试使用；此时路径解析仍照常进行，便于断言
    「被检查的那份字节」与「节点声称加载的路径」一致。
    """
    lookup_path, resolved_path = resolve_record_path(
        explicit=explicit, share_dir=share_dir, package=package, filename=filename
    )
    if data is None:
        try:
            data = Path(resolved_path).read_bytes()
        except FileNotFoundError as exc:
            raise CalibrationRecordError(
                CALIB_RECORD_MISSING, f"{resolved_path} does not exist"
            ) from exc
        except OSError as exc:
            raise CalibrationRecordError(
                CALIB_RECORD_UNREADABLE, f"{resolved_path}: {exc}"
            ) from exc
    file_sha256 = hashlib.sha256(data).hexdigest()
    document = _parse_yaml(data)

    errors: tuple[str, ...] = ()
    unknown = sorted(
        (key for key in document if key not in TOP_LEVEL_KEYS), key=repr)
    if unknown:
        errors = (f"{CALIB_SECTION_UNKNOWN}: unknown top-level keys {unknown}",)

    raw_version = document.get("schema_version")
    # ``bool`` is a subclass of ``int`` in Python; YAML ``true`` must not be
    # accepted as schema version 1.
    schema_version = raw_version if type(raw_version) is int else None
    if schema_version != SCHEMA_VERSION:
        errors = errors + (
            f"{CALIB_SCHEMA_UNSUPPORTED}: schema_version={raw_version!r},"
            f" this loader supports {SCHEMA_VERSION}",
        )

    sources = {
        name: build_source_record(name, document.get(name), schema_version=SCHEMA_VERSION)
        for name in SOURCES
    }
    return CalibrationRecord(
        lookup_path=lookup_path,
        resolved_path=resolved_path,
        file_sha256=file_sha256,
        read_ok=not errors,
        schema_version=schema_version,
        sources=sources,
        errors=errors,
        acceptance_evaluated=acceptance_evaluated,
    )


# ---------------------------------------------------------------------------
# 启动分级
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SourceDecision:
    source: str
    enabled: bool
    exempt: bool
    admitted: bool
    level: str
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    @property
    def starts_up(self) -> bool:
        return self.level != LEVEL_ERROR


@dataclass(frozen=True)
class StartupDecision:
    start: bool
    level: str
    errors: tuple[str, ...]
    warnings: tuple[str, ...]
    notes: tuple[str, ...]
    sources: Mapping[str, SourceDecision] = field(default_factory=dict)
    exempt_sources: frozenset[str] = field(default_factory=frozenset)


def check_frame_binding(
    source: str,
    source_record: SourceRecord,
    *,
    expected_from: str | None,
    expected_to: str | None,
) -> list[str]:
    """交叉校验记录声明的帧与运行时配置的帧。

    记录来自另一个帧（例如换过相机、或相机 335L→335Le 后 optical frame 变了）时，
    点云会被静默按错误几何变换，所以这里按 fail-closed 处理。
    """
    errors: list[str] = []
    if not source_record.frame_to or not source_record.frame_to.strip():
        errors.append(
            f"{CALIB_FRAME_MISMATCH}: {source} record has no non-empty frame_to")
    elif expected_to is not None and source_record.frame_to != expected_to:
        errors.append(
            f"{CALIB_FRAME_MISMATCH}: {source} record maps into"
            f" frame_to={source_record.frame_to!r} but world_frame={expected_to!r}")
    if not source_record.frame_from or not source_record.frame_from.strip():
        errors.append(
            f"{CALIB_FRAME_MISMATCH}: {source} record has no non-empty frame_from")
    elif expected_from is not None and source_record.frame_from != expected_from:
        errors.append(
            f"{CALIB_FRAME_MISMATCH}: {source} record is for"
            f" frame_from={source_record.frame_from!r} but the configured source frame is"
            f" {expected_from!r}")
    return errors


def decide_startup(
    record: CalibrationRecord,
    *,
    enabled_sources: Iterable[str],
    exempt_sources: Iterable[str] = (),
    frame_expectations: Mapping[str, tuple[str | None, str | None]] | None = None,
    legacy_overrides: Mapping[str, Any] | None = None,
) -> StartupDecision:
    """把一次加载快照 + 运行时配置变成启动结论（ADR 0009 决定 3/4）。

    ``enabled_sources``：实际被启用的源（订阅存在）。
    ``exempt_sources``：部署 profile 显式允许「未标定调试」的源 —— 只影响能否启动，
    不改变准入结论。
    ``frame_expectations``：``{source: (expected_from, expected_to)}``。
    ``legacy_overrides``：检测到的旧外参参数覆盖 ``{名称: 值}``，非空即拒绝
    （T7 交接口径 `27-calibration-ssot.md:74`）。
    """
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()
    notes: tuple[str, ...] = ()

    if not record.read_ok:
        errors = record.errors
        return StartupDecision(
            start=False, level=LEVEL_ERROR, errors=errors, warnings=warnings,
            notes=notes, sources={},
        )

    frame_expectations = frame_expectations or {}
    enabled_set = set(enabled_sources)
    enabled = [name for name in SOURCES if name in enabled_set]
    exempt = set(exempt_sources)
    decisions: dict[str, SourceDecision] = {}

    for name in enabled:
        source = record.source(name)
        source_errors: list[str] = []
        expected_from, expected_to = frame_expectations.get(name, (None, None))
        if source is None or not source.present:
            source_errors.append(f"{CALIB_SOURCE_MISSING}: no '{name}' section")
        else:
            if not source.math_valid:
                source_errors += list(source.math_errors)
            source_errors += check_frame_binding(
                name, source, expected_from=expected_from, expected_to=expected_to)

        is_exempt = name in exempt
        if source_errors:
            decisions[name] = SourceDecision(
                source=name, enabled=True, exempt=is_exempt, admitted=False,
                level=LEVEL_ERROR, errors=tuple(source_errors))
            errors = errors + tuple(source_errors)
            continue

        assert source is not None and source.present  # source_errors empty ⇒ section present
        if source.admission_ready:
            decisions[name] = SourceDecision(
                source=name, enabled=True, exempt=is_exempt, admitted=True,
                level=LEVEL_OK)
            continue

        # 没通过标定门的**具体**原因（不能一律说成「未标定」：身份不一致的记录
        # 可能 calibrated: true）。
        blocking_gate: list[str] = []
        warning_gate: list[str] = []
        if not source.calibrated:
            blocking_gate.append(
                f"{CALIB_NOT_CALIBRATED}: {name} record is calibrated: false")
        if not source.identity_consistent:
            warning_gate.append(
                f"{CALIB_ID_MISMATCH}: declared {source.declared_calibration_id!r}"
                f" != computed {source.calibration_id!r}")
        blocking_gate += list(source.provenance_errors)
        blocking_gate = list(dict.fromkeys(blocking_gate))
        warning_gate = list(dict.fromkeys(warning_gate))

        if blocking_gate and is_exempt:
            gate = blocking_gate + warning_gate
            decisions[name] = SourceDecision(
                source=name, enabled=True, exempt=True, admitted=False,
                level=LEVEL_WARN, errors=(), warnings=tuple(gate))
            warnings = warnings + (
                f"{name}: starting without a passing calibration gate"
                f" ({'; '.join(gate)})",)
        elif blocking_gate:
            refusal = blocking_gate + [
                f"{CALIB_SOURCE_NOT_EXEMPT}: {name} is enabled but its record did not"
                f" pass the calibration gate; declare it in the deployment profile's"
                f" allow_uncalibrated_debug only for debug runs"]
            decisions[name] = SourceDecision(
                source=name, enabled=True, exempt=False, admitted=False,
                level=LEVEL_ERROR, errors=tuple(refusal),
                warnings=tuple(warning_gate))
            errors = errors + tuple(refusal)
            if warning_gate:
                warnings = warnings + (
                    f"{name}: calibration identity is inconsistent"
                    f" ({'; '.join(warning_gate)})",)
        else:
            # A declared identity mismatch is deliberately non-fatal.  It
            # blocks admission and emits WARN, but must not require the debug
            # exemption: a future canonicalisation change must not brick an
            # otherwise well-formed calibrated deployment (ADR 0009 decision 3).
            assert warning_gate
            decisions[name] = SourceDecision(
                source=name, enabled=True, exempt=is_exempt, admitted=False,
                level=LEVEL_WARN, warnings=tuple(warning_gate))
            warnings = warnings + (
                f"{name}: calibration identity is inconsistent"
                f" ({'; '.join(warning_gate)})",)

    # A missing disabled source is allowed (it means that source is absent), but
    # a section that is present must never carry malformed geometry.  Otherwise
    # a camera-only run could bless a corrupt shared calibration record and the
    # failure would surface only after LiDAR was enabled.
    for name in SOURCES:
        if name in enabled_set:
            continue
        source = record.source(name)
        if source is not None and source.present and not source.math_valid:
            errors = errors + tuple(source.math_errors)

    if legacy_overrides:
        legacy = [
            f"{CALIB_LEGACY_PARAM_PRESENT}: {key}={value!r} is a removed extrinsics"
            f" parameter; the transform now comes from the calibration record only"
            for key, value in sorted(legacy_overrides.items())
        ]
        errors = errors + tuple(legacy)

    if not record.acceptance_evaluated:
        # ADR 0009 决定 4：20mm/5° 实测验收不在本轮范围，诊断必须显式标「未评估」，
        # 不得用占位数据充数，也不因此降级 —— 它既不是通过也不是失败。
        notes = notes + (
            f"{CALIB_ACCEPTANCE_NOT_EVALUATED}: measured acceptance (known-size object"
            f" < 20 mm) is not evaluated by this startup gate",
        )

    level = LEVEL_ERROR if errors else (
        LEVEL_WARN if any(d.level == LEVEL_WARN for d in decisions.values())
        else LEVEL_OK)
    if not decisions and not errors:
        level = LEVEL_WARN
        warnings = warnings + (
            "no sensor source is enabled; the perception bridge has nothing to fuse",
        )
    return StartupDecision(
        start=not errors,
        level=level,
        errors=errors,
        warnings=warnings,
        notes=notes,
        sources=decisions,
        exempt_sources=frozenset(exempt),
    )


# ---------------------------------------------------------------------------
# 诊断条目（纯数据；ROS 节点负责映射成 DiagnosticArray）
# ---------------------------------------------------------------------------
def _source_time_domain(declared: str | None) -> dict[str, str]:
    """逐源时间域：只报显式声明的值，未知就报 unknown。

    绝不照抄驱动默认值 —— 同样的参数默认值在不同机型/固件上语义不同（见
    `docs/planning/oscbf-reuse/research/sensor-time-alignment-20260914.md`）。
    """
    if declared is None or not str(declared).strip():
        return {
            "time_domain": "unknown",
            "time_domain_source": "unknown",
        }
    return {"time_domain": str(declared), "time_domain_source": "declared_by_profile"}


def calibration_status_entries(
    record: CalibrationRecord,
    decision: StartupDecision,
    *,
    node_identity: str,
    time_domains: Mapping[str, str | None] | None = None,
    acceptance_note: str | None = None,
) -> list[dict[str, Any]]:
    """构造 ``/perception/calibration_status`` 的条目（name/level/values）。

    ``node_identity`` 必须能区分节点实例（如 ``name#pid``）：消费端据此判断
    「这份报告属于当前实例」，并配合本机单调时钟判断报告新鲜度（ROS 时间可暂停、
    回跳、归零，不能用消息时间戳当新鲜度依据）。
    """
    time_domains = time_domains or {}
    record_warnings = list(decision.warnings) + list(decision.notes)
    entries: list[dict[str, Any]] = [{
        "name": "perception_bridge:record",
        "level": decision.level,
        "message": (
            "calibration record loaded"
            if decision.start else
            "calibration record refused: " + "; ".join(decision.errors)
        ),
        "values": {
            "node_identity": node_identity,
            "lookup_path": record.lookup_path,
            "resolved_path": record.resolved_path,
            "file_sha256": record.file_sha256 or "unknown",
            "schema_version": str(record.schema_version),
            "sources_present": ",".join(
                sorted(n for n, s in record.sources.items() if s.calibration_id)
            ) or "none",
            "start": "true" if decision.start else "false",
            "errors": "; ".join(decision.errors) or "none",
            "warnings": "; ".join(record_warnings) or "none",
            "acceptance": (
                acceptance_note or CALIB_ACCEPTANCE_NOT_EVALUATED
            ),
        },
    }]

    for name in SOURCES:
        source = record.source(name)
        if source is None:
            continue
        # 记录里存在的每个分节都上报（含未启用的源），用 enabled 区分：「记录里有什么」
        # 和「这次实际用了什么」是 T6 要能分开看到的两件事。
        source_decision = decision.sources.get(name)
        enabled = source_decision.enabled if source_decision else False
        if not enabled and source.calibration_id is None:
            continue
        level = source_decision.level if source_decision else LEVEL_WARN
        reported_exempt = (
            source_decision.exempt
            if source_decision else name in decision.exempt_sources
        )
        issues = list(source_decision.errors) + list(source_decision.warnings) \
            if source_decision else []
        issues += list(source.math_errors) + list(source.provenance_errors)
        if not source.identity_consistent:
            issues.append(
                f"{CALIB_ID_MISMATCH}: declared={source.declared_calibration_id}"
                f" computed={source.calibration_id}")
        values = {
            "node_identity": node_identity,
            "enabled": "true" if enabled else "false",
            "exempt": "true" if reported_exempt else "false",
            "calibrated": "true" if source.calibrated else "false",
            "math_valid": "true" if source.math_valid else "false",
            "provenance_valid": "true" if source.provenance_valid else "false",
            "identity_consistent": "true" if source.identity_consistent else "false",
            "admission_ready": "true" if source.admission_ready else "false",
            "calibration_id": source.calibration_id or "unavailable",
            "declared_calibration_id": source.declared_calibration_id or "none",
            "frame_from": source.frame_from or "unknown",
            "frame_to": source.frame_to or "unknown",
            "method": str(source.provenance.get("method") or "unknown"),
            "operator": str(source.provenance.get("operator") or "unknown"),
            "sensor_serial": str(source.provenance.get("sensor_serial") or "unknown"),
            "timestamp": str(source.provenance.get("timestamp") or "unknown"),
            "issues": "; ".join(dict.fromkeys(issues)) or "none",
        }
        values.update(_source_time_domain(time_domains.get(name)))
        entries.append({
            "name": f"perception_bridge:{name}",
            "level": level,
            "message": (
                "no calibration record section"
                if source.calibration_id is None else
                ("calibration gate passed" if source.admission_ready else
                 "not admissible" + ("" if not source.calibrated else
                                     " (identity or provenance problem)"))
            ),
            "values": values,
        })
    return entries
