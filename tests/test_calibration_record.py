#!/usr/bin/env python3
"""OFF-01 标定记录加载器契约测试（纯 Python，不启动 ROS）。

覆盖 ADR 0009 的四组决定：记录 schema、两种身份、准入与逐源豁免、启动期失败分级。
外加 ADR 0004 的「运行时文件身份」：单一解析路径、被检查字节 == 加载字节、
软链接安装与复制安装两种部署形态。

固定测试向量（``v1:96f15f0f8ddf2b33`` 等）是 ADR 0009 决定 2 要求的：标定工具
（OFF-16）必须复用同一规范化函数，否则 T6 比对必然失败。
"""

from __future__ import annotations

import hashlib
import shutil
from pathlib import Path

import pytest
import yaml

from robot_safecontrol_moveit import calibration_record as calib
from robot_safecontrol_moveit.calibration_record import (
    CALIB_ACCEPTANCE_NOT_EVALUATED,
    CALIB_FIELD_MISSING,
    CALIB_FRAME_MISMATCH,
    CALIB_ID_MISMATCH,
    CALIB_LEGACY_PARAM_PRESENT,
    CALIB_MATRIX_BOTTOM_ROW,
    CALIB_MATRIX_DETERMINANT,
    CALIB_MATRIX_NOT_FINITE,
    CALIB_MATRIX_NOT_ORTHONORMAL,
    CALIB_MATRIX_SHAPE,
    CALIB_NOT_CALIBRATED,
    CALIB_PROVENANCE_MISSING,
    CALIB_RECORD_MISSING,
    CALIB_RECORD_YAML_INVALID,
    CALIB_SCHEMA_UNSUPPORTED,
    CALIB_SECTION_UNKNOWN,
    CALIB_SOURCE_MISSING,
    CALIB_SOURCE_NOT_EXEMPT,
    CALIB_TRANSLATION_OUT_OF_ENVELOPE,
    CalibrationRecordError,
    LEVEL_ERROR,
    LEVEL_OK,
    LEVEL_WARN,
    canonical_bytes,
    compute_calibration_id,
    decide_startup,
    load_calibration_record,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
SHIPPED_RECORD = REPO_ROOT / "config" / "sensor_extrinsics.yaml"
DEMO_PROFILE = REPO_ROOT / "config" / "perception_runtime.yaml"
DEPLOY_PROFILE = REPO_ROOT / "config" / "perception_dual_sensor_real.yaml"

# 假定安装位姿（原 perception_runtime.yaml 的相机矩阵，现迁移到记录里）。
ASSUMED_MATRIX = [
    0.0, 0.0, 1.0, -0.9,
    -1.0, 0.0, 0.0, 0.4,
    0.0, -1.0, 0.0, 1.7,
    0.0, 0.0, 0.0, 1.0,
]
IDENTITY = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]

# 固定向量：由 camera 分节（假定几何、calibrated: false）规范化得到。
BASE_ID = "v1:96f15f0f8ddf2b33"
BASE_ENTRY = {
    "frame_from": "camera_color_optical_frame",
    "frame_to": "base_link",
    "matrix": ASSUMED_MATRIX,
    "calibrated": False,
    "method": "assumed",
    "operator": "assignment",
    "timestamp": None,
    "sensor_serial": None,
    "residual": None,
    "error_estimate": None,
}


def _entry(**overrides) -> dict:
    entry = dict(BASE_ENTRY)
    entry.update(overrides)
    return entry


def _record_yaml(camera: dict | None = None, lidar: dict | None = None,
                 schema_version: int = 1) -> str:
    document = {
        "schema_version": schema_version,
        "camera": camera if camera is not None else _entry(),
        "lidar": lidar if lidar is not None else _entry(
            frame_from="livox_frame", matrix=IDENTITY, method="placeholder"),
        "orbbec_launch": {"depth_width": 640, "depth_registration": True},
    }
    return yaml.safe_dump(document, allow_unicode=True, sort_keys=False)


def _load(tmp_path: Path, text: str, filename: str = "sensor_extrinsics.yaml"):
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True, exist_ok=True)
    path = share / "config" / filename
    path.write_text(text, encoding="utf-8")
    return load_calibration_record(share_dir=share, filename=filename)


def _calibrated_entry(**overrides) -> dict:
    entry = _entry(
        calibrated=True,
        method="target_based",
        timestamp="2026-09-14T02:00:00Z",
        sensor_serial="CP0A2B3C",
        residual={"translation_mm": 2.5, "rotation_deg": 0.3},
        error_estimate={"translation_mm": 4.0, "rotation_deg": 0.6},
    )
    entry.update(overrides)
    return entry


# ---------------------------------------------------------------------------
# 决定 2：两种身份
# ---------------------------------------------------------------------------
def test_canonical_bytes_fixed_vector():
    assert canonical_bytes("camera", BASE_ENTRY).decode() == (
        '{"calibrated":false,"error_estimate":null,'
        '"frame_from":"camera_color_optical_frame","frame_to":"base_link",'
        '"matrix":[0.0,0.0,1.0,-0.9,-1.0,0.0,0.0,0.4,0.0,-1.0,0.0,1.7,0.0,0.0,0.0,1.0],'
        '"method":"assumed","operator":"assignment","residual":null,'
        '"schema_version":1,"sensor_serial":null,"source":"camera","timestamp":null}'
    )


def test_calibration_id_fixed_vector():
    assert compute_calibration_id("camera", BASE_ENTRY) == BASE_ID


def test_int_and_float_matrix_share_identity():
    """YAML 里写 1 还是 1.0 不应改变身份（数值叶子统一按 float 规范化）。"""
    ints = _entry(matrix=[int(v) if float(v).is_integer() else v
                          for v in ASSUMED_MATRIX])
    assert compute_calibration_id("camera", ints) == BASE_ID


def test_source_is_part_of_identity():
    assert compute_calibration_id("lidar", BASE_ENTRY) != BASE_ID


def test_matrix_change_changes_identity():
    moved = list(ASSUMED_MATRIX)
    moved[3] = -0.91
    assert compute_calibration_id("camera", _entry(matrix=moved)) != BASE_ID


def test_calibrated_flag_changes_identity():
    """把 false 改成 true 必须改变身份，否则一次状态升级能沿着未变 id 溜过 T6。"""
    assert compute_calibration_id(
        "camera", _entry(calibrated=True)) != BASE_ID


def test_absent_and_null_are_equivalent():
    absent = {k: v for k, v in BASE_ENTRY.items()
              if k not in ("residual", "error_estimate")}
    assert compute_calibration_id("camera", absent) == BASE_ID


def test_declared_id_is_excluded_from_identity():
    """identity 不能自我引用：写上 calibration_id 不改变计算结果。"""
    with_id = _entry(calibration_id="v1:deadbeefdeadbeef")
    assert compute_calibration_id("camera", with_id) == BASE_ID


def test_non_finite_matrix_cannot_be_canonicalised():
    with pytest.raises(CalibrationRecordError) as excinfo:
        compute_calibration_id("camera", _entry(matrix=[float("nan")] * 16))
    assert excinfo.value.code == calib.CALIB_ID_UNAVAILABLE


def test_comment_change_moves_file_identity_only(tmp_path):
    """注释改动：文件身份变、记录身份不变（两身份回答两个不同问题）。"""
    plain = _record_yaml()
    commented = "# 手改一行注释\n" + plain
    first = _load(tmp_path, plain)
    second = _load(tmp_path, commented)
    assert first.file_sha256 == hashlib.sha256(plain.encode()).hexdigest()
    assert first.file_sha256 != second.file_sha256
    assert (first.source("camera").calibration_id
            == second.source("camera").calibration_id)


# ---------------------------------------------------------------------------
# 决定 1/4：schema、解析与读文件失败
# ---------------------------------------------------------------------------
def test_shipped_record_loads_and_reports_both_sources():
    record = load_calibration_record(explicit=SHIPPED_RECORD)
    assert record.read_ok, record.errors
    assert record.schema_version == calib.SCHEMA_VERSION
    camera = record.source("camera")
    assert camera.math_valid and camera.provenance_valid
    assert camera.calibrated is False
    assert camera.matrix is not None and camera.matrix.shape == (4, 4)
    # 迁移后 camera 分节必须仍是那份假定安装位姿（单相机行为不变）。
    assert camera.matrix.flatten().tolist() == ASSUMED_MATRIX
    assert camera.frame_from == "camera_color_optical_frame"
    assert camera.frame_to == "base_link"
    assert camera.admission_ready is False


def test_duplicate_key_is_rejected(tmp_path):
    text = _record_yaml() + "\ncamera:\n  frame_from: other\n"
    with pytest.raises(CalibrationRecordError) as excinfo:
        _load(tmp_path, text)
    assert excinfo.value.code == CALIB_RECORD_YAML_INVALID


def test_missing_file_is_reported(tmp_path):
    with pytest.raises(CalibrationRecordError) as excinfo:
        load_calibration_record(explicit=tmp_path / "nope.yaml")
    assert excinfo.value.code == CALIB_RECORD_MISSING


def test_symlink_loop_is_reported_as_unreadable(tmp_path):
    loop = tmp_path / "sensor_extrinsics.yaml"
    loop.symlink_to(loop.name)
    with pytest.raises(CalibrationRecordError) as excinfo:
        load_calibration_record(explicit=loop)
    assert excinfo.value.code == calib.CALIB_RECORD_UNREADABLE


def test_schema_version_mismatch_is_not_read_ok(tmp_path):
    record = _load(tmp_path, _record_yaml(schema_version=2))
    assert record.read_ok is False
    assert any(CALIB_SCHEMA_UNSUPPORTED in error for error in record.errors)


def test_boolean_is_not_accepted_as_schema_version_one(tmp_path):
    record = _load(tmp_path, _record_yaml(schema_version=True))
    assert record.read_ok is False
    assert any(CALIB_SCHEMA_UNSUPPORTED in error for error in record.errors)


def test_unknown_top_level_section_is_rejected(tmp_path):
    text = _record_yaml() + "\nCamera:\n  frame_from: typo\n"
    record = _load(tmp_path, text)
    assert record.read_ok is False
    assert any(CALIB_SECTION_UNKNOWN in error for error in record.errors)


def test_missing_source_section_is_reported(tmp_path):
    document = {"schema_version": 1, "camera": _entry()}
    record = _load(tmp_path, yaml.safe_dump(document, sort_keys=False))
    assert record.source("lidar").math_valid is False
    assert any(CALIB_SOURCE_MISSING in error
               for error in record.source("lidar").math_errors)


def test_enabling_a_missing_source_reports_only_source_missing(tmp_path):
    document = {"schema_version": 1, "camera": _calibrated_entry()}
    record = _load(tmp_path, yaml.safe_dump(document, sort_keys=False))
    decision = decide_startup(
        record,
        enabled_sources=["lidar"],
        frame_expectations={"lidar": ("livox_frame", "base_link")},
    )
    assert decision.start is False
    assert any(CALIB_SOURCE_MISSING in error for error in decision.errors)
    assert not any(CALIB_FRAME_MISMATCH in error for error in decision.errors)


# ---------------------------------------------------------------------------
# 决定 4：矩阵数学合法性（软件容差，不是标定精度）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("mutate,code", [
    (lambda m: m[:15], CALIB_MATRIX_SHAPE),
    (lambda m: m[:15] + [float("inf")], CALIB_MATRIX_NOT_FINITE),
    (lambda m: m[:15] + [float("nan")], CALIB_MATRIX_NOT_FINITE),
    (lambda m: [2.0] + m[1:], CALIB_MATRIX_NOT_ORTHONORMAL),
    (lambda m: [-1.0, 0.0, 0.0, 0.0] + m[4:], CALIB_MATRIX_DETERMINANT),
    (lambda m: m[:12] + [0.0, 0.0, 0.0, 2.0], CALIB_MATRIX_BOTTOM_ROW),
])
def test_math_validation_rejects_broken_matrix(tmp_path, mutate, code):
    record = _load(tmp_path, _record_yaml(camera=_entry(matrix=mutate(ASSUMED_MATRIX))))
    camera = record.source("camera")
    assert camera.math_valid is False
    assert any(code in error for error in camera.math_errors), camera.math_errors


def test_mirrored_rotation_is_rejected(tmp_path):
    """det = -1 是镜像：不能取绝对值放行，否则点云左右手性反转。"""
    matrix = list(ASSUMED_MATRIX)
    matrix[0], matrix[4], matrix[8] = -1.0, 0.0, 0.0  # 第一列取反
    record = _load(tmp_path, _record_yaml(camera=_entry(matrix=matrix)))
    assert record.source("camera").math_valid is False


def test_translation_envelope_is_enforced(tmp_path):
    matrix = list(ASSUMED_MATRIX)
    matrix[3] = 25.0
    record = _load(tmp_path, _record_yaml(camera=_entry(matrix=matrix)))
    camera = record.source("camera")
    assert camera.math_valid is False
    assert any(CALIB_TRANSLATION_OUT_OF_ENVELOPE in error
               for error in camera.math_errors)


def test_non_numeric_matrix_element_is_rejected(tmp_path):
    matrix = list(ASSUMED_MATRIX)
    matrix[5] = "1.0"
    record = _load(tmp_path, _record_yaml(camera=_entry(matrix=matrix)))
    assert record.source("camera").math_valid is False


def test_present_but_invalid_disabled_source_refuses_startup(tmp_path):
    """A corrupt section invalidates the shared record even when that source is disabled."""
    broken = list(IDENTITY)
    broken[0] = 2.0
    record = _load(tmp_path, _record_yaml(
        camera=_calibrated_entry(),
        lidar=_entry(frame_from="livox_frame", matrix=broken, method="placeholder"),
    ))
    decision = decide_startup(
        record,
        enabled_sources=["camera"],
        frame_expectations={
            "camera": ("camera_color_optical_frame", "base_link"),
        },
    )
    assert decision.start is False
    assert any(CALIB_MATRIX_NOT_ORTHONORMAL in error for error in decision.errors)


def test_missing_disabled_source_does_not_refuse_startup(tmp_path):
    """The schema defines a missing section as an absent source, not a corrupt record."""
    document = {"schema_version": 1, "camera": _calibrated_entry()}
    record = _load(tmp_path, yaml.safe_dump(document, sort_keys=False))
    decision = _decide(record)
    assert decision.start is True
    assert decision.level == LEVEL_OK


# ---------------------------------------------------------------------------
# 决定 3：准入与 provenance
# ---------------------------------------------------------------------------
def test_uncalibrated_record_is_not_admissible(tmp_path):
    camera = _load(tmp_path, _record_yaml()).source("camera")
    assert camera.math_valid and camera.provenance_valid
    assert camera.calibrated is False
    assert camera.admission_ready is False


def test_calibrated_record_with_full_provenance_is_admissible(tmp_path):
    record = _load(tmp_path, _record_yaml(camera=_calibrated_entry()))
    camera = record.source("camera")
    assert camera.calibrated is True
    assert camera.provenance_valid is True
    assert camera.admission_ready is True


@pytest.mark.parametrize("field", ["method", "operator", "timestamp",
                                   "sensor_serial"])
def test_calibrated_record_requires_provenance(tmp_path, field):
    record = _load(tmp_path, _record_yaml(camera=_calibrated_entry(**{field: None})))
    camera = record.source("camera")
    assert camera.provenance_valid is False
    assert camera.admission_ready is False
    assert any(CALIB_PROVENANCE_MISSING in error
               for error in camera.provenance_errors)


@pytest.mark.parametrize("field", ["method", "operator", "timestamp",
                                   "sensor_serial"])
def test_provenance_text_fields_reject_non_strings(tmp_path, field):
    record = _load(tmp_path, _record_yaml(camera=_calibrated_entry(**{field: 42})))
    camera = record.source("camera")
    assert camera.provenance_valid is False
    assert camera.admission_ready is False
    assert any(CALIB_PROVENANCE_MISSING in error
               for error in camera.provenance_errors)


def test_calibrated_record_needs_a_quality_estimate(tmp_path):
    record = _load(tmp_path, _record_yaml(camera=_calibrated_entry(
        residual=None, error_estimate=None)))
    assert record.source("camera").admission_ready is False


def test_calibrated_must_be_an_explicit_bool(tmp_path):
    record = _load(tmp_path, _record_yaml(camera=_entry(calibrated="true")))
    camera = record.source("camera")
    assert camera.calibrated is False
    assert any(CALIB_FIELD_MISSING in error for error in camera.provenance_errors)


def test_declared_id_mismatch_blocks_admission_but_not_startup(tmp_path):
    """手改了矩阵却没更新身份：可以启动（不 brick 生产），但不得准入。"""
    record = _load(tmp_path, _record_yaml(
        camera=_calibrated_entry(calibration_id="v1:0000000000000000")))
    camera = record.source("camera")
    assert camera.calibrated and camera.provenance_valid and camera.math_valid
    assert camera.identity_consistent is False
    assert camera.admission_ready is False

    decision = decide_startup(
        record, enabled_sources=["camera"],
        frame_expectations={"camera": ("camera_color_optical_frame", "base_link")})
    assert decision.start is True
    assert decision.level == LEVEL_WARN
    assert any(CALIB_ID_MISMATCH in warning for warning in decision.warnings)


@pytest.mark.parametrize("declared", [123, ""])
def test_malformed_declared_id_is_inconsistent_but_non_fatal(tmp_path, declared):
    record = _load(tmp_path, _record_yaml(
        camera=_calibrated_entry(calibration_id=declared)))
    camera = record.source("camera")
    assert camera.identity_consistent is False
    assert camera.admission_ready is False

    decision = _decide(record)
    assert decision.start is True
    assert decision.level == LEVEL_WARN
    assert any(CALIB_ID_MISMATCH in warning for warning in decision.warnings)


def test_matching_declared_id_is_accepted(tmp_path):
    record = _load(tmp_path, _record_yaml(camera=_calibrated_entry(
        calibration_id=BASE_ID.replace("96f15f0f8ddf2b33", "irrelevant"))))
    # 声明值必须等于该内容算出来的 id：这里先算出再写回。
    entry = _calibrated_entry()
    identity = compute_calibration_id("camera", entry)
    record = _load(tmp_path, _record_yaml(camera=dict(entry, calibration_id=identity)))
    assert record.source("camera").identity_consistent is True
    assert record.source("camera").admission_ready is True


# ---------------------------------------------------------------------------
# 决定 3/4：启动分级
# ---------------------------------------------------------------------------
def _decide(record, **kwargs):
    kwargs.setdefault("enabled_sources", ["camera"])
    kwargs.setdefault("frame_expectations",
                      {"camera": ("camera_color_optical_frame", "base_link")})
    return decide_startup(record, **kwargs)


def test_unreadable_record_refuses_startup():
    record = calib.CalibrationRecord(
        lookup_path="/nope.yaml", resolved_path="/nope.yaml", file_sha256=None,
        read_ok=False, schema_version=None, sources={},
        errors=(f"{CALIB_SCHEMA_UNSUPPORTED}: not this loader",))
    decision = _decide(record)
    assert decision.start is False
    assert decision.level == LEVEL_ERROR


def test_demo_profile_scenario_starts_with_warning(tmp_path):
    """未标定 + 被 profile 豁免：启动 + WARN + 不可准入（Q3(iii)/Q8 已定）。"""
    decision = _decide(_load(tmp_path, _record_yaml()), exempt_sources=["camera"])
    assert decision.start is True
    assert decision.level == LEVEL_WARN
    assert decision.sources["camera"].admitted is False
    assert any(CALIB_NOT_CALIBRATED in warning
               for warning in decision.sources["camera"].warnings)


def test_enabling_an_uncalibrated_source_refuses_startup(tmp_path):
    """启用一个源就要求它过标定门；豁免只认 profile 里的显式逐源声明。"""
    decision = _decide(_load(tmp_path, _record_yaml()), exempt_sources=["lidar"],
                       enabled_sources=["camera", "lidar"])
    assert decision.start is False
    assert decision.level == LEVEL_ERROR
    assert any(CALIB_SOURCE_NOT_EXEMPT in error for error in decision.errors)


def test_exemption_only_affects_startup_not_admission(tmp_path):
    record = _load(tmp_path, _record_yaml())
    decision = _decide(record, exempt_sources=["camera"])
    assert decision.start is True
    assert record.source("camera").admission_ready is False
    assert decision.sources["camera"].admitted is False


def test_calibrated_source_starts_ok_and_admitted(tmp_path):
    decision = _decide(_load(tmp_path, _record_yaml(camera=_calibrated_entry())))
    assert decision.start is True
    assert decision.level == LEVEL_OK
    assert decision.sources["camera"].admitted is True


def test_frame_mismatch_refuses_startup(tmp_path):
    decision = _decide(_load(tmp_path, _record_yaml()), exempt_sources=["camera"],
                       frame_expectations={
                           "camera": ("some_other_camera", "base_link")})
    assert decision.start is False
    assert any(CALIB_FRAME_MISMATCH in error for error in decision.errors)


def test_world_frame_mismatch_refuses_startup(tmp_path):
    decision = _decide(_load(tmp_path, _record_yaml()), exempt_sources=["camera"],
                       frame_expectations={
                           "camera": ("camera_color_optical_frame", "odom")})
    assert decision.start is False
    assert any(CALIB_FRAME_MISMATCH in error for error in decision.errors)


def test_empty_configured_source_frame_cannot_bypass_binding(tmp_path):
    decision = _decide(
        _load(tmp_path, _record_yaml()),
        exempt_sources=["camera"],
        frame_expectations={"camera": ("", "base_link")},
    )
    assert decision.start is False
    assert any(CALIB_FRAME_MISMATCH in error for error in decision.errors)


def test_enabled_source_generator_is_consumed_once(tmp_path):
    record = _load(tmp_path, _record_yaml(
        camera=_calibrated_entry(),
        lidar=_calibrated_entry(
            frame_from="livox_frame", matrix=IDENTITY,
            sensor_serial="LIVOX-TEST"),
    ))
    decision = decide_startup(
        record,
        enabled_sources=(name for name in ("camera", "lidar")),
        frame_expectations={
            "camera": ("camera_color_optical_frame", "base_link"),
            "lidar": ("livox_frame", "base_link"),
        },
    )
    assert decision.start is True
    assert set(decision.sources) == {"camera", "lidar"}


def test_legacy_extrinsics_override_refuses_startup(tmp_path):
    """策略 B：旧参数只留声明作探测器，非默认值即拒绝（不是静默失效）。"""
    decision = _decide(_load(tmp_path, _record_yaml()), exempt_sources=["camera"],
                       legacy_overrides={"use_tf": True})
    assert decision.start is False
    assert any(CALIB_LEGACY_PARAM_PRESENT in error for error in decision.errors)


def test_acceptance_is_reported_as_not_evaluated(tmp_path):
    """20mm/5° 实测验收未实现：诊断必须显式标未评估，既不通过也不降级。"""
    decision = _decide(_load(tmp_path, _record_yaml(camera=_calibrated_entry())))
    assert decision.level == LEVEL_OK
    assert any(CALIB_ACCEPTANCE_NOT_EVALUATED in note for note in decision.notes)


def test_no_enabled_source_is_a_warning_not_an_error(tmp_path):
    decision = _decide(_load(tmp_path, _record_yaml()), enabled_sources=[])
    assert decision.start is True
    assert decision.level == LEVEL_WARN


# ---------------------------------------------------------------------------
# ADR 0004：运行时文件身份与两种安装形态
# ---------------------------------------------------------------------------
def test_path_resolution_uses_injected_share_dir(tmp_path):
    share = tmp_path / "install" / "share"
    (share / "config").mkdir(parents=True)
    (share / "config" / "sensor_extrinsics.yaml").write_text(
        _record_yaml(), encoding="utf-8")
    lookup, resolved = calib.resolve_record_path(share_dir=share)
    assert lookup == str(share / "config" / "sensor_extrinsics.yaml")
    assert resolved == str((share / "config" / "sensor_extrinsics.yaml").resolve())


def test_path_resolution_prefers_ament_index(tmp_path, monkeypatch):
    """生产路径必须经 ament 查询，不允许硬编码源码树 fallback。"""
    share = tmp_path / "share_dir"
    (share / "config").mkdir(parents=True)
    (share / "config" / "sensor_extrinsics.yaml").write_text(
        _record_yaml(), encoding="utf-8")
    monkeypatch.setattr(calib, "_ament_share_dir", lambda package: share)
    lookup, resolved = calib.resolve_record_path()
    assert lookup.startswith(str(share))
    assert resolved.endswith("sensor_extrinsics.yaml")


def test_path_resolution_without_ament_is_an_error(monkeypatch):
    def boom(package):
        raise RuntimeError("package not found")
    monkeypatch.setattr(calib, "_ament_share_dir", boom)
    with pytest.raises(CalibrationRecordError) as excinfo:
        calib.resolve_record_path()
    assert excinfo.value.code == CALIB_RECORD_MISSING


def test_copy_install_reads_the_installed_copy(tmp_path):
    """复制安装（两个物理副本）：加载的是 install 侧那份，身份跟着被加载字节走。"""
    share = tmp_path / "install" / "share"
    (share / "config").mkdir(parents=True)
    installed = share / "config" / "sensor_extrinsics.yaml"
    installed.write_text(_record_yaml(), encoding="utf-8")
    source_side = tmp_path / "src" / "config"
    source_side.mkdir(parents=True)
    (source_side / "sensor_extrinsics.yaml").write_text(
        "schema_version: 1\n", encoding="utf-8")

    record = load_calibration_record(share_dir=share)
    assert record.resolved_path == str(installed)
    assert record.file_sha256 == hashlib.sha256(
        installed.read_bytes()).hexdigest()
    assert record.read_ok


def test_symlink_install_reports_lookup_vs_resolved(tmp_path):
    """软链接安装（本机现状）：lookup 在 share 下、resolved 指向源码树。"""
    source = tmp_path / "src" / "config"
    source.mkdir(parents=True)
    real = source / "sensor_extrinsics.yaml"
    real.write_text(_record_yaml(), encoding="utf-8")
    share = tmp_path / "install" / "share"
    (share / "config").mkdir(parents=True)
    link = share / "config" / "sensor_extrinsics.yaml"
    link.symlink_to(real)

    record = load_calibration_record(share_dir=share)
    assert record.lookup_path == str(link)
    assert record.resolved_path == str(real.resolve())
    assert record.lookup_path != record.resolved_path
    assert record.file_sha256 == hashlib.sha256(real.read_bytes()).hexdigest()


def test_checked_bytes_are_the_loaded_bytes(tmp_path):
    """同一份字节快照同时用于 hash、解析与校验（不许读第二遍）。"""
    text = _record_yaml()
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    path = share / "config" / "sensor_extrinsics.yaml"
    path.write_text(text, encoding="utf-8")
    record = load_calibration_record(share_dir=share)
    # 读完之后把文件换掉：快照身份不应跟着变。
    path.write_text(_record_yaml(camera=_calibrated_entry()), encoding="utf-8")
    assert record.file_sha256 == hashlib.sha256(text.encode()).hexdigest()
    assert record.source("camera").calibrated is False


# ---------------------------------------------------------------------------
# 配置回归守卫：部署 profile 的既有约定
# ---------------------------------------------------------------------------
def test_shipped_profiles_drop_legacy_extrinsics_keys():
    """外参只有一个可写来源：profile 不得再出现旧参数键（否则探测器会拒绝启动）。"""
    for profile in (DEMO_PROFILE, DEPLOY_PROFILE):
        params = yaml.safe_load(profile.read_text(encoding="utf-8"))
        bridge = params["perception_bridge"]["ros__parameters"]
        for legacy in ("use_tf", "camera_to_world_static", "lidar_to_world_static"):
            assert legacy not in bridge, f"{profile.name} still sets {legacy}"
        assert "calibration_record_path" in bridge


def test_demo_profile_exempts_camera_only():
    params = yaml.safe_load(DEMO_PROFILE.read_text(encoding="utf-8"))
    exempt = params["perception_bridge"]["ros__parameters"][
        "allow_uncalibrated_debug"]
    assert exempt == ["camera"]


def test_deploy_profile_declares_no_exemption():
    params = yaml.safe_load(DEPLOY_PROFILE.read_text(encoding="utf-8"))
    bridge = params["perception_bridge"]["ros__parameters"]
    assert "allow_uncalibrated_debug" not in bridge


def test_runtime_snapshot_matches_shipped_record(tmp_path):
    """模拟启动：把仓库里的记录复制进临时 share，断言诊断条目可断言。"""
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    shutil.copy(SHIPPED_RECORD, share / "config" / "sensor_extrinsics.yaml")
    record = load_calibration_record(share_dir=share)
    decision = _decide(record, exempt_sources=["camera"])
    entries = calib.calibration_status_entries(
        record, decision, node_identity="perception_bridge#1",
        time_domains={"camera": None})
    names = [entry["name"] for entry in entries]
    assert names == [
        "perception_bridge:record", "perception_bridge:camera",
        "perception_bridge:lidar"]
    record_entry = entries[0]
    assert record_entry["values"]["file_sha256"] == record.file_sha256
    assert record_entry["values"]["resolved_path"] == record.resolved_path
    camera_entry = entries[1]
    assert camera_entry["level"] == LEVEL_WARN
    assert camera_entry["values"]["admission_ready"] == "false"
    assert camera_entry["values"]["exempt"] == "true"
    assert camera_entry["values"]["time_domain"] == "unknown"
    assert camera_entry["values"]["time_domain_source"] == "unknown"
    # 未启用的源也要上报身份，但必须能与实际启用的源区分开。
    lidar_entry = entries[2]
    assert lidar_entry["values"]["enabled"] == "false"
    assert lidar_entry["values"]["exempt"] == "false"
    assert lidar_entry["values"]["calibrated"] == "false"


def test_diagnostic_reports_exemption_declared_for_disabled_source(tmp_path):
    record = _load(tmp_path, _record_yaml())
    decision = _decide(record, exempt_sources=["camera", "lidar"])
    entries = calib.calibration_status_entries(
        record, decision, node_identity="perception_bridge#1")
    lidar = next(entry for entry in entries
                 if entry["name"] == "perception_bridge:lidar")
    assert lidar["values"]["enabled"] == "false"
    assert lidar["values"]["exempt"] == "true"
