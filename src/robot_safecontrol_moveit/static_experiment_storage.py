from __future__ import annotations

from collections import Counter
from contextlib import closing
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import time

import msgpack
import numpy as np

from .static_experiment_config import content_hash
from .static_experiment_provenance import recording_hashes
from .static_experiment_recording import FORMAT_VERSION, RunRecorder, iter_records, write_json


DERIVED_FILES = ("feedback_metrics.npz", "clearance_metrics.npz", "feedback_arc_metrics.npz")


def record_identity(path: Path) -> dict:
    digest, counts = hashlib.sha256(), Counter()
    for row in iter_records(path):
        digest.update(msgpack.packb(row, use_bin_type=True, use_single_float=False))
        counts[row["stream"]] += 1
    return {"sha256": digest.hexdigest(), "counts": dict(counts)}


def compact_database(source: Path, destination: Path) -> dict:
    source = source.resolve(strict=True)
    with closing(sqlite3.connect(source.as_uri() + "?mode=ro", uri=True)) as connection:
        if connection.execute("PRAGMA user_version").fetchone()[0] != 0:
            raise ValueError("转换入口要求旧 JSON 数据库")
    wal = Path(str(source) + "-wal")
    if wal.exists() and wal.stat().st_size:
        raise ValueError("转换前必须完成原始数据库 checkpoint")
    original_hash = content_hash(source)
    expected, counts, durations = hashlib.sha256(), Counter(), []
    started = time.monotonic()
    with RunRecorder(destination) as recorder:
        for row in iter_records(source):
            expected.update(msgpack.packb(row, use_bin_type=True, use_single_float=False))
            counts[row["stream"]] += 1
            durations.append(recorder.append(
                row["stream"], sequence=row["sequence"], monotonic_s=row["monotonic_s"],
                simulation_s=row["simulation_s"], ros_time_ns=row["ros_time_ns"], values=row["values"],
            ))
    writing_s = time.monotonic() - started
    started = time.monotonic()
    actual = record_identity(destination)
    reading_s = time.monotonic() - started
    if actual != {"sha256": expected.hexdigest(), "counts": dict(counts)}:
        raise ValueError("转换记录内容或数量不一致")
    if content_hash(source) != original_hash:
        raise ValueError("转换期间原始记录发生变化")
    return {
        "source_sha256": original_hash, "database_sha256": content_hash(destination),
        "records": actual, "source_bytes": source.stat().st_size,
        "compact_bytes": destination.stat().st_size,
        "conversion_s": writing_s, "verification_read_s": reading_s,
        "append_s": {
            "mean": float(np.mean(durations)), "p999": float(np.quantile(durations, .999)),
            "maximum": float(max(durations)),
        } if durations else None,
    }


def compact_run(source: Path, destination: Path) -> dict:
    source, destination = source.resolve(strict=True), destination.resolve()
    if destination.is_relative_to(source):
        raise ValueError("转换目录必须位于原始运行目录之外")
    source_report = json.loads((source / "feedback_report.json").read_text())
    manifest = json.loads((source / "manifest.json").read_text())
    identities = {
        "records_sha256": recording_hashes(source),
        "path_sha256": content_hash(source / "path.npz"),
        "config_sha256": manifest["config_sha256"],
    }
    if any(source_report.get(key) != value for key, value in identities.items()):
        raise ValueError("来源报告与数据库、路径或配置身份不一致")
    optional_identities = {
        "manifest_sha256": content_hash(source / "manifest.json"),
        "result_sha256": content_hash(source / "result.json"),
    }
    for key, expected in optional_identities.items():
        if key in source_report and source_report[key] != expected:
            raise ValueError(f"来源报告身份不一致：{key}")
    for filename, expected in source_report.get("derived_data_sha256", {}).items():
        if content_hash(source / filename) != expected:
            raise ValueError(f"来源派生数组身份不一致：{filename}")
    original = {path.relative_to(source).as_posix(): content_hash(path)
                for path in source.rglob("*") if path.is_file()
                and not path.name.endswith(("-wal", "-shm"))}
    omitted = (*DERIVED_FILES, "plant.sqlite3", "controller.sqlite3",
               "plant.sqlite3-wal", "plant.sqlite3-shm",
               "controller.sqlite3-wal", "controller.sqlite3-shm")
    shutil.copytree(source, destination, ignore=shutil.ignore_patterns(*omitted))
    reports = destination / "source_reports"
    reports.mkdir()
    for filename in ("feedback_report.json", "feedback_report.md", "clearance_report.json"):
        path = destination / filename
        if path.exists():
            path.rename(reports / filename)
    records = {}
    for role in ("plant", "controller"):
        database = source / f"{role}.sqlite3"
        if database.exists():
            records[role] = compact_database(database, destination / database.name)
    for filename, expected in original.items():
        if content_hash(source / filename) != expected:
            raise ValueError(f"转换期间来源文件变化：{filename}")
    report = {
        "schema_version": 1, "recording_format": FORMAT_VERSION,
        "source_run": str(source), "source_files_sha256": original,
        "source_manifest": json.loads((source / "manifest.json").read_text()),
        "source_report_missing_identity_fields": [
            key for key in optional_identities if key not in source_report
        ],
        "databases": records, "derived_data_policy": "on_demand",
    }
    write_json(destination / "storage_migration.json", report)
    return report
