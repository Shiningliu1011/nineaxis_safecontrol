from __future__ import annotations

import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import sqlite3
import subprocess
import sys
import zipfile

from .static_experiment_config import content_hash
from .static_experiment_recording import FORMAT_VERSION, write_json


def source_identity(resource_root: Path) -> dict:
    import robot_static_fcl

    paths = [
        *Path(__file__).parent.glob("*.py"),
        *(resource_root / "portable_oscbf/work").glob("*.py"),
        *(resource_root / "portable_oscbf/vendor/dpax/dpax").glob("*.py"),
        *(resource_root / "portable_oscbf/native").glob("*.cpp"),
        Path(robot_static_fcl.__file__),
    ]
    return {str(path.resolve()): content_hash(path) for path in sorted(paths)}


def environment_identity() -> dict:
    cpu = json.loads(subprocess.run(
        ["lscpu", "--json"], check=True, capture_output=True, text=True,
    ).stdout)
    packages = subprocess.run(
        ["dpkg-query", "-W", "-f=${Package}\t${Version}\n",
         "libfcl*", "libccd*", "libeigen3*", "ros-humble-rclpy", "ros-humble-rmw-*"],
        check=True, capture_output=True, text=True,
    ).stdout
    distributions = sorted(
        {f"{item.metadata['Name']}=={item.version}" for item in importlib.metadata.distributions()},
    )
    identity = {
        "python": sys.version, "executable": sys.executable,
        "sqlite_version": sqlite3.sqlite_version,
        "platform": platform.platform(), "machine": platform.machine(),
        "hostname": platform.node(), "cpu": cpu,
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "memory_bytes": os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE"),
        "packages": distributions, "system_packages": packages,
        "runtime_environment": {name: os.environ.get(name) for name in (
            "RMW_IMPLEMENTATION", "JAX_PLATFORMS", "XLA_FLAGS", "OMP_NUM_THREADS",
            "OPENBLAS_NUM_THREADS", "OPENBLAS_CORETYPE", "MKL_NUM_THREADS",
        )},
    }
    # CPU 当前频率随负载变化，只用于来源记录。
    stable_cpu = [
        row for row in cpu["lscpu"]
        if row["field"] not in ("CPU(s) scaling MHz:", "CPU MHz:", "BogoMIPS:")
    ]
    stable = {**identity, "cpu": stable_cpu}
    identity["sha256"] = hashlib.sha256(
        json.dumps(stable, sort_keys=True, allow_nan=False).encode(),
    ).hexdigest()
    return identity


def snapshot_sources(resource_root: Path, manifests: list[dict], directory: Path) -> dict:
    root, directory = resource_root.resolve(strict=True), directory.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    files = set()
    package_marker = (
        root.parent / "ament_index/resource_index/packages/robot_safecontrol_moveit"
        if root.parent.name == "share" and root.name == "robot_safecontrol_moveit"
        else root / "resource/robot_safecontrol_moveit"
    )
    for manifest in manifests:
        for value in manifest["resources"].values():
            source = Path(value["path"]).resolve(strict=True)
            if content_hash(source) != value["sha256"]:
                raise ValueError(f"资源身份变化：{source}")
            files.add(source)
        for filename, expected in manifest["source_sha256"].items():
            source = Path(filename).resolve(strict=True)
            if content_hash(source) != expected:
                raise ValueError(f"源文件身份变化：{source}")
            files.add(source)
    files.update((root / "portable_oscbf/config").rglob("*.yaml"))
    files.update((root / "portable_oscbf/config").rglob("*.json"))
    files.update([
        root / "setup.py", root / "setup.cfg", root / "package.xml",
        package_marker,
        root / "portable_oscbf/requirements.txt", root / "portable_oscbf/requirements-geometry.txt",
        root / "portable_oscbf/data/ik_input.mat",
        root / "requirements-static-report.txt", root / "requirements-static-recording.txt",
        root / "scripts/static_obstacle_experiment.py",
        root / "docs/static_obstacle_experiments.md",
        root / "docs/static_experiment_storage.md",
        root / "docs/specs/static-obstacle-experiments.md",
    ])
    archive = directory / "sources.zip"
    records = {}
    logical_paths = {
        path.resolve(): path.relative_to(root).as_posix()
        for name in ("portable_oscbf", "models", "config", "src", "scripts", "docs")
        for path in (root / name).rglob("*") if path.is_file()
    }
    logical_paths.update({
        (root / name).resolve(): name
        for name in ("setup.py", "setup.cfg", "package.xml", "requirements-static-report.txt",
                     "requirements-static-recording.txt",
                     "resource/robot_safecontrol_moveit")
    })
    logical_paths[package_marker.resolve(strict=True)] = "resource/robot_safecontrol_moveit"
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as output:
        for path in sorted({item.resolve(strict=True) for item in files}):
            if path.is_relative_to(root):
                relative = path.relative_to(root).as_posix()
            elif path in logical_paths:
                relative = logical_paths[path]
            elif path.parent.name == "robot_safecontrol_moveit" and path.suffix == ".py":
                relative = f"src/robot_safecontrol_moveit/{path.name}"
            else:
                relative = f"external/{content_hash(path)}/{path.name}"
            output.write(path, relative)
            records[relative] = {"original_path": str(path), "sha256": content_hash(path)}
    environment = environment_identity()
    write_json(directory / "environment.json", environment)
    report = {
        "schema_version": 1, "archive": archive.name, "archive_sha256": content_hash(archive),
        "files": records, "environment_sha256": content_hash(directory / "environment.json"),
        "environment_identity": environment["sha256"],
    }
    write_json(directory / "snapshot.json", report)
    verify_snapshot(directory)
    return report


def verify_snapshot(directory: Path) -> dict:
    directory = Path(directory)
    report = json.loads((directory / "snapshot.json").read_text())
    archive = directory / report["archive"]
    if content_hash(archive) != report["archive_sha256"]:
        raise ValueError("源文件归档身份不一致")
    if content_hash(directory / "environment.json") != report["environment_sha256"]:
        raise ValueError("软件环境记录身份不一致")
    with zipfile.ZipFile(archive) as source:
        if set(source.namelist()) != set(report["files"]) or source.testzip() is not None:
            raise ValueError("源文件归档不完整")
        for name, record in report["files"].items():
            if hashlib.sha256(source.read(name)).hexdigest() != record["sha256"]:
                raise ValueError(f"归档内容身份不一致：{name}")
    return report


def relocate_manifest(manifest: dict, resource_root: Path) -> dict:
    if "resource_root" not in manifest:
        return manifest
    original = Path(manifest["resource_root"])
    destination = Path(resource_root).resolve(strict=True)

    def relocate(value):
        if isinstance(value, str):
            path = Path(value)
            if path.is_absolute() and path.is_relative_to(original):
                return str(destination / path.relative_to(original))
        if isinstance(value, list):
            return [relocate(item) for item in value]
        if isinstance(value, dict):
            return {relocate(key): relocate(item) for key, item in value.items()}
        return value

    return relocate(manifest)


def recording_hashes(directory: Path) -> dict:
    records = {}
    for role in ("plant", "controller"):
        database = directory / f"{role}.sqlite3"
        wal = directory / f"{role}.sqlite3-wal"
        if wal.exists() and wal.stat().st_size:
            raise ValueError(f"实验数据库尚未完成 checkpoint：{database}")
        if database.exists():
            records[role] = content_hash(database)
    return records


def validate_storage_migration(directory: Path, identities: dict) -> str:
    path = directory / "storage_migration.json"
    migration = json.loads(path.read_text())
    if migration["schema_version"] != 1 or migration["recording_format"] != FORMAT_VERSION:
        raise ValueError(f"不支持的转换说明版本：{path}")
    records = identities["records_sha256"]
    databases = migration["databases"]
    if set(databases) != set(records):
        raise ValueError(f"转换数据库集合不一致：{directory}")
    for role, actual in records.items():
        if databases[role]["database_sha256"] != actual:
            raise ValueError(f"转换数据库身份不一致：{directory / (role + '.sqlite3')}")
        if databases[role]["source_sha256"] != migration["source_files_sha256"][f"{role}.sqlite3"]:
            raise ValueError(f"转换来源数据库身份不一致：{role}")
    for filename, key in (
        ("manifest.json", "manifest_sha256"), ("result.json", "result_sha256"),
        ("path.npz", "path_sha256"),
    ):
        if migration["source_files_sha256"][filename] != identities[key]:
            raise ValueError(f"转换保留文件身份不一致：{directory / filename}")
    if migration["source_manifest"] != json.loads((directory / "manifest.json").read_text()):
        raise ValueError(f"转换来源 manifest 不一致：{directory}")
    return content_hash(path)


def validate_run_report(directory: Path) -> tuple[dict, dict]:
    manifest_path, report_path = directory / "manifest.json", directory / "feedback_report.json"
    manifest = json.loads(manifest_path.read_text())
    report = json.loads(report_path.read_text())
    identities = {
        "manifest_sha256": content_hash(manifest_path),
        "result_sha256": content_hash(directory / "result.json"),
        "config_sha256": manifest["config_sha256"],
        "path_sha256": content_hash(directory / "path.npz"),
        "records_sha256": recording_hashes(directory),
    }
    migration = directory / "storage_migration.json"
    if migration.exists() or "storage_migration_sha256" in report:
        identities["storage_migration_sha256"] = validate_storage_migration(directory, identities)
    if (any(report.get(key) != value for key, value in identities.items())
            or identities["path_sha256"] != manifest["path_sha256"]):
        raise ValueError(f"反馈报告与原始运行身份不一致：{directory}")
    if report["verdict"] == "pass" and set(identities["records_sha256"]) != {"plant", "controller"}:
        raise ValueError(f"通过报告缺少原始数据库：{directory}")
    for filename, expected in report.get("derived_data_sha256", {}).items():
        if content_hash(directory / filename) != expected:
            raise ValueError(f"反馈派生数据身份不一致：{directory / filename}")
    return manifest, report
