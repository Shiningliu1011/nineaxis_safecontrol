from __future__ import annotations

import argparse
from collections import Counter
from contextlib import closing
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import zlib

import msgpack
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.static_experiment_capacity import CapacityBudget, assess_capacity
from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_recording import iter_records, write_json
from robot_safecontrol_moveit.static_formal_experiments import experiment_rows, SCENARIOS


def inspect_database(path: Path) -> dict:
    before = content_hash(path)
    streams, codecs, compressed = {}, {}, Counter()
    with closing(sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)) as connection:
        format_version = connection.execute("PRAGMA user_version").fetchone()[0]
        sizes = dict(connection.execute(
            "SELECT stream, sum(length(payload)) FROM samples GROUP BY stream",
        ))
    for row in iter_records(path):
        stream = row["stream"]
        raw = msgpack.packb(row, use_bin_type=True, use_single_float=False)
        payload = msgpack.packb(row["values"], use_bin_type=True, use_single_float=False)
        record = streams.setdefault(stream, {
            "count": 0, "first_monotonic_s": row["monotonic_s"],
            "last_monotonic_s": row["monotonic_s"],
            "maximum_payload_bytes": 0, "messagepack_bytes": 0,
            "maximum_trials": 0, "maximum_geometry_pairs": 0,
        })
        record["count"] += 1
        record["last_monotonic_s"] = row["monotonic_s"]
        record["maximum_payload_bytes"] = max(record["maximum_payload_bytes"], len(payload))
        record["messagepack_bytes"] += len(raw)
        record["maximum_trials"] = max(record["maximum_trials"], len(row["values"].get("trials", [])))
        record["maximum_geometry_pairs"] = max(
            record["maximum_geometry_pairs"], len(row["values"].get("geometry", [])),
        )
        if stream not in codecs:
            codecs[stream] = (
                zlib.compressobj(level=6), zlib.decompressobj(),
                hashlib.sha256(), hashlib.sha256(),
            )
        compressor, decoder, original_digest, decoded_digest = codecs[stream]
        original_digest.update(raw)
        encoded = compressor.compress(raw)
        compressed[stream] += len(encoded)
        decoded_digest.update(decoder.decompress(encoded))
    for stream, (compressor, decoder, original_digest, decoded_digest) in codecs.items():
        encoded = compressor.flush()
        compressed[stream] += len(encoded)
        decoded_digest.update(decoder.decompress(encoded) + decoder.flush())
        if not decoder.eof or decoded_digest.digest() != original_digest.digest():
            raise ValueError(f"完整流无损编码核查失败：{path}:{stream}")
        streams[stream]["stream_zlib6_bytes"] = compressed[stream]
        streams[stream]["stream_roundtrip_sha256"] = original_digest.hexdigest()
        streams[stream]["stored_payload_bytes"] = sizes[stream]
    if content_hash(path) != before:
        raise ValueError(f"原始数据库身份变化：{path}")
    return {
        "path": str(path), "sha256": before, "format_version": format_version,
        "file_bytes": path.stat().st_size, "streams": streams,
        "stream_zlib6_bytes": sum(compressed.values()),
    }


def audit_capacity(catalog_path: Path, validation_path: Path, output: Path, *,
                   resource_root: Path, retry_runs: int, reserve_bytes: int) -> dict:
    catalog = json.loads(catalog_path.read_text())
    validation = json.loads(validation_path.read_text())
    if (catalog["kind"] != "development" or len(catalog["initial_positions"]) != 3
            or catalog["feed_factors"] != [.5, 1., 1.5]
            or {row["id"] for row in catalog["scenarios"]} != set(SCENARIOS)):
        raise ValueError("开发场景清单缺少固定矩阵所需组合")
    if validation["status"] != "pass":
        raise ValueError("紧凑存储验证尚未通过")
    output.mkdir(parents=True, exist_ok=False)
    sources = {str(catalog_path): content_hash(catalog_path),
               str(validation_path): content_hash(validation_path)}
    configurations = {}
    for entry in catalog["scenarios"]:
        path = catalog_path.parent / entry["config"]
        if content_hash(path) != entry["config_sha256"]:
            raise ValueError(f"场景配置身份变化：{path}")
        values = yaml.safe_load(path.read_text())
        sources[str(path)] = content_hash(path)
        configurations[entry["id"]] = {
            "timeout_s": values["timeout_s"], "settle_s": values["settle_s"],
            "period_s": values["control"]["period_s"],
            "nominal_feed_multiplier": values["control"]["nominal_feed_multiplier"],
            "obstacle_count": len(values["obstacles"]),
        }
    samples = []
    names = ("s0-joint-full-geometry-01", "s2-joint-development-01",
             "s4-joint-development-10", "s5-joint-development-01")
    for name in names:
        run = validation_path.parent / name
        migration = json.loads((run / "storage_migration.json").read_text())
        original = Path(migration["source_run"])
        for filename, expected in migration["source_files_sha256"].items():
            if content_hash(original / filename) != expected:
                raise ValueError(f"原始运行身份变化：{original / filename}")
        databases = {}
        for role in ("plant", "controller"):
            path = run / f"{role}.sqlite3"
            expected = migration["databases"][role]["database_sha256"]
            if content_hash(path) != expected:
                raise ValueError(f"紧凑数据库身份变化：{path}")
            databases[role] = inspect_database(path)
            sources[str(path)] = expected
        source_manifest = migration["source_manifest"]
        sources[str(run / "storage_migration.json")] = content_hash(run / "storage_migration.json")
        files = [path for path in run.rglob("*") if path.is_file()]
        derived = sum((original / filename).stat().st_size for filename in (
            "feedback_metrics.npz", "feedback_arc_metrics.npz", "clearance_metrics.npz",
        ) if (original / filename).exists())
        feedback = databases["plant"]["streams"]["feedback"]
        duration = feedback["count"] / 500.
        samples.append({
            "name": name, "scenario": source_manifest["config"]["scenario"],
            "solver_mode": source_manifest["config"]["control"]["solver_mode"],
            "recovery_mode": source_manifest["config"]["control"]["recovery"]["mode"],
            "source_run": str(original), "source_files_sha256": migration["source_files_sha256"],
            "compact_directory_bytes": sum(path.stat().st_size for path in files),
            "compact_allocated_bytes": sum(path.stat().st_blocks * 512 for path in files),
            "derived_bytes": derived, "feedback_duration_s": duration,
            "databases": databases,
        })
        write_json(output / f"{name}.json", samples[-1])
        print(f"已核查真实记录：{name}", flush=True)
    maximum_rate = max(sample["compact_directory_bytes"] / sample["feedback_duration_s"]
                       for sample in samples)
    maximum_run = max(sample["compact_directory_bytes"] for sample in samples)
    maximum_derived_rate = max(sample["derived_bytes"] / sample["feedback_duration_s"]
                               for sample in samples)
    rows = []
    for row in experiment_rows():
        config = configurations[row["scene"]]
        # plant 在 controller 启动期间运行；两个 worker 各有 20+5 秒退出等待。
        lifetime = config["timeout_s"] + 180 + config["settle_s"] + 50 + 1
        rows.append({
            **row, **config, "plant_lifetime_allowance_s": lifetime,
            "feedback_count_allowance": math.ceil(lifetime * 500),
            "controller_timer_callback_allowance": math.ceil(lifetime * 500),
            "maximum_trials_per_candidate": 9,
            "time_scaled_sample_bytes": math.ceil(maximum_rate * lifetime),
        })
    largest_lifetime = max(row["plant_lifetime_allowance_s"] for row in rows)
    transient = max(
        validation["peak_validation_allocated_bytes"],
        math.ceil(validation["peak_validation_allocated_bytes"] * largest_lifetime
                  / min(sample["feedback_duration_s"] for sample in samples)),
    )
    budget = CapacityBudget(
        planned_runs=len(rows), retry_runs=retry_runs,
        retained_run_bytes=max(row["time_scaled_sample_bytes"] for row in rows),
        transient_run_bytes=transient,
        derived_workspace_bytes=math.ceil(maximum_derived_rate * largest_lifetime),
        provenance_bytes=100_000_000, reserve_bytes=reserve_bytes,
    )
    available = shutil.disk_usage(output).free
    report = {
        **assess_capacity(budget, available_bytes=available),
        "measured_at": datetime.now(timezone.utc).isoformat(),
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "command": sys.argv, "python": sys.version, "sqlite_version": sqlite3.sqlite_version,
        "resource_root": str(resource_root), "output": str(output),
        "formal_collection": "stopped", "evidence_sha256": sources,
        "source_sha256": {
            str(path): content_hash(path) for path in (
                Path(__file__).resolve(),
                Path(__file__).resolve().parents[1] / "src/robot_safecontrol_moveit/static_experiment_capacity.py",
                resource_root / "src/robot_safecontrol_moveit/static_experiment_nodes.py",
                resource_root / "src/robot_safecontrol_moveit/static_experiment.py",
                resource_root / "src/robot_safecontrol_moveit/static_experiment_recording.py",
                resource_root / "portable_oscbf/work/static_control.py",
            )
        },
        "matrix_runs": len(rows), "rows": rows, "samples": samples,
        "sample_projections": {
            "direct_bytes": maximum_run * len(rows),
            "inverse_feed_bytes": math.ceil(sum(maximum_run / row["feed_factor"] for row in rows)),
            "timeout_scaled_bytes": sum(row["time_scaled_sample_bytes"] for row in rows),
            "maximum_observed_bytes_per_feedback_second": maximum_rate,
            "compression_study": "完整流 zlib6 仅测量编码潜力；没有替换记录格式或逐条提交语义",
        },
        "unproven_conditions": [
            "真实样本覆盖 S0/S2/S4/S5 的 joint；S1/S3、separated、SNS 与其他初态进给尚缺少记录大小证据",
            "payload 最大值和完整流压缩率为观测值，尚未证明完整固定字段及最多九次 trial 的编码上界",
            "候选拒绝后可以在后续 timer 再次计算，候选数量上界不能使用共同控制周期倒数",
            "wal_autocheckpoint=0；WAL 体积与逐条事务更新页数相关，观测峰值尚未覆盖最长运行",
            "时间留量包含 180 秒 controller 启动、50 秒退出、settle 和 1 秒轮询留量；调度延迟尚无执行界限",
            "派生数组与 provenance 采用独立工作空间留量，尚未验证全部场景最坏编码大小",
            f"预算明确保留 {retry_runs} 次完整失败重试；追加重试需要重新计算",
        ],
        "mounted_filesystems": json.loads(subprocess.run(
            ["findmnt", "--json", "--bytes", "--output", "TARGET,SOURCE,FSTYPE,AVAIL,SIZE"],
            check=True, capture_output=True, text=True,
        ).stdout),
        "disk_usage": shutil.disk_usage(output)._asdict(),
    }
    write_json(output / "capacity_plan.json", report)
    print(json.dumps({key: report[key] for key in (
        "status", "is_validated_capacity_upper_bound", "required_free_bytes",
        "available_bytes", "shortfall_bytes",
    )}, ensure_ascii=False))
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="核查固定静态实验矩阵的容量证据")
    parser.add_argument("--catalog", required=True, type=Path)
    parser.add_argument("--validation", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--resource-root", required=True, type=Path)
    parser.add_argument("--retry-runs", type=int, required=True)
    parser.add_argument("--reserve-bytes", type=int, default=10_000_000_000)
    args = parser.parse_args()
    report = audit_capacity(
        args.catalog.resolve(strict=True), args.validation.resolve(strict=True),
        args.output.resolve(), resource_root=args.resource_root.resolve(strict=True),
        retry_runs=args.retry_runs, reserve_bytes=args.reserve_bytes,
    )
    return 0 if report["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
