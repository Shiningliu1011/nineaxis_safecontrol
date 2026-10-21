import argparse
from contextlib import closing
import json
from itertools import islice
import os
from pathlib import Path
import resource
import signal
import sqlite3
import struct
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_evaluation import evaluate_run, evaluation_samples
from robot_safecontrol_moveit.static_experiment_provenance import environment_identity, validate_run_report
from robot_safecontrol_moveit.static_experiment_recording import (
    RunRecorder, finalize_recording, iter_records, read_record, write_json,
)
from robot_safecontrol_moveit.static_experiment_storage import compact_run, record_identity


DEFAULT_RUNS = ("s0-joint-full-geometry-01", "s2-joint-development-01",
                "s4-joint-development-10", "s5-joint-development-01")
IDENTITY_FIELDS = {
    "manifest_sha256", "records_sha256", "evaluation_sources_sha256", "evaluator_sha256",
    "derived_data_sha256", "derived_data_policy", "storage_migration_sha256",
}


def compare_values(first, second, *, exact=True, path=""):
    if type(first) is not type(second):
        raise AssertionError(f"字段类型变化：{path}")
    if isinstance(first, dict):
        if first.keys() != second.keys():
            raise AssertionError(f"字段集合变化：{path}")
        for key in first:
            compare_values(first[key], second[key], exact=exact, path=f"{path}/{key}")
    elif isinstance(first, list):
        if len(first) != len(second):
            raise AssertionError(f"记录数量变化：{path}")
        for index, (a, b) in enumerate(zip(first, second)):
            compare_values(a, b, exact=exact, path=f"{path}/{index}")
    elif isinstance(first, float):
        equal = (struct.pack("!d", first) == struct.pack("!d", second) if exact
                 else np.isclose(first, second, atol=1e-12, rtol=1e-12))
        if not equal:
            raise AssertionError(f"数值变化：{path}: {first!r}, {second!r}")
    elif first != second:
        raise AssertionError(f"内容变化：{path}")


def measurement_fields(report):
    return {key: value for key, value in report.items() if key not in IDENTITY_FIELDS}


def legacy_write(source, destination):
    durations = []
    with closing(sqlite3.connect(destination, isolation_level=None)) as connection:
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA synchronous=NORMAL")
        connection.execute("PRAGMA wal_autocheckpoint=0")
        connection.execute(
            "CREATE TABLE samples (sequence INTEGER PRIMARY KEY, stream TEXT NOT NULL, "
            "monotonic_s REAL NOT NULL, simulation_s REAL, ros_time_ns INTEGER, payload TEXT NOT NULL)"
        )
        for row in iter_records(source):
            started = time.monotonic()
            payload = json.dumps(row["values"], ensure_ascii=False, separators=(",", ":"), allow_nan=False)
            connection.execute("INSERT INTO samples VALUES (?, ?, ?, ?, ?, ?)", (
                row["sequence"], row["stream"], row["monotonic_s"], row["simulation_s"],
                row["ros_time_ns"], payload,
            ))
            durations.append(time.monotonic() - started)
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    return {"mean": float(np.mean(durations)), "p999": float(np.quantile(durations, .999)),
            "maximum": float(max(durations))}


def interrupted_writer(source, destination):
    recorder = RunRecorder(destination)
    for index, row in enumerate(iter_records(source)):
        recorder.append(
            row["stream"], sequence=row["sequence"], monotonic_s=row["monotonic_s"],
            simulation_s=row["simulation_s"], ros_time_ns=row["ros_time_ns"], values=row["values"],
        )
        if index == 255:
            print(json.dumps({"count": index + 1, "sequence": row["sequence"]}), flush=True)
            sys.stdin.read(1)
            raise RuntimeError("中断检查等待 SIGKILL")
    raise ValueError("真实来源记录数量不足")


def check_interruption(source, target):
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), "--interrupt-source", str(source),
         "--output", str(target)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
    )
    try:
        acknowledgement = json.loads(process.stdout.readline())
        process.kill()
        if process.wait(timeout=30) != -signal.SIGKILL:
            raise AssertionError("未获得规定的 SIGKILL 退出状态")
        recovered = list(iter_records(target))
        if len(recovered) != acknowledgement["count"]:
            raise AssertionError("中断恢复数量不一致")
        for expected, actual in zip(iter_records(source), recovered):
            compare_values(expected, actual)
        finalize_recording(target)
        if len(list(iter_records(target))) != len(recovered):
            raise AssertionError("checkpoint 改变了已提交数据")
        return {"status": "pass", **acknowledgement, "exit_code": process.returncode}
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdin.close()
        process.stdout.close()


class DiskMonitor:
    def __init__(self, directory, derived_root):
        self.directory = directory
        self.derived_root = derived_root
        self.peak_bytes = 0
        self.peak_allocated_bytes = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.measure, daemon=True)

    def measure(self):
        while not self.stop.is_set():
            sizes = []
            paths = list(self.directory.rglob("*"))
            for generated in self.derived_root.glob("derived-*"):
                paths.extend(generated.rglob("*"))
            for path in paths:
                try:
                    if path.is_file():
                        sizes.append(path.stat())
                except FileNotFoundError:
                    continue
            self.peak_bytes = max(self.peak_bytes, sum(item.st_size for item in sizes))
            self.peak_allocated_bytes = max(
                self.peak_allocated_bytes, sum(item.st_blocks * 512 for item in sizes),
            )
            self.stop.wait(.025)


def validate(args):
    root = args.resource_root.resolve(strict=True)
    args.output.mkdir(parents=True, exist_ok=False)
    monitor = DiskMonitor(args.output, root / ".scratch/static-experiments")
    monitor.thread.start()
    result = {"environment": environment_identity(), "runs": {}, "status": "running"}
    write_json(args.output / "validation.json", result)
    try:
        for name in args.runs:
            print(f"验证 {name}", flush=True)
            source = root / "output/static-experiments" / name
            destination = args.output / name
            migration = compact_run(source, destination)
            row = {"migration": migration["databases"]}
            result["runs"][name] = row
            for role in ("plant", "controller"):
                original, compact = source / f"{role}.sqlite3", destination / f"{role}.sqlite3"
                started = time.monotonic()
                original_identity = record_identity(original)
                row.setdefault("read_s", {})[role] = {"legacy": time.monotonic() - started}
                started = time.monotonic()
                if record_identity(compact) != original_identity:
                    raise AssertionError("完整记录身份不一致")
                row["read_s"][role]["compact"] = time.monotonic() - started
                for expected, actual in zip(iter_records(original), iter_records(compact), strict=True):
                    compare_values(expected, actual)
                selected = next(islice(iter_records(original, "candidate"), 128, None), None)
                if selected is not None:
                    compare_values(selected, read_record(compact, selected["sequence"], stream="candidate"))
                with tempfile.TemporaryDirectory(dir=args.output, prefix="write-benchmark-") as temporary:
                    row.setdefault("legacy_append_s", {})[role] = legacy_write(
                        original, Path(temporary) / "records.sqlite3",
                    )
                row.setdefault("interruption", {})[role] = check_interruption(
                    original, args.output / f"{name}-{role}-interrupted.sqlite3",
                )
            started = time.monotonic()
            original_report = evaluate_run(
                source, resource_root=root, save_samples=True,
                output_directory=args.output / f"{name}-source-replay",
            )
            row["source_replay_s"] = time.monotonic() - started
            original_report = json.loads(
                (args.output / f"{name}-source-replay/feedback_report.json").read_text(),
            )
            saved_report = json.loads((source / "feedback_report.json").read_text())
            row["historical_compared_sections"] = [
                "metrics", "feedback", "endpoint", "arc_statistics",
                "external_clearance", "self_clearance",
            ]
            for section in row["historical_compared_sections"]:
                compare_values(saved_report[section], original_report[section], exact=False, path=section)
            started = time.monotonic()
            report = evaluate_run(destination, resource_root=root)
            row["compact_replay_s"] = time.monotonic() - started
            report = json.loads((destination / "feedback_report.json").read_text())
            compare_values(measurement_fields(original_report), measurement_fields(report))
            validate_run_report(destination)
            started = time.monotonic()
            with evaluation_samples(destination, root, report) as (samples, recomputed):
                hashes = recomputed["derived_data_sha256"]
                row["derived_bytes"] = sum((samples / filename).stat().st_size for filename in hashes)
                row["historical_derived_max_abs_difference"] = {}
                for filename in hashes:
                    reference = args.output / f"{name}-source-replay" / filename
                    with np.load(reference, allow_pickle=False) as old, np.load(samples / filename, allow_pickle=False) as new:
                        if set(old.files) != set(new.files):
                            raise AssertionError(f"派生字段变化：{filename}")
                        for key in old.files:
                            if old[key].dtype != new[key].dtype or old[key].shape != new[key].shape:
                                raise AssertionError(f"派生数组类型或形状变化：{filename}:{key}")
                            np.testing.assert_array_equal(old[key], new[key])
                    with np.load(source / filename, allow_pickle=False) as archived, np.load(samples / filename, allow_pickle=False) as new:
                        differences = {}
                        for key in archived.files:
                            if archived[key].dtype.kind == "f":
                                differences[key] = float(np.max(np.abs(archived[key] - new[key]), initial=0.))
                            else:
                                np.testing.assert_array_equal(archived[key], new[key])
                        row["historical_derived_max_abs_difference"][filename] = differences
            row["derived_regeneration_s"] = time.monotonic() - started
            row["source_bytes"] = sum(path.stat().st_size for path in source.rglob("*") if path.is_file())
            row["compact_bytes"] = sum(path.stat().st_size for path in destination.rglob("*") if path.is_file())
            row["verdict"] = report["verdict"]
            row["feedback_samples"] = report["feedback"]["samples"]
            row["status"] = "pass"
            for filename, expected in migration["source_files_sha256"].items():
                if content_hash(source / filename) != expected:
                    raise AssertionError(f"原始文件变化：{filename}")
            write_json(args.output / "validation.json", result)
            print(json.dumps({name: {key: value for key, value in row.items()
                                     if key not in ("migration",)}}), flush=True)
        result["status"] = "pass"
    finally:
        monitor.stop.set()
        monitor.thread.join()
        if result["status"] == "running":
            result["status"] = "failed"
        result.update(
            peak_validation_bytes=monitor.peak_bytes,
            peak_validation_allocated_bytes=monitor.peak_allocated_bytes,
            peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
            available_bytes=os.statvfs(args.output).f_bavail * os.statvfs(args.output).f_frsize,
        )
        write_json(args.output / "validation.json", result)


def main():
    parser = argparse.ArgumentParser(description="使用完整真实运行验证紧凑存储，保留源目录")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resource-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--runs", nargs="+", default=DEFAULT_RUNS)
    parser.add_argument("--interrupt-source", type=Path)
    args = parser.parse_args()
    if args.interrupt_source is not None:
        interrupted_writer(args.interrupt_source, args.output)
    else:
        validate(args)


if __name__ == "__main__":
    main()
