from __future__ import annotations

import argparse
from collections import Counter
import hashlib
from itertools import islice
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time

import msgpack
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_evaluation import evaluate_run, evaluation_samples
from robot_safecontrol_moveit.static_experiment_provenance import environment_identity, validate_run_report
from robot_safecontrol_moveit.static_experiment_recording import (
    RunRecorder, finalize_recording, iter_records, read_record, write_json,
)
from robot_safecontrol_moveit.static_experiment_storage import compact_run
from validate_static_storage import (
    DEFAULT_RUNS, DiskMonitor, compare_values, measurement_fields,
)


def interrupt_writer(source, destination):
    recorder = RunRecorder(destination)
    digest = hashlib.sha256()
    for index, row in enumerate(iter_records(source), start=1):
        recorder.append(
            row["stream"], sequence=row["sequence"], monotonic_s=row["monotonic_s"],
            simulation_s=row["simulation_s"], ros_time_ns=row["ros_time_ns"], values=row["values"],
        )
        digest.update(msgpack.packb(row, use_bin_type=True, use_single_float=False))
        if index == 1200:
            print(json.dumps({"count": index, "sha256": digest.hexdigest()}), flush=True)
            sys.stdin.read(1)
            raise RuntimeError("恢复验证等待 SIGKILL")
    raise ValueError("真实来源记录少于 1200 条")


def verify_interruption(source, destination):
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), "--interrupt-source", str(source),
         "--output", str(destination)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
    )
    try:
        expected = json.loads(process.stdout.readline())
        process.kill()
        if process.wait(timeout=30) != -signal.SIGKILL:
            raise ValueError("恢复验证未取得 SIGKILL")
        counts, digest = Counter(), hashlib.sha256()
        for original, recovered in zip(
                islice(iter_records(source), expected["count"]), iter_records(destination), strict=True):
            compare_values(original, recovered)
            counts[recovered["stream"]] += 1
            digest.update(msgpack.packb(recovered, use_bin_type=True, use_single_float=False))
        if digest.hexdigest() != expected["sha256"] or sum(counts.values()) != expected["count"]:
            raise ValueError("已提交记录恢复数量或字节身份变化")
        finalize_recording(destination)
        return {**expected, "counts": dict(counts), "exit_code": process.returncode}
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdin.close()
        process.stdout.close()


def validate(source_root, resource_root, output):
    output.mkdir(parents=True, exist_ok=False)
    monitor = DiskMonitor(output, resource_root / ".scratch/static-experiments")
    monitor.thread.start()
    report = {
        "status": "running", "environment": environment_identity(),
        "command": sys.argv, "script_sha256": content_hash(Path(__file__)), "runs": {},
    }
    try:
        for name in DEFAULT_RUNS:
            source, destination = source_root / name, output / name
            print(f"升级并核查 {name}", flush=True)
            validate_run_report(source)
            migration = compact_run(source, destination)
            result = {"migration": migration["databases"], "source": str(source), "counts": {}}
            report["runs"][name] = result
            for role in ("plant", "controller"):
                counts = Counter()
                for original, current in zip(
                        iter_records(source / f"{role}.sqlite3"),
                        iter_records(destination / f"{role}.sqlite3"), strict=True):
                    compare_values(original, current)
                    counts[current["stream"]] += 1
                    if counts[current["stream"]] in (1, 127, 128, 129, 256, 300):
                        compare_values(original, read_record(
                            destination / f"{role}.sqlite3", current["sequence"], stream=current["stream"],
                        ))
                result["counts"][role] = dict(counts)
                result.setdefault("sigkill", {})[role] = verify_interruption(
                    source / f"{role}.sqlite3", output / f"{name}-{role}-sigkill.sqlite3",
                )
            started = time.perf_counter()
            original_report = evaluate_run(
                source, resource_root=resource_root, save_samples=True,
                output_directory=output / f"{name}-source-replay",
            )
            result["source_replay_s"] = time.perf_counter() - started
            started = time.perf_counter()
            current_report = evaluate_run(destination, resource_root=resource_root)
            result["upgraded_replay_s"] = time.perf_counter() - started
            compare_values(measurement_fields(original_report), measurement_fields(current_report))
            validate_run_report(destination)
            with evaluation_samples(destination, resource_root, current_report) as (samples, recomputed):
                result["derived_bytes"] = sum(
                    (samples / filename).stat().st_size for filename in recomputed["derived_data_sha256"]
                )
                for filename in recomputed["derived_data_sha256"]:
                    original_path = output / f"{name}-source-replay" / filename
                    with np.load(original_path, allow_pickle=False) as first, np.load(
                            samples / filename, allow_pickle=False) as second:
                        if first.files != second.files:
                            raise ValueError(f"派生数组字段变化：{filename}")
                        for field in first.files:
                            if (first[field].dtype != second[field].dtype
                                    or first[field].shape != second[field].shape
                                    or first[field].tobytes() != second[field].tobytes()):
                                raise ValueError(f"派生数组位模式变化：{filename}/{field}")
            for filename, expected in migration["source_files_sha256"].items():
                if content_hash(source / filename) != expected:
                    raise ValueError(f"来源文件身份变化：{source / filename}")
            result.update(
                status="pass", verdict=current_report["verdict"],
                source_directory_bytes=sum(path.stat().st_size for path in source.rglob("*") if path.is_file()),
                upgraded_directory_bytes=sum(
                    path.stat().st_size for path in destination.rglob("*") if path.is_file()),
            )
            write_json(output / "validation.json", report)
            print(json.dumps({name: {key: result[key] for key in (
                "status", "verdict", "source_directory_bytes", "upgraded_directory_bytes",
            )}}, ensure_ascii=False), flush=True)
        report["status"] = "pass"
    finally:
        monitor.stop.set()
        monitor.thread.join()
        if report["status"] == "running":
            report["status"] = "failed"
        report.update(
            peak_validation_bytes=monitor.peak_bytes,
            peak_validation_allocated_bytes=monitor.peak_allocated_bytes,
            peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
            available_bytes=os.statvfs(output).f_bavail * os.statvfs(output).f_frsize,
        )
        write_json(output / "validation.json", report)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--resource-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interrupt-source", type=Path)
    args = parser.parse_args()
    if args.interrupt_source is not None:
        interrupt_writer(args.interrupt_source, args.output)
    else:
        validate(args.source_root.resolve(strict=True), args.resource_root.resolve(strict=True),
                 args.output.resolve())


if __name__ == "__main__":
    main()
