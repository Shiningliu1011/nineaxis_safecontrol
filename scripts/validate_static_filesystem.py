from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import time

import msgpack
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_provenance import validate_run_report
from robot_safecontrol_moveit.static_experiment_recording import (
    RunRecorder, finalize_recording, iter_records, read_record, write_json,
)
from validate_static_recording_upgrade import verify_interruption
from validate_static_storage import compare_values


def copy_and_verify(source: Path, target: Path) -> dict:
    original_hash = content_hash(source)
    counts, durations, expected = Counter(), {}, hashlib.sha256()
    started = time.monotonic()
    with RunRecorder(target) as recorder:
        for row in iter_records(source):
            elapsed = recorder.append(
                row["stream"], sequence=row["sequence"], monotonic_s=row["monotonic_s"],
                simulation_s=row["simulation_s"], ros_time_ns=row["ros_time_ns"], values=row["values"],
            )
            counts[row["stream"]] += 1
            durations.setdefault(row["stream"], []).append(elapsed)
            expected.update(msgpack.packb(row, use_bin_type=True, use_single_float=False))
        wal = Path(str(target) + "-wal")
        wal_bytes = wal.stat().st_size
        wal_allocated = wal.stat().st_blocks * 512
    copying_s = time.monotonic() - started
    decoded, stream_counts, random_reads = hashlib.sha256(), Counter(), 0
    for original, restored in zip(iter_records(source), iter_records(target), strict=True):
        compare_values(original, restored)
        decoded.update(msgpack.packb(restored, use_bin_type=True, use_single_float=False))
        stream = restored["stream"]
        stream_counts[stream] += 1
        if stream_counts[stream] in (1, 127, 128, 129, 256, 300, counts[stream]):
            compare_values(original, read_record(target, restored["sequence"], stream=stream))
            random_reads += 1
    if expected.digest() != decoded.digest() or counts != stream_counts:
        raise ValueError(f"文件系统逐条记录身份变化：{target}")
    finalize_recording(target)
    if content_hash(source) != original_hash:
        raise ValueError(f"来源数据库身份变化：{source}")
    return {
        "status": "pass", "source": str(source), "source_sha256": original_hash,
        "target_sha256": content_hash(target), "record_sha256": decoded.hexdigest(),
        "counts": dict(counts), "random_reads": random_reads, "copying_s": copying_s,
        "target_bytes": target.stat().st_size, "allocated_bytes": target.stat().st_blocks * 512,
        "wal_bytes": wal_bytes, "wal_allocated_bytes": wal_allocated,
        "append_s": {
            name: {
                "mean": float(np.mean(values)), "p999": float(np.quantile(values, .999)),
                "maximum": float(max(values)),
            } for name, values in durations.items()
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    filesystem = json.loads(subprocess.run(
        ["findmnt", "--json", "--target", str(output), "--output", "SOURCE,TARGET,FSTYPE,OPTIONS,UUID"],
        check=True, text=True, capture_output=True,
    ).stdout)["filesystems"][0]
    report = {
        "status": "running", "filesystem": filesystem, "command": sys.argv,
        "script_sha256": content_hash(Path(__file__)),
        "interruption_script_sha256": content_hash(
            Path(__file__).with_name("validate_static_recording_upgrade.py"),
        ),
        "recording_source_sha256": content_hash(
            Path(__file__).resolve().parents[1] / "src/robot_safecontrol_moveit/static_experiment_recording.py"
        ),
        "cpu_affinity": sorted(os.sched_getaffinity(0)), "runs": [],
        "free_bytes_before": shutil.disk_usage(output).free,
    }
    shutil.copyfile(Path(__file__), output / "validation_source.py")
    for index, run in enumerate(args.run):
        run = run.resolve(strict=True)
        manifest, _ = validate_run_report(run)
        item = {
            "source": str(run), "manifest_sha256": content_hash(run / "manifest.json"),
            "config_sha256": manifest["config_sha256"], "source_sha256": manifest["source_sha256"],
            "databases": {},
        }
        for role in ("plant", "controller"):
            source, target = run / f"{role}.sqlite3", output / f"{index}-{role}.sqlite3"
            item["databases"][role] = copy_and_verify(source, target)
            item["databases"][role]["sigkill"] = verify_interruption(
                source, output / f"{index}-{role}-sigkill.sqlite3",
                count=min(1200, sum(item["databases"][role]["counts"].values())),
            )
        report["runs"].append(item)
        write_json(output / "validation.json", report)
        print(json.dumps({"run": str(run), "status": "pass"}, ensure_ascii=False), flush=True)
    pending = output / "atomic.json.pending"
    write_json(pending, {"source_count": len(report["runs"])})
    pending.rename(output / "atomic.json")
    if json.loads((output / "atomic.json").read_text()) != {"source_count": len(report["runs"])}:
        raise ValueError("文件系统原子结果发布检查失败")
    report.update(
        status="pass", free_bytes_after=shutil.disk_usage(output).free,
        rss_maximum_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
    )
    write_json(output / "validation.json", report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
