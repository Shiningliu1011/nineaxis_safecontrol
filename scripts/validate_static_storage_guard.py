import argparse
from dataclasses import replace
import hashlib
import json
import multiprocessing
from pathlib import Path
import signal
import time

import msgpack

from robot_safecontrol_moveit.static_experiment_capacity import RecordingBudgetExhausted
from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_recording import (
    RunRecorder, finalize_recording, iter_records, write_json,
)
from robot_safecontrol_moveit.static_experiment_provenance import source_identity
from robot_safecontrol_moveit.static_recording_capacity import RecordingStorageLimits


def _digest(rows):
    digest = hashlib.sha256()
    count = 0
    for row in rows:
        digest.update(msgpack.packb(row, use_bin_type=True, use_single_float=False))
        count += 1
    return count, digest.hexdigest()


def _write(source, output, limits, pipe):
    recorder = RunRecorder(output, storage_limits=limits)
    count, reason = 0, None
    try:
        for row in iter_records(source):
            recorder.check_capacity()
            recorder.append(row["stream"], **{key: row[key] for key in (
                "sequence", "monotonic_s", "simulation_s", "ros_time_ns", "values",
            )})
            count += 1
        recorder.check_capacity()
    except RecordingBudgetExhausted as error:
        reason = str(error)
        recorder.append("event", monotonic_s=time.monotonic(), values={"reason": "budget_exhausted"})
    if pipe is None:
        if reason is not None:
            raise RuntimeError(f"完整记录重写触发预算：{reason}")
        snapshot = recorder.capacity_snapshot()
        if not snapshot["completion_within_reserve"]:
            raise ValueError("完整记录的收尾增长超过已配置余量")
        recorder.close()
        return count, snapshot
    pipe.send({"count": count, "reason": reason, "snapshot": recorder.capacity_snapshot()})
    pipe.recv()


def validate_database(source: Path, output: Path, limits: dict) -> dict:
    source_sha256 = content_hash(source)
    normal = output / source.name
    count, snapshot = _write(source, normal, limits, None)
    if _digest(iter_records(source)) != _digest(iter_records(normal)):
        raise ValueError("完整记录重写字段或 float64 位模式不一致")
    forced = replace(RecordingStorageLimits(**limits), wal_bytes=8 * 1048576)
    interrupted = output / (source.stem + "-interrupted.sqlite3")
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(target=_write, args=(source, interrupted, forced.__dict__, child))
    process.start()
    try:
        if not parent.poll(120):
            raise TimeoutError("记录额度 SIGKILL 验证没有完成写入")
        acknowledged = parent.recv()
        if acknowledged["reason"] is None or acknowledged["count"] < 300:
            raise ValueError("真实记录不足以覆盖预算触发与跨 keyframe 恢复")
        if not acknowledged["snapshot"]["completion_within_reserve"]:
            raise ValueError("预算触发记录和终止记录超过收尾余量")
        process.kill()
        process.join(30)
        if process.exitcode != -signal.SIGKILL:
            raise ValueError("记录进程没有按 SIGKILL 退出")
        prefix = list(iter_records(interrupted))
        terminal = prefix.pop()
        if terminal["stream"] != "event" or terminal["values"]["reason"] != "budget_exhausted":
            raise ValueError("预算触发后的终止记录缺失")
        source_prefix = []
        for row in iter_records(source):
            if len(source_prefix) == acknowledged["count"]:
                break
            source_prefix.append(row)
        if _digest(prefix) != _digest(source_prefix):
            raise ValueError("SIGKILL 恢复后的完整记录不一致")
        finalize_recording(interrupted)
    finally:
        if process.is_alive():
            process.kill()
            process.join()
        parent.close()
        child.close()
    if source_sha256 != content_hash(source):
        raise ValueError("原始数据库身份变化")
    return {
        "source": str(source), "source_sha256": source_sha256,
        "rewritten": str(normal), "rewritten_sha256": content_hash(normal), "count": count,
        "storage_snapshot": snapshot,
        "sigkill": {**acknowledged, "exit_code": process.exitcode,
                    "path": str(interrupted), "sha256": content_hash(interrupted)},
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--storage-budget", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    budget = json.loads(args.storage_budget.read_text())
    sources = source_identity(Path(__file__).resolve().parents[1])
    if budget["source_sha256"] != sources:
        raise ValueError("运行空间预算与验证源码身份不一致")
    results = {}
    for role in ("plant", "controller"):
        directory = args.output / role
        directory.mkdir()
        results[role] = validate_database(
            args.source / f"{role}.sqlite3", directory, budget["writers"][role],
        )
    report = {
        "status": "pass", "script_sha256": content_hash(Path(__file__)),
        "budget_sha256": content_hash(args.storage_budget), "databases": results,
        "source_sha256": sources,
    }
    if source_identity(Path(__file__).resolve().parents[1]) != sources:
        raise ValueError("验证期间源码身份变化")
    write_json(args.output / "validation.json", report)
    print(json.dumps({"status": report["status"], "records": sum(row["count"] for row in results.values())}))


if __name__ == "__main__":
    main()
