import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description="依据完整第三卷证据重建工程容量")
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    root = args.resource_root.resolve(strict=True)
    capacity = root / "output/static-capacity-20261020"
    second = Path("/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020")
    third = Path("/media/lsn/Windows/robot_safecontrol_static_experiments_20261021")
    output = capacity / "three-volume-capacity-01"
    require(not output.exists(), f"输出路径已存在：{output}")
    third_runs = third / "formal-runs"
    require(not third_runs.exists(), f"第三卷正式输出路径已存在：{third_runs}")
    audits = [capacity / name / "envelopes.json" for name in (
        "record-envelopes-02", "record-envelopes-03", "record-envelopes-04",
    )]
    names = (
        "d-s5", "e-s5", "b-s5", "d-s4-initial1-feed1.5", "d-s4-initial3-feed0.5",
        "d-s2", "a-s2", "b-s2", "e-s4", "d-timeout", "d-wal-limit",
    )
    guarded = {name: second / "guarded-preflight-01" / name / "lifecycle-validation.json"
               for name in names}
    lifecycle_names = ("d-s5", "d-s4-initial1-feed1.5", "d-s4-initial3-feed0.5", "d-s2", "d-timeout")
    lifecycles = [*(guarded[name] for name in lifecycle_names),
                  capacity / "guarded-ext4-s5-01/lifecycle-validation.json",
                  third / "guarded-s5-01/lifecycle-validation.json"]
    filesystems = [capacity / "guarded-capacity-review-01/ext4-filesystem-evidence.json",
                   capacity / "guarded-capacity-review-01/ntfs-filesystem-evidence.json",
                   third / "filesystem-01/validation.json"]
    guard_path = third / "storage-guard-01/validation.json"
    summary_path = capacity / "guarded-preflight-01/validation-summary.json"
    archive = capacity / "guarded-preflight-01/source/sources.zip"
    config = root / "output/static-experiments/development-scenes-05/s0.yaml"
    inputs = list(dict.fromkeys([
        *audits, *guarded.values(), *lifecycles, *filesystems, guard_path, summary_path, archive, config,
    ]))
    missing = [str(path) for path in inputs if not path.is_file()]
    if missing:
        print(json.dumps({"status": "pending_evidence", "missing": missing}, ensure_ascii=False))
        return 2

    summary = load(summary_path)
    require(summary["status"] == "pass" and {row["name"] for row in summary["runs"]} == set(names),
            "十一项 guarded 汇总未完整通过")
    guard = load(guard_path)
    require(guard["status"] == "pass" and guard["source_sha256"] == summary["source_sha256"],
            "第三卷容量守卫证据或来源未通过")
    require(set(guard["databases"]) == {"plant", "controller"}, "第三卷容量守卫缺少 writer")
    for row in guard["databases"].values():
        recovery = row["sigkill"]
        require(row["count"] > 0 and recovery["exit_code"] == -9 and recovery["count"] >= 300
                and recovery["reason"] is not None
                and recovery["snapshot"]["completion_within_reserve"],
                "第三卷 SIGKILL 或预算终止证据未通过")
    require(load(filesystems[-1])["status"] == "pass", "第三卷文件系统验证未通过")
    require(load(filesystems[-1])["filesystem"]["uuid"] == "6C001EAA001E7B72",
            "第三卷文件系统证据 UUID 不符")
    require(all(load(path)["status"] == "pass" for path in lifecycles), "完整生命周期证据未通过")
    require(load(guarded["d-wal-limit"])["inspection"]["result"]["termination"] == "budget_exhausted",
            "原始强制 WAL 额度终止证据不符")

    volumes = [capacity / "formal-runs-ext4", second / "formal-runs", third_runs]
    identities = []
    for directory in (volumes[0], volumes[1], third):
        report = json.loads(subprocess.run(
            ["findmnt", "--json", "--target", str(directory), "--output", "SOURCE,TARGET,FSTYPE,OPTIONS,UUID"],
            check=True, capture_output=True, text=True,
        ).stdout)["filesystems"][0]
        require("rw" in report["options"].split(","), f"文件系统未读写挂载：{directory}")
        identities.append({**report, "available_bytes": shutil.disk_usage(directory).free})
    require(identities[1]["uuid"] == "083CBB753CBB5BFE"
            and identities[2]["uuid"] == "6C001EAA001E7B72", "NTFS 输出卷 UUID 不符")
    require(len({row["uuid"] for row in identities}) == 3, "输出卷没有对应三个独立文件系统")
    subprocess.run(["git", "-C", str(root), "diff", "--exit-code", "40d43bb", "--", "src", "portable_oscbf"], check=True)
    command = [sys.executable, str(root / "scripts/plan_static_engineering_capacity.py")]
    for option, paths in (("--audit", audits), ("--lifecycle", lifecycles), ("--filesystem", filesystems)):
        for path in paths:
            command.extend([option, str(path)])
    for directory in volumes:
        command.extend(["--volume", str(directory)])
    command.extend(["--runtime-archive", str(archive), "--config", str(config),
                    "--resource-root", str(root), "--retry-runs", "0",
                    "--output", str(output / "engineering-plan")])
    if not args.execute:
        print(json.dumps({"status": "ready_to_execute", "command": command}, ensure_ascii=False))
        return 0

    source_hashes = {str(path): digest(path) for path in inputs}
    output.mkdir()
    third_runs.mkdir()
    shutil.copyfile(Path(__file__), output / "execution-source.py")
    started = datetime.now(timezone.utc).isoformat()
    with (output / "generation.log").open("x") as log:
        process = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, check=False)
    execution = {
        "command": command, "returncode": process.returncode, "formal_bound": False,
        "started_utc": started, "finished_utc": datetime.now(timezone.utc).isoformat(),
        "cpu_affinity": sorted(os.sched_getaffinity(0)), "input_sha256": source_hashes,
        "script_sha256": digest(Path(__file__)), "filesystems_before": identities,
        "guarded_runs": {name: {"report": str(path), "capacity_lifecycle": name in lifecycle_names}
                         for name, path in guarded.items()},
        "output_sha256": {str(path): digest(path) for path in (
            output / "engineering-plan/engineering_evidence.json",
            output / "engineering-plan/capacity_plan.json",
        ) if path.is_file()},
    }
    (output / "execution.json").write_text(json.dumps(execution, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"returncode": process.returncode, "output": str(output)}, ensure_ascii=False))
    return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
