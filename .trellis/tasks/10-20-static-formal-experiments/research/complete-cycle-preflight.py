import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


ENGINEERING_SHA256 = "2119aa8150c5335858649f0066a08f6bd1a89b56426530b6c68b48cfae020422"
OUTPUT_NAME = "complete-cycle-preflight-01"
SOURCE_COUNTS = {"plant": 18492, "controller": 1099}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description="完整周期计时集成后的三卷串行 preflight")
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--runtime-revision", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    root = args.resource_root.resolve(strict=True)
    sys.path.insert(0, str(root / "src"))

    from robot_safecontrol_moveit.static_cycle_timing import (
        CYCLE_TIMING_BOUNDARY, cycle_timing_summary, load_cycle_timing,
    )
    from robot_safecontrol_moveit.static_experiment_config import content_hash
    from robot_safecontrol_moveit.static_experiment_provenance import snapshot_sources, source_identity
    from robot_safecontrol_moveit.static_experiment_recording import iter_records, write_json

    capacity = root / "output/static-capacity-20261020"
    scene = root / "output/static-scene-validation-20261020"
    configs = scene / "development-scenes-07"
    second = Path("/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020")
    third = Path("/media/lsn/Windows/robot_safecontrol_static_experiments_20261021")
    volumes = {
        "ext4": (capacity, "03b3e264-cb91-440b-b594-0bdc7c9d190c"),
        "ntfs": (second, "083CBB753CBB5BFE"),
        "windows": (third, "6C001EAA001E7B72"),
    }
    control = capacity / OUTPUT_NAME
    source_run = second / "guarded-preflight-01/e-s5/run"
    engineering = capacity / "three-volume-capacity-01/engineering-plan/engineering_evidence.json"
    require(content_hash(engineering) == ENGINEERING_SHA256, "工程输入 SHA-256 不一致")
    evidence = load(engineering)
    revision = subprocess.run(
        ["git", "rev-parse", "--verify", args.runtime_revision + "^{commit}"],
        cwd=root, check=True, capture_output=True, text=True,
    ).stdout.strip()
    source_hashes = source_identity(root)
    script_names = (
        "prepare_static_storage_budget.py", "validate_static_storage_guard.py",
        "validate_static_filesystem.py", "validate_static_lifecycle.py",
        "validate_static_recording_upgrade.py", "plan_static_engineering_capacity.py",
    )
    helper_hashes = {str(root / "scripts" / name): content_hash(root / "scripts" / name)
                     for name in script_names}
    input_paths = [engineering, source_run / "manifest.json", source_run / "result.json",
                   source_run / "source_configuration.yaml", configs / "s0.yaml",
                   configs / "s5.yaml", configs / "s5_blockage.json",
                   configs / "s4-configurations/s4-initial-3-feed-0.5.yaml"]
    input_hashes = {str(path): content_hash(path) for path in input_paths}

    def bind_sources():
        subprocess.run(["git", "diff", "--exit-code", revision, "--", "src", "portable_oscbf"],
                       cwd=root, check=True, capture_output=True, text=True)
        untracked = subprocess.run(
            ["git", "ls-files", "--others", "--exclude-standard", "--", "src", "portable_oscbf"],
            cwd=root, check=True, capture_output=True, text=True,
        ).stdout.strip()
        require(not untracked, "生产源码目录包含未登记文件")
        require(source_identity(root) == source_hashes, "生产源码或 native 模块身份变化")
        require(all(content_hash(Path(path)) == value for path, value in helper_hashes.items()),
                "公共验证入口身份变化")
        require(all(content_hash(Path(path)) == value for path, value in input_hashes.items()),
                "配置或工程输入身份变化")

    def identify_volume(name):
        base, expected_uuid = volumes[name]
        require(base.resolve(strict=True) == base, f"输出卷路径包含重定向：{base}")
        identity = json.loads(subprocess.run(
            ["findmnt", "--json", "--target", str(base), "--output", "SOURCE,TARGET,FSTYPE,OPTIONS,UUID"],
            cwd=root, check=True, capture_output=True, text=True,
        ).stdout)["filesystems"][0]
        require(identity["uuid"] == expected_uuid and "rw" in identity["options"].split(","),
                f"输出卷身份或读写状态不符：{name}")
        return {**identity, "available_bytes": shutil.disk_usage(base).free}

    bind_sources()
    identities = {name: identify_volume(name) for name in volumes}
    for base, _ in volumes.values():
        require(not (base / OUTPUT_NAME).exists(), f"输出已存在：{base / OUTPUT_NAME}")
    cases = [
        {"name": "d-s5", "volume": name, "config": configs / "s5.yaml", "pairs": 66,
         "evidence": configs / "s5_blockage.json", "termination": "pause_timeout"}
        for name in volumes
    ] + [
        {"name": "d-s0", "volume": "ntfs", "config": configs / "s0.yaml", "pairs": 56,
         "termination": "completed"},
        {"name": "d-s4-initial3-feed0.5", "volume": "ntfs",
         "config": configs / "s4-configurations/s4-initial-3-feed-0.5.yaml", "pairs": 76,
         "evidence": scene / "s4-paired-independent-witness-01/witness", "termination": "completed"},
        {"name": "d-timeout", "volume": "ntfs", "config": configs / "s0.yaml", "pairs": 56,
         "feed_factor": .01, "termination": "timeout"},
        {"name": "d-wal-limit", "volume": "ntfs", "config": configs / "s0.yaml", "pairs": 56,
         "force_wal": 8388608, "termination": "budget_exhausted"},
    ]
    plan = {
        "runtime_revision": revision, "source_sha256": source_hashes,
        "script_sha256": content_hash(Path(__file__)), "helper_sha256": helper_hashes,
        "input_sha256": input_hashes, "source_run": str(source_run), "source_counts": SOURCE_COUNTS,
        "cycle_timing_boundary": CYCLE_TIMING_BOUNDARY, "filesystems_before": identities,
        "output_directories": {name: str(base / OUTPUT_NAME) for name, (base, _) in volumes.items()},
        "cases": [{key: str(value) if isinstance(value, Path) else value for key, value in case.items()}
                  for case in cases],
        "cpus": {"workers": "0-15", "filesystem": "16-21", "monitor": "22-27"},
        "domain": 200, "formal_collection_authorized_by_this_runner": False,
    }
    if not args.execute:
        print(json.dumps({"status": "prepared_without_execution", **plan}, ensure_ascii=False, indent=2))
        return 0

    os.sched_setaffinity(0, set(range(22, 28)))
    for base, _ in volumes.values():
        (base / OUTPUT_NAME).mkdir()
    (control / "logs").mkdir()
    (control / "budgets").mkdir()
    write_json(control / "plan.json", plan)
    shutil.copyfile(Path(__file__), control / "execution-source.py")
    commands, offline, results, manifests = [], [], [], []
    state = {"status": "running", "active": None, "commands": commands, "offline": offline, "runs": results}

    def save_state():
        write_json(control / "status.json", state)

    def run_stage(name, volume, command, *, monitor=False, allowed=(0,)):
        bind_sources()
        identify_volume(volume)
        command = ["taskset", "-c", "22-27" if monitor else "16-21", *command]
        state["active"] = name
        save_state()
        row = {"name": name, "command": command, "started_monotonic_s": time.monotonic()}
        commands.append(row)
        save_state()
        print(json.dumps({"event": "start", "name": name}), flush=True)
        with (control / "logs" / f"{name}.log").open("x") as log:
            process = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT)
        row.update(returncode=process.returncode, elapsed_s=time.monotonic() - row["started_monotonic_s"])
        save_state()
        bind_sources()
        require(process.returncode in allowed, f"公共验证入口失败：{name}，returncode={process.returncode}")
        return process.returncode

    def prepare_budget(name, volume, config, *, method="D", pairs, feed_factor=1., force_wal=None):
        budget = control / "budgets" / f"{name}.storage.json"
        command = [sys.executable, str(root / "scripts/prepare_static_storage_budget.py"),
                   "--engineering-evidence", str(engineering), "--resource-root", str(root),
                   "--config", str(config), "--volume", str(volumes[volume][0]), "--output", str(budget),
                   "--method", method, "--maximum-pairs", str(pairs), "--feed-factor", str(feed_factor)]
        if force_wal is not None:
            command.extend(["--force-wal", str(force_wal)])
        run_stage(name + ".budget", volume, command)
        require(load(budget)["source_sha256"] == source_hashes, "预算生产源码身份不符")
        return budget

    try:
        for volume, (base, uuid) in volumes.items():
            directory = base / OUTPUT_NAME
            budget = prepare_budget(volume + ".recording", volume, source_run / "source_configuration.yaml",
                                    method="E", pairs=66)
            guard_path = directory / "recording-guard/validation.json"
            run_stage(volume + ".guard", volume, [
                sys.executable, str(root / "scripts/validate_static_storage_guard.py"),
                "--source", str(source_run), "--storage-budget", str(budget),
                "--output", str(guard_path.parent),
            ])
            guard = load(guard_path)
            require(guard["status"] == "pass" and guard["source_sha256"] == source_hashes,
                    "真实记录容量守卫验证或源码身份未通过")
            require(set(guard["databases"]) == set(SOURCE_COUNTS), "容量守卫缺少 writer")
            for role, row in guard["databases"].items():
                killed = row["sigkill"]
                require(row["count"] == SOURCE_COUNTS[role] and killed["exit_code"] == -9
                        and killed["count"] >= 300 and killed["reason"] is not None
                        and killed["snapshot"]["completion_within_reserve"], "容量守卫记录或 SIGKILL 证据不符")
            fs_path = directory / "filesystem/validation.json"
            run_stage(volume + ".filesystem", volume, [
                sys.executable, str(root / "scripts/validate_static_filesystem.py"),
                "--run", str(source_run), "--output", str(fs_path.parent),
            ])
            fs = load(fs_path)
            require(fs["status"] == "pass" and fs["filesystem"]["uuid"] == uuid
                    and fs["recording_source_sha256"] == content_hash(
                        root / "src/robot_safecontrol_moveit/static_experiment_recording.py"),
                    "文件系统验证或 recording 源码身份不符")
            require(len(fs["runs"]) == 1 and fs["runs"][0]["source"] == str(source_run), "原始 run 身份不符")
            require(all(sum(row["counts"].values()) == SOURCE_COUNTS[role]
                        for role, row in fs["runs"][0]["databases"].items()), "文件系统真实记录数量不符")
            offline.append({"volume": volume, "guard": str(guard_path), "guard_sha256": content_hash(guard_path),
                            "filesystem": str(fs_path), "filesystem_sha256": content_hash(fs_path),
                            "budget": str(budget), "budget_sha256": content_hash(budget)})
            save_state()

        for case in cases:
            volume, name = case["volume"], case["volume"] + "." + case["name"]
            output = volumes[volume][0] / OUTPUT_NAME / case["name"]
            budget = prepare_budget(name, volume, case["config"], pairs=case["pairs"],
                                    feed_factor=case.get("feed_factor", 1.), force_wal=case.get("force_wal"))
            command = [sys.executable, str(root / "scripts/validate_static_lifecycle.py"),
                       "--config", str(case["config"]), "--method", "D", "--resource-root", str(root),
                       "--domain", "200", "--cpus", "0-15", "--storage-budget", str(budget),
                       "--output", str(output), "--feed-factor", str(case.get("feed_factor", 1.))]
            if "evidence" in case:
                command.extend(["--scene-evidence", str(case["evidence"])])
            if case["termination"] == "timeout":
                command.append("--expect-timeout")
            returncode = run_stage(name, volume, command, monitor=True, allowed=(0, 1))
            run = output / "run"
            manifest, result, feedback = (load(run / filename) for filename in (
                "manifest.json", "result.json", "feedback_report.json"))
            validation_path = output / "lifecycle-validation.json"
            validation = load(validation_path)
            measured = load_cycle_timing(run, manifest)
            timing = cycle_timing_summary(measured, manifest["config"]["control"]["period_s"])
            admissions = {record["values"]["command_id"]: record["values"]
                          for record in iter_records(run / "plant.sqlite3", "admission")}
            timing["admission_rejections"] = sum(not record["accepted"] for record in admissions.values())
            require(manifest["source_sha256"] == source_hashes, "运行生产源码身份不符")
            require(measured["eligible_for_period"] and measured["boundary"] == CYCLE_TIMING_BOUNDARY
                    and timing["samples"] > 0 and timing == feedback["execution"]["timing"],
                    "完整周期 sidecar、边界或报告数值不一致")
            statistics = {}
            for role in ("plant", "controller"):
                path = run / f"{role}.recording.json"
                record = load(path)
                storage = record["storage_budget"]
                require(result["recording_statistics_sha256"][role] == content_hash(path), "统计文件身份不符")
                require(storage["completion_within_reserve"] and path.stat().st_blocks * 512
                        <= storage["limits"]["statistics_bytes"], "统计文件或收尾空间超过预算")
                require(all(value <= storage["limits"]["record_bytes"][stream]
                            for stream, value in storage["maximum_record_bytes"].items())
                        and all(value <= storage["limits"]["record_write_bytes"][stream]
                                for stream, value in storage["maximum_write_bytes"].items()),
                        "记录大小或 WAL 写入超过预算")
                statistics[role] = record
            forced = case["termination"] == "budget_exhausted"
            row = {
                "name": name, "run": str(run), "returncode": returncode,
                "termination": result["termination"], "worker_exit_codes": result["worker_exit_codes"],
                "expected_failure": case["termination"] in ("timeout", "budget_exhausted"),
                "lifecycle_status": validation["status"], "functional_verdict": feedback["task_verdict"],
                "runtime_timing": timing, "verdict": feedback["verdict"], "statistics": statistics,
                "lifecycle": str(validation_path), "lifecycle_sha256": content_hash(validation_path),
                "budget": str(budget), "budget_sha256": content_hash(budget),
                "sha256": {path.name: content_hash(path) for path in sorted(run.iterdir())
                           if path.is_file() and path.suffix in (".sqlite3", ".json", ".ready", ".yaml", ".log")},
                "terminal_events": {role: [record["values"] for record in iter_records(
                    run / f"{role}.sqlite3", "event")] for role in ("plant", "controller")},
            }
            results.append(row)
            manifests.append(manifest)
            save_state()
            require(result["worker_exit_codes"] == [0, 0] and result["termination"] == case["termination"],
                    f"worker 或终止原因不符合预期：{name}")
            if forced:
                require(returncode == 1 and feedback["task_verdict"] != "pass"
                        and any(item["storage_budget"]["reason"] is not None for item in statistics.values())
                        and any(event["reason"] == "budget_exhausted"
                                for events in row["terminal_events"].values() for event in events),
                        "8 MiB WAL 预算终止及收尾证据不完整")
            else:
                require(returncode == 0 and validation["status"] == "pass"
                        and all(item["storage_budget"]["reason"] is None for item in statistics.values()),
                        f"生命周期或容量收尾未通过：{name}")
                if case["termination"] == "completed":
                    require(feedback["task_verdict"] == "pass" and feedback["completed"], "S0/S4 功能未通过")
                if case["termination"] == "pause_timeout":
                    require(feedback["task_verdict"] == "pass", "S5 功能未通过")
                if case["termination"] == "timeout":
                    require(feedback["task_verdict"] != "pass"
                            and validation["inspection"]["full_timeout_recording_verified"],
                            "420 秒 timeout 证据不完整")

        bind_sources()
        snapshot = snapshot_sources(root, manifests, control / "source")
        audits = list(evidence["audit_sha256"])
        lifecycles = [row["lifecycle"] for row in results if row["termination"] != "budget_exhausted"]
        filesystems = [row["filesystem"] for row in offline]
        formal_directories = [capacity / "formal-runs-ext4", second / "formal-runs", third / "formal-runs"]
        command = ["taskset", "-c", "16-21", sys.executable, str(root / "scripts/plan_static_engineering_capacity.py")]
        for option, paths in (("--audit", audits), ("--lifecycle", lifecycles), ("--filesystem", filesystems),
                              ("--volume", formal_directories)):
            for path in paths:
                command.extend([option, str(path)])
        command.extend(["--runtime-archive", str(control / "source/sources.zip"), "--config", str(configs / "s0.yaml"),
                        "--resource-root", str(root), "--retry-runs", "0", "--output", str(control / "engineering-plan")])
        capacity_inputs = [*audits, *lifecycles, *filesystems, str(control / "source/sources.zip"),
                           str(configs / "s0.yaml")]
        write_json(control / "capacity-inputs.json", {
            "runtime_revision": revision, "source_sha256": source_hashes, "formal_bound": False,
            "command": command, "executed": False, "runtime_archive": snapshot,
            "input_sha256": {path: content_hash(Path(path)) for path in capacity_inputs},
            "supplementary_guard_reports": offline,
            "forced_wal_lifecycle": results[-1]["lifecycle"],
        })
        state.update(status="completed", active=None, runtime_timing_statuses={
            row["name"]: row["runtime_timing"]["status"] for row in results},
            filesystems_after={name: identify_volume(name) for name in volumes})
        write_json(control / "completed.json", state)
        print(json.dumps({"status": "completed", "output": str(control),
                          "runtime_timing_statuses": state["runtime_timing_statuses"]}, ensure_ascii=False))
    finally:
        if state["status"] == "running":
            state["status"] = "incomplete"
        save_state()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
