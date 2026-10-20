from __future__ import annotations

import argparse
from dataclasses import fields
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

import numpy as np
import robot_static_fcl
import rclpy
from rclpy.executors import ExternalShutdownException

from .oscbf_trajectory import bootstrap_portable, default_portable_root
from .static_experiment_config import ExperimentConfig, content_hash, METHODS
from .static_experiment_evaluation import evaluate_run
from .static_experiment_recording import FORMAT_VERSION, RunRecorder, finalize_recording, write_json
from .static_experiment_provenance import source_identity
from .static_experiment_capacity import (
    RecordingBudgetExhausted, RecordingLimits, SUPERVISOR_POLL_S, WORKER_STARTUP_TIMEOUT_S,
    WORKER_STOP_TIMEOUT_S, WORKER_INTERRUPT_TIMEOUT_S,
)


def finish_worker_recording(node, recorder, directory: Path, *, role: str,
                            stop_reason: str | None, terminal: dict) -> None:
    original = sys.exc_info()[1]
    errors = []
    if node is not None:
        try:
            node.finish_recording(stop_reason)
        except BaseException as error:
            errors.append(error)
        try:
            node.destroy_node()
        except BaseException as error:
            errors.append(error)
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except BaseException as error:
            errors.append(error)
    if errors:
        terminal = {
            "reason": "fault", "phase": "FAULT",
            "error": repr(original or errors[0]),
            "cleanup_errors": [repr(error) for error in errors],
        }
    recorder.append("event", monotonic_s=time.monotonic(), values=terminal)
    wal = Path(str(recorder.path) + "-wal")
    write_json(directory / f"{role}.recording.json", {
        "completed_monotonic_s": time.monotonic(),
        "wal_bytes_before_checkpoint": wal.stat().st_size,
        "wal_allocated_bytes_before_checkpoint": wal.stat().st_blocks * 512,
        "budget": None if node is None else node.recording_budget.snapshot(),
        "stream_counts": dict(recorder.connection.execute(
            "SELECT stream,count(*) FROM samples GROUP BY stream",
        )),
    })
    if errors:
        raise original or errors[0]


def _publish_worker_result(path: Path, result: dict) -> None:
    pending = path.with_suffix(".json.pending")
    write_json(pending, result)
    pending.rename(path)


def _worker(args, config: ExperimentConfig) -> None:
    bootstrap_portable(config.resource_root / "portable_oscbf")
    from .static_experiment_nodes import StaticExperimentController, StaticExperimentPlant

    directory = args.output.resolve()
    rclpy.init(args=[])
    node = None
    stop_reason = None
    terminal = {"reason": "worker_stopped"}
    with RunRecorder(directory / f"{args.role}.sqlite3") as recorder:
        try:
            if args.role == "plant":
                node = StaticExperimentPlant(config, recorder)
            else:
                node = StaticExperimentController(config, recorder)
            ready = directory / f"{args.role}.ready"
            pending_ready = ready.with_suffix(".ready.pending")
            write_json(pending_ready, {
                "role": args.role, "monotonic_s": time.monotonic(),
                "manifest_sha256": content_hash(directory / "manifest.json"),
                "source_sha256": _sources(config),
            })
            pending_ready.rename(ready)
            while rclpy.ok():
                node.recording_budget.check()
                if (directory / "stop").exists() and (args.role != "plant" or node.stopped):
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
                if args.role == "controller" and node.execution_state == "holding":
                    snapshot = node.progress_snapshot()
                    _publish_worker_result(directory / "controller_result.json", {
                        "termination": snapshot["termination"], "steps": snapshot["steps"],
                        "monotonic_s": time.monotonic(),
                    })
                    break
        except RecordingBudgetExhausted as error:
            stop_reason = "budget_exhausted"
            terminal = {"reason": stop_reason, "phase": "FAULT", "error": str(error)}
            _publish_worker_result(directory / f"{args.role}_result.json", {
                "termination": stop_reason, "reason": str(error), "monotonic_s": time.monotonic(),
            })
        except (KeyboardInterrupt, ExternalShutdownException):
            stop_reason = "interrupted"
            terminal = {"reason": stop_reason}
        except BaseException as error:
            stop_reason = "fault"
            terminal = {"reason": stop_reason, "phase": "FAULT", "error": repr(error)}
            raise
        finally:
            finish_worker_recording(
                node, recorder, directory, role=args.role, stop_reason=stop_reason, terminal=terminal,
            )


def _sources(config: ExperimentConfig) -> dict:
    return source_identity(config.resource_root)


def _stop_processes(processes: list, directory: Path) -> None:
    (directory / "stop").touch()
    for process in processes:
        try:
            process.wait(timeout=WORKER_STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=WORKER_INTERRUPT_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def run_experiment(args, config: ExperimentConfig) -> dict:
    import json

    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "tmp").mkdir()
    source_configuration = directory / "source_configuration.yaml"
    shutil.copyfile(config.source, source_configuration)
    geometry, _ = config.load_path()
    np.savez_compressed(directory / "path.npz", **{
        field.name: getattr(geometry, field.name) for field in fields(geometry)
    })
    manifest = {
        **config.manifest(), "source_sha256": _sources(config),
        "path_sha256": content_hash(directory / "path.npz"),
        "recording_format": FORMAT_VERSION,
        "recording_limits": RecordingLimits(
            config.values["timeout_s"], config.values["settle_s"],
            config.values["control"]["period_s"],
        ).manifest(),
        "python": sys.version, "command": sys.argv,
        "geometry_runtime": {
            "fcl_version": robot_static_fcl.fcl_version,
            "distance_tolerance_m": robot_static_fcl.distance_tolerance_m,
            "extension": robot_static_fcl.__file__,
        },
    }
    if args.scene_evidence is not None:
        from .static_scenario_evidence import verify_blockage, verify_reconfiguration
        scenario = config.values["scenario"]
        if scenario == "S4":
            manifest["scene_evidence"] = verify_reconfiguration(config, args.scene_evidence)
        elif scenario == "S5":
            manifest["scene_evidence"] = verify_blockage(config, args.scene_evidence)
        else:
            raise ValueError("--scene-evidence 适用于 S4 或 S5")
    manifest["resources"]["experiment_configuration"] = {
        "path": str(source_configuration), "sha256": content_hash(source_configuration),
    }
    write_json(directory / "manifest.json", manifest)
    environment = dict(
        os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1",
        TMPDIR=str(directory / "tmp"), ROS_LOG_DIR=str(directory / "ros-log"),
        PYTHONPATH=str(Path(__file__).resolve().parents[1]) + os.pathsep + os.environ.get("PYTHONPATH", ""),
    )
    processes, streams = {}, []
    overrides = [
        value for name, setting in config.overrides.items()
        for value in (f"--{name.replace('_', '-')}", str(setting))
    ]
    failure = None
    outcome = {"termination": "interrupted", "reason": "supervisor_interrupted"}
    lifecycle = {"startup_order": ["controller", "plant"]}
    try:
        for role in ("controller", "plant"):
            stream = (directory / f"{role}.log").open("w", encoding="utf-8")
            streams.append(stream)
            process = subprocess.Popen(
                [sys.executable, "-m", "robot_safecontrol_moveit.static_experiment",
                 "worker", "--role", role, "--config", str(source_configuration),
                 "--resource-root", str(config.resource_root), "--output", str(directory), *overrides],
                env=environment, stdout=stream, stderr=subprocess.STDOUT,
            )
            processes[role] = process
            deadline = time.monotonic() + WORKER_STARTUP_TIMEOUT_S
            while not (directory / f"{role}.ready").exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"{role} 启动失败，检查 {role}.log")
                time.sleep(SUPERVISOR_POLL_S)
        deadline = time.monotonic() + config.values["timeout_s"]
        lifecycle.update(run_started_monotonic_s=time.monotonic(), deadline_monotonic_s=deadline)
        while processes["controller"].poll() is None:
            budget_results = [
                json.loads(path.read_text()) for role in ("controller", "plant")
                if (path := directory / f"{role}_result.json").exists()
            ]
            exhausted = next((row for row in budget_results
                              if row["termination"] == "budget_exhausted"), None)
            if exhausted is not None:
                outcome = exhausted
                break
            if processes["plant"].poll() is not None:
                raise RuntimeError("plant 在控制器结束前退出")
            if time.monotonic() >= deadline:
                outcome = {"termination": "timeout", "reason": "run_timeout"}
                break
            time.sleep(SUPERVISOR_POLL_S)
        else:
            if processes["controller"].returncode != 0:
                raise RuntimeError("controller 异常退出")
            outcome = json.loads((directory / "controller_result.json").read_text())
            time.sleep(config.values["settle_s"])
    except BaseException as error:
        outcome = {"termination": "fault", "reason": repr(error)}
        failure = error
    finally:
        lifecycle["stop_requested_monotonic_s"] = time.monotonic()
        _stop_processes(list(processes.values()), directory)
        lifecycle["workers_exited_monotonic_s"] = time.monotonic()
        for stream in streams:
            stream.close()
        outcome["worker_exit_codes"] = [
            processes[role].returncode for role in ("plant", "controller") if role in processes
        ]
        outcome["lifecycle"] = lifecycle
        budget_results = [
            json.loads(path.read_text()) for role in ("controller", "plant")
            if (path := directory / f"{role}_result.json").exists()
        ]
        exhausted = next((row for row in budget_results
                          if row["termination"] == "budget_exhausted"), None)
        if exhausted is not None:
            outcome.update(termination="budget_exhausted", reason=exhausted["reason"])
        outcome["ready_sha256"] = {
            role: content_hash(directory / f"{role}.ready")
            for role in ("plant", "controller") if (directory / f"{role}.ready").exists()
        }
        outcome["recording_statistics_sha256"] = {
            role: content_hash(directory / f"{role}.recording.json")
            for role in ("plant", "controller") if (directory / f"{role}.recording.json").exists()
        }
        write_json(directory / "result.json", outcome)
        for role in ("plant", "controller"):
            database = directory / f"{role}.sqlite3"
            if database.exists():
                finalize_recording(database)
    if any(content_hash(Path(path)) != expected
           for path, expected in manifest["source_sha256"].items()):
        raise RuntimeError("运行期间代码发生变化")
    report = evaluate_run(directory, resource_root=config.resource_root)
    print(f"反馈报告：{directory / 'feedback_report.json'}", flush=True)
    if failure is not None:
        raise failure
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="静态障碍物仿真实验")
    parser.add_argument("mode", choices=(
        "run", "replay", "show", "worker", "prepare", "witness", "witness-replay", "benchmark",
        "freeze", "batch", "paired-report", "report", "compact",
    ))
    parser.add_argument("--resource-root", type=Path, default=default_portable_root().parent)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--domain", type=int, default=198)
    parser.add_argument("--role", choices=("plant", "controller"))
    parser.add_argument("--method", choices=tuple(METHODS))
    parser.add_argument("--solver-mode", choices=("separated", "joint", "sns"))
    parser.add_argument("--recovery-mode", choices=("basic", "improved"))
    parser.add_argument("--period-s", type=float, choices=(.02, .04, .05, .1))
    parser.add_argument("--feed-factor", type=float)
    parser.add_argument("--joint-travel-scale", type=float, choices=(1., .7, .4))
    parser.add_argument("--clearance-reserve-m", type=float)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--save-samples", action="store_true",
                        help="重放时保存逐反馈派生数组")
    parser.add_argument("--scene-selection", type=Path)
    parser.add_argument("--witness-request", type=Path)
    parser.add_argument("--scene-evidence", type=Path)
    parser.add_argument("--catalog", type=Path)
    parser.add_argument("--period-study", type=Path)
    parser.add_argument("--evidence-index", type=Path)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--report-python", type=Path, default=Path(sys.executable))
    parser.add_argument("--figures-output", type=Path)
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232:
        parser.error("--domain 必须在 0–232 范围")
    if args.mode == "compact":
        if args.source is None or args.output is None:
            parser.error("compact 需要 --source 和尚不存在的 --output")
        from .static_experiment_storage import compact_run
        compact_run(args.source, args.output)
        evaluate_run(args.output, resource_root=args.resource_root)
        print(f"已转换并重算：{args.output}", flush=True)
        return 0
    if args.mode == "report":
        if args.output is None:
            parser.error("report 需要 --output 指定已经评价的运行目录")
        command = [
            str(args.report_python.absolute()), "-m",
            "robot_safecontrol_moveit.static_experiment_reporting",
            "--run", str(args.output), "--resource-root", str(args.resource_root),
        ]
        if args.figures_output is not None:
            command.extend(["--output", str(args.figures_output)])
        return subprocess.run(
            command, check=False,
            env={**os.environ, "PYTHONPATH": os.pathsep.join(filter(None, (
                str(Path(__file__).resolve().parents[1]), os.environ.get("PYTHONPATH"),
            )))},
        ).returncode
    if args.mode in ("freeze", "batch", "paired-report"):
        from .static_formal_experiments import freeze_formal, run_formal, paired_report
        if args.output is None:
            parser.error("正式实验入口需要 --output")
        if args.mode == "freeze":
            if any(value is None for value in (args.catalog, args.period_study, args.evidence_index)):
                parser.error("freeze 需要 --catalog、--period-study 和 --evidence-index")
            manifest = freeze_formal(
                args.catalog, args.period_study, args.evidence_index, args.output, args.resource_root,
            )
            print(f"正式清单：{len(manifest['runs'])} 个组合", flush=True)
            return 0
        report = (
            run_formal(args.output, resource_root=args.resource_root, domain=args.domain, limit=args.limit)
            if args.mode == "batch" else paired_report(args.output)
        )
        print(f"正式记录：{report['recorded_runs']}/{report['planned_runs']}；"
              f"D 组功能：{report['d_functional_verdict']}", flush=True)
        return 0 if report["collection_complete"] and report["d_functional_verdict"] == "pass" else 1
    if args.mode == "replay":
        if args.output is None:
            parser.error("replay 需要 --output")
        report = evaluate_run(
            args.output, resource_root=args.resource_root, save_samples=args.save_samples,
        )
        print(f"重算结果：{report['verdict']}", flush=True)
        return 0 if report["verdict"] == "pass" else 1
    config = ExperimentConfig.load(
        args.config or args.resource_root / "config/static_experiment_s0.yaml",
        resource_root=args.resource_root, solver_mode=args.solver_mode, recovery_mode=args.recovery_mode,
        method=args.method, period_s=args.period_s, feed_factor=args.feed_factor,
        joint_travel_scale=args.joint_travel_scale,
        clearance_reserve_m=args.clearance_reserve_m,
    )
    if args.mode == "show":
        from .mujoco_viewer_with_cylinder import main as viewer_main
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        return viewer_main([
            "--ros-args", "-p", f"static_experiment_config:={config.source}",
            "-p", f"static_experiment_resource_root:={config.resource_root}",
        ])
    if args.output is None:
        parser.error("run 和 worker 需要 --output")
    if args.mode == "benchmark":
        from .static_period_measurement import run_period_study
        report = run_period_study(config, args.output, args.domain)
        print(f"周期测量：{report['status']}；共同周期：{report['selected_period_s']}", flush=True)
        return 0 if report["status"] == "pass" else 1
    if args.mode == "witness-replay":
        from .static_scenario_evidence import verify_reconfiguration
        report = verify_reconfiguration(config, args.output)
        write_json(args.output / "verification.json", report)
        print(f"构型见证重放：{report['status']}", flush=True)
        return 0
    if args.mode == "prepare":
        if args.baseline is None:
            parser.error("prepare 需要 --baseline")
        from .static_experiment_campaign import prepare_development
        catalog = prepare_development(config, args.baseline, args.output, args.scene_selection)
        print(f"开发场景：{args.output / 'catalog.json'}；{len(catalog['scenarios'])} 个配置", flush=True)
        return 0
    if args.mode == "witness":
        if args.witness_request is None:
            parser.error("witness 需要 --witness-request")
        from .static_reconfiguration_evidence import run_witness_request
        report = run_witness_request(config, args.witness_request, args.output)
        print(f"独立构型见证：{report['status']}", flush=True)
        return 0 if report["status"] == "proven" else 1
    if args.mode == "worker":
        if args.role is None:
            parser.error("worker 需要 --role")
        _worker(args, config)
        return 0
    report = run_experiment(args, config)
    return 0 if report["verdict"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
