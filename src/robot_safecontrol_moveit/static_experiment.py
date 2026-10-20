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
from .static_experiment_recording import RunRecorder, finalize_recording, write_json
from .static_experiment_provenance import source_identity


def _worker(args, config: ExperimentConfig) -> None:
    bootstrap_portable(config.resource_root / "portable_oscbf")
    from .static_experiment_nodes import StaticExperimentController, StaticExperimentPlant

    directory = args.output.resolve()
    rclpy.init(args=[])
    node = None
    with RunRecorder(directory / f"{args.role}.sqlite3") as recorder:
        try:
            if args.role == "plant":
                node = StaticExperimentPlant(config, recorder)
            else:
                node = StaticExperimentController(config, recorder)
            (directory / f"{args.role}.ready").touch()
            while rclpy.ok():
                if (directory / "stop").exists() and (args.role != "plant" or node.stopped):
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
                if args.role == "controller" and node.execution_state == "holding":
                    snapshot = node.progress_snapshot()
                    write_json(directory / "controller_result.json", {
                        "termination": snapshot["termination"], "steps": snapshot["steps"],
                        "monotonic_s": time.monotonic(),
                    })
                    break
        except (KeyboardInterrupt, ExternalShutdownException):
            recorder.append("event", monotonic_s=time.monotonic(), values={"reason": "interrupted"})
        except BaseException as error:
            if args.role == "controller" and node is not None:
                node.record_fault(repr(error))
            recorder.append("event", monotonic_s=time.monotonic(),
                            values={"reason": "fault", "phase": "FAULT", "error": repr(error)})
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def _sources(config: ExperimentConfig) -> dict:
    return source_identity(config.resource_root)


def _stop_processes(processes: list, directory: Path) -> None:
    (directory / "stop").touch()
    for process in processes:
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
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
    processes, streams = [], []
    overrides = [
        value for name, setting in config.overrides.items()
        for value in (f"--{name.replace('_', '-')}", str(setting))
    ]
    failure = None
    outcome = {"termination": "interrupted", "reason": "supervisor_interrupted"}
    try:
        for role in ("plant", "controller"):
            stream = (directory / f"{role}.log").open("w", encoding="utf-8")
            streams.append(stream)
            process = subprocess.Popen(
                [sys.executable, "-m", "robot_safecontrol_moveit.static_experiment",
                 "worker", "--role", role, "--config", str(source_configuration),
                 "--resource-root", str(config.resource_root), "--output", str(directory), *overrides],
                env=environment, stdout=stream, stderr=subprocess.STDOUT,
            )
            processes.append(process)
            deadline = time.monotonic() + 180
            while not (directory / f"{role}.ready").exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"{role} 启动失败，检查 {role}.log")
                time.sleep(0.1)
        deadline = time.monotonic() + config.values["timeout_s"]
        while processes[1].poll() is None:
            if processes[0].poll() is not None:
                raise RuntimeError("plant 在控制器结束前退出")
            if time.monotonic() >= deadline:
                outcome = {"termination": "timeout", "reason": "run_timeout"}
                break
            time.sleep(0.1)
        else:
            if processes[1].returncode != 0:
                raise RuntimeError("controller 异常退出")
            outcome = json.loads((directory / "controller_result.json").read_text())
            time.sleep(config.values["settle_s"])
    except BaseException as error:
        outcome = {"termination": "fault", "reason": repr(error)}
        failure = error
    finally:
        _stop_processes(processes, directory)
        for stream in streams:
            stream.close()
        outcome["worker_exit_codes"] = [process.returncode for process in processes]
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
        "freeze", "batch", "paired-report", "report",
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
    if args.mode == "report":
        if args.output is None:
            parser.error("report 需要 --output 指定已经评价的运行目录")
        command = [
            str(args.report_python.resolve(strict=True)), "-m",
            "robot_safecontrol_moveit.static_experiment_reporting",
            "--run", str(args.output), "--resource-root", str(args.resource_root),
        ]
        if args.figures_output is not None:
            command.extend(["--output", str(args.figures_output)])
        return subprocess.run(
            command, check=False,
            env={**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])},
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
        report = evaluate_run(args.output, resource_root=args.resource_root)
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
