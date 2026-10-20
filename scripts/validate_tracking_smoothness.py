import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from robot_safecontrol_moveit.oscbf_controller import OscbfController
from robot_safecontrol_moveit.oscbf_plant import OscbfPlant
from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from measure_qos_latency import START_POSITION


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


class RecordingPlant(OscbfPlant):
    def __init__(self):
        self.samples = []
        self.commands = []
        parameter = rclpy.parameter.Parameter
        values = yaml.safe_load((ROOT / "config/oscbf_plant.yaml").read_text(encoding="utf-8"))["/**"]["ros__parameters"]
        values.update(portable_oscbf_root=str(ROOT / "portable_oscbf"), start_position=START_POSITION)
        super().__init__(parameter_overrides=[parameter(name, value=value) for name, value in values.items()])

    def _tick(self):
        super()._tick()
        self.samples.append(np.concatenate((
            [time.monotonic(), len(self.samples) / self._frequency_hz],
            self.state, self._driver.velocity, self._driver.acceleration,
        )))

    def _on_command(self, message):
        super()._on_command(message)
        self.commands.append([time.monotonic(), *message.position])


def worker(args):
    directory = args.output.resolve()
    rclpy.init()
    if args.role == "plant":
        node = RecordingPlant()
        write_json(directory / "plant_parameters.json", {
            "publish_frequency_hz": node._frequency_hz,
            "jerk_time": float(node.get_parameter("jerk_time").value),
            "minimum_duration_s": float(node.get_parameter("minimum_duration_s").value),
        })
    else:
        parameter = rclpy.parameter.Parameter
        node = OscbfController(parameter_overrides=[
            parameter("production_config_yaml", value=str(ROOT / "config/oscbf_controller.yaml")),
            parameter("portable_oscbf_root", value=str(ROOT / "portable_oscbf")),
            parameter("trajectory_mat", value=str(ROOT / "data/nurbs/ik_input.mat")),
            parameter("portable_config_yaml", value=str(ROOT / "portable_oscbf/config/nineaxis.yaml")),
            parameter("perf_report_path", value=str(directory / "controller_perf.md")),
        ])
    (directory / f"{args.role}.ready").write_text("ready", encoding="utf-8")
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.role == "controller" and node.execution_state == "holding":
                snapshot = node.progress_snapshot()
                write_json(directory / "completion.json", {
                    "started_monotonic_s": started,
                    "completed_monotonic_s": time.monotonic(),
                    "termination": snapshot["termination"],
                    "steps": snapshot["steps"],
                    "performance": node._run.performance_snapshot(),
                })
                if snapshot["termination"] != "completed":
                    raise RuntimeError(f"tracking did not complete: {snapshot['termination']}")
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if args.role == "plant":
            np.savez_compressed(directory / "plant.npz", samples=np.asarray(node.samples), commands=np.asarray(node.commands))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run(args):
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "tmp").mkdir()
    sources = ["config/oscbf_controller.yaml", "config/oscbf_plant.yaml",
               "src/robot_safecontrol_moveit/oscbf_plant.py",
               "src/robot_safecontrol_moveit/tracking_run.py",
               "src/robot_safecontrol_moveit/tracking_evaluator.py",
               "src/robot_safecontrol_moveit/tracking_contract.py",
               "portable_oscbf/work/jax_kernel_factory.py",
               "portable_oscbf/work/tool_axis_task.py",
               "portable_oscbf/work/jax_path_following.py",
               "portable_oscbf/work/driver_simulator.py",
               "portable_oscbf/config/actuator_modules.yaml",
               "scripts/validate_tracking_smoothness.py"]
    source_hashes = {path: hashlib.sha256((ROOT / path).read_bytes()).hexdigest() for path in sources}
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
                       TMPDIR=str(directory / "tmp"), ROS_LOG_DIR=str(directory / "ros-log"))
    processes = []
    streams = []
    try:
        for role in ("plant", "controller"):
            stream = (directory / f"{role}.log").open("w", encoding="utf-8")
            streams.append(stream)
            process = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve()), "worker", "--role", role,
                 "--output", str(directory)], cwd=ROOT, env=environment,
                stdout=stream, stderr=subprocess.STDOUT,
            )
            processes.append(process)
            deadline = time.monotonic() + 180
            while not (directory / f"{role}.ready").exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"{role} startup failed; inspect {directory / (role + '.log')}")
                time.sleep(0.1)
        deadline = time.monotonic() + args.timeout
        while processes[1].poll() is None:
            if processes[0].poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("simulation stopped or exceeded its completion timeout")
            time.sleep(0.2)
        if processes[1].returncode != 0 or not (directory / "completion.json").exists():
            raise RuntimeError("controller did not produce a successful completion record")
        # 保留最终目标后的执行器状态，用于检查静止时速度与加速度的收敛。
        time.sleep(2.0)
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            if process.poll() is None:
                process.wait(timeout=20)
        for stream in streams:
            stream.close()
    if any(process.returncode != 0 for process in processes):
        raise RuntimeError("simulation process exited with an error")
    if source_hashes != {path: hashlib.sha256((ROOT / path).read_bytes()).hexdigest() for path in sources}:
        raise RuntimeError("source files changed during the simulation")
    write_json(directory / "sources.json", {
        "source_sha256": source_hashes,
        "command": sys.argv, "start_position": START_POSITION,
        "measurement": "two ROS processes; simulated position, velocity and acceleration; separate wall and simulation times",
    })
    validate_motion(directory)
    print(f"完整跟踪记录：{directory}", flush=True)


def validate_motion(directory):
    bootstrap_portable(ROOT / "portable_oscbf")
    from work.actuator_limits import load_actuator_limit_profile

    with np.load(directory / "plant.npz", allow_pickle=False) as archive:
        samples = archive["samples"]
    if samples.ndim != 2 or samples.shape[1] != 29 or len(samples) < 100:
        raise RuntimeError("incomplete actuator state recording")
    if not np.all(np.isfinite(samples)) or np.any(np.diff(samples[:, :2], axis=0) <= 0):
        raise RuntimeError("actuator state values or timestamps are invalid")
    profile = load_actuator_limit_profile()
    velocity, acceleration = samples[:, 11:20], samples[:, 20:29]
    jerk = np.diff(acceleration, axis=0) / np.diff(samples[:, 1])[:, None]
    limits = profile.acceleration_limits
    plant_parameters = json.loads((directory / "plant_parameters.json").read_text(encoding="utf-8"))
    result = {
        "velocity_max_ratio": float(np.max(np.abs(velocity) / profile.velocity_limits)),
        "acceleration_max_ratio": float(np.max(np.abs(acceleration) / limits)),
        "jerk_max_ratio": float(np.max(np.abs(jerk) / (limits / plant_parameters["jerk_time"]))),
        "final_velocity_max": float(np.max(np.abs(velocity[-50:]))),
        "final_acceleration_max": float(np.max(np.abs(acceleration[-50:]))),
        "samples": len(samples),
    }
    if any(result[name] > 1 + 1e-8 for name in ("velocity_max_ratio", "acceleration_max_ratio", "jerk_max_ratio")):
        raise RuntimeError(f"actuator kinematic limits exceeded: {result}")
    if result["final_velocity_max"] > 1e-6 or result["final_acceleration_max"] > 1e-6:
        raise RuntimeError(f"actuator did not settle after the final hold: {result}")
    write_json(directory / "motion_validation.json", result)
    report = json.loads((directory / "tracking_report.json").read_text(encoding="utf-8"))
    if not report["completed"] or not report["full_path_covered"] or report["task_verdict"] != "pass":
        raise RuntimeError("tracking interval or task accuracy did not pass; inspect tracking_report.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["run", "worker"], nargs="?", default="run")
    parser.add_argument("--role", choices=["plant", "controller"])
    parser.add_argument("--output", type=Path, default=ROOT / "output/oscillation_fix/verified")
    parser.add_argument("--domain", type=int, default=196)
    parser.add_argument("--timeout", type=float, default=300)
    arguments = parser.parse_args()
    if arguments.mode == "worker":
        worker(arguments)
    else:
        run(arguments)
