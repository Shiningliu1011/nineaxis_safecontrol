import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import JointState

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from robot_safecontrol_moveit.oscbf_controller import OscbfController
from robot_safecontrol_moveit.oscbf_plant import OscbfPlant
from robot_safecontrol_moveit.ros_conventions import (
    COMMAND_STREAM_QOS_DEPTH,
    STATE_STREAM_QOS_DEPTH,
)

# 由现有完整闭环测试的 _path_start_configuration() 求得。
START_POSITION = [
    0.4660888592305836, -1.3046859102762256, 0.4878612339833056,
    1.383984286344668, 1.1002338370759628, 0.4395349231726782,
    0.8587900071557233, -1.4530208372077718, -0.9479481508485612,
]
SOURCE_FILES = (
    "scripts/measure_qos_latency.py",
    "src/robot_safecontrol_moveit/ros_conventions.py",
    "src/robot_safecontrol_moveit/oscbf_controller.py",
    "src/robot_safecontrol_moveit/oscbf_plant.py",
    "config/oscbf_controller.yaml",
    "portable_oscbf/config/nineaxis.yaml",
    "data/nurbs/ik_input.mat",
)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _benchmark_qos(depth: int) -> QoSProfile:
    if depth not in (5, 20):
        raise ValueError(f"QoS 缓存条数必须为 5 或 20：{depth}")
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class RecordingPublisher:
    def __init__(self, publisher, record) -> None:
        self._publisher = publisher
        self._record = record

    def publish(self, message: JointState) -> None:
        self._record(message)
        self._publisher.publish(message)


class MeasuredController(OscbfController):
    def __init__(
        self, state_depth: int, command_depth: int, production_qos: bool,
        ready_path: Path, trace_path: Path,
    ) -> None:
        self._state_depth = state_depth
        self._command_depth = command_depth
        self._production_qos = production_qos
        self._trace_path = trace_path
        self._latest_state_stamp_ns: int | None = None
        self._last_traced_state_stamp_ns: int | None = None
        self._trace: list[list[int]] = []
        self._received_states: list[list[int]] = []
        overrides = [
            rclpy.parameter.Parameter(
                "production_config_yaml",
                value=str(REPO_ROOT / "config" / "oscbf_controller.yaml"),
            ),
            rclpy.parameter.Parameter(
                "portable_oscbf_root",
                value=str(REPO_ROOT / "portable_oscbf"),
            ),
            rclpy.parameter.Parameter(
                "trajectory_mat",
                value=str(REPO_ROOT / "data" / "nurbs" / "ik_input.mat"),
            ),
            rclpy.parameter.Parameter(
                "portable_config_yaml",
                value=str(REPO_ROOT / "portable_oscbf" / "config" / "nineaxis.yaml"),
            ),
            rclpy.parameter.Parameter(
                "perf_report_path", value=str(trace_path.parent / "controller_perf.md"),
            ),
        ]
        super().__init__(parameter_overrides=overrides)
        self._publisher = RecordingPublisher(self._publisher, self._record_command)
        ready_path.write_text("ready\n", encoding="utf-8")

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        if (not self._production_qos and msg_type is JointState
                and topic == self._runtime_config["joint_state_topic"]):
            qos_profile = _benchmark_qos(self._state_depth)
        return super().create_subscription(
            msg_type, topic, callback, qos_profile, **kwargs
        )

    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        if (not self._production_qos and msg_type is JointState
                and topic == self._runtime_config["publish_joint_state_topic"]):
            qos_profile = _benchmark_qos(self._command_depth)
        return super().create_publisher(msg_type, topic, qos_profile, **kwargs)

    def _joint_state_callback(self, message: JointState) -> None:
        self._received_states.append([
            _stamp_ns(message.header.stamp), time.monotonic_ns(),
        ])
        previous = self._last_state_time
        super()._joint_state_callback(message)
        if self._last_state_time != previous:
            self._latest_state_stamp_ns = _stamp_ns(message.header.stamp)

    def _record_command(self, message: JointState) -> None:
        state_stamp = self._latest_state_stamp_ns
        if self._hold_q is not None or state_stamp is None:
            return
        if state_stamp == self._last_traced_state_stamp_ns:
            return
        self._trace.append([
            state_stamp,
            _stamp_ns(message.header.stamp),
            time.monotonic_ns(),
        ])
        self._last_traced_state_stamp_ns = state_stamp

    def write_trace(self) -> None:
        with self._trace_path.open("w", encoding="utf-8") as stream:
            json.dump(
                {"states_received": self._received_states, "commands": self._trace},
                stream,
                allow_nan=False,
            )


class MeasuredPlant(OscbfPlant):
    def __init__(
        self, command_depth: int, state_publish_depth: int,
        production_qos: bool,
        ready_path: Path, trace_path: Path,
    ) -> None:
        self._command_depth = command_depth
        self._state_publish_depth = state_publish_depth
        self._production_qos = production_qos
        self._trace_path = trace_path
        self._state_trace: list[list[int]] = []
        self._command_trace: list[list[int]] = []
        overrides = [
            rclpy.parameter.Parameter(
                "portable_oscbf_root",
                value=str(REPO_ROOT / "portable_oscbf"),
            ),
            rclpy.parameter.Parameter("start_position", value=START_POSITION),
        ]
        super().__init__(parameter_overrides=overrides)
        self._state_pub = RecordingPublisher(self._state_pub, self._record_state)
        ready_path.write_text("ready\n", encoding="utf-8")

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        if (not self._production_qos and msg_type is JointState
                and topic == str(self.get_parameter("command_topic").value)):
            qos_profile = _benchmark_qos(self._command_depth)
        return super().create_subscription(
            msg_type, topic, callback, qos_profile, **kwargs
        )

    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        if (not self._production_qos and msg_type is JointState
                and topic == str(self.get_parameter("state_topic").value)):
            qos_profile = _benchmark_qos(self._state_publish_depth)
        return super().create_publisher(msg_type, topic, qos_profile, **kwargs)

    def _record_state(self, message: JointState) -> None:
        self._state_trace.append([
            _stamp_ns(message.header.stamp), time.monotonic_ns(),
        ])

    def _on_command(self, message: JointState) -> None:
        received_ns = time.monotonic_ns()
        previous = self._commands_received
        super()._on_command(message)
        if self._commands_received != previous:
            self._command_trace.append([
                _stamp_ns(message.header.stamp), received_ns,
            ])

    def write_trace(self) -> None:
        with self._trace_path.open("w", encoding="utf-8") as stream:
            json.dump(
                {"states": self._state_trace, "commands": self._command_trace},
                stream,
                allow_nan=False,
            )


def _run_worker(args: argparse.Namespace) -> None:
    rclpy.init()
    stop_path = Path(args.case_dir) / "stop"
    ready_path = Path(args.case_dir) / f"{args.role}.ready"
    trace_path = Path(args.case_dir) / f"{args.role}.json"
    if args.role == "plant":
        node = MeasuredPlant(
            args.command_depth, args.state_publish_depth,
            args.production_qos,
            ready_path, trace_path,
        )
    else:
        node = MeasuredController(
            args.state_depth, args.command_depth, args.production_qos,
            ready_path, trace_path,
        )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop_path.exists():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.write_trace()
        if args.role == "controller":
            node.write_perf_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _wait_ready(path: Path, process: subprocess.Popen, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while not path.exists():
        if process.poll() is not None:
            raise RuntimeError(f"子进程就绪前退出：{path}，code={process.returncode}")
        if time.monotonic() >= deadline:
            raise RuntimeError(f"子进程启动超时：{path}")
        time.sleep(0.2)


def _start_worker(
    role: str, case_dir: Path, domain_id: int, state_depth: int,
    command_depth: int, state_publish_depth: int, production_qos: bool,
) -> subprocess.Popen:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    command = [
        sys.executable, str(Path(__file__).resolve()), "worker",
        "--role", role,
        "--case-dir", str(case_dir),
        "--state-depth", str(state_depth),
        "--command-depth", str(command_depth),
        "--state-publish-depth", str(state_publish_depth),
    ]
    if production_qos:
        command.append("--production-qos")
    with (case_dir / f"{role}.log").open("w", encoding="utf-8") as log:
        return subprocess.Popen(
            command, cwd=REPO_ROOT, env=environment,
            stdout=log, stderr=subprocess.STDOUT,
        )


def _stop_workers(processes: list[subprocess.Popen], case_dir: Path) -> None:
    (case_dir / "stop").touch()
    for process in processes:
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def _distribution(values: list[float]) -> dict[str, float]:
    if not values:
        raise RuntimeError("没有完整的时延样本")
    samples = np.asarray(values, dtype=float)
    return {
        "p50_ms": float(np.percentile(samples, 50)),
        "p95_ms": float(np.percentile(samples, 95)),
        "p99_ms": float(np.percentile(samples, 99)),
        "max_ms": float(np.max(samples)),
    }


def _summarize_case(case_dir: Path, start_ns: int, stop_ns: int) -> dict:
    with (case_dir / "plant.json").open(encoding="utf-8") as stream:
        plant = json.load(stream)
    with (case_dir / "controller.json").open(encoding="utf-8") as stream:
        controller = json.load(stream)
    states = {stamp: sent for stamp, sent in plant["states"]}
    received_states = {
        stamp: received for stamp, received in controller["states_received"]
    }
    arrivals = {stamp: received for stamp, received in plant["commands"]}
    if len(states) != len(plant["states"]):
        raise RuntimeError(f"状态消息身份重复：{case_dir}")
    if len(received_states) != len(controller["states_received"]):
        raise RuntimeError(f"接收的状态消息身份重复：{case_dir}")
    if len(arrivals) != len(plant["commands"]):
        raise RuntimeError(f"指令消息身份重复：{case_dir}")
    # 停止前留出 100 ms，使最后一批发送的状态有机会进入订阅回调。
    delivery_window = {
        stamp for stamp, sent in states.items()
        if start_ns <= sent < stop_ns - 100_000_000
    }
    delivered_states = delivery_window & received_states.keys()
    used_states = {
        state_stamp for state_stamp, _, done_ns in controller["commands"]
        if done_ns >= start_ns
    }
    state_to_done: list[float] = []
    done_to_arrival: list[float] = []
    total: list[float] = []
    missing_state = missing_arrival = noncausal = 0
    active_commands = 0
    for state_stamp, command_stamp, done_ns in controller["commands"]:
        if done_ns < start_ns:
            continue
        active_commands += 1
        sent_ns = states.get(state_stamp)
        received_ns = arrivals.get(command_stamp)
        if sent_ns is None:
            missing_state += 1
            continue
        if sent_ns < start_ns:
            continue
        if received_ns is None:
            missing_arrival += 1
            continue
        if not sent_ns <= done_ns <= received_ns:
            noncausal += 1
            continue
        state_to_done.append((done_ns - sent_ns) / 1_000_000)
        done_to_arrival.append((received_ns - done_ns) / 1_000_000)
        total.append((received_ns - sent_ns) / 1_000_000)
    return {
        "state_messages": sum(sent >= start_ns for _, sent in plant["states"]),
        "state_delivery_window_messages": len(delivery_window),
        "states_received": len(delivered_states),
        "states_not_received": len(delivery_window - received_states.keys()),
        "states_received_without_command": len(delivered_states - used_states),
        "active_commands": active_commands,
        "received_commands": sum(received >= start_ns for _, received in plant["commands"]),
        "complete_samples": len(total),
        "missing_state": missing_state,
        "missing_arrival": missing_arrival,
        "noncausal": noncausal,
        "state_to_controller": _distribution(state_to_done),
        "controller_to_plant": _distribution(done_to_arrival),
        "end_to_end": _distribution(total),
    }


def _run_case(
    case_dir: Path, domain_id: int, state_depth: int, command_depth: int,
    state_publish_depth: int, production_qos: bool,
    warmup_s: float, run_s: float,
    min_samples: int,
) -> dict:
    case_dir.mkdir(parents=True, exist_ok=False)
    processes: list[subprocess.Popen] = []
    try:
        plant = _start_worker(
            "plant", case_dir, domain_id, state_depth,
            command_depth, state_publish_depth, production_qos,
        )
        processes.append(plant)
        _wait_ready(case_dir / "plant.ready", plant, 120)
        controller = _start_worker(
            "controller", case_dir, domain_id, state_depth,
            command_depth, state_publish_depth, production_qos,
        )
        processes.append(controller)
        _wait_ready(case_dir / "controller.ready", controller, 240)
        time.sleep(warmup_s)
        start_ns = time.monotonic_ns()
        deadline = time.monotonic() + run_s
        while time.monotonic() < deadline:
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"子进程在测量期间退出：{case_dir}，"
                        f"pid={process.pid}, code={process.returncode}"
                    )
            time.sleep(0.2)
        stop_ns = time.monotonic_ns()
    finally:
        _stop_workers(processes, case_dir)
    for process in processes:
        if process.returncode != 0:
            raise RuntimeError(
                f"子进程运行失败：{case_dir}，pid={process.pid}，"
                f"code={process.returncode}"
            )
    result = _summarize_case(case_dir, start_ns, stop_ns)
    result.update({
        "state_subscriber_depth": state_depth,
        "state_publisher_depth": state_publish_depth,
        "command_depth": command_depth,
        "production_qos": production_qos,
        "domain_id": domain_id,
        "warmup_seconds": warmup_s,
        "run_seconds": run_s,
    })
    if result["complete_samples"] < min_samples:
        raise RuntimeError(
            f"完整样本数量不足：{result['complete_samples']} "
            f"< {min_samples}，目录：{case_dir}"
        )
    (case_dir / "summary.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return result


def _run_matrix(args: argparse.Namespace) -> None:
    cases = [
        (state_depth, command_depth)
        for state_depth in args.state_depths
        for command_depth in args.command_depths
    ]
    if args.reverse:
        cases.reverse()
    if not cases or any(depth not in (5, 20) for pair in cases for depth in pair):
        raise ValueError("状态流与指令流缓存条数必须为 5 或 20")
    base_domain = args.base_domain
    if base_domain is None:
        base_domain = 170 + os.getpid() % 45
    if not 0 <= base_domain <= 233 - len(cases):
        raise ValueError(f"ROS domain 超出可用范围：{base_domain}")
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = REPO_ROOT / "output" / "i13-qos-latency" / f"{run_id}-{os.getpid()}"
    run_dir.mkdir(parents=True, exist_ok=False)
    run_summary = {
        "commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
        ).strip(),
        "utc_started": run_id,
        "host": platform.uname()._asdict(),
        "python": sys.version,
        "rmw": get_rmw_implementation_identifier(),
        "start_position": START_POSITION,
        "source_sha256": {
            path: hashlib.sha256((REPO_ROOT / path).read_bytes()).hexdigest()
            for path in SOURCE_FILES
        },
        "cases": [],
    }
    for index, (state_depth, command_depth) in enumerate(cases):
        case_dir = run_dir / f"state-{state_depth}-command-{command_depth}"
        print(f"测量 {case_dir.name}，ROS_DOMAIN_ID={base_domain + index}", flush=True)
        result = _run_case(
            case_dir, base_domain + index, state_depth, command_depth,
            args.state_publish_depth, args.production_qos, args.warmup_seconds,
            args.run_seconds, args.min_samples,
        )
        run_summary["cases"].append(result)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
        (run_dir / "summary.json").write_text(
            json.dumps(run_summary, ensure_ascii=False, indent=2, allow_nan=False)
            + "\n",
            encoding="utf-8",
        )
    print(f"测量结果：{run_dir / 'summary.json'}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", nargs="?", choices=("run", "worker"), default="run")
    parser.add_argument("--role", choices=("plant", "controller"))
    parser.add_argument("--case-dir")
    parser.add_argument("--state-depth", type=int)
    parser.add_argument("--command-depth", type=int)
    parser.add_argument("--state-depths", nargs="+", type=int, default=[5, 20])
    parser.add_argument("--command-depths", nargs="+", type=int, default=[5, 20])
    parser.add_argument("--state-publish-depth", type=int, default=20)
    parser.add_argument("--base-domain", type=int)
    parser.add_argument("--warmup-seconds", type=float, default=2.0)
    parser.add_argument("--run-seconds", type=float, default=20.0)
    parser.add_argument("--min-samples", type=int, default=300)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--production-qos", action="store_true")
    args = parser.parse_args()
    if args.mode == "worker":
        if not args.role or not args.case_dir:
            parser.error("worker 需要 --role 和 --case-dir")
        _run_worker(args)
    else:
        if args.warmup_seconds < 0 or args.run_seconds <= 0:
            parser.error("测量时长参数无效")
        if args.production_qos and (
            args.state_depths != [STATE_STREAM_QOS_DEPTH]
            or args.command_depths != [COMMAND_STREAM_QOS_DEPTH]
            or args.state_publish_depth != STATE_STREAM_QOS_DEPTH
        ):
            parser.error("生产 QoS 缓存条数必须与 ros_conventions 一致")
        _run_matrix(args)


if __name__ == "__main__":
    main()
