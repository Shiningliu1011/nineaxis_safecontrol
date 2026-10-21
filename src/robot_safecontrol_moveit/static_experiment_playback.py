from dataclasses import dataclass
from contextlib import ExitStack, nullcontext
import hashlib
import json
import os
from pathlib import Path
import signal
import time
import uuid

import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
import yaml

from .mujoco_viewer_with_cylinder import MuJoCoJointStateViewer
from .robot_spec import DEFAULT_JOINT_NAMES
from .static_experiment_config import content_hash
from .static_experiment_provenance import relocate_manifest, source_identity, validate_run_report
from .static_experiment_recording import read_stream, write_json


@dataclass(frozen=True)
class PlaybackRecording:
    directory: Path
    resource_root: Path
    manifest: dict
    feedback: tuple[dict, ...]
    identity: dict


def load_playback_recording(directory: Path, resource_root: Path) -> PlaybackRecording:
    directory = Path(directory).resolve(strict=True)
    resource_root = Path(resource_root).resolve(strict=True)
    original, report = validate_run_report(directory)
    manifest = relocate_manifest(original, resource_root)
    if manifest["config"]["mode"] != "sim":
        raise ValueError("保存反馈重放仅接受 sim 记录")
    encoded = json.dumps(manifest["config"], sort_keys=True, allow_nan=False).encode()
    if hashlib.sha256(encoded).hexdigest() != manifest["config_sha256"]:
        raise ValueError("重放配置身份不一致")
    for name, resource in manifest["resources"].items():
        if name != "experiment_configuration" and content_hash(Path(resource["path"])) != resource["sha256"]:
            raise ValueError(f"重放资源身份不一致：{resource['path']}")
    feedback = tuple(read_stream(directory / "plant.sqlite3", "feedback"))
    if len(feedback) < 2:
        raise ValueError("重放需要至少两条真实反馈")
    times = np.asarray([row["simulation_s"] for row in feedback], dtype=float)
    if not np.all(np.isfinite(times)) or times[0] < 0 or np.any(np.diff(times) <= 0):
        raise ValueError("保存反馈的 simulation_s 必须有限且严格递增")
    for row in feedback:
        if not isinstance(row["ros_time_ns"], int) or row["ros_time_ns"] < 0:
            raise ValueError("保存反馈缺少有效原始 ROS 时间戳")
        for field in ("q", "velocity"):
            values = np.asarray(row["values"][field], dtype=float)
            if values.shape != (len(DEFAULT_JOINT_NAMES),) or not np.all(np.isfinite(values)):
                raise ValueError(f"保存反馈 {field} 必须包含九个有限数值")
    identity = {
        "run": str(directory), "manifest_sha256": report["manifest_sha256"],
        "report_sha256": content_hash(directory / "feedback_report.json"),
        "result_sha256": report["result_sha256"], "path_sha256": report["path_sha256"],
        "records_sha256": report["records_sha256"], "source_sha256": original["source_sha256"],
        "source_verdict": report["verdict"],
    }
    return PlaybackRecording(directory, resource_root, manifest, feedback, identity)


class PlaybackNode(Node):
    def __init__(self, recording: PlaybackRecording, *, context, time_scale: float = 1.) -> None:
        if not np.isfinite(time_scale) or time_scale <= 0:
            raise ValueError("time_scale 必须为有限正数")
        identifier = uuid.uuid4().hex
        super().__init__(f"static_playback_{identifier}", context=context)
        self.recording = recording
        self.time_scale = float(time_scale)
        self.topic = f"/static_experiment_playback/session_{identifier}/joint_states"
        # 保存数据的显示流采用可靠传输，现有 viewer 的 BEST_EFFORT 订阅保持兼容。
        try:
            self._publisher = self.create_publisher(
                JointState, self.topic, QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE),
            )
            self._timer = self.create_timer(.001, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self._timer.cancel()
        except BaseException:
            self.destroy_node()
            raise
        self.status = "ready"
        self.published_count = 0
        self._started = None
        self._finished = None
        self.maximum_lateness_s = 0.

    @property
    def ready(self) -> bool:
        return self._publisher.get_subscription_count() > 0

    def start(self) -> None:
        if self.status != "ready" or not self.ready:
            raise RuntimeError("重放需要显示订阅且只能启动一次")
        self.status = "playing"
        self.get_logger().info(f"STATIC_PLAYBACK_STARTED source={self.recording.directory} topic={self.topic}")
        self._timer.reset()

    def stop(self, reason: str = "interrupted") -> None:
        if self.status in ("ready", "playing"):
            self.status = reason
            self._finished = time.monotonic()
            self._timer.cancel()

    def summary(self) -> dict:
        frames = self.recording.feedback
        last = frames[self.published_count - 1] if self.published_count else None
        return {
            "status": self.status, "topic": self.topic, "time_scale": self.time_scale,
            "total_samples": len(frames), "published_samples": self.published_count,
            "first_simulation_s": frames[0]["simulation_s"],
            "last_simulation_s": None if last is None else last["simulation_s"],
            "scheduled_duration_s": (frames[-1]["simulation_s"] - frames[0]["simulation_s"]) * self.time_scale,
            "elapsed_s": None if self._started is None else (self._finished or time.monotonic()) - self._started,
            "maximum_lateness_s": self.maximum_lateness_s,
        }

    def _tick(self) -> None:
        frames = self.recording.feedback
        while self.published_count < len(frames):
            row = frames[self.published_count]
            now = time.monotonic()
            due = now if self._started is None else (
                self._started + (row["simulation_s"] - frames[0]["simulation_s"]) * self.time_scale
            )
            if now < due:
                return
            message = JointState()
            message.header.frame_id = "base_link"
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(row["ros_time_ns"], 1_000_000_000)
            message.name = list(DEFAULT_JOINT_NAMES)
            message.position = list(row["values"]["q"])
            message.velocity = list(row["values"]["velocity"])
            self._publisher.publish(message)
            if self._started is None:
                self._started = time.monotonic()
            self.maximum_lateness_s = max(self.maximum_lateness_s, now - due)
            self.published_count += 1
        self.status = "completed"
        self._finished = time.monotonic()
        self._timer.cancel()
        self.get_logger().info(f"STATIC_PLAYBACK_COMPLETED samples={self.published_count}")


def playback_run(directory: Path, output: Path, *, resource_root: Path,
                 domain: int, time_scale: float = 1., headless: bool = False) -> dict:
    if not 0 <= domain <= 232:
        raise ValueError("domain 必须在 0–232 范围")
    if not np.isfinite(time_scale) or time_scale <= 0:
        raise ValueError("time_scale 必须为有限正数")
    recording = load_playback_recording(directory, resource_root)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    config_path = output / "config.yaml"
    config_path.write_text(yaml.safe_dump(recording.manifest["config"], sort_keys=False), encoding="utf-8")
    write_json(output / "playback.json", {
        "schema_version": 1, "mode": "sim", "domain": domain,
        "source": recording.identity, "resource_root": str(recording.resource_root),
        "source_sha256": source_identity(recording.resource_root),
        "config_sha256": content_hash(config_path), "time_scale": time_scale,
        "headless": headless, "cpu_affinity": sorted(os.sched_getaffinity(0)),
    })
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    report = {"status": "failed", "nodes_destroyed": False}
    try:
        with ExitStack() as cleanup:
            for signum in (signal.SIGINT, signal.SIGTERM):
                previous = signal.signal(signum, _interrupt_playback)
                cleanup.callback(signal.signal, signum, previous)
            context = Context()
            rclpy.init(context=context, args=[], domain_id=domain,
                       signal_handler_options=SignalHandlerOptions.NO)
            cleanup.callback(context.shutdown)
            player = PlaybackNode(recording, context=context, time_scale=time_scale)
            cleanup.callback(player.destroy_node)
            viewer = MuJoCoJointStateViewer(context=context, parameter_overrides=[
                Parameter("static_experiment_config", value=str(config_path)),
                Parameter("static_experiment_resource_root", value=str(recording.resource_root)),
                Parameter("joint_state_topic", value=player.topic),
            ])
            cleanup.callback(viewer.destroy_node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(player)
            executor.add_node(viewer)
            cleanup.callback(executor.shutdown)
            try:
                with ExitStack() as window_cleanup:
                    window = None if headless else window_cleanup.enter_context(
                        mujoco.viewer.launch_passive(viewer.model, viewer.data),
                    )
                    deadline = time.monotonic() + 5.
                    while not player.ready and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.01)
                    if not player.ready:
                        raise RuntimeError("MuJoCo 显示订阅未连接")
                    player.start()
                    final_stamp = recording.feedback[-1]["ros_time_ns"]
                    drain_deadline = None
                    while True:
                        if window is not None and not window.is_running():
                            player.stop()
                            break
                        executor.spin_once(timeout_sec=.001)
                        with window.lock() if window is not None else nullcontext():
                            viewer.apply_latest_joint_state()
                        if window is not None:
                            window.sync()
                        if player.status == "completed":
                            if viewer.applied_joint_state_time_ns == final_stamp:
                                break
                            if drain_deadline is None:
                                drain_deadline = time.monotonic() + 5.
                            if time.monotonic() > drain_deadline:
                                raise RuntimeError("MuJoCo 未接收最后一条保存反馈")
                    if player.status == "completed":
                        positions = _model_position(viewer)
                        if not np.allclose(positions, recording.feedback[-1]["values"]["q"], atol=1e-12, rtol=0.):
                            raise RuntimeError("MuJoCo 末态与保存反馈不一致")
            except KeyboardInterrupt:
                player.stop()
            finally:
                report.update(player.summary())
                report["model_position"] = _model_position(viewer)
                report["applied_ros_time_ns"] = viewer.applied_joint_state_time_ns
        report["nodes_destroyed"] = True
    except BaseException as error:
        report["status"] = "failed"
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        write_json(output / "playback_result.json", report)
    return report


def _model_position(viewer: MuJoCoJointStateViewer) -> list[float]:
    return [float(viewer.data.qpos[viewer.model.joint(name).qposadr[0]]) for name in DEFAULT_JOINT_NAMES]


def _interrupt_playback(signum, frame) -> None:
    raise KeyboardInterrupt
