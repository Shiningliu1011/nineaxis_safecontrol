import json
import os
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from pathlib import Path

from construct import ConstructError

from .drempower_serial import DrEmpowerSerial
from .joint_table import require_hardware_qualification
from .joint_table_execution import JointTableExecutor
from .measurement_files import prepare_measurement_files
from .motor_stop_latch import MotorStopLatch
from .robot_spec import DEFAULT_JOINT_NAMES
from .single_joint_execution import (
    ArrivalWindow,
    MotionRejected,
    SingleJointMonitor,
    VendorFeedbackReader,
)

POSITION_NODES = {1: "prismatic", **{node: "revolute" for node in range(2, 10)}}


class JointTableDevice:
    def __init__(self, vendor: DrEmpowerSerial, executor: JointTableExecutor):
        if type(vendor) is not DrEmpowerSerial or vendor.position_nodes != POSITION_NODES:
            raise ValueError("必须提供明确的九轴位置串口连接")
        if executor.feedback_source != "hardware_feedback":
            raise ValueError("真实设备执行必须使用独立设备反馈")
        self.vendor, self.executor = vendor, executor
        self.reader = VendorFeedbackReader(vendor, executor.control, 0.2)
        self.commands_attempted = 0

    def prepare(self):
        for joint in DEFAULT_JOINT_NAMES:
            self.vendor.prepare_motion(self.executor.control._node(joint))
            self.reader.require_closed_loop(joint)

    def read_cycle(self):
        samples = {}
        for joint in DEFAULT_JOINT_NAMES:
            self.reader.require_closed_loop(joint)
            samples[joint] = self.reader.read(joint)
        return samples

    def send(self, commands, record):
        if [entry["joint"] for entry in commands] != list(DEFAULT_JOINT_NAMES):
            raise ValueError("位置命令必须完整按 J1–J9 顺序提供")
        for command in commands:
            joint, arguments = command["joint"], command["arguments"]
            self.reader.require_closed_loop(joint)
            self.executor.check_time(time.monotonic())
            record("command_requested", joint=joint, arguments=arguments)
            self.executor.check_time(time.monotonic())
            self.commands_attempted += 1
            self.vendor.set_angle(**arguments)
            record("command_call_returned", joint=joint)

    def hold_and_observe(self, record):
        if self.reader.in_transaction or self.vendor._failed:
            return {"stop_result": "external_stop_required", "stop_error": "serial_session_untrusted"}
        monitors = {}
        arrivals = {}
        try:
            samples = self.read_cycle()
            commands = []
            for joint in DEFAULT_JOINT_NAMES:
                sample = self.executor.windows[joint].add(samples[joint])
                limit = self.executor.limits[joint]
                monitor = SingleJointMonitor(self.executor.control._node(joint), limit)
                previous = self.executor.monitors[joint]
                monitor.last_request_s = previous.last_request_s
                monitor.last_received_s = previous.last_received_s
                monitor.last_angle_deg = previous.last_angle_deg
                monitor.validate_hold_feedback(sample, time.monotonic())
                monitors[joint] = monitor
                arrivals[joint] = ArrivalWindow(sample.state.angle_deg, limit)
                commands.append({"joint": joint, "arguments": self.executor.control.position_arguments(
                    joint, sample.state.angle_deg, limit)})
            # 只有全部位置可信时才形成保持请求；通信失效时禁止重新连接后继续发送。
            for command in commands:
                joint = command["joint"]
                if time.monotonic() - samples[joint].request_s > self.executor.limits[joint].feedback_timeout_s:
                    raise MotionRejected(f"{joint}:hold_feedback_timeout")
                record("hold_requested", **command)
                self.vendor.set_angle(**command["arguments"])
                record("hold_command_call_returned", joint=joint)
            until = time.monotonic() + max(limit.hold_timeout_s for limit in self.executor.limits.values())
            settled_since = None
            while time.monotonic() < until:
                samples = self.read_cycle()
                for joint in DEFAULT_JOINT_NAMES:
                    sample = self.executor.windows[joint].add(samples[joint])
                    monitors[joint].validate_feedback(sample, time.monotonic())
                    arrivals[joint].observe(sample, sample.received_s)
                    record("hold_feedback", joint=joint, sample=asdict(sample))
                stationary = all(all(arrival.last_conditions.values()) for arrival in arrivals.values())
                if not stationary:
                    settled_since = None
                elif settled_since is None:
                    settled_since = max(sample.received_s for sample in samples.values())
                elif (min(sample.request_s for sample in samples.values()) - settled_since
                      >= max(limit.settle_s for limit in self.executor.limits.values())):
                    return {"stop_result": "hold_observed", "stop_error": "",
                            "physical_stop_independently_verified": False}
                time.sleep(0.02)
            return {"stop_result": "external_stop_required", "stop_error": "hold_observation_timeout",
                    "conditions": {joint: arrival.last_conditions for joint, arrival in arrivals.items()}}
        except (RuntimeError, ValueError, OSError, ConstructError, KeyboardInterrupt) as error:
            return {"stop_result": "external_stop_required", "stop_error": str(error)}


def run_joint_table(executor: JointTableExecutor, *, port: str, log_path: Path,
                    measurement_dir: Path):
    # 本边界在创建文件和真实 I/O 前检查资格；当前缺项时保持拒绝。
    require_hardware_qualification(executor.model.root)
    frames_path = log_path.with_suffix(".frames.json")
    report_path = log_path.with_suffix(".report.json")
    if any(path.exists() for path in (log_path, frames_path, report_path)):
        raise FileExistsError("执行日志、收发记录或报告已经存在")
    if not measurement_dir.is_dir():
        raise FileNotFoundError(measurement_dir)
    latch = MotorStopLatch(executor.model.root / ".scratch/motor_control/stop.json")
    with log_path.open("x", encoding="utf-8") as log:
        def record(event, **fields):
            log.write(json.dumps(dict(event=event, monotonic_s=time.monotonic(), **fields),
                                 ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
            os.fsync(log.fileno())

        with DrEmpowerSerial(port, position_nodes=POSITION_NODES) as vendor:
            device = JointTableDevice(vendor, executor)
            file_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="measurement-files")
            pending_files = None
            pending_identity = None
            try:
                record("session_started", **executor.report())
                device.prepare()
                while executor.phase != "completed":
                    latch.require_clear()
                    executor.check_time(time.monotonic())
                    samples = device.read_cycle()
                    record("feedback", samples={joint: asdict(sample) for joint, sample in samples.items()})
                    executor.observe(samples, now_s=time.monotonic())
                    if executor.phase == "ready":
                        commands = executor.begin(now_s=time.monotonic())
                        device.send(commands, record)
                    elif executor.phase == "awaiting_measurement":
                        request_path = measurement_dir / f"{executor.index + 1:04d}.json"
                        if pending_files is None and request_path.exists():
                            with request_path.open(encoding="utf-8") as stream:
                                request = json.load(stream)
                            if set(request) != {"session_id", "task_id", "pose_id", "measurement_kind", "files"}:
                                raise ValueError("测量记录字段无效")
                            files = [Path(path) for path in request.pop("files")]
                            executor.validate_measurement_identity(now_s=time.monotonic(), **request)
                            pending_identity = request
                            pending_files = file_worker.submit(prepare_measurement_files, files)
                        if pending_files is not None and pending_files.done():
                            if pending_identity is None:
                                raise RuntimeError("测量文件处理缺少本组身份")
                            # 文件处理线程不读取设备、不修改执行器；本轮九轴反馈已经重新检查。
                            prepared = pending_files.result()
                            result = executor.record_prepared_measurement(
                                prepared, now_s=time.monotonic(), **pending_identity)
                            record("measurement_recorded", measurement=result)
                            pending_files, pending_identity = None, None
                    time.sleep(0.02)
                record("completed")
            except BaseException as error:
                stop = (device.hold_and_observe(record) if device.commands_attempted
                        else {"stop_result": "no_motion_command_attempted", "stop_error": ""})
                if not executor.reason:
                    executor.reason = str(error)
                executor.phase = "aborted"
                executor.termination = {"reason": executor.reason, **stop}
                attempt = executor.current_attempt()
                if attempt is not None:
                    attempt.update(status="aborted", rejection_reason=executor.reason, **stop)
                latch.trip(reason=executor.reason, log_path=log_path)
                record("aborted", reason=executor.reason, **stop)
                if stop["stop_result"] == "external_stop_required":
                    print("无法确认全部关节已经停车，需要现场执行停止操作。", flush=True)
                raise
            finally:
                file_worker.shutdown(wait=False, cancel_futures=True)
                try:
                    with frames_path.open("x", encoding="utf-8") as stream:
                        json.dump(vendor.frames, stream, ensure_ascii=False, indent=2, allow_nan=False)
                finally:
                    executor.write_report(report_path)
