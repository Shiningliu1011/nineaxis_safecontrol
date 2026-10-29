from collections import deque
from dataclasses import asdict, dataclass, replace
import json
from math import degrees, isfinite
from pathlib import Path
import time
from threading import Lock
from types import ModuleType
from typing import Any, NoReturn, Protocol
import xml.etree.ElementTree as ET

from .drempower_can import (
    AXIS_STATE_CLOSED_LOOP, CMD_PROPERTY_READ, PROP_AXIS_CURRENT_STATE,
    decode_feedback, decode_property_reply, encode_property_read,
)
from .drempower_joint_control import DrEmpowerJointControl, JointFeedback
from .motor_motion_limits import (
    MAX_SINGLE_JOINT_SPEED_RPM, MotionBounds, feedback_speed_limit, validate_motion_parameters,
)


_EXECUTION_LOCK = Lock()


@dataclass(frozen=True)
class ExecutionLimits:
    lower_deg: float
    upper_deg: float
    max_step_deg: float | None = None
    speed_rpm: float = 0.05
    accel_rpm_s: float = 0.05
    feedback_timeout_s: float = 0.2
    motion_timeout_s: float = 20.0
    settle_s: float = 0.5
    tolerance_deg: float = 0.1
    stopped_rpm: float = 0.02
    max_feedback_rpm: float = MAX_SINGLE_JOINT_SPEED_RPM
    velocity_window_s: float = 1.0
    position_noise_deg: float = 0.01
    axis_type: str = "revolute"
    bounds: MotionBounds | None = None
    stationary_window_s: float = 1.0
    stationary_velocity_noise_rpm: float = 0.0

    @property
    def hold_timeout_s(self) -> float:
        return (self.max_feedback_rpm / self.accel_rpm_s + self.stationary_window_s
                + self.settle_s + 2 * self.feedback_timeout_s)

    def __post_init__(self):
        values = asdict(self)
        del values["axis_type"]
        del values["bounds"]
        if self.max_step_deg is None:
            del values["max_step_deg"]
        if not all(isfinite(value) for value in values.values()):
            raise ValueError("执行限制必须为有限数值")
        if self.lower_deg >= self.upper_deg:
            raise ValueError("关节下限必须小于上限")
        for name, value in values.items():
            if name == "stationary_velocity_noise_rpm":
                if value < 0:
                    raise ValueError("停车速度测量噪声必须非负")
                continue
            if name not in ("lower_deg", "upper_deg") and value <= 0:
                raise ValueError(f"{name} 必须为正数")
        validate_motion_parameters(self.speed_rpm, self.accel_rpm_s, axis_type=self.axis_type,
                                   bounds=self.bounds)
        maximum = self.bounds.feedback_rpm if self.bounds else feedback_speed_limit(self.axis_type)
        if self.max_feedback_rpm > maximum:
            raise ValueError("单关节检查反馈速度超过对应轴的保护上限")
        if not self.stopped_rpm < self.speed_rpm <= self.max_feedback_rpm:
            raise ValueError("静止、指令及反馈速度阈值顺序无效")
        if self.feedback_timeout_s > 0.2 or self.settle_s < 0.5:
            raise ValueError("反馈超时最多 0.2 秒，到位持续时间至少 0.5 秒")
        if self.tolerance_deg > 0.1 or self.motion_timeout_s <= self.settle_s:
            raise ValueError("到位误差或运动超时无效")
        if not 0.5 <= self.velocity_window_s <= 1.0 or self.position_noise_deg > 0.01:
            raise ValueError("速度窗口必须在 0.5–1 秒，位置测量裕量最多 0.01°")
        if not self.velocity_window_s <= self.stationary_window_s <= 10:
            raise ValueError("停车速度窗口必须覆盖运动速度窗口且不超过 10 秒")

    def for_motion(self, delta_deg: float):
        if not isfinite(delta_deg) or delta_deg == 0:
            raise ValueError("运动角度增量必须为有限非零数值")
        travel_s = abs(delta_deg) / (6 * self.speed_rpm) + self.speed_rpm / self.accel_rpm_s
        return replace(self, motion_timeout_s=max(self.motion_timeout_s, travel_s + 2))

    @classmethod
    def from_urdf(cls, path: Path, joint: str):
        matches = [item for item in ET.parse(path).getroot().findall("joint")
                   if item.get("name") == joint]
        if len(matches) != 1 or matches[0].get("type") != "revolute":
            raise ValueError("必须指定唯一的旋转关节")
        limit = matches[0].find("limit")
        if limit is None:
            raise ValueError("URDF 缺少关节限位")
        return cls(degrees(float(limit.attrib["lower"])), degrees(float(limit.attrib["upper"])))


@dataclass(frozen=True)
class ObservedFeedback:
    node_id: int
    state: JointFeedback
    request_s: float
    received_s: float
    traj_done: bool
    axis_error: bool
    mean_speed_rpm: float | None = None
    position_speed_rpm: float | None = None
    window_duration_s: float = 0.0
    stationary_mean_speed_rpm: float | None = None
    stationary_window_duration_s: float = 0.0
    stationary_position_span_deg: float | None = None


class VelocityWindow:
    def __init__(self, duration_s: float, stationary_duration_s: float | None = None):
        self.duration_s = duration_s
        self.stationary_duration_s = duration_s if stationary_duration_s is None else stationary_duration_s
        if not 0 < duration_s <= self.stationary_duration_s <= 10:
            raise ValueError("反馈速度窗口范围无效")
        self.rows: deque[ObservedFeedback] = deque()

    def add(self, sample: ObservedFeedback) -> ObservedFeedback:
        if self.rows and sample.request_s <= self.rows[-1].received_s:
            raise MotionRejected("feedback_repeated_or_out_of_order")
        if self.rows and sample.request_s - self.rows[-1].received_s > 0.2:
            raise MotionRejected("feedback_gap")
        if not all(isfinite(v) for v in (sample.state.angle_deg, sample.state.speed_rpm,
                                        sample.request_s, sample.received_s)):
            raise MotionRejected("nonfinite_feedback")
        if not 0 <= sample.request_s <= sample.received_s:
            raise MotionRejected("feedback_clock_invalid")
        sample = replace(sample, mean_speed_rpm=None, position_speed_rpm=None, window_duration_s=0.,
                         stationary_mean_speed_rpm=None, stationary_window_duration_s=0.,
                         stationary_position_span_deg=None)
        self.rows.append(sample)
        while len(self.rows) > 2 and sample.received_s - self.rows[1].received_s >= self.stationary_duration_s:
            self.rows.popleft()
        motion_rows = list(self.rows)
        while len(motion_rows) > 2 and sample.received_s - motion_rows[1].received_s >= self.duration_s:
            motion_rows.pop(0)
        span = sample.received_s - motion_rows[0].received_s
        if span < self.duration_s:
            return sample
        # 对不等间隔的原始速度做时间积分；位置速度来自独立的位置变化。
        integral = sum((b.received_s - a.received_s) * (a.state.speed_rpm + b.state.speed_rpm) / 2
                       for a, b in zip(motion_rows, motion_rows[1:]))
        recent = [row for row in self.rows if sample.received_s - row.received_s <= 0.2]
        if len(recent) < 2:
            raise MotionRejected("insufficient_recent_position_feedback")
        first = recent[0]
        position_speed = ((sample.state.angle_deg - first.state.angle_deg)
                          / (sample.received_s - first.received_s) / 6)
        result = replace(sample, mean_speed_rpm=integral / span,
                         position_speed_rpm=position_speed, window_duration_s=span)
        stationary_span = sample.received_s - self.rows[0].received_s
        if stationary_span >= self.stationary_duration_s:
            stationary_integral = sum(
                (b.received_s - a.received_s) * (a.state.speed_rpm + b.state.speed_rpm) / 2
                for a, b in zip(self.rows, list(self.rows)[1:]))
            angles = [row.state.angle_deg for row in self.rows]
            result = replace(result, stationary_mean_speed_rpm=stationary_integral / stationary_span,
                             stationary_window_duration_s=stationary_span,
                             stationary_position_span_deg=max(angles) - min(angles))
        return result


def checked_speed(sample: ObservedFeedback) -> float:
    if sample.mean_speed_rpm is None and sample.position_speed_rpm is None:
        return abs(sample.state.speed_rpm)
    if sample.mean_speed_rpm is None or sample.position_speed_rpm is None:
        raise MotionRejected("incomplete_velocity_window")
    if not all(isfinite(value) for value in (sample.mean_speed_rpm, sample.position_speed_rpm)):
        raise MotionRejected("nonfinite_velocity_window")
    return max(abs(sample.mean_speed_rpm), abs(sample.position_speed_rpm))


def stationary_conditions(sample: ObservedFeedback, limits: ExecutionLimits) -> dict[str, bool]:
    if limits.stationary_window_s == limits.velocity_window_s:
        return {"speed": checked_speed(sample) <= limits.stopped_rpm}
    mean = sample.stationary_mean_speed_rpm
    span = sample.stationary_position_span_deg
    speed = sample.position_speed_rpm
    if mean is None or span is None or speed is None:
        return {"speed": False, "position_stability": False, "stationary_window": False}
    if (not all(isfinite(value) for value in (mean, span, speed, sample.stationary_window_duration_s))
            or span < 0 or sample.stationary_window_duration_s < 0):
        raise MotionRejected("nonfinite_stationary_window")
    return {
        "speed": abs(speed) <= limits.stopped_rpm,
        "raw_speed_consistent": abs(mean) <= limits.stopped_rpm + limits.stationary_velocity_noise_rpm,
        "position_stability": span <= limits.position_noise_deg,
        "stationary_window": sample.stationary_window_duration_s >= limits.stationary_window_s,
    }


class MotionRejected(RuntimeError):
    pass


class ArrivalWindow:
    def __init__(self, target_deg: float, limits: ExecutionLimits):
        self.target_deg = target_deg
        self.limits = limits
        self.since_s: float | None = None
        self.max_settled_s = 0.0
        self.last_conditions: dict[str, bool] = {}

    def observe(self, sample: ObservedFeedback, now_s: float) -> bool:
        self.last_conditions = {
            "position": abs(sample.state.angle_deg - self.target_deg) <= self.limits.tolerance_deg,
            **stationary_conditions(sample, self.limits),
            "trajectory_done": sample.traj_done,
        }
        settled = all(self.last_conditions.values())
        if not settled:
            self.since_s = None
            return False
        if self.since_s is None:
            self.since_s = now_s
        duration = now_s - self.since_s
        self.max_settled_s = max(self.max_settled_s, duration)
        return duration >= self.limits.settle_s


def decode_observation(control, joint, frame_id, data, request_s, received_s):
    node = control._node(joint)
    if frame_id & 0x18:
        raise MotionRejected("unexpected_feedback_command")
    motor = decode_feedback(frame_id, data)
    if motor.node_id != node:
        raise MotionRejected("feedback_node_mismatch")
    state = control.decode_state(joint, motor.pos_deg, motor.vel_rpm)
    return ObservedFeedback(node, state, request_s, received_s, motor.traj_done, motor.axis_error)


class SingleJointMonitor:
    def __init__(self, node_id: int, limits: ExecutionLimits):
        if type(node_id) is not int or not 1 <= node_id <= 63:
            raise ValueError("必须指定单个电机节点")
        self.node_id = node_id
        self.limits = limits
        self.phase = "ready"
        self.reason = ""
        self.target_deg: float | None = None
        self.start_deg: float | None = None
        self.started_s: float | None = None
        self.last_request_s: float | None = None
        self.last_received_s: float | None = None
        self.last_angle_deg: float | None = None
        self.arrival: ArrivalWindow | None = None
        self.position_arrived_s: float | None = None

    def reject(self, reason: str) -> NoReturn:
        if not self.reason:
            self.reason = reason
        self.phase = "aborted"
        raise MotionRejected(self.reason)

    def validate_feedback(self, sample: ObservedFeedback, now_s: float):
        self.validate_hold_feedback(sample, now_s)
        if checked_speed(sample) > self.limits.max_feedback_rpm:
            self.reject("velocity_limit")

    def validate_hold_feedback(self, sample: ObservedFeedback, now_s: float):
        # 保持请求需要可信位置；运动超速仍由 validate_feedback 拒绝。
        values = (sample.request_s, sample.received_s, now_s,
                  sample.state.angle_deg, sample.state.speed_rpm)
        if not all(isfinite(value) for value in values):
            self.reject("nonfinite_feedback")
        if sample.node_id != self.node_id:
            self.reject("feedback_node_mismatch")
        if not 0 <= sample.request_s <= sample.received_s <= now_s:
            self.reject("feedback_clock_invalid")
        # 请求时间作为保守的主机时效边界；不声称它是设备采样时间。
        if now_s - sample.request_s > self.limits.feedback_timeout_s:
            self.reject("feedback_timeout")
        if sample.axis_error:
            self.reject("axis_error")
        if not self.limits.lower_deg <= sample.state.angle_deg <= self.limits.upper_deg:
            self.reject("joint_limit")
        checked_speed(sample)
        if self.last_received_s is not None and sample.request_s <= self.last_received_s:
            self.reject("feedback_repeated_or_out_of_order")
        if (self.last_received_s is not None
                and sample.request_s - self.last_received_s > self.limits.feedback_timeout_s):
            self.reject("feedback_gap")
        if self.last_received_s is not None:
            if self.last_request_s is None or self.last_angle_deg is None:
                self.reject("feedback_history_invalid")
            # 两次事务时长会变化，接收间隔不能代表设备采样间隔。
            # 使用主机事务的外侧时间范围检查位置一致性，不声明设备采样年龄。
            elapsed = sample.received_s - self.last_request_s
            allowed = self.limits.max_feedback_rpm * 6 * elapsed + self.limits.position_noise_deg
            if abs(sample.state.angle_deg - self.last_angle_deg) > allowed:
                self.reject("feedback_position_jump")
        self.last_request_s = sample.request_s
        self.last_received_s = sample.received_s
        self.last_angle_deg = sample.state.angle_deg

    def start(self, sample: ObservedFeedback, delta_deg: float, now_s: float) -> float:
        return self._start(sample, delta_deg, now_s, allow_zero=False)

    def start_absolute(self, sample: ObservedFeedback, target_deg: float, now_s: float) -> float:
        return self._start(sample, target_deg - sample.state.angle_deg, now_s, allow_zero=True)

    def _start(self, sample, delta_deg, now_s, *, allow_zero):
        if self.phase != "ready":
            self.reject("execution_not_ready")
        self.validate_feedback(sample, now_s)
        if (not isfinite(delta_deg) or (delta_deg == 0 and not allow_zero)
                or (self.limits.max_step_deg is not None
                    and abs(delta_deg) > self.limits.max_step_deg)):
            self.reject("step_limit")
        if not all(stationary_conditions(sample, self.limits).values()):
            self.reject("initial_joint_moving")
        target = sample.state.angle_deg + delta_deg
        if not self.limits.lower_deg <= target <= self.limits.upper_deg:
            self.reject("target_joint_limit")
        self.start_deg = sample.state.angle_deg
        self.target_deg = target
        self.started_s = now_s
        self.arrival = ArrivalWindow(target, self.limits)
        self.phase = "moving"
        return target

    def observe(self, sample: ObservedFeedback, now_s: float) -> bool:
        if (self.phase not in ("moving", "settling") or self.started_s is None
                or self.start_deg is None or self.target_deg is None or self.arrival is None):
            self.reject("execution_not_moving")
        self.validate_feedback(sample, now_s)
        if self.phase == "moving" and now_s - self.started_s > self.limits.motion_timeout_s:
            self.reject("motion_timeout")
        low, high = sorted((self.start_deg, self.target_deg))
        if not low - self.limits.tolerance_deg <= sample.state.angle_deg <= high + self.limits.tolerance_deg:
            self.reject("motion_outside_requested_interval")
        position_arrived = abs(sample.state.angle_deg - self.target_deg) <= self.limits.tolerance_deg
        if self.phase == "moving" and position_arrived and sample.traj_done:
            self.phase = "settling"
            self.position_arrived_s = now_s
        if self.phase == "settling":
            if self.position_arrived_s is None:
                self.reject("settling_clock_invalid")
            if not position_arrived:
                self.reject("target_left_settling_tolerance")
            if now_s - self.position_arrived_s > self.limits.hold_timeout_s:
                self.reject("settling_timeout")
        if self.arrival.observe(sample, now_s):
            self.phase = "completed"
            return True
        return False


class FeedbackControl(Protocol):
    def _node(self, joint: str) -> int: ...

    def decode_state(self, joint: str, angle_deg: float, speed_rpm: float) -> JointFeedback: ...


class FeedbackVendor(Protocol):
    uart: Any
    READ_FLAG: int
    enable_replay_state: int

    def send_command(self, id_num, cmd, data, rtr=0): ...

    def receive_data(self, return_id=False): ...


class VendorFeedbackReader:
    def __init__(self, vendor: ModuleType | FeedbackVendor, control: FeedbackControl, timeout_s: float):
        if not 0 < timeout_s <= 0.2:
            raise ValueError("串口事务超时必须在 (0, 0.2] 内")
        # 厂家库打开的串口必须已经独占；这里不改变设备配置或连接。
        if vendor.uart.exclusive is not True:
            raise ValueError("厂家库串口必须以 exclusive=True 打开")
        if vendor.enable_replay_state != 0:
            raise ValueError("单请求读取要求关闭命令自动反馈")
        for name in ("timeout", "write_timeout"):
            value = getattr(vendor.uart, name)
            if value is None or not isfinite(value) or not 0 < value <= timeout_s / 2:
                raise ValueError(f"厂家库串口 {name} 必须在 (0, {timeout_s / 2}] 秒内")
        self.vendor = vendor
        self.control = control
        self.timeout_s = timeout_s
        self.in_transaction = False

    def _request(self, node: int, payload: bytes):
        if self.in_transaction:
            raise MotionRejected("serial_session_requires_reconnect")
        self.in_transaction = True
        if self.vendor.uart.in_waiting:
            raise MotionRejected("unsolicited_or_stale_serial_data")
        started = time.monotonic()
        self.vendor.send_command(id_num=node, cmd=CMD_PROPERTY_READ, data=list(payload), rtr=0)
        raw = self.vendor.receive_data(return_id=True)
        received = time.monotonic()
        if received - started > self.timeout_s:
            raise MotionRejected("feedback_timeout")
        if self.vendor.READ_FLAG != 1 or raw is None or len(raw) != 11 or raw[0] != 8:
            raise MotionRejected("invalid_feedback_frame")
        frame_id = int.from_bytes(bytes(raw[1:3]), "big")
        return frame_id, bytes(raw[3:]), started, received

    def require_closed_loop(self, joint: str):
        node = self.control._node(joint)
        frame_id, data, _, _ = self._request(
            node, encode_property_read(node, PROP_AXIS_CURRENT_STATE),
        )
        state = decode_property_reply(frame_id, data, node_id=node, address=PROP_AXIS_CURRENT_STATE)
        if state != AXIS_STATE_CLOSED_LOOP:
            raise MotionRejected("motor_not_closed_loop")
        self.in_transaction = False

    def read(self, joint: str) -> ObservedFeedback:
        node = self.control._node(joint)
        frame_id, data, started, received = self._request(node, bytes(8))
        sample = decode_observation(self.control, joint, frame_id, data, started, received)
        self.in_transaction = False
        return sample


def run_single_joint(
    vendor: ModuleType, control: DrEmpowerJointControl, joint: str, delta_deg: float | None,
    limits: ExecutionLimits, log_path: Path, *, target_deg: float | None = None,
) -> None:
    """执行单关节事务；调用方负责现场停车、标定及串口所有权。"""
    if not _EXECUTION_LOCK.acquire(blocking=False):
        raise MotionRejected("another_joint_execution_active")
    try:
        _run_single_joint(vendor, control, joint, delta_deg, limits, log_path, target_deg)
    finally:
        _EXECUTION_LOCK.release()


def _run_single_joint(vendor, control, joint, delta_deg, limits, log_path, target_deg):
    if limits.axis_type != control.axis_type:
        raise ValueError("执行限制与关节类型不符")
    node = control._node(joint)
    if (delta_deg is None) == (target_deg is None):
        raise ValueError("相对增量和绝对目标必须选择一项")
    if delta_deg is not None and (not isfinite(delta_deg) or delta_deg == 0
                                  or (limits.max_step_deg is not None
                                      and abs(delta_deg) > limits.max_step_deg)):
        raise ValueError("相对增量必须为有限非零数值，并满足配置范围")
    if target_deg is not None and (not isfinite(target_deg) or not limits.lower_deg <= target_deg <= limits.upper_deg):
        raise ValueError("绝对目标必须位于关节范围内")
    monitor = SingleJointMonitor(node, limits)
    reader = VendorFeedbackReader(vendor, control, limits.feedback_timeout_s)
    window = VelocityWindow(limits.velocity_window_s)
    # 使用 x 模式，保留每次独立的命令、反馈和终态记录。
    with Path(log_path).open("x", encoding="utf-8") as log:
        def record(event, **fields):
            log.write(json.dumps(dict(event=event, monotonic_s=time.monotonic(), **fields),
                                 ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()

        record("start", joint=joint, node_id=node, delta_deg=delta_deg, requested_target_deg=target_deg,
               limits=asdict(limits),
               position_change_time_basis="previous_request_to_current_receive",
               device_sample_time_known=False,
               coordinate_basis=("forward_output_shaft_deg" if limits.axis_type == "prismatic"
                                 else "joint_angle_deg"),
               calibration=asdict(control.converter.calibration.for_joint(joint)))
        command_attempted = False
        try:
            reader.require_closed_loop(joint)
            startup_until = time.monotonic() + 5
            while True:
                initial = window.add(reader.read(joint))
                record("feedback", sample=asdict(initial))
                if initial.axis_error:
                    raise MotionRejected("axis_error")
                if (initial.window_duration_s >= limits.velocity_window_s
                        and checked_speed(initial) <= limits.stopped_rpm):
                    break
                if time.monotonic() > startup_until:
                    raise MotionRejected("initial_joint_not_stationary")
                time.sleep(0.02)
            actual_delta = delta_deg if target_deg is None else target_deg - initial.state.angle_deg
            limits = limits.for_motion(actual_delta)
            monitor = SingleJointMonitor(node, limits)
            target = monitor.start(initial, actual_delta, time.monotonic())
            reader.require_closed_loop(joint)
            arguments = control.position_arguments(
                joint, target, speed_rpm=limits.speed_rpm, accel_rpm_s=limits.accel_rpm_s,
            )
            record("command_requested", arguments=arguments, motion_timeout_s=limits.motion_timeout_s)
            # 日志写入也可能耗时，发送前再次检查时效。
            if time.monotonic() - initial.request_s > limits.feedback_timeout_s:
                raise MotionRejected("feedback_timeout_before_send")
            command_attempted = True
            control.set_angle(vendor, joint, target, speed_rpm=limits.speed_rpm,
                              accel_rpm_s=limits.accel_rpm_s)
            record("command_call_returned")
            while True:
                time.sleep(0.02)
                reader.require_closed_loop(joint)
                sample = window.add(reader.read(joint))
                record("feedback", sample=asdict(sample))
                previous_phase = monitor.phase
                done = monitor.observe(sample, time.monotonic())
                if previous_phase == "moving" and monitor.phase == "settling":
                    record("position_arrived", target_deg=target,
                           settling_timeout_s=limits.hold_timeout_s)
                if done:
                    record("completed", target_deg=target)
                    return
        except BaseException as error:
            # 包括 Ctrl+C：终止后续运动；不清错、不失能、不广播。
            stop_result = "no_motion_command_attempted"
            stop_error = ""
            stop_observation = None
            if command_attempted:
                stop_result = "external_stop_required"
            can_hold = (command_attempted and not reader.in_transaction
                        and (isinstance(error, KeyboardInterrupt) or monitor.reason in
                             ("motion_timeout", "motion_outside_requested_interval",
                              "velocity_limit", "settling_timeout",
                              "target_left_settling_tolerance")))
            if can_hold:
                try:
                    reader.require_closed_loop(joint)
                    current = window.add(reader.read(joint))
                    record("hold_requested", sample=asdict(current))
                    stop_monitor = SingleJointMonitor(node, limits)
                    stop_monitor.last_request_s = monitor.last_request_s
                    stop_monitor.last_received_s = monitor.last_received_s
                    stop_monitor.last_angle_deg = monitor.last_angle_deg
                    stop_monitor.validate_hold_feedback(current, time.monotonic())
                    control.set_angle(vendor, joint, current.state.angle_deg,
                                      speed_rpm=limits.speed_rpm, accel_rpm_s=limits.accel_rpm_s)
                    record("hold_command_call_returned")
                    # 命令返回不证明物理停车，必须继续读取并观察。
                    hold_started = time.monotonic()
                    until = hold_started + limits.hold_timeout_s
                    arrival = ArrivalWindow(current.state.angle_deg, limits)
                    record("hold_observation_started", timeout_s=limits.hold_timeout_s)
                    while time.monotonic() < until:
                        time.sleep(0.02)
                        reader.require_closed_loop(joint)
                        observed = window.add(reader.read(joint))
                        record("hold_feedback", sample=asdict(observed))
                        stop_monitor.validate_feedback(observed, time.monotonic())
                        if arrival.observe(observed, time.monotonic()):
                            stop_result = "hold_observed"
                            break
                    if stop_result != "hold_observed":
                        stop_error = "hold_observation_timeout"
                        stop_observation = dict(
                            conditions=arrival.last_conditions,
                            max_settled_s=arrival.max_settled_s,
                            required_settle_s=limits.settle_s,
                            stopped_rpm=limits.stopped_rpm,
                        )
                except BaseException as hold_error:
                    # 停止尝试失败仍保留原始异常；必须通知现场操作人员。
                    stop_result = "external_stop_required"
                    stop_error = str(hold_error)
            print(f"执行已终止：{error}；停止状态：{stop_result}", flush=True)
            if stop_result == "external_stop_required":
                print("无法确认电机已经停止，请立即操作现场急停。", flush=True)
            record("aborted", reason=str(error), stop_result=stop_result, stop_error=stop_error,
                   stop_observation=stop_observation)
            raise
