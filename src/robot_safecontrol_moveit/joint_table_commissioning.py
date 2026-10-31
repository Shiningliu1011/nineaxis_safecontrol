import argparse
import json
import os
import time
from dataclasses import asdict, dataclass, replace
from decimal import ROUND_FLOOR, Decimal
from hashlib import sha256
from math import isfinite
from pathlib import Path
from zipfile import ZipFile

from .drempower_can import CMD_PROPERTY_READ
from .drempower_joint_control import JointFeedback
from .drempower_serial import SERIAL_CAN_FRAME
from .joint_table import JointTable, joint_execution_limits
from .joint_table_execution import JointTableExecutor
from .motor_motion_limits import MotionBounds
from .motor_stop_latch import MotorStopLatch
from .oscbf_trajectory import bootstrap_portable
from .robot_spec import DEFAULT_JOINT_NAMES
from .single_joint_execution import ObservedFeedback, VelocityWindow


def scaled_parameter(value: float, scale: float) -> float:
    return float((Decimal(str(value)) * Decimal(str(scale))).quantize(
        Decimal("0.01"), rounding=ROUND_FLOOR))


@dataclass(frozen=True)
class SinglePoseCommissioning:
    request_path: Path
    request_sha256: str
    settings_path: Path
    settings_sha256: str
    table_sha256: str
    model_hash: str
    config_hashes: tuple[tuple[str, str], ...]
    port: str
    expires_unix_s: float
    stop_latch_sha256: str
    expected_j1_serial_number_l: int
    expected_j1_zero_offset_counts: float
    gui_scale: float
    noise_evidence_path: Path
    noise_evidence_sha256: str

    @classmethod
    def load(cls, path: Path, table: JointTable, model, *, port: str):
        data = path.read_bytes()
        request = json.loads(data)
        required = {
            "purpose", "operator_attended", "operator_emergency_stop_confirmed", "operator_j1_zero_confirmed",
            "settings_path", "settings_sha256", "table_sha256", "model_hash", "config_hashes", "port",
            "expires_unix_s", "stop_latch_sha256", "expected_j1_serial_number_l",
            "expected_j1_zero_offset_counts", "gui_scale",
            "noise_evidence_path", "noise_evidence_sha256",
        }
        if not isinstance(request, dict) or set(request) != required:
            raise ValueError("单组调试请求字段无效")
        if (request["purpose"] != "single_pose_commissioning" or len(table.rows) != 1
                or any(request[name] is not True for name in (
                    "operator_attended", "operator_emergency_stop_confirmed", "operator_j1_zero_confirmed"))):
            raise ValueError("单组调试需要现场人员、独立急停及当前 J1 零位确认")
        if request["gui_scale"] != .75:
            raise ValueError("本次调试仅接受用户指定的上位机参数比例 0.75")
        permit = cls(path.resolve(), sha256(data).hexdigest(), Path(request["settings_path"]).resolve(),
                     request["settings_sha256"], request["table_sha256"], request["model_hash"],
                     tuple(sorted(request["config_hashes"].items())), request["port"],
                     request["expires_unix_s"], request["stop_latch_sha256"],
                     request["expected_j1_serial_number_l"], request["expected_j1_zero_offset_counts"],
                     request["gui_scale"], Path(request["noise_evidence_path"]).resolve(),
                     request["noise_evidence_sha256"])
        permit.require_scope(table, model, port=port)
        return permit

    def require_scope(self, table, model, *, port):
        if (not 0 < self.expires_unix_s - time.time() <= 600 or len(table.rows) != 1
                or table.source_sha256 != self.table_sha256 or model.model_hash != self.model_hash
                or port != self.port or self.gui_scale != .75):
            raise RuntimeError("单组调试请求过期或目标、模型、设备范围变化")
        if sha256(self.request_path.read_bytes()).hexdigest() != self.request_sha256:
            raise RuntimeError("单组调试请求内容变化")
        if set(dict(self.config_hashes)) != {"drempower.yaml", "hardware_joint_zero.yaml", "j1_transmission.yaml"}:
            raise ValueError("单组调试请求缺少配置身份")
        for name, digest in self.config_hashes:
            if sha256((model.root / "config" / name).read_bytes()).hexdigest() != digest:
                raise RuntimeError(f"单组调试配置变化：{name}")
        if sha256(self.settings_path.read_bytes()).hexdigest() != self.settings_sha256:
            raise RuntimeError("单组调试上位机参数来源变化")
        latch = model.root / ".scratch/motor_control/stop.json"
        if not latch.is_file() or sha256(latch.read_bytes()).hexdigest() != self.stop_latch_sha256:
            raise RuntimeError("待人工恢复的运动锁定身份变化")
        MotorStopLatch(latch).require_recoverable(expected_sha256=self.stop_latch_sha256)
        self.settings()
        if sha256(self.noise_evidence_path.read_bytes()).hexdigest() != self.noise_evidence_sha256:
            raise RuntimeError("停车反馈噪声证据身份变化")

    def settings(self):
        source = json.loads(self.settings_path.read_bytes())
        if (source["status"] != "readonly_completed" or source["scope"] != "position"
                or source["motion_commands_sent"] != 0 or source["port"] != self.port
                or set(source["properties"]) != set(DEFAULT_JOINT_NAMES)):
            raise ValueError("调试参数需要完整的真实设备只读来源")
        for index, joint in enumerate(DEFAULT_JOINT_NAMES, 1):
            values = source["properties"][joint]
            if values["node_id"] != index or values["axis_state"] != 8 or values["axis_error"] != 0 or values["traj_mode"] != 1:
                raise ValueError(f"{joint} 参数来源的身份、状态或轨迹模式无效")
        return source["properties"]

    def limits(self, root: Path):
        result = joint_execution_limits(root)
        noise = self.stationary_noise(root)
        for joint, values in self.settings().items():
            speed = scaled_parameter(values["trap_velocity_rpm"], self.gui_scale)
            accel = scaled_parameter(min(values["trap_acceleration_rpm_s"],
                                         values["trap_deceleration_rpm_s"]), self.gui_scale)
            feedback = scaled_parameter(values["output_vel_limit_rpm"], self.gui_scale)
            bounds = MotionBounds(speed, accel, feedback, self.settings_sha256)
            maximum_step = result[joint].max_step_deg
            if joint == "J1":
                if maximum_step is None:
                    raise ValueError("J1 缺少单次行程范围")
                maximum_step += result[joint].position_noise_deg
            result[joint] = replace(result[joint], speed_rpm=speed, accel_rpm_s=accel,
                                     max_feedback_rpm=feedback, bounds=bounds, stationary_window_s=5.,
                                     stationary_velocity_noise_rpm=noise[joint],
                                     max_step_deg=maximum_step)
        return result

    def stationary_noise(self, root: Path):
        return stationary_noise(root, self.noise_evidence_path, self.noise_evidence_sha256, self.model_hash)

    def consume(self, log_path: Path):
        # 每个请求只允许一次执行尝试；读取失败同样需要新的现场请求。
        with self.request_path.with_suffix(".used.json").open("x", encoding="utf-8") as stream:
            json.dump({"request_sha256": self.request_sha256, "log_path": str(log_path.resolve()),
                       "started_unix_s": time.time()}, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())

    def verify_device(self, device):
        return verify_positioning_device(
            device, source=self.settings(), expected_j1_serial_number_l=self.expected_j1_serial_number_l,
            expected_j1_zero_offset_counts=self.expected_j1_zero_offset_counts)

    def require_initial_zero(self, executor):
        require_initial_zero(executor)

    def report(self):
        return {**asdict(self), "request_path": str(self.request_path), "settings_path": str(self.settings_path),
                "noise_evidence_path": str(self.noise_evidence_path),
                "operator_emergency_stop_confirmed": True, "operator_attended": True,
                "maximum_poses": 1, "automatic_csv_execution_qualified": False,
                "internal_velocity_limit_enable_verified": False, "can_timeout_protection_verified": False,
                "device_sample_time_known": False, "physical_stop_independently_verified": False}


def stationary_noise(root: Path, evidence_path: Path, evidence_sha256: str, model_hash: str):
    if sha256(evidence_path.read_bytes()).hexdigest() != evidence_sha256:
        raise RuntimeError("停车反馈噪声证据身份变化")
    with ZipFile(evidence_path) as archive:
        provenance = json.loads(archive.read("provenance.json"))
        for item in provenance["files"]:
            content = archive.read(item["member"])
            if len(content) != item["bytes"] or sha256(content).hexdigest() != item["sha256"]:
                raise RuntimeError("停车反馈原始归档校验失败")
        report = json.loads(archive.read("first-group-gui-readiness-20261029.report.json"))
        frames = json.loads(archive.read("first-group-gui-readiness-20261029.frames.json"))
        events = [json.loads(line) for line in archive.read("first-group-gui-readiness-20261029.jsonl").splitlines()]
    if report["motion_commands_sent"] != 0 or report["parameter_writes_sent"] != 0:
        raise ValueError("停车噪声证据中存在设备动作")
    current = {name: sha256((root / "config" / name).read_bytes()).hexdigest()
               for name in ("hardware_joint_zero.yaml", "j1_transmission.yaml", "drempower.yaml")}
    if (report["executor"]["zero_calibration_sha256"] != current["hardware_joint_zero.yaml"]
            or report["executor"]["transmission_sha256"] != current["j1_transmission.yaml"]
            or report["executor"]["node_mapping_sha256"] != current["drempower.yaml"]
            or report["executor"]["model_hash"] != model_hash):
        raise RuntimeError("停车噪声证据的模型或设备坐标配置不符")
    for row in frames:
        if row["direction"] == "tx":
            frame = SERIAL_CAN_FRAME.parse(bytes.fromhex(row["raw_hex"]))
            if frame.frame_id & 31 != CMD_PROPERTY_READ or not 1 <= frame.frame_id >> 5 <= 9:
                raise ValueError("停车噪声证据含非读取命令")
    windows = {joint: VelocityWindow(1.) for joint in DEFAULT_JOINT_NAMES}
    angles: dict[str, list[float]] = {joint: [] for joint in DEFAULT_JOINT_NAMES}
    means: dict[str, list[float]] = {joint: [] for joint in DEFAULT_JOINT_NAMES}
    for event in events:
        if event["event"] != "feedback":
            continue
        if set(event["samples"]) != set(DEFAULT_JOINT_NAMES):
            raise ValueError("停车噪声证据缺少完整九轴")
        for index, joint in enumerate(DEFAULT_JOINT_NAMES, 1):
            row = event["samples"][joint]
            sample = ObservedFeedback(**{**row, "state": JointFeedback(**row["state"])})
            if sample.node_id != index or sample.axis_error or not sample.traj_done:
                raise ValueError(f"{joint} 停车噪声证据状态无效")
            checked = windows[joint].add(sample)
            angles[joint].append(sample.state.angle_deg)
            if checked.mean_speed_rpm is not None:
                means[joint].append(abs(checked.mean_speed_rpm))
    noise = {}
    for joint in DEFAULT_JOINT_NAMES:
        if (len(means[joint]) < 50 or max(angles[joint]) - min(angles[joint]) > .01
                or max(means[joint]) >= .2):
            raise ValueError(f"{joint} 停车噪声证据范围或时长无效")
        # 原始速度保持原值；此范围仅检查速度估计与静止位置记录是否相容。
        noise[joint] = max(means[joint]) + .01
    return noise


def verify_positioning_device(device, *, source: dict | None, expected_j1_serial_number_l: int,
                              expected_j1_zero_offset_counts: float, include_return_zero: bool = False,
                              require_zero_turn: bool = True):
    properties = {}
    for joint in DEFAULT_JOINT_NAMES:
        node = device.executor.control._node(joint)
        values = {}
        for name, address, kind in (
            ("node_id", 31001, "u32"), ("axis_state", 30002, "u32"),
            ("axis_error", 30001, "u32"), ("can_error", 20001, "u32"),
            ("motor_error", 33001, "u32"), ("encoder_error", 34001, "u32"),
            ("controller_error", 32001, "u32"),
            ("circular_limit_enabled", 31202, "u32"), ("circular_min_deg", 38010, "f32"),
            ("circular_max_deg", 38011, "f32"),
            ("output_vel_limit_rpm", 38012, "f32"), ("trap_velocity_rpm", 38014, "f32"),
            ("trap_acceleration_rpm_s", 38015, "f32"), ("trap_deceleration_rpm_s", 38016, "f32"),
            ("traj_mode", 35104, "u32"),
        ):
            values[name] = device.vendor.read_property(node, address, kind)
        properties[joint] = values
        if not all(isfinite(value) for value in values.values()):
            raise RuntimeError(f"{joint} 设备参数包含非有限数值")
        if values["node_id"] != node or values["axis_state"] != 8:
            raise RuntimeError(f"{joint} 实际身份或闭环状态无效：{values}")
        if any(values[name] != 0 for name in ("axis_error", "can_error", "motor_error", "encoder_error", "controller_error")):
            raise RuntimeError(f"{joint} 实际故障检查失败：{values}")
        if source is not None:
            for name in ("output_vel_limit_rpm", "trap_velocity_rpm", "trap_acceleration_rpm_s",
                         "trap_deceleration_rpm_s", "traj_mode"):
                if values[name] != source[joint][name]:
                    raise RuntimeError(f"{joint} 上位机参数在授权后变化：{name}")
        if (values["traj_mode"] != 1
                or values["output_vel_limit_rpm"] < device.executor.limits[joint].speed_rpm):
            raise RuntimeError(f"{joint} 轨迹模式或设备速度限制不支持本次目标")
    j1 = properties["J1"]
    for name, address, kind in (("serial_number_l", 3, "u32"), ("zero_offset_counts", 31205, "f32"),
                                 ("multi_circle_enabled", 31207, "u32"), ("encoder_turn_abs", 34010, "s32")):
        j1[name] = device.vendor.read_property(1, address, kind)
    if (j1["serial_number_l"] != expected_j1_serial_number_l
            or j1["zero_offset_counts"] != expected_j1_zero_offset_counts
            or j1["multi_circle_enabled"] != 1 or (require_zero_turn and j1["encoder_turn_abs"] != 0)):
        raise RuntimeError("J1 设备身份或现场零位恢复回读变化")
    commands = [command for row in device.executor.preview["rows"] for command in row["device_commands"]]
    if include_return_zero:
        commands.extend({"joint": joint, "arguments": device.executor.control.position_arguments(joint, 0., limit)}
                        for joint, limit in device.executor.limits.items())
    for command in commands:
        joint = command["joint"]
        values = properties[joint]
        target = command["arguments"]["angle"]
        if values["circular_limit_enabled"] not in (0, 1) or (
                values["circular_limit_enabled"] == 1
                and not values["circular_min_deg"] <= target <= values["circular_max_deg"]):
            raise RuntimeError(f"{joint} 目标超出当前设备限位")
    return properties


def require_initial_zero(executor):
    require_stationary_position(executor)
    if any(abs(sample.state.angle_deg) > (executor.limits[joint].position_noise_deg if joint == "J1"
                                          else executor.limits[joint].tolerance_deg)
           for joint, sample in executor.latest.items()):
        raise RuntimeError("当前姿态与用户确认的九轴初始零位不符")


def require_stationary_position(executor):
    executor.check_time(time.monotonic())
    if executor.phase != "ready" or set(executor.latest) != set(DEFAULT_JOINT_NAMES):
        raise RuntimeError("九轴共同停车尚未通过")


def main():
    parser = argparse.ArgumentParser(description="执行现场明确授权的单组九轴调试目标")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    bootstrap_portable(args.root / "portable_oscbf")
    from work.nineaxis_model import NineaxisModel

    from .joint_table_runtime import run_joint_table

    model = NineaxisModel.load(root=args.root)
    table = JointTable.load(args.input, model)
    permit = SinglePoseCommissioning.load(args.request, table, model, port=args.port)
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="single-pose-commissioning",
                                  feedback_source="hardware_feedback", limits=permit.limits(args.root))
    run_joint_table(executor, port=args.port, log_path=args.output,
                    measurement_dir=args.output.parent, commissioning=permit)


if __name__ == "__main__":
    main()
