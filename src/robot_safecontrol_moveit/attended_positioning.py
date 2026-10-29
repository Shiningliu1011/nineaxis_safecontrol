import json
from dataclasses import dataclass, replace
from hashlib import sha256
from pathlib import Path

from .joint_table_commissioning import (
    require_stationary_position,
    stationary_noise,
    verify_positioning_device,
)
from .motor_motion_limits import MotionBounds
from .motor_stop_latch import MotorStopLatch


@dataclass(frozen=True)
class AttendedPositioning:
    root: Path
    policy_path: Path
    policy_sha256: str
    policy: dict
    table_sha256: str
    limits: dict

    @classmethod
    def load(cls, root: Path, path: Path, table, model, *, port: str, limits: dict):
        content = path.read_bytes()
        policy = json.loads(content)
        fields = {"purpose", "port", "model_hash", "config_hashes", "reviewed_stop_sha256",
                  "expected_j1_serial_number_l", "expected_j1_zero_offset_counts",
                  "noise_evidence_path", "noise_evidence_sha256", "conditions", "maximum_feedback_deg_s"}
        if not isinstance(policy, dict) or set(policy) != fields:
            raise ValueError("有人值守定位的现场记录字段无效")
        if (policy["purpose"] != "attended_positioning"
                or policy["conditions"] != {"motion_area_empty": True, "operator_attended": True,
                                            "powered_holding_only": True, "return_zero_on_confirmation": True}
                or any(value is not True for value in policy["conditions"].values())):
            raise ValueError("有人值守定位需要现场范围、持续看护、通电保持及确认回零条件")
        if type(policy["maximum_feedback_deg_s"]) not in (int, float) or policy["maximum_feedback_deg_s"] != 5.:
            raise ValueError("本次有人值守实验的旋转轴反馈速度上限为 5°/s")
        session = cls(root.resolve(), path.resolve(), sha256(content).hexdigest(), policy,
                      table.source_sha256, dict(limits))
        session.require_identity(table, model, port=port)
        noise = stationary_noise(root, root / policy["noise_evidence_path"],
                                 policy["noise_evidence_sha256"], model.model_hash)
        adjusted = {}
        for joint, limit in limits.items():
            if limit.bounds is not None:
                raise ValueError("有人值守定位必须采用既有低速参数范围")
            # J1 的最大行程包含已验证的位置测量裕量，目标行程范围继续单独检查。
            maximum_step = limit.max_step_deg
            if joint == "J1" and maximum_step is not None:
                maximum_step += limit.position_noise_deg
            bounds = (None if joint == "J1" else MotionBounds(limit.speed_rpm, limit.accel_rpm_s,
                                                              policy["maximum_feedback_deg_s"] / 6,
                                                              session.policy_sha256))
            adjusted[joint] = replace(limit, stationary_window_s=5., stationary_velocity_noise_rpm=noise[joint],
                                      max_step_deg=maximum_step, bounds=bounds,
                                      max_feedback_rpm=limit.max_feedback_rpm if bounds is None else bounds.feedback_rpm)
        return replace(session, limits=adjusted)

    @property
    def stop_latch(self):
        return MotorStopLatch(self.root / ".scratch/motor_control/attended-positioning-stop.json")

    def check_stop(self):
        self.stop_latch.require_clear()
        original = self.root / ".scratch/motor_control/stop.json"
        if sha256(original.read_bytes()).hexdigest() != self.policy["reviewed_stop_sha256"]:
            raise RuntimeError("有人值守定位的已审阅停止记录变化，需要检查新原因")
        if sha256(self.policy_path.read_bytes()).hexdigest() != self.policy_sha256:
            raise RuntimeError("有人值守定位的现场记录在运行期间变化")

    def require_identity(self, table, model, *, port: str):
        self.check_stop()
        if (model.root.resolve() != self.root or table.source_sha256 != self.table_sha256
                or model.model_hash != self.policy["model_hash"] or port != self.policy["port"]):
            raise RuntimeError("有人值守定位的设备、目标或模型范围不符")
        hashes = self.policy["config_hashes"]
        if set(hashes) != {"drempower.yaml", "hardware_joint_zero.yaml", "j1_transmission.yaml"}:
            raise ValueError("有人值守定位缺少完整设备坐标配置身份")
        for name, digest in hashes.items():
            if sha256((self.root / "config" / name).read_bytes()).hexdigest() != digest:
                raise RuntimeError(f"有人值守定位的设备配置变化：{name}")

    def require_scope(self, executor, *, port: str):
        self.require_identity(executor.table, executor.model, port=port)
        if (not executor.manual_positioning or executor.measurement_kind != "positioning"
                or executor.feedback_source != "hardware_feedback" or executor.limits != self.limits):
            raise ValueError("有人值守定位仅用于采用本次低速参数和真实反馈的手动入口")

    def verify_device(self, device):
        return verify_positioning_device(
            device, source=None, expected_j1_serial_number_l=self.policy["expected_j1_serial_number_l"],
            expected_j1_zero_offset_counts=self.policy["expected_j1_zero_offset_counts"], include_return_zero=True,
            require_zero_turn=False)

    def require_initial_position(self, executor):
        require_stationary_position(executor)

    def report(self):
        return {"mode": "attended_positioning", "policy_path": str(self.policy_path),
                "policy_sha256": self.policy_sha256, "conditions": self.policy["conditions"],
                "reviewed_stop_sha256": self.policy["reviewed_stop_sha256"],
                "noise_evidence_path": self.policy["noise_evidence_path"],
                "noise_evidence_sha256": self.policy["noise_evidence_sha256"],
                "maximum_feedback_deg_s": self.policy["maximum_feedback_deg_s"],
                "gravity_safe_stop_verified": False, "physical_stop_independently_verified": False,
                "automatic_csv_execution_qualified": False, "device_sample_time_known": False}
