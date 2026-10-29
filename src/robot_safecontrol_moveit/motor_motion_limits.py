from math import isfinite
from dataclasses import dataclass
from decimal import Decimal


MAX_SINGLE_JOINT_SPEED_RPM = 2.5 / 6
MAX_J1_SPEED_RPM = 3.0
MAX_J1_ACCEL_RPM_S = 0.6
MAX_J1_FEEDBACK_RPM = 3.6


@dataclass(frozen=True)
class MotionBounds:
    speed_rpm: float
    accel_rpm_s: float
    feedback_rpm: float
    source_sha256: str

    def __post_init__(self):
        if (not all(isfinite(value) and 0.01 <= value < 300
                    for value in (self.speed_rpm, self.accel_rpm_s, self.feedback_rpm))
                or self.speed_rpm > self.feedback_rpm
                or len(self.source_sha256) != 64
                or any(value not in "0123456789abcdef" for value in self.source_sha256)):
            raise ValueError("调试参数范围及来源 SHA-256 无效")


def feedback_speed_limit(axis_type: str) -> float:
    if axis_type == "prismatic":
        return MAX_J1_FEEDBACK_RPM
    if axis_type == "revolute":
        return MAX_SINGLE_JOINT_SPEED_RPM
    raise ValueError("axis_type 必须为 revolute 或 prismatic")


def validate_motion_parameters(
    speed_rpm: float, accel_rpm_s: float, *, axis_type: str = "revolute",
    bounds: MotionBounds | None = None,
) -> None:
    feedback_speed_limit(axis_type)
    max_speed = MAX_J1_SPEED_RPM if axis_type == "prismatic" else MAX_SINGLE_JOINT_SPEED_RPM
    max_accel = MAX_J1_ACCEL_RPM_S if axis_type == "prismatic" else 0.1
    if bounds is not None:
        max_speed, max_accel = bounds.speed_rpm, bounds.accel_rpm_s
    values = (("speed_rpm", speed_rpm), ("accel_rpm_s", accel_rpm_s))
    for name, value in values:
        if not isfinite(value):
            raise ValueError(f"{name} 必须为有限数值")
    if not 0.01 <= speed_rpm <= max_speed:
        raise ValueError(f"speed_rpm 必须在 [0.01, {max_speed}] rpm 内")
    if not 0.01 <= accel_rpm_s <= max_accel:
        raise ValueError(f"accel_rpm_s 必须在 [0.01, {max_accel}] rpm/s 内")
    for name, value in values:
        if Decimal(str(value)) % Decimal("0.01"):
            raise ValueError(f"{name} 必须为 0.01 的整数倍")
