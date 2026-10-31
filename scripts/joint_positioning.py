import sys
from pathlib import Path

# J1 使用 mm，J2–J9 使用 °；全部数值为相对已有机械零位的绝对目标。
TARGET_JOINTS = {
    "J1": 50.0, "J2": 10.0, "J3": -20.0, "J4": 30.0, "J5": -30.0,
    "J6": 60.0, "J7": 10.0, "J8": 20.0, "J9": 30.0,
}

# J1 使用 mm/s 和 mm/s²，J2–J9 使用 °/s 和 °/s²。
MAXIMUM_SPEED = {
    "J1": 0.25, "J2": 2.46, "J3": 2.46, "J4": 2.46, "J5": 2.46,
    "J6": 2.46, "J7": 2.46, "J8": 2.46, "J9": 2.46,
}
ACCELERATION = {
    "J1": 0.05, "J2": 0.3, "J3": 0.3, "J4": 0.3, "J5": 0.3,
    "J6": 0.3, "J7": 0.3, "J8": 0.3, "J9": 0.3,
}

CSV_PATH = None
SERIAL_PORT = "/dev/serial/by-id/usb-Dr-Tech_DR-USB_CAN_87949380174B-if00"

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from robot_safecontrol_moveit.joint_positioning import run_positioning_cli

if __name__ == "__main__":
    raise SystemExit(run_positioning_cli(root=ROOT, target_joints=TARGET_JOINTS,
                                        maximum_speed=MAXIMUM_SPEED, acceleration=ACCELERATION,
                                        csv_path=CSV_PATH, port=SERIAL_PORT))
