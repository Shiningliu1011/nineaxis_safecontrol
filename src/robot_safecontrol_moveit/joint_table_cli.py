import argparse
import json
from pathlib import Path

from .joint_table import JointTable, MEASUREMENT_KINDS
from .motor_stop_latch import MotorStopLatch
from .oscbf_trajectory import bootstrap_portable


def main():
    parser = argparse.ArgumentParser(description="检查标定姿态表并导出共同模型目标")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--measurement-kind", choices=MEASUREMENT_KINDS, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.execute:
        MotorStopLatch(args.root / ".scratch/motor_control/stop.json").require_clear()
        raise RuntimeError("joint_table_live_disabled: 多轴真实反馈与停车资格尚未完成验证")
    if args.output.exists():
        raise FileExistsError(args.output)
    bootstrap_portable(args.root / "portable_oscbf")
    from work.nineaxis_model import NineaxisModel

    model = NineaxisModel.load(root=args.root)
    table = JointTable.load(args.input, model)
    report = table.preview(model, measurement_kind=args.measurement_kind)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"已检查 {len(table.rows)} 组目标；报告：{args.output}")


if __name__ == "__main__":
    main()
