import argparse
import json
from pathlib import Path

from .joint_table import JointTable, MEASUREMENT_KINDS, require_hardware_qualification
from .joint_table_execution import JointTableExecutor
from .oscbf_trajectory import bootstrap_portable


def main():
    parser = argparse.ArgumentParser(description="检查标定姿态表并导出共同模型目标")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--measurement-kind", choices=MEASUREMENT_KINDS, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--task-id")
    parser.add_argument("--measurement-dir", type=Path)
    parser.add_argument("--measurement-timeout-s", type=float, default=300.0)
    args = parser.parse_args()
    if args.execute:
        require_hardware_qualification(args.root)
    if args.output.exists():
        raise FileExistsError(args.output)
    bootstrap_portable(args.root / "portable_oscbf")
    from work.nineaxis_model import NineaxisModel

    model = NineaxisModel.load(root=args.root)
    table = JointTable.load(args.input, model)
    if args.execute:
        from .joint_table_runtime import run_joint_table

        if not args.port or not args.task_id or args.measurement_dir is None:
            parser.error("执行需要 --port、--task-id 和 --measurement-dir")
        executor = JointTableExecutor(
            table, model, measurement_kind=args.measurement_kind, task_id=args.task_id,
            feedback_source="hardware_feedback", measurement_timeout_s=args.measurement_timeout_s)
        run_joint_table(executor, port=args.port, log_path=args.output, measurement_dir=args.measurement_dir)
        return
    report = table.preview(model, measurement_kind=args.measurement_kind)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"已检查 {len(table.rows)} 组目标；报告：{args.output}")


if __name__ == "__main__":
    main()
