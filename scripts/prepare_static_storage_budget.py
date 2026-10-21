import argparse
import json
import math
from pathlib import Path
import shutil
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="生成静态实验开发验证的运行空间预算")
    parser.add_argument("--engineering-evidence", type=Path, required=True)
    parser.add_argument("--resource-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--volume", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--method", choices=tuple("ABCDE"), default="D")
    parser.add_argument("--feed-factor", type=float, default=1.)
    parser.add_argument("--period-s", type=float, choices=(.02, .04, .05, .1))
    parser.add_argument("--maximum-pairs", type=int, required=True)
    parser.add_argument("--force-wal", type=int)
    parser.add_argument("--study-index", action="store_true")
    args = parser.parse_args()
    root = args.resource_root.resolve(strict=True)
    sys.path.insert(0, str(root / "src"))

    from robot_safecontrol_moveit.static_experiment_capacity import RecordingLimits
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, content_hash
    from robot_safecontrol_moveit.static_experiment_engineering_capacity import engineering_model
    from robot_safecontrol_moveit.static_experiment_provenance import source_identity
    from robot_safecontrol_moveit.static_experiment_recording import write_json
    from robot_safecontrol_moveit.static_experiment_volumes import filesystem_identity
    from robot_safecontrol_moveit.static_period_measurement import PERIODS
    from robot_safecontrol_moveit.static_recording_capacity import runtime_storage_configuration

    evidence_path = args.engineering_evidence.resolve(strict=True)
    evidence = json.loads(evidence_path.read_text())
    audits = []
    for path, expected in evidence["audit_sha256"].items():
        if content_hash(Path(path)) != expected:
            raise ValueError(f"编码审计身份变化：{path}")
        audits.append(json.loads(Path(path).read_text()))
    sources = source_identity(root)
    volume = filesystem_identity(args.volume)
    entries = {}
    combinations = ([(period, method) for period in PERIODS for method in "ABCDE"]
                    if args.study_index else [(args.period_s, args.method)])
    for period, method in combinations:
        config = ExperimentConfig.load(
            args.config, resource_root=root, method=method,
            feed_factor=args.feed_factor, period_s=period,
        )
        model = engineering_model(
            audits, RecordingLimits(config.values["timeout_s"], config.values["settle_s"],
                                    config.values["control"]["period_s"]),
            sns=True if method == "E" else None, maximum_pairs=args.maximum_pairs,
        )
        volume_budget = dict(evidence["per_volume_budget"])
        volume_budget["transient_run_bytes"] = max(
            volume_budget["transient_run_bytes"],
            sum(math.ceil(evidence["wal_bytes_per_record"][role] * item["maximum_records"]
                          * 1.25 / 4096) * 4096 for role, item in model["databases"].items()),
        )
        volume_budget["reserve_bytes"] = max(
            volume_budget["reserve_bytes"], 2 * model["retained_run_bytes"],
        )
        budget = runtime_storage_configuration(model, evidence["wal_bytes_per_record"], volume_budget)
        if args.force_wal is not None:
            for writer in budget["writers"].values():
                writer["wal_bytes"] = args.force_wal
        name = f"period-{round(period * 1000)}-{method}" if args.study_index else args.output.stem
        required = (
            model["retained_run_bytes"]
            + sum(item["wal_bytes"] for item in budget["writers"].values())
            + budget["reserve_bytes"]
        )
        available = shutil.disk_usage(args.volume).free
        if available < required:
            raise ValueError(f"单次开发验证空间不足：{available} < {required}")
        budget.update(
            config_sha256=config.identity, source_sha256=sources,
            filesystem_uuid=volume["filesystem_uuid"], run_id=name,
            development_only=True, model=model,
            evidence_sha256=content_hash(evidence_path), audit_sha256=evidence["audit_sha256"],
            available_at_preparation_bytes=available, single_run_required_bytes=required,
        )
        target = args.output.parent / f"{name}.storage.json" if args.study_index else args.output
        if target.exists():
            raise FileExistsError(target)
        write_json(target, budget)
        entries[name] = {"path": str(target.resolve()), "sha256": content_hash(target)}
    if args.study_index:
        if args.output.exists():
            raise FileExistsError(args.output)
        write_json(args.output, entries)
    print(json.dumps(entries, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
