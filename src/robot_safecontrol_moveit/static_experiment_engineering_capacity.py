from __future__ import annotations

import ast
import hashlib
import json
import math
from pathlib import Path
import zipfile

import yaml

from .static_experiment_capacity import RecordingLimits, statistics_storage_bytes
from .static_experiment_config import METHODS, content_hash
from .static_experiment_recording import KEYFRAME_INTERVAL
from .static_experiment_volumes import filesystem_identity
from .static_recording_capacity import MAXIMUM_RECORD_NAME_BYTES, runtime_storage_configuration


BATCH_SOURCES = {
    "src/robot_safecontrol_moveit/static_formal_experiments.py",
    "src/robot_safecontrol_moveit/static_experiment_volumes.py",
    "src/robot_safecontrol_moveit/static_experiment_engineering_capacity.py",
}
CAPACITY_SOURCE = "src/robot_safecontrol_moveit/static_experiment_capacity.py"


def relative_source_map(sources: dict) -> dict:
    result = {}
    for filename, digest in sources.items():
        path = Path(filename)
        marker = next(name for name in ("build", "portable_oscbf", "src") if name in path.parts)
        relative = Path(*path.parts[path.parts.index(marker):]).as_posix()
        if relative in result:
            raise ValueError(f"源码相对路径重复：{relative}")
        result[relative] = digest
    return result


def execution_source_compatibility(sources: dict, baseline: dict, archive: Path) -> dict:
    current, previous = relative_source_map(sources), relative_source_map(baseline)
    changed = sorted(name for name in current.keys() | previous.keys()
                     if current.get(name) != previous.get(name))
    if set(changed) - BATCH_SOURCES - {CAPACITY_SOURCE}:
        return {"status": "fail", "changed_sources": changed}
    with zipfile.ZipFile(archive) as snapshot:
        for name, expected in previous.items():
            if hashlib.sha256(snapshot.read(name)).hexdigest() != expected:
                raise ValueError(f"运行源码归档身份不一致：{name}")
        previous_code = snapshot.read(CAPACITY_SOURCE).decode()
    filename = next(name for name in sources if name.endswith("/" + CAPACITY_SOURCE))
    current_code = Path(filename).read_text()
    if content_hash(Path(filename)) != current[CAPACITY_SOURCE]:
        raise ValueError("当前容量源码身份不一致")

    def lifecycle_ast(code):
        nodes = []
        for node in ast.parse(code).body:
            if isinstance(node, ast.ClassDef) and node.name == "CapacityBudget":
                break
            nodes.append(ast.dump(node, include_attributes=False))
        return hashlib.sha256("\n".join(nodes).encode()).hexdigest()

    previous_ast, current_ast = lifecycle_ast(previous_code), lifecycle_ast(current_code)
    return {
        "status": "pass" if previous_ast == current_ast else "fail",
        "changed_sources": changed, "batch_sources": sorted(BATCH_SOURCES),
        "baseline_lifecycle_ast_sha256": previous_ast,
        "current_lifecycle_ast_sha256": current_ast,
        "archive_sha256": content_hash(archive),
    }


def bounded_stream_counts(limits: RecordingLimits) -> dict[str, int]:
    manifest = limits.manifest()
    counts = {
        f"{role}/{name}": count
        for role, streams in manifest["stream_record_limits"].items()
        for name, count in streams.items()
    }
    # 同步 feedback 写入耗时严格超过 20ms 才产生该记录，最后已开始任务完整保存。
    counts["plant/recording_delay"] = min(
        counts["plant/recording_delay"], math.ceil(manifest["maximum_recording_s"] / .02) + 1,
    )
    return counts


def encoded_stream_bytes(count: int, *, keyframe: int, delta: int) -> int:
    if any(type(value) is not int or value < 0 for value in (count, keyframe, delta)):
        raise ValueError("逐流容量输入必须为非负整数")
    anchors = math.ceil(count / KEYFRAME_INTERVAL)
    return anchors * keyframe + (count - anchors) * delta


def engineering_model(audits: list[dict], limits: RecordingLimits, *, sns: bool | None = None,
                      maximum_pairs: int | None = None) -> dict:
    runs = [run for audit in audits for run in audit["runs"]]
    if not runs or any(audit["status"] != "pass" for audit in audits):
        raise ValueError("工程容量需要完整通过的真实记录审计")
    counts = bounded_stream_counts(limits)
    streams, factors, other = {}, {"plant": 1., "controller": 1.}, 0
    row_allowance = 96
    for run in runs:
        other = max(other, run["other_allocated_bytes"])
        for role, database in run["databases"].items():
            if database["format"] != 3:
                raise ValueError("工程容量需要格式 3 编码证据")
            candidate = database["streams"].get("candidate")
            eligible = (
                (sns is None or (run["config"]["control"]["solver_mode"] == "sns") == sns)
                and (maximum_pairs is None or candidate is None or candidate["maximum_pairs"] <= maximum_pairs)
            )
            if role == "plant" or eligible:
                factors[role] = max(factors[role], (
                    database["database_bytes"] - row_allowance * database["record_count"]
                ) / database["payload_bytes"])
            for name, item in database["streams"].items():
                if name == "candidate" and not eligible:
                    continue
                key = f"{role}/{name}"
                if key not in counts:
                    raise ValueError(f"记录流缺少次数界限：{key}")
                current = streams.setdefault(key, {
                    "keyframe_maximum_bytes": 0, "delta_maximum_bytes": 0,
                    "maximum_raw_bytes": 0, "fields": [], "trial_fields": [],
                    "maximum_trials": 0, "maximum_pairs": 0, "problem_shapes": {},
                })
                for field in ("keyframe_maximum_bytes", "delta_maximum_bytes", "maximum_raw_bytes",
                              "maximum_trials", "maximum_pairs"):
                    current[field] = max(current[field], item[field])
                for field in ("fields", "trial_fields"):
                    current[field] = sorted(set(current[field]) | set(item[field]))
                for field, shape in item["problem_shapes"].items():
                    current["problem_shapes"][field] = max(
                        shape, current["problem_shapes"].get(field, []),
                    )
    required = set(counts) - {
        "plant/event", "controller/event", "controller/qp_failure", "controller/sns_failure",
    }
    if required - streams.keys():
        raise ValueError(f"缺少真实记录流：{sorted(required - streams.keys())}")
    for key in counts.keys() - streams.keys():
        streams[key] = {
            "keyframe_maximum_bytes": 65536, "delta_maximum_bytes": 65536,
            "maximum_raw_bytes": 65536, "allowance": "terminal_or_solver_failure",
        }
    databases, raw = {}, 0
    for role in factors:
        payload, records = 0, 0
        for key, item in streams.items():
            if not key.startswith(role + "/"):
                continue
            item["maximum_records"] = counts[key]
            item["encoded_budget_bytes"] = encoded_stream_bytes(
                counts[key], keyframe=math.ceil(item["keyframe_maximum_bytes"] * 1.05),
                delta=math.ceil(item["delta_maximum_bytes"] * 1.05),
            )
            payload += item["encoded_budget_bytes"]
            records += counts[key]
            raw += counts[key] * item["maximum_raw_bytes"]
        databases[role] = {
            "payload_bytes": payload, "maximum_records": records,
            "sqlite_payload_factor": factors[role],
            "database_budget_bytes": math.ceil(
                (math.ceil(payload * factors[role]) + records * row_allowance + 16384) / 4096,
            ) * 4096,
        }
    statistics_bytes = {
        "plant": statistics_storage_bytes(0),
        "controller": statistics_storage_bytes(limits.manifest()["maximum_candidate_records"]),
    }
    other_retained = math.ceil(other * 1.25) + sum(statistics_bytes.values())
    retained = sum(row["database_budget_bytes"] for row in databases.values()) + other_retained
    return {
        "kind": "validated_engineering_envelope", "recording_limits": limits.manifest(),
        "candidate_group": "all" if sns is None else "sns" if sns else "qp",
        "maximum_geometry_pairs": maximum_pairs,
        "payload_margin_factor": 1.05, "sqlite_row_allowance_bytes": row_allowance,
        "streams": streams, "databases": databases,
        "other_retained_bytes": other_retained,
        "statistics_bytes": statistics_bytes,
        "retained_run_bytes": math.ceil(retained / 4096) * 4096,
        "raw_msgpack_at_sample_dimensions_bytes": raw,
        "basis": "observed_stream_encoding_with_fixed_counts_and_per_run_guard",
    }


def build_engineering_evidence(*, audits: dict[str, dict], lifecycles: dict[str, dict],
                               filesystems: dict[str, dict], sources: dict, archive: Path,
                               limits: RecordingLimits, common_control: dict,
                               output_volumes: list[dict]) -> dict:
    from .oscbf_trajectory import bootstrap_portable

    source_path = next(path for path in sources if path.endswith("/" + CAPACITY_SOURCE))
    bootstrap_portable(Path(source_path).parents[2] / "portable_oscbf")
    from work.obb_collision_model import OBB_LINK_NAMES

    runs = [run for audit in audits.values() for run in audit["runs"]]
    geometry_names = sorted({
        "workpiece", "table", *OBB_LINK_NAMES,
        *(obstacle["name"] for run in runs for obstacle in run["config"]["obstacles"]),
    })
    maximum_name_bytes = max(len(name.encode("utf-8")) for name in geometry_names)
    for run in runs:
        manifest_path = Path(run["path"]) / "manifest.json"
        if content_hash(manifest_path) != run["manifest_sha256"]:
            raise ValueError(f"容量审计清单身份变化：{manifest_path}")
        for database in run["databases"].values():
            if content_hash(Path(database["path"])) != database["sha256"]:
                raise ValueError(f"容量审计数据库身份变化：{database['path']}")
            if ("original_path" in database
                    and content_hash(Path(database["original_path"])) != database["original_sha256"]):
                raise ValueError(f"容量审计原始数据库身份变化：{database['original_path']}")
    pair_counts = sorted({
        run["databases"]["controller"]["streams"]["candidate"]["maximum_pairs"] for run in runs
    })
    models = {
        f"all-pairs-{pairs}": engineering_model(
            list(audits.values()), limits, maximum_pairs=pairs,
        ) for pairs in pair_counts
    }
    for pairs in pair_counts:
        if any(run["config"]["control"]["solver_mode"] == "sns"
               and run["databases"]["controller"]["streams"]["candidate"]["maximum_pairs"] == pairs
               for run in runs):
            models[f"sns-pairs-{pairs}"] = engineering_model(
                list(audits.values()), limits, sns=True, maximum_pairs=pairs,
            )
    methods, scenarios, scene_pairs = set(), set(), {}
    for run in runs:
        control = run["config"]["control"]
        methods.update(name for name, pair in METHODS.items()
                       if pair == (control["solver_mode"], control["recovery"]["mode"]))
        scenario = run["config"]["scenario"].lower()
        if scenario == "s3":
            scenario += f"-travel-{round(run['config']['joint_travel_scale'] * 100)}"
        scenarios.add(scenario)
        scene_pairs[scenario] = max(
            scene_pairs.get(scenario, 0),
            run["databases"]["controller"]["streams"]["candidate"]["maximum_pairs"],
        )
    lifecycle_rows, proofs, covered_volumes = [], {}, set()
    for filename, report in lifecycles.items():
        inspection = report["inspection"]
        directory = Path(inspection["directory"])
        manifest = json.loads((directory / "manifest.json").read_text())
        for name, expected in inspection["identities_sha256"].items():
            if content_hash(directory / name) != expected:
                raise ValueError(f"生命周期原件身份变化：{directory / name}")
        proof = execution_source_compatibility(sources, manifest["source_sha256"], archive)
        proofs[filename] = proof
        control = dict(manifest["config"]["control"])
        expected_control = dict(common_control)
        for item in (control, expected_control):
            item.pop("nominal_feed_multiplier")
        valid = (
            report["status"] == "pass" and inspection["status"] == "pass"
            and inspection["bound_source_ready_statistics"] and inspection["within_recording_limits"]
            and inspection.get("within_storage_limits") is True
            and inspection["recording_limits"] == limits.manifest()
            and proof["status"] == "pass" and control == expected_control
        )
        if valid:
            covered_volumes.add(filesystem_identity(directory)["filesystem_uuid"])
        lifecycle_rows.append({
            "path": filename, "valid": valid, "scenario": manifest["config"]["scenario"],
            "full_timeout": inspection.get("full_timeout_recording_verified") is True,
            "inspection": inspection,
        })
    recorder_name = "src/robot_safecontrol_moveit/static_experiment_recording.py"
    recorder_hash = relative_source_map(sources)[recorder_name]
    verified_volumes = {
        report["filesystem"]["uuid"] for report in filesystems.values()
        if report["status"] == "pass" and report["recording_source_sha256"] == recorder_hash
        and report["runs"] and all(
            row["databases"] and all(
                database["status"] == "pass" and database["random_reads"] > 0
                and database["sigkill"]["exit_code"] == -9 and database["sigkill"]["count"] >= 300
                for database in row["databases"].values()
            ) for row in report["runs"]
        )
    }
    requested_volumes = {row["filesystem_uuid"] for row in output_volumes}
    checks = {
        "methods_a_through_e": methods == set(METHODS),
        "all_scenarios_and_s3_travel": scenarios == {
            "s0", "s1", "s2", "s3-travel-100", "s3-travel-70", "s3-travel-40", "s4", "s5",
        },
        "current_s4_schema_and_control": any(row["valid"] and row["scenario"] == "S4"
                                            for row in lifecycle_rows),
        "current_s5_schema_and_control": any(row["valid"] and row["scenario"] == "S5"
                                            for row in lifecycle_rows),
        "full_timeout_with_local_recording_limits": any(
            row["valid"] and row["full_timeout"] for row in lifecycle_rows
        ),
        "complete_sim_on_output_filesystems": requested_volumes <= covered_volumes,
        "exact_values_random_reads_sigkill_on_output_filesystems": requested_volumes <= verified_volumes,
        "all_lifecycle_sources_compatible": all(row["valid"] for row in lifecycle_rows),
        "filesystem_allocation_units": all(row["allocation_unit_bytes"] == 4096 for row in output_volumes),
        "geometry_names_fit_recording_schema": maximum_name_bytes <= MAXIMUM_RECORD_NAME_BYTES,
    }
    wal_per_record = {"plant": 0., "controller": 0.}
    for row in lifecycle_rows:
        for role, recorded in row["inspection"]["recording"].items():
            count = sum(item["count"] for name, item in row["inspection"]["streams"].items()
                        if name.startswith(role + "/"))
            wal_per_record[role] = max(
                wal_per_record[role], recorded["wal_bytes_before_checkpoint"] / count,
            )
    counts = bounded_stream_counts(limits)
    wal = math.ceil(sum(
        rate * sum(count for name, count in counts.items() if name.startswith(role + "/"))
        for role, rate in wal_per_record.items()
    ) * 1.25 / 1048576) * 1048576
    per_volume = {
        "transient_run_bytes": wal, "derived_workspace_bytes": 512 * 1048576,
        "provenance_bytes": 128 * 1048576,
        "reserve_bytes": max(1073741824, *(model["raw_msgpack_at_sample_dimensions_bytes"]
                                         for model in models.values())),
    }
    workspace_uuid = filesystem_identity(Path(source_path).parent)["filesystem_uuid"]
    if workspace_uuid not in requested_volumes:
        raise ValueError("容量计划必须包含来源归档与派生处理所在文件系统")
    workspaces = {
        row["filesystem_uuid"]: {
            **per_volume,
            "derived_workspace_bytes": per_volume["derived_workspace_bytes"]
            if row["filesystem_uuid"] == workspace_uuid else 0,
            "provenance_bytes": per_volume["provenance_bytes"]
            if row["filesystem_uuid"] == workspace_uuid else 0,
        } for row in output_volumes
    }
    paths = [*audits, *lifecycles, *filesystems, str(archive)]
    runtime_storage = {
        uuid: {
            name: runtime_storage_configuration(model, wal_per_record, workspace)
            for name, model in models.items()
        } for uuid, workspace in workspaces.items()
    }
    return {
        "schema_version": 1, "status": "pass" if all(checks.values()) else "insufficient_evidence",
        "checks": checks, "source_sha256": sources, "common_control": common_control,
        "limits": {"timeout_s": limits.timeout_s, "settle_s": limits.settle_s, "period_s": limits.period_s},
        "models": models, "scene_pairs": scene_pairs,
        "geometry_names": geometry_names, "maximum_name_utf8_bytes": maximum_name_bytes,
        "per_volume_budget": per_volume, "volume_workspaces": workspaces,
        "workspace_filesystem_uuid": workspace_uuid, "wal_bytes_per_record": wal_per_record,
        "audit_sha256": {path: content_hash(Path(path)) for path in audits},
        "lifecycle_paths": list(lifecycles), "filesystem_paths": list(filesystems),
        "runtime_archive": str(archive), "source_compatibility": proofs,
        "runtime_storage": runtime_storage,
        "output_volumes": output_volumes,
        "evidence_sha256": {path: content_hash(Path(path)) for path in paths},
        "scope": {
            "feed_factors": [.5, 1., 1.5], "initial_positions": 3, "repetitions": 3,
            "failed_runs_retained": True, "extra_retries_require_new_admission": True,
            "compression_basis": "per_stream_measured_envelope_and_completed_run_allocation_guard",
            "deadline_limits_new_work_only": True,
            "derived_arrays": "serial_on_demand_feedback_clearance_arc",
        },
    }


def run_capacity_group(row: dict, evidence: dict) -> str:
    pairs = evidence["scene_pairs"][row["scene"]]
    specific = f"sns-pairs-{pairs}"
    return specific if row["method"] == "E" and specific in evidence["models"] else f"all-pairs-{pairs}"


def require_engineering_evidence(report: dict, manifest: dict, directory: Path) -> None:
    path = Path(report["engineering_evidence"])
    if report["evidence_sha256"].get(str(path)) != content_hash(path):
        raise ValueError("工程容量证据身份缺失或变化")
    evidence = json.loads(path.read_text())
    if (evidence["status"] != "pass" or evidence["source_sha256"] != manifest["source_sha256"]
            or not evidence["checks"] or not all(evidence["checks"].values())):
        raise ValueError("工程容量验证范围或代码证据不完整")
    if manifest["period_s"] != evidence["limits"]["period_s"]:
        raise ValueError("工程容量共同周期与正式清单不一致")
    if filesystem_identity(directory)["filesystem_uuid"] != evidence["workspace_filesystem_uuid"]:
        raise ValueError("正式目录与已预算来源归档文件系统不一致")
    for name in manifest["configurations"]:
        config = yaml.safe_load((directory / name).read_text())
        control, expected = dict(config["control"]), dict(evidence["common_control"])
        control.pop("nominal_feed_multiplier")
        expected.pop("nominal_feed_multiplier")
        if (control != expected or config["timeout_s"] != evidence["limits"]["timeout_s"]
                or config["settle_s"] != evidence["limits"]["settle_s"]):
            raise ValueError("工程容量公共参数与正式配置不一致")
    for row in manifest["runs"]:
        config = yaml.safe_load((directory / row["configuration"]).read_text())
        if 36 + 10 * (2 + len(config["obstacles"])) != evidence["scene_pairs"][row["scene"]]:
            raise ValueError("工程容量几何对象数量与正式场景不一致")
        if any(len(obstacle["name"].encode("utf-8")) > MAXIMUM_RECORD_NAME_BYTES
               for obstacle in config["obstacles"]):
            raise ValueError("正式场景名称超出已验证记录 schema")
    audits = {}
    for filename, expected in evidence["audit_sha256"].items():
        if report["evidence_sha256"].get(filename) != expected or content_hash(Path(filename)) != expected:
            raise ValueError(f"工程容量逐流证据身份变化：{filename}")
        audits[filename] = json.loads(Path(filename).read_text())
    for filename, expected in evidence["evidence_sha256"].items():
        if report["evidence_sha256"].get(filename) != expected or content_hash(Path(filename)) != expected:
            raise ValueError(f"工程容量原始证据身份变化：{filename}")
    rebuilt = build_engineering_evidence(
        audits=audits,
        lifecycles={name: json.loads(Path(name).read_text()) for name in evidence["lifecycle_paths"]},
        filesystems={name: json.loads(Path(name).read_text()) for name in evidence["filesystem_paths"]},
        sources=manifest["source_sha256"], archive=Path(evidence["runtime_archive"]),
        limits=RecordingLimits(**evidence["limits"]), common_control=evidence["common_control"],
        output_volumes=evidence["output_volumes"],
    )
    if rebuilt != evidence:
        raise ValueError("工程容量验证证据无法完整复算")
    models = evidence["models"]
    if max(model["retained_run_bytes"] for model in models.values()) != report["budget"]["retained_run_bytes"]:
        raise ValueError("工程容量逐流计算结果不一致")
    allowances = {
        name: value for volume in report["volumes"] for name, value in volume["run_retained_bytes"].items()
    }
    if allowances != {
        row["id"]: models[run_capacity_group(row, evidence)]["retained_run_bytes"]
        for row in manifest["runs"]
    }:
        raise ValueError("工程容量分组与正式方法不一致")
    for volume in report["volumes"]:
        for name, value in evidence["volume_workspaces"][volume["filesystem_uuid"]].items():
            if volume["budget"][name] != value:
                raise ValueError(f"工程容量工作空间位置不一致：{name}")
    for name in ("transient_run_bytes", "derived_workspace_bytes", "provenance_bytes", "reserve_bytes"):
        if report["budget"][name] != sum(row[name] for row in evidence["volume_workspaces"].values()):
            raise ValueError(f"工程容量各卷工作空间不一致：{name}")
