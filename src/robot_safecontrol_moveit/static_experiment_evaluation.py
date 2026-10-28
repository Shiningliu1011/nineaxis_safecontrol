from __future__ import annotations

import json
import hashlib
from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
import tempfile

import jax
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .oscbf_trajectory import bootstrap_portable
from .static_clearance_evaluation import evaluate_clearances
from .static_experiment_config import content_hash
from .static_experiment_recording import read_stream, write_json
from .static_experiment_provenance import (
    recording_hashes, relocate_manifest, source_identity, validate_storage_migration,
)
from .static_execution_evaluation import evaluate_execution, arc_statistics, ordered_coverage
from .static_cycle_timing import LEGACY_CYCLE_TIMING_BOUNDARY


def distribution(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(values)
    measured = values[valid]
    return {
        "samples": len(values), "valid": int(valid.sum()),
        "invalid": int((~valid).sum()),
        "rms": float(np.sqrt(np.mean(measured ** 2))) if len(measured) else None,
        "p95": float(np.percentile(measured, 95)) if len(measured) else None,
        "maximum": float(measured.max()) if len(measured) else None,
        "minimum": float(measured.min()) if len(measured) else None,
    }


def ordered_projection(geometry, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    progress = np.empty(len(positions))
    distances = np.empty(len(positions))
    anchor = 0
    previous = 0.0
    for index, position in enumerate(positions):
        value, anchor = geometry.project_local(
            position, anchor_segment=anchor, half_window_segments=96,
            half_window_m=0.01,
            anchor_progress_m=previous,
        )
        progress[index] = value
        previous = value
        nearest = geometry.sample(value).position_m
        distances[index] = np.linalg.norm(position - nearest)
    return progress, distances


def evaluate_run(directory: Path, *, resource_root: Path, save_samples: bool = False,
                 output_directory: Path | None = None) -> dict:
    directory = Path(directory)
    output = directory if output_directory is None else Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    manifest = relocate_manifest(json.loads((directory / "manifest.json").read_text()), resource_root)
    result = json.loads((directory / "result.json").read_text())
    identities = {
        "manifest_sha256": content_hash(directory / "manifest.json"),
        "result_sha256": content_hash(directory / "result.json"),
        "config_sha256": manifest["config_sha256"],
        "path_sha256": content_hash(directory / "path.npz"),
        "records_sha256": recording_hashes(directory),
        "evaluation_sources_sha256": source_identity(Path(resource_root)),
    }
    migration = directory / "storage_migration.json"
    if migration.exists():
        identities["storage_migration_sha256"] = validate_storage_migration(directory, identities)
    identity = hashlib.sha256(json.dumps(
        manifest["config"], sort_keys=True, allow_nan=False,
    ).encode()).hexdigest()
    if identity != manifest["config_sha256"]:
        raise ValueError("保存的实验配置身份不一致")
    for name, resource in manifest["resources"].items():
        # 独立重放使用 manifest 内已核查的最终配置，源 YAML 另行保留来源信息。
        if name == "experiment_configuration":
            continue
        if content_hash(Path(resource["path"])) != resource["sha256"]:
            raise ValueError(f"运行资源身份变化：{resource['path']}")
    for filename, expected in manifest["source_sha256"].items():
        if (Path(filename).name in ("nineaxis_manipulator_jax.py", "kinematics_data.py")
                and content_hash(Path(filename)) != expected):
            raise ValueError(f"运行代码身份变化：{filename}")
    bootstrap_portable(Path(resource_root) / "portable_oscbf")
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.path_following import PathGeometry

    jax.config.update("jax_enable_x64", True)
    with np.load(directory / "path.npz", allow_pickle=False) as archive:
        geometry = PathGeometry.from_arrays({key: archive[key] for key in archive.files})
    if content_hash(directory / "path.npz") != manifest["path_sha256"]:
        raise ValueError("path.npz 身份不一致")
    feedback = (
        read_stream(directory / "plant.sqlite3", "feedback")
        if (directory / "plant.sqlite3").is_file() else []
    )
    commands = (
        read_stream(directory / "controller.sqlite3", "command")
        if (directory / "controller.sqlite3").is_file() else []
    )
    if len(feedback) < 2 or not commands:
        report = {
            **identities,
            "verdict": "insufficient_evidence", "completed": False,
            "task_verdict": "insufficient_evidence",
            "functional_verdict": "insufficient_evidence",
            "runtime_verdict": "insufficient_evidence",
            "termination": result, "reason": "缺少反馈或命令记录",
        }
        write_json(output / "feedback_report.json", report)
        return report
    command_times = np.array([row["monotonic_s"] for row in commands])
    feedback_times = np.array([row["monotonic_s"] for row in feedback])
    # 保留第一条命令之前最近的反馈，完整覆盖起始状态。
    start = max(0, int(np.searchsorted(feedback_times, command_times[0])) - 1)
    feedback = feedback[start:]
    feedback_times = feedback_times[start:]
    indices = np.maximum(np.searchsorted(command_times, feedback_times, side="right") - 1, 0)
    measured = np.array([row["values"]["q"] for row in feedback])
    simulation_times = np.array([row["simulation_s"] for row in feedback])
    if (measured.shape != (len(feedback), 9) or not np.all(np.isfinite(measured))
            or np.any(np.diff(feedback_times) <= 0)
            or not np.allclose(np.diff(simulation_times), 0.002, rtol=0, atol=1e-10)):
        raise ValueError("反馈位置、时间或 500 Hz 采样不完整")
    robot = NineaxisManipulatorJAX()
    positions = np.asarray(jax.jit(jax.vmap(robot.ee_position))(measured))
    rotations = np.asarray(jax.jit(jax.vmap(robot.ee_rotation))(measured))
    progress, distances = ordered_projection(geometry, positions)
    if "reference_progress_m" in feedback[0]["values"]:
        reference_progress = np.array([row["values"]["reference_progress_m"] for row in feedback])
        if (np.any(reference_progress < 0) or np.any(reference_progress > geometry.total_length_m + 1e-12)
                or np.any(np.diff(reference_progress) < -1e-12)):
            raise ValueError("plant 结算的参考进度无效")
        reference_progress = np.minimum(reference_progress, geometry.total_length_m)
        if geometry.continuous_reference is not None:
            chunks = [geometry.continuous_reference.query(values / geometry.total_length_m)
                      for values in np.array_split(reference_progress, max(1, int(np.ceil(len(reference_progress) / 1024))))]
            reference_positions = np.concatenate([sample.position_m for sample in chunks])
            reference_axes = np.concatenate([sample.tool_axis for sample in chunks])
        else:
            reference_positions = np.column_stack([
                np.interp(reference_progress, geometry.arc_length_m, geometry.positions_m[:, axis])
                for axis in range(3)
            ])
            reference_axes = Slerp(
                geometry.arc_length_m, Rotation.from_matrix(geometry.rotations),
            )(reference_progress).as_matrix()[:, :, 0]
        phases = np.array([row["values"]["phase"] for row in feedback])
    else:
        reference_positions = np.array([commands[index]["values"]["reference_position_m"] for index in indices])
        reference_axes = np.array([commands[index]["values"]["reference_axis"] for index in indices])
        reference_progress = np.array([commands[index]["values"]["reference_progress_m"] for index in indices])
        phases = np.array([
            "ENDPOINT_HOLD" if commands[index]["values"]["at_endpoint"] else "RUNNING"
            for index in indices
        ])
    axes = rotations[:, :, 0]
    angles = np.arctan2(np.linalg.norm(np.cross(axes, reference_axes), axis=1),
                        np.einsum("ij,ij->i", axes, reference_axes))
    cylinder = manifest["scene"]["workpiece"]
    relative = positions - cylinder["center_m"]
    radial = relative - np.outer(relative @ cylinder["axis"], cylinder["axis"])
    standoff = np.linalg.norm(radial, axis=1) - cylinder["radius_m"]
    endpoint_position = np.linalg.norm(positions - geometry.positions_m[-1], axis=1)
    endpoint_axis = geometry.rotations[-1, :, 0]
    endpoint_angle = np.arctan2(
        np.linalg.norm(np.cross(axes, endpoint_axis), axis=1), axes @ endpoint_axis,
    )
    series = {
        "path_distance_m": distances,
        "reference_position_error_m": np.linalg.norm(positions - reference_positions, axis=1),
        "tool_axis_angle_rad": angles,
        "tcp_workpiece_distance_m": standoff,
    }
    metrics = {name: distribution(values) for name, values in series.items()}
    end_condition = (endpoint_position <= 0.0005) & (endpoint_angle <= np.deg2rad(0.5))
    final_valid_start = len(end_condition)
    while final_valid_start > 0 and end_condition[final_valid_start - 1]:
        final_valid_start -= 1
    held_s = (
        simulation_times[-1] - simulation_times[final_valid_start]
        if final_valid_start < len(end_condition) else 0.0
    )
    coverage_report = ordered_coverage(
        progress, reference_progress, geometry.total_length_m,
        float(np.linalg.norm(positions[0] - geometry.positions_m[0])),
        float(endpoint_position[-1]),
    )
    coverage = coverage_report["ordered_full_path"]
    task_pass = bool(
        metrics["path_distance_m"]["rms"] <= 0.00015
        and metrics["path_distance_m"]["p95"] <= 0.0005
        and metrics["path_distance_m"]["maximum"] <= 0.002
        and metrics["tool_axis_angle_rad"]["rms"] <= np.deg2rad(0.05)
        and metrics["tool_axis_angle_rad"]["maximum"] <= np.deg2rad(0.5)
    )
    completed = bool(
        result["termination"] == "completed" and coverage
        and held_s >= manifest["config"]["endpoint_hold_s"]
    )
    clearance = evaluate_clearances(output, manifest["scene"], measured, save_samples=save_samples)
    geometry_pass = all(clearance[kind]["status"] == "pass" for kind in ("external", "self"))
    geometry_fail = any(clearance[kind]["status"] == "fail" for kind in ("external", "self"))
    execution = evaluate_execution(directory, manifest, feedback, commands)
    for event in execution["recovery"]["events"]:
        selected = ((simulation_times >= event["time_s"])
                    & (simulation_times < event["following_interval_end_s"]))
        event["maximum_position_error_m"] = float(series["reference_position_error_m"][selected].max())
        event["maximum_tool_axis_angle_rad"] = float(angles[selected].max())
    execution_status = [
        execution[name]["status"] for name in ("qp", "sns", "joint_limits", "reference_settlement")
    ]
    execution_pass = all(value in ("pass", "not_applicable") for value in execution_status)
    execution_fail = "fail" in execution_status
    from .static_scenario_evidence import scenario_outcome
    scenario = scenario_outcome(manifest, result, completed, task_pass, execution, feedback)
    functional = (
        "fail" if not task_pass or scenario["status"] == "fail" or geometry_fail or execution_fail else
        "pass" if geometry_pass and execution_pass and scenario["status"] == "pass"
        else "insufficient_evidence"
    )
    runtime = execution["timing"]["status"]
    completion_time = None
    if completed:
        reference_end = int(np.searchsorted(reference_progress, geometry.total_length_m - 1e-12))
        completion_time = float(
            simulation_times[max(reference_end, final_valid_start)]
            + manifest["config"]["endpoint_hold_s"] - simulation_times[0],
        )
    report = {
        **identities,
        "schema_version": 1, "measurement_boundary": "simulated_actuator_feedback",
        "config_sha256": manifest["config_sha256"], "path_sha256": manifest["path_sha256"],
        "evaluator_sha256": content_hash(Path(__file__)),
        "termination": result, "completed": completed,
        "completion_time_s": completion_time,
        "tracking_precision_verdict": "pass" if task_pass else "fail",
        "task_verdict": scenario["status"],
        "functional_verdict": functional,
        "runtime_verdict": runtime,
        "scenario": scenario,
        "verdict": ("fail" if "fail" in (functional, runtime) else
                    "pass" if functional == runtime == "pass" else "insufficient_evidence"),
        "metrics": metrics,
        "execution": execution,
        "arc_statistics": arc_statistics(
            output, geometry.total_length_m, progress, series, save_samples=save_samples,
        ),
        "phases": {
            phase: {"duration_s": float(np.count_nonzero(phases == phase) * .002),
                    **{name: distribution(values[phases == phase]) for name, values in series.items()}}
            for phase in np.unique(phases)
        },
        "coverage": coverage_report,
        "endpoint": {"position_error_m": float(endpoint_position[-1]),
                     "tool_axis_angle_rad": float(endpoint_angle[-1]), "held_s": float(held_s)},
        "feedback": {
            "samples": len(feedback), "simulation_frequency_hz": 500.0,
            "wall_frequency_hz": (len(feedback) - 1) / (feedback_times[-1] - feedback_times[0]),
            "simulation_duration_s": float(simulation_times[-1] - simulation_times[0]),
        },
        "external_clearance": {**clearance["external"], "required_m": .03},
        "self_clearance": {**clearance["self"], "required_m": .01},
    }
    if save_samples:
        np.savez_compressed(
            output / "feedback_metrics.npz", monotonic_s=feedback_times,
            simulation_s=simulation_times, positions_m=positions, tool_axes=axes,
            projected_progress_m=progress, reference_progress_m=reference_progress,
            phase=phases, **series,
        )
    report["derived_data_policy"] = "saved" if save_samples else "on_demand"
    report["derived_data_sha256"] = {
        filename: content_hash(output / filename)
        for filename in ("feedback_metrics.npz", "clearance_metrics.npz", "feedback_arc_metrics.npz")
    } if save_samples else {}
    write_json(output / "feedback_report.json", report)
    write_json(output / "clearance_report.json", clearance)
    rows = [
        f"# {manifest['config']['scenario']} 反馈评价",
        "",
        f"任务评价：`{report['task_verdict']}`；综合证据：`{report['verdict']}`。",
        f"结束原因：`{result['termination']}`；有序完整覆盖：`{coverage}`。",
        "",
        "| 指标 | RMS | P95 | 最大值 |",
        "|---|---:|---:|---:|",
    ]
    for name, scale, label in (
        ("path_distance_m", 1000, "路径距离（mm）"),
        ("reference_position_error_m", 1000, "参考位置误差（mm）"),
        ("tool_axis_angle_rad", 180 / np.pi, "真实工具轴角度（°）"),
        ("tcp_workpiece_distance_m", 1000, "TCP 作业距离（mm）"),
    ):
        values = metrics[name]
        rows.append(f"| {label} | {values['rms'] * scale:.6f} | "
                    f"{values['p95'] * scale:.6f} | {values['maximum'] * scale:.6f} |")
    rows.extend([
        "", f"有效反馈数量：{len(feedback)}；终点保持：{held_s:.6f} s。",
        f"外部间距：{clearance['external']['status']}；自碰撞间距：{clearance['self']['status']}。",
        f"关节限制：{execution['joint_limits']['status']}；进度结算：{execution['reference_settlement']['status']}。",
        f"QP：{execution['qp']['status']}；完整周期：{execution['timing']['status']}。",
        "全部测量来自 Ruckig 仿真被控对象的关节反馈。",
    ])
    (output / "feedback_report.md").write_text("\n".join(rows) + "\n", encoding="utf-8")
    return report


@contextmanager
def evaluation_samples(directory: Path, resource_root: Path, report: dict):
    if report.get("derived_data_policy") != "on_demand":
        if not report.get("derived_data_sha256"):
            raise ValueError("绘图需要完整的反馈评价，请执行 replay")
        yield directory, report
        return
    temporary_root = resource_root / ".scratch/static-experiments"
    temporary_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="derived-", dir=temporary_root) as temporary:
        output = Path(temporary)
        recomputed = evaluate_run(
            directory, resource_root=resource_root, save_samples=True, output_directory=output,
        )
        excluded = {"evaluation_sources_sha256", "evaluator_sha256",
                    "derived_data_policy", "derived_data_sha256"}
        expected = deepcopy({key: value for key, value in report.items() if key not in excluded})
        actual = deepcopy({key: value for key, value in recomputed.items() if key not in excluded})
        if ("boundary" not in expected.get("execution", {}).get("timing", {})
                and actual["execution"]["timing"]["boundary"] == LEGACY_CYCLE_TIMING_BOUNDARY):
            for item in (expected, actual):
                item.pop("verdict", None)
                item.pop("runtime_verdict", None)
                for key in ("status", "schema_version", "boundary", "eligible_for_period", "sidecar_sha256"):
                    item["execution"]["timing"].pop(key, None)
        if actual != expected:
            raise ValueError("按需生成的评价与保存的报告不一致，请核查后执行 replay")
        yield output, recomputed
