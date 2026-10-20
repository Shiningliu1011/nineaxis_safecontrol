from __future__ import annotations

from contextlib import closing
import json
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial.transform import Rotation

from .oscbf_trajectory import bootstrap_portable
from .static_experiment_config import content_hash
from .static_experiment_nodes import _resources
from .static_reconfiguration_evidence import _position_motion, certify_fixed_pose, independent_forward_candidate


def verify_reconfiguration(config, directory: Path) -> dict:
    directory = directory.resolve(strict=True)
    report = json.loads((directory / "witness.json").read_text())
    if report["status"] != "proven" or report["initial_blockage"] is not True:
        raise ValueError("S4 尚未取得独立构型调整见证")
    if content_hash(directory / "states.npz") != report["states_sha256"]:
        raise ValueError("构型见证状态身份不一致")
    path, geometry, planner = _resources(config)
    scene, original = config.scene(), dict(report["manifest"]["scene"])
    scene.pop("config_sha256")
    original.pop("config_sha256")
    if scene != original or config.execution_limits() != report["manifest"]["execution_limits"]:
        raise ValueError("构型见证的场景或执行器限制不一致")
    source = report["source_candidate"]
    database = Path(source["run"]) / "controller.sqlite3"
    if content_hash(database) != source["database_sha256"]:
        raise ValueError("构型见证的实际候选记录身份不一致")
    with closing(sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True)) as connection:
        value = connection.execute(
            "SELECT payload FROM samples WHERE stream='candidate' AND sequence=?", (source["sequence"],),
        ).fetchone()
    if value is None:
        raise ValueError("缺少构型见证的实际候选")
    candidate = json.loads(value[0])
    if not candidate["trials"] or any(
        trial["feed_m_s"] > 1e-8 and trial["certificate"]["accepted"] for trial in candidate["trials"]
    ):
        raise ValueError("构型见证的实际起始候选未被阻塞")
    with np.load(directory / "states.npz", allow_pickle=False) as archive:
        states = archive["positions"]
    if (states.ndim != 2 or states.shape[1] != 9 or len(states) < 2
            or not np.all(np.isfinite(states))
            or not np.array_equal(states[0], candidate["initial"]["position"])
            or report["initial_velocity"] != candidate["initial"]["velocity"]
            or report["initial_acceleration"] != candidate["initial"]["acceleration"]
            or report["reference_progress_m"] != candidate["reference_progress_m"]):
        raise ValueError("构型见证与实际起始状态不一致")
    position, rotation = geometry.kinematics.ee_pose(states[0])
    segments = []
    for index, (start, end) in enumerate(zip(states[:-1], states[1:])):
        motion = _position_motion(
            planner, start, end,
            report["initial_velocity"] if index == 0 else None,
            report["initial_acceleration"] if index == 0 else None,
        )
        segments.append(certify_fixed_pose(motion, geometry, planner, position, rotation[:, 0]))
    forward = independent_forward_candidate(
        config, path, geometry, planner, states[-1], report["reference_progress_m"],
    )
    if forward["status"] != "proven" or np.linalg.norm(states[-1] - states[0]) < 1e-4:
        raise ValueError("构型见证终态缺少正向候选或构型变化")
    return {
        "kind": "reconfiguration", "status": "proven",
        "source": str(directory), "witness_sha256": content_hash(directory / "witness.json"),
        "states_sha256": report["states_sha256"], "source_candidate": source,
        "segments": segments, "forward": forward,
        "reference_progress_m": report["reference_progress_m"],
    }


def verify_blockage(config, evidence_path: Path) -> dict:
    evidence = json.loads(evidence_path.read_text())
    if evidence["status"] != "proven" or evidence["obstacle"] not in config.scene()["obstacles"]:
        raise ValueError("S5 阻塞证据与固定障碍物不一致")
    bootstrap_portable(config.resource_root / "portable_oscbf")
    from work.nineaxis_kinematics import NineaxisKinematics
    from work.obb_collision_model import OBB_HALF_EXTENTS_M, OBB_LOCAL_CENTERS_M, OBB_LOCAL_ROTATIONS

    transforms = NineaxisKinematics().forward_kinematics(config.initial_position)
    ee, link = transforms["ee_link"], transforms["Link9"]
    tool_offset = evidence["tool_axis_offset_m"]
    point = ee[:3, 3] + tool_offset * ee[:3, 0]
    center = link[:3, 3] + link[:3, :3] @ OBB_LOCAL_CENTERS_M[9]
    local = (link[:3, :3] @ OBB_LOCAL_ROTATIONS[9]).T @ (point - center)
    if not np.isfinite(tool_offset) or np.any(abs(local) >= OBB_HALF_EXTENTS_M[9]):
        raise ValueError("阻塞见证点未处于 Link9 内部")
    path, _ = config.load_path()
    invariant = path.positions_m + tool_offset * path.rotations[:, :, 0]
    rotations = Rotation.from_matrix(path.rotations)
    angular_steps = (rotations[:-1].inv() * rotations[1:]).magnitude()
    center = np.asarray(evidence["obstacle"]["center_m"])
    radius = evidence["obstacle"]["radius_m"]
    distance = np.linalg.norm(np.maximum(abs(invariant - center) - radius, 0.), axis=1)
    uncertainty = .002 + 2 * abs(tool_offset) * np.sin(np.deg2rad(.5) / 2)
    upper = distance[:-1] + np.diff(path.arc_length_m) + abs(tool_offset) * angular_steps + uncertainty
    anchor = int(np.argmin(abs(path.arc_length_m - evidence["witness_progress_m"])))
    start, end = anchor, anchor
    while start > 0 and upper[start - 1] < config.values["external_clearance_m"]:
        start -= 1
    while end < len(upper) and upper[end] < config.values["external_clearance_m"]:
        end += 1
    if start == end:
        raise ValueError("S5 未证明必须经过的路径区间被阻断")
    return {
        "kind": "blocked_path", "status": "proven", "source": str(evidence_path.resolve()),
        "source_sha256": content_hash(evidence_path), "obstacle": evidence["obstacle"],
        "blocked_interval_m": [float(path.arc_length_m[start]), float(path.arc_length_m[end])],
        "position_tolerance_m": .002, "angle_tolerance_rad": float(np.deg2rad(.5)),
        "tool_axis_offset_m": tool_offset,
        "path_length_m": path.total_length_m,
        "maximum_distance_bound_m": float(upper[start:end].max()),
    }


def scenario_outcome(manifest, result, completed, task_pass, execution, feedback) -> dict:
    scenario = manifest["config"]["scenario"]
    evidence = manifest.get("scene_evidence")
    if scenario not in ("S4", "S5"):
        return {"status": "pass" if completed and task_pass else "fail",
                "requirement": "complete_tracking"}
    if scenario == "S4" and (not completed or not task_pass):
        return {
            "status": "fail", "reason": "tracking_incomplete_or_inaccurate",
            "requirement": "complete_tracking_and_reconfiguration",
            "independent_witness": evidence,
        }
    expected = "reconfiguration" if scenario == "S4" else "blocked_path"
    if evidence is None or evidence.get("status") != "proven" or evidence.get("kind") != expected:
        return {"status": "insufficient_evidence", "reason": "missing_independent_scene_evidence"}
    recovery = execution["recovery"]
    if scenario == "S4":
        first_running = next(
            (row["simulation_s"] for row in feedback if row["values"]["phase"] == "RUNNING"), np.inf,
        )
        events = [
            pause for pause in recovery["pauses"]
            if pause["resumed"] and pause["start_simulation_s"] > first_running
            and pause["reference_range_m"] <= 1e-12
            and max(pause["maximum_joint_change"]) >= 1e-4
        ]
        return {
            "status": "pass" if completed and task_pass and events else "fail",
            "requirement": "complete_tracking_and_reconfiguration",
            "observed_reconfigurations": events, "independent_witness": evidence,
        }
    final = feedback[-1]["values"]
    interval = evidence["blocked_interval_m"]
    stopped = bool(
        np.max(abs(np.asarray(final["velocity"]))) <= 1e-10
        and np.max(abs(np.asarray(final["acceleration"]))) <= 1e-10
    )
    faults = [row for row in recovery["faults"] if row["reason"] != "pause_timeout"]
    safe_end = (
        result["termination"] == "pause_timeout" and not faults and stopped
        and final["reference_progress_m"] < interval[0]
        and result.get("worker_exit_codes") == [0, 0]
    )
    return {
        "status": "pass" if safe_end and task_pass else "fail",
        "requirement": "safe_blocked_termination", "stopped": stopped,
        "termination_reason": result["termination"], "independent_blockage": evidence,
        "terminal_constraints": execution["terminal_constraints"],
        "uncompleted_interval_m": [final["reference_progress_m"], evidence["path_length_m"]],
        "unexpected_faults": faults,
    }
