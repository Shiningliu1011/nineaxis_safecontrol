from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path

import jax.numpy as jnp
import jsonschema
import numpy as np
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Synchronization, Trajectory
from scipy.optimize import linprog, minimize

from .static_experiment_config import content_hash
from .static_experiment_nodes import _resources
from .static_experiment_recording import read_record, write_json
from .static_scene_evidence import project_fixed_pose


def _position_motion(planner, initial, target, velocity=None, acceleration=None):
    from work.static_motion import ActuatorState, CandidateMotion

    state = ActuatorState(
        initial, np.zeros(9) if velocity is None else velocity,
        np.zeros(9) if acceleration is None else acceleration,
    )
    data = InputParameter(9)
    data.control_interface = ControlInterface.Position
    data.synchronization = Synchronization.Phase
    data.current_position = state.position.tolist()
    data.current_velocity, data.current_acceleration = state.velocity.tolist(), state.acceleration.tolist()
    data.target_position = np.asarray(target).tolist()
    data.target_velocity, data.target_acceleration = [0.] * 9, [0.] * 9
    data.max_velocity = planner.max_velocity.tolist()
    data.max_acceleration = planner.max_acceleration.tolist()
    data.max_jerk = planner.max_jerk.tolist()
    data.minimum_duration = planner.minimum_duration_s
    ruckig, trajectory = Ruckig(9, .002), Trajectory(9)
    ruckig.validate_input(data, True, True)
    if ruckig.calculate(data, trajectory) not in (Result.Working, Result.Finished):
        raise ValueError("独立构型见证无法生成连续运动")
    end = ActuatorState(*trajectory.at_time(trajectory.duration))
    stopping = planner.propose(end, np.zeros(9), period_s=.1).tracking
    return CandidateMotion(state, np.zeros(9), trajectory.duration, trajectory, stopping)


def certify_fixed_pose(motion, geometry, planner, position, axis, *,
                       position_tolerance_m=5e-5, angle_tolerance_rad=1e-4):
    from work.kinematics_data import JOINT_CHAIN
    from work.static_motion import certify_motion

    certificate = certify_motion(motion, geometry, planner)
    if not certificate.accepted:
        raise ValueError(f"构型见证的连续运动不满足限制：{certificate.reason}")
    coefficients = np.empty(9)
    joint = 0
    for index, row in enumerate(JOINT_CHAIN):
        if row[2] == "fixed":
            continue
        coefficients[joint] = (
            1. if row[2] == "prismatic" else
            sum(np.linalg.norm(item[3:6]) for item in JOINT_CHAIN[index + 1:])
        )
        joint += 1
    intervals = [(0., motion.duration_s)]
    checked, maximum_position, maximum_angle = 0, 0., 0.
    while intervals:
        start, end = intervals.pop()
        midpoint = (start + end) / 2
        state = motion.at_time(midpoint)
        actual_position, actual_rotation = geometry.kinematics.ee_pose(state.position)
        actual_axis = actual_rotation[:, 0]
        position_error = np.linalg.norm(actual_position - position)
        angle = np.arctan2(np.linalg.norm(np.cross(actual_axis, axis)), actual_axis @ axis)
        if position_error > position_tolerance_m or angle > angle_tolerance_rad:
            raise ValueError("构型见证没有保持固定位置和工具轴")
        low, high = motion.bounds.position_range(start, end)
        displacement = np.maximum(state.position - low, high - state.position)
        position_bound = float(position_error + coefficients @ displacement)
        angle_bound = float(angle + np.sum(displacement[1:]))
        checked += 1
        if position_bound <= position_tolerance_m and angle_bound <= angle_tolerance_rad:
            maximum_position, maximum_angle = max(maximum_position, position_bound), max(maximum_angle, angle_bound)
            continue
        if checked >= 4096 or end - start <= 1e-8:
            raise ValueError("构型见证的连续任务范围尚未证明")
        intervals.extend([(midpoint, end), (start, midpoint)])
    return {
        "motion": asdict(certificate), "task_intervals": checked,
        "maximum_position_bound_m": maximum_position, "maximum_angle_bound_rad": maximum_angle,
        "duration_s": motion.duration_s,
    }


def independent_forward_candidate(config, path, geometry, planner, q, progress):
    from work.jax_path_following import as_jax_path_geometry
    from work.moving_reference_task import moving_reference_terms_jax
    from work.static_motion import ActuatorState, certify_motion
    from work.static_velocity_solver import VelocityLimits, build_constraints

    transforms, spatial = geometry.kinematics.forward_kinematics_and_jacobian(q)
    ee = transforms["ee_link"]
    jacobian = np.vstack([spatial[3:] + np.cross(spatial[:3].T, ee[:3, 3]).T, spatial[:3]])
    terms = moving_reference_terms_jax(
        jnp.asarray(ee[:3, 3]), jnp.asarray(ee[:3, :3]), jnp.asarray(jacobian),
        as_jax_path_geometry(path), jnp.asarray(progress),
    )
    if not bool(terms.valid):
        raise ValueError("构型见证终态的移动参考任务无效")
    settings = config.values["control"]
    limits = VelocityLimits(planner.lower, planner.upper, planner.max_velocity)
    constraints = build_constraints(
        q, limits, geometry.query(q), period_s=settings["period_s"], alpha=settings["cbf_alpha"],
        clearance_reserve_m=settings.get("clearance_reserve_m", 0.),
    )
    task, rate = np.asarray(terms.jacobian), np.asarray(terms.reference_rate)
    correction = -np.asarray(settings["task_gains"]) * np.asarray(terms.error)
    sample = path.sample(progress)
    nominal = max(sample.feedrate_m_s, path.feedrate_m_s[sample.segment_index + 1])
    nominal = min(nominal, (path.total_length_m - progress) / settings["period_s"])
    attempts = []
    for factor in (1., .5, .25, .125, .0625, .03125, .015625, .0078125):
        linear = linprog(
            [0.] * 9 + [-1.],
            A_ub=np.column_stack([constraints.G, np.zeros(len(constraints.h))]),
            b_ub=constraints.h, A_eq=np.column_stack([task, -rate]), b_eq=correction,
            bounds=[(-value, value) for value in planner.max_velocity] + [(0., nominal * factor)],
            method="highs",
        )
        if not linear.success or linear.x[9] <= 1e-8:
            attempts.append({"feed_cap_m_s": nominal * factor, "status": "no_positive_linear_solution"})
            continue
        feed = float(linear.x[9])
        minimum = minimize(
            lambda velocity: float(velocity @ velocity), linear.x[:9],
            jac=lambda velocity: 2 * velocity, method="SLSQP",
            constraints=[
                {"type": "eq", "fun": lambda velocity: task @ velocity - rate * feed - correction,
                 "jac": lambda velocity: task},
                {"type": "ineq", "fun": lambda velocity: constraints.h - constraints.G @ velocity,
                 "jac": lambda velocity: -constraints.G},
            ], options={"ftol": 1e-13, "maxiter": 300},
        )
        if (not minimum.success or np.max(constraints.G @ minimum.x - constraints.h) > 1e-9
                or np.linalg.norm(task @ minimum.x - rate * feed - correction) > 1e-8):
            raise ValueError("独立正向候选的最小速度求解未通过残差检查")
        motion = planner.propose(ActuatorState(q, np.zeros(9), np.zeros(9)), minimum.x, period_s=settings["period_s"])
        certificate = certify_motion(motion, geometry, planner)
        attempts.append({
            "feed_cap_m_s": nominal * factor, "feed_m_s": feed,
            "velocity": minimum.x.tolist(), "certificate": asdict(certificate),
        })
        if certificate.accepted:
            return {"status": "proven", "attempts": attempts, "motion": motion.to_record(), "feed_m_s": feed}
    return {"status": "insufficient_evidence", "attempts": attempts}


def create_reconfiguration_witness(config, start, goal, progress, directory: Path, *,
                                   initial_velocity=None, initial_acceleration=None,
                                   source_candidate=None) -> dict:
    path, geometry, planner = _resources(config)
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if (start.shape != (9,) or goal.shape != (9,) or not np.all(np.isfinite(np.r_[start, goal]))
            or not np.isfinite(progress) or not 0 <= progress < path.total_length_m):
        raise ValueError("构型见证的关节或路径进度无效")
    position, rotation = geometry.kinematics.ee_pose(start)
    intervals = max(2, int(np.ceil(np.max(abs(goal - start)) / .003)))
    states, certificates = [start], []
    try:
        for fraction in np.linspace(0., 1., intervals + 1)[1:]:
            target = (1 - fraction) * start + fraction * goal
            q = project_fixed_pose(
                geometry, states[-1], target, planner.lower, planner.upper, position, rotation,
            )
            motion = _position_motion(
                planner, states[-1], q,
                initial_velocity if len(states) == 1 else None,
                initial_acceleration if len(states) == 1 else None,
            )
            certificates.append(certify_fixed_pose(motion, geometry, planner, position, rotation[:, 0]))
            states.append(q)
        forward = independent_forward_candidate(config, path, geometry, planner, states[-1], progress)
    except BaseException as error:
        np.savez_compressed(directory / "states.npz", positions=np.array(states))
        write_json(directory / "witness.json", {
            "status": "insufficient_evidence", "reason": repr(error),
            "config_sha256": config.identity, "segments": certificates,
            "source_candidate": source_candidate,
            "states_sha256": content_hash(directory / "states.npz"),
        })
        raise
    np.savez_compressed(directory / "states.npz", positions=np.array(states))
    report = {
        "status": forward["status"], "config_sha256": config.identity,
        "reference_progress_m": progress, "fixed_position_m": position.tolist(),
        "fixed_tool_axis": rotation[:, 0].tolist(), "segments": certificates,
        "duration_s": sum(row["duration_s"] for row in certificates),
        "joint_change": (states[-1] - start).tolist(), "forward": forward,
        "states_sha256": content_hash(directory / "states.npz"),
        "manifest": config.manifest(),
        "initial_velocity": np.asarray(initial_velocity if initial_velocity is not None else np.zeros(9)).tolist(),
        "initial_acceleration": np.asarray(initial_acceleration if initial_acceleration is not None else np.zeros(9)).tolist(),
        "source_candidate": source_candidate,
        "initial_blockage": source_candidate["blocked"] if source_candidate else "insufficient_evidence",
    }
    if np.linalg.norm(states[-1] - start) < 1e-4:
        report["status"] = "insufficient_evidence"
        report["reason"] = "未取得构型变化"
    write_json(directory / "witness.json", report)
    return report


def run_witness_request(config, request_path: Path, directory: Path):
    request = json.loads(request_path.read_text())
    vector = {"type": "array", "items": {"type": "number"}, "minItems": 9, "maxItems": 9}
    jsonschema.validate(request, {
        "type": "object", "required": ["goal", "source_run", "candidate_sequence"],
        "additionalProperties": False, "properties": {
            "goal": vector, "source_run": {"type": "string"},
            "candidate_sequence": {"type": "integer", "minimum": 1},
        },
    })
    run = Path(request["source_run"]).resolve(strict=True)
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest["config_sha256"] != config.identity:
        raise ValueError("见证配置与实际暂停候选来源不一致")
    database = run / "controller.sqlite3"
    candidate = read_record(database, request["candidate_sequence"], stream="candidate")["values"]
    blocked = bool(candidate["trials"]) and not any(
        trial["feed_m_s"] > 1e-8 and trial["certificate"]["accepted"] for trial in candidate["trials"]
    )
    if not blocked:
        raise ValueError("来源候选具有有效正向运动，不能证明初态阻塞")
    state = candidate["initial"]
    return create_reconfiguration_witness(
        config, state["position"], request["goal"], candidate["reference_progress_m"], directory,
        initial_velocity=state["velocity"], initial_acceleration=state["acceleration"],
        source_candidate={
            "run": str(run), "database_sha256": content_hash(database),
            "sequence": request["candidate_sequence"], "blocked": True,
            "request_sha256": content_hash(request_path),
        },
    )
