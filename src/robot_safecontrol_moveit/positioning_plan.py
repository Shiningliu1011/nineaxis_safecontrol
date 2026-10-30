from dataclasses import asdict, dataclass
from hashlib import sha256
import json
from pathlib import Path
from typing import Any

import jsonschema
import numpy as np
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Trajectory
import yaml

from .joint_table_execution import JointTableControl
from .robot_spec import DEFAULT_JOINT_NAMES


@dataclass(frozen=True)
class _ReferenceProfile:
    t: tuple[float, ...]
    j: tuple[float, ...]
    a: tuple[float, ...]
    v: tuple[float, ...]
    p: tuple[float, ...]
    brake: Any


class _CommonTrajectory:
    def __init__(self, initial, target, planner):
        self.initial, self.target = initial.copy(), target.copy()
        self.delta = self.target - self.initial
        moving = self.delta != 0
        data = InputParameter(1)
        data.control_interface = ControlInterface.Position
        data.current_position, data.current_velocity, data.current_acceleration = [0.], [0.], [0.]
        data.target_position = [1. if np.any(moving) else 0.]
        data.target_velocity, data.target_acceleration = [0.], [0.]
        for name in ("max_velocity", "max_acceleration", "max_jerk"):
            limit = getattr(planner, name)
            normalized = float(np.min(limit[moving] / np.abs(self.delta[moving]))) if np.any(moving) else 1.
            setattr(data, name, [normalized])
        data.minimum_duration = planner.minimum_duration_s
        generator, self.progress = Ruckig(1, .002), Trajectory(1)
        generator.validate_input(data, True, True)
        if generator.calculate(data, self.progress) not in (Result.Working, Result.Finished):
            raise ValueError("无法生成共同进度参考")
        self.duration = self.progress.duration
        basis = self.progress.profiles[0][0]
        self.terminal_duration, self.terminal_jerk = basis.t[-1], basis.j[-1]
        self.progress_target = data.target_position[0]
        self.profiles = [[_ReferenceProfile(
            t=tuple(basis.t), j=tuple(self.delta[joint] * np.asarray(basis.j)),
            a=tuple(self.delta[joint] * np.asarray(basis.a)),
            v=tuple(self.delta[joint] * np.asarray(basis.v)),
            p=tuple((1 - np.asarray(basis.p)) * self.initial[joint] + np.asarray(basis.p) * self.target[joint]),
            brake=basis.brake,
        ) for joint in range(9)]]

    def at_time(self, time_s):
        if time_s <= 0:
            return self.initial.copy(), np.zeros(9), np.zeros(9)
        if time_s >= self.duration:
            return self.target.copy(), np.zeros(9), np.zeros(9)
        remaining = self.duration - time_s
        if remaining <= self.terminal_duration:
            # 最后一个常 jerk 时间段从静止终点计算，保留终点位置与零导数。
            progress = self.progress_target - self.terminal_jerk * remaining ** 3 / 6
            speed = self.terminal_jerk * remaining ** 2 / 2
            acceleration = -self.terminal_jerk * remaining
        else:
            progress, speed, acceleration = np.asarray(self.progress.at_time(time_s))[:, 0]
        if not 0 <= progress <= 1 or speed < 0:
            raise ValueError(f"共同进度超出单调区间：time_s={time_s!r}, duration_s={self.duration!r}, "
                             f"progress={progress!r}, speed={speed!r}, acceleration={acceleration!r}")
        # 凸组合保留目标边界的确切定义，测量起点保持原值。
        return ((1 - progress) * self.initial + progress * self.target,
                self.delta * speed, self.delta * acceleration)


def recorded_start(path: Path, root: Path, model) -> tuple[np.ndarray, dict]:
    from .joint_table_readonly import replay_readonly_report
    from work.robot_units import joints_mm_deg_to_si

    source = path.read_bytes()
    declared = json.loads(source)
    replay = replay_readonly_report(source)
    if replay["status"] != "replayed_completed" or declared.get("status") != "readonly_completed":
        raise ValueError("定位起点需要完整的原始只读记录")
    expected = {name: sha256((root / "config" / name).read_bytes()).hexdigest()
                for name in ("drempower.yaml", "hardware_joint_zero.yaml", "j1_transmission.yaml")}
    if declared.get("configuration_sha256") != expected:
        raise ValueError("定位起点的设备坐标配置与当前文件不符")
    control = JointTableControl(root)
    latest = {row["node_id"]: row for row in replay["feedback_observations"]}
    nodes = [control._node(joint) for joint in DEFAULT_JOINT_NAMES]
    if set(latest) != set(nodes) or set(replay["nodes"]) != {str(node) for node in nodes}:
        raise ValueError("定位起点缺少完整九轴原始反馈与属性")
    finish = max(row["received_s"] for row in latest.values())
    values, observations = [], {}
    for joint, node in zip(DEFAULT_JOINT_NAMES, nodes):
        properties = replay["nodes"][str(node)]
        if properties["node_id"] != node or properties["axis_state"] != 8:
            raise ValueError(f"{joint} 起点节点或闭环状态无效")
        for name in ("axis_error", "can_error", "motor_error", "encoder_error", "controller_error"):
            if properties[name] != 0:
                raise ValueError(f"{joint} 起点存在设备错误：{name}")
        if (not np.isfinite(properties["battery_voltage"]) or properties["battery_voltage"] <= 0
                or properties["multi_circle_enabled"] != 1):
            raise ValueError(f"{joint} 起点缺少有效的电池字段或多圈配置")
        row = latest[node]
        if row["axis_error"] or not row["traj_done"] or finish - row["request_s"] > .2:
            raise ValueError(f"{joint} 起点反馈故障、尚未完成或九轴时间跨度无效")
        feedback = control.decode_state(joint, row["pos_deg"], row["vel_rpm"])
        values.append(control.project_position(joint, feedback.angle_deg))
        observations[joint] = {**row, "properties": properties}
    q = joints_mm_deg_to_si(values)
    if np.any(q < model.position_lower) or np.any(q > model.position_upper):
        raise ValueError("原始反馈起点超出共同模型范围；需要取得有效机械坐标")
    return q, {
        "source": str(path.resolve()), "source_sha256": sha256(source).hexdigest(),
        "configuration_sha256": expected, "source_status": replay["source_status"],
        "feedback_source": "recorded_device_feedback", "observations": observations,
        "device_sample_time_known": False, "live_freshness_verified": False,
        "stationary_speed_qualified": False, "coordinate_rules_verified": False,
        "start_mm_deg": values,
    }


def _common_progress(trajectory, initial, target) -> int | None:
    delta = target - initial
    moving = np.flatnonzero(delta != 0)
    if not len(moving):
        return None
    reference = int(moving[np.argmax(np.abs(delta[moving]))])
    profiles = trajectory.profiles[0]
    basis = profiles[reference]
    for joint in range(9):
        profile = profiles[joint]
        if profile.brake.duration != 0:
            raise ValueError("共同进度起点必须静止，不允许隐含制动前段")
        if delta[joint] == 0:
            if (not np.allclose(profile.p, initial[joint], rtol=0, atol=1e-12)
                    or any(np.any(np.asarray(getattr(profile, name)) != 0) for name in ("j", "a", "v"))):
                raise ValueError("零位移关节的参考没有保持原位置")
            continue
        if not np.allclose(profile.t, basis.t, rtol=0, atol=1e-10):
            raise ValueError("Ruckig 未生成可证明的共同进度时间段")
        for name in ("j", "a", "v", "p"):
            first = np.asarray(getattr(profile, name))
            second = np.asarray(getattr(basis, name))
            if name == "p":
                first, second = first - initial[joint], second - initial[reference]
            if not np.allclose(first / delta[joint], second / delta[reference], rtol=1e-9, atol=1e-9):
                raise ValueError("Ruckig 各轴参考未满足同一平滑进度")
    # 常 jerk 时间段的速度极值发生于端点或加速度为零的位置。
    for duration, velocity, acceleration, jerk in zip(basis.t, basis.v, basis.a, basis.j):
        times = [0., duration]
        if jerk != 0 and 0 < -acceleration / jerk < duration:
            times.append(-acceleration / jerk)
        if min((velocity + acceleration * value + .5 * jerk * value ** 2) / delta[reference]
               for value in times) < -1e-10:
            raise ValueError("共同进度发生反向运动")
    return reference


def synchronized_preview(root: Path, model, table, limits: dict, start_path: Path,
                         *, scene_config: Path | None = None) -> dict:
    from work.robot_units import joints_si_to_mm_deg
    from work.static_motion import ActuatorState, CandidateMotion, MotionPlanner

    initial, evidence = recorded_start(start_path, root, model)
    plant_path = root / "config/oscbf_plant.yaml"
    plant = yaml.safe_load(plant_path.read_bytes())["/**"]["ros__parameters"]
    jerk_time, minimum_duration = plant["jerk_time"], plant["minimum_duration_s"]
    control = JointTableControl(root)
    factors = np.array([control.linear.degrees_per_mm * 1000, *([180 / np.pi] * 8)])
    velocity = np.array([limits[joint].speed_rpm * 6 for joint in DEFAULT_JOINT_NAMES]) / factors
    acceleration = np.array([limits[joint].accel_rpm_s * 6 for joint in DEFAULT_JOINT_NAMES]) / factors
    planner = MotionPlanner(model.position_lower, model.position_upper, velocity, acceleration,
                            jerk_time=jerk_time, minimum_duration_s=minimum_duration)
    geometry, scene_identity = None, None
    if scene_config is not None:
        from .static_experiment_config import ExperimentConfig
        from work.static_collision import StaticGeometry
        from work.static_collision_meshes import mesh_sources

        if scene_config.suffix == ".json":
            scene = json.loads(scene_config.read_bytes())
            fields = ("frame_id", "workpiece", "table", "obstacles", "clearances", "self_collision")
            jsonschema.validate(scene, {"type": "object", "required": list(fields),
                                       "additionalProperties": False,
                                       "properties": {name: {} for name in (*fields, "config_sha256")}})
            config_identity = sha256(scene_config.read_bytes()).hexdigest()
        else:
            config = ExperimentConfig.load(scene_config, resource_root=root)
            scene, config_identity = config.scene(), config.identity
        if (scene["self_collision"] != mesh_sources(root / "portable_oscbf/config/collision_geometry_sources.json")
                or scene["clearances"]["external_m"] < .03 or scene["clearances"]["self_m"] < .01):
            raise ValueError("场景必须保留当前完整自碰撞来源及既有安全距离")
        geometry = StaticGeometry(scene)
        scene_identity = {"config_sha256": config_identity, "scene": scene,
                          "scene_sha256": sha256(json.dumps(scene, sort_keys=True, allow_nan=False).encode()).hexdigest(),
                          "physical_scene_verified": False, "model_hash": model.model_hash,
                          "tool_geometry_qualified": not bool(model.missing_sources["tool_collision_geometry"])}
    rows = []
    for row in table.rows:
        target = np.asarray(row.q_si)
        trajectory = _CommonTrajectory(initial, target, planner)
        reference = _common_progress(trajectory, initial, target)
        end = ActuatorState(*trajectory.at_time(trajectory.duration))
        stopping = planner.propose(end, np.zeros(9), period_s=minimum_duration).tracking
        motion = CandidateMotion(ActuatorState(initial, np.zeros(9), np.zeros(9)), np.zeros(9),
                                 trajectory.duration, trajectory, stopping)
        if (np.any(motion.bounds.lower < model.position_lower) or np.any(motion.bounds.upper > model.position_upper)
                or np.any(motion.bounds.max_velocity > velocity + 1e-12)
                or np.any(motion.bounds.max_acceleration > acceleration + 1e-12)
                or np.any(motion.bounds.max_jerk > planner.max_jerk + 1e-12)):
            raise ValueError(f"{row.pose_id} 共同进度参考的完整范围或导数超过限制："
                             f"lower={motion.bounds.lower.tolist()}, upper={motion.bounds.upper.tolist()}, "
                             f"velocity={motion.bounds.max_velocity.tolist()}, "
                             f"acceleration={motion.bounds.max_acceleration.tolist()}")
        profiles = [{name: list(getattr(profile, name)) for name in ("t", "j", "a", "v", "p")}
                    for profile in trajectory.profiles[0]]
        knots = np.unique(np.concatenate([
            np.array([0., trajectory.duration]), *[np.cumsum(profile["t"]) for profile in profiles]]))
        times = np.unique(np.r_[np.linspace(0., trajectory.duration, 129), knots])
        times = times[(times >= 0) & (times <= trajectory.duration)]
        samples = []
        for time_s in times:
            position, speed, accel = np.asarray(trajectory.at_time(float(time_s)))
            progress = (time_s / trajectory.duration if reference is None else
                        (position[reference] - initial[reference]) / (target[reference] - initial[reference]))
            samples.append({"time_s": float(time_s), "progress": float(progress),
                            "position_mm_deg": joints_si_to_mm_deg(position).tolist(),
                            "velocity_mm_deg_s": joints_si_to_mm_deg(speed).tolist(),
                            "acceleration_mm_deg_s2": joints_si_to_mm_deg(accel).tolist(),
                            "tcp_position_mm": (model.fk(position)[:3, 3] * 1000).tolist()})
        collision = {"status": "not_checked", "reason": "scene_config_required", "accepted": False}
        if geometry is not None:
            from work.static_motion import certify_motion

            policy = {"minimum_interval_s": 1e-5, "maximum_intervals": 4096}
            certificate = certify_motion(motion, geometry, planner, **policy)
            collision = {**asdict(certificate), "status": "proven" if certificate.accepted else "rejected",
                         "coverage": "complete_reference_and_nominal_terminal_stop",
                         "policy": policy,
                         "checker_sha256": sha256((root / "portable_oscbf/work/static_motion.py").read_bytes()).hexdigest(),
                         "mid_motion_hardware_stop_qualified": False}
        rows.append({"pose_id": row.pose_id, "duration_s": trajectory.duration,
                     "start_mm_deg": joints_si_to_mm_deg(initial).tolist(),
                     "target_mm_deg": joints_si_to_mm_deg(target).tolist(),
                     "samples": samples, "reference_profiles": profiles,
                     "profile_units": "m_rad_with_seconds", "collision": collision,
                     "bounds": {"position_lower_mm_deg": joints_si_to_mm_deg(motion.bounds.lower).tolist(),
                                "position_upper_mm_deg": joints_si_to_mm_deg(motion.bounds.upper).tolist(),
                                "maximum_velocity_mm_deg_s": joints_si_to_mm_deg(motion.bounds.max_velocity).tolist(),
                                "maximum_acceleration_mm_deg_s2": joints_si_to_mm_deg(motion.bounds.max_acceleration).tolist()}})
        initial = target
    route_accepted = geometry is not None and all(row["collision"]["accepted"] for row in rows)
    return {"status": "route_rejected" if geometry is not None and not route_accepted else "planned_reference_only",
            "duration_s": sum(row["duration_s"] for row in rows),
            "duration_scope": "reference_motion_excluding_operator_waits_and_holds",
            "start_evidence": evidence, "rows": rows, "model_hash": model.model_hash,
            "scene_identity": scene_identity, "table_sha256": table.source_sha256,
            "common_progress_verified": True, "coordinate_rules_verified": False,
            "collision_checked": geometry is not None, "reference_route_accepted": route_accepted,
            "hardware_executable": False, "device_progress_verified": False,
            "motion_commands_sent": 0, "parameter_writes_sent": 0,
            "parameter_source": {"velocity_acceleration": "requested_protocol_quantized_limits",
                                 "jerk_time_s": jerk_time, "minimum_duration_s": minimum_duration,
                                 "smoothing_config_sha256": sha256(plant_path.read_bytes()).hexdigest(),
                                 "hardware_limits_qualified": False}}
