from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass

import jax.numpy as jnp
import numpy as np

from work.jax_path_following import as_jax_path_geometry
from work.moving_reference_task import moving_reference_terms_jax
from work.static_motion import ActuatorState, CandidateMotion, MotionCertificate, certify_motion
from work.static_recovery import RecoveryRules, RecoveryState
from work.static_velocity_solver import (
    VelocityLimits, VelocitySolution, build_constraints, common_posture_velocity, solve_velocity,
)


@dataclass(frozen=True)
class ControlCandidate:
    motion: CandidateMotion
    solution: VelocitySolution
    certificate: MotionCertificate
    phase: str


class StaticControl:
    def __init__(self, path, geometry, planner, settings):
        self.path = path
        self.path_jax = as_jax_path_geometry(path)
        self.geometry, self.planner = geometry, planner
        self.settings = settings
        self.period_s = settings["period_s"]
        self.limits = VelocityLimits(planner.lower, planner.upper, planner.max_velocity)
        self.recovery = RecoveryState(RecoveryRules(**settings["recovery"]))
        self.last_trials: list[dict] = []
        self.last_measurements = ()
        self.last_terms = None
        self._proposal_sequence = 0
        self._previous_solutions = []
        self._previous_motions = []
        self._previous_motion_inputs = None

    def _motion_inputs(self, state):
        return {
            "initial": state.to_record(), "period_s": self.period_s,
            "geometry": id(self.geometry), "scene": self.geometry.scene,
            "clearances": [self.geometry.external_clearance_m, self.geometry.self_clearance_m],
            "planner": id(self.planner),
            "limits": {name: getattr(self.planner, name).tolist() for name in (
                "lower", "upper", "max_velocity", "max_acceleration", "max_jerk",
            )},
            "minimum_duration_s": self.planner.minimum_duration_s,
        }

    def propose(
            self, state: ActuatorState, progress_m: float, *,
            time_s: float, executed_feed: float) -> ControlCandidate | None:
        if not np.isfinite(progress_m) or not 0 <= progress_m <= self.path.total_length_m + 1e-12:
            raise ValueError("参考进度超出完整路径")
        progress_m = min(progress_m, self.path.total_length_m)
        transforms, spatial = self.geometry.kinematics.forward_kinematics_and_jacobian(state.position)
        end = transforms["ee_link"]
        jacobian = np.concatenate([
            spatial[3:] + np.cross(spatial[:3].T, end[:3, 3]).T, spatial[:3],
        ])
        terms = moving_reference_terms_jax(
            jnp.asarray(end[:3, 3]), jnp.asarray(end[:3, :3]), jnp.asarray(jacobian),
            self.path_jax, jnp.asarray(progress_m),
        )
        if not bool(terms.valid):
            raise ValueError("五维移动参考任务无效")
        self.last_terms = terms
        measurements = self.geometry.query(state.position)
        self.last_measurements = measurements
        constraints = build_constraints(
            state.position, self.limits, measurements, period_s=self.period_s,
            alpha=self.settings["cbf_alpha"],
            clearance_reserve_m=self.settings.get("clearance_reserve_m", 0.),
        )
        posture = common_posture_velocity(
            state.position, self.limits, terms.jacobian, measurements,
            speed_limit=self.settings["posture_speed_limit"],
            influence_m=self.settings["posture_influence_m"],
            joint_margin_fraction=self.settings["posture_joint_margin_fraction"],
        )
        sample = self.path.sample(progress_m)
        profile_feed = max(sample.feedrate_m_s, self.path.feedrate_m_s[sample.segment_index + 1])
        remaining_feed = (self.path.total_length_m - progress_m) / self.period_s
        nominal = min(
            profile_feed,
            remaining_feed,
            executed_feed + self.recovery.rules.feed_acceleration_m_s2 * self.period_s,
        )
        endpoint = progress_m >= self.path.total_length_m
        self.last_trials = []
        self._proposal_sequence += 1
        previous_solutions, self._previous_solutions = self._previous_solutions, []
        motion_inputs = self._motion_inputs(state)
        previous_motions = (
            self._previous_motions if motion_inputs == self._previous_motion_inputs else []
        )
        self._previous_motions = []
        self._previous_motion_inputs = deepcopy(motion_inputs)
        verified_motions = []
        zero_feed_solution = None
        zero_feed_trial = None

        def candidate(feed_cap):
            nonlocal zero_feed_solution, zero_feed_trial
            terminal_interval = bool(feed_cap == remaining_feed)
            trial_index = len(self.last_trials)
            origin = {
                "proposal_sequence": self._proposal_sequence,
                "proposal_time_s": time_s, "trial": trial_index,
            }
            previous = previous_solutions[trial_index] if trial_index < len(previous_solutions) else None
            solution = solve_velocity(
                terms, constraints, nominal_feed=feed_cap,
                previous_velocity=state.velocity, posture_velocity=posture,
                mode=self.settings["solver_mode"], gains=self.settings["task_gains"],
                task_weights=self.settings["task_weights"],
                posture_weight=self.settings["posture_weight"],
                smooth_weight=self.settings["smooth_weight"],
                feed_weight=self.settings["feed_weight"],
                max_velocity=self.planner.max_velocity,
                fixed_feed=terminal_interval,
                previous_zero_feed=zero_feed_solution,
                previous_problem=previous[0] if previous is not None else None,
            )
            qp_origin = previous[1] if solution.reused_problem else None
            self._previous_solutions.append((solution, qp_origin or origin))
            motion_origin = None
            for velocity, motion, certificate, source in (*verified_motions, *previous_motions):
                if np.array_equal(solution.velocity, velocity):
                    if source["proposal_sequence"] != self._proposal_sequence:
                        motion_origin = source
                    break
            else:
                motion = self.planner.propose(state, solution.velocity, period_s=self.period_s)
                certificate = certify_motion(motion, self.geometry, self.planner)
                source = origin
            verified_motions.append((solution.velocity.copy(), motion, certificate, source))
            self._previous_motions.append(verified_motions[-1])
            self.last_trials.append({
                "proposal_sequence": self._proposal_sequence, "proposal_time_s": time_s,
                "feed_cap_m_s": feed_cap, "feed_m_s": solution.feed_m_s,
                "terminal_interval": terminal_interval,
                "velocity": solution.velocity.tolist(), "certificate": asdict(certificate),
                "problem": {key: value.tolist() for key, value in solution.problem.items()},
                "solution": solution.solution.tolist(), "kkt_residual": solution.kkt_residual,
                "hard_violation": solution.hard_violation, "iterations": solution.iterations,
                "solver": solution.solver, "task_scale": solution.task_scale,
                "task_residual": solution.task_residual, "saturated": list(solution.saturated),
                "dual": None if solution.dual is None else solution.dual.tolist(),
                "qp_reused_from_trial": zero_feed_trial if solution.reused_zero_feed else None,
                "qp_reused_from_proposal": qp_origin,
                "motion_reused_from_proposal": motion_origin,
            })
            if solution.solver == "qp" and solution.feed_m_s == 0.:
                zero_feed_solution, zero_feed_trial = solution, len(self.last_trials) - 1
            return ControlCandidate(motion, solution, certificate, "")

        proposed = None
        if nominal > 0. and not endpoint:
            for factor in (1., .5, .25, .125, .0625, .03125, .015625, .0078125):
                value = candidate(nominal * factor)
                if value.certificate.accepted:
                    proposed = value
                    break
        threshold = 0. if remaining_feed <= 1e-8 else 1e-8
        positive = (proposed is not None and proposed.solution.feed_m_s > threshold
                    and progress_m + proposed.solution.feed_m_s * self.period_s > progress_m)
        feed = self.recovery.update(
            time_s=time_s, position_error_m=float(np.linalg.norm(terms.error[:3])),
            angle_rad=float(terms.angle_rad), positive_candidate=positive, endpoint=endpoint,
            nominal_feed=proposed.solution.feed_m_s if positive else 0.,
            executed_feed=executed_feed, period_s=self.period_s,
        )
        if self.recovery.phase == "FAULT":
            return None
        if feed == 0. and (proposed is None or proposed.solution.feed_m_s != 0.):
            proposed = candidate(0.)
        if proposed is None or not proposed.certificate.accepted:
            return None
        return ControlCandidate(
            proposed.motion, proposed.solution, proposed.certificate, self.recovery.phase,
        )
