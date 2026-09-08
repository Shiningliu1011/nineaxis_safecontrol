#!/usr/bin/env python3
"""Run ONE planning benchmark trial.

Usage: python3 single_run.py <scenario_id> <planner_id> <seed> <budget_s>
Prints CSV row to stdout.
"""
import csv
import sys
import time

# MUST set path before any other imports
sys.path.insert(0, '/home/lsn/robot_safecontrol/src')

import numpy as np
import ompl.base as ob
import ompl.geometric as og

# These imports use the path we just set
from aeb_rrtstar.aeb_rrtstar_planner import AEBRRTstar, _euclidean_distance, _joint_vector
from aeb_rrtstar.collision_checker import RobotStateValidityChecker, RobotMotionValidator
from aeb_rrtstar.robot_model import DIMENSION, JOINT_LIMITS
from aeb_rrtstar.scenarios import SCENARIOS

_SCENARIO_MAP = {s.scenario_id: s for s in SCENARIOS}

_CSV_FIELDS = [
    "scenario_id", "planner_id", "seed", "time_budget_s", "solved",
    "approximate", "first_solution_time_s", "total_time_s", "raw_cost",
    "path_states", "nodes_start", "nodes_goal", "motion_checks",
    "all_states_valid", "all_edges_valid", "failure_reason", "error",
]


def main():
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} <scenario_id> <planner_id> <seed> <budget_s>",
              file=sys.stderr)
        sys.exit(2)

    scenario_id = sys.argv[1]
    planner_id = sys.argv[2]
    seed = int(sys.argv[3])
    budget = float(sys.argv[4])

    scenario = _SCENARIO_MAP.get(scenario_id)
    if scenario is None:
        print(f"Unknown scenario: {scenario_id}", file=sys.stderr)
        sys.exit(1)

    result = {
        "scenario_id": scenario_id, "planner_id": planner_id,
        "seed": seed, "time_budget_s": budget,
        "solved": 0, "approximate": 0,
        "first_solution_time_s": float("nan"), "total_time_s": float("nan"),
        "raw_cost": float("nan"), "path_states": 0,
        "nodes_start": 0, "nodes_goal": 0, "motion_checks": 0,
        "all_states_valid": 0, "all_edges_valid": 0,
        "failure_reason": "", "error": "",
    }

    np.random.seed(seed)

    # --- Build SpaceInformation ---
    space = ob.RealVectorStateSpace(DIMENSION)
    bounds = ob.RealVectorBounds(DIMENSION)
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        bounds.setLow(i, lo)
        bounds.setHigh(i, hi)
    space.setBounds(bounds)
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(RobotStateValidityChecker(si))
    si.setMotionValidator(RobotMotionValidator(si))
    si.setStateValidityCheckingResolution(0.01)
    si.setup()

    # --- Start/Goal ---
    s1 = si.allocState()
    s2 = si.allocState()
    for i in range(DIMENSION):
        s1[i] = scenario.start_joints[i]
        s2[i] = scenario.goal_joints[i]

    if not si.isValid(s1):
        result["failure_reason"] = "invalid_start"
        _write_csv(result)
        return
    if not si.isValid(s2):
        result["failure_reason"] = "invalid_goal"
        _write_csv(result)
        return

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(s1, s2)

    # --- Planner ---
    if planner_id == "AEB_RRTSTAR_FAITHFUL":
        planner = AEBRRTstar(si, step_size=0.3, connect_threshold=0.6,
                             stop_on_first_solution=True, enable_aeb_shortcut=False)
    elif planner_id == "AEB_RRTSTAR_ANYTIME":
        planner = AEBRRTstar(si, step_size=0.3, connect_threshold=0.6,
                             stop_on_first_solution=False, enable_aeb_shortcut=False)
    elif planner_id == "OMPL_RRTSTAR":
        planner = og.RRTstar(si)
        planner.setRange(0.3)
    elif planner_id == "OMPL_RRTCONNECT":
        planner = og.RRTConnect(si)
        planner.setRange(0.3)
    elif planner_id == "OMPL_RRT":
        planner = og.RRT(si)
        planner.setRange(0.3)
    else:
        result["failure_reason"] = "unknown_planner"
        _write_csv(result)
        return

    planner.setProblemDefinition(pdef)
    try:
        planner.setup()
    except Exception as exc:
        result["failure_reason"] = "setup_error"
        result["error"] = str(exc)
        _write_csv(result)
        return

    # --- Solve ---
    ptc = ob.timedPlannerTerminationCondition(budget)
    t_start = time.perf_counter()

    try:
        status = planner.solve(ptc)
    except Exception as exc:
        result["failure_reason"] = "solve_error"
        result["error"] = str(exc)
        result["total_time_s"] = time.perf_counter() - t_start
        _write_csv(result)
        return

    result["total_time_s"] = time.perf_counter() - t_start

    if status == ob.PlannerStatus.EXACT_SOLUTION:
        result["solved"] = 1
    elif status == ob.PlannerStatus.APPROXIMATE_SOLUTION:
        result["solved"] = 1
        result["approximate"] = 1
    elif status == ob.PlannerStatus.TIMEOUT:
        result["failure_reason"] = "timeout"
    elif status == ob.PlannerStatus.INVALID_START:
        result["failure_reason"] = "invalid_start"
    elif status == ob.PlannerStatus.INVALID_GOAL:
        result["failure_reason"] = "invalid_goal"
    else:
        result["failure_reason"] = f"status:{status}"

    # --- Tree stats ---
    if isinstance(planner, AEBRRTstar):
        result["nodes_start"] = planner.last_solve_nodes_start
        result["nodes_goal"] = planner.last_solve_nodes_goal
        result["motion_checks"] = planner.last_solve_checks_motion

    # --- Path validation ---
    if result["solved"] and pdef.hasSolution():
        path = pdef.getSolutionPath()
        result["path_states"] = path.getStateCount()
        result["first_solution_time_s"] = result["total_time_s"]

        # Raw cost
        total = 0.0
        n = path.getStateCount()
        for i in range(n - 1):
            total += _euclidean_distance(
                _joint_vector(path.getState(i)),
                _joint_vector(path.getState(i + 1)),
            )
        result["raw_cost"] = total

        # Validate
        sv = all(si.isValid(path.getState(i)) for i in range(n))
        ev = True
        for i in range(n - 1):
            if not si.checkMotion(path.getState(i), path.getState(i + 1)):
                ev = False
                break
        result["all_states_valid"] = int(sv)
        result["all_edges_valid"] = int(ev)

    _write_csv(result)


def _write_csv(result):
    writer = csv.DictWriter(sys.stdout, fieldnames=_CSV_FIELDS)
    writer.writeheader()
    writer.writerow(result)
    sys.stdout.flush()


if __name__ == "__main__":
    main()
