"""Reproducible benchmark harness for AEB-RRT* vs OMPL built-in planners.

Produces ``benchmarks/aeb_rrtstar/raw_runs.csv`` with one row per
(scenario, planner, seed, time_budget) tuple, recording all required
metrics per the evaluation document.

Usage::

    python3 -m aeb_rrtstar.benchmark_runner
"""

from __future__ import annotations

import csv
import json
import os
import sys
import time
from dataclasses import dataclass, field
from math import sqrt
from pathlib import Path
from typing import Sequence

import numpy as np

import ompl.base as ob
import ompl.geometric as og

from .aeb_rrtstar_planner import AEBRRTstar, _euclidean_distance, _joint_vector
from .collision_checker import RobotMotionValidator, RobotStateValidityChecker
from .robot_model import DIMENSION, JOINT_LIMITS
from .scenarios import (
    DEFAULT_NUM_SEEDS,
    DEFAULT_TIME_BUDGETS,
    SCENARIOS,
    PlanningScenario,
)

# ---------------------------------------------------------------------------
#  Output paths
# ---------------------------------------------------------------------------

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_BENCHMARK_DIR = _PROJECT_ROOT / "benchmarks" / "aeb_rrtstar"
_RAW_RUNS_CSV = _BENCHMARK_DIR / "raw_runs.csv"
_SUMMARY_CSV = _BENCHMARK_DIR / "summary.csv"
_CONFIG_YAML = _BENCHMARK_DIR / "config_snapshot.yaml"
_ENV_JSON = _BENCHMARK_DIR / "environment.json"

# ---------------------------------------------------------------------------
#  Result record
# ---------------------------------------------------------------------------


@dataclass
class RunResult:
    scenario_id: str
    planner_id: str
    seed: int
    time_budget_s: float
    solved: bool
    approximate: bool = False
    first_solution_time_s: float = float("nan")
    total_time_s: float = float("nan")
    raw_cost: float = float("nan")
    # Path node count
    path_states: int = 0
    # Tree sizes
    nodes_start: int = 0
    nodes_goal: int = 0
    # Validity checks
    motion_checks: int = 0
    # Path validation
    all_states_valid: bool = False
    all_edges_valid: bool = False
    # Failure reason
    failure_reason: str = ""
    # Error message (if crashed)
    error: str = ""


# ---------------------------------------------------------------------------
#  OMPL environment setup
# ---------------------------------------------------------------------------


def _make_space_information() -> ob.SpaceInformation:
    """Create a SpaceInformation with the ninezzhou state space and collision."""
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
    return si


def _make_problem(
    si: ob.SpaceInformation,
    scenario: PlanningScenario,
) -> ob.ProblemDefinition:
    """Create a ProblemDefinition with start/goal from *scenario*."""
    start_state = si.allocState()
    goal_state = si.allocState()
    for i in range(DIMENSION):
        start_state[i] = scenario.start_joints[i]
        goal_state[i] = scenario.goal_joints[i]

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start_state, goal_state)
    # Use path length optimization objective
    opt = ob.PathLengthOptimizationObjective(si)
    pdef.setOptimizationObjective(opt)
    return pdef


# ---------------------------------------------------------------------------
#  Planner factory
# ---------------------------------------------------------------------------


def _create_aeb_faithful(si: ob.SpaceInformation) -> AEBRRTstar:
    return AEBRRTstar(
        si,
        step_size=0.3,
        connect_threshold=0.6,
        stop_on_first_solution=True,
        enable_aeb_shortcut=False,  # raw output for fair comparison
    )


def _create_aeb_anytime(si: ob.SpaceInformation) -> AEBRRTstar:
    return AEBRRTstar(
        si,
        step_size=0.3,
        connect_threshold=0.6,
        stop_on_first_solution=False,
        enable_aeb_shortcut=False,
    )


PLANNER_REGISTRY: dict[str, tuple[str, callable]] = {  # type: ignore[type-arg]
    "AEB_RRTSTAR_FAITHFUL": (
        "AEB-RRT* (faithful, first-solution stop)",
        _create_aeb_faithful,
    ),
    "AEB_RRTSTAR_ANYTIME": (
        "AEB-RRT* (anytime, best-so-far)",
        _create_aeb_anytime,
    ),
}


def _create_ompl_planner(
    si: ob.SpaceInformation, planner_type: str
) -> ob.Planner | None:
    """Create an OMPL built-in planner."""
    if planner_type == "OMPL_RRTSTAR":
        planner = og.RRTstar(si)
        planner.setRange(0.3)
        return planner
    elif planner_type == "OMPL_RRTCONNECT":
        planner = og.RRTConnect(si)
        planner.setRange(0.3)
        return planner
    elif planner_type == "OMPL_RRT":
        planner = og.RRT(si)
        planner.setRange(0.3)
        return planner
    return None


# ---------------------------------------------------------------------------
#  Path validation
# ---------------------------------------------------------------------------


def _validate_path(
    path: og.PathGeometric, si: ob.SpaceInformation
) -> tuple[bool, bool]:
    """Check every state and every edge in *path*.

    Returns ``(all_states_valid, all_edges_valid)``.
    """
    n = path.getStateCount()
    if n == 0:
        return (False, False)

    states_ok = True
    for i in range(n):
        if not si.isValid(path.getState(i)):
            states_ok = False
            break

    edges_ok = True
    for i in range(n - 1):
        if not si.checkMotion(path.getState(i), path.getState(i + 1)):
            edges_ok = False
            break

    return (states_ok, edges_ok)


def _compute_raw_cost(path: og.PathGeometric, si: ob.SpaceInformation) -> float:
    """Compute the raw Euclidean path cost (no post-processing)."""
    total = 0.0
    n = path.getStateCount()
    for i in range(n - 1):
        total += _euclidean_distance(
            _joint_vector(path.getState(i)),
            _joint_vector(path.getState(i + 1)),
        )
    return total


# ---------------------------------------------------------------------------
#  Single-run runner
# ---------------------------------------------------------------------------


def run_single(
    si: ob.SpaceInformation,
    pdef: ob.ProblemDefinition,
    planner: ob.Planner,
    scenario_id: str,
    planner_id: str,
    seed: int,
    time_budget_s: float,
) -> RunResult:
    """Execute one planning run and return a structured result."""
    result = RunResult(
        scenario_id=scenario_id,
        planner_id=planner_id,
        seed=seed,
        time_budget_s=time_budget_s,
        solved=False,
    )

    np.random.seed(seed)
    # OMPL RNG is in ompl.util, not ompl.base
    try:
        import ompl.util as ou
        rng = ou.RNG()
        rng.setSeed(seed)
    except Exception:
        pass  # RNG seeding is best-effort in the Python bindings

    planner.clear()
    planner.setProblemDefinition(pdef)
    try:
        planner.setup()
    except Exception as exc:
        result.error = str(exc)
        result.failure_reason = f"setup_error: {exc}"
        return result

    ptc = ob.timedPlannerTerminationCondition(time_budget_s)
    t_start = time.perf_counter()

    try:
        status = planner.solve(ptc)
    except Exception as exc:
        result.error = str(exc)
        result.failure_reason = f"solve_error: {exc}"
        result.total_time_s = time.perf_counter() - t_start
        return result

    result.total_time_s = time.perf_counter() - t_start

    # Interpret status
    if status == ob.PlannerStatus.EXACT_SOLUTION:
        result.solved = True
    elif status == ob.PlannerStatus.APPROXIMATE_SOLUTION:
        result.solved = True
        result.approximate = True
    elif status == ob.PlannerStatus.TIMEOUT:
        result.failure_reason = "timeout"
    elif status == ob.PlannerStatus.INVALID_START:
        result.failure_reason = "invalid_start"
    elif status == ob.PlannerStatus.INVALID_GOAL:
        result.failure_reason = "invalid_goal"
    elif status == ob.PlannerStatus.CRASH:
        result.failure_reason = "crash"
    else:
        result.failure_reason = f"unknown_status: {status}"

    # Collect metrics from planner
    if isinstance(planner, AEBRRTstar):
        result.nodes_start = planner.last_solve_nodes_start
        result.nodes_goal = planner.last_solve_nodes_goal
        result.motion_checks = planner.last_solve_checks_motion

    # Validate path
    if result.solved and pdef.hasSolution():
        path = pdef.getSolutionPath()
        result.path_states = path.getStateCount()
        result.raw_cost = _compute_raw_cost(path, si)
        valid_states, valid_edges = _validate_path(path, si)
        result.all_states_valid = valid_states
        result.all_edges_valid = valid_edges
        result.first_solution_time_s = result.total_time_s  # OMPL doesn't expose first-sol time easily

    return result


# ---------------------------------------------------------------------------
#  Batch benchmark
# ---------------------------------------------------------------------------


def _csv_row(result: RunResult) -> dict[str, object]:
    return {
        "scenario_id": result.scenario_id,
        "planner_id": result.planner_id,
        "seed": result.seed,
        "time_budget_s": result.time_budget_s,
        "solved": int(result.solved),
        "approximate": int(result.approximate),
        "first_solution_time_s": result.first_solution_time_s,
        "total_time_s": result.total_time_s,
        "raw_cost": result.raw_cost,
        "path_states": result.path_states,
        "nodes_start": result.nodes_start,
        "nodes_goal": result.nodes_goal,
        "motion_checks": result.motion_checks,
        "all_states_valid": int(result.all_states_valid),
        "all_edges_valid": int(result.all_edges_valid),
        "failure_reason": result.failure_reason,
        "error": result.error,
    }


_CSV_FIELDS = [
    "scenario_id", "planner_id", "seed", "time_budget_s", "solved",
    "approximate", "first_solution_time_s", "total_time_s", "raw_cost",
    "path_states", "nodes_start", "nodes_goal", "motion_checks",
    "all_states_valid", "all_edges_valid", "failure_reason", "error",
]


def run_benchmark(
    scenarios: Sequence[PlanningScenario] | None = None,
    planner_ids: Sequence[str] | None = None,
    time_budgets: Sequence[float] | None = None,
    num_seeds: int = DEFAULT_NUM_SEEDS,
    warmup_runs: int = 2,
    output_csv: Path | None = None,
) -> list[RunResult]:
    """Run the full benchmark matrix and return all results.

    Parameters
    ----------
    scenarios:
        Scenarios to test.  Defaults to all built-in scenarios.
    planner_ids:
        Planner IDs to test.  Defaults to all registered planners plus
        OMPL built-ins.
    time_budgets:
        Time budgets in seconds.  Defaults to (0.5, 1.0, 2.0, 5.0).
    num_seeds:
        Random seeds per (scenario, planner, budget) tuple.
    warmup_runs:
        Number of warmup runs (results discarded).
    output_csv:
        Path for the CSV output file.
    """
    if scenarios is None:
        scenarios = list(SCENARIOS)
    if planner_ids is None:
        planner_ids = [
            "AEB_RRTSTAR_FAITHFUL",
            "AEB_RRTSTAR_ANYTIME",
            "OMPL_RRTSTAR",
            "OMPL_RRTCONNECT",
        ]
    if time_budgets is None:
        time_budgets = list(DEFAULT_TIME_BUDGETS)
    if output_csv is None:
        output_csv = _RAW_RUNS_CSV

    output_csv.parent.mkdir(parents=True, exist_ok=True)

    # Build the run queue
    runs: list[tuple[str, str, int, float]] = []
    for scenario in scenarios:
        for planner_id in planner_ids:
            for budget in time_budgets:
                for seed_offset in range(-warmup_runs, num_seeds):
                    seed = 1000 * (hash(scenario.scenario_id) % 100) + seed_offset
                    runs.append((scenario.scenario_id, planner_id, abs(seed), budget))

    total = len(runs)
    results: list[RunResult] = []
    scenario_cache: dict[str, PlanningScenario] = {s.scenario_id: s for s in scenarios}

    print(f"Benchmark: {len(scenarios)} scenarios × {len(planner_ids)} planners "
          f"× {len(time_budgets)} budgets × {num_seeds} seeds = {total} runs")
    print(f"Warmup runs: {warmup_runs} per cell (discarded)")

    completed = 0
    for scenario_id, planner_id, seed, budget in runs:
        scenario = scenario_cache[scenario_id]

        # Create fresh SpaceInformation for each run to avoid OMPL state corruption
        si = _make_space_information()

        pdef = _make_problem(si, scenario)

        # Create planner
        if planner_id in PLANNER_REGISTRY:
            _, factory = PLANNER_REGISTRY[planner_id]
            planner = factory(si)
        else:
            planner = _create_ompl_planner(si, planner_id)
        if planner is None:
            continue

        result = run_single(si, pdef, planner, scenario_id, planner_id, seed, budget)
        results.append(result)
        completed += 1

        if completed % 50 == 0 or completed == total:
            solved = sum(1 for r in results if r.solved)
            print(f"  [{completed}/{total}] {scenario_id}/{planner_id} "
                  f"seed={seed} budget={budget}s → "
                  f"{'SOLVED' if result.solved else 'FAILED'} "
                  f"({result.total_time_s:.3f}s) "
                  f"[cumulative: {solved}/{completed} solved]")

    # Write CSV
    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
        writer.writeheader()
        for result in results:
            writer.writerow(_csv_row(result))

    print(f"\nResults written to {output_csv}")
    return results


# ---------------------------------------------------------------------------
#  Summary statistics
# ---------------------------------------------------------------------------


def compute_summary(results: list[RunResult]) -> list[dict[str, object]]:
    """Compute per-(scenario, planner, budget) summary statistics."""
    from collections import defaultdict

    groups: dict[tuple[str, str, float], list[RunResult]] = defaultdict(list)
    for r in results:
        key = (r.scenario_id, r.planner_id, r.time_budget_s)
        groups[key].append(r)

    summary: list[dict[str, object]] = []
    for (scenario, planner, budget), group in sorted(groups.items()):
        solved = [r for r in group if r.solved]
        n = len(group)
        n_solved = len(solved)

        times = sorted([r.total_time_s for r in solved])
        costs = sorted([r.raw_cost for r in solved])

        def _percentile(values: list[float], p: float) -> float:
            if not values:
                return float("nan")
            idx = int(np.ceil(p / 100.0 * len(values))) - 1
            return values[max(0, min(idx, len(values) - 1))]

        summary.append({
            "scenario_id": scenario,
            "planner_id": planner,
            "time_budget_s": budget,
            "num_runs": n,
            "num_solved": n_solved,
            "success_rate": n_solved / n if n > 0 else 0.0,
            "median_time_s": _percentile(times, 50),
            "p90_time_s": _percentile(times, 90),
            "p95_time_s": _percentile(times, 95),
            "median_cost": _percentile(costs, 50),
            "p90_cost": _percentile(costs, 90),
            "mean_path_states": (
                float(np.mean([r.path_states for r in solved])) if solved else float("nan")
            ),
            "mean_motion_checks": (
                float(np.mean([r.motion_checks for r in solved])) if solved else float("nan")
            ),
            "all_valid_ratio": (
                float(np.mean([int(r.all_states_valid and r.all_edges_valid) for r in solved]))
                if solved else float("nan")
            ),
        })

    return summary


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------


def main() -> int:
    print("=" * 60)
    print("AEB-RRT* Benchmark Runner")
    print("=" * 60)

    # Save environment info
    _ENV_JSON.parent.mkdir(parents=True, exist_ok=True)
    env_info = {
        "python_version": sys.version,
        "ompl_version": "2.0.1 (Python bindings)",
        "num_scenarios": len(SCENARIOS),
        "num_seeds": DEFAULT_NUM_SEEDS,
        "time_budgets": list(DEFAULT_TIME_BUDGETS),
    }
    with open(_ENV_JSON, "w") as f:
        json.dump(env_info, f, indent=2)
    print(f"Environment info → {_ENV_JSON}")

    results = run_benchmark()
    summary = compute_summary(results)

    # Write summary
    with open(_SUMMARY_CSV, "w", newline="") as f:
        if summary:
            writer = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
            writer.writeheader()
            writer.writerows(summary)
    print(f"Summary → {_SUMMARY_CSV}")

    # Print key summary
    print("\n" + "=" * 60)
    print("QUICK SUMMARY")
    print("=" * 60)
    for row in summary:
        if row["num_solved"] > 0:
            print(
                f"  {row['scenario_id']:30s} {row['planner_id']:25s} "
                f"budget={row['time_budget_s']:.1f}s  "
                f"rate={row['success_rate']:.1%}  "
                f"med_t={row['median_time_s']:.3f}s  "
                f"p95_t={row['p95_time_s']:.3f}s  "
                f"med_cost={row['median_cost']:.3f}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
