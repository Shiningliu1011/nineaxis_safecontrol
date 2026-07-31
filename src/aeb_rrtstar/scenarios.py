"""Fixed test scenarios for the AEB-RRT* benchmark.

Each scenario defines start/goal configurations and a label.  Scenarios
are version-controlled so that the benchmark is reproducible.  The
collision environment (obstacles) always uses the project's obstacles.yaml.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from .robot_model import JOINT_LIMITS

# ---------------------------------------------------------------------------
#  Helper to build joint vectors
# ---------------------------------------------------------------------------


def _mid(low: float, high: float) -> float:
    return (low + high) / 2.0


def _lerp(low: float, high: float, t: float) -> float:
    return low + t * (high - low)


# ---------------------------------------------------------------------------
#  Scenario definitions
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlanningScenario:
    scenario_id: str
    description: str
    start_joints: tuple[float, ...]
    goal_joints: tuple[float, ...]
    category: str  # e.g. "easy", "medium", "hard", "regression"


# Pre-compute some useful positions
_ZEROS: tuple[float, ...] = tuple(0.0 for _ in range(9))
_MID: tuple[float, ...] = tuple(_mid(lo, hi) for lo, hi in JOINT_LIMITS)
_HALF_TO_MAX: tuple[float, ...] = tuple(_lerp(lo, hi, 0.75) for lo, hi in JOINT_LIMITS)
_QUARTER_TO_MAX: tuple[float, ...] = tuple(_lerp(lo, hi, 0.55) for lo, hi in JOINT_LIMITS)
_NEAR_LIMITS: tuple[float, ...] = tuple(
    _lerp(lo, hi, 0.9) for lo, hi in JOINT_LIMITS
)
# A configuration where J1 is raised and some revolute joints are at extremes
_EXTREME_MIX: tuple[float, ...] = (
    0.5,        # J1 near top
    1.2,        # J2 near +limit
    -0.8,       # J3 mid
    1.4,        # J4 near +limit
    -2.0,       # J5 mid-range
    0.0,        # J6 centre
    -1.0,       # J7
    1.0,        # J8
    -0.5,       # J9
)
# Near-zero but with J1 raised slightly
_NEAR_ZERO_RAISED: tuple[float, ...] = (
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
)

SCENARIOS: tuple[PlanningScenario, ...] = (
    # ---- Easy ----
    PlanningScenario(
        scenario_id="easy_zeros_to_mid",
        description="Zero config → mid-range config",
        start_joints=_ZEROS,
        goal_joints=_MID,
        category="easy",
    ),
    PlanningScenario(
        scenario_id="easy_zeros_to_quarter",
        description="Zero config → 55%-range config",
        start_joints=_ZEROS,
        goal_joints=_QUARTER_TO_MAX,
        category="easy",
    ),
    PlanningScenario(
        scenario_id="easy_near_start",
        description="Two nearby configurations (short path)",
        start_joints=_NEAR_ZERO_RAISED,
        goal_joints=tuple(
            _NEAR_ZERO_RAISED[i] + 0.1 * (JOINT_LIMITS[i][1] - JOINT_LIMITS[i][0])
            for i in range(9)
        ),
        category="easy",
    ),
    # ---- Medium ----
    PlanningScenario(
        scenario_id="medium_zeros_to_halfmax",
        description="Zero config → 75%-range config",
        start_joints=_ZEROS,
        goal_joints=_HALF_TO_MAX,
        category="medium",
    ),
    PlanningScenario(
        scenario_id="medium_mid_to_extreme",
        description="Mid-range → near-limit config",
        start_joints=_MID,
        goal_joints=_EXTREME_MIX,
        category="medium",
    ),
    # ---- Hard ----
    PlanningScenario(
        scenario_id="hard_extreme_to_extreme",
        description="Extreme config → opposite extreme through obstacles",
        start_joints=_NEAR_LIMITS,
        goal_joints=_EXTREME_MIX,
        category="hard",
    ),
    PlanningScenario(
        scenario_id="hard_zeros_to_extreme",
        description="Zero → extreme mixed config",
        start_joints=_ZEROS,
        goal_joints=_EXTREME_MIX,
        category="hard",
    ),
    # ---- Regression (project scenarios) ----
    PlanningScenario(
        scenario_id="regression_zero_to_ik_start",
        description="Zero (home) → first IK waypoint (transition planning)",
        start_joints=_ZEROS,
        goal_joints=(
            0.200,        # J1 (raised)
            -0.0447331,   # J2
            0.640448,     # J3
            0.320403,     # J4
            0.163702,     # J5
            0.14385,      # J6
            -0.707206,    # J7
            0.390431,     # J8
            0.47935,      # J9
        ),
        category="regression",
    ),
)

# Default time budgets (seconds)
DEFAULT_TIME_BUDGETS: tuple[float, ...] = (0.5, 1.0, 2.0, 5.0)

# Default number of random seeds
DEFAULT_NUM_SEEDS: int = 30
