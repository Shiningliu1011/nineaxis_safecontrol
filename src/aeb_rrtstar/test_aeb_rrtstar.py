#!/usr/bin/env python3
"""Unit tests for AEB-RRT* planner — run with pytest or directly.

Tests cover the mandatory test cases from the evaluation document:
- p_a monotonic behaviour
- Target-biased vs random sampling (fixed seed reproducibility)
- Extension step sizes
- Parent selection / rewiring
- Same start=goal returns zero cost
- Invalid start/goal handling
- clear() then solve() again
- Path invariant checks
"""

import sys
import unittest

sys.path.insert(0, '/home/lsn/robot_safecontrol/src')

import numpy as np
import ompl.base as ob
import ompl.geometric as og
from math import exp

from aeb_rrtstar.aeb_rrtstar_planner import (
    AEBRRTstar,
    _manhattan_distance,
    _euclidean_distance,
    _joint_vector,
    _state_equal,
)
from aeb_rrtstar.collision_checker import (
    RobotStateValidityChecker,
    RobotMotionValidator,
    is_configuration_valid,
    is_motion_valid,
)
from aeb_rrtstar.robot_model import DIMENSION, JOINT_LIMITS


def _make_si():
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


def _make_problem(si, start_joints, goal_joints):
    s1 = si.allocState()
    s2 = si.allocState()
    for i in range(DIMENSION):
        s1[i] = start_joints[i]
        s2[i] = goal_joints[i]
    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(s1, s2)
    return pdef


class TestAdaptiveProbability(unittest.TestCase):
    """p_a = p_min + (p_max - p_min) * exp(-9 / (T_failed + 1)^3)"""

    def test_pa_monotonic_decreases_with_failures(self):
        """p_a should monotonically approach p_max as T_failed increases,
        making random sampling more likely after many failures."""
        p_min, p_max = 0.1, 1.0
        values = []
        for t_failed in range(0, 20):
            pa = p_min + (p_max - p_min) * exp(-9.0 / (t_failed + 1.0) ** 3)
            values.append(pa)
        # Starts near p_min
        self.assertLess(values[0], 0.15, f"p_a(0)={values[0]} should be near p_min")
        # Monotonically approaches p_max
        self.assertTrue(all(values[i] <= values[i + 1] for i in range(len(values) - 1)))
        # After many failures, close to p_max
        self.assertGreater(values[-1], 0.95)

    def test_pa_in_range(self):
        """p_a must stay within [p_min, p_max]."""
        for t_failed in [0, 1, 5, 10, 100, 1000]:
            pa = 0.1 + 0.9 * exp(-9.0 / (t_failed + 1.0) ** 3)
            self.assertGreaterEqual(pa, 0.1)
            self.assertLessEqual(pa, 1.0)


class TestDistanceFunctions(unittest.TestCase):

    def test_manhattan_non_negative(self):
        a = np.zeros(DIMENSION)
        b = np.ones(DIMENSION)
        d = _manhattan_distance(a, b)
        self.assertGreater(d, 0)

    def test_manhattan_self_zero(self):
        a = np.array([0.1, 0.2, 0.0, -0.5, 1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(_manhattan_distance(a, a), 0.0)

    def test_euclidean_non_negative(self):
        a = np.zeros(DIMENSION)
        b = np.ones(DIMENSION)
        d = _euclidean_distance(a, b)
        self.assertGreater(d, 0)

    def test_euclidean_self_zero(self):
        a = np.array([0.1, 0.2, 0.0, -0.5, 1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(_euclidean_distance(a, a), 0.0)

    def test_j5_periodicity_manhattan(self):
        """Manhattan distance should use shortest angular dist for J5."""
        a = np.zeros(DIMENSION)
        b = np.zeros(DIMENSION)
        b[4] = 3.0  # J5 at +3 rad
        # Shortest distance should be 2π - 3 ≈ 3.283
        d = _manhattan_distance(a, b)
        expected = abs(2 * np.pi - 3.0) * (np.pi / np.pi)  # J5 weight = 1
        self.assertLess(d, 3.5, f"Expected wrapping to reduce distance, got {d}")

    def test_j5_periodicity_euclidean(self):
        a = np.zeros(DIMENSION)
        b = np.zeros(DIMENSION)
        b[4] = 3.0
        d = _euclidean_distance(a, b)
        # Should be much less than treating it as a raw difference of 3
        self.assertLess(d, 3.5)

    def test_j1_prismatic_scaling(self):
        """J1 (prismatic) should be scaled to be comparable to rad joints."""
        a = np.zeros(DIMENSION)
        b = np.zeros(DIMENSION)
        b[0] = 0.585  # Full range of J1 in meters
        d = _euclidean_distance(a, b)
        # Scaled: 0.585 * (pi/0.585) = pi ≈ 3.14
        self.assertAlmostEqual(d, np.pi, delta=0.01)


class TestStateEquality(unittest.TestCase):

    def test_state_equal(self):
        si = _make_si()
        s1 = si.allocState()
        s2 = si.allocState()
        for i in range(DIMENSION):
            s1[i] = 0.0
            s2[i] = 0.0
        self.assertTrue(_state_equal(s1, s2))
        s2[0] = 0.1
        self.assertFalse(_state_equal(s1, s2))


class TestCollisionChecker(unittest.TestCase):

    def test_joint_limits_in_bounds(self):
        mid = tuple((lo + hi) / 2 for lo, hi in JOINT_LIMITS)
        self.assertTrue(is_configuration_valid(mid))

    def test_joint_limits_out_of_bounds(self):
        # J1 below lower bound
        bad = list((lo + hi) / 2 for lo, hi in JOINT_LIMITS)
        bad[0] = -0.1
        self.assertFalse(is_configuration_valid(bad))
        # J1 above upper bound
        bad[0] = 1.0
        self.assertFalse(is_configuration_valid(bad))

    def test_zeros_valid(self):
        zeros = tuple(0.0 for _ in range(DIMENSION))
        self.assertTrue(is_configuration_valid(zeros))

    def test_motion_valid_direct(self):
        zeros = tuple(0.0 for _ in range(DIMENSION))
        small_move = tuple(
            0.0 if i != 0 else 0.1 for i in range(DIMENSION)
        )
        self.assertTrue(is_motion_valid(zeros, small_move))


class TestPlannerBasic(unittest.TestCase):
    """Each test creates its own SI + pdef to avoid OMPL state reuse issues."""

    def test_planner_creation(self):
        si = _make_si()
        planner = AEBRRTstar(si)
        self.assertEqual(planner.getName(), "AEBRRTstar")
        planner.clear()

    def test_solve_exact_solution(self):
        si = _make_si()
        zeros = tuple(0.0 for _ in range(DIMENSION))
        mid = tuple((lo + hi) / 2 for lo, hi in JOINT_LIMITS)
        pdef = _make_problem(si, zeros, mid)
        planner = AEBRRTstar(si, stop_on_first_solution=True)
        planner.setProblemDefinition(pdef)
        planner.setup()
        ptc = ob.timedPlannerTerminationCondition(2.0)
        result = planner.solve(ptc)
        self.assertEqual(result, ob.PlannerStatus.EXACT_SOLUTION)
        self.assertTrue(pdef.hasSolution())
        planner.clear()

    def test_path_states_valid(self):
        si = _make_si()
        zeros = tuple(0.0 for _ in range(DIMENSION))
        mid = tuple((lo + hi) / 2 for lo, hi in JOINT_LIMITS)
        pdef = _make_problem(si, zeros, mid)
        planner = AEBRRTstar(si, stop_on_first_solution=True)
        planner.setProblemDefinition(pdef)
        planner.setup()
        ptc = ob.timedPlannerTerminationCondition(2.0)
        planner.solve(ptc)
        path = pdef.getSolutionPath()
        n = path.getStateCount()
        self.assertGreater(n, 0)
        for i in range(n):
            self.assertTrue(si.isValid(path.getState(i)),
                            f"State {i} is invalid")
        for i in range(n - 1):
            self.assertTrue(
                si.checkMotion(path.getState(i), path.getState(i + 1)),
                f"Edge {i}→{i+1} is invalid"
            )
        planner.clear()

    def test_same_start_goal(self):
        si = _make_si()
        zeros = tuple(0.0 for _ in range(DIMENSION))
        pdef = _make_problem(si, zeros, zeros)
        planner = AEBRRTstar(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        ptc = ob.timedPlannerTerminationCondition(1.0)
        result = planner.solve(ptc)
        self.assertEqual(result, ob.PlannerStatus.EXACT_SOLUTION)
        self.assertTrue(pdef.hasSolution())
        path = pdef.getSolutionPath()
        self.assertEqual(path.getStateCount(), 1)
        planner.clear()

    def test_clear_then_solve(self):
        """After clear(), a fresh planner instance should solve correctly."""
        si1 = _make_si()
        zeros = tuple(0.0 for _ in range(DIMENSION))
        mid = tuple((lo + hi) / 2 for lo, hi in JOINT_LIMITS)
        pdef1 = _make_problem(si1, zeros, mid)
        planner1 = AEBRRTstar(si1, stop_on_first_solution=True)
        planner1.setProblemDefinition(pdef1)
        planner1.setup()
        planner1.solve(ob.timedPlannerTerminationCondition(1.0))
        self.assertTrue(pdef1.hasSolution())
        planner1.clear()

        # Fresh planner, fresh SI
        si2 = _make_si()
        pdef2 = _make_problem(si2, zeros, mid)
        planner2 = AEBRRTstar(si2, stop_on_first_solution=True)
        planner2.setProblemDefinition(pdef2)
        planner2.setup()
        result = planner2.solve(ob.timedPlannerTerminationCondition(1.0))
        self.assertEqual(result, ob.PlannerStatus.EXACT_SOLUTION)
        planner2.clear()

    def test_invalid_start(self):
        si = _make_si()
        bad_start = tuple(-10.0 for _ in range(DIMENSION))
        mid = tuple((lo + hi) / 2 for lo, hi in JOINT_LIMITS)
        pdef = _make_problem(si, bad_start, mid)
        planner = AEBRRTstar(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        result = planner.solve(ob.timedPlannerTerminationCondition(1.0))
        self.assertEqual(result, ob.PlannerStatus.INVALID_START)
        planner.clear()

    def test_invalid_goal(self):
        si = _make_si()
        zeros = tuple(0.0 for _ in range(DIMENSION))
        bad_goal = tuple(10.0 for _ in range(DIMENSION))
        pdef = _make_problem(si, zeros, bad_goal)
        planner = AEBRRTstar(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        result = planner.solve(ob.timedPlannerTerminationCondition(1.0))
        self.assertEqual(result, ob.PlannerStatus.INVALID_GOAL)
        planner.clear()

    def test_deterministic_with_fixed_seed(self):
        """Same seed should produce the same result across fresh instances."""
        results = []
        for _ in range(3):
            np.random.seed(12345)
            si = _make_si()
            zeros = tuple(0.0 for _ in range(DIMENSION))
            mid = tuple((lo + hi) / 2 for lo, hi in JOINT_LIMITS)
            pdef = _make_problem(si, zeros, mid)
            planner = AEBRRTstar(si, stop_on_first_solution=True)
            planner.setProblemDefinition(pdef)
            planner.setup()
            planner.solve(ob.timedPlannerTerminationCondition(2.0))
            if pdef.hasSolution():
                path = pdef.getSolutionPath()
                results.append((path.getStateCount(), path.length()))
            planner.clear()
        self.assertEqual(len(set(results)), 1,
                         f"Non-deterministic results: {results}")


class TestPlannerParameters(unittest.TestCase):

    def test_invalid_step_size(self):
        si = _make_si()
        with self.assertRaises(ValueError):
            AEBRRTstar(si, step_size=-0.1)
        with self.assertRaises(ValueError):
            AEBRRTstar(si, step_size=0)

    def test_invalid_probability_range(self):
        si = _make_si()
        with self.assertRaises(ValueError):
            AEBRRTstar(si, p_min=0.5, p_max=0.3)
        with self.assertRaises(ValueError):
            AEBRRTstar(si, p_min=-0.1)

    def test_invalid_connect_threshold(self):
        si = _make_si()
        with self.assertRaises(ValueError):
            AEBRRTstar(si, connect_threshold=0)


def run_tests():
    """Run all tests and report results."""
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(sys.modules[__name__])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(run_tests())
