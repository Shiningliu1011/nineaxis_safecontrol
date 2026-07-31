"""AEB-RRT* : Adaptive Extension Bidirectional RRT*

Implements the algorithm from the AEB-RRT* paper as an ``ompl.base.Planner``
subclass.  Two running modes are supported:

``stop_on_first_solution=True`` (Faithful)
    Returns as soon as the two trees connect.  Matches the paper's primary
    experimental setup.

``stop_on_first_solution=False`` (Anytime)
    After the first connection the planner keeps improving the solution until
    the PlannerTerminationCondition fires, maintaining the best (lowest-cost)
    complete path seen so far.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import exp, log, sqrt
from typing import Sequence

import numpy as np

import ompl.base as ob
import ompl.geometric as og

from .robot_model import DIMENSION, JOINT_LIMITS, WRAPPING_JOINT_INDICES

# ---------------------------------------------------------------------------
#  Default parameters (paper values, scaled for the ninezzhou joint-space)
# ---------------------------------------------------------------------------

_DEFAULT_STEP_SIZE = 0.15          # ~10 % of typical joint range
_DEFAULT_CONNECT_THRESHOLD = 0.3   # 2 × step_size
_DEFAULT_P_MIN = 0.1
_DEFAULT_P_MAX = 1.0
_DEFAULT_MAX_FAILED_EXTENSIONS = 500
_DEFAULT_REWIRE_FACTOR = 1.1       # RRT* connection radius multiplier
_DEFAULT_INTERP_COUNT = 30         # AEB shortcut interpolation count


# ======================================================================
#  Tree node
# ======================================================================


@dataclass
class _Node:
    state: ob.State
    parent: int                    # index into tree node list, -1 for root
    children: list[int] = field(default_factory=list)
    cost: float = 0.0              # accumulated cost from tree root


# ======================================================================
#  Distance helpers
# ======================================================================


def _joint_vector(state: ob.State) -> np.ndarray:
    """Extract a (DIMENSION,) numpy array from an OMPL RealVector state."""
    return np.array([float(state[i]) for i in range(DIMENSION)], dtype=float)


def _manhattan_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Weighted L1 distance with shortest-angle handling for wrapping joints.

    J1 (prismatic) is scaled to rad-equivalent so that 1 m ≈ π rad for
    balanced weighting across the 9-D space.
    """
    diff = np.abs(a - b)
    # Wrap J5 (index 4) to shortest angle
    for idx in WRAPPING_JOINT_INDICES:
        d = diff[idx]
        if d > np.pi:
            diff[idx] = 2.0 * np.pi - d
    # Scale J1 (prismatic, index 0): 0.585 m range ≈ π rad
    diff[0] = diff[0] * (np.pi / 0.585)
    return float(np.sum(diff))


def _euclidean_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Euclidean distance with shortest-angle handling."""
    diff = a - b
    for idx in WRAPPING_JOINT_INDICES:
        d = diff[idx]
        # Wrap to [-π, π]
        diff[idx] = (d + np.pi) % (2.0 * np.pi) - np.pi
    # Scale J1
    diff[0] = diff[0] * (np.pi / 0.585)
    return float(np.linalg.norm(diff))


def _state_equal(a: ob.State, b: ob.State) -> bool:
    return all(float(a[i]) == float(b[i]) for i in range(DIMENSION))


# ======================================================================
#  AEB-RRT* Planner
# ======================================================================


class AEBRRTstar(ob.Planner):
    """Adaptive Extension Bidirectional RRT* motion planner."""

    def __init__(
        self,
        si: ob.SpaceInformation,
        *,
        step_size: float = _DEFAULT_STEP_SIZE,
        connect_threshold: float = _DEFAULT_CONNECT_THRESHOLD,
        p_min: float = _DEFAULT_P_MIN,
        p_max: float = _DEFAULT_P_MAX,
        max_failed_extensions: int = _DEFAULT_MAX_FAILED_EXTENSIONS,
        rewire_factor: float = _DEFAULT_REWIRE_FACTOR,
        stop_on_first_solution: bool = True,
        interpolate_count: int = _DEFAULT_INTERP_COUNT,
        enable_aeb_shortcut: bool = True,
        biased_range_multiplier: float = 2.0,
    ) -> None:
        super().__init__(si, "AEBRRTstar")

        self._step_size = float(step_size)
        self._connect_threshold = float(connect_threshold)
        self._p_min = float(p_min)
        self._p_max = float(p_max)
        self._max_failed_extensions = int(max_failed_extensions)
        self._rewire_factor = float(rewire_factor)
        self._stop_on_first_solution = bool(stop_on_first_solution)
        self._interp_count = int(interpolate_count)
        self._enable_aeb_shortcut = bool(enable_aeb_shortcut)
        self._biased_range_multiplier = float(biased_range_multiplier)

        # Per-tree nearest-neighbour point arrays (rebuilt lazily)
        self._tree_start: list[_Node] = []
        self._tree_goal: list[_Node] = []

        # Best solution (anytime mode)
        self._best_cost: float = float("inf")
        self._best_path_states: list[ob.State] = []

        # Metrics
        self._validity_checks: int = 0
        self._motion_checks: int = 0

        # Stats exposed after solve()
        self.last_solve_nodes_start: int = 0
        self.last_solve_nodes_goal: int = 0
        self.last_solve_checks_validity: int = 0
        self.last_solve_checks_motion: int = 0

        self._validate_params()

    # ------------------------------------------------------------------
    #  Parameter validation
    # ------------------------------------------------------------------

    def _validate_params(self) -> None:
        if self._step_size <= 0:
            raise ValueError("step_size must be positive")
        if self._connect_threshold <= 0:
            raise ValueError("connect_threshold must be positive")
        if not 0.0 <= self._p_min < self._p_max <= 1.0:
            raise ValueError("Require 0 <= p_min < p_max <= 1")
        if self._max_failed_extensions < 1:
            raise ValueError("max_failed_extensions must be >= 1")
        if self._rewire_factor <= 0:
            raise ValueError("rewire_factor must be positive")
        if self._interp_count < 0:
            raise ValueError("interp_count must be non-negative")
        if self._biased_range_multiplier <= 1.0:
            raise ValueError("biased_range_multiplier must be > 1")

    # ------------------------------------------------------------------
    #  OMPL Planner interface
    # ------------------------------------------------------------------

    def clear(self) -> None:
        super().clear()
        self._free_trees()
        self._best_cost = float("inf")
        self._best_path_states.clear()
        self._validity_checks = 0
        self._motion_checks = 0

    def setup(self) -> None:
        super().setup()
        si = self.getSpaceInformation()
        if si is None:
            raise RuntimeError("SpaceInformation not set")
        # Ensure the state space dimension matches
        space = si.getStateSpace()
        if space.getDimension() != DIMENSION:
            raise RuntimeError(
                f"AEBRRTstar expects a {DIMENSION}-D state space, "
                f"got {space.getDimension()}"
            )

    def solve(self, ptc: ob.PlannerTerminationCondition) -> ob.PlannerStatus:
        """Run the AEB-RRT* algorithm.

        NOTE: This method deliberately avoids ``freeState()`` calls because
        the OMPL Python bindings (nanobind) can double-free states, causing
        a segfault.  We accept the memory leak in the bindings layer.
        """
        self.clear()

        pdef = self.getProblemDefinition()
        if pdef is None:
            return ob.PlannerStatus(ob.PlannerStatus.CRASH)

        si = self.getSpaceInformation()
        if si is None:
            return ob.PlannerStatus(ob.PlannerStatus.CRASH)

        # Validate start/goal using ProblemDefinition references directly
        start_raw = pdef.getStartState(0)
        goal_target = pdef.getGoal()
        if start_raw is None or goal_target is None:
            return ob.PlannerStatus(ob.PlannerStatus.INVALID_START)
        if not hasattr(goal_target, 'getState'):
            return ob.PlannerStatus(ob.PlannerStatus.UNRECOGNIZED_GOAL_TYPE)
        goal_raw = goal_target.getState()
        if goal_raw is None:
            return ob.PlannerStatus(ob.PlannerStatus.INVALID_GOAL)

        if not si.isValid(start_raw):
            return ob.PlannerStatus(ob.PlannerStatus.INVALID_START)
        if not si.isValid(goal_raw):
            return ob.PlannerStatus(ob.PlannerStatus.INVALID_GOAL)

        # Same start/goal → zero-cost path
        if _state_equal(start_raw, goal_raw):
            path = og.PathGeometric(si)
            path.append(si.cloneState(start_raw))
            self._add_solution(path, pdef)
            self.last_solve_checks_validity = self._validity_checks
            self.last_solve_checks_motion = self._motion_checks
            return ob.PlannerStatus(ob.PlannerStatus.EXACT_SOLUTION)

        # Build tree roots — clone because OMPL owns the ProblemDefinition states
        root_start = si.cloneState(start_raw)
        root_goal = si.cloneState(goal_raw)

        # Init trees
        self._tree_start = [_Node(state=root_start, parent=-1, cost=0.0)]
        self._tree_goal = [_Node(state=root_goal, parent=-1, cost=0.0)]

        fail_start = 0
        fail_goal = 0
        opt_obj = pdef.getOptimizationObjective()

        iteration = 0
        solved = False

        try:
            while not ptc():
                iteration += 1

                # --- Grow start tree ---
                fail_start, connected = self._extend_tree(
                    self._tree_start,
                    self._tree_goal,
                    fail_start,
                    si,
                    opt_obj,
                    iteration,
                )
                if connected:
                    path = self._build_path(si)
                    if path is not None:
                        cost = self._compute_path_cost(path, opt_obj)
                        if self._stop_on_first_solution:
                            self._add_solution(path, pdef)
                            solved = True
                            break
                        elif cost < self._best_cost:
                            self._best_cost = cost
                            self._replace_best(path, si, pdef)
                            solved = True

                if ptc():
                    break

                # --- Grow goal tree ---
                fail_goal, connected = self._extend_tree(
                    self._tree_goal,
                    self._tree_start,
                    fail_goal,
                    si,
                    opt_obj,
                    iteration,
                )
                if connected:
                    path = self._build_path(si)
                    if path is not None:
                        cost = self._compute_path_cost(path, opt_obj)
                        if self._stop_on_first_solution:
                            self._add_solution(path, pdef)
                            solved = True
                            break
                        elif cost < self._best_cost:
                            self._best_cost = cost
                            self._replace_best(path, si, pdef)
                            solved = True

                # Guard: max failed extensions
                max_fail = max(fail_start, fail_goal)
                if max_fail > self._max_failed_extensions and not solved:
                    break

        finally:
            self.last_solve_nodes_start = len(self._tree_start)
            self.last_solve_nodes_goal = len(self._tree_goal)
            self.last_solve_checks_validity = self._validity_checks
            self.last_solve_checks_motion = self._motion_checks

        # Anytime mode: submit the best solution found
        if not self._stop_on_first_solution and solved:
            # Best solution was already registered via _replace_best
            return ob.PlannerStatus(ob.PlannerStatus.EXACT_SOLUTION)

        if solved:
            return ob.PlannerStatus(ob.PlannerStatus.EXACT_SOLUTION)

        # No solution found
        return ob.PlannerStatus(ob.PlannerStatus.TIMEOUT)

    def getPlannerData(self, data: ob.PlannerData) -> None:
        """Export tree edges to OMPL PlannerData for inspection."""
        for tree_nodes, tree_tag in (
            (self._tree_start, 1),
            (self._tree_goal, 2),
        ):
            for idx, node in enumerate(tree_nodes):
                if node.parent >= 0:
                    data.addEdge(
                        ob.PlannerDataVertex(node.state, tree_tag),
                        ob.PlannerDataVertex(tree_nodes[node.parent].state, tree_tag),
                    )

    # ------------------------------------------------------------------
    #  Tree extension (single iteration of one tree)
    # ------------------------------------------------------------------

    def _extend_tree(
        self,
        tree: list[_Node],
        other_tree: list[_Node],
        fail_count: int,
        si: ob.SpaceInformation,
        opt_obj: ob.OptimizationObjective | None,
        iteration: int,
    ) -> tuple[int, bool]:
        """Perform one AEB-RRT* extension step on *tree* toward *other_tree*.

        Returns (new_fail_count, connected).
        """
        # Adaptive probability
        p_a = self._p_min + (self._p_max - self._p_min) * exp(
            -9.0 / (fail_count + 1.0) ** 3
        )

        # Use a simple random number via numpy for the biased/random decision
        if np.random.random() > p_a:
            # Target-biased: sample toward other tree's root
            x_sample = _joint_vector(other_tree[0].state)
            step = self._step_size * self._biased_range_multiplier
        else:
            # Random sample in the state space bounds
            x_sample = self._random_sample(si)
            step = self._step_size

        # Nearest node (Manhattan distance per paper)
        idx_near = self._nearest_manhattan(tree, x_sample)
        x_near = _joint_vector(tree[idx_near].state)

        # Steer
        x_new = self._steer(x_near, x_sample, step)

        # ---- allocate OMPL state for x_new ----
        new_state = si.allocState()
        for i in range(DIMENSION):
            new_state[i] = x_new[i]

        # Collision check along the edge
        self._motion_checks += 1
        if not si.checkMotion(tree[idx_near].state, new_state):
            # NOTE: we do not freeState(new_state) because the Python
            # OMPL bindings can double-free.  Accept the leak.
            return (fail_count + 1, False)

        fail_count = 0  # reset on success

        # ---- RRT* : choose parent ----
        radius = self._neighbourhood_radius(len(tree) + 1)
        near_indices = self._near_euclidean(tree, x_new, radius)

        best_parent = idx_near
        best_cost = tree[idx_near].cost + _euclidean_distance(
            x_near, x_new
        )

        for ni in near_indices:
            if ni == idx_near:
                continue
            cand = _joint_vector(tree[ni].state)
            cand_cost = tree[ni].cost + _euclidean_distance(cand, x_new)
            if cand_cost < best_cost:
                self._motion_checks += 1
                if si.checkMotion(tree[ni].state, new_state):
                    best_parent = ni
                    best_cost = cand_cost

        # Insert node
        new_idx = len(tree)
        node = _Node(state=new_state, parent=best_parent, cost=best_cost)
        tree.append(node)
        tree[best_parent].children.append(new_idx)

        # ---- RRT* : rewire neighbours ----
        for ni in near_indices:
            if ni == best_parent or ni == new_idx:
                continue
            cand = _joint_vector(tree[ni].state)
            via_new_cost = best_cost + _euclidean_distance(x_new, cand)
            if via_new_cost + 1e-12 < tree[ni].cost:
                self._motion_checks += 1
                if si.checkMotion(new_state, tree[ni].state):
                    delta = via_new_cost - tree[ni].cost
                    # Unlink from old parent
                    old_parent = tree[ni].parent
                    if old_parent >= 0 and ni in tree[old_parent].children:
                        tree[old_parent].children.remove(ni)
                    # Relink
                    tree[ni].parent = new_idx
                    tree[ni].cost = via_new_cost
                    node.children.append(ni)
                    # Propagate cost delta to descendants
                    self._update_descendant_costs(tree, ni, delta)

        # ---- Connect trees ----
        idx_con = self._nearest_manhattan(other_tree, x_new)
        x_con = _joint_vector(other_tree[idx_con].state)

        if (
            _euclidean_distance(x_new, x_con) <= self._connect_threshold
        ):
            self._motion_checks += 1
            if si.checkMotion(new_state, other_tree[idx_con].state):
                # Store connection info for path building
                self._connect_start_idx = new_idx
                self._connect_goal_idx = idx_con
                self._connect_on_start_tree = True
                return (fail_count, True)

        return (fail_count, False)

    # ------------------------------------------------------------------
    #  Sampling, steering, nearest-neighbour
    # ------------------------------------------------------------------

    def _random_sample(self, si: ob.SpaceInformation) -> np.ndarray:
        """Generate a uniform random sample within joint limits."""
        sample = np.empty(DIMENSION)
        for i, (low, high) in enumerate(JOINT_LIMITS):
            sample[i] = np.random.uniform(low, high)
        return sample

    @staticmethod
    def _steer(x_from: np.ndarray, x_to: np.ndarray, step: float) -> np.ndarray:
        """Steer from *x_from* toward *x_to* by at most *step*."""
        direction = x_to - x_from
        dist = float(np.linalg.norm(direction))
        if dist <= step:
            return x_to.copy()
        return x_from + (step / dist) * direction

    def _nearest_manhattan(
        self, tree: list[_Node], target: np.ndarray
    ) -> int:
        """Return index of the tree node closest to *target* by Manhattan distance."""
        best_idx = 0
        best_dist = float("inf")
        for i, node in enumerate(tree):
            d = _manhattan_distance(_joint_vector(node.state), target)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _near_euclidean(
        self, tree: list[_Node], target: np.ndarray, radius: float
    ) -> list[int]:
        """Return indices of tree nodes within Euclidean distance *radius*."""
        result: list[int] = []
        for i, node in enumerate(tree):
            d = _euclidean_distance(_joint_vector(node.state), target)
            if d <= radius:
                result.append(i)
        return result

    def _neighbourhood_radius(self, n: int) -> float:
        """RRT* connection radius based on cardinality."""
        if n <= 2:
            return self._connect_threshold
        gamma = self._rewire_factor * (DIMENSION ** 0.25)
        r = gamma * (log(n) / n) ** (1.0 / DIMENSION)
        return max(self._step_size, min(r, self._connect_threshold * 3.0))

    # ------------------------------------------------------------------
    #  Tree maintenance
    # ------------------------------------------------------------------

    @staticmethod
    def _update_descendant_costs(
        tree: list[_Node], changed_idx: int, delta: float
    ) -> None:
        """Propagate a cost delta to all descendants of *changed_idx*."""
        queue = [changed_idx]
        while queue:
            current = queue.pop(0)
            for child_idx in tree[current].children:
                tree[child_idx].cost += delta
                queue.append(child_idx)

    # ------------------------------------------------------------------
    #  Path construction
    # ------------------------------------------------------------------

    def _build_path(self, si: ob.SpaceInformation) -> og.PathGeometric | None:
        """Build a PathGeometric from the two trees after connection."""
        try:
            from_start = self._trace_path(self._tree_start, self._connect_start_idx)
            from_goal = self._trace_path(self._tree_goal, self._connect_goal_idx)
        except (AttributeError, IndexError):
            return None

        path = og.PathGeometric(si)
        # start → ... → connection point
        for s in from_start:
            path.append(s)
        # connection point on goal tree → ... → goal (reversed)
        for s in reversed(from_goal[1:]):  # skip duplicate connection point
            path.append(s)

        return path

    @staticmethod
    def _trace_path(tree: list[_Node], idx: int) -> list[ob.State]:
        """Return states from root to node *idx* (inclusive)."""
        chain: list[ob.State] = []
        current = idx
        while current >= 0:
            chain.append(tree[current].state)
            current = tree[current].parent
        chain.reverse()
        return chain

    @staticmethod
    def _compute_path_cost(
        path: og.PathGeometric,
        opt_obj: ob.OptimizationObjective | None,
    ) -> float:
        """Compute the total path cost using the OptimizationObjective."""
        if opt_obj is not None:
            return float(opt_obj.pathCost(path))
        # Fallback: Euclidean sum
        total = 0.0
        n = path.getStateCount()
        for i in range(n - 1):
            a = path.getState(i)
            b = path.getState(i + 1)
            total += _euclidean_distance(_joint_vector(a), _joint_vector(b))
        return total

    # ------------------------------------------------------------------
    #  OMPL solution management
    # ------------------------------------------------------------------

    def _add_solution(
        self, path: og.PathGeometric, pdef: ob.ProblemDefinition
    ) -> None:
        """Register the path as a solution with the ProblemDefinition."""
        # AEB paper shortcut (if enabled and this is the raw path)
        if self._enable_aeb_shortcut and self._interp_count > 0:
            path = self._aeb_shortcut(path, pdef.getSpaceInformation())

        pdef.addSolutionPath(path)

    def _replace_best(
        self, path: og.PathGeometric, si: ob.SpaceInformation,
        pdef: ob.ProblemDefinition,
    ) -> None:
        """Replace any worse existing solutions with this better one."""
        # Apply AEB shortcut if enabled
        if self._enable_aeb_shortcut and self._interp_count > 0:
            path = self._aeb_shortcut(path, si)

        # OMPL uses approximate solution when an exact one already exists
        # and the new one is better.  For simplicity we use addSolutionPath
        # which handles replacement internally.
        pdef.addSolutionPath(path)

    # ------------------------------------------------------------------
    #  AEB paper post-processing: interpolate + triangle-inequality shortcut
    # ------------------------------------------------------------------

    def _aeb_shortcut(
        self, path: og.PathGeometric, si: ob.SpaceInformation
    ) -> og.PathGeometric:
        """Apply the paper's interpolation + farthest-visible-point shortcut."""
        # Step 1: interpolate
        interp_path = self._interpolate_path(path, si)
        # Step 2: triangle-inequality shortcut
        short_path = self._shortcut_path(interp_path, si)
        return short_path

    def _interpolate_path(
        self, path: og.PathGeometric, si: ob.SpaceInformation
    ) -> og.PathGeometric:
        """Insert *interp_count* intermediate states between each pair."""
        n = path.getStateCount()
        if n < 2 or self._interp_count <= 0:
            return path

        result = og.PathGeometric(si)
        for i in range(n - 1):
            s_from = path.getState(i)
            s_to = path.getState(i + 1)
            result.append(si.cloneState(s_from))
            for k in range(1, self._interp_count + 1):
                alpha = k / (self._interp_count + 1)
                interp = si.allocState()
                for d in range(DIMENSION):
                    interp[d] = float(s_from[d]) + alpha * (float(s_to[d]) - float(s_from[d]))
                result.append(interp)
        # Last state
        result.append(si.cloneState(path.getState(n - 1)))
        return result

    def _shortcut_path(
        self, path: og.PathGeometric, si: ob.SpaceInformation
    ) -> og.PathGeometric:
        """Greedy farthest-visible-point shortcut."""
        n = path.getStateCount()
        if n < 3:
            return path

        result = og.PathGeometric(si)
        result.append(si.cloneState(path.getState(0)))

        i = 0
        while i < n - 1:
            best_j = i + 1
            for j in range(n - 1, i, -1):
                self._motion_checks += 1
                if si.checkMotion(path.getState(i), path.getState(j)):
                    best_j = j
                    break
            result.append(si.cloneState(path.getState(best_j)))
            i = best_j

        return result

    # ------------------------------------------------------------------
    #  Cleanup
    # ------------------------------------------------------------------

    def _free_trees(self) -> None:
        """Release tree node lists (states are NOT freed due to OMPL bindings)."""
        self._tree_start.clear()
        self._tree_goal.clear()
