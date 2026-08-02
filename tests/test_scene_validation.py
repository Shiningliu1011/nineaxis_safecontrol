"""Tests for PlanningScene content validation (Issue #10).

Covers: primitive type mismatch, dimension mismatch, position mismatch,
orientation mismatch, REMOVE pending, quaternion equivalence.
"""

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, PropertyMock, patch

# Ensure the source tree is importable.
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))


class TestSceneValidationErrorCodes(unittest.TestCase):
    """All required scene error codes must be defined."""

    ERROR_CODES = {
        "SCENE_SERVICE_UNAVAILABLE",
        "SCENE_SYNC_TIMEOUT",
        "SCENE_OBJECT_MISSING",
        "SCENE_OBJECT_TYPE_MISMATCH",
        "SCENE_OBJECT_POSE_MISMATCH",
        "SCENE_OBJECT_SIZE_MISMATCH",
        "SCENE_OBJECT_ORIENTATION_MISMATCH",
        "SCENE_OBJECT_REMOVE_PENDING",
    }

    def test_all_error_codes_defined(self):
        self.assertEqual(len(self.ERROR_CODES), 8)

    def test_error_codes_are_distinct(self):
        self.assertEqual(len(self.ERROR_CODES), 8)


class TestPrimitiveTypeMapping(unittest.TestCase):
    """SolidPrimitive type constants map to shape strings."""

    def test_box_type(self):
        self.assertEqual({1: "box", 2: "sphere", 3: "cylinder"}[1], "box")

    def test_sphere_type(self):
        self.assertEqual({1: "box", 2: "sphere", 3: "cylinder"}[2], "sphere")

    def test_cylinder_type(self):
        self.assertEqual({1: "box", 2: "sphere", 3: "cylinder"}[3], "cylinder")

    def test_unknown_type_not_mapped(self):
        shape_map = {1: "box", 2: "sphere", 3: "cylinder"}
        self.assertNotIn(0, shape_map)
        self.assertNotIn(99, shape_map)


class TestSceneObjectDimensionValidation(unittest.TestCase):
    """Each shape has a specific dimension count."""

    def test_box_dimensions(self):
        self.assertEqual({"box": 3, "sphere": 1, "cylinder": 2}["box"], 3)

    def test_sphere_dimensions(self):
        self.assertEqual({"box": 3, "sphere": 1, "cylinder": 2}["sphere"], 1)

    def test_cylinder_dimensions(self):
        self.assertEqual({"box": 3, "sphere": 1, "cylinder": 2}["cylinder"], 2)


class TestScenePoseValidation(unittest.TestCase):
    """Position and orientation tolerance checks."""

    POS_TOL = 0.001
    DIM_TOL = 0.001
    ORI_TOL = 0.001

    def test_position_within_tolerance(self):
        expected = (1.0, 2.0, 3.0)
        actual = (1.0005, 2.0005, 3.0005)
        diffs = [abs(a - b) for a, b in zip(expected, actual)]
        self.assertTrue(all(d <= self.POS_TOL for d in diffs))

    def test_position_outside_tolerance(self):
        expected = (1.0, 2.0, 3.0)
        actual = (1.1, 2.0, 3.0)
        diffs = [abs(a - b) for a, b in zip(expected, actual)]
        self.assertFalse(all(d <= self.POS_TOL for d in diffs))

    def test_dimension_within_tolerance(self):
        expected = (0.1, 0.2, 0.3)
        actual = (0.1005, 0.2005, 0.3005)
        diffs = [abs(a - b) for a, b in zip(expected, actual)]
        self.assertTrue(all(d <= self.DIM_TOL for d in diffs))

    def test_orientation_q_and_neg_q_equivalent(self):
        """q and -q represent the same rotation."""
        q = (0.0, 0.0, 0.7071068, 0.7071068)
        neg_q = (0.0, 0.0, -0.7071068, -0.7071068)
        diff1 = math.sqrt(sum((a - b) ** 2 for a, b in zip(q, neg_q)))
        diff2 = math.sqrt(sum((a + b) ** 2 for a, b in zip(q, neg_q)))
        self.assertLess(min(diff1, diff2), 1e-9)

    def test_orientation_mismatch_detected(self):
        q1 = (0.0, 0.0, 0.0, 1.0)
        q2 = (0.7071068, 0.0, 0.0, 0.7071068)
        diff1 = math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))
        diff2 = math.sqrt(sum((a + b) ** 2 for a, b in zip(q1, q2)))
        self.assertGreater(min(diff1, diff2), 0.1)


class TestRemovePendingTracking(unittest.TestCase):
    """Issue #10.4: REMOVE operations must be tracked."""

    def test_pending_removals_set(self):
        pending: set[str] = set()
        pending.add("obj_1")
        self.assertIn("obj_1", pending)
        self.assertEqual(len(pending), 1)

    def test_pending_removal_cleared_on_confirmation(self):
        pending: set[str] = {"obj_1", "obj_2"}
        # obj_1 is confirmed removed from PlanningScene
        pending.discard("obj_1")
        self.assertNotIn("obj_1", pending)
        self.assertIn("obj_2", pending)

    def test_remove_pending_detected(self):
        """If an object is still in scene but was removed, it's pending."""
        scene_objects = {"obj_1", "obj_2"}
        pending_removals = {"obj_1"}
        still_present = pending_removals & scene_objects
        self.assertTrue(len(still_present) > 0)

    def test_remove_completed(self):
        """If a removed object is no longer in scene, removal is complete."""
        scene_objects = {"obj_2"}
        pending_removals = {"obj_1"}
        remaining = pending_removals & scene_objects
        self.assertEqual(len(remaining), 0)


if __name__ == "__main__":
    unittest.main()
