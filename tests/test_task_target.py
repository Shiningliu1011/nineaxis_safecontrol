"""Tests for shared task_target module (Issue #9).

Covers: first-target position, surface-normal orientation computation,
orientation quaternion normalisation.
"""

import math
import sys
import unittest
from pathlib import Path

# Ensure the source tree is importable.
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))


class TestSurfaceNormalOrientations(unittest.TestCase):
    """Test the compute_surface_normal_orientations function."""

    def setUp(self):
        from robot_safecontrol_moveit.cylinder_geometry import (
            compute_surface_normal_orientations,
        )
        self.compute = compute_surface_normal_orientations

    def test_basic_cylinder_orientations(self):
        """Points on a circle around Y-axis should produce valid orientations."""
        import numpy as np
        # Points on a circle of radius 1 in XZ plane, centred on Y axis
        points = []
        for angle in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            points.append((float(math.cos(angle)), 0.0, float(math.sin(angle))))
        orientations = self.compute(
            points, (0.0, 1.0, 0.0), fit_points=points
        )
        self.assertEqual(len(orientations), len(points))
        for q in orientations:
            # Each quaternion must be normalised.
            norm = math.sqrt(sum(v * v for v in q))
            self.assertAlmostEqual(norm, 1.0, places=5)

    def test_axis_direction_nonzero_required(self):
        """Zero axis direction must raise."""
        points = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)]
        with self.assertRaises(ValueError):
            self.compute(points, (0.0, 0.0, 0.0))

    def test_fit_points_minimum_count(self):
        """At least 3 fit points required."""
        points = [(1.0, 0.0, 0.0)]
        with self.assertRaises(ValueError):
            self.compute(points, (0.0, 1.0, 0.0))

    def test_orientation_xyzw_format(self):
        """Each orientation is xyzw (x, y, z, w)."""
        points = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)]
        orientations = self.compute(
            points, (0.0, 1.0, 0.0), fit_points=points
        )
        for q in orientations:
            self.assertEqual(len(q), 4)


class TestFirstTargetOrientation(unittest.TestCase):
    """Test compute_first_task_orientation switching behaviour."""

    def setUp(self):
        from robot_safecontrol_moveit.task_target import (
            compute_first_task_orientation,
        )
        self.compute = compute_first_task_orientation

    def test_surface_normal_disabled_uses_xyzw(self):
        """When align=False, orientation_xyzw is used directly."""
        positions = [(1.0, 0.0, 0.0)]
        first, per_point = self.compute(
            positions,
            align_tool_x_to_surface_normal=False,
            cylinder_axis_direction=(0.0, 1.0, 0.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        self.assertEqual(first, (0.0, 0.0, 0.0, 1.0))
        self.assertIsNone(per_point)

    def test_surface_normal_enabled_computes_orientation(self):
        """When align=True, orientation is computed from surface normal."""
        import numpy as np
        points = []
        for angle in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            points.append((float(math.cos(angle)), 0.0, float(math.sin(angle))))
        first, per_point = self.compute(
            points,
            align_tool_x_to_surface_normal=True,
            cylinder_axis_direction=(0.0, 1.0, 0.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        self.assertNotEqual(first, (0.0, 0.0, 0.0, 1.0))
        self.assertIsNotNone(per_point)
        self.assertEqual(len(per_point), len(points))


class TestRotationMatrixToQuaternion(unittest.TestCase):
    """Internal conversion from rotation matrix to quaternion."""

    def setUp(self):
        from robot_safecontrol_moveit.cylinder_geometry import (
            _rotation_matrix_to_quaternion_xyzw,
        )
        self.convert = _rotation_matrix_to_quaternion_xyzw

    def test_identity_matrix(self):
        import numpy as np
        q = self.convert(np.eye(3))
        self.assertAlmostEqual(q[0], 0.0, places=5)
        self.assertAlmostEqual(q[1], 0.0, places=5)
        self.assertAlmostEqual(q[2], 0.0, places=5)
        self.assertAlmostEqual(q[3], 1.0, places=5)

    def test_output_is_normalised(self):
        import numpy as np
        # 90-degree rotation around Z
        rot = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        q = self.convert(rot)
        norm = math.sqrt(sum(v * v for v in q))
        self.assertAlmostEqual(norm, 1.0, places=5)


class TestSharedLogicIntegration(unittest.TestCase):
    """Verify the transition pipeline's shared helpers stay importable."""

    def test_same_load_mat_trajectory(self):
        """The pipeline helpers live in task_target."""
        from robot_safecontrol_moveit.task_target import (
            load_mat_trajectory,
            compute_first_task_orientation,
            solve_first_task_state,
        )
        # Verify they are callable.
        self.assertTrue(callable(load_mat_trajectory))
        self.assertTrue(callable(compute_first_task_orientation))
        self.assertTrue(callable(solve_first_task_state))


if __name__ == "__main__":
    unittest.main()
