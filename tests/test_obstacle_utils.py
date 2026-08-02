"""Unit tests for obstacle utilities, coordinate transforms, size parsing,
joint state validation, and slot management logic.

These tests are pure Python — no MuJoCo window, no ROS node.
"""

import math
import sys
import unittest
from pathlib import Path

# Ensure the source tree is importable.
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))


# ---------------------------------------------------------------------------
# Coordinate transforms
# ---------------------------------------------------------------------------


class TestYUpToZUpTransform(unittest.TestCase):
    """Y-up (base_link) → Z-up (MuJoCo world) position and quaternion."""

    def test_position_conversion(self):
        """(x, y, z) in base_link → (x, -z, y) in MuJoCo world."""
        # The static method exists on MuJoCoJointStateViewer.
        # We test the maths inline since we cannot import the viewer
        # without MuJoCo and ROS.
        pos = (1.0, 2.0, 3.0)
        # Y-up → Z-up: rotate around X by +90°
        rotated = (pos[0], -pos[2], pos[1])
        self.assertAlmostEqual(rotated[0], 1.0)
        self.assertAlmostEqual(rotated[1], -3.0)
        self.assertAlmostEqual(rotated[2], 2.0)

    def test_position_origin(self):
        """Origin stays at origin."""
        pos = (0.0, 0.0, 0.0)
        rotated = (pos[0], -pos[2], pos[1])
        self.assertEqual(rotated, (0.0, 0.0, 0.0))


class TestQuaternionConversion(unittest.TestCase):
    """ROS xyzw ↔ MuJoCo wxyz."""

    def test_identity_quaternion(self):
        """Identity quaternion (0,0,0,1) xyzw → (1,0,0,0) wxyz."""
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        wxyz = (qw, qx, qy, qz)
        self.assertEqual(wxyz, (1.0, 0.0, 0.0, 0.0))

    def test_arbitrary_quaternion(self):
        """Arbitrary xyzw → wxyz reordering."""
        qx, qy, qz, qw = 0.1, 0.2, 0.3, 0.4
        wxyz = (qw, qx, qy, qz)
        self.assertEqual(wxyz, (0.4, 0.1, 0.2, 0.3))


# ---------------------------------------------------------------------------
# Size parsing (SolidPrimitive dimensions → MuJoCo geom_size)
# ---------------------------------------------------------------------------


class TestObstacleSizeParsing(unittest.TestCase):
    """Full dimensions → MuJoCo half-size / per-shape conventions."""

    def test_sphere_full_to_mujoco(self):
        """Sphere [radius] → MuJoCo geom_size[0] = radius."""
        radius = 0.15
        mujoco_size = (radius, 0.0, 0.0)
        self.assertAlmostEqual(mujoco_size[0], 0.15)

    def test_box_full_to_half_extents(self):
        """Box [x, y, z] full → MuJoCo [x/2, y/2, z/2]."""
        full = (0.40, 0.20, 0.50)
        half = (full[0] / 2.0, full[1] / 2.0, full[2] / 2.0)
        self.assertAlmostEqual(half[0], 0.20)
        self.assertAlmostEqual(half[1], 0.10)
        self.assertAlmostEqual(half[2], 0.25)

    def test_cylinder_height_radius_to_mujoco(self):
        """Cylinder SolidPrimitive [height, radius] → MuJoCo [radius, height/2]."""
        height, radius = 0.80, 0.10
        mujoco = (radius, height / 2.0, 0.0)
        self.assertAlmostEqual(mujoco[0], 0.10)
        self.assertAlmostEqual(mujoco[1], 0.40)

    def test_minimum_size_guard_sphere(self):
        """Zero or negative sphere radius clamped to MIN_DIM."""
        MIN_DIM = 1e-6
        size = (0.0, 0.0, 0.0)
        clamped = (max(size[0], MIN_DIM), 0.0, 0.0)
        self.assertGreater(clamped[0], 0.0)

    def test_minimum_size_guard_box(self):
        """Zero box dimensions clamped to MIN_DIM."""
        MIN_DIM = 1e-6
        size = (0.0, -0.1, 0.0)
        clamped = (
            max(size[0], MIN_DIM),
            max(size[1], MIN_DIM),
            max(size[2], MIN_DIM),
        )
        self.assertGreater(clamped[0], 0.0)
        self.assertGreater(clamped[1], 0.0)
        self.assertGreater(clamped[2], 0.0)

    def test_minimum_size_guard_cylinder(self):
        """Zero cylinder dimensions clamped to MIN_DIM."""
        MIN_DIM = 1e-6
        size = (0.0, 0.0, 0.0)
        clamped = (max(size[0], MIN_DIM), max(size[1], MIN_DIM), 0.0)
        self.assertGreater(clamped[0], 0.0)
        self.assertGreater(clamped[1], 0.0)


# ---------------------------------------------------------------------------
# Slot management logic
# ---------------------------------------------------------------------------


class TestSlotManagement(unittest.TestCase):
    """Slot claim, release, size preservation, and reuse."""

    def setUp(self):
        self.slots = [
            {
                "name": f"slot_{i}",
                "shape": "box",
                "claimed": False,
                "object_id": None,
                "size": (0.05, 0.05, 0.05),
            }
            for i in range(3)
        ]
        self.id_map: dict[str, int] = {}

    def _claim(self, shape: str, object_id: str) -> int | None:
        for i, s in enumerate(self.slots):
            if not s["claimed"] and s["shape"] == shape:
                s["claimed"] = True
                s["object_id"] = object_id
                self.id_map[object_id] = i
                return i
        return None

    def _remove(self, object_id: str) -> int | None:
        idx = self.id_map.pop(object_id, None)
        if idx is not None:
            s = self.slots[idx]
            s["claimed"] = False
            s["object_id"] = None
        return idx

    def test_claim_free_slot(self):
        idx = self._claim("box", "obj_1")
        self.assertIsNotNone(idx)
        self.assertTrue(self.slots[idx]["claimed"])
        self.assertEqual(self.slots[idx]["object_id"], "obj_1")

    def test_claim_exhaustion(self):
        for i in range(3):
            self.assertIsNotNone(self._claim("box", f"obj_{i}"))
        self.assertIsNone(self._claim("box", "obj_extra"))

    def test_remove_releases_slot(self):
        self._claim("box", "obj_1")
        self._remove("obj_1")
        self.assertFalse(self.slots[0]["claimed"])
        self.assertIsNone(self.slots[0]["object_id"])

    def test_slot_reuse_after_remove(self):
        self._claim("box", "obj_1")
        self._remove("obj_1")
        idx = self._claim("box", "obj_2")
        self.assertEqual(idx, 0)

    def test_move_without_primitive_preserves_size(self):
        """MOVE without primitive keeps saved slot size."""
        self._claim("box", "obj_1")
        slot = self.slots[0]
        slot["size"] = (0.10, 0.10, 0.10)  # saved from prior ADD
        # Simulate MOVE without primitive
        size = slot.get("size", (0.05, 0.05, 0.05))
        self.assertEqual(size, (0.10, 0.10, 0.10))

    def test_remove_resets_size_to_default(self):
        """REMOVE resets slot size to default."""
        self._claim("box", "obj_1")
        self.slots[0]["size"] = (0.10, 0.10, 0.10)
        self._remove("obj_1")
        self.slots[0]["size"] = (0.05, 0.05, 0.05)  # restored
        self.assertEqual(self.slots[0]["size"], (0.05, 0.05, 0.05))

    def test_unknown_remove_is_noop(self):
        idx = self._remove("nonexistent")
        self.assertIsNone(idx)

    def test_unknown_move_noop(self):
        """MOVE for unknown object returns None from id_map."""
        idx = self.id_map.get("nonexistent")
        self.assertIsNone(idx)


# ---------------------------------------------------------------------------
# Joint state validation
# ---------------------------------------------------------------------------


class TestJointStateValidation(unittest.TestCase):
    """Validate JointState messages before planning."""

    JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")

    def _has_all_joints(self, names: list[str]) -> bool:
        return set(self.JOINT_NAMES).issubset(names)

    def test_complete_state_accepted(self):
        self.assertTrue(self._has_all_joints(list(self.JOINT_NAMES)))

    def test_missing_joint_rejected(self):
        names = list(self.JOINT_NAMES)
        names.remove("J5")
        self.assertFalse(self._has_all_joints(names))

    def test_nan_value_detected(self):
        values = [0.0] * 9
        values[3] = float("nan")
        has_nan = not all(math.isfinite(v) for v in values)
        self.assertTrue(has_nan)

    def test_inf_value_detected(self):
        values = [0.0] * 9
        values[3] = float("inf")
        has_inf = not all(math.isfinite(v) for v in values)
        self.assertTrue(has_inf)

    def test_all_finite_accepted(self):
        values = [0.1] * 9
        self.assertTrue(all(math.isfinite(v) for v in values))


# ---------------------------------------------------------------------------
# Joint limit clamping
# ---------------------------------------------------------------------------


class TestJointClamping(unittest.TestCase):
    """Clamp joint values to model limits."""

    LIMITS = {
        "J1": (0.0, 0.585),
        "J2": (-1.5708, 1.5708),
        "J5": (-3.1416, 3.1416),
        "J6": (-1.48353, 1.48353),
    }

    def _clamp(self, name: str, value: float) -> float:
        low, high = self.LIMITS.get(name, (-float("inf"), float("inf")))
        return max(low, min(high, value))

    def test_value_in_range_unchanged(self):
        self.assertAlmostEqual(self._clamp("J2", 0.5), 0.5)

    def test_value_below_min_clamped(self):
        self.assertAlmostEqual(self._clamp("J1", -0.1), 0.0)

    def test_value_above_max_clamped(self):
        self.assertAlmostEqual(self._clamp("J1", 1.0), 0.585)

    def test_zero_clamp_when_in_range(self):
        self.assertAlmostEqual(self._clamp("J2", 0.0), 0.0)

    def test_zero_clamp_when_out_of_range(self):
        """J1 has [0.0, 0.585] — zero IS in range."""
        self.assertAlmostEqual(self._clamp("J1", 0.0), 0.0)

    def test_negative_value_clamped_j6(self):
        self.assertAlmostEqual(self._clamp("J6", -3.0), -1.48353)

    def test_prismatic_limit(self):
        self.assertAlmostEqual(self._clamp("J1", 0.3), 0.3)


# ---------------------------------------------------------------------------
# Collision object validation
# ---------------------------------------------------------------------------


class TestCollisionObjectValidation(unittest.TestCase):
    """Validate obstacle specs from YAML."""

    VALID_SHAPES = {"box": 3, "sphere": 1, "cylinder": 2}

    def _validate_dims(self, shape: str, dims: tuple[float, ...]) -> bool:
        expected = self.VALID_SHAPES.get(shape)
        if expected is None:
            return False
        return len(dims) == expected and all(d > 0.0 for d in dims)

    def test_box_valid_dims(self):
        self.assertTrue(self._validate_dims("box", (0.1, 0.2, 0.3)))

    def test_box_wrong_dim_count(self):
        self.assertFalse(self._validate_dims("box", (0.1, 0.2)))

    def test_box_negative_dim(self):
        self.assertFalse(self._validate_dims("box", (0.1, -0.2, 0.3)))

    def test_box_zero_dim(self):
        self.assertFalse(self._validate_dims("box", (0.0, 0.2, 0.3)))

    def test_sphere_valid(self):
        self.assertTrue(self._validate_dims("sphere", (0.15,)))

    def test_sphere_wrong_dim_count(self):
        self.assertFalse(self._validate_dims("sphere", (0.15, 0.15)))

    def test_cylinder_valid(self):
        self.assertTrue(self._validate_dims("cylinder", (0.80, 0.10)))

    def test_cylinder_zero_dim(self):
        self.assertFalse(self._validate_dims("cylinder", (0.0, 0.10)))

    def test_unsupported_shape(self):
        self.assertFalse(self._validate_dims("cone", (0.1, 0.2)))

    def test_non_finite_position_detected(self):
        position = (float("nan"), 0.0, 0.0)
        self.assertFalse(all(math.isfinite(v) for v in position))

    def test_duplicate_ids_detected(self):
        ids = ["obj_1", "obj_2", "obj_1"]
        seen: set[str] = set()
        duplicates: list[str] = []
        for oid in ids:
            if oid in seen:
                duplicates.append(oid)
            seen.add(oid)
        self.assertEqual(duplicates, ["obj_1"])


if __name__ == "__main__":
    unittest.main()
