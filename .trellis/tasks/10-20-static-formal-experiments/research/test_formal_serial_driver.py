import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path("/home/lsn/robot/robot_safecontrol")
ENTRY = Path(__file__).with_name("formal-serial-driver.py")
REVISION = "504a274740a1e0c27c597c8d61a889fe39154aed"


class FormalDriverTests(unittest.TestCase):
    def entry_module(self):
        specification = importlib.util.spec_from_file_location("formal_serial_driver", ENTRY)
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
        return module

    def test_default_preparation_keeps_actual_pending_study_and_outputs_unchanged(self):
        output = ROOT / "output/static-capacity-20261020/formal-serial-driver-01"
        self.assertFalse(output.exists())

        result = subprocess.run(
            [sys.executable, str(ENTRY), "--resource-root", str(ROOT),
             "--runtime-revision", REVISION], cwd=ROOT, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        plan = json.loads(result.stdout)
        self.assertEqual(plan["status"], "pending_study")
        self.assertEqual(plan["planned_runs"], 1080)
        self.assertEqual(plan["proven_configurations"], 39)
        self.assertEqual(len(plan["representatives"]), 12)
        self.assertFalse(plan["executed"])
        self.assertFalse(output.exists())

    def test_actual_legacy_study_is_rejected_before_formal_preparation(self):
        directory = Path("/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-period-study-01")
        with self.assertRaisesRegex(ValueError, "共同周期计时边界不符"):
            self.entry_module().period_status(directory)

    def test_execute_rejects_actual_old_source_before_creating_outputs(self):
        result = subprocess.run(
            [sys.executable, str(ENTRY), "--resource-root", str(ROOT),
             "--runtime-revision", "40d43bb", "--execute"],
            cwd=ROOT, capture_output=True, text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("生产源码与 runtime revision 不一致", result.stderr)
        self.assertFalse((ROOT / "output/static-capacity-20261020/formal-serial-driver-01").exists())


if __name__ == "__main__":
    unittest.main()
