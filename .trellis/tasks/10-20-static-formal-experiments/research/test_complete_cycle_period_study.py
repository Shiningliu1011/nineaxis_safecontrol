import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path("/home/lsn/robot/robot_safecontrol")
ENTRY = Path(__file__).with_name("complete-cycle-period-study.py")


class PeriodStudyEntryTests(unittest.TestCase):
    def entry_module(self):
        specification = importlib.util.spec_from_file_location("complete_cycle_period_study", ENTRY)
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
        return module

    def test_rejects_actual_legacy_revision_before_preparation(self):
        result = subprocess.run(
            [sys.executable, str(ENTRY), "--resource-root", str(ROOT),
             "--runtime-revision", "40d43bb", "--check-only"],
            cwd=ROOT, capture_output=True, text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("生产源码与 runtime revision 不一致", result.stderr)

    def test_missing_current_capacity_keeps_exact_twenty_case_plan_pending(self):
        capacity = ROOT / "output/static-capacity-20261020/complete-cycle-preflight-01/engineering-plan/capacity_plan.json"
        if capacity.exists():
            self.skipTest("当前工程容量已经生成，需要使用其实际准入结果")
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, check=True,
                                  capture_output=True, text=True).stdout.strip()
        result = subprocess.run(
            [sys.executable, str(ENTRY), "--resource-root", str(ROOT),
             "--runtime-revision", revision, "--check-only"],
            cwd=ROOT, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        plan = json.loads(result.stdout)
        self.assertEqual(plan["status"], "pending_capacity")
        self.assertIn(str(capacity), plan["missing"])
        self.assertEqual(plan["cases"], [
            "period-20-A", "period-20-B", "period-20-C", "period-20-D", "period-20-E",
            "period-40-A", "period-40-B", "period-40-C", "period-40-D", "period-40-E",
            "period-50-A", "period-50-B", "period-50-C", "period-50-D", "period-50-E",
            "period-100-A", "period-100-B", "period-100-C", "period-100-D", "period-100-E",
        ])
        self.assertEqual(plan["domain"], 200)
        self.assertEqual(plan["worker_cpus"], "0-15")
        self.assertEqual(plan["monitor_cpus"], "22-27")

    def test_actual_old_capacity_cannot_admit_current_production_sources(self):
        sys.path.insert(0, str(ROOT / "src"))
        from robot_safecontrol_moveit.static_experiment_provenance import source_identity

        directory = ROOT / "output/static-capacity-20261020/three-volume-capacity-01/engineering-plan"
        with self.assertRaisesRegex(ValueError, "工程容量与当前生产源码身份不一致"):
            self.entry_module().validate_capacity_documents(
                directory / "engineering_evidence.json", directory / "capacity_plan.json", source_identity(ROOT),
            )

    def test_actual_legacy_study_cannot_supply_complete_cycle_evidence(self):
        sys.path.insert(0, str(ROOT / "src"))
        from robot_safecontrol_moveit.static_cycle_timing import CYCLE_TIMING_BOUNDARY

        directory = Path("/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-period-study-01")
        with self.assertRaisesRegex(ValueError, "共同周期计时边界不符"):
            self.entry_module().validate_study_documents(directory, CYCLE_TIMING_BOUNDARY)


if __name__ == "__main__":
    unittest.main()
