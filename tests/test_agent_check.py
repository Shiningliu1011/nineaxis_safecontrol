import os
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_agent_check_verifies_ros_conventions_import_boundary():
    temp_dir = REPO_ROOT / "output" / "agent-tmp" / "test-agent-check"
    temp_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["TMPDIR"] = str(temp_dir)

    completed = subprocess.run(
        ["bash", "scripts/agent_check.sh"],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "[PASS] ros_conventions import without rclpy" in completed.stdout
