from __future__ import annotations

import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR = (
    REPO_ROOT / "portable_oscbf" / "scripts" / "generate_kinematics_data.py"
)
DEFAULT_AXIS_URDF = (
    Path(__file__).resolve().parent / "fixtures" / "default_axis.urdf"
)


def test_generated_kinematics_data_matches_urdf() -> None:
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_generator_uses_urdf_default_axis() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--urdf",
            str(DEFAULT_AXIS_URDF),
            "--stdout",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert "(1.0, 0.0, 0.0)" in result.stdout


def test_portable_urdf_copy_is_absent() -> None:
    assert not (REPO_ROOT / "portable_oscbf" / "urdf").exists()
