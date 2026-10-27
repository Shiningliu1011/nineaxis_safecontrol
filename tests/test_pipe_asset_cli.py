import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from robot_safecontrol_moveit.pipe_assets.cli import main
from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/data/pipe_assets"


def test_cli_regenerates_reference_and_fits_real_path(tmp_path):
    reference = tmp_path / "reference"
    subprocess.run([
        sys.executable, str(ROOT / "scripts/pipe_assets.py"), "reference",
        "--model", "demo", "--input", str(FIXTURES / "pipe_path_20261013_180806.mat"),
        "--output", str(reference),
    ], check=True, capture_output=True, text=True)
    expected = load_mat(FIXTURES / "pipe_path_20261013_180806.mat")["trajectory"]
    regenerated = load_mat(reference / "pipe_path.mat")["trajectory"]
    np.testing.assert_allclose(regenerated["tcpPoints"], expected["tcpPoints"], atol=2e-9)
    fitted = tmp_path / "fit"
    subprocess.run([
        sys.executable, str(ROOT / "scripts/pipe_assets.py"), "fit",
        "--input", str(reference / "pipe_path.csv"), "--output", str(fitted),
    ], check=True, capture_output=True, text=True)
    np.testing.assert_allclose(np.loadtxt(fitted / "control_points.txt"),
                               np.loadtxt(FIXTURES / "control_points.txt"), atol=2e-8)
    manifest = json.loads((fitted / "manifest.json").read_text())
    assert manifest["command"] == "fit"
    assert manifest["length_unit"] == "mm"
    assert manifest["input"]["sha256"]


def test_cli_rejects_overwriting_output(tmp_path):
    process = subprocess.run([
        sys.executable, str(ROOT / "scripts/pipe_assets.py"), "reference",
        "--output", str(tmp_path),
    ], capture_output=True, text=True)
    assert process.returncode != 0
    assert "FileExistsError" in process.stderr


def test_numeric_pose_cli_does_not_create_figures(tmp_path):
    import matplotlib.pyplot as plt

    before = set(plt.get_fignums())
    try:
        status = main([
            "pose", "--fit-dir", str(FIXTURES),
            "--input", str(FIXTURES / "pipe_path_20261013_180806.mat"),
            "--offline", str(FIXTURES / "nurbs_blocks.mat"),
            "--output", str(tmp_path / "pose"),
        ])
        assert status == 0
        assert set(plt.get_fignums()) == before
    finally:
        for number in set(plt.get_fignums()) - before:
            plt.close(number)
