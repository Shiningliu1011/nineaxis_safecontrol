#!/usr/bin/env python3
"""M3 acceptance: DCOL alpha calibration script outputs."""

from __future__ import annotations

import tempfile
from pathlib import Path

import _path_setup  # noqa: F401
import pytest
import yaml

from work.obb_collision_model import OBB_COLLISION_PAIRS


def _run_calibration(tmp_dir: Path, max_global: int, max_local: int):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    import calibrate_dcol_alpha as module

    output_yaml = tmp_dir / "dcol_alpha.yaml"
    output_report = tmp_dir / "dcol_alpha_calibration_report.md"
    calibrations = module.calibrate(
        max_global_samples=max_global,
        max_local_samples_per_pair=max_local,
        seed=20260806,
        output_yaml=output_yaml,
        output_report=output_report,
    )
    return calibrations, output_yaml, output_report


def test_calibration_outputs_positive_alphas_for_all_pairs():
    with tempfile.TemporaryDirectory() as tmp:
        calibrations, output_yaml, output_report = _run_calibration(
            Path(tmp), max_global=400, max_local=60)

        assert len(calibrations) == len(OBB_COLLISION_PAIRS)
        for calibration in calibrations:
            assert calibration.alpha > 0.0
            assert calibration.samples >= 0
            assert calibration.rms_m >= 0.0
            assert calibration.v_max_m_s >= 0.0

        assert output_yaml.is_file()
        document = yaml.safe_load(output_yaml.read_text(encoding="utf-8"))
        assert document["schema_version"] == 1
        assert len(document["pairs"]) == len(OBB_COLLISION_PAIRS)
        for pair in document["pairs"]:
            assert pair["alpha"] > 0.0
            assert "rms_m" in pair
            assert "samples" in pair
            assert "v_max_m_s" in pair


def test_calibration_report_records_residual_rms():
    with tempfile.TemporaryDirectory() as tmp:
        _, _, output_report = _run_calibration(
            Path(tmp), max_global=400, max_local=60)
        assert output_report.is_file()
        text = output_report.read_text(encoding="utf-8")
        assert "RMS" in text
        assert "alpha" in text.lower()
        assert "synthetic" in text.lower()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
