"""Unit/row identity and disabled/invalid measurements for OFF-15 diagnostics."""

from types import SimpleNamespace

import numpy as np

from work.jax_control_facade import JaxControlLoop


def _diagnostics(residuals, margins, enabled=False):
    # Declared 1P8R topology: 18 joint, 14 self, 10 obstacle, 1 singularity rows.
    loop = object.__new__(JaxControlLoop)
    loop.robot = SimpleNamespace(num_joints=9)
    loop._config = SimpleNamespace(obstacle_h_start=32, obstacle_h_stop=42,
                                   esdf_h_start=42, esdf_h_stop=42, enable_sdf=False)
    return loop._path_constraint_metrics(residuals, margins, {
        "obs_enabled": np.full(8, float(enabled)), "sdf_enabled": 0.,
    })


def test_linear_angular_and_collision_rows_keep_physical_units():
    residuals, margins = np.zeros(61), np.ones(43)
    residuals[0], residuals[9] = 0.0004, 0.0002
    residuals[2], residuals[18], residuals[33] = 0.03, 0.01, 0.2
    margins[18] = -0.005
    result = _diagnostics(residuals, margins, enabled=True)
    assert result["joint_linear.residual"]["value"] == 0.0004
    assert result["joint_linear.residual"]["unit"] == "m/s"
    assert result["joint_angular.residual"]["value"] == 0.03
    assert result["joint_angular.residual"]["unit"] == "rad/s"
    assert result["self_collision.residual"]["value"] == 0.01
    assert result["self_collision.static_margin"]["value"] == -0.005
    assert result["obstacle.residual"]["value"] == 0.2
    assert "before health gate" in result["obstacle.residual"]["source"]


def test_disabled_geometry_remains_unmeasured_instead_of_using_sentinels():
    result = _diagnostics(np.zeros(61), np.full(43, 1000.))
    assert result["obstacle.static_margin"]["value"] is None
    assert result["obstacle.residual"]["value"] is None
    assert result["obstacle.residual"]["active"] is False
    assert result["esdf.residual"]["value"] is None


def test_nonfinite_row_is_not_swallowed_by_positive_part():
    residuals = np.zeros(61)
    residuals[0] = np.nan
    result = _diagnostics(residuals, np.ones(43))
    assert np.isnan(result["joint_linear.residual"]["value"])
