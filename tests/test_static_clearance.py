from pathlib import Path

import jax
import numpy as np

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_clearance_evaluation import evaluate_clearances
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")
jax.config.update("jax_enable_x64", True)

from work.static_collision import StaticGeometry


def test_feedback_clearance_uses_actual_states_and_preserves_every_pair(tmp_path):
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    scene = config.scene()
    states = np.array([config.initial_position, config.initial_position + .001])
    report = evaluate_clearances(tmp_path, scene, states)
    assert report["external"]["status"] == "pass"
    assert report["external"]["samples"] == 2
    assert report["external"]["measured_pairs"] == 20
    assert report["self"]["status"] == "pass"
    assert report["self"]["measured_pairs"] == 36
    geometry = StaticGeometry(scene)
    independent = np.array([[row.distance.distance_m for row in geometry.query(q)] for q in states])
    with np.load(tmp_path / "clearance_metrics.npz", allow_pickle=False) as archive:
        np.testing.assert_allclose(archive["distance_m"], independent, atol=1e-8)
        assert archive["pairs"].shape == (56, 2)
