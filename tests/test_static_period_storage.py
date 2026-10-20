import json
from pathlib import Path

import pytest

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_period_measurement import PERIODS


def test_period_budget_index_requires_all_twenty_bound_files(tmp_path):
    from robot_safecontrol_moveit.static_period_measurement import load_period_storage_budgets

    entries = {}
    for period in PERIODS:
        for method in "ABCDE":
            name = f"period-{round(period * 1000)}-{method}"
            path = tmp_path / f"{name}.json"
            path.write_text(json.dumps({"run_id": name}))
            entries[name] = {"path": path.name, "sha256": content_hash(path)}
    index = tmp_path / "index.json"
    index.write_text(json.dumps(entries))
    loaded = load_period_storage_budgets(index)
    assert len(loaded) == 20
    assert loaded["period-20-A"]["path"] == str(tmp_path / "period-20-A.json")
    (tmp_path / "period-20-A.json").write_text("{}")
    with pytest.raises(ValueError, match="身份"):
        load_period_storage_budgets(index)
    del entries["period-20-A"]
    index.write_text(json.dumps(entries))
    with pytest.raises(ValueError, match="20"):
        load_period_storage_budgets(index)


def test_period_measurement_rejects_missing_guard_before_creating_output(tmp_path):
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from robot_safecontrol_moveit.static_period_measurement import run_period_study

    root = Path(__file__).resolve().parents[1]
    config = ExperimentConfig.load(root / "config/static_experiment_s0.yaml", resource_root=root)
    output = tmp_path / "study"
    with pytest.raises(ValueError, match="容量"):
        run_period_study(config, output, 230, storage_budget_index=None)
    assert not output.exists()
