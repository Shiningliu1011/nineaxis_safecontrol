from __future__ import annotations

from copy import deepcopy
import itertools
import json
from pathlib import Path

import numpy as np
import yaml

from .static_experiment_config import content_hash
from .static_experiment_recording import write_json
from .static_scene_evidence import initial_configurations, obstacle_candidates, blocked_path_evidence


def prepare_development(config, baseline: Path, directory: Path, selection_path: Path | None = None) -> dict:
    baseline, directory = baseline.resolve(strict=True), directory.resolve()
    report = json.loads((baseline / "feedback_report.json").read_text())
    for role in ("plant", "controller"):
        if content_hash(baseline / f"{role}.sqlite3") != report["records_sha256"][role]:
            raise ValueError("S0 原始记录身份与评价不一致")
    directory.mkdir(parents=True, exist_ok=False)
    initial_positions = initial_configurations(config)
    from work.static_collision import StaticGeometry

    evidence = obstacle_candidates(config, baseline)
    write_json(directory / "obstacle_candidates.json", evidence)
    write_json(directory / "initial_configurations.json", {"positions": initial_positions})
    base = config.manifest()["config"]
    base_scene = config.scene()

    def legal(obstacles):
        scene = deepcopy(base_scene)
        scene["obstacles"] = obstacles
        geometry = StaticGeometry(scene)
        return all(
            row.distance.valid and row.distance.distance_m >= row.required_m
            for q in initial_positions for row in geometry.query(np.asarray(q))
        )

    single = next((item for item in evidence["candidates"] if legal([item["obstacle"]])), None)
    if single is None:
        raise ValueError("S1 候选缺少对三组初态均有效的障碍物")
    pair = None
    for first, second in itertools.combinations(evidence["candidates"], 2):
        if first["fraction"] != second["fraction"] or first["link"] == second["link"]:
            continue
        if np.linalg.norm(np.asarray(first["obstacle"]["center_m"]) - second["obstacle"]["center_m"]) <= .12:
            continue
        obstacles = [deepcopy(first["obstacle"]), deepcopy(second["obstacle"])]
        obstacles[1]["name"] = "obstacle_2"
        if legal(obstacles):
            pair = first, second, obstacles
            break
    if pair is None:
        raise ValueError("S2 候选缺少同时限制不同连杆且三组初态合法的组合")
    selected = {"S1": [single], "S2": [pair[0], pair[1]], "S4": [pair[0], pair[1]]}
    reasons = dict.fromkeys(selected, "S0 实际反馈的中间连杆距离低于要求，三组初态及完整工具通道均合法")
    if selection_path is not None:
        selection = json.loads(selection_path.read_text())
        if (selection["candidate_source_sha256"] != content_hash(directory / "obstacle_candidates.json")
                or set(selection["scenes"]) != set(selected)):
            raise ValueError("场景选择必须对应当前候选身份并包含 S1、S2、S4")
        for scenario, item in selection["scenes"].items():
            indices = item["candidate_indices"]
            expected_lengths = {"S1": (1,), "S2": (2,), "S4": (1, 2)}
            if (not isinstance(indices, list) or len(indices) not in expected_lengths[scenario]
                    or any(type(index) is not int or not 0 <= index < len(evidence["candidates"])
                           for index in indices)
                    or len(set(indices)) != len(indices)
                    or not isinstance(item["reason"], str) or not item["reason"].strip()):
                raise ValueError(f"{scenario}: 候选编号或选择理由无效")
            selected[scenario] = [evidence["candidates"][index] for index in indices]
            reasons[scenario] = item["reason"]
    obstacles_by_scene = {}
    for scenario, items in selected.items():
        if len(items) == 2 and (
            items[0]["fraction"] != items[1]["fraction"] or items[0]["link"] == items[1]["link"]
            or np.linalg.norm(np.asarray(items[0]["obstacle"]["center_m"])
                              - items[1]["obstacle"]["center_m"]) <= .12
        ):
            raise ValueError(f"{scenario}: 两个障碍物需要在同一进度处限制不同连杆且彼此分离")
        obstacles = [
            {**deepcopy(item["obstacle"]), "name": f"obstacle_{index}"}
            for index, item in enumerate(items, start=1)
        ]
        if not legal(obstacles):
            raise ValueError(f"{scenario}: 选择的障碍物不满足三组初态的全部几何要求")
        obstacles_by_scene[scenario] = obstacles
    blocked = blocked_path_evidence(config, initial_positions)
    write_json(directory / "s5_blockage.json", blocked)
    scenes = [
        ("S0", [], 1.), ("S1", obstacles_by_scene["S1"], 1.),
        ("S2", obstacles_by_scene["S2"], 1.),
        *[("S3", obstacles_by_scene["S1"], scale) for scale in (1., .7, .4)],
        ("S4", obstacles_by_scene["S4"], 1.), ("S5", [blocked["obstacle"]], 1.),
    ]
    entries = []
    for scenario, obstacles, scale in scenes:
        values = deepcopy(base)
        values["scenario"], values["obstacles"] = scenario, obstacles
        if scenario == "S3":
            values["joint_travel_scale"] = scale
        name = scenario.lower() + (f"-travel-{round(scale * 100)}" if scenario == "S3" else "")
        path = directory / f"{name}.yaml"
        path.write_text(yaml.safe_dump(values, sort_keys=False, allow_unicode=True), encoding="utf-8")
        entries.append({
            "id": name, "scenario": scenario, "config": path.name, "config_sha256": content_hash(path),
            "feasibility": "blockage_proven" if scenario == "S5" else "not_yet_verified",
        })
    catalog = {
        "schema_version": 1, "kind": "development", "baseline": str(baseline),
        "baseline_report_sha256": content_hash(baseline / "feedback_report.json"),
        "initial_positions": initial_positions, "feed_factors": [.5, 1., 1.5],
        "scenarios": entries,
        "selected_obstacles": {scenario: {"candidates": items, "reason": reasons[scenario]}
                               for scenario, items in selected.items()},
        "selection_sha256": content_hash(selection_path) if selection_path is not None else None,
        "s4_witness": {"status": "insufficient_evidence"},
    }
    write_json(directory / "catalog.json", catalog)
    return catalog
