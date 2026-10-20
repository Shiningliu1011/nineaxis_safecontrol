from __future__ import annotations

import argparse
from contextlib import closing
import json
import os
from pathlib import Path
import sqlite3
import sys

import msgpack

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_provenance import validate_run_report
from robot_safecontrol_moveit.static_experiment_recording import iter_records, write_json
from robot_safecontrol_moveit.static_experiment_storage import compact_database


def inspect_database(path: Path, *, conversion: Path) -> dict:
    identity = content_hash(path)
    with closing(sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)) as connection:
        version = connection.execute("PRAGMA user_version").fetchone()[0]
        if version != 3:
            migration = compact_database(path, conversion)
            return {
                **inspect_database(conversion, conversion=conversion),
                "original_path": str(path), "original_sha256": identity, "migration": migration,
            }
        streams = {
            name: {
                "count": count, "encoded_bytes": total,
                "keyframe_maximum_bytes": keyframe, "delta_maximum_bytes": delta or 0,
                "maximum_raw_bytes": 0, "fields": [], "trial_fields": [],
                "maximum_trials": 0, "maximum_pairs": 0, "problem_shapes": {},
            }
            for name, count, total, keyframe, delta in connection.execute(
                "SELECT stream,count(*),sum(length(payload)),"
                "max(CASE WHEN keyframe=sequence THEN length(payload) END),"
                "max(CASE WHEN keyframe!=sequence THEN length(payload) END) "
                "FROM samples GROUP BY stream"
            )
        }
        page_size = connection.execute("PRAGMA page_size").fetchone()[0]
        page_count = connection.execute("PRAGMA page_count").fetchone()[0]
    for row in iter_records(path):
        values, item = row["values"], streams[row["stream"]]
        item["maximum_raw_bytes"] = max(item["maximum_raw_bytes"], len(msgpack.packb(
            values, use_bin_type=True, use_single_float=False,
        )))
        item["fields"] = sorted(set(item["fields"]) | values.keys())
        item["maximum_trials"] = max(item["maximum_trials"], len(values.get("trials", [])))
        item["maximum_pairs"] = max(item["maximum_pairs"], len(values.get("geometry", [])))
        for trial in values.get("trials", []):
            item["trial_fields"] = sorted(set(item["trial_fields"]) | trial.keys())
            for name, value in trial["problem"].items():
                shape = [len(value)]
                if value and isinstance(value[0], list):
                    shape.append(len(value[0]))
                item["problem_shapes"][name] = max(shape, item["problem_shapes"].get(name, []))
    if content_hash(path) != identity:
        raise ValueError(f"容量审计期间来源变化：{path}")
    records = sum(item["count"] for item in streams.values())
    payload_bytes = sum(item["encoded_bytes"] for item in streams.values())
    return {
        "path": str(path), "sha256": identity, "format": version, "streams": streams,
        "page_size": page_size, "page_count": page_count, "record_count": records,
        "database_bytes": path.stat().st_size, "allocated_bytes": path.stat().st_blocks * 512,
        "payload_bytes": payload_bytes,
        "sqlite_overhead_bytes_per_record": (path.stat().st_size - payload_bytes) / records,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = {
        "status": "running", "command": sys.argv, "script_sha256": content_hash(Path(__file__)),
        "cpu_affinity": sorted(os.sched_getaffinity(0)), "runs": [],
    }
    for index, run in enumerate(args.run):
        run = run.resolve(strict=True)
        manifest, _ = validate_run_report(run)
        item = {
            "path": str(run), "manifest_sha256": content_hash(run / "manifest.json"),
            "config": manifest["config"], "config_sha256": manifest["config_sha256"],
            "source_sha256": manifest["source_sha256"],
            "resource_root": manifest["resource_root"], "overrides": manifest["overrides"],
            "recording_limits": manifest.get("recording_limits"),
            "databases": {
                role: inspect_database(
                    run / f"{role}.sqlite3", conversion=output / f"{index}-{role}-format3.sqlite3",
                ) for role in ("plant", "controller")
            },
            "other_allocated_bytes": sum(
                path.stat().st_blocks * 512 for path in run.rglob("*")
                if path.is_file() and path.suffix != ".sqlite3" and not path.name.endswith(("-wal", "-shm"))
            ),
        }
        result["runs"].append(item)
        write_json(output / "envelopes.json", result)
        print(json.dumps({"run": str(run), "status": "pass"}, ensure_ascii=False), flush=True)
    result["status"] = "pass"
    write_json(output / "envelopes.json", result)


if __name__ == "__main__":
    main()
