import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
PACK = ROOT / "docs/proposal-static-obstacles"


def checksum(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main(originals):
    inventory = json.loads((PACK / "checksums.json").read_text())
    for name, expected in inventory.items():
        path = ROOT / name
        if path.stat().st_size != expected["bytes"] or checksum(path) != expected["sha256"]:
            raise ValueError(f"文件身份不一致：{name}")
        if path.suffix == ".png":
            with Image.open(path) as image:
                if image.width < 600 or image.height < 400:
                    raise ValueError(f"图片尺寸不足：{name}")
                image.verify()
        if path.suffix == ".svg":
            ET.parse(path)
    rows = json.loads((PACK / "data/metrics.json").read_text())
    if len(rows) != 10 or len({row["run"] for row in rows}) != 10:
        raise ValueError("运行清单数量不一致")
    series_samples = 0
    for row in rows:
        directory = PACK / "data" / row["run"]
        report = json.loads((directory / "feedback_report.json").read_text())
        source = json.loads((directory / "source.json").read_text())
        manifest = json.loads((directory / "manifest.json").read_text())
        if row["scenario"] != manifest["config"]["scenario"] or row["termination"] != report["termination"]["termination"]:
            raise ValueError("摘要状态不一致")
        for column, metric, statistic in (("path_rms_mm", "path_distance_m", "rms"), ("path_p95_mm", "path_distance_m", "p95"), ("path_max_mm", "path_distance_m", "maximum")):
            if row[column] != report["metrics"][metric][statistic] * 1000:
                raise ValueError(f"摘要指标不一致：{column}")
        for name, expected in source["files_sha256"].items():
            if checksum(directory / name) != expected:
                raise ValueError("复制文件身份不一致")
        if originals:
            original = Path(source["original_directory"])
            for name, expected in source["files_sha256"].items():
                if checksum(original / name) != expected:
                    raise ValueError("原始文件身份不一致")
            for role, expected in source["database_sha256"].items():
                if checksum(original / f"{role}.sqlite3") != expected:
                    raise ValueError("原始数据库身份不一致")
        if source["series_exported"]:
            with gzip.open(directory / "feedback-full.csv.gz", "rt", encoding="utf-8", newline="") as stream:
                data = list(csv.DictReader(stream))
            if len(data) != report["feedback"]["samples"]:
                raise ValueError("反馈数量不一致")
            times = np.array([float(item["simulation_s"]) for item in data])
            if not np.all(np.diff(times) > 0):
                raise ValueError("反馈时间顺序不一致")
            for key in ("path_distance_m", "reference_position_error_m", "tool_axis_angle_rad", "tcp_workpiece_distance_m"):
                values = np.array([float(item[key]) for item in data])
                statistics = {"rms": np.sqrt(np.mean(values ** 2)), "p95": np.percentile(values, 95), "maximum": values.max(), "minimum": values.min()}
                for statistic, actual in statistics.items():
                    if not np.isclose(actual, report["metrics"][key][statistic], rtol=1e-12, atol=1e-15):
                        raise ValueError(f"完整反馈重算不一致：{row['run']}/{key}/{statistic}")
            with (directory / "feedback-25hz.csv").open(encoding="utf-8", newline="") as stream:
                sampled = list(csv.DictReader(stream))
            indices = np.unique(np.r_[np.arange(0, len(data), 20), len(data) - 1])
            if sampled != [data[index] for index in indices]:
                raise ValueError("显示用反馈抽样不一致")
            series_samples += len(data)
    print(json.dumps({"status": "pass", "files": len(inventory), "runs": len(rows), "full_series_samples": series_samples, "original_files_and_databases_checked": originals}, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--originals", action="store_true")
    main(parser.parse_args().originals)
