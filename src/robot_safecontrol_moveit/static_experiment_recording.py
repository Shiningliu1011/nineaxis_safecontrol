from __future__ import annotations

import json
import math
import sqlite3
import time
from contextlib import closing
from pathlib import Path


class RunRecorder:
    def __init__(self, path: Path):
        self.path = Path(path)
        if self.path.exists():
            raise FileExistsError(self.path)
        self.connection = sqlite3.connect(self.path, isolation_level=None)
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=NORMAL")
        self.connection.execute("PRAGMA wal_autocheckpoint=0")
        self.connection.execute(
            "CREATE TABLE samples (sequence INTEGER PRIMARY KEY, stream TEXT NOT NULL, "
            "monotonic_s REAL NOT NULL, simulation_s REAL, ros_time_ns INTEGER, "
            "payload TEXT NOT NULL)"
        )
        self._last_times: dict[str, float] = {}

    def append(self, stream: str, *, monotonic_s: float, values: dict,
               simulation_s: float | None = None, ros_time_ns: int | None = None) -> float:
        started = time.monotonic()
        if not stream or not math.isfinite(monotonic_s):
            raise ValueError("stream 和 monotonic_s 必须有效")
        if stream in self._last_times and monotonic_s <= self._last_times[stream]:
            raise ValueError(f"{stream}: monotonic_s 必须严格递增")
        if simulation_s is not None and (not math.isfinite(simulation_s) or simulation_s < 0):
            raise ValueError("simulation_s 必须有限且非负")
        if ros_time_ns is not None and (not isinstance(ros_time_ns, int) or ros_time_ns < 0):
            raise ValueError("ros_time_ns 必须为非负整数")
        payload = json.dumps(values, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        self.connection.execute(
            "INSERT INTO samples (stream, monotonic_s, simulation_s, ros_time_ns, payload) "
            "VALUES (?, ?, ?, ?, ?)",
            (stream, monotonic_s, simulation_s, ros_time_ns, payload),
        )
        self._last_times[stream] = monotonic_s
        return time.monotonic() - started

    def close(self) -> None:
        checkpoint = self.connection.execute("PRAGMA wal_checkpoint(TRUNCATE)").fetchone()
        if checkpoint[0]:
            raise RuntimeError(f"实验记录尚未完成 checkpoint：{self.path}")
        self.connection.close()

    def __enter__(self) -> RunRecorder:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def read_stream(path: Path, stream: str) -> list[dict]:
    with closing(sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)) as connection:
        rows = connection.execute(
            "SELECT monotonic_s, simulation_s, ros_time_ns, payload FROM samples "
            "WHERE stream=? ORDER BY sequence", (stream,),
        ).fetchall()
    return [
        {"monotonic_s": row[0], "simulation_s": row[1], "ros_time_ns": row[2],
         "values": json.loads(row[3])}
        for row in rows
    ]


def finalize_recording(path: Path) -> None:
    with closing(sqlite3.connect(path.resolve().as_uri() + "?mode=rw", uri=True)) as connection:
        checkpoint = connection.execute("PRAGMA wal_checkpoint(TRUNCATE)").fetchone()
        if checkpoint[0]:
            raise RuntimeError(f"实验记录仍有活动写入连接：{path}")
        if connection.execute("PRAGMA integrity_check").fetchone() != ("ok",):
            raise ValueError(f"实验数据库完整性检查失败：{path}")


def write_json(path: Path, values: dict) -> None:
    path.write_text(
        json.dumps(values, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
