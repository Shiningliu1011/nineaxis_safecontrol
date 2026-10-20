from __future__ import annotations

import json
import math
import sqlite3
import time
import zlib
from contextlib import closing
from pathlib import Path

import msgpack
import zstandard


FORMAT_VERSION = 3
KEYFRAME_INTERVAL = 128
DICTIONARY_BYTES = 131072
ZLIB_DICTIONARY_BYTES = 32768


def _validate_values(value) -> None:
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("记录数值必须有限")
    elif isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError("记录字段名必须是字符串")
            _validate_values(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _validate_values(item)
    elif value is not None and not isinstance(value, (str, int, bool)):
        raise TypeError(f"不支持的记录类型：{type(value).__name__}")


def _version(connection) -> int:
    version = connection.execute("PRAGMA user_version").fetchone()[0]
    if version not in (0, 2, FORMAT_VERSION):
        raise ValueError(f"不支持的记录格式：{version}")
    return version


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
            "payload BLOB NOT NULL, keyframe INTEGER NOT NULL, "
            "codec INTEGER NOT NULL CHECK(codec IN (0, 1)))"
        )
        self.connection.execute("CREATE INDEX stream_sequence ON samples (stream, sequence)")
        self.connection.execute(f"PRAGMA user_version={FORMAT_VERSION}")
        self._last_times: dict[str, float] = {}
        self._previous: dict[str, tuple[int, int, bytes]] = {}
        self._sequence = 0

    def append(self, stream: str, *, monotonic_s: float, values: dict,
               simulation_s: float | None = None, ros_time_ns: int | None = None,
               sequence: int | None = None) -> float:
        started = time.monotonic()
        if not isinstance(stream, str) or not stream or not math.isfinite(monotonic_s):
            raise ValueError("stream 和 monotonic_s 必须有效")
        if stream in self._last_times and monotonic_s <= self._last_times[stream]:
            raise ValueError(f"{stream}: monotonic_s 必须严格递增")
        if simulation_s is not None and (not math.isfinite(simulation_s) or simulation_s < 0):
            raise ValueError("simulation_s 必须有限且非负")
        if ros_time_ns is not None and (not isinstance(ros_time_ns, int) or ros_time_ns < 0):
            raise ValueError("ros_time_ns 必须为非负整数")
        sequence = self._sequence + 1 if sequence is None else sequence
        if type(sequence) is not int or sequence <= self._sequence:
            raise ValueError("sequence 必须为严格递增的正整数")
        if not isinstance(values, dict):
            raise TypeError("values 必须为字典")
        _validate_values(values)
        raw = msgpack.packb(values, use_bin_type=True, use_single_float=False)
        count, keyframe, dictionary = self._previous.get(stream, (0, sequence, b""))
        if count % KEYFRAME_INTERVAL == 0:
            keyframe, dictionary = sequence, b""
        compressor = zlib.compressobj(level=6, zdict=dictionary[-ZLIB_DICTIONARY_BYTES:])
        zlib_payload = compressor.compress(raw) + compressor.flush()
        prefix = zstandard.ZstdCompressionDict(dictionary, dict_type=zstandard.DICT_TYPE_RAWCONTENT)
        zstd_payload = zstandard.ZstdCompressor(
            level=3, dict_data=prefix, write_checksum=True,
        ).compress(raw)
        codec, payload = (0, zlib_payload) if len(zlib_payload) <= len(zstd_payload) else (1, zstd_payload)
        self.connection.execute(
            "INSERT INTO samples (sequence, stream, monotonic_s, simulation_s, ros_time_ns, payload, keyframe, codec) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (sequence, stream, monotonic_s, simulation_s, ros_time_ns, payload, keyframe, codec),
        )
        self._sequence = sequence
        self._previous[stream] = (count + 1, keyframe, raw[-DICTIONARY_BYTES:])
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


def iter_records(path: Path, stream: str | None = None, *, sequence: int | None = None):
    with closing(sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)) as connection:
        version = _version(connection)
        conditions, parameters = [], []
        if stream is not None:
            conditions.append("stream=?")
            parameters.append(stream)
        if sequence is not None:
            anchor = connection.execute(
                "SELECT stream, " + ("keyframe" if version else "sequence") +
                " FROM samples WHERE sequence=?", (sequence,),
            ).fetchone()
            if anchor is None or (stream is not None and anchor[0] != stream):
                return
            conditions.extend(["stream=?", "sequence BETWEEN ? AND ?"])
            parameters.extend([anchor[0], anchor[1], sequence])
        where = " WHERE " + " AND ".join(conditions) if conditions else ""
        rows = connection.execute(
            "SELECT sequence, stream, monotonic_s, simulation_s, ros_time_ns, payload, " +
            ("keyframe" if version else "sequence") + ", " + ("codec" if version == 3 else "0") +
            " FROM samples" + where + " ORDER BY sequence",
            parameters,
        )
        previous = {}
        for number, name, monotonic_s, simulation_s, ros_time_ns, payload, keyframe, codec in rows:
            if version:
                if keyframe == number:
                    dictionary = b""
                else:
                    if name not in previous or previous[name][0] != keyframe:
                        raise ValueError(f"记录缺少压缩字典：{path}:{number}")
                    dictionary = previous[name][1]
                if codec == 0:
                    decoder = zlib.decompressobj(zdict=dictionary[-ZLIB_DICTIONARY_BYTES:])
                    raw = decoder.decompress(payload) + decoder.flush()
                    if not decoder.eof or decoder.unused_data:
                        raise ValueError(f"压缩记录不完整：{path}:{number}")
                elif codec == 1:
                    prefix = zstandard.ZstdCompressionDict(
                        dictionary, dict_type=zstandard.DICT_TYPE_RAWCONTENT,
                    )
                    raw = zstandard.ZstdDecompressor(dict_data=prefix).decompress(
                        payload, allow_extra_data=False,
                    )
                else:
                    raise ValueError(f"不支持的记录编码：{path}:{number}:{codec}")
                dictionary_bytes = DICTIONARY_BYTES if version == 3 else ZLIB_DICTIONARY_BYTES
                previous[name] = (keyframe, raw[-dictionary_bytes:])
                values = msgpack.unpackb(raw, raw=False)
            else:
                values = json.loads(payload)
            if sequence is None or number == sequence:
                yield {"sequence": number, "stream": name, "monotonic_s": monotonic_s,
                       "simulation_s": simulation_s, "ros_time_ns": ros_time_ns, "values": values}


def read_record(path: Path, sequence: int, *, stream: str) -> dict:
    row = next(iter_records(path, stream, sequence=sequence), None)
    if row is None:
        raise ValueError(f"缺少指定记录：{path}:{stream}:{sequence}")
    return row


def read_stream(path: Path, stream: str) -> list[dict]:
    return [{key: row[key] for key in ("monotonic_s", "simulation_s", "ros_time_ns", "values")}
            for row in iter_records(path, stream)]


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
