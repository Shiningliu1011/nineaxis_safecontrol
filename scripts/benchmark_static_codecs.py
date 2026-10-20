from __future__ import annotations

import argparse
from collections import Counter
from contextlib import closing
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time
import zlib

import msgpack
import numpy as np
import zstandard

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_recording import (
    finalize_recording, iter_records, read_record, write_json,
)


CODECS = {"zlib-32": ("zlib", 1, 32768), "zstd-128": ("zstd", 3, 131072),
          "zstd-256": ("zstd", 3, 262144), "zlib6-32": ("zlib", 6, 32768),
          "zlib9-32": ("zlib", 9, 32768), "zstd6-128": ("zstd", 6, 131072),
          "zstd9-128": ("zstd", 9, 131072)}
RUNS = ("s0-joint-full-geometry-01", "s2-joint-development-01",
        "s4-joint-development-10", "s5-joint-development-01")


def encode(raw, dictionary, codec):
    algorithm, level, _ = CODECS[codec]
    if algorithm == "zlib":
        compressor = zlib.compressobj(level=level, zdict=dictionary)
        return compressor.compress(raw) + compressor.flush()
    prefix = zstandard.ZstdCompressionDict(dictionary, dict_type=zstandard.DICT_TYPE_RAWCONTENT)
    return zstandard.ZstdCompressor(
        level=level, dict_data=prefix, write_checksum=True,
    ).compress(raw)


def decode(payload, dictionary, codec):
    if CODECS[codec][0] == "zlib":
        decoder = zlib.decompressobj(zdict=dictionary)
        raw = decoder.decompress(payload) + decoder.flush()
        if not decoder.eof or decoder.unused_data:
            raise ValueError("zlib 记录不完整")
        return raw
    prefix = zstandard.ZstdCompressionDict(dictionary, dict_type=zstandard.DICT_TYPE_RAWCONTENT)
    return zstandard.ZstdDecompressor(dict_data=prefix).decompress(
        payload, allow_extra_data=False,
    )


def benchmark_write(source, destination, codec, *, interrupt=False):
    if destination.exists():
        raise FileExistsError(destination)
    connection = sqlite3.connect(destination, isolation_level=None)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=NORMAL")
    connection.execute("PRAGMA wal_autocheckpoint=0")
    connection.execute(
        "CREATE TABLE samples (sequence INTEGER PRIMARY KEY, stream TEXT NOT NULL, "
        "monotonic_s REAL NOT NULL, simulation_s REAL, ros_time_ns INTEGER, "
        "payload BLOB NOT NULL, keyframe INTEGER NOT NULL)"
    )
    connection.execute("CREATE INDEX stream_sequence ON samples (stream, sequence)")
    connection.execute("CREATE TABLE benchmark_codec (name TEXT NOT NULL)")
    connection.execute("INSERT INTO benchmark_codec VALUES (?)", (codec,))
    # 实验编码使用独立版本号，生产 reader 不接受这些测量数据库。
    connection.execute("PRAGMA user_version=100")
    previous, durations, counts, sizes = {}, {}, Counter(), Counter()
    digest = hashlib.sha256()
    for row in iter_records(source):
        stream, number = row["stream"], row["sequence"]
        started = time.perf_counter()
        raw = msgpack.packb(row["values"], use_bin_type=True, use_single_float=False)
        count, keyframe, dictionary = previous.get(stream, (0, number, b""))
        if count % 128 == 0:
            keyframe, dictionary = number, b""
        payload = encode(raw, dictionary, codec)
        connection.execute("INSERT INTO samples VALUES (?, ?, ?, ?, ?, ?, ?)", (
            number, stream, row["monotonic_s"], row["simulation_s"], row["ros_time_ns"],
            payload, keyframe,
        ))
        previous[stream] = count + 1, keyframe, raw[-CODECS[codec][2]:]
        durations.setdefault(stream, []).append(time.perf_counter() - started)
        digest.update(msgpack.packb(row, use_bin_type=True, use_single_float=False))
        counts[stream] += 1
        sizes[stream] += len(payload)
        if interrupt and sum(counts.values()) == 300:
            print(json.dumps({"counts": dict(counts), "sha256": digest.hexdigest()}), flush=True)
            sys.stdin.read(1)
            raise RuntimeError("恢复验证等待 SIGKILL")
    wal_bytes = Path(str(destination) + "-wal").stat().st_size
    connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    connection.close()
    return {
        "counts": dict(counts), "sha256": digest.hexdigest(),
        "database_bytes": destination.stat().st_size, "wal_bytes_before_checkpoint": wal_bytes,
        "payload_bytes": dict(sizes),
        "encode_and_transaction_s": {
            stream: {"mean": float(np.mean(values)), "p999": float(np.quantile(values, .999)),
                     "maximum": float(max(values)), "sum": float(sum(values))}
            for stream, values in durations.items()
        },
    }


def benchmark_read(path, *, sequence=None):
    with closing(sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)) as connection:
        codec = connection.execute("SELECT name FROM benchmark_codec").fetchone()[0]
        parameters, where = (), ""
        if sequence is not None:
            stream, keyframe = connection.execute(
                "SELECT stream, keyframe FROM samples WHERE sequence=?", (sequence,),
            ).fetchone()
            where, parameters = " WHERE stream=? AND sequence BETWEEN ? AND ?", (stream, keyframe, sequence)
        previous = {}
        for number, stream, monotonic_s, simulation_s, ros_time_ns, payload, keyframe in connection.execute(
                "SELECT sequence, stream, monotonic_s, simulation_s, ros_time_ns, payload, keyframe "
                "FROM samples" + where + " ORDER BY sequence", parameters):
            dictionary = b"" if keyframe == number else previous[stream]
            raw = decode(payload, dictionary, codec)
            previous[stream] = raw[-CODECS[codec][2]:]
            if sequence is None or number == sequence:
                yield {
                    "sequence": number, "stream": stream, "monotonic_s": monotonic_s,
                    "simulation_s": simulation_s, "ros_time_ns": ros_time_ns,
                    "values": msgpack.unpackb(raw, raw=False),
                }


def verify(source, destination, expected):
    digest, counts = hashlib.sha256(), Counter()
    started = time.perf_counter()
    for actual, original in zip(benchmark_read(destination), iter_records(source), strict=True):
        packed = msgpack.packb(actual, use_bin_type=True, use_single_float=False)
        if packed != msgpack.packb(original, use_bin_type=True, use_single_float=False):
            raise ValueError("逐条记录或 float64 位模式变化")
        digest.update(packed)
        counts[actual["stream"]] += 1
        if counts[actual["stream"]] in (1, 127, 128, 129, 256, 300):
            indexed = next(benchmark_read(destination, sequence=actual["sequence"]))
            if msgpack.packb(indexed) != msgpack.packb(actual):
                raise ValueError("跨 keyframe 随机读取内容变化")
    if digest.hexdigest() != expected["sha256"] or dict(counts) != expected["counts"]:
        raise ValueError("记录身份或数量变化")
    return time.perf_counter() - started


def verify_interruption(source, destination, codec):
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), "--interrupt-source", str(source),
         "--output", str(destination), "--codec", codec],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
    )
    try:
        acknowledgement = json.loads(process.stdout.readline())
        process.kill()
        if process.wait(timeout=30) != -signal.SIGKILL:
            raise ValueError("恢复检查未获得 SIGKILL")
        digest, counts = hashlib.sha256(), Counter()
        for actual in benchmark_read(destination):
            original = read_record(source, actual["sequence"], stream=actual["stream"])
            packed = msgpack.packb(actual, use_bin_type=True, use_single_float=False)
            if packed != msgpack.packb(original, use_bin_type=True, use_single_float=False):
                raise ValueError("恢复记录内容变化")
            digest.update(packed)
            counts[actual["stream"]] += 1
        if digest.hexdigest() != acknowledgement["sha256"] or dict(counts) != acknowledgement["counts"]:
            raise ValueError("恢复记录未包含全部已提交内容")
        finalize_recording(destination)
        return acknowledgement
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdin.close()
        process.stdout.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interrupt-source", type=Path)
    parser.add_argument("--codec", choices=tuple(CODECS))
    parser.add_argument("--codecs", nargs="+", choices=tuple(CODECS),
                        default=["zlib-32", "zstd-128", "zstd-256"])
    args = parser.parse_args()
    if args.interrupt_source is not None:
        benchmark_write(args.interrupt_source, args.output, args.codec, interrupt=True)
        return
    args.output.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "running", "codecs": {key: CODECS[key] for key in args.codecs},
        "rows": [], "command": sys.argv,
        "cpu_affinity": sorted(os.sched_getaffinity(0)), "python": sys.version,
        "zstandard": zstandard.__version__, "zstd": zstandard.ZSTD_VERSION,
        "sqlite": sqlite3.sqlite_version, "script_sha256": content_hash(Path(__file__)),
        "measured_at": datetime.now(timezone.utc).isoformat(),
    }
    for run in RUNS:
        for role in ("plant", "controller"):
            source = args.source_root / run / f"{role}.sqlite3"
            before = content_hash(source)
            for codec in args.codecs:
                directory = args.output / run / codec
                directory.mkdir(parents=True, exist_ok=True)
                database = directory / f"{role}.sqlite3"
                values = benchmark_write(source, database, codec)
                values["read_and_compare_s"] = verify(source, database, values)
                values["sigkill"] = verify_interruption(source, directory / f"{role}-sigkill.sqlite3", codec)
                if content_hash(source) != before:
                    raise ValueError(f"来源记录身份变化：{source}")
                report["rows"].append({
                    "run": run, "role": role, "codec": codec, "source": str(source),
                    "source_sha256": before, "database_sha256": content_hash(database), **values,
                })
                write_json(args.output / "benchmark.json", report)
                print(f"{run}/{role}/{codec}: {values['database_bytes']} bytes", flush=True)
    report["status"] = "pass"
    write_json(args.output / "benchmark.json", report)


if __name__ == "__main__":
    main()
