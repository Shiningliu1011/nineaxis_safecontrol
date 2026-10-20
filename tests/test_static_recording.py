import json
import math
import multiprocessing
from pathlib import Path
import signal
import sqlite3
import struct
import time
import zlib

import msgpack
import numpy as np
import pytest
import zstandard

from robot_safecontrol_moveit.static_experiment_recording import (
    RunRecorder, finalize_recording, iter_records, read_record, read_stream,
)
from robot_safecontrol_moveit.static_experiment_storage import compact_database


ROOT = Path(__file__).resolve().parents[1]


def _recorded_problem():
    with np.load(ROOT / "tests/data/static_qp_feed_boundary.npz") as archive:
        return {key: archive[key].tolist() for key in archive.files}


def _assert_exact(actual, expected):
    assert type(actual) is type(expected)
    if isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in expected:
            _assert_exact(actual[key], expected[key])
    elif isinstance(expected, list):
        assert len(actual) == len(expected)
        for first, second in zip(actual, expected):
            _assert_exact(first, second)
    elif isinstance(expected, float):
        assert struct.pack("!d", actual) == struct.pack("!d", expected)
    else:
        assert actual == expected


def test_compact_records_preserve_values_and_random_access_across_keyframes(tmp_path):
    path = tmp_path / "records.sqlite3"
    values = {"problem": _recorded_problem(), "中文": [None, True, False, -0.0, 2**63 - 1]}
    with RunRecorder(path) as recorder:
        for number in range(1, 270):
            recorder.append(
                "candidate", sequence=number * 2, monotonic_s=float(number),
                simulation_s=number * .002, ros_time_ns=1234567890123456789 + number,
                values=values,
            )
        for number in (1, 127, 128, 129, 256, 269):
            row = read_record(path, number * 2, stream="candidate")
            _assert_exact(row["values"], values)
            assert row["ros_time_ns"] == 1234567890123456789 + number
        with pytest.raises(ValueError, match="缺少指定记录"):
            read_record(path, 3, stream="candidate")
    assert len(read_stream(path, "candidate")) == 269


def test_invalid_append_does_not_change_committed_records_or_dictionary(tmp_path):
    path = tmp_path / "records.sqlite3"
    with RunRecorder(path) as recorder:
        recorder.append("feedback", monotonic_s=1., values={"value": .1})
        for value in (math.nan, math.inf, -math.inf):
            with pytest.raises(ValueError, match="有限"):
                recorder.append("feedback", monotonic_s=2., values={"value": value})
        recorder.append("feedback", monotonic_s=2., values={"value": .2})
    assert [row["values"] for row in read_stream(path, "feedback")] == [{"value": .1}, {"value": .2}]


def test_legacy_conversion_preserves_source_bytes_and_all_columns(tmp_path):
    original, destination = tmp_path / "original.sqlite3", tmp_path / "compact.sqlite3"
    with sqlite3.connect(original) as connection:
        connection.execute(
            "CREATE TABLE samples (sequence INTEGER PRIMARY KEY, stream TEXT, "
            "monotonic_s REAL, simulation_s REAL, ros_time_ns INTEGER, payload TEXT)"
        )
        values = _recorded_problem()
        for number in range(1, 260):
            connection.execute("INSERT INTO samples VALUES (?, ?, ?, ?, ?, ?)",
                               (number, "candidate", float(number), None, None, json.dumps(values)))
    before = original.read_bytes()
    result = compact_database(original, destination)
    assert original.read_bytes() == before
    assert result["compact_bytes"] < result["source_bytes"] / 4
    for actual, expected in zip(iter_records(destination), iter_records(original), strict=True):
        _assert_exact(actual, expected)
    with pytest.raises(FileExistsError):
        compact_database(original, destination)


def test_version_two_upgrade_preserves_float64_and_random_access(tmp_path):
    source, destination = tmp_path / "version-two.sqlite3", tmp_path / "upgraded.sqlite3"
    values = {"problem": _recorded_problem(), "signed_zero": -0.0}
    with sqlite3.connect(source) as connection:
        connection.execute(
            "CREATE TABLE samples (sequence INTEGER PRIMARY KEY, stream TEXT, monotonic_s REAL, "
            "simulation_s REAL, ros_time_ns INTEGER, payload BLOB, keyframe INTEGER)"
        )
        connection.execute("PRAGMA user_version=2")
        previous = b""
        for number in range(1, 260):
            raw = msgpack.packb(values, use_bin_type=True, use_single_float=False)
            if (number - 1) % 128 == 0:
                previous, keyframe = b"", number
            compressor = zlib.compressobj(level=1, zdict=previous)
            payload = compressor.compress(raw) + compressor.flush()
            connection.execute("INSERT INTO samples VALUES (?, ?, ?, ?, ?, ?, ?)", (
                number, "candidate", float(number), number * .002, 1234567890123456789 + number,
                payload, keyframe,
            ))
            previous = raw[-32768:]
    before = source.read_bytes()
    result = compact_database(source, destination)
    assert result["recording_format"] == 3
    assert result["source_recording_format"] == 2
    assert source.read_bytes() == before
    for number in (1, 127, 128, 129, 256, 259):
        _assert_exact(read_record(destination, number, stream="candidate"),
                      read_record(source, number, stream="candidate"))


def _write_then_wait(path, pipe):
    recorder = RunRecorder(path)
    for number in range(300):
        recorder.append("feedback", monotonic_s=float(number), values=_recorded_problem())
    pipe.send(300)
    pipe.recv()


def test_sigkill_recovers_every_acknowledged_record_across_keyframes(tmp_path):
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    database = tmp_path / "records.sqlite3"
    process = context.Process(target=_write_then_wait, args=(database, child))
    process.start()
    try:
        assert parent.poll(30)
        expected = parent.recv()
        process.kill()
        process.join(30)
        assert process.exitcode == -signal.SIGKILL
        assert len(read_stream(database, "feedback")) == expected
        finalize_recording(database)
        _assert_exact(read_record(database, expected, stream="feedback")["values"], _recorded_problem())
    finally:
        if process.is_alive():
            process.kill()
            process.join()
        parent.close()
        child.close()


def test_unknown_format_and_corruption_are_rejected(tmp_path):
    path = tmp_path / "records.sqlite3"
    with RunRecorder(path) as recorder:
        recorder.append("feedback", monotonic_s=time.monotonic(), values=_recorded_problem())
    with sqlite3.connect(path) as connection:
        connection.execute("PRAGMA user_version=999")
    with pytest.raises(ValueError, match="格式"):
        read_stream(path, "feedback")
    with sqlite3.connect(path) as connection:
        connection.execute("PRAGMA user_version=3")
        connection.execute("UPDATE samples SET payload=?", (b"corrupted",))
    with pytest.raises((zlib.error, zstandard.ZstdError)):
        read_stream(path, "feedback")
