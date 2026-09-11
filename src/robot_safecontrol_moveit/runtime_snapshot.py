"""Atomic runtime evidence and software identity for production starts."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
from typing import Any, Mapping, Sequence
from uuid import uuid4


def persist_runtime_snapshot(payload: Mapping[str, Any], directory: Path) -> Path:
    """Atomically persist one uniquely named JSON runtime snapshot."""
    output_dir = directory.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    run_id = (
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        + "-"
        + uuid4().hex
    )
    final_path = output_dir / f"oscbf-runtime-{run_id}.json"
    temporary_path = output_dir / f".{final_path.name}.{uuid4().hex}.tmp"
    document = dict(payload)
    document["run_id"] = run_id
    document["created_at_utc"] = datetime.now(timezone.utc).isoformat()
    document["snapshot_path"] = str(final_path)
    try:
        with temporary_path.open("x", encoding="utf-8") as stream:
            json.dump(
                document,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, final_path)
    except Exception:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
        raise
    return final_path


def sha256_bytes(content: bytes) -> str:
    """Return the lowercase SHA-256 identity for an exact byte sequence."""
    return hashlib.sha256(content).hexdigest()


def collect_software_identity(
    *, repository_hint: Path, source_paths: Sequence[Path]
) -> dict[str, Any]:
    """Identify loaded control source and its surrounding Git checkout."""
    files: list[Path] = []
    for candidate in source_paths:
        resolved = candidate.expanduser().resolve()
        if resolved.is_file():
            files.append(resolved)
        elif resolved.is_dir():
            files.extend(sorted(resolved.rglob("*.py")))

    file_hashes: dict[str, str] = {}
    aggregate = hashlib.sha256()
    for source in sorted(set(files), key=str):
        content = source.read_bytes()
        label = str(source)
        digest = sha256_bytes(content)
        file_hashes[label] = digest
        aggregate.update(label.encode("utf-8"))
        aggregate.update(b"\0")
        aggregate.update(content)
        aggregate.update(b"\0")

    result: dict[str, Any] = {
        "source_sha256": aggregate.hexdigest(),
        "source_files": file_hashes,
        "git_head": None,
        "git_dirty": False,
        "git_root": None,
        "git_status": None,
    }
    try:
        root = subprocess.run(
            ["git", "-C", str(repository_hint), "rev-parse", "--show-toplevel"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        head = subprocess.run(
            ["git", "-C", root, "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "-C", root, "status", "--porcelain=v1"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.rstrip()
    except (OSError, subprocess.CalledProcessError):
        return result

    result.update(
        {
            "git_head": head,
            "git_dirty": bool(status),
            "git_root": root,
            "git_status": status,
        }
    )
    return result
