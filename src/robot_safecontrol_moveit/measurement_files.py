from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from stat import S_ISREG


@dataclass(frozen=True)
class PreparedMeasurementFile:
    path: Path
    sha256: str
    size: int
    modified_ns: int
    inode: int

    def require_unchanged(self) -> None:
        current = self.path.stat()
        if (current.st_size, current.st_mtime_ns, current.st_ino) != (
            self.size, self.modified_ns, self.inode
        ):
            raise ValueError(f"采集文件在计算 SHA-256 后发生变化：{self.path}")


def prepare_measurement_files(files: list[Path]) -> tuple[PreparedMeasurementFile, ...]:
    if not files:
        raise ValueError("本组测量需要已经保存的非空文件")
    prepared = []
    for path in files:
        source = path.resolve(strict=True)
        before = source.stat()
        if not S_ISREG(before.st_mode) or before.st_size <= 0:
            raise ValueError(f"采集文件必须为非空普通文件：{source}")
        digest = sha256()
        with source.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        entry = PreparedMeasurementFile(source, digest.hexdigest(), before.st_size,
                                        before.st_mtime_ns, before.st_ino)
        entry.require_unchanged()
        prepared.append(entry)
    return tuple(prepared)
