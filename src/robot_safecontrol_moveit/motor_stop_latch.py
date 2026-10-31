import json
import os
from hashlib import sha256
from pathlib import Path


class MotorStopLatch:
    def __init__(self, path: Path):
        self.path = Path(path)

    def require_clear(self):
        if self.path.exists():
            with self.path.open(encoding="utf-8") as stream:
                report = json.load(stream)
            detail = report.get("hardware_blocker", report["reason"])
            raise RuntimeError(f"实机执行已锁定：{detail}；记录：{self.path}")

    def trip(self, *, reason: str, log_path: Path):
        if self.path.exists():
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("x", encoding="utf-8") as stream:
            json.dump(dict(reason=reason, log_path=str(log_path), manual_review_required=True),
                      stream, ensure_ascii=False, indent=2)
            stream.write("\n")

    def require_recoverable(self, *, expected_sha256: str) -> bytes:
        original = self.path.read_bytes()
        if sha256(original).hexdigest() != expected_sha256:
            raise RuntimeError("待恢复的运动锁定身份变化")
        blocker = json.loads(original).get("hardware_blocker")
        if blocker:
            raise RuntimeError(f"single_pose_hardware_blocked: {blocker}")
        return original

    def acknowledge_stop(self, *, expected_sha256: str, archive_path: Path):
        original = self.require_recoverable(expected_sha256=expected_sha256)
        with archive_path.open("xb") as stream:
            stream.write(original)
            stream.flush()
            os.fsync(stream.fileno())
        if self.path.read_bytes() != original:
            raise RuntimeError("人工恢复期间运动锁定内容变化")
        self.path.unlink()
