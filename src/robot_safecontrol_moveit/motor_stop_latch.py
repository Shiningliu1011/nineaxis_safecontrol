import json
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
