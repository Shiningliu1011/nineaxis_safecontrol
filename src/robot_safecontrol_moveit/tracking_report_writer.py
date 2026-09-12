"""Background report persistence, independent of ROS and command publication."""

from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from tempfile import TemporaryDirectory

from .tracking_evaluator import TrackingEvaluator


def write_tracking_bundle(evaluator: TrackingEvaluator, path: str) -> str:
    """Render before replacing files; publish the hash-bearing summary last.

    The automatic writer only receives finished evaluators. Explicit offline
    callers may also write a live evaluator when they own its sampling thread.
    """
    target = Path(path)
    report = evaluator.report()
    artifacts = (
        (target.with_suffix(".samples.json"), evaluator.trace_json()),
        (target, report.markdown()),
        (target.with_suffix(".json"), report.json()),
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=".tracking-", dir=target.parent) as temporary:
        staged = []
        for index, (destination, content) in enumerate(artifacts):
            source = Path(temporary) / str(index)
            source.write_text(content, encoding="utf-8")
            staged.append((source, destination))
        for source, destination in staged:
            source.replace(destination)
    return str(target)


class TrackingReportWriter:
    """One terminal report per run; submissions and status never wait for I/O.

    Sampling already owns/deep-copies each input. finish() seals its public
    mutation API, so handing that evaluator to the worker needs no O(n) copy or
    serialization in the control callback. The worker never accesses a ROS node.
    """

    def __init__(self, *, write=None) -> None:
        self._write = write or write_tracking_bundle
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="tracking-report")
        self._future: Future | None = None
        self._evaluator: TrackingEvaluator | None = None
        self._path: str | None = None

    def submit(self, evaluator: TrackingEvaluator, path: str) -> None:
        if evaluator.termination is None:
            raise ValueError("finish sampling before background report submission")
        if self._future is not None:
            if self._evaluator is evaluator and self._path == path:
                return
            raise RuntimeError("a terminal report is already assigned to this writer")
        self._evaluator, self._path = evaluator, path
        try:
            self._future = self._executor.submit(self._write, evaluator, path)
        except Exception as exc:
            self._future = Future()
            self._future.set_exception(exc)

    def status(self) -> dict:
        if self._future is None:
            return {"state": "idle", "path": None, "error": None}
        if not self._future.done():
            return {"state": "writing", "path": self._path, "error": None}
        error = self._future.exception()
        return {"state": "failed" if error is not None else "saved",
                "path": self._path, "error": str(error) if error is not None else None}

    def close(self) -> None:
        """Drain only during node teardown, after its executor has stopped."""
        self._executor.shutdown(wait=True)
