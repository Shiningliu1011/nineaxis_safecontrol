import time
from copy import deepcopy
from typing import TypedDict


class _Timing(TypedDict):
    count: int
    seconds: float


class _TraceState(TypedDict):
    timings: dict[str, _Timing]
    counters: dict[str, float]


_state: _TraceState = {"timings": {}, "counters": {}}
_started = time.monotonic()
_last_progress = float("-inf")


def pipe_motion_trace(action, *args):
    global _state, _started, _last_progress
    if action == "reset":
        _state = {"timings": {}, "counters": {}}
        _started, _last_progress = time.monotonic(), float("-inf")
    elif action == "add":
        key, seconds = args
        item = _state["timings"].setdefault(key, {"count": 0, "seconds": 0.})
        item["count"] += 1
        item["seconds"] += seconds
    elif action == "count":
        key, value = args
        _state["counters"][key] = _state["counters"].get(key, 0) + value
    elif action in ("note", "progress"):
        elapsed = time.monotonic() - _started
        if action == "note" or elapsed - _last_progress >= 8:
            print(f"[A2 {elapsed:.1f}s] {args[0]}", flush=True)
            _last_progress = elapsed
    elif action == "get":
        return deepcopy(_state) | {"elapsed_seconds": time.monotonic() - _started}
    else:
        raise ValueError(f"PipeMotion:TraceAction：未知操作 {action}")
    return None
