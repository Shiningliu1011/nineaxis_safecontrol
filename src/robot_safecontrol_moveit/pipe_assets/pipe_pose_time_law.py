from copy import deepcopy
from functools import reduce
from math import gcd

import numpy as np

from .pipe_pose_math import _parameters


def _normalize_plan(plan):
    result = dict(plan)
    for name in ("block_u", "block_s", "vs", "ve", "vfs", "stage_speed", "na", "nc", "nd", "length_residual", "original_boundary_u", "kept_boundary_indices"):
        if name in result:
            result[name] = np.asarray(result[name]).reshape(-1)
    return result


def _starts(plan):
    counts = sum(np.asarray(plan[name], dtype=float).reshape(-1) for name in ("na", "nc", "nd"))
    return np.r_[0, np.cumsum(counts * float(plan["Ts"]))]


def _phase(time, ta, tc, td, start_speed, middle_speed, end_speed):
    time = np.asarray(time, dtype=float)
    sa = (start_speed + middle_speed) * ta / 2
    distance = sa + middle_speed * (time - ta)
    speed = np.full_like(time, middle_speed)
    acceleration = np.zeros_like(time)
    jerk = np.zeros_like(time)
    if ta > 0:
        active = time <= ta
        x, delta = time[active], middle_speed - start_speed
        distance[active] = start_speed * x + delta / 2 * (x - ta / np.pi * np.sin(np.pi * x / ta))
        speed[active] = start_speed + delta / 2 * (1 - np.cos(np.pi * x / ta))
        acceleration[active] = delta * np.pi / (2 * ta) * np.sin(np.pi * x / ta)
        jerk[active] = delta * np.pi**2 / (2 * ta**2) * np.cos(np.pi * x / ta)
    if td > 0:
        active = time > ta + tc if ta + tc else time >= 0
        x, delta = time[active] - ta - tc, end_speed - middle_speed
        distance[active] = sa + middle_speed * tc + middle_speed * x + delta / 2 * (x - td / np.pi * np.sin(np.pi * x / td))
        speed[active] = middle_speed + delta / 2 * (1 - np.cos(np.pi * x / td))
        acceleration[active] = delta * np.pi / (2 * td) * np.sin(np.pi * x / td)
        jerk[active] = delta * np.pi**2 / (2 * td**2) * np.cos(np.pi * x / td)
    return distance, speed, acceleration, jerk


def _evaluate(plan, time):
    time = _parameters(time)
    starts = _starts(plan)
    if np.any((time < 0) | (time > starts[-1] + 1e-10)):
        raise ValueError("PipePose:Time 时间超出计划范围")
    time = np.minimum(time, starts[-1])
    block = np.clip(np.searchsorted(starts, time, side="right") - 1, 0, len(starts) - 2)
    values = [np.zeros_like(time) for _ in range(4)]
    middle = plan["stage_speed"] if "stage_speed" in plan else plan["vfs"]
    for index in np.unique(block):
        active = block == index
        local = _phase(time[active] - starts[index], *(float(plan[name][index]) * float(plan["Ts"]) for name in ("na", "nc", "nd")), float(plan["vs"][index]), float(middle[index]), float(plan["ve"][index]))
        for order in range(4):
            values[order][active] = local[order]
        values[0][active] += plan["block_s"][index]
    values[0][time == 0] = plan["block_s"][0]
    values[0][time == starts[-1]] = plan["block_s"][-1]
    values[0] = np.clip(values[0], plan["block_s"][0], plan["block_s"][-1])
    return {"time": time, "s": values[0], "v": np.maximum(values[1], 0), "a": values[2], "j": values[3], "block_index": block + 1}


def _invert(plan, arc):
    arc = _parameters(arc)
    block_s = np.asarray(plan["block_s"], dtype=float)
    if np.any((arc < 0) | (arc > block_s[-1])):
        raise ValueError("PipePose:Arc 弧长超出计划范围")
    starts = _starts(plan)
    index = np.clip(np.searchsorted(block_s, arc, side="right") - 1, 0, len(starts) - 2)
    low, high = starts[index].copy(), starts[index + 1].copy()
    for _ in range(52):
        middle = (low + high) / 2
        above = _evaluate(plan, middle)["s"] > arc
        high[above] = middle[above]
        low[~above] = middle[~above]
    time = (low + high) / 2
    time[arc == 0], time[arc == block_s[-1]] = 0, starts[-1]
    return time


def _stretch(plan, factor):
    factor = float(factor)
    if not np.isfinite(factor) or factor < 1:
        raise ValueError("PipePose:TimeScale 时间伸长倍数必须至少为 1")
    counts = np.concatenate([np.asarray(plan[name], dtype=int).reshape(-1) for name in ("na", "nc", "nd")])
    positive = counts[counts > 0]
    if not len(positive):
        raise ValueError("PipePose:TimeScale 计划缺少正阶段周期")
    divisor = reduce(gcd, (int(value) for value in positive))
    factor = np.ceil(factor * divisor) / divisor
    result = deepcopy(plan)
    for name in ("na", "nc", "nd"):
        result[name] = np.rint(np.asarray(plan[name], dtype=float) * factor).astype(int)
    for name in ("vs", "ve", "vfs", "stage_speed", "speed_ceiling"):
        if name in plan:
            result[name] = np.asarray(plan[name], dtype=float) / factor
    result.pop("validation", None)
    result["pose_time_stretch"] = factor
    result["pose_stretch_method"] = "uniform phase dilation on integer Ts grid; speeds divided by same factor"
    return result


def pipe_pose_time_law(action: str, *arguments):
    if action in ("eval", "time_at_arc", "stretch", "phase_times"):
        arguments = (_normalize_plan(arguments[0]), *arguments[1:])
    if action == "eval":
        return _evaluate(*arguments)
    if action == "time_at_arc":
        return _invert(*arguments)
    if action == "stretch":
        return _stretch(*arguments)
    if action == "phase_times":
        plan = arguments[0]
        starts = _starts(plan)
        return np.unique(np.r_[starts, starts[:-1] + np.asarray(plan["na"]) * plan["Ts"], starts[:-1] + (np.asarray(plan["na"], dtype=float) + np.asarray(plan["nc"], dtype=float)) * plan["Ts"]])
    raise ValueError(f"PipePose:TimeAction 未知动作：{action}")
