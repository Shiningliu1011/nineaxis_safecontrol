#!/usr/bin/env python3
"""
cbf_types.py
============
CBF 约束基础类型 — 被 dynamic_obstacles.py、oscbf_qp_solver.py 等模块共用。

从 dynamic_obstacles.py 提取，消除循环导入风险。
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class CbfConstraint:
    """单个 CBF 不等式约束: G_row @ u <= h_bound"""
    name: str                # 约束名称 (用于调试)
    G_row: np.ndarray       # 梯度行向量 (9,) 或矩阵 (k,9)
    h_bound: float          # 上界值
    h_value: float          # 当前 h 值 (用于日志)
    active: bool            # 是否激活


def _cbf_upper_bound(alpha: float, h_value: float,
                     h_dot_time: float = 0.0,
                     floor: Optional[float] = None) -> float:
    """CBF upper bound for G=-dh/dq with optional time-varying obstacle term.

    h(q,t) must satisfy dh/dq * u + dh/dt >= -alpha*h.
    With G=-dh/dq, the QP row is G*u <= alpha*h + dh/dt.
    """
    h_for_alpha = max(h_value, floor) if floor is not None else h_value
    return alpha * h_for_alpha + h_dot_time
