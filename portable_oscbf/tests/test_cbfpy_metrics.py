#!/usr/bin/env python3
"""
test_cbfpy_metrics.py
=====================
验证 cbfpy 分支在 _step_tracking 中写入 _last_metrics，
使 PerfSummary 能得到 strict validation 所需字段。

验收标准:
- cbfpy 分支源码里会写 self._last_metrics
- 至少包含 ee_err_mm, orient_err_deg, dyn_min, h_min, qp_ok
- 测试在修复前失败, 修复后通过
"""

import ast
import os
import textwrap

import pytest

pytestmark = pytest.mark.skip(
    reason="asserts metrics written by the ROS-side newaxis tracking loop "
           "(excluded by OSCBF_PORTING_GUIDE.md §4.7); portable metrics "
           "contract is covered by M5"
)

# ── 源码路径 ──────────────────────────────────────────────────
_TRACKING = os.path.join(os.path.dirname(__file__), '..', 'newaxis', 'tracking_execution.py')

# strict validation 依赖的最小字段集
REQUIRED_FIELDS = {
    'ee_err_mm',
    'orient_err_deg',
    'dyn_min',
    'h_min',
    'qp_ok',
}

# 额外期望字段 (非严格必须, 但公平对比需要)
DESIRED_FIELDS = {
    'ee_x', 'ee_y', 'ee_z',
    't_cmd',
    'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8',
    'u_nom0', 'u_nom1', 'u_nom2', 'u_nom3', 'u_nom4', 'u_nom5', 'u_nom6', 'u_nom7', 'u_nom8',
    'u_safe0', 'u_safe1', 'u_safe2', 'u_safe3', 'u_safe4', 'u_safe5', 'u_safe6', 'u_safe7', 'u_safe8',
    'u_nom_norm',
    'du_norm',
    'static_min',
    'static_count',
    'dyn_count',
}


def _get_step_tracking_source() -> str:
    """提取 _step_tracking 方法的完整源码。"""
    with open(_TRACKING, 'r', encoding='utf-8') as fh:
        source = fh.read()
    tree = ast.parse(source, filename=_TRACKING)

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_step_tracking':
            return ast.get_source_segment(source, node) or ''
    pytest.fail('_step_tracking method not found in tracking_execution.py')


def _get_cbfpy_branch_source(tracking_src: str) -> str:
    """提取 cbfpy 分支 (jax_loop 路径) 的源码段。

    cbfpy 分支以 'if hasattr(self, 'jax_loop')' 开始,
    到该分支的 'return' 结束。
    """
    lines = tracking_src.splitlines()
    in_branch = False
    branch_lines = []
    base_indent = None

    for line in lines:
        stripped = line.strip()
        if 'hasattr(self' in stripped and 'jax_loop' in stripped and 'is_initialized' in stripped:
            in_branch = True
            base_indent = len(line) - len(line.lstrip())
            branch_lines.append(line)
            continue
        if in_branch:
            current_indent = len(line) - len(line.lstrip())
            # 分支内行 (缩进更深或空行)
            if stripped == '' or current_indent > base_indent:
                branch_lines.append(line)
            else:
                # 遇到同级或更浅缩进 → 分支结束
                break
    return '\n'.join(branch_lines)


class TestCbfpyMetricsWritten:
    """验证 cbfpy tracking 分支写入 _last_metrics。"""

    def test_cbfpy_branch_writes_last_metrics(self):
        """cbfpy 分支必须在 return 前赋值 self._last_metrics。"""
        src = _get_step_tracking_source()
        branch = _get_cbfpy_branch_source(src)

        assert 'self._last_metrics' in branch, (
            'cbfpy 分支 (_step_tracking 中 jax_loop 路径) 没有写入 self._last_metrics。'
            'PerfSummary.summary() 将缺失 ee_err_mm/orient_err_deg/dyn_min/h_min/qp_ok，'
            '导致 strict validation 报告 max_ee=inf, max_oe=inf, dyn_min=0。'
        )

    def test_cbfpy_branch_required_fields(self):
        """cbfpy 分支的 _last_metrics 必包含 strict validation 字段。"""
        src = _get_step_tracking_source()
        branch = _get_cbfpy_branch_source(src)

        # 从源码中提取 _last_metrics dict 的 key 字面量
        # 查找 self._last_metrics = { ... } 段
        metrics_start = branch.find('self._last_metrics')
        if metrics_start < 0:
            pytest.fail('cbfpy 分支未写入 self._last_metrics')

        # 取从赋值开始到下一个顶层语句之间的源码
        metrics_src = branch[metrics_start:]
        # 用 AST 解析提取 dict keys
        try:
            # 补全为合法表达式
            expr_src = metrics_src.split('=', 1)[1].strip()
            # 截取到匹配的 }
            depth = 0
            end = 0
            for i, ch in enumerate(expr_src):
                if ch == '{':
                    depth += 1
                elif ch == '}':
                    depth -= 1
                    if depth == 0:
                        end = i + 1
                        break
            expr_src = expr_src[:end]
            # 解析为 AST
            expr_tree = ast.parse(expr_src, mode='eval')
        except (SyntaxError, ValueError):
            # 如果无法解析, 回退到字符串搜索
            for field in REQUIRED_FIELDS:
                assert f"'{field}'" in branch or f'"{field}"' in branch, (
                    f'cbfpy 分支 _last_metrics 缺少必需字段: {field}'
                )
            return

        # 遍历 AST 提取所有字符串 key
        found_keys = set()
        for node in ast.walk(expr_tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                found_keys.add(node.value)

        for field in REQUIRED_FIELDS:
            assert field in found_keys, (
                f'cbfpy 分支 _last_metrics 缺少必需字段: {field}'
            )

    def test_cbfpy_branch_desired_fields(self):
        """cbfpy 分支的 _last_metrics 应包含公平对比所需字段。"""
        src = _get_step_tracking_source()
        branch = _get_cbfpy_branch_source(src)

        if 'self._last_metrics' not in branch:
            pytest.skip('cbfpy 分支未写入 _last_metrics, 先通过 test_cbfpy_branch_writes_last_metrics')

        # 检查 desired fields (允许缺失, 但报告)
        missing = []
        for field in DESIRED_FIELDS:
            if f"'{field}'" not in branch and f'"{field}"' not in branch:
                # 也检查 f-string 或变量引用
                if field not in branch:
                    missing.append(field)

        if missing:
            pytest.xfail(
                f'cbfpy 分支 _last_metrics 缺少以下期望字段 (非阻塞): {missing}'
            )

    def test_cbfpy_branch_records_jump_diagnostics_and_remembers_velocity(self):
        """JAX 分支也必须记录真实跳变，并保存下一帧的速度历史。"""
        src = _get_step_tracking_source()
        branch = _get_cbfpy_branch_source(src)

        assert 'self._jump_diag.update(' in branch, (
            'cbfpy 分支未调用跳变诊断器，输出的“0 事件”不能代表真实关节速度连续。'
        )
        assert 'self._remember_joint_velocity_cmd(u_safe)' in branch, (
            'cbfpy 分支未保存上一帧 u_safe，避障/恢复状态和跳变诊断会使用陈旧速度。'
        )

    def test_osqp_branch_still_writes_last_metrics(self):
        """OSQP 分支不受影响, 仍写入 _last_metrics。"""
        src = _get_step_tracking_source()
        # OSQP 分支在 cbfpy return 之后
        lines = src.splitlines()
        cbfpy_return_idx = None
        for i, line in enumerate(lines):
            if 'hasattr(self' in line and 'jax_loop' in line and 'is_initialized' in line:
                # 找到这个分支的 return
                for j in range(i + 1, len(lines)):
                    if lines[j].strip() == 'return':
                        cbfpy_return_idx = j
                        break
                break

        if cbfpy_return_idx is None:
            pytest.skip('未找到 cbfpy 分支 return')

        osqp_src = '\n'.join(lines[cbfpy_return_idx + 1:])
        assert 'self._last_metrics' in osqp_src, (
            'OSQP 分支 (cbfpy return 之后) 未找到 self._last_metrics 赋值'
        )


class TestPerfSummaryStrictFields:
    """验证 PerfSummary.summarize() 能从 cbfpy 指标中提取 strict 字段。"""

    def test_summarize_with_minimal_metrics(self):
        """PerfSummary 能从 ee_err_mm/orient_err_deg/dyn_min/h_min/qp_ok 提取 strict 字段。"""
        from work.perf_metrics import PerfSummary

        ps = PerfSummary()
        # 模拟 cbfpy 分支写入的最小指标
        for i in range(100):
            ps.record_step({
                'step': float(i),
                't': float(i) * 0.002,
                'ee_err_mm': 0.05,
                'orient_err_deg': 0.03,
                'dyn_min': 0.02,
                'h_min': 0.05,
                'qp_ok': 1.0,
            })

        stats = ps.summarize()
        assert 'max_ee_err_mm' in stats, 'PerfSummary 缺少 max_ee_err_mm'
        assert 'max_oe_deg' in stats, 'PerfSummary 缺少 max_oe_deg'
        assert 'min_dyn_min_mm' in stats, 'PerfSummary 缺少 min_dyn_min_mm'
        assert 'qp_fail_count' in stats, 'PerfSummary 缺少 qp_fail_count'

    def test_strict_pass_with_valid_metrics(self):
        """有效指标下 strict validation 应 PASS。"""
        from work.perf_metrics import PerfSummary, validate_strict

        ps = PerfSummary()
        for i in range(100):
            ps.record_step({
                'step': float(i),
                'ee_err_mm': 0.05,
                'orient_err_deg': 0.03,
                'dyn_min': 0.02,
                'h_min': 0.05,
                'qp_ok': 1.0,
            })

        stats = ps.summarize()
        result = validate_strict(stats)
        assert result['pass'], f'strict validation 应 PASS, 实际: {result["failures"]}'

    def test_strict_fail_without_metrics(self):
        """缺少指标时 strict validation 应 FAIL (max_ee=inf)。"""
        from work.perf_metrics import PerfSummary, validate_strict

        ps = PerfSummary()
        # 模拟当前 cbfpy 分支: 只有 step/t, 没有 ee_err_mm 等
        for i in range(100):
            ps.record_step({
                'step': float(i),
                't': float(i) * 0.002,
            })

        stats = ps.summarize()
        result = validate_strict(stats)
        assert not result['pass'], '缺少指标时 strict validation 应 FAIL'
        assert any('inf' in f or 'ee' in f.lower() for f in result['failures']), (
            f'应报告 ee_err 缺失, 实际: {result["failures"]}'
        )
