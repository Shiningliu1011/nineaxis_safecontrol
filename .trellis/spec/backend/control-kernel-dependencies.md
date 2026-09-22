# 控制内核运行依赖

## 场景：固定 cbfpy、JAX 与 qpax 组合

### 1. 范围与触发条件

- 本规范适用于 `portable_oscbf/work` 中直接使用 `cbfpy`、`jax`、`jaxlib` 或 `qpax` 的代码，以及安装这些包的配置。
- 修改依赖版本、`CBFConfig` 参数、`CBF` 成员、QP 求解调用或返回值时，必须按本规范处理。
- 2026-09-19 的 [`cbfpy` 版本研究](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/75)确认当前内核依赖 `cbfpy 0.0.1` 的接口；随后在 [速度级 OSCBF 复用选型](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/68)中决定继续通过 pip 安装并固定该版本。
- `cbfpy 0.0.4` 更改了 `relax_cbf`、`cbf_relaxation_penalty`，并删除 `CBF.qp_solver`。升级会改变控制内核的 QP 调用与结果解析，必须作为独立的内核迁移进行验证。
- 决定版本时的 379 步记录中，QP 成功 `379/379`，`qp_primal_residual` 均为 `0`；控制步耗时平均 `9.062 ms`、P95 `10.174 ms`、最大 `13.027 ms`，满足 `20 ms` 限制。这些数值记录选择依据，后续升级仍要用当前代码、相同配置和相同输入重新取得证据。

### 2. 签名

`portable_oscbf/requirements.txt` 必须声明完整版本：

```text
cbfpy==0.0.1
jax==0.6.2
jaxlib==0.6.2
qpax==0.1.4
```

`setup.py` 必须用同一组值提供 ROS package 的运行依赖：

```python
PORTABLE_OSCBF_RUNTIME_REQUIREMENTS = [
    "cbfpy==0.0.1",
    "jax==0.6.2",
    "jaxlib==0.6.2",
    "qpax==0.1.4",
]

setup(
    install_requires=["setuptools", *PORTABLE_OSCBF_RUNTIME_REQUIREMENTS],
)
```

内核依赖以下运行接口：

```python
class NineaxisOSCBFVelocityConfig(CBFConfig):
    ...

cbf = CBF.from_config(config)
```

创建后的 `cbf` 必须提供这些成员：

```python
(
    "m",
    "num_cbf",
    "relax_cbf",
    "cbf_relaxation_penalty",
    "solver_tol",
    "P_qp",
    "q_qp",
    "G_qp",
    "h_qp",
    "qp_solver",
)
```

启用 `relax_cbf` 时，当前求解调用与返回值为：

```python
(
    x_qp,
    t_qp,
    s1_qp,
    s2_qp,
    z1_qp,
    z2_qp,
    converged,
    qp_iterations,
) = cbf.qp_solver(
    P_solve,
    q_solve,
    G_solve,
    h_solve,
    cbf.cbf_relaxation_penalty,
    solver_tol=cbf.solver_tol,
)
```

### 3. 运行要求

- `portable_oscbf/requirements.txt` 与 `PORTABLE_OSCBF_RUNTIME_REQUIREMENTS` 的包名和版本必须完全相同。
- 四个包必须使用 `==` 固定版本。上游包没有提供足够的 Python 与 JAX 兼容范围，不能改用开放版本范围。
- `portable_oscbf/requirements.txt` 必须随 ROS package 安装到 package share 目录。
- `cbfpy`、`jax` 和 `qpax` 是 required dependency，使用直接 import；缺少包时立即报告 `ImportError`。
- `jax_kernel_factory.py` 可以直接使用上述 `CBF` 成员。不得用 `hasattr` 或替代求解器分支掩盖接口差异。
- 当前依赖不读取环境变量；安装结果仅由依赖清单与 Python package metadata 判定。

### 4. 校验与错误矩阵

| 条件 | 必须得到的结果 |
| --- | --- |
| 依赖清单与 `setup.py` 不一致 | `test_dependency_manifest_matches_install_configuration` 失败 |
| 任一包没有使用规定版本 | 依赖清单检查或已安装版本检查失败 |
| 当前环境缺少任一 required dependency | import 或 package metadata 查询立即失败 |
| `CBF.from_config` 返回对象缺少规定成员 | `test_cbfpy_interface_contract` 失败 |
| 求解器返回值数量或含义改变 | 内核迁移完成前不得更新依赖版本 |
| 新版更改配置字段名称 | 同步修改配置类、内核调用、依赖检查和数值测试后才能接受 |

### 5. 正常、基准与异常场景

- 正常：四个规定版本已经安装，依赖清单与 `setup.py` 一致，`CBF.from_config` 生成的对象包含全部规定成员。
- 基准：在新的 Python 环境中安装 ROS package 或执行 `pip install -r portable_oscbf/requirements.txt`，得到相同的四包组合。
- 异常：只修改一处版本、使用 `cbfpy>=0.0.1`、缺少 `qp_solver`，或求解器返回值与八项结构不一致。任何一种情况都必须在接口检查或控制测试中报告失败。

### 6. 必需测试

运行：

```bash
pytest -q portable_oscbf/tests/test_cbfpy_dependency_contract.py
```

断言位置：

- `test_dependency_manifest_matches_install_configuration`：比较两份依赖定义，并确认四个版本都使用 `==`。
- `test_pinned_runtime_dependency_versions_are_installed`：通过 package metadata 比较当前环境中的完整版本。
- `test_cbfpy_interface_contract`：创建真实的 `NineaxisOSCBFVelocityConfig` 与 `CBF`，检查继承关系和十个运行成员。

任何有意的版本升级还必须运行 `bash run_all_tests.sh`，并重新记录 QP 成功率、残差、迭代次数与控制步耗时。记录必须包含 commit、日期、环境、配置、完整命令和退出码。

### 7. 错误与正确示例

错误：开放版本范围，并在运行时猜测新版接口。

```python
install_requires = ["cbfpy>=0.0.1", "jax", "jaxlib", "qpax"]

if hasattr(cbf, "qp_solver"):
    solver = cbf.qp_solver
else:
    solver = qpax.solve_qp
```

正确：安装经过验证的完整组合，并直接使用经过接口检查的成员。

```python
PORTABLE_OSCBF_RUNTIME_REQUIREMENTS = [
    "cbfpy==0.0.1",
    "jax==0.6.2",
    "jaxlib==0.6.2",
    "qpax==0.1.4",
]

cbf = CBF.from_config(config)
solution = cbf.qp_solver(
    P_solve,
    q_solve,
    G_solve,
    h_solve,
    cbf.cbf_relaxation_penalty,
    solver_tol=cbf.solver_tol,
)
```
