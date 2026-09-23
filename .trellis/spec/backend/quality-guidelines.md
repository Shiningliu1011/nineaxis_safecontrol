# 质量与验证

## 编码规则

- Python identifier 使用英文，模块和函数使用 `snake_case`，类型使用 `PascalCase`，常量使用 `UPPER_SNAKE_CASE`。
- 数据对象优先使用带类型标注的 `@dataclass(frozen=True)`；JAX 编译边界使用 JAX 可识别且形状固定的结构。
- required dependency 直接 import。不能用 import fallback 隐藏缺失依赖。
- 输入校验放在资源创建和副作用之前；错误信息包含字段、阶段或资源路径。
- 纯逻辑与 ROS I/O 分开，使状态机和数值规则可在没有 ROS graph 的情况下验证。
- 复用公共身份、topic、坐标变换、配置验证和生成脚本，不能复制同义常量。
- 注释说明安全原因、边界条件和容易误解的语义；identifier 保持原名。
- 新增 Python 文件不添加文件首部 docstring 或 shebang。已有文件风格仅作为当前代码事实，后续改动遵守 `AGENTS.md`。

## JAX 与数值代码

- JIT 路径保持固定 shape、固定 dtype 和可追踪数据结构。
- 不适用的数值槽位使用独立的 measured/valid 字段，不能只依赖 sentinel。
- host boundary 负责把 JAX array 转成 Python 或 NumPy 结果，编译函数内不能执行 ROS I/O 或文件写入。
- QP 终态必须核对 finite、stationarity、complementarity、primal residual、非负性和原始 inequality；不能只相信 solver 的单一布尔值。
- 碰撞与控制几何保持项目要求的精度，降低 dtype 需要对应数值证据与回归测试。

`portable_oscbf/work/qp_solver_health.py` 的接受条件示例：

```python
finite = jnp.all(jnp.isfinite(jnp.concatenate([
    jnp.ravel(solution), jnp.ravel(slack), jnp.ravel(dual),
    jnp.ravel(equality_dual),
])))
accepted = finite & (kkt_residual <= solver_tol)
```

## 测试要求

按影响范围选择验证：

- 文档与简单配置：检查链接、引用路径、占位内容和格式。
- 纯逻辑改动：运行对应 `tests/test_<module>.py` 或 `portable_oscbf/tests/test_<module>.py`。
- ROS 接口或 launch：运行结构测试，并在需要时运行 launch testing。
- 生成文件：运行生成脚本的 `--check` 和相关一致性测试。
- AEB C++ 插件：运行 `bash build_aeb_moveit.sh`，重新加载环境并重启相关 launch。
- 跨主包和控制内核的行为改动：运行 `bash run_all_tests.sh`。

`bash scripts/agent_check.sh` 只提供快速启发式检查，内容包括 compileall、纯模块 import、shell 语法、Git diff 空白检查和少量纯安全测试。它不能代替受影响行为测试或完整测试。

新增测试使用真实纯逻辑对象、真实文件格式库或隔离的本地服务。不得引入 mock、fake、伪造成功结果或仅为通过测试的 workaround。安全断言、阈值、状态码和失败条件不能为测试通过而降低要求。

## 审查清单

- 规则来源是否来自 `docs/CONTEXT.md`、相关 ADR 或公共源码入口。
- 数据流是否覆盖输入、转换、状态保存、输出和诊断。
- 关节身份、顺序、frame、单位、dtype 和时间基准是否在每个边界一致。
- topic、QoS、remap、callback group 和 node 所有权是否明确。
- 生产配置字段能否追踪到来源、override 和最终消费者。
- 真机路径是否仍满足反馈来源、freshness、watchdog、标定和恢复条件。
- 改动是否复用已有模块，并包含防止公共规则分叉的测试。
- 文档中的当前状态是否与代码一致，设计目标是否明确标注。

## 工作区保护

- 保留用户未提交的改动，只修改当前任务涉及的文件。
- 不使用 Git 命令撤销文件内容或重写历史。
- 文件编辑使用明确补丁，不能使用程序脚本批量改写代码。
- 中间文件放在仓库内专用且已忽略的目录，不能写入 `/tmp`。
- 提交、远程推送、发布、真实设备动作和扩大共享范围需要用户明确授权。

## 验证记录

交付说明要区分：

- 已运行并通过的命令；
- 仅通过源码检查得到的结论；
- 因环境缺失而未运行的检查。

性能、安全或真机结论需要保留 commit、日期、环境、配置、完整命令和退出码。历史报告中的测试数量和结论不能替代当前运行结果。
