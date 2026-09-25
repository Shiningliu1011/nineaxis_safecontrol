# CollisionSafety 接口审查

基准：`main` 的 `d5a1ac1050aa0140195ea32f923550030e90669c`。本次审查包含新增接口、接口测试、模块文档和 Trellis 任务文件。`docs/agents/issue-tracker.md` 是开始本票前已有的用户改动，未纳入本票修改。

## 项目规范与代码质量

- `portable_oscbf/work/collision_safety.py` 使用生成的 `N_JOINTS`，没有 ROS import；新增运行依赖仍是现有的 JAX、NumPy。
- 配置在构造时校验，JAX x64 已启用；固定 shape、dtype 和有效 mask 保持 JIT 数据结构稳定。非法输入在公开方法边界拒绝，未计算的数值不会获得有效标志。
- 文档位于 `docs/modules/portable-oscbf/`，并由模块索引链接。没有改动生产入口、硬件边界或现有碰撞权限。
- 仓库没有配置可执行的 Python lint 或静态类型检查命令；已运行 `compileall`、`git diff --check`、项目快速检查与测试。

## 票据与规格

- `CollisionSafety` 仅有 `prepare_scene`、`query`、`certify` 三个公开操作；固定结果包含全部规定的 header、OSCBF 行、毫米距离和区间证明字段。
- `prepare_scene` 在 JAX 编译路径把 `float32` support 坐标转换为 `float64`；查询与区间证明的 typed result 可作为 PyTree 通过 JIT。
- 九个公开状态均有接口行为测试；非法 shape、dtype、非有限数值、未知模式、身份不符、deadline 与无 ROS 进程调用均有测试。
- 当前没有真实碰撞 kernel 或连续区间证明，`query`、`certify` 返回明确的非 `OK` 状态并关闭全部准入 mask。这与本票已确认的处理方式一致。

## 验证与限制

- 接口测试 22 项通过；控制内核测试 200 项通过、34 项跳过；快速检查通过，其中纯逻辑测试 69 项通过。
- 公开 `query`、`certify` 在 host 完成输入校验与实际计时，计算路径由后续票填入私有 JAX kernel。当前结果不能授权命令或轨迹。
- 本任务按 Codex inline 模式由主会话检查两个审查维度，没有安排独立审查者。未发现阻碍本票验收的新增问题。
