# 正式容量公共参数比较

本次基于 `0393614`，修改限定在工程容量模块、相关纯配置测试与文档。
`capacity_common_control` 验证配置属于 A–E 方法组合，然后仅排除
`solver_mode`、`recovery.mode` 和既有进给档位字段 `nominal_feed_multiplier`。
生命周期证据与正式准入调用同一函数。控制周期、全部求解权重、构型辅助、
恢复阈值和其余字段逐值保留；传入配置不变。生成器仍可使用 D 作为公共
参数来源，正式配置继续保存 separated/basic，运行按既有 A–E override 执行。

## 实际输入与验证

环境为主仓库 `.scratch/host-recovery/env.sh`，PYTHONPATH 指向
`static-capacity` worktree；本次检查使用 CPU 16–21，没有启动 sim 或性能测量。

实际 RED 调用了 `require_engineering_evidence`，输入为既有三卷工程证据与
依照 `freeze_formal` 规则保存的 S0 配置，返回公共参数不一致。输入保存于
`output/static-capacity-20261020/formal-control-red-01/`。

GREEN 使用相同真实证据逐个检查 A–E，公共参数阶段全部接受；有意移除一项
审计身份后，后续审计身份检查仍拒绝。改变 `confirmation_s` 时，公共参数
阶段直接拒绝。该检查未执行完整工程证据重建，不代表正式容量已准入。
结果保存于主仓库
`output/static-capacity-20261020/formal-control-green-01/validation.json`，SHA-256
`d2a26f916c957d5d34d390a87970ad5b45b700b324c4d222a00497a684b427c1`。

定向测试覆盖真实 `ExperimentConfig` 的五种方法、三档进给、全部 17 个
其他公共参数的变化拒绝、不修改输入，以及 1,080 项矩阵和每种方法 216 项。
实际结果为 35 项通过、2 项未选中，162.96 秒；两个受影响 Python 文件
`py_compile` 和 `git diff --check` 通过。

日志位于独立 worktree 的 `.scratch/`：

- `formal-capacity-control-red-01.log`：实际正式比较 RED；SHA-256
  `73e1bea6246dbe672023027511a63184b8fd9b8392226c543f9eadcfc60e08b3`。
- `formal-capacity-control-red-02.log`：新公共比较入口 RED。
- `formal-capacity-control-green-01.log`：纯配置、容量、矩阵及来源相关测试。
- `formal-capacity-control-green-02.log`：真实证据的 A–E 准入阶段与来源兼容验证；
  SHA-256 `010228214b8dfe772ded8858ddfe590b2e3358143d494b9ec2ed51260e427764`。

复现入口为 `.scratch/reproduce_capacity_formal_control.py` 和
`.scratch/verify_capacity_formal_control.py`；输入原件及归档继续保留。

## 来源影响

实际调用既有 `execution_source_compatibility` 返回 pass，唯一变化为
`src/robot_safecontrol_moveit/static_experiment_engineering_capacity.py`，该文件
原本就在 `BATCH_SOURCES`。兼容白名单未修改；生命周期 AST 前后均为
`5097e9aea56c782315716a73dfcf3f33c39f5e6925b35ef2fc909bdc9cb48a5c`。
对照归档 SHA-256 为
`098531f782e90f1a529a210f0efbe1fd8163ef0b125f2d84a7afbb08e4715a32`。

recording、controller、plant、控制求解及容量运行守卫内容均保持。本次比较
修复无需增加三卷 FS 或生命周期重跑；前轮完整计时改动的既定复验继续执行。
工程证据与正式容量清单仍须在最终配置和现场空间确定后重新生成并绑定来源。
