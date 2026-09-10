# [T0] spec 与参数文件坐标系/术语修订 (impl)

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/20
- 日期：2026-09-09；执行者 Codex；认领给 Shiningliu1011。
- 状态：2026-09-11 已应用补丁并提交 `83545ea`；验证与审查结果见文末。下文 2026-09-09 记录保留为历史交接。

## 起点与证据

用户指令：“[$wayfinder] 继续下一个ticket”。继承 [SocketCAN 交接](13-socketcan.md)，按地图指定的 DECISION-WINDOW-ORDER 顺序选取本票；本票无开放 blocker、认领前无 assignee。前票仍开放，不将执行链视为通过。

本机 main 为 `659da6c6db598abddc272ad0941b72f77793211b`；本次远端查询见 `output/audit-20/remote-main.txt`。保留原 MAP.md 修改、handoffs、.scratch/oscbf-reuse-wayfinder 和大然电机资料。证据目录 `output/audit-20/` 被忽略，包含起点、远端 SHA、原票正文/评论、tracked 文件术语检索；跨机器需另行复制。

## 核验结果

- `docs/specs/dual_sensor_perception_fusion_spec.md` 用户故事19（76行）、坐标系与TF（194行起）、Further Notes（285行起）已正确描述固定 base_link、Y-up、J1 链内直线导轨及 ADR 0002。无需重做三处修订。
- `config/perception_runtime.yaml` 13–23行已声明 base_link 固定、相机外参 ASSUMED/无 provenance、SSOT 尚待接线。没有依据改动矩阵或参数默认值。
- `git grep -n -E '升降轴|升降台|lift axis|lifting axis'` 找到唯一仍用旧名称描述当前 J1 的有效规格：`docs/specs/real_robot_landing_spec.md:40`。其余命中是 `.scratch` 历史审计、ADR 的错误引用或 spec 明示作废记录；机械替换会篡改历史论证，不属于修订当前语义。
- runtime YAML 27–28行声称使用 profile 启用双源，但 `perception_dual_sensor_real.yaml` 默认 `source_topic_lidar: ""`，且头注明确仍为单相机。补丁只澄清加载模板不等于启用。
- `perception_bridge.py:263` 起仍声明独立静态矩阵参数，当前 SSOT 目标不能写成已完成。接线仍交给后续标定票。
- 继承已有九轴/5D 工具轴路径跟随决议，不引入6D姿态或时间轨迹变更。本票不修改任务阈值或安全预算。

## 决定与复用

无新增用户取舍。沿用已接受的固定 base_link/Y-up、标定权威及九轴/5D 语义。开源版本/许可证选型不适用：本票是文档归档，不新增算法、依赖或适配器。

## 可审阅改动与验证

[待应用补丁](20-docs-proposed.patch) 包含两处文档/注释修改：J1 名称修正、双源 profile 说明澄清。未应用到产品文件；未提交、合入或部署。

`git apply --check docs/planning/oscbf-reuse/handoffs/20-docs-proposed.patch` 已通过（exit 0）。修改不触及 YAML 数据或代码；本次未运行运行时测试，不能据此宣称运行功能或实机验收通过。

## 编码门与剩余缺口

可以开始本票文档实施，无需重新选择坐标系：应用补丁，核对 diff 与参数值不变，复查有效规格用词，提交并完成原票要求的 code review 后再关闭。本票本身不授权变更外参值、TF 路径、默认开启 LiDAR 或改运行代码。

按照地图的新窗口约定，本窗口默认研究/决策，交接后等待实施指令。原票 Done when 要求“已提交并通过 code review”，本次未满足，不发布虚假 resolution，不追加地图 Decisions so far。

## 回退与下一窗口

本窗口只新增本交接、提案补丁和 ignored 证据。回退仅移除这些新资产，保留起点用户文件。若进入实施则按本票范围完成；若用户继续规划队列，下一项是 [标定 SSOT 接线: sensor_extrinsics.yaml 唯一真源](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27)，其原生依赖仍需本票完成。本窗口不自动开始下一票。

## 继续实施（2026-09-11）

用户明确要求“继续执行 ticket#20”，本轮据此前已定范围实施。基点为
`3b47f76d08af1b2544febc256fcb53ac373c0c8b`，产品文字修订提交为 `83545ea`。

- 应用既有补丁：将有效规格中的 J1 旧称修订为直线导轨；澄清双源模板加载本身不会开启 LiDAR。
- 再核对感知 spec 的用户故事19、坐标系与 TF、Further Notes，以及 YAML 的固定 base_link、ASSUMED/provenance 说明，既有修订仍满足要求。
- `git diff 3b47f76...83545ea --check` 通过；PyYAML 对比提交前后解析结果完全相同，参数值未变。
- tracked 文档与配置的旧称检索只剩明确作废引文、ADR 历史论证和本票历史记录。保留这些记录。
- 仅修改文字及注释，未运行运行时或硬件测试；本票不产生 SSOT 接线或实机准入证据。

### Code review

两名独立审查 agent 按 code-review 技能审查 `git diff 3b47f76...83545ea`：

- Standards：0 项发现；符合 ADR 0002/0004，无适用代码异味。
- Spec：0 项发现；原票三处规格及 YAML 注释要求均满足，无遗漏、错误实现或范围扩张。

提交与审查门已满足。修订及交接发布到 `codex/ticket20-docs` 分支供读取，
不将本机提交和远端 main 合入混为一谈。后续标定 SSOT 接线仍按原票单独实施。
