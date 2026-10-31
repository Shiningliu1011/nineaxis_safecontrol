# 完整地图的模块任务组织

用户确认采用本轮 wayfinder 建议，并要求当前专项规格在各自范围内优先。授权范围为 GitHub 地图、任务正文、验收和原生前置，以及相应本地规划维护。

## 需求与负责对象

[完整项目地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/147)共用实施任务。[碰撞与双传感器规格](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/196)和[模块边界与清理规格](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/197)分别保存正文，原生依赖连接具体实现和验收能力。

| 新增任务 | 负责内容 |
|---|---|
| [OBB 几何资产与完整覆盖](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/198) | 原始几何覆盖、拓扑、工具附件、外扩与路径可通过性 |
| [凸体 DCOL 后端及导数](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/199) | 共同缩放、有效导数、毫米诊断、数值与后端依据 |
| [双传感器 PCL/OctoMap 场景](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/200) | 独立来源和时间、完整占据、有效覆盖及唯一场景发布 |
| [共同模型、限制与连续参考](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/201) | 当前消费者接入及重复计算清理 |
| [共同入口、配置与演示](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/202) | 实际可用方法、共同装配及替换后的业务入口清理 |
| [独立 benchmark 清理](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/203) | 专用近似能力及入口清理，正式 AEB-RRT* 继续保留 |
| [模块边界、复用与清理验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/204) | 模块规格 R01–R10、T01–T09 的完整证据 |
| [在线双传感器与丝杠障碍](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/205) | 实际来源、基座标定、同步及装置独立反馈证据 |

查询、连续证明、数值验收、规划、CBF、状态/进度、公共运动、QP、Runtime、记录评价、来源切换和文档继续由对应原任务负责。完整责任表见[需求覆盖](../../../../docs/planning/nineaxis-full-project/coverage.md)，全部前置见[依赖表](../../../../docs/planning/nineaxis-full-project/dependencies.md)。

公共运动及当前 Ruckig 功能集中在[仿真执行与最终准入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)。共同 v/a/jerk 沿用原确认，取消 Ruckig 需要模块规格 I08 的全部证据。公共前视和碰撞仿真在已具备的算法上验收，完整轨迹优化、逐点方法和 MPC 在各自任务内验证接入。

## 实际核验

- 读取全部任务及原生前置，40 项开放任务的正文采用当前专项要求；新增 8 项任务均已关联原地图。
- 67 项任务、239 条直接前置，48 项开放、19 项已完成；原有状态、指派、标签保留。
- 48 项更新或新增正文和地图全文经过远程回读，与准备文件一致。两份专项规格正文保持原文。
- 原生前置逐项匹配计划，图无循环，开放阻塞摘要与实际前置状态一致。
- 模块整合与在线双源验收均未直接或间接等待完整 MPC、完整逐点比较、三方法统计或专项规格关闭。
- 当前无开放前置且未指派的任务为 OBB 覆盖、双源场景、公共复用和 benchmark 清理。

核验使用实际 `gh api` 读取和本地 JSON 解析，没有以准备文件代替远程状态。`.scratch/module-map-update/` 保存修改前记录、正文、计划、原生操作记录、最终子任务、全部前置及 `validation.json`。

本轮没有修改功能源码，没有执行算法测试、仿真或设备动作。相关运行能力由实施和验收任务分别取得证据。
