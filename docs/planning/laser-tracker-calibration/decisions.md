# 决定与依赖

本页列出本地准备资料中的研究问题和待决定事项。任务标题用于阅读与引用；文件路径表示本地草稿身份。正式 GitHub 任务身份仍由项目 tracker 管理。

## 已确认的任务条件

- 目标为完整准备方案，覆盖现场采集、离线求解和独立验收。来源：2026-10-26 当前会话用户答复。
- 激光跟踪仪、靶球及安装座、测量数据均未具备。来源：用户对现场条件问题回答“都没有”。
- 测量对象拟安装在机械臂末端；具体球心位置和安装重复性需要测量。来源：引用会话最后一条用户描述。
- 定位精度数值和作业范围尚未确认。

## 研究与决定草稿

| 事项 | 类型 | 前置事项 | 当前状态 |
|---|---|---|---|
| [现有项目怎样提供标定所需的关节与模型数据](tickets/project-interfaces.md) | research | 无 | 研究完成 |
| [单靶球测量能够确定哪些零位参数](tickets/measurement-observability.md) | research | 无 | 研究完成 |
| [定位精度与适用范围怎样验收](tickets/accuracy-and-workspace.md) | grilling | 无 | 等待用户要求 |
| [取得设备能力与安装几何资料](tickets/equipment-evidence.md) | task / HITL | 无 | 等待设备资料 |
| [怎样建立独立基准并安装靶球](tickets/reference-and-target.md) | grilling | 两项研究；设备能力与安装资料；精度与范围 | 等待前置 |
| [怎样组织采样与原始数据](tickets/dataset-and-sampling.md) | grilling | 独立基准与靶球方案 | 等待前置 |
| [怎样求解、验收并交付标定参数](tickets/solver-and-validation.md) | grilling | 采样与数据方案 | 等待前置 |

领取和完成研究的记录保存在对应研究问题的 Resolution；需要用户决定的事项由用户答复后更新。

## 正式项目接续

- [M02 九轴模型、TCP 与标定接口（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/136)：承接关节坐标、模型身份、TCP、FK/Jacobian 与标定输入。
- [完成 TCP、工件标定与伺服时序测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/165)：承接实际测量、重复性、原始记录和独立验收。
- 正式归属和依赖见[完整项目资料](../nineaxis-full-project/README.md)。本地准备完成的范围以各文件记录为准。
