# 准备工作与验证计划

## 本轮准备

- [x] 读取引用会话、项目指令与标定相关源码。
- [x] 确认准备方案的完成目标以及设备和数据现状。
- [x] 建立本地准备入口、研究问题和后续决定的依赖关系。
- [x] 编写现场条件、数据要求、求解及独立验收的讨论资料。
- [x] 完成测量可辨识性研究，引用一手来源并说明推导条件。
- [x] 完成项目接口研究及名义模型计算，记录实际验证结果。
- [x] 核对文档引用、JSON 元数据、依赖关系及措辞。
- [x] 记录交接状态和尚待用户或现场决定的条件。

## 验证范围

本轮检查本地文档、既有纯计算接口与名义 URDF 几何。现场标定结果需要真实测量和独立验收。

检查脚本和中间输出使用已被 Git 忽略的 `.scratch/laser-tracker-calibration/`。研究计算若作为证据交付，保存在本任务 `research/`。具体命令与输出在研究记录和 `research/preparation-validation.md` 中保存。

## 后续继续条件

从[决定与依赖](../../../docs/planning/laser-tracker-calibration/decisions.md)选择前提已满足的事项。用户确认设备、精度、基准和数据方案后，形成可开发的参数集与接口定义；现场采集按正式项目现场任务的条件进行。

## 本地计算准备与跨设备接手

2026-10-27，设备 `lsn`，工作分支 `codex/static-obstacle-experiments`。
项目目录为 `/home/lsn/robot/robot_safecontrol`，验证环境为
`.scratch/host-recovery/venv`，Python 3.10.12、NumPy 1.26.4、SciPy 1.15.3、pymoo 0.6.2。

- POE 正运动学覆盖单个配置、全部连杆和批量配置；独立 URDF 检查通过。
- PSO 逆运动学在关节限位内搜索，位置与姿态分别验收；188 次可达目标求解全部通过。
- 项目输入、展示和报告使用 mm/°；ROS、URDF 与控制计算使用 m/rad，在接口边界转换。
- 3000 步模型闭环最大稳态位置误差为 0.15281 mm，RMS 为 0.11133 mm，满足 ≤0.5 mm 要求；QP 全部通过，53 项相关回归检查通过。
- 逆解的默认位置容差为 0.1 mm、姿态容差约 0.0572958°。闭环精度、模型逆解精度与现场标定精度分别验收。

源码、测试和结果摘要随仓库同步。接口与运动学验证见
[运动学说明](../../../docs/modules/portable-oscbf/kinematics.md)，
闭环误差定义与验证入口见[跟踪评价](../../../docs/tracking_evaluation.md#控制内核位置精度回归)。

三份核心数值记录保存在本机，合计 1,060,513 bytes：

| 仓库相对路径 | bytes | SHA-256 |
|---|---:|---|
| `.scratch/pso-ik/validation-results.json` | 165325 | `ef1a78db82f37855ff37f0e00f822ec4687c17ecf167580ce55f91e22f72a281` |
| `.scratch/tracking-precision/baseline-after/baseline_tracking.npz` | 894552 | `22c6977688078b7b51c83d077feb031321a9955d436de8a3271993f66c999b7d` |
| `.scratch/tracking-precision/baseline-after/baseline_tracking.summary.json` | 636 | `b163b79e2d0377712bb19e4d2afadd20bff4c40bfa9246da7d51dc2b4f4d34c8` |

下一台设备按已提交依赖安装环境，运行相关测试；需要复核本机数值记录时，传输上述文件并核对 SHA-256。设备、靶球安装、真实测量数据和独立基准仍按本任务的规划状态继续确认。
