# OSCBF 移植执行文档（M0–M12）

> 历史阶段计划。当前运行路径和验收结果以现有源码、测试及[文档总目录](../README.md)中的现行说明为准。

> 关联文档: [OSCBF_PORTING_GUIDE.md](OSCBF_PORTING_GUIDE.md)（目标架构/公式/参数权威来源）
> 生成日期: 2026-08-06
> 执行规则: 阶段严格串行；每个阶段**全部验收标准（AC）通过**才算完成；AC 未通过时停在当前阶段、记录偏差，不跳过、不带病进入下一阶段。

---

## 1. 文档目的

把 OSCBF 框架移植大任务拆成 13 个可独立验收的阶段（M0–M12）。每个阶段都采用统一的**目标模式模板**：

| 字段 | 含义 |
|------|------|
| 目标 Goal | 本阶段要交付的结果，SMART 表述（具体、可测、可达、相关、有时限） |
| 前置条件 | 进入本阶段前必须已满足的状态（通常=上一阶段 AC 全过） |
| 任务 Tasks | 为实现目标必须执行的具体动作 |
| 验收标准 AC | 每个标准都是可执行/可测量的断言，注明测试名、命令与阈值 |
| 证据/产出 | 阶段完成后必须落盘的文件、日志、报告 |
| 完成判定 | 判定完成的唯一条件：全部 AC 通过 + 证据齐备 |

---

## 2. 阶段总览与依赖

| 阶段 | 名称 | 依赖 | 一句话目标 | 出口（完成标志） |
|------|------|------|-----------|-----------------|
| M0 | 脚手架与参考基线 | 无 | 参考代码在新仓库可导入、可测试 | 参考 25 测试可执行，失败清单登记，dpax 指向 vendor |
| M1 | 机器人模型移植 | M0 | JAX POE 模型与本仓库 URDF 一致 | FK/Jacobian/限位对照测试全过 |
| M2 | OBB 包络盒标定 | M1 | 生成贴合 STL 的 OBB 模型与碰撞对 | OBB 数据+YAML 生成，零位无假碰撞 |
| M3 | DCOL 热路径 + alpha 校准 | M2 | DCOL 距离/梯度进控制热路径并完成逐对标定 | DCOL vs FCL 对照、alpha YAML、无重编译 |
| M4 | 轨迹接入 | M1 | 当前仓库轨迹加载成弧长路径几何 | PathGeometry 连续、14992 点、<5s |
| M5 | 基线控制内核跑通 | M3+M4 | 原版整块内核跑通闭环并保存基线 | 3000 步 qp_ok=100%、误差收敛、基线 npy 落盘 |
| M6 | 模块化 JIT 重构 | M5 | 整块内核拆为独立 JIT 子函数 | 与 M5 输出一致 <1e-12、cache=1、性能不劣化 |
| M7 | 弹性 QP + 限幅 + DCOL 障碍物 | M6 | relax_cbf=True、控制器内 clip、障碍物 DCOL | δ≈0、qp_fail=0、dyn_min>0、u_nom 限内 |
| M8 | 可操作度零空间 | M7 | 新增可操作度零空间策略替换关节中点 | 梯度 FD 对照、零空间残差、φ 上升 |
| M9 | 环境感知接口 | M7 | 感知链路（ESDF/点云→obs_*/sdf_*）接通 | 接口测试通过、默认 disabled 行为不变 |
| M10 | ROS2 控制节点 | M8+M9 | oscbf_controller 节点可独立运行 | 冒烟测试、JointState 合法、p95<10ms |
| M11 | Launch 集成与端到端 | M10 | 控制节点并入现有 launch 闭环 | launch 测试、端到端轨迹完成、回归不破 |
| M12 | 调优与全量验收 | M11 | 按指南 §10 全量验收 | 验收报告 + LESSONS_LEARNED + pytest 全绿 |

---

## 3. 全局验收指标（指南 §10，M12 最终核对）

| 类别 | 指标 | 阈值 |
|------|------|------|
| 精度 | 最大位置误差 max_ee | < 0.1 mm |
| 精度 | 最大姿态误差 max_oe | < 0.1° |
| 安全 | 最小动态裕度 dyn_min | > 0 mm |
| 安全 | QP 失败 qp_fail | = 0 |
| 安全 | CBF slack δ_slack（正常工况） | ≈ 0（仅约束冲突时启用） |
| 安全 | 速度跳变（>0.1 rad/s）次数 | 0 |
| 安全 | OBB 包络贴合 | 体积比记录，包络空气最小 |
| 安全 | JIT 缓存大小 | = 1（首帧后不重编译） |
| 零空间 | 梯度有限差分对照 | 相对误差 < 容差 |
| 零空间 | 零空间残差 ‖J·qdot_N‖ | 数值精度量级 |
| 零空间 | 可操作度上升 | 固定末端时 φ 上升后稳定 |
| 零空间 | 末端跟踪误差劣化 | 相比 JointCenterPolicy 不明显（<20%） |
| 性能 | 控制周期 p95 | < 10 ms |
| 性能 | 首次 JIT 预热 | < 30 s（单线程 XLA） |

---

## 4. 阶段详述

- [M0：参考基线](oscbf-execution/m00.md)
- [M1：机器人模型](oscbf-execution/m01.md)
- [M2：OBB 标定](oscbf-execution/m02.md)
- [M3：DCOL 与 alpha](oscbf-execution/m03.md)
- [M4：轨迹接入](oscbf-execution/m04.md)
- [M5：控制内核](oscbf-execution/m05.md)
- [M6：模块化 JIT](oscbf-execution/m06.md)
- [M7：QP 与障碍物](oscbf-execution/m07.md)
- [M8：零空间](oscbf-execution/m08.md)
- [M9：环境感知](oscbf-execution/m09.md)
- [M10：ROS 2 控制节点](oscbf-execution/m10.md)
- [M11：Launch 集成](oscbf-execution/m11.md)
- [M12：调优与验收](oscbf-execution/m12.md)

[假设、默认值与变更记录](oscbf-execution/assumptions-and-history.md)按需查阅。
