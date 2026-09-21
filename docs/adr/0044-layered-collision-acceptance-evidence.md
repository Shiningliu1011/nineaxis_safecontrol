---
status: accepted
date: 2026-09-21
---

# 碰撞模块切换要求分层证据全部通过

新 CollisionSafety Module 取得命令权限前必须通过以下全部证据层：

1. 几何 kernel：解析用例、独立高精度参考、接触附近、相交状态、极端长宽比、gradient 与
   solver residual；
2. 机器人几何 artifact：collision mesh 四面体覆盖证明、mesh hash、`geometry_hash`、允许
   接触列表和固定槽位身份；
3. 感知与动态场景：保留原始时间、标定身份和关节状态的真实 LiDAR 回放，覆盖 self-filter、
   occupied/free/unknown、遮挡、运动、tracking 中断、过期数据和容量边界；
4. 规划与 OSCBF：规划边、轨迹区间、恢复轨迹、动态接近、重新规划、revision 更新和 QP
   不可行；
5. 故障注入：identity、checksum、revision、共享内存、solver、unknown、capacity、constraint
   和 deadline 失败；
6. 运行时间：在目标计算设备记录 scene preparation、JAX transfer、collision query、QP 与
   完整 100 Hz 路径的最大值和 `p99.9`；
7. 独立验收数据：参数调整数据与最终 LiDAR 回放、任务场景分离，验收开始后禁止修改阈值。

每项容差、deadline、数据 identity、通过条件和随机种子必须在执行验收前写入版本化验收规范。
任一必需项失败都会阻止切换。证据 manifest 保存代码 revision、配置身份、geometry identity、
kernel identity、数据 hash、目标设备信息和每项结果。

现有 MoveIt/FCL、OBB 与球体结果只作为 shadow 差异诊断，不属于正确性参考。正确性依据由
解析几何、独立高精度计算、覆盖证明和区间证明提供。

## Decision Basis

碰撞模块同时跨越几何、感知、规划、控制、进程通信和运行时间边界，单一测试类别不能覆盖
全部授权条件。分层证据让每个边界都有独立且可重复的判断依据。

## Consequences

实现工作必须同时提供验收规范、数据 manifest、证据生成入口和结果归档格式。缺少目标计算
设备或真实 LiDAR 验收数据时可以继续开发与仿真验证，但不能完成生产切换。
