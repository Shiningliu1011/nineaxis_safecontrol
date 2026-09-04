# 11 — 真机执行链路:SocketCAN 后端与参数注入

**What to build:** `hardware_bridge` 目前不读任何 ROS 参数,backend 从不注入 →
恒为 shadow 且不发状态;SocketCAN 后端 `NotImplementedError`(CANable 硬件未到货)。
需将真机链路推进到「可下真机验证」的状态:

1. 实现 DrEmpower SocketCAN 后端(帧编解码、启停、错误计数),接口与现有
   shadow 后端一致;
2. hardware_bridge 参数化:backend 选择 / CAN 设备 / 关节映射 / 使能门;
3. 验证路径:无硬件时 socketcan 后端显式报错而不是静默 shadow;
   有硬件后做 1Hz 慢速使能-回读-停止冒烟。
4. 与 `CommandSafetyGate`(见 .scratch/real-robot-landing/issues/03)接口核对。

**Blocked by:** 硬件验证部分被 CANable 到货阻塞;代码部分可先行(仿真 socketcan:
`vcan` 可用)。

**Queue:** implementation-backlog — 暂移出 Wayfinder(硬件条件不足以形成完整决策)
**Tracker:** #13 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)

**Status:** blocked(硬件);若先行,仅取 vcan 代码部分拆小票

- [ ] SocketCAN 后端实现 + vcan 单测
- [ ] hardware_bridge 参数化 + 后端注入测试
- [ ] 无硬件路径显式错误(不再静默 shadow)
- [ ] 硬件冒烟脚本(待 CANable 到货执行,标注 blocked)
