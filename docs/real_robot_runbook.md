# 实机运行手册（Runbook）

> 日期：2026-08-22
> 适用：robot_safecontrol 真机落地（H2–H7 阶段）
> 安全规则：首次联机必须单关节；软件限幅必须先于实机；急停链路必须在运动前验证。

## 0. 上电前检查（每次实验必做）

| 检查项 | 预期 | 未达标处理 |
|--------|------|-----------|
| 电源电压（24V/36V） | 在规格范围内 | 不得上电 |
| 急停按钮位置 | 实验人员可快速触及 | 调整位置后再开始 |
| 急停链路测试 | 按下急停 → 电机断使能/断电 | 排查急停链路后再开始 |
| CAN 适配器连接 | CANable 插入、can0 可见（`ip link show can0`） | 重新插拔/检查驱动 |
| CAN 波特率 | 1 Mbps（`ip -d link show can0` 确认） | `sudo ip link set can0 type can bitrate 1000000` |
| 机械臂工作空间 | 无人员/障碍物在内 | 清空后再开始 |
| 末端工具 | 已安装且固定 | 检查螺丝/夹具 |

## 1. 急停试验（每次实验必做）

```
1. 启动系统（见 §2）
2. 手动触发急停按钮
3. 确认：电机立即停止运动（断使能/断电）
4. 确认：软件端检测到 estop_active → 锁存停车原因
5. 恢复急停 → 人工确认 → 系统复位
6. 记录：急停延迟（从触发到完全停止的时间）
```

**停止条件**：急停不生效（电机继续运动）→ 立即断电，排查链路。

## 2. 系统启动

### 2.1 仿真模式（默认，安全）

```bash
bash build_aeb_moveit.sh
source install/setup.bash
bash run_demo.sh
```

### 2.2 真机 shadow 模式（只读验证）

```bash
bash build_aeb_moveit.sh
source install/setup.bash
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=shadow start_oscbf_plant:=false
```

### 2.3 真机 live 模式（发送命令）

```bash
bash build_aeb_moveit.sh
source install/setup.bash
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false
```

### 2.4 带感知的真机模式

```bash
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false start_perception:=true
```

## 3. 只读反馈验证（H3 前置）

**目标**：确认 9 路编码器反馈正确（单位、方向、零位）。

```
1. 启动 shadow 模式（§2.2）
2. 手动盘车各关节，观察 RViz /mujoco_joint_states：
   - J1 升降：位置值随升降变化
   - J2-J9 旋转：位置值随旋转变化
   - 正方向与 URDF 一致（对比仿真 FK）
3. 检查日志：无 axis_error、无丢帧
4. 连续运行 10 分钟：无异常
```

**停止条件**：方向反了（与 URDF 不一致）→ 需重新标定零位（§5）。

## 4. 单关节低速测试（H2）

**目标**：单轴低速正弦跟踪，误差 <1mm（线性）/ <0.1°（旋转）。

```
1. 确认急停链路（§1）
2. 启动 live 模式，仅使能一个轴（其余断使能）
3. 发送低速正弦命令（amplitude=5°, period=2s）
4. 观察：跟踪误差、丢帧、故障码
5. 记录：误差曲线、急停触发次数
```

**停止条件**：误差超标 / 急停触发 / 故障码 → 停止并排查。

## 5. 零位标定（首次联机必做）

```bash
# dry-run 查看流程
python3 scripts/calibrate_zero.py --dry-run

# 实际标定（需 CANable + 电机上电）
python3 scripts/calibrate_zero.py --interface can0
```

标定产物：`config/hardware_joint_zero.yaml`（sign + zero_offset_deg）。

## 6. 全臂 shadow 验证（H3）

**目标**：9 路反馈延迟 <5ms、丢帧 <0.1%。

```
1. 启动 shadow 模式
2. 手动盘车各关节，观察日志：
   - CANBusMetrics.latency_p99 < 5ms
   - CANBusMetrics.loss_rate < 0.001
3. 连续运行 10 分钟：无异常
```

## 7. 低速全链闭环（H4）

**目标**：10% 速度跑通过渡+跟踪，末端误差 <2mm。

```
1. 确认急停链路（§1）
2. 确认工作空间无障碍物
3. 启动 live 模式
4. 等待过渡规划完成、跟踪开始
5. 观察：qp_fail=0、无急停触发
6. 记录：末端误差曲线、制动距离
```

**停止条件**：qp_fail / 急停 / 误差超标 → 停止并排查。

## 8. 传感器标定（H6 前置）

```
1. 固定 Orbbec Gemini 335L（三脚架/支架）
2. 测量相机到 base_link 的 x/y/z + roll/pitch/yaw
3. 填入 config/sensor_extrinsics.yaml
4. 启动感知（§2.4）
5. 验收：已知尺寸物体放在 base_link 下已知位置，点云中物体位置误差 <20mm
```

**停止条件**：误差 >20mm → 重新测量外参。

## 9. 动态避障实验（H6）

**目标**：软质泡沫球低速避障，无碰撞、可急停。

```
1. 确认传感器标定（§8）
2. 确认急停链路（§1）
3. 确认隔离区（人员不进入工作空间）
4. 启动 live + 感知模式（§2.4）
5. 低速运行（仿真速度 10%）
6. 手持/滑轨移动软质泡沫球进入工作空间
7. 观察：避障反应、dyn_min>0、qp_fail=0
8. 记录：避障日志、急停触发次数
```

**停止条件**：碰撞 / 急停失效 / 无法观察避障反应 → 停止并排查。

## 10. 故障恢复

| 故障 | 恢复步骤 |
|------|---------|
| 急停触发 | 按下急停 → 确认停止 → 恢复急停 → 人工确认 → 重启节点 |
| CAN 丢帧 | 检查布线/终端电阻 → 重启节点 |
| 驱动故障码 | clear_error → 重新使能 → 若仍故障则断电排查 |
| 软件锁存 | acknowledge_stop → 确认健康 → 恢复 |
| J1 传动未标定 | 补齐传动参数 → 重启节点 |

## 11. 实验记录模板

```
日期：
实验阶段：H2 / H3 / H4 / H5 / H6
操作人员：
机械臂状态：正常 / 异常（描述）
急停测试：通过 / 未通过
CAN 延迟 p99：___ ms
丢帧率：___
跟踪误差：___ mm
急停触发次数：___
备注：
```
