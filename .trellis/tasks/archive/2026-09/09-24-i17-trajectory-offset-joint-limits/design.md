# 技术设计

## 轨迹入口

`TransitionExecutor` 直接以轨迹文件、采样数量与步长调用 `task_target.load_first_task_target`。姿态计算继续调用统一的标定轨迹加载函数。删除两个未读取的 `offset_m` 形参与两个节点的旧参数声明；MuJoCo viewer 继续显示统一标定后的路径。

修改前后使用同一份 `data/nurbs/ik_input.mat`，比较完整标定轨迹、采样轨迹和时间序列。比较使用逐值相等，保存修改前的本地数据作为基准；首目标与采样轨迹起点的一致性由现有测试检查。

## 关节限幅

删除 `robot_params.yaml` 与 `nineaxis.yaml` 的 `joint_limits` 段。`portable_oscbf/work/kinematics_data.py` 仍由 URDF 生成，供 `NineaxisKinematics` 和 `NineaxisManipulatorJAX` 读取位置限幅；`actuator_modules.yaml` 仍供 `ActuatorLimitProfile` 读取速度与加速度限幅。

轨迹配置加载函数检查 `nineaxis.yaml` 的 `joint_names` 与内核执行器配置采用的 `JOINT_NAMES` 顺序相同，并拒绝重新加入手写的 `joint_limits`。ROS 适配层和内核共用该加载函数。相关测试检查真实配置、项目公共关节顺序、URDF 生成结果与加载行为。

## 范围

`models/ninezzhou_moveit_config/config/joint_limits.yaml` 是 MoveIt 配置，继续保留。当前工作不调整控制增益、关节限幅数值、硬件模式或其他执行票。
