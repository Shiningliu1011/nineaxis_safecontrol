# 全系统以 base_link（Y-up 机架固定系）为规范世界坐标系，放弃独立固定世界系

2026-09 定案（05B）：九轴机械臂为 1P8R 拓扑——J1 是固定工作台上的直线导轨/丝杠
移动模组（移动关节 q1，滑台沿导轨平移），J2–J9 为滑台之上的旋转主臂与腕部；
`base_link` 固定在机架/工作环境中，不随任何关节运动。相机与激光雷达均安装于
臂外固定支架，其静态外参（sensor_frame → base_link）为常量、不随时间变化。因此
`base_link` 在本项目中就是一个固定环境坐标系，与任何新设的固定世界系在数学上
等价。我们决定以 `base_link` 作为全系统唯一规范坐标系：感知融合、控制器
（OSCBF/CBF）、MoveIt、POE FK 全链路共用，视觉显示层（MuJoCo viewer）经
`display_frame` 旋转（Y-up → Z-up）仅用于显示。

**Considered Options**：A（本决定）全系统 base_link——感知、控制、几何全在
base_link 中表达，无任何跨系变换，与现有代码/测试/参数零迁移量，软件内部参考系
自洽（注：软件内部自洽不等于物理标定精度，后者另见 ADR 0003 与 05A 结论）；
B 全系统 fixed-world——感知、FK 与 CBF 全部表达在独立环境系（map/workcell/
world，repo 中不存在该帧，需先定义物理基准并做基座→env 标定），被否：需定义
新帧 + 整链路（内核/调用方/感知）迁移，固定基座下与 A 数学等价、短期收益为零；
C 感知 fixed-world + 控制器/CBF base_link（混合）——perception 以固定环境系为
canonical persistence frame（三层占据/ESDF/track 关联在其内做），OSCBF/POE/
碰撞几何留在 base_link，感知→控制器边界用**带 timestamp 的显式 transform** 转换
obstacle position/velocity——**保留为演进路径记录**（未来若 base_link 变为移动
基座/外部工装对齐需求出现时启用），当前不实现。

**Consequences**：规格中「world_frame 是固定环境坐标系、不是 base_link」与
「base_link 随升降轴运动」等表述错误，须修订（T0）；`base_link` 语义正式定义为
用户规定的 Y-up 机架系（+Y 竖直向上、+Z 直线导轨方向、+X 横向），J1 沿该系 Z 轴
滑动、系本身固定；`/perception/tracks` 现为无 frame_id/时间戳的隐式
Float32MultiArray 契约，升级为显式障碍物观测消息（timestamp + frame_id）列入
T1 后备；B/C 候选保留为文档记录，不再作为实现路径。
