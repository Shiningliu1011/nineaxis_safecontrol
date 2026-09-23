## 机器人

**ninezzhou**:
本项目使用的 9-DOF 串联冗余机械臂，运动学拓扑 1P8R：第 1 轴为固定工作台上的直线
导轨/丝杠移动模组（移动关节，位移 d1，滑台沿导轨平移；base_link 固定于工作环境，
直线轴运动显式表示为关节变量 q1，而非基座坐标系运动）；滑台之上的 8 个旋转关节
（J2–J9）组成主臂与腕部。结构分层：直线移动基座 + 主臂串联关节 + 腕部关节组 +
末端执行器接口。
**base_link（Y-up）**: 用户规定的机架基坐标系，沿用仓库 legacy Y-up 约定——+Y 竖直
向上（J2 肩部 origin (0, 0.343, 0)、轨迹偏移/工作空间/相机假定姿态均以此为准），
+Z 为 J1 直线导轨方向（水平），+X 横向；J1 沿该系 Z 轴滑动、base_link 本身固定。
感知/控制器/MoveIt/POE FK 全链路共用此系；MuJoCo viewer 经 display_frame 旋
转（Y-up→Z-up）仅用于显示。
_Avoid_: 机械臂、robot arm

**世界坐标系（base_link 固定环境系）**: 05B 定案——本项目的「世界坐标系」就是
base_link 本身：基座固定且 J1 直线导轨在链内显式建模，base_link 不随任何关节运动，
等价于任何固定环境系；感知融合、控制器、MoveIt、POE FK 全链路以此为唯一规范系，
不再另设独立固定世界系。相机/激光雷达均为臂外固定安装，其静态外参
（sensor_frame → base_link）是常量。
_Avoid_: world_frame（独立 env 帧）、map、odom

**标定 provenance**: 每个传感器静态外参的来源记录——方法（如 FAST-Calib2 /
AX=YB）、日期、操作者、残差与误差估计。外参进入 `config/sensor_extrinsics.yaml`
时必须携带 provenance（含 `calibrated`、`sensor_serial`、`calibration_id`、
`timestamp`、`operator`、`residual`、`error_estimate`、`frame_from`、
`frame_to`）；该文件是**唯一 calibration authority**（ADR 0004，标定工具写它、
bridge/launch 读它、T6 校验「被校验 record == runtime 加载 record（ID/hash）」），
`perception_runtime.yaml` 不持有标定矩阵（T7 落地后）。启动期自检分两组——
数学合法性（元素有限/R^T R ≈ I/det≈1/bottom row/translation 机械范围，
**非单位阵不是判据**——identity 可以是合法外参）与标定状态（provenance 完整 +
与记录比对 ≤20mm/5° + 已知物体验收）——任一不通过则拒绝进入避障模式。
_Avoid_: 外参来源、calibration record
