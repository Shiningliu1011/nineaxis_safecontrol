# [T5] 标定工具链：复用现有工具 + AX=YB 输出

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25
- 日期：2026-09-08；执行者/认领：Shiningliu1011。
- 状态：本地离线方向核验与上游研究完成；尚未实施，不满足关闭条件。
- 已同步[阶段进度评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25#issuecomment-5586670600)，不是 resolution；本票保持 OPEN。

## 起点与范围

起点 main，commit `659da6c6db598abddc272ad0941b72f77793211b`。保留已有 ADR 0004、MAP、感知规格修改，以及未跟踪 handoffs、research/calibration-status-interface.md、.scratch 与大然电机资料；没有重置、覆盖或提交这些内容。

上一窗口[标定 SSOT 接线交接](27-calibration-ssot.md)指向本票。GitHub 本票仍由该 SSOT 票原生阻塞；按 DECISION-WINDOW-ORDER 的“前置不足时完成不依赖缺失证据的工作”规则，先认领并研究工具/离线契约，不删除阻塞。上次交接日期记为 2026-09-09，本机当前日期为 2026-09-08；沿用其已记录决定，不推断额外事件顺序。

本轮默认规划，不自动实施产品代码或运行实机。

## 本地事实与证据

- `config/sensor_extrinsics.yaml` 仍是两个单位阵占位，旧 provenance 键不足；文件 SHA256 `b8327f546f4febdfa37ed06b644599c9513fa65eed427a500846ac0522dfedd4`。单位阵的数学合法性不等于标定完成。
- 本地配置声明 Gemini 335L、Mid-360S，但不作为实物型号/序列号/固件核验。
- 本轮对 data、docs、.scratch 的文件名及 scripts/tests 文本盘点没有找到本票可用的配对标定数据或传感器外参求解入口；`scripts/calibrate_zero.py` 是关节零位工具。未全盘扫描主机，不宣称其他路径不存在数据。
- `/usr/bin/python3` 的系统 OpenCV 为 4.5.4，模块 `/usr/lib/python3/dist-packages/cv2.cpython-310-x86_64-linux-gnu.so`，已有 `calibrateRobotWorldHandEye`。
- 隔离验证脚本：[`check_ax_yb.py`](../../../../.scratch/oscbf-reuse-wayfinder/25/check_ax_yb.py)，SHA256 `0bae562a439c450580887bf80d5aed3030279be119572ac09416c7e25fc109b1`。
- 命令：在仓库根目录运行 `python3 .scratch/oscbf-reuse-wayfinder/25/check_ax_yb.py`，退出码 0；[完整结果](../../../../.scratch/oscbf-reuse-wayfinder/25/ax-yb-result.json)，SHA256 `0c9dfa19b4d5bb3d8a097e67081836d2e814854603e3e8f5d4001f504e4075f1`。
- seed=25，24 组无噪声合成位姿；SHAH/LI 输出外参相对真值的最大矩阵元素绝对误差分别约 9.85e-16 / 2.00e-15；闭合误差约 1.83e-15 / 4.88e-15。这不是米级误差指标或现场精度证明，未测噪声、退化、真实FK和数据同步。

## 离线输入输出建议

统一记号 `T_dst_src` 表示把 src 点变到 dst，平移用 m、旋转矩阵无量纲；不要把 API 参数名直接当成本项目物理帧。

固定相机、末端刚性标定板场景：

- `A_i = T_camera_target(i)`：对应内参/畸变模型的板观测位姿。
- `B_i = T_base_gripper(i)`：同一采集时刻机器人 FK，含 J1 位移，保留固定 base_link Y-up。
- 未知 `X = T_target_gripper`、`Y = T_camera_base`，满足 `A_i X = Y B_i`。
- 将 A、B 分别作为 OpenCV 两组输入，输出按 X、Y 解释；相机 SSOT 为 `inverse(Y) = T_base_camera`。合成实验验证了这层语义映射。
- LiDAR 工具若输出 `T_camera_lidar`，则 `T_base_lidar = T_base_camera @ T_camera_lidar`。若输出反方向先求逆；必须确认 camera 是同一个光学帧，不能把深度帧或未注册点云当作彩色光学帧。

每次求解保存数据 manifest（样本ID、采集时间、帧、图像/点云/关节数据 hash、内参/畸变、板尺寸与夹具说明、FK模型版本、单位）、上游版本/参数、原始输出与转换记录。训练残差、留出样本误差和现场独立已知物体验收分别保存，像素误差不直接换算为毫米精度。

provenance 沿用 SSOT 的逐源必填字段；导出器与加载器共享规范化/ID 算法，文件 hash 与 record ID 区分。合成数据只能导出独立测试夹具，不能写成真实设备 calibrated=true。规划/离线工具不能自行覆盖当前生产配置。

## 决定与研究

用户本轮原始指令：“继续下一个ticket”。无新选型决定；继承开源复用择优、base_link Y-up、双源逐源 provenance、运行时 SSOT 与标准诊断状态接口决定。

上游选型核验见[标定工具链研究](../research/calibration-toolchain.md)（本轮生成）。推荐直接复用现有 OpenCV 求解 AX=YB，保留 FAST-Calib2 为 LiDAR–camera 离线候选；不预设移植 ROS 2。FAST-Calib2 的 ROS 1 构建/数据接口需要隔离环境或格式薄适配，尚未在本机复现。默认大型 LiDAR 标定板与末端轻型视觉板建议分开，实际板尺寸与采样可行性待核验。数据薄适配必须核对强度字段类型及 ring 存在与否：上游读取器对这些字段有具体假设，不能仅转换 bag 容器便宣称兼容。以上是证据支持的工程建议，不记录为用户已接受选型。

## 验收与编码门

可以继续隔离离线验证：方向/单位测试、合成无噪声恢复、噪声和退化样本拒绝、已有数据的只读分析。正式 exporter/loader 的共享 schema 与 ID 算法尚需 SSOT 实施契约落地。

实施时至少验证：输入数量/有限值/SE(3)、帧名与内参匹配、混合单位拒绝、采样激励/退化、独立留出与重复采集误差、单源外参和变换链闭合、provenance/ID 可复现、错误数据不产生可准入记录；部署后由实际节点报告 resolved ament 路径/hash/ID，验证 source/install 分叉不会误报通过。

尚未执行真实数据求解、LiDAR 工具构建或样例复现、ROS 启动/部署、现场验收；不能关闭本票，不能宣称满足20mm/5°或双传感器真机准入。现场标定板/夹具与配对数据是否存在已询问用户，未答前记为“未核验”。

## 改动、回退与下一窗口

本轮只新增研究、交接与 .scratch 隔离验证材料；不修改运行代码、配置、已接受 ADR，不提交、不部署。停止使用隔离脚本即可回退本轮验证；未来实际替换记录必须保留旧代码/配置配对快照，并在重新部署重启后核验身份，旧假定外参不能因回退获得准入。

后续队列为[双传感器官方驱动接入与版本冻结](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31)，依赖仍以 GitHub 为准；本窗口不自动开展下一票。实施须由用户另外指示。
