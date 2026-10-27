# Python 路径资产转换设计

## 当前能力与目标

ZIP 内含原始管道、路径、NURBS、速度及姿态程序；仓库现有 Python 主要读取已生成的 MAT。目标为直接运行 Python 生成和核验这些资产。

## 模块边界

新增 `src/robot_safecontrol_moveit/pipe_assets/`，全部程序保持无 ROS 依赖。源 MATLAB 的公开函数名称、动作名称及结构字段继续使用原名，结构使用 Python dict，数值数组使用 NumPy，向量默认一维、点列为 N×D。

- 几何模块：`combined_pipe_model`、`pipe_demo_model`、`pipe_complex_demo_model`、`pipe_prepare_model`、`pipe_surface_eval`、`pipe_path_curve`、`pipe_reference_path`、`pipe_path_curvature`。
- NURBS 与运动模块：`fit_pipe_path_nurbs`、`pipe_nurbs_geometry`、`pipe_curvature_keypoints`、`pipe_joint_schedule`、`pipe_feed_profile`、`pipe_motion_trace`。
- 姿态模块：保留原 `pipe_pose_*`、`pipe_axis_smoothing`、`pipe_orientation_reference`、`pipe_surface_correspondence` 的公开行为。
- 交互及入口：`pipe_painter`、`run_pipe_painter`、模块命令行入口，支持生成与检查原始资产。
- 公共文件读写模块由主会话提供：`mat_io.load_mat(path)` 返回 dict，`mat_io.save_mat(path, mapping)` 使用成熟 MAT 库；CSV 使用标准库或成熟表格库。

数学求值使用 SciPy 的 BSpline/BPoly、积分、插值及优化能力。MAT v5 与 v7.3 由成熟库读取。优化失败直接报告，不替换目标或隐藏约束。画笔使用 Matplotlib 的真实事件接口和控件，数值与交互状态可以独立运行。

## 数据与兼容

源结构字段保留原拼写，`segmentId` 与 `input_indices` 等对外导出序号维持源程序从 1 开始的规则；NumPy 内部索引遵循 Python。曲面和路径保留 mm、画布角度 deg、局部角度 rad；输出元数据注明单位。源工具 +Z 与工件坐标继续保留。接点的左右极限按各源程序分别处理。

真实测试资料从原 ZIP 通过标准 zipfile 读取或从固定身份的本地源副本读取；提交的测试应能够通过明确的资产根目录定位原始资料，不能把缺少资料标记为通过。用真实模型、原始 MAT/CSV 和数值结果验证公开接口。

## 文件与验证责任

几何、运动、姿态、交互由互不重叠的文件负责；主会话负责公共 MAT I/O、依赖、命令行、安装配置与整体验证。原始程序和数据保持只读。所有中间结果位于 `.scratch/`；图形导出若需要使用项目 `picture/` 目录及字体规则。

Python 转换保留来源身份，算法选择不改变地图中尚待确认的工艺、机器人模型或系统准入规则。代码检查范围为独立资产程序及其测试；已有控制程序的结果由基线核验单独记录。
