# 总体规格的事实依据

核查日期：2026-10-26。当前分支 `codex/static-obstacle-experiments`，同步后提交 `6a97052`。本次控制文件与同步前 `957631e` 一致；同步内容为标定资料。以下区分源码检查、数据重算与历史实验记录。

## 需求来源

- 用户确认继续原完整项目地图，并以在线约束协调、工艺与电机限制、末端路径和工具轴向、实时全身避障及效率为研究目标。
- 用户提供《九轴预测协调控制_项目修改交接.md》《九轴预测协调控制_接手提示词.md》。本轮依据当前请求更新规划文档；其中提及的实现或实验步骤作为后续工作资料。
- 用户说明 NURBS 压缩包及技术路线来自师兄，运动参数应核查源数据，预测方法需要继续研究。
- [总体规格](../../specs/predictive-coordination-control.md)保存当前项目要求；本页保存核查证据。

## 当前控制能力

| 项目 | 已核查事实 | 位置 |
|---|---|---|
| 五维任务 | 三维位置、二维工具轴误差；真实夹角独立计算 | `portable_oscbf/work/moving_reference_task.py:50` |
| 共同求解 | `joint` 使用九关节速度和一个进给变量 | `portable_oscbf/work/static_velocity_solver.py:321` |
| 参考执行 | `reference_start_m + feed_m_s * effective_s`，区间内固定进给 | `portable_oscbf/work/static_execution.py:19` |
| Ruckig | 从当前位置、速度、加速度生成运动与停止；检查关节运动及碰撞区间 | `portable_oscbf/work/static_motion.py:96`、`:150` |
| 恢复 | 误差阈值、正向候选、持续确认及超时处理 | `portable_oscbf/work/static_recovery.py:59` |
| 实际工具指标 | 已有几何、参考、轴向及工作距离；工具工艺速度及高阶导数检查需增加 | `src/robot_safecontrol_moveit/static_experiment_evaluation.py:138` |
| 命令生效预测 | 预测下一命令生效状态后调用一次当前控制 | `src/robot_safecontrol_moveit/static_experiment_nodes.py:286` |
| MPC | 相关控制目录和跟踪文件中未找到多时段优化实现 | 当前实验入口仅 `separated/joint/sns` |
| 其他机构 | `StaticGeometry` 使用 `NineaxisKinematics`，运动生成固定九轴 | `portable_oscbf/work/static_collision.py:87` |

以上为静态核查，本轮没有重新运行控制测试。已有测试是后续验证入口，不能据其存在声明新要求通过。

## NURBS 源资料

本机来源为 `/home/lsn/robot/NURBS曲线.zip`，SHA-256：

`907fbfc97e961aca04e83b69b8a38839aff2e4f6e32e98c8e89f7a2c1d06e58d`

包内共同目录为 `NURBS曲线/DiYiJieDuan/shiyanjianmo/jianmo_guiji/yilaixiang/jiemian/lujing/nihe_yilai/nurbs_nihe/`。

| 源参数 | 数值 | 含义及依据 |
|---|---|---|
| `F` | 60 mm/s | 最大进给；`sudu_yilai/sudu_fenpei/A_2_lixianjisuan.m:10` |
| `At` | 100 mm/s² | 进给时间律的切向加速度；同文件第 11 行 |
| `Jt` | 16400 mm/s³ | 进给速度的二阶时间导数；同文件第 12 行、`A_3_shishichabu.m:117` |
| `An` | 300 mm/s² | 法向加速度 `kappa*v²`；同文件第 13 行、`pipe_feed_profile.m:588` |
| `Jn` | 16400 mm/s³ | 用于 `kappa²*v³` 及完整 XYZ jerk 的法向投影；`pipe_feed_profile.m:589`、`:650` |
| `delta` | 0.001 mm | 插补弦误差；`A_2_lixianjisuan.m:9` |
| `Ts` | 0.002 s | 源资料插补周期；同文件第 15 行 |
| `Fmin` | 10 mm/s | 几何扫描参数；同文件第 19 行 |
| `position_tol` | 0.000001 mm | 曲线求值、弧长反解与分段闭合数值检查；`nurbs_blocks.mat.limits` |

这些量的来源为工具 Cartesian 路径与时间律。九个电机的关节运动能力需要独立资料。完整 XYZ jerk 模长在源程序中单独记录；正常加工的正进给下限、实际轴向误差和机器人跟踪容差需要对应任务确定。

`A_3_shishichabu.m:61`、`:107` 声明 `velocity_series` 为 XYZ 瞬时速度，`acceleration_series` 和 `jerk_series` 为完整 XYZ 导数，`speed_series`、`feedrate_cmd_series` 为名义进给，`tangent_acc_cmd_series` 和 `tangent_jerk_cmd_series` 为进给时间律导数，`step_feedrate_series` 为相邻点弦长除以 `Ts`。允许限值保存于 `limits`。

压缩包 A3/A4 轨迹共有 4756 个点、4 段、总时长 9.51 s、弧长 478.7913638426983 mm。源文件 `validation` 保存最大进给 57.89918925134624 mm/s、切向加速度 99.93936394784006 mm/s²、法向加速度 288.3506292415532 mm/s²、完整 jerk 模长 3361.116858583284 mm/s³，并保存 `passed=1`。本轮读取了这些值，没有重新执行 MATLAB 验证程序。

实际独立重算：SciPy `BSpline` 对全部 `u_series` 求值，与源位置最大差为 `1.7346558842822478e-13 mm`；`velocity_series` 模长与 `speed_series` 最大差为 `1.4210854715202004e-14 mm/s`；A3/A4 的位置、速度、加速度、jerk、时间、参数及进给逐元素一致。

姿态入口为 `sudu_yilai/sudu_fenpei/zitai_yilai/zitai_gouzao/ik_input.mat`。格式为 4、方案 V3，包含工具轴导数、`R_WT`、`omega_W`、`alpha_W`、`angular_jerk_W`。`pose_reference.mat.reference.config` 中 `hard_axis_error_rad`、`soft_axis_error_rad`、`omega_max_rad_s`、`alpha_max_rad_s2` 和 `minimum_cruise_speed_mm_s` 均为空。角运动限值状态为 `not_configured`；表面与工作距离验证状态为 `deferred`。

源工具参考轴为工具坐标 `+Z`，`R_WT` 将工具坐标映射到工件坐标。仓库任务使用工具 `+X`；导入应明确轴变换。源 `progress=u`，`arc_mm` 单独保存。`tool_axis_ref_W` 为平滑后的给定执行轴，`normal_raw_W` 保留原外法向，导入时保存两者关系。

现有 `data/nurbs/ik_input.mat` 为 14992 个点、23 段，与压缩包路径不同。旧控制点与采样的独立求值误差约 `8.76e-14 mm`。原始 `.m`、控制点、节点及 MAT 文件受到 `.gitignore:49` 的本地保留规则影响，跨机器需另外传输。

检查脚本位于 `.scratch/predictive-coordination-spec/inspect_nurbs_archive.py`，MAT v7.3 使用该目录独立环境中的 `pymatreader/h5py` 读取，其他 MAT 使用 SciPy。原 ZIP 和现有路径文件保持原内容。

## 实验与数据处置

2026-10-26 核查时，GitHub 十项静态基础任务已经关闭。历史共同周期仅有 A/B 的 20 ms 两份完整失败记录，实际 2/20；正式采集为 0/1080，最终共同周期未取得，容量 `formal_bound=false`。这些数字保留其原实验身份；现行任务状态见[前期证据入口](../../static_obstacle_experiments_handoff.md)。

本轮测量 `output/static-*` 实际分配 9,004,523,520 字节，全部 `output/` 为 12,457,467,904 字节；根文件系统可用 72,944,881,664 字节。没有发现相关实验进程；检查的历史服务为 inactive、MainPID=0。

两个历史卷当前未挂载，不能核查其当前内容：

- UUID `083CBB753CBB5BFE`，原目录 `/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`。
- UUID `6C001EAA001E7B72`，原目录 `/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/`。

保留原始数据库、manifest、路径、结果、源码归档、见证、失败、报告与报告关联数组。当前20份报告绑定的60个数组均存在；删除这些数组会破坏报告身份验证。

上述数据保存原始身份。主线的实验预算由 #144 定义要求、#169 完成测量和清单核验。可再生成的编码副本与重复源码目录仅作为容量整理候选，具体文件、引用及恢复条件核对完成后再处理。

## 后续入口

[总体规格](../../specs/predictive-coordination-control.md)、[实现方案](implementation-design.md)、[需求覆盖](coverage.md)及[任务依赖](dependencies.md)共同说明当前工作。旧数据继续服务于回归、复现和方法来源说明，新方法使用独立身份记录。
