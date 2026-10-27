# NURBS、进给规划与固定周期插补

Python 程序位于 `src/robot_safecontrol_moveit/pipe_assets/`，按原 MATLAB 的公开函数执行曲线插值、弧长求值、曲率关键点筛选、整数周期联合规划和完整运动验证。原始程序与真实数据身份见[资产清单](planning/nineaxis-full-project/asset-inventory.md)，机器人控制的当前能力见[基线核验](planning/nineaxis-full-project/baseline-audit.md)。

## 输入和单位

- `fit_pipe_path_nurbs(pathFile, outputFolder="", showFigure=False)` 读取 MAT 的 `data_points` 或 `trajectory.tcpPoints`；CSV 按 `tcp_x_mm/tcp_y_mm/tcp_z_mm` 字段读取。MAT 同时提供两份坐标时检查一致性；声明的 `lengthUnit` 必须为 `mm`。
- 插值只删除连续相同 TCP 点，保留路径返回位置的点与顺序。`input_indices`、`removed_input_indices` 以及对外导出的 block 和 boundary 序号继续从 1 开始。
- 位置、弧长使用 mm，时间使用 s，进给与其导数使用 mm/s、mm/s²、mm/s³。曲线导数针对无量纲参数 `u`。`u` 与弧长各自保存。
- 曲线为三次夹持、单位权重、内部节点不重复的 NURBS。参数域为 `[0,1]`，内部节点取右侧，终点取最后区间左极限。
- 默认数值调用关闭绘图。`showFigure=True` 创建 Matplotlib figure；调用方负责交互或导出。数值转换和测试不需要显示服务器。

## 公开接口

结构使用 dict，MATLAB 多个返回值对应 Python tuple，点列为 N×3，标量序列为一维 NumPy array。

```python
from pathlib import Path

from robot_safecontrol_moveit.pipe_assets.fit_pipe_path_nurbs import fit_pipe_path_nurbs
from robot_safecontrol_moveit.pipe_assets.pipe_nurbs_geometry import pipe_nurbs_geometry
from robot_safecontrol_moveit.pipe_assets.A_2_lixianjisuan import A_2_lixianjisuan
from robot_safecontrol_moveit.pipe_assets.A_3_shishichabu import A_3_shishichabu

obj, fit_result = fit_pipe_path_nurbs(pathFile, fit_dir, False)
geo = pipe_nurbs_geometry("load", fit_dir)
P, D1, D2, D3 = pipe_nurbs_geometry("eval", geo, [0.0, 0.5, 1.0])
arc = pipe_nurbs_geometry("arc", geo, [0.0, 1.0])
u, residual = pipe_nurbs_geometry("invert", geo, arc, [0.0, arc["total_length"]])
offline = A_2_lixianjisuan(fit_dir, output_dir)
realtime = A_3_shishichabu(Path(output_dir) / "nurbs_blocks.mat", output_dir)
```

`fit_result` 保留原输入、法向和工具轴的来源信息，以及实际弦长参数、平均节点、输入点误差和加密曲线。输出目录包含 `control_points.txt`、`knot_vector.txt` 和 `curve_data.csv`；CSV 对输入点保留原始行号及误差，新增采样点的对应输入字段为 NaN。

`geo` 包含 `ctrl_pts/knots/p/w/control_file/knot_file`。`arc` 包含 `u/s/block_u/block_s/total_length/integration_error_estimate/inversion_tolerance_mm`。弧长表在全部节点及分块边界间比较 8/16 点 Gauss-Legendre 积分，按误差要求增加区间；反解采用带区间限制的 Newton 更新并直接核验弧长残差。

## A2 规划

`A_2_lixianjisuan(fit_dir, output_dir, options=None)` 从控制点和节点执行 PCI 扫描、曲率峰筛选、精确弧长积分、联合整数调度、局部几何反馈和边界结构搜索。程序重新求取计划，无需已有的 `nurbs_blocks.mat` 或保存轨迹。

默认源参数为 `F=60 mm/s`、`At=100 mm/s²`、`Jt=16400 mm/s³`、`An=300 mm/s²`、`Jn=16400 mm/s³`、`Ts=0.002 s`、`delta=0.001 mm`。`Fmin=10 mm/s` 用于扫描。其他可配置源字段包括 `eps/eps_PCI/beta/kcbc/position_tol/max_samples`，未知字段和无效数值直接报错。

`pipe_curvature_keypoints(u,kappa,kappa_cr,arc_mm,options)` 返回 `(critical_idx,audit)`。它只选择原始样本索引，逐组检查鞍点深度、物理距离、曲率距离尺度和端点边界，保存所有合并决定。

`pipe_joint_schedule(arc,seed,physical_boundary_limit,lim,scale,feedback=None)` 使用 SciPy HiGHS 的 dual-simplex LP 联合求取共享边界速度与中间阶段速度。整数周期候选保留半余弦加减速、精确位移、速度、加速度和 jerk 约束；候选以总速度变差和边界速度进行词典序比较。原始毫米等式和不等式在求解后再次检查。结果保留 `lp_solver_calls`、`candidate_integer_schedules`、`dynamic_projection_history`、`boundary_coarsening_audit` 与实际失败状态。

`pipe_feed_profile("build",geo,arc,seed,lim)` 返回 `(plan,validation)`。`seed` 的必需字段为 `block_u`；原边界对应的 `arc.block_s` 必须一致。每个结构候选都需要完整几何验证，最终选中的计划重新采样并验证全部限制。

A2 输出 `nurbs_blocks.mat`，保留源几何、原始扫描、曲率、峰选择审计、原边界、实际计划、限制、完整验收及性能记录。驻点、无效校正分母、无法推进、未完成扫描或无可行调度均直接报告，未通过验证的计划不会保存。

## A3 插补与验证

`A_3_shishichabu(plan_file,output_dir)` 读取已验证的离线计划，并重新读取 `source_curve.control_file` 所在目录的控制点与节点；内容不同即拒绝。跨设备复制时应将拟合文本一起传输，并在新位置重新执行 A2。

`pipe_feed_profile("sample",geo,arc,plan)` 根据同一半余弦解析积分生成全部采样点，使用弧长反解取得 `u`，计算完整 XYZ 速度、加速度和 jerk；区分瞬时进给与每周期弦长除以周期。起终点零速由时间律产生。

`pipe_feed_profile("check",geo,data,plan,lim)` 同时检查：

- 整数周期、速度阶段连接、真实峰值、块内速度形状、逐块位置与长度闭合、严格递增参数和起终点速度。
- 瞬时运动、解析阶段极值、相邻周期差分、七个区间内点的弦误差，以及完整 XYZ jerk 的法向投影。
- 不超过 0.5 ms 的稠密几何探针、所有节点和原关键位置的左右两侧、全部几何与运动光滑区间内的连续局部极值。

A3 输出 `realtime_interpolation_results.mat` 和位置型 `ik_input.mat`，字段沿用源资料。工具轴和姿态由[姿态模块](planning/nineaxis-full-project/asset-inventory.md)继续生成。`pipe_motion_trace` 提供 `reset/add/count/note/progress/get`，计时与计数独立于运动数值。

## 验证入口

真实回归数据固定于 `tests/data/pipe_assets/`，`provenance.json` 保存原 ZIP member 与 SHA-256。测试使用三份 407/480/743 点的原始路径、480 点源控制点和节点、4107 点原 PCI 扫描、4756 点原插补，以及独立解析直线。

```bash
export TMPDIR="$PWD/.scratch/pipe-feed/tmp"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
.scratch/pipe-assets-runtime/venv/bin/python -m pytest \
  tests/test_pipe_nurbs.py tests/test_pipe_feed_profile.py \
  -q --basetemp .scratch/pipe-feed/pytest-isolated
```

独立环境中的 Python 3.10.12、NumPy 2.2.6、SciPy 1.15.3 已执行上述两份测试，14 项全部通过，最后单独检查耗时 10.27 s，随后纳入全部 60 项资产测试并通过。绘图选项通过真实 figure、曲线数据、字体配置与 backend 属性检查，没有生成截图。源 4756 点重建的最大分量差：位置 `1.71e-13 mm`，速度 `4.05e-12 mm/s`，加速度 `4.90e-10 mm/s²`，jerk `5.24e-8 mm/s³`；源控制点最大差 `6.83e-13 mm`，弧长差约 `6.3e-13 mm`。

实际验证日志位于 `.scratch/pipe-feed/`；`numeric-evidence.json` 保存上述误差、源文件 SHA-256 和依赖版本。上述资产链保持工件坐标、毫米和源工具轴约定，机器人 M01/M02 的坐标与工具轴转换由相应实现任务管理。
