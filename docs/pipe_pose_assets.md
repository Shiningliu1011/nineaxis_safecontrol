# Python 工具轴与连续姿态资产

`src/robot_safecontrol_moveit/pipe_assets/` 提供源 V3 姿态程序的 Python 接口。依赖 NumPy、SciPy、OSQP、Matplotlib、pymatreader 与 h5py，运行时无需 MATLAB 或 ROS。原始文件和来源身份见[资产核验](planning/nineaxis-full-project/asset-inventory.md)。

## A4 入口

```python
from robot_safecontrol_moveit.pipe_assets.A_4_zitai_gouzao import A_4_zitai_gouzao

result = A_4_zitai_gouzao(
    options={"show_figures": False},
    sourceFile="tests/data/pipe_assets/pipe_path_20261013_180806.mat",
    outputDir=".scratch/pipe-pose/example",
    fitDir="tests/data/pipe_assets",
    offlineFile="tests/data/pipe_assets/nurbs_blocks.mat",
)
```

`fitDir` 提供 `control_points.txt`、`knot_vector.txt` 与 `curve_data.csv`；`offlineFile` 提供同一曲线的 A2 计划。自行生成新路径时，依次调用 `fit_pipe_path_nurbs`、`A_2_lixianjisuan` 和本入口，A4 严格检查曲线身份。

入口返回 `reference`、`plan`、`data`、`pose`、`validation`、`ik_input` 与 `output_directory`。只有几何、连续角运动和位置运动检查通过时才导出结果。输出包括 `fit_reference.mat`、`pose_reference.mat`、`pose_results.mat`、`ik_input.mat` 与 `pose_diagnostics.csv`。省略 `sourceFile` 时在拟合目录及其两层父目录内寻找原始画笔 MAT，原始点关联必须唯一；其余路径建议显式提供。

## 查询与数据约定

结构使用 Python `dict`，标识符和字段沿用源名称。向量使用一维数组，点列使用 N×D；`R_WT`、`dR_darc`、`d2R_darc2` 保留 3×3×N。多输出使用 tuple。

```python
from robot_safecontrol_moveit.pipe_assets.pipe_pose_reference import pipe_pose_reference

reference = pipe_pose_reference("load", ".scratch/pipe-pose/example/pose_reference.mat")
pose = pipe_pose_reference("eval_progress", reference, [0, 0.5, 1])
state = pipe_pose_reference("eval_arclength_state", reference, [0, 100], 20, 0)
```

`progress=u`，`arc_mm` 为最终 NURBS 的积分弧长，`path_tau` 为原始画布路径参数。位置、长度、速度和加速度分别使用 mm、mm、mm/s、mm/s²；角度和角运动分别使用 rad、rad/s、rad/s²、rad/s³。周向画布输入继续使用 deg。`normal_raw_W` 为外法向，`tool_axis_raw_W=-normal_raw_W`，`tool_axis_ref_W` 为平滑执行轴。

`R_WT` 将工具坐标映射到工件坐标 W，工具 `+Z` 对应执行轴；`quat_wxyz` 使用 Hamilton 标量在前约定。乱序、重复查询使用固定 quaternion 符号基准。任务跟随 TCP 三维位置和执行轴二维方向，完整滚转保存为名义参考。

其余公开函数保持源动作：

| 函数 | 动作或用途 |
|---|---|
| `pipe_surface_correspondence` | `pairs_csv/build/eval/model`，核对原始 MAT 行号、参数、工件、单位、模型签名和源码身份 |
| `pipe_axis_smoothing` | `build/eval/controls`，五次向量样条、归一化导数和区间控制向量 |
| `pipe_orientation_reference` | `build/eval`，平行移动积分、共享 C2 导数、投影和完整旋转 |
| `pipe_pose_math` | `basis/bernstein/beval/extract/normalize/gauss/arc/quat/cross_controls/hermite` |
| `pipe_pose_time_law` | `eval/time_at_arc/phase_times/stretch`，共同半余弦时间律及整数周期时间伸长 |
| `pipe_pose_angular_jerk` | `eval/sample`，解析三阶角运动、接点左右极限与绘图断点 |
| `pipe_pose_validation` | `geometry/motion`，来源、旋转、连续性、独立差分、区间下界和连续极值检查 |
| `pipe_pose_schedule` | 复用 A2/A3 位置计划，协调角运动上限、最低巡航速度与共同采样 |
| `pipe_pose_prepare_model` | 保持位置接缝检查，允许显式列出的法向跳变 |
| `pipe_pose_options` | 配置校验；未配置的工艺限制保持为空 |
| `pipe_pose_export` | 导出通过检查的 V3/V4 参考与结果 |
| `pipe_pose_plot` | 创建八幅 Matplotlib 图形，中文 SimSun、英文和数字 Times New Roman |

## 约束与数值行为

平滑轴 QP 使用 OSQP，检查 stationarity、primal residual、complementarity、dual 非负性及原始半空间限制。区间范数下界、投影下界和 quaternion 旋转行程分别保留。完整旋转由最终投影和归一化表达式求导，`omega_W`、`alpha_W` 和 `angular_jerk_W` 使用同一时间律。

位置采样保留 A3 阶段接点的左侧 jerk；解析 angular jerk 在内部节点采用右侧、终点采用左侧。C2 姿态允许三阶导数跳变，绘图使用真实左右极限与 NaN 断点。

角速度或角加速度超限时，按阶段周期公约数计算共同时间伸长，保持 `Ts` 不变，并重新检查位置与角运动。若与 `minimum_cruise_speed_mm_s` 冲突、未允许时间伸长或超过 `max_time_scale`，程序报告错误。`initial_x_W` 或起始切向与工具轴平行时需要提供有效初始方向。

默认未设置 `hard_axis_error_rad`、`soft_axis_error_rad`、`omega_max_rad_s`、`alpha_max_rad_s2` 和 `minimum_cruise_speed_mm_s`，对应状态保留 `not_configured`。表面距离、实际工作距离和工艺作用点保留 `deferred`。这些配置与机器人关节能力、机器人跟踪误差分别处理。

新参考保存 Python 源码快照，加载时核对版本、源码、模型数值及可选的期望几何和配置。随本项目核验的原 MATLAB V3 参考支持按其固定 SHA-256 身份加载。其他来源需要重新构建参考。端点仅处理 64 个浮点间距以内的积分舍入差，范围外的查询立即报告错误。

## 实际验证

2026-10-27 执行命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src \
TMPDIR="$PWD/.scratch/pipe-pose/tmp" \
MPLCONFIGDIR="$PWD/.scratch/pipe-pose/mpl" MPLBACKEND=Agg \
.scratch/pipe-assets-runtime/venv/bin/python -m pytest \
  tests/test_pipe_pose.py --basetemp="$PWD/.scratch/pipe-pose/pytest" -q
```

13 项测试通过。覆盖真实 V3 参考的轴、旋转、导数和解析 angular jerk；原始关联与来源拒绝；QP 重建；连续极值与 C2 检查；A4 重建、MAT/CSV 导出和重载；八幅真实图形对象；共同时间伸长、最低巡航冲突及已确认几何接缝。

`tests/data/pipe_assets/pose_samples.npz` 保留原始 MAT 的 916 个真实行，覆盖首尾、全部姿态节点与时间阶段接点的邻近原始行。来源 MAT SHA-256、原始行号和每个数组的 SHA-256 保存于 `pose_samples_provenance.json`。完整原始输出的本轮独立复算还保存在 `.scratch/full-project-assets/numerical-checks.json`；保存的 MATLAB validation 与 Python 新运行证据分别记录。
