# 工件、路径原始资产与 Python 运行核验

核验日期：2026-10-27。对应任务：[取得完整工件、路径与 MATLAB 程序资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/135)。本页分别记录来源身份、读取保存结果、本轮独立复算和运行条件。

## 来源与交接目录

原 ZIP：`/home/lsn/robot/NURBS曲线.zip`，16,019,306 字节，SHA-256 `907fbfc97e961aca04e83b69b8a38839aff2e4f6e32e98c8e89f7a2c1d06e58d`。ZIP CRC 检查通过，包含 64 个文件；原压缩包和全部成员逐字节保留。

本机核验目录为 `.scratch/full-project-assets/`，受到已有 `.gitignore` 规则保护。`payload/` 保留 98 个来源文件：原 ZIP、全部解压成员、仓库 `data/nurbs/` 的 11 个文件、`/home/lsn/robot/nurbs_data/` 的 11 个文件以及 `九轴技术路线/` 的 11 个文件。两个旧路径目录的全部同名文件 SHA-256 相同。

完整成员路径、原 ZIP 时间、字节数及 SHA-256 见 `inventory.json`；全部真实 MAT 的成熟库读取结果见 `mat-structures.json`。`SHA256SUMS` 与 `full-project-assets.tar.gz` 提供跨机器传输，`bundle-identity.json` 保存资料包身份。资料包为 44,695,055 字节，SHA-256 `958e0eea9168dfe7522935d8c7919651b8231a2ccc08fa3dd8293d028346d11c`，105 个文件已从归档回读并逐项核验。Git 同步需要另外传输该资料包。

以下 `B/` 代表 ZIP 成员目录：

`NURBS曲线/DiYiJieDuan/shiyanjianmo/jianmo_guiji/yilaixiang/`

以下 `F/` 代表 `B/jiemian/lujing/nihe_yilai/nurbs_nihe/`；`G/` 代表 `F/sudu_yilai/`；`S/` 代表 `G/sudu_fenpei/`；`P/` 代表 `S/zitai_yilai/zitai_gouzao/`。本机程序路径为 `.scratch/full-project-assets/payload/source/` 加相应成员路径，目录结构保持原样。

## 程序、接口与依赖

| 程序 | 行数 / 字节数 | 公开接口与直接依赖 |
|---|---:|---|
| `B/combined_pipe_model.m` | 154 / 6552 | `model=combined_pipe_model()`；`segments(i).evaluate(u,v)` 返回 `P,N,Pu,Pv`；解析模型自包含 |
| `B/pipe_surface_eval.m` | 37 / 1747 | `(model,canvasUV,offsetMM)` 返回 `tcp,surface,normal,segmentId,localUV`；使用 prepared model 的 `evaluate` |
| `B/pipe_reference_path.m` | 100 / 4001 | `(model,anchors,offsetMM,spacingMM,pathDefinition)` 返回 `data`；依赖 `pipe_path_curve`、`pipe_surface_eval`、`spline`、`fzero`、`interp1` |
| `B/jiemian/run_pipe_painter.m` | 33 / 2260 | 脚本；默认 `pipe_demo_model`，调用 `pipe_painter`，导出间隔 1 mm，法向外距离 5 mm |
| `B/pipe_painter.m` | 979 / 50178 | `app=pipe_painter(model,options)`；依赖 `pipe_prepare_model`、`pipe_reference_path`、`pipe_surface_eval`、`pipe_path_curvature` 和 MATLAB 图形接口 |
| `B/jiemian/lujing/nihe_yilai/fit_pipe_path_nurbs.m` | 267 / 13783 | `(pathFile,outputFolder,showFigure)`；MAT/CSV 弦长参数三次单位权重插值，输出控制点、节点、`curve_data.csv`；需要 `NURBS` 类 |
| `G/pipe_nurbs_geometry.m` | 345 / 13986 | `load/eval/arc/invert`；曲线及前三阶参数导数、分节点积分、弧长反解 |
| `G/pipe_feed_profile.m` | 688 / 35054 | `build/sample/check`；依赖 `pipe_joint_schedule`、`pipe_nurbs_geometry`、`pipe_motion_trace` |
| `G/pipe_joint_schedule.m` | 629 / 26533 | `(arc,seed,physical_boundary_limit,lim,scale,feedback)`；整数周期与边界/平台速度联合计算，需要 Optimization Toolbox 的 `linprog` |
| `S/A_2_lixianjisuan.m` | 492 / 21639 | 离线脚本；还使用 `pipe_curvature_keypoints`，写入 `nurbs_blocks.mat` |
| `S/A_3_shishichabu.m` | 214 / 13701 | 固定周期脚本；读取已保存时间律，采样并检查，写入位置运动 MAT |
| `P/A_4_zitai_gouzao.m` | 57 / 3421 | `(options,sourceFile,outputDir)`；依赖父目录中的姿态系列、曲面、曲线和速度规划模块；平滑轴使用 Optimization Toolbox 的 `quadprog` |

`pipe_painter` 的公开操作包括 `setPath/setOffset/getState/getSamples/setExamplePath/showCurvature/getCurvature/export/load/moveKeypoint/beginStroke/appendStroke/endStroke/cancelStroke/undo/setLayer/close`，另有画布视图、缩放和平移操作。公开字段位于源程序第 154–178 行。

源模型和画笔入口声明 MATLAB R2020b；姿态采用 `scheme_version=V3`，A4 `ik_input.format_version=4`。ZIP 没有独立发布版本或许可文件，已核查源码也没有提供许可声明；外部 `NURBS.m` 的作者、版本和许可同样没有随本机文件给出。当前仅保存用户提供资料及其身份。

ZIP 没有 `NURBS.m`。本机两个旧路径目录均有 21,203 字节的同一份类文件，已全部收录；SHA-256 为 `9a16b26041b5ef410187af107b8ada9821fa2762a4f63126799bf0448a396c3c`。它提供源插值所用类接口，其 MATLAB 运行兼容性本轮未执行确认。

## 模型与真实输入样本

`combined_pipe_model` 为五段解析组合：100 mm 直管、70 mm 变径管、中心半径 90 mm 的 90° 弯管、100 mm 直管、80 mm 变径管，入口/中间/出口半径分别为 20/30/18 mm。画布累计长度 491.371669411541 mm，出口中心 `[270,0,260] mm`。本轮以 NumPy 独立求值检查全部四处接缝，每处采样 361 个周向位置，位置最大分量残差 `5.69e-14 mm`，单位外法向残差为零。直线与弯管的曲率连续性保持源说明中的限制。

源三组画笔 MAT/CSV 均保存 `pipe_demo_model` 的直管和变径管输入：

| `B/jiemian/lujing/` 中的文件名 | 点数 | 原始折线弧长 |
|---|---:|---:|
| `pipe_path_20261013_170314.mat/.csv` | 407 | 405.364860223426 mm |
| `pipe_path_20261013_180806.mat/.csv` | 480 | 478.751700651401 mm |
| `pipe_path_20261014_165040.mat/.csv` | 743 | 741.700706665122 mm |

480 点样本关联当前控制点与 A3/A4 输出，源 `pose_reference.correspondence.source_snapshot` 的 TCP 与该 MAT 逐元素一致。全部三组样本的 `tcpPoints=surfacePoints+5*normals`、`toolAxis=-normals`、单位法向及累计折线弧长检查通过。以当前源解析模型独立复算，表面位置最大向量残差不超过 `1.71e-13 mm`。

## 文件身份

| 文件 | SHA-256 |
|---|---|
| `B/combined_pipe_model.m` | `2065947fe89adfd03ed13ae61cbf705b7a9bc5a2be0d1d73ec4963df8a7fe94a` |
| `B/pipe_surface_eval.m` | `efb9790629e5c207f8d4d71fed9ecbf0dbf246b0f51e42abe87b93adbd55b717` |
| `B/pipe_reference_path.m` | `dd70e3a594e8451b5eb0f98fbb8565a7a981f55f4dbf7d5264d8f780088944e7` |
| `B/jiemian/run_pipe_painter.m` | `56ce1423708d7b6536b5dd95deefe77a95be05c36daa02cae5b991eacea69e68` |
| `B/pipe_painter.m` | `76c6a0cecd23f0f852929fb4474f57252ce0436bfb213f3e562adb4a697df0c4` |
| `F/control_points.txt` | `b78e940f344e987ac2624e328d7be0a7972a30f0f83ff49ffdebbed08fc53b88` |
| `F/knot_vector.txt` | `08c33b190a07b290b73793f609e9f586763d9907ec700b460c35f8c03d3fcbcd` |
| `S/nurbs_blocks.mat` | `7f7295cb7b7bf2b73b2b0e07f4e555a4cdbec4815cdd516982edd7dc8c63a4d0` |
| `S/ik_input.mat` | `cd724fcfb13dfcdc86a8f21de4bdd69a3ab88a97e66f6eff613774ba1d08d341` |
| `P/ik_input.mat` | `74acea6a2033e72e0be182d07e800c414d453ed3477170f9cb7cfe2976bc0942` |
| `P/pose_reference.mat` | `0deb8a83cb47f3397085eede4b84765933c1d08acf404189521f71c18a3f85b9` |
| `data/nurbs/ik_input.mat` | `79868b03afd68f9d4cca1cba568320f0f48efd355d020a9372f2037549c3293f` |

旧仓库路径保留独立身份：14992 点、23 段。独立 BSpline 求值与旧保存位置最大分量差 `8.53e-14 mm`。当前源路径为 480 个控制点、484 个节点、三次、单位权重、内部简单节点；按样条结构满足 C2。保存参考中 15 个模型与姿态源码全文与 ZIP 对应成员一致。

## 本轮独立复算

实际命令：

```bash
TMPDIR="$PWD/.scratch/full-project-assets/tmp" .scratch/predictive-coordination-spec/mat-reader-env/bin/python .scratch/full-project-assets/audit_assets.py prepare
TMPDIR="$PWD/.scratch/full-project-assets/tmp" .scratch/predictive-coordination-spec/mat-reader-env/bin/python .scratch/full-project-assets/verify_numerics.py
```

两项命令退出码均为零。环境为 Python 3.13.12、NumPy 2.4.6、SciPy 1.17.1、pymatreader 1.3.2、h5py 3.16.0。MAT v5 使用 SciPy，MAT v7.3 使用 pymatreader/h5py；没有自行解析 MAT 字节。

`verify_numerics.py` 从保存计划的整数阶段周期和速度独立复算半余弦时间律，以 SciPy BSpline、积分和 BPoly 重算曲线、姿态及导数，结果保存在 `numerical-checks.json`。4756 点、4 段、9.51 s；独立积分弧长为 478.791363842698 mm。A3/A4 的位置、速度、加速度、jerk、时间、参数和进给逐元素一致。

| 检查量 | 最大分量残差 |
|---|---:|
| TCP 位置 | `1.71e-13 mm` |
| XYZ 速度 | `4.05e-12 mm/s` |
| XYZ 加速度 | `4.07e-10 mm/s²` |
| XYZ jerk | `5.24e-8 mm/s³` |
| 时间律弧长与独立积分 | `9.86e-11 mm` |
| 平滑工具轴、`R_WT` | `2.03e-15` |
| `omega_W` | `1.90e-13 rad/s` |
| `alpha_W` | `2.26e-11 rad/s²` |
| `angular_jerk_W` | `1.37e-9 rad/s³` |

旋转正交性、行列式、Hamilton `wxyz` 四元数与旋转矩阵关系、`R_WT` 的工具 `+Z`、原始工具轴与外法向的关系均通过检查。姿态五次多项式的全部内部节点分别核验零、一、二阶导数，最大残差 `1.12e-15`。

采样限值检查通过：最大进给 57.899189 mm/s、切向加速度 99.938768 mm/s²、法向加速度 288.287289 mm/s²、时间律 jerk 2037.853187 mm/s³、XYZ jerk 法向投影 2470.568637 mm/s³、`κ²v³` 为 3031.314644 mm/s³；每个采样区间七个内部位置的最大弦误差 0.000144019 mm。对应源上限及定义见[运动限制研究](motor-nurbs-constraint-limits-research.md)。独立记录的 XYZ 加速度、jerk 模长分别为 302.135512 mm/s²、3361.116859 mm/s³。

源 `progress=u`，`arc_mm` 为最终 NURBS 的积分弧长；画笔 `pathParameter` 与画布累计位置分别保留原定义。坐标为工件坐标 W，几何单位 mm、局部角度 rad、画布周向角 deg、时间 s。`R_WT` 将工具坐标映射到工件坐标，源工具轴为 `+Z`。仓库控制任务中的 `+X` 工具轴需要在导入边界明确转换。

平滑执行轴相对原始内法向最大采样夹角 0.762096944°；最大完整角速度、角加速度、angular jerk 分别为 84.506506 °/s、446.754060 °/s²、4908.221979 °/s³。源配置的最低巡航进给、轴向工艺误差和角运动限值均为空。

本轮对应保存表面点计算的距离为 4.999979193–5.000049185 mm，沿法向距离为 4.999978761–5.000049044 mm，相切分量最大 0.002344234 mm。对应点计算覆盖源关联关系；最近表面距离、工艺作用点和实际作业精度仍需各自验收。源表面与工作距离状态保留 `deferred`。

## 保存结果与可运行条件

`numerical-checks.json.saved_validation_only` 原样保留源 `validation`。其中 `passed=1` 及保存的密集极值检查来自源运行结果；本轮独立检查的范围为上文数值复算和采样约束。源 MATLAB 完整规划、全部连续极值和交互回归本轮没有重新执行。

本机 `command -v matlab octave octave-cli` 没有找到可执行程序，`/opt/MATLAB` 和 `/usr/local/MATLAB` 目录不存在。源 A2/A3 默认向源程序目录写入 MAT，A4 支持 `outputDir`，回归程序支持指定记录目录；跨机器执行应明确提供独立输出目录并配置 `NURBS.m` 及 Optimization Toolbox。Octave 兼容性尚未验证。

用户要求的 Python 转换已完成。程序位于 `src/robot_safecontrol_moveit/pipe_assets/`，使用方式见[Python 入口](../../pipe_assets.md)、[进给与插补](../../pipe_feed_assets.md)及[姿态接口](../../pipe_pose_assets.md)。几何、画笔、NURBS、A2、A3、A4 均从实际输入执行；数值命令行关闭绘图，画笔使用真实 Matplotlib/Tk 窗口。

最终独立环境运行 `tests/test_pipe_*.py`：60 项通过，136.72 s，退出码 0。覆盖三组原始路径、MAT/CSV、鼠标键盘、独立画布、真实 HiGHS/OSQP、完整约束、单块时间律和单区间姿态参考的保存重载。主窗口与独立画布的 TkAgg 运行检查通过，关闭后没有遗留 figure。代码规范与需求两项独立复审均无待处理问题；LP 第二目标失败处理采用静态分支核对，没有构造求解器故障。

最终完整命令从 480 点原始 MAT 重新拟合、新建 A2 计划、运行 A3 和 A4，输出 4756 点、9.51 s，全部已配置约束验证通过。结果位于 `.scratch/pipe-assets-runtime/final-complete/`。新生成结果与源保存结果的最大分量差分别为位置 `3.49e-12 mm`、速度 `4.89e-11 mm/s`、加速度 `5.33e-9 mm/s²`、jerk `5.89e-7 mm/s³`；时间逐元素一致。40 个当前 Python 源文件和 11 个输出文件的 SHA-256 已与最终 manifest 逐项核验。

实际命令、环境、日志和验收记录位于 `.trellis/tasks/10-27-python-path-assets/implement.md`。Python 程序、真实测试资料、文档和任务记录通过 `main` 同步。独立验证快照与最终运行证据另保存在 `.scratch/pipe-assets-runtime/python-path-assets.tar.gz`，身份与回读结果见同目录 `python-bundle-identity.json`。
