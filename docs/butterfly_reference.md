# 蝴蝶连续路径参考

现有九轴的 M01 使用 `data/nurbs/butterfly_reference_v1/` 中的连续参考。`production` 保存控制器默认圆柱，`s0` 保存 `config/static_experiment_s0.yaml` 的圆柱和名义工作距离。原 `data/nurbs/ik_input.mat` 保持原有内容。

## 数据与单位

每个版本目录包含以下文件。

| 文件 | 内容 |
|---|---|
| `source/` | 原 MAT、49 个控制点、53 个节点及原名义规划 MAT 的字节快照 |
| `reference.npz` | 三次区间系数、完整两次圆柱投影、最终弧长表、积分规则与源参数、时间和名义进给 |
| `source_samples.npz` | 原数值字段、各字段的有效性、原名义规划参数及 MATLAB cell 的数值内容 |
| `samples.npz` | 全部 14992 个原始参数处的新连续位置、执行轴及各阶导数，原参数、源弧长、时间和索引 |
| `manifest.json` | 版本、来源与产物 SHA256、实际变换、模型身份、单位、事件、生成命令和验证结果 |

源控制点使用 mm，区间系数使用 m。生成器将既有相似变换与两次圆柱投影组合到原 NURBS；最终曲线长度约为 `1.816765734864 m`，S0 约为 `1.816761985238 m`。最终曲线按真实弧长 `s=ell/L` 查询，`s` 无量纲，位置与其一至三阶 `s` 导数使用 m，单位工具轴与其 `s` 导数无量纲。

工具局部 `+X` 指向圆柱轴，局部 `+Y` 使用圆柱轴向，完整旋转提供名义滚转；绕工作轴的转动自由。导出源工具 `+Z` 表达时，使用 manifest 中的 `C_source_tcp=Ry(-pi/2)`。`normal` 是生成的圆柱向外法向，原始表面关联与原法向的状态单独保存。

S0 已应用的名义工作距离为 50 mm。默认拟合 profile 的物理工件距离状态记录为 `not_provided`。生成器保存当前路径半径，不额外移动目标位置。模型与坐标身份为名义仿真；真实安装、标定、工艺与执行能力由相应任务提供。

模型身份的受控关节名称及顺序直接读取主包 `robot_spec.DEFAULT_JOINT_NAMES`；manifest 同时保存该公共来源文件的 SHA256。

原 MAT 的名义运动字段和规划参数保留原单位及来源。原 `speed_series` 中的一个非有限值随有效性数组保存。末端两个 `u=1` 样本的源时间区间也保留；源时间插值用于名义信息查询。时间运动量通过给定的 `nu=ds/dt`、`nu_dot` 与 `nu_ddot` 计算。

## 生成、检查与加载

在具有项目依赖的 Python 环境中，从仓库根目录执行：

```bash
export PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PWD/portable_oscbf/vendor/dpax${PYTHONPATH:+:$PYTHONPATH}"
python -m robot_safecontrol_moveit.pipe_assets.butterfly_reference check data/nurbs/butterfly_reference_v1/production
python -m robot_safecontrol_moveit.pipe_assets.butterfly_reference query data/nurbs/butterfly_reference_v1/production --s 0.5
python -m robot_safecontrol_moveit.pipe_assets.butterfly_reference generate \
  --source-dir data/nurbs/butterfly_reference_v1/production/source \
  --kinematics-config portable_oscbf/config/nineaxis.yaml \
  --controller-profile config/oscbf_controller.yaml \
  --output .scratch/map147-152/reproduced-production
```

生成 S0 时添加 `--scenario-config config/static_experiment_s0.yaml` 并指定新的输出目录。生成入口拒绝已经存在的输出目录。来源快照可以在没有原本地研究文件的机器上重新生成参考。

```python
from work.continuous_path_reference import load_reference_bundle, time_derivatives

bundle = load_reference_bundle("data/nurbs/butterfly_reference_v1/production")
sample = bundle.reference.query(0.5)
motion = time_derivatives(sample, nu=0.02, nu_dot=0.0, nu_ddot=0.0)
```

`sample` 同时返回 `u`、位置、工具轴、名义旋转、位置和轴向的一至三阶 `s` 导数、名义时间区间、源索引区间、事件区间、弧长反解残差及有效性。`query_u` 用于来源核验。内部节点取右侧，终点取左侧；可以显式传入 `side="left"`。节点保持 C2，三阶导数按区间分别定义。

`work.jax_continuous_reference.query_reference_jax` 接受同一固定形状数组，支持 JIT 和进度自动微分。连续参考要求 `float64` 与 `jax_enable_x64=True`。JAX 查询返回 `valid`，调用者必须在采用结果前检查该字段。主机入口直接拒绝越界、非有限、退化、身份不符和反解残差超限。

JAX 的 `side` 参数同时确定空间查询及弧长反解的一至三阶自动微分。在源节点查询左侧运动量时，位置、工具轴和它们的自动微分使用相同的左侧区间；终点使用最后一个区间。

空间查询可在暂停时复用，`time_derivatives(sample, 0)` 返回零时间导数。`project(position_m, s_interval=(start, end))` 在给定的有序路径区间内完成局部投影，返回进度、原参数与距离。阶段转换、运动限制和实际覆盖验收由 Runtime、M07 和评价任务消费这些结果。

## 实际消费与保存

控制器的 `surface_normal` 分支与 `ExperimentConfig.load_path()` 明确要求连续包。加载时核验 MAT、模型、相似变换和两次圆柱投影身份；资源缺失或来源身份变化时拒绝构建路径。`PathGeometry.sample`、JAX 采样和移动参考使用同一连续表达，样本数组保留分支索引与既有形状接口。

连续与纯采样构造共用样本筛选处理。位置、旋转、名义进给、时间和连续弧长、导数采用同一组保留索引；完整连续参考仍保留全部源样本、源节点事件及重复终点的时间区间。

公开 `set_orientation_mode("fixed", fixed_R=...)` 会清除连续包与路径缓存，并按当前固定旋转重新构建采样路径。明确传入 `require_continuous=True` 时要求 `surface_normal` 模式。

使用 `PathGeometry.to_arrays()` 保存 NPZ，使用 `PathGeometry.from_arrays()` 恢复，读取时设置 `allow_pickle=False`。保存内容包含实际连续系数、弧长和最终参考身份。实时评价与记录回放均查询保存的连续参考。

路径记录保存版本、`continuous`/`sampled` 表示及绑定完整连续数组和参考身份的 SHA256。恢复时检查全部字段、维度、数值和方向；连续记录还会重新构建弧长与几何，核对积分规则、源参数、名义时间和进给，以及八组采样数组。连续身份与表达必须共同保存。既有纯采样记录可以通过原有八组数值数组恢复。

## 数值验证

验证入口为 `tests/test_butterfly_reference_assets.py` 和 `portable_oscbf/tests/test_continuous_path_reference.py`，覆盖真实文件生成与安装、全部原始样本、独立 BSpline、SciPy 积分、独立自动微分、逐阶差分及步长收敛、节点左右、源参数/最终弧长反解、单位、工具轴、工作距离、暂停、记录恢复和输入拒绝。

构造时使用 Bernstein 凸包界检查整个源区间的径向范围，并以轴向与角向导数多项式的区间界证明最终参数速度具有正下界。无法在规定区间预算内证明时直接拒绝。运行反解最多执行六次 Newton 更新和固定 16 点 Gauss 积分；结果要求弧长残差不超过 `2e-11 m`。这些数值检查参数用于连续表达计算。
