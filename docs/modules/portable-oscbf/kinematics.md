# 正逆运动学

模型来源为 `models/ninezzhou/urdf/ninezzhou.urdf`。`kinematics_data.py` 由生成脚本维护，J1 为直线关节，J2–J9 为转动关节。计算末端 `ee_link` 对应 URDF 的 `tool0`，包括 Link9 到工具端的 235 mm 固定延伸。

## POE 正运动学

`NineaxisKinematics` 使用空间坐标系指数积：

\[
T(q)=e^{[S_1]q_1}e^{[S_2]q_2}\cdots e^{[S_9]q_9}M.
\]

`S_i` 来自 URDF 零位下的关节轴，`M` 为零位末端变换。单配置、全部连杆及粒子群使用的批量计算共享这些参数。空间 Jacobian 使用同一组螺旋轴和伴随变换。

## 单位与接口

遵守[全项目单位约定](../../units.md)。旋转矩阵始终无量纲。

| 方法 | 输入 | 返回 |
|---|---|---|
| `ee_pose_mm_deg(q_mm_deg)` | J1 mm，J2–J9 ° | 位置 mm、旋转矩阵 |
| `ik_mm_deg(target_pos_mm, target_R, q_init_mm_deg)` | 位置 mm，初值 mm/° | 关节解 mm/°，或 `None` |
| `ee_pose(q)` | 内部关节 m/rad | 位置 m、旋转矩阵 |
| `solve_ik(target_pos, target_R, q_init)` | 内部位置 m、初值 m/rad | `ParticleSwarmResult` |
| `ik(target_pos, target_R, q_init)` | 内部位置 m、初值 m/rad | 满足容差的关节解 m/rad，或 `None` |

省略 `target_R` 时仅约束末端位置。指定 `target_R` 时分别检查位置和姿态。`target_R` 须为有限、正交、行列式为 1 的 3×3 矩阵。`q_init` 默认使用零位，调用者给出的初值必须满足 URDF 关节限位。

```python
import numpy as np
from work.nineaxis_kinematics import NineaxisKinematics

kinematics = NineaxisKinematics()
q_mm_deg = np.array([250, 30, -20, 40, -10, 20, -30, 40, -50], dtype=float)
position_mm, rotation = kinematics.ee_pose_mm_deg(q_mm_deg)
solution_mm_deg = kinematics.ik_mm_deg(position_mm, rotation, q_init_mm_deg=q_mm_deg)
```

## PSO 求解与终止

`particle_swarm_ik.py` 使用 `pymoo` 的有界 PSO。关节范围映射为 `[0, 1]`，粒子经反向映射后调用批量 POE。三个子群分别从初值附近及整个关节范围搜索，使用独立随机数生成器。停滞时重新初始化子群，保留已经找到的最佳结果。

目标函数为 `(position_error / tol_pos)² + 25 × (rotation_error / tol_rot)²`。姿态项权重用于控制搜索时两类误差的相对影响，最终通过条件仍使用各自容差。惯性系数为 0.7298，个体与群体学习系数均为 1.49618；速度上限为每个关节完整范围的 20%。全部被评估粒子都在关节范围内。

默认位置容差为 0.1 mm，姿态容差为约 0.0572958°。内部参数分别为 `tol_pos=1e-4` m 和 `tol_rot=1e-3` rad；项目接口使用 `tol_pos_mm` 和 `tol_rot_deg`。接受解时分别检查两个物理误差，任一未满足均继续搜索。

`max_iter` 默认 1500，限制搜索轮次；`swarm_size` 默认 128，指每个子群的粒子数量；`seed` 默认 0。指定相同模型、输入、参数与依赖版本时可重复结果。`ParticleSwarmResult` 提供 `success`、`q`、位置与姿态误差、迭代次数、评估次数、重新初始化次数与收敛曲线；其中 `q`、误差字段使用名称对应的内部 SI 单位。

达到预算仍未满足容差时，`solve_ik` 返回 `success=False` 及最佳候选用于诊断，`ik` 与 `ik_mm_deg` 返回 `None`。`ik_multiple` 在第一个未满足要求的目标处抛出异常。输入形状、有限性、限位或容差不合法时立即抛出 `ValueError`。

PSO 用于离线求解与轨迹起点准备。数值搜索成功表示模型下的位姿误差满足要求；机械臂执行仍由既有轨迹、碰撞与控制流程检查。

仿真演示、姿态复现程序和控制闭环基准使用关节范围中点作为轨迹起点搜索的初值，随后检查位置及姿态容差。初值会影响冗余机械臂得到的关节构型；整条轨迹的可执行性由闭环检查验证。

## 验证入口

`portable_oscbf/tests/test_particle_swarm_ik.py` 使用 URDF 解析库建立独立的逐关节变换，检查 POE、关节范围、可达目标、位置和姿态独立验收、预算不足、随机数重复性以及 mm/° 边界。目标由合法关节配置生成。

```bash
python3 -m pytest -q portable_oscbf/tests/test_particle_swarm_ik.py \
  portable_oscbf/tests/test_fk_matches_urdf.py \
  portable_oscbf/tests/test_generated_kinematics_data.py \
  portable_oscbf/tests/test_cbfpy_dependency_contract.py
```

## 本机验证记录

2026-10-26 使用项目环境 `.scratch/host-recovery/venv` 验证，Python 3.10.12、NumPy 1.26.4、SciPy 1.15.3、pymoo 0.6.2。输入和返回关节均检查 URDF 限位，目标及最终误差使用独立 URDF 逐关节变换计算。

| 检查 | 覆盖 | 结果 |
|---|---|---|
| POE 正运动学 | 278 组配置、3058 个连杆变换，包含零位、各关节边界和随机配置 | 全部通过，最大位置差约 1.3×10⁻¹² mm |
| Jacobian | 33 组配置，边界使用单侧差分 | 全部通过 |
| 仅位置 PSO | 28 个目标，各使用目标初值、附近初值和零位初值 | 84/84，通过误差最大 0.09997 mm |
| 完整位姿 PSO | 上述 84 次求解，另加 20 个独立随机目标与多个随机种子 | 104/104，最大位置误差 0.09992 mm，最大姿态误差约 0.05570° |
| 控制闭环（2026-10-27） | 当前仓库轨迹、3000 个控制步 | QP 全部通过，最大稳态位置误差 0.15281 mm，满足 ≤0.5 mm 要求 |
| 相关 pytest | 运动学、单位、依赖、轨迹与控制 | 71 项通过 |
| 安装与程序入口 | ROS 主包构建、仿真演示及姿态复现的真实起点函数 | 构建通过，两个入口的位置误差均约 0.09078 mm、姿态误差均约 0.05549° |

28 个目标由零位、所有关节下限、所有关节上限、范围中点及 24 组随机配置组成。另 20 个目标使用独立随机序列生成。各项成功条件均使用原定的 0.1 mm / 0.0572958° 容差。

本次完整位姿搜索最慢约 18.2 s，最高 1302 轮。时间记录用于判断离线计算成本。详细数值及源码摘要保存在本机 `.scratch/pso-ik/validation-results.json`，控制记录位于 `.scratch/tracking-precision/baseline-after/baseline_tracking.npz`。闭环误差定义、进度与限值检查见[控制内核位置精度回归](../../tracking_evaluation.md#控制内核位置精度回归)。

相关 pytest 覆盖运动学、依赖版本、单位换算、轨迹加载与控制闭环；全仓测试入口包含使用 mock 的测试，本次按工作约定执行真实数值和实际模块的相关检查。
