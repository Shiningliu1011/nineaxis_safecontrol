# 自碰撞 OBB 距离内核：隔离复现与准入核验（2026-09-12）

日期：2026-09-12。范围：对 `portable_oscbf/work/dpax_collision.py` 的**隔离复现与核验**，不是产品替换决议、不是控制准入、不是真机报告。产品代码未修改；未运行 ROS/真机；未安装依赖。本文件是票据 [12] 自碰撞 OBB 与环境几何模型核对（研究）本轮的研究证据，票据保持 OPEN。

上一轮（2026-09-08）的[交接](../../../handoffs/8-obb-environment-geometry.md)给出五组反例并据此把距离内核判为未准入。本轮用独立参考把这些反例逐条重做，结论分两半：**上一轮的两个距离/导数反例是调用形状产物，不是生产路径的内核缺陷**；但**重叠盲区（overlap blindness）是真实的**，在真实连杆几何上至少有 3 处真实网格互穿被内核报成正间隙。详细结论见下。

## 复现方式与环境

```bash
cd docs/planning/oscbf-reuse/research/geometry-audit/kernel-validation-2026-09-12
for s in sanity_refs minrepro3 minrepro minrepro2 facesbug helperdiff hlo isolate callpath \
         bisect_regimes validate_kernel verify_qcase mech shapes rule2x2 recon exclusion_audit \
         realpair_accuracy mesh_collision_confirm derivatives; do
  PYTHONPATH=$PWD/../../../../../portable_oscbf/vendor/dpax JAX_PLATFORMS=cpu \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -u $s.py
done
```

脚本按顺序写入 `RERUN.log`（含每个脚本的 `exit=` 标记），并把结构化结果写到同目录 `*.json`；`RERUN.log` 是全绿复现记录。本机 Python 3.10.12，JAX/JAXlib 0.6.2（`jax_enable_x64=True`）、numpy 1.26.4、trimesh 5.1.0、scipy 1.15.3、python-fcl 0.7.0.11，CPU 后端。`rtree` 未安装，因此**没有**使用 trimesh 的 `contains`/`proximity`/`signed_distance`；网格层真值用 FCL BVH 与自写的三角–三角 SAT 两条独立路径。

被核验源码的 SHA256（与上一轮审计记录一致）：

| 文件 | SHA256 |
|---|---|
| `portable_oscbf/work/dpax_collision.py` | `91a7c144fd0f4a3ff9fe603796197f2ed1513e094626b870f29540637dda4dcb` |
| `portable_oscbf/work/obb_collision_model.py` | `1ea08f39ade0fe943068522b2e14e2a8601170d0d6639cd8b1f7b112d651a0a7` |
| `portable_oscbf/work/nineaxis_manipulator_jax.py` | `b01c18d8b4ea4b8b7267f76a483ff11ea3407e5bc44574359f1c5e9f04f01396` |
| `portable_oscbf/work/nineaxis_kinematics.py` | `7cec2c14d630272d5c4e3ea38186284d90b459fd611c9554e92ceb80496e035f` |
| `portable_oscbf/tests/test_obb_distance_jvp.py` | `9f3b8983c562cb96e9a5e1b1226bda84c706c532c776d1a00889990b8faf2fe5` |

独立参考共 7 条，互不共用代码：解析轴对齐闭式（`refs.analytic_aa`）、SLSQP 约束 QP（`refs.qp_reference`，可多起点）、支撑映射 GJK（`refs.gjk_reference`）、FCL 0.7 Box–Box（`refs.fcl_reference`）、15 轴 SAT（`oracles.sat`，给出碰撞状态、穿透深度与**分离态的净空下界**）、FCL BVH 三角网格 collide/distance（`oracles.mesh_collision`）、依赖自由的三角–三角 SAT（`mesh_collision_confirm.tri_tri_sat`，11 轴）。后两条是全套参考里唯一能回答“**连杆**（而不只是它们的包围盒）是否接触”的。

## 结论一：上一轮的两个距离/导数反例是调用形状产物

`validate_kernel.json::harness_self_check` 记录了同一批数字在四种调用形状下的取值（轴对齐异尺寸盒，解析值 1.7）：

| 形状 | 值 |
|---|---:|
| 解析闭式 | 1.700000000 |
| eager（逐参数具体值直接调用） | 1.700000000 |
| `jax.jit`，所有实参被 trace | 1.700000000 |
| 生产批量路径 `_pair_distance_vmap` | 1.700000000 |
| `jax.jit`，旋转作为**闭包常量** | **1.506585544** |

机器人尺度场景同样：生产形状 0.170000000，闭包常量形状 0.150658554 —— 这正是上一轮记入交接的“内核低估 0.0194 m”的那个数。也就是说：**该反例只在一个手搭的调用形状里出现，在该形状下旋转矩阵是编译期常量**。

`shapes.py`（16 种常量/被 trace 组合的完整矩阵）与 `rule2x2.py` 把这个条件钉死了：

| 世界旋转 | 半长 | 面心与 numpy 真值最大差 |
|---|---|---:|
| 常量（transform 与 local 旋转都是常量） | 常量 | **9.464e-01 m** |
| 常量 | 被 trace | **9.464e-01 m** |
| 被 trace | 常量 | 2.776e-17 m |
| 被 trace | 被 trace | 2.776e-17 m |

规则只有一条：**世界旋转 `transform[:3,:3] @ rotation_local` 是编译期常量时被错误降级；只要它是被 trace 的，结果就是精确的**（半长常量还是被 trace 无关）。`rule2x2.py` B 段用真实模型几何复核：同一个 q 下 QP 参考 0.009799083，生产形状误差 −8.1e-16，而上一轮的常量形状误差 −4.1e-03。

`mech.py` 给出该错误降级的可证伪刻画（以恒等世界旋转为例，把 jit 结果与逐项重建比较）：

- eager (6,3) 与 numpy 真值一致；jit 结果**是正确值的置换**（每列排序相等、但元素位置被打乱），最大差 1.0。
- jit 结果**逐位等于**“把 (3,6) 乘积按 C 序 reshape 成 (6,3) 再逐元素加面心”：`reshape_reconstruction_exact = true`（恒等旋转与 `rot_z_0.4` 精确成立；`rot_xy_1.1` 最大差 0.189，说明它不总是一个纯 reshape，而是更一般的布局/索引错配，本报告只按“置换”这一可观测事实陈述）。
- **去掉 `.T` 之后乘积本身已经错**（`prod_no_transpose_maxdiff = 1.0`），所以这不是 `.T` 的语法问题。把 `.T` 改写成 `jnp.swapaxes`／`jnp.transpose`／`jnp.einsum('ij->ji')` 四种写法结果完全相同且同样错（`mech.json::rewrites_max_jit_vs_eager` 四者相等），HLO 里仍是真实的 `stablehlo.transpose`——**改写语法不能规避**。

**本机边界（必须明说）**：这是 JAX/JAXlib 0.6.2 + CPU 后端在这台机器上的常量折叠行为；本轮**没有**定位到 XLA 内部的根因，因此不宣称“已修复”或“上游 bug 编号”。上一轮交接里我记录的“`.T` 被降级为 C 序 reshape”这一因果说法，只对部分旋转成立，本报告已按上述实测重述。

### 生产路径是否可达该形状：不可达（本次实测结论）

`shapes.py` 对公开生产入口 `self_collision_distances` 做了 6 种调用形状，全部与 eager 逐位相同（最大差 `0.000e+00`）：eager、`self_collision_distances_jit`、外层 `jax.jit`、`jax.vmap`、**把 q 作为闭包常量**（`jax.jit(lambda: self_collision_distances(Q_CONST))()`）、以及逐对 `pair_distance`。同一 q 下与 14 个声明对的 QP 参考最大差 3.109e-15（eager 与 jit 同值）。

原因是结构性的：本地几何（`OBB_LOCAL_CENTERS_M`／`OBB_HALF_EXTENTS_M`）与半长取自同一组模块常量，而世界旋转始终由 q 派生的 transform 引入；即使传入全具体的 q，`jax.jit` 也会把实参重新 trace，因此世界旋转永远是被 trace 的。要踩到该缺陷，必须在 jitted 函数内部**闭包持有旋转矩阵**——上一轮审计脚本与本次的 stress 形状都是这么搭的，生产路径不是。

**由此得到的建议（不是决议）**：(1) 保留一个把生产形状钉住的回归用例（`self_collision_distances` / `obb_sphere_distances` 对 QP 参考），防止未来调用者改成闭包常量形状；(2) 任何把几何提升为编译期常量的“预编译 jitted 可调用对象”都应视为需要重新验收的改动。产品代码本轮未改，是否加固内核留给实施指令。

## 结论二：分离态的精度与导数（生产形状）

- 真实连杆几何：`realpair_accuracy.py` 60 个限位内构型 × 36 个非相邻对，共 2160 个配对样本，最坏单对最大误差 **2.292e-07 m**，出现在 Link3/Link5（一个**被省略**的对）在其最近采样净空 1.8377e-04 m 处的低估，即近接触区相对误差约 1.2e-3、绝对量仍为亚微米；36 对的最大高估均为 5.20e-16 m 量级；每个配对在其最近采样构型上 kernel == QP == GJK（6 位小数）。
- 合成旋转电池：`validate_kernel.json::random`，N=400（319 分离），生产形状与 eager 同值，最大 |误差| **4.808e-06 m**，p95 1.018e-14，中位 2.220e-16，最大低估 −5.00e-12 m；超过 1 µm 的只有 3 例，无 1 mm 量级。
- 九轴 q 空间链：`derivatives.py`，41 构型 × 14 对 = 574 个配对样本对 `qp_reference`，p99 5.395e-13、中位 3.053e-16；超过 1 µm 的**只有 2 行**，且两行都是互穿（见结论三：参考分别报 0 和 7.2e-09 的 QP 容差噪声，而 GJK 判 overlap、FCL 判 colliding）。
- 特征切换扫描：41 步路径上，参考非零的 25 行（分离区）**|kernel − ref| ≤ 7.09e-14**；`max_abs_err = 4.712e-04 m` 只出现在参考报 0 的 16 行互穿平台段，内核在该段返回常量 4.712e-4（第 9 行更低，8.07e-05）—— 这与“分离区没有切换错误”并不矛盾（见结论三）。
- 导数：4 个解析可算的场景中，`∂d/∂c` 与解析法向最大差 **3.33e-16**；九轴 q 空间 autodiff 对中心差分最大绝对差 3.84e-10（相对误差最大 13.3 只出现在梯度≈0 的分量上）。
- `verify_qcase.py` 另修正了上一轮 `derivatives.json` 里的一个harness错误：`fk_agreement_max = 0.188649997115` 是把连杆原点平移与 OBB 中心（`R@c+t`）相比导致的，正确比较同构型下 transform 对 transform 的最大差为 **5.551e-16**。该字段不应被引用为 FK 误差。

## 结论三：重叠盲区是真实的，且存在 3 处真实网格互穿

内核的距离是“边–边最小值”与“点–面最小值”的 `min`；两个 OBB 相交时两个项都饱和到 0，`min` 仍返回 0，而 `_obb_pair_distance` 是被梯度用的标量输出，CBF 里 `h = d − d_safe` 因此把“互穿”和“刚好接触”混为一谈。本轮量化了这个盲区（`exclusion_audit.py`）：

- 3001 个限位内构型 × 36 个非相邻对，SAT 判相交、且内核返回值仍 **> 1e-9 m** 的事件共 6682 个（这就是 `blind` 事件的定义：不存在任何一个内核返回 ≤1e-9 的 SAT 相交事件；20 例样本的返回值范围 1.05e-04 … 1.86e-02 m，见 `blind_events_head`）。这是按 OBB 计的统计，OBB 相交大多是包络粗糙而不是真实接触（非碰撞事件处网格最小净空 8.512e-03 m）。
- 对这 6682 个事件做 **FCL BVH 网格级真值**扫描：**3 个是真实网格互穿**，全部是 `base_link/Link9`（已声明对）：

| 构型 | SAT 穿透深度 | 内核返回值 | 网格真值 | 三角–三角 SAT 独立确认 |
|---|---:|---:|---|---:|
| cfg=196 | 0.10447 m | +3.169e-03 m | 互穿 | 97 个相交三角对 |
| cfg=1371 | 0.08699 m | +4.305e-03 m | 互穿 | 36 个相交三角对 |
| cfg=2039 | 0.07664 m | +9.113e-04 m | 互穿 | 27 个相交三角对 |

  零构型对照在同一对、更大采样量（4,000,000 三角对）下相交三角对为 **0**，所以这不是三角–三角 SAT 的假阳性。三个构型的 q 见 `mesh_collision_confirm.json`，均在关节限位内（限位从 `nineaxis_manipulator_jax` 导入，未硬编码）。
- 三点均可复现且互相独立（FCL 与自写 SAT 两条路径）；`base_link/Link9` 的真实穿透最深处约 2.8 cm，而内核报的是 +0.9… +4.3 mm 的正间隙。

**含义（不代替用户决议）**：`h ≥ 0` **不蕴含**不互穿；在 OBB 相交时内核的数值既不是距离也不是穿透深度。修正方向（在 OBB 相交时给出显式状态，例如 SAT/GJK companion 判据，把饱和区从 `min` 里暴露出来）属于内核语义变更；“用多大的 d_safe/margin 才能覆盖这类盲区”属于安全阈值取舍——按地图的人工复核规则，本报告**不替用户决定**，也不把该建议写进任何决议。

## 结论四：排除对与覆盖审计

36 个非相邻连杆对中只声明了 14 对（`OBB_COLLISION_PAIRS`）。按 SAT 的 OBB 接触统计，被省略但对 OBB 达到了接触的只有 5 对，且抽样中没有一对发生真实网格碰撞：

| 省略对 | OBB 相交构型数 | 最大 OBB 穿透 | 该处网格最小净空 |
|---|---:|---:|---:|
| Link5/Link7 | 2470 | 0.0233 m | 0.0265 m |
| Link6/Link8 | 2266 | 0.0203 m | 0.0550 m |
| Link3/Link5 | 1680 | 0.0089 m | 0.0195 m |
| Link7/Link9 | 239 | 0.0014 m | 0.0367 m |
| Link1/Link6 | 2 | 0.0339 m | 0.0706 m |

其余 17 个省略对在 3001 个抽样构型中从未达到 OBB 接触（例如 Link2/Link6 分离态 SAT 下界最小 0.1113 m）。已声明对里，分离态净空下界最小的是 Link1/Link9（1.135e-03 m）、Link1/Link5（1.635e-03 m）、base_link/Link8（3.755e-03 m）；这些对达到 OBB 接触时内核=QP=GJK=0 而网格仍相距 30.4–99.3 mm —— 这是**裕度预算**问题，不是内核错误。

`pair_minimum_clearance` 表（内核梯度下降寻优 + QP/GJK/网格复核）显示在每个配对找到的最小净空点上 **kernel_min == ref_qp**（36/36），即寻优也没有暴露分离态的距离错误。

**Link6 的依据必须重写。** Link6 没有进入任何自碰撞对；生成器与 mesh checker 沿用的注释称“Link5 已覆盖 Link6”。上一轮实测（本轮复核保留）在零构型下把 Link6 网格变换到 Link5 坐标后，**5122/32647 个顶点在 Link5 OBB 外，最大轴向超出 0.113879 m**，所以该注释不是集合包含，**不能**当作覆盖依据。但本轮在 3001 个限位内构型 × 全部含 Link6 的非相邻对上，`mesh_collision` 没有发现任何一次真实 Link6 网格接触。因此正确表述是：**“在本次抽样范围内未观察到 Link6 的真实网格接触”是一个抽样几何界，既不是注释所说的覆盖证明，也不能外推为全关节域的证明**。

## 候选修复（未实施）

1. **内核语义**：在 `_obb_pair_distance` 内增加重叠判定（SAT 15 轴或 GJK companion），使 OBB 相交时输出显式状态/负穿透，而不是 `min` 饱和出的正值。这会改变下游 `h` 的取值符号语义，需与 `d_safe` 策略一起验收，**未实施**。
2. **调用形状守卫**：不要在 jitted 函数内以闭包常量传旋转；生产路径已满足。可加回归用例钉住（见结论一）。**未实施**。
3. **排除表**：是否把 Link5/Link7、Link6/Link8、Link3/Link5 加入约束是控制设计取舍（会显著增加约束数），**未实施、未决议**。

以上三条都不在生产代码里；`git status` 的产品文件未变（本次新增内容全部在 `docs/` 与 gitignore 的 `.scratch/`）。

## 证据清单

`docs/planning/oscbf-reuse/research/geometry-audit/kernel-validation-2026-09-12/`：`sanity_refs`、`minrepro`/`minrepro2`/`minrepro3`、`facesbug`、`helperdiff`、`hlo`、`isolate`、`callpath`、`bisect_regimes`、`validate_kernel`、`verify_qcase`、`mech`、`shapes`、`rule2x2`、`recon`、`exclusion_audit`、`realpair_accuracy`、`mesh_collision_confirm`、`derivatives` 各脚本与其 JSON，共 20 个脚本 + 2 个共享模块（`refs.py` 解析/QP/GJK/FCL 参考、`oracles.py` SAT/FK/网格 BVH）+ 10 个 JSON + `RERUN.log`。`bisect_regimes.py` 本轮修正了两处自身缺陷（原先把世界旋转同时经 transform 与 local 旋转施加了两次，因此报出的 −5.0e-02 “分离态误差”是 harness 约定错误；修正后同一构型全部形状误差 −5.55e-17）。

## 未决与边界

- **未做**：全关节域穷举（本轮为 3001/3001+ 抽样）、真实硬件、ROS、控制器链路、尾延迟与频率、附件/TCP/夹具包络、`d_safe` 标定。
- **需要用户决议**：重叠盲区对应的 `d_safe`/裕度策略；是否扩排除表；Link6 依据改写的口径；是否加固内核（结论一/三的候选修复）。本轮不代答。
- **可以开始编码**（在用户实施指令下）：把结论一的生产形状回归用例、结论二的参考对照、结论三的重叠状态判据做成隔离回归；扩展排除审计到全关节域抽样。
- **尚不能**：把现有内核接作“已验证的连续环境距离/导数并允许真机运行”，或在未决定裕度策略前冻结 `d_safe` 与真机准入。票据保持 OPEN。
