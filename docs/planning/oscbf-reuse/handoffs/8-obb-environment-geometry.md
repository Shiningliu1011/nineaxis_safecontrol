# [12] 自碰撞 OBB 与环境几何模型核对（研究）

- Tracker：[自碰撞 OBB 与环境几何模型核对（研究）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8)。
- 日期：2026-09-08；认领：Shiningliu1011。
- 状态：本轮上游核验和隔离反例复现完成；整票 OPEN，几何与导数准入尚未满足。本记录不是 resolution。
- 已同步[阶段进度评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8#issuecomment-5586937158)。

## 起点与选票

用户原话：“继续下一个ticket”。正式地图及推荐顺序已读取；前票[官方驱动交接](31-official-sensor-drivers.md)建议时间同步窗口，但该票被未完成的官方驱动原生阻塞。按 wayfinder 的未阻塞、未认领 frontier 规则，本轮选取推荐顺序中下一个符合条件的本票；先认领再研究。未开始时间同步或其他票。

主工作区 main，HEAD `659da6c6db598abddc272ad0941b72f77793211b`。保留已有 ADR、MAP、感知规格修改，以及未跟踪 handoffs、研究、.scratch 和大然电机资料。产品文件未修改。上游研究在独立 worktree `.scratch/oscbf-reuse-wayfinder/8/research-worktree`、分支 `codex/research/obb-environment-geometry` 提交 `dc85e38`，已复制回主工作区，未推送。

## 证据与复现

- [上游接口、许可证和误差预算研究](../research/obb-environment-geometry.md)。
- [隔离审计脚本](../research/geometry-audit/audit_geometry.py)、[完整结构化结果](../research/geometry-audit/audit-results.json)、[审计输出](../research/geometry-audit/audit.log)、[既有测试输出](../research/geometry-audit/existing-tests.log)。JSON 记录解释器、依赖版本、URDF collision 原点/mesh、模型和源码 SHA256。
- 本机 Python 3.10，JAX/jaxlib 0.6.2、numpy 1.26.4、trimesh 5.1.0、python-fcl 0.7.0.11；Python binding 版本不自动证明链接的 FCL 核心等于上游研究标签。

在仓库根目录运行：

```bash
PYTHONPATH=portable_oscbf/vendor/dpax OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 JAX_PLATFORMS=cpu python3 docs/planning/oscbf-reuse/research/geometry-audit/audit_geometry.py
python3 -m pytest portable_oscbf/tests/test_obb_model.py portable_oscbf/tests/test_obb_distance_jvp.py portable_oscbf/tests/test_fk_matches_urdf.py -q
```

最终两条命令均退出 0。既有测试 **11 passed in 10.19s**；审计脚本输出测量值，不将发现反例伪装为通过验收。初次 scratch 运行未注入 vendor 路径，因缺少 dpax 退出 1；最终复现命令已显式补齐路径，没有安装依赖。

## 本次实测结论

### OBB 表示和碰撞对

10 个连杆的全部 STL 顶点均在各自 OBB 内（1 µm 检查容差；最大正超出约 8.52e-13 m，仅浮点/常量截断量级）。两份模型目录的对应 STL SHA256 全部一致；两份 URDF 的这 10 个 collision 均为零 origin、未指定 mesh scale。对于这些三角网格，盒为凸集，顶点包围也覆盖三角面；这不是实物装配、公差或附件覆盖的证明。

当前 14 对只覆盖 36 个非相邻组合中的 14 个，另 22 对见 JSON。未列入不等于漏碰撞已经发生，但每个排除需独立机械/构型依据。Link6 完全没有进入自碰撞对，生成器与 mesh checker 沿用“Link5 已覆盖 Link6”的注释。零构型将 Link6 网格变换到 Link5 坐标后，**32,647 个顶点中 5,122 个在 Link5 OBB 外，最大轴向超出 0.113879 m**。所以旧注释不能作为当前 OBB 覆盖依据；不能从此实验直接推断所有这些点会撞上其他连杆。

`tool0` 没有 collision 几何；控制点位置不等于实际工具体积。TCP 附件、线缆、相机安装件、固定工装和真实装配仍缺 CAD/实测清单。Link3–Link5 排除的历史网格扫描仅在源码注释中引用，本轮未重跑全限位扫描，也未证明其适用于所有模型变化。

### 距离及导数的独立反例

调用现有 `_obb_pair_distance`，轴对齐盒的实体无符号解析参考为 `norm(max(abs(t)-half_A-half_B,0))`，同时使用本机 FCL Box–Box 距离及独立 collide 查询。

| 场景 | 解析实体距离 | FCL 普通距离返回 | 现有内核 |
|---|---:|---:|---:|
| 相同盒分离，半长 1，中心差 (3,0,0) | 1 m | 1 m | 1 m |
| 异尺寸分离：半长 A=(.1,.07,.06)、B=(.04,.03,.02)，中心差 (.31,.017,.023) | .170 m | .170 m | .150658554 m |
| 相同盒接触 | 0 | 0 | 0 |
| 相同盒部分相交 | 0 | -1（哨兵，非穿透深度） | 0 |
| 同中心完全包含，外盒半长 1、内盒 .2 | 0 | -1（collide=true） | +.2 m |

异尺寸分离的十倍尺度场景中，解析梯度为 `(1,0,0)`；现有 JAX 梯度约 `(0.929254,-0.351789,-0.112838)`，最大分量误差 .351789。它与**自身函数**中心差分却只相差 7.88e-12。这说明旧的“导数与同一几何实现一致”测试不能独立证明几何/梯度正确。该场景保持沿 x 的正间隙，y/z 投影重叠，解析距离在邻域内光滑，不是在非光滑点强求唯一梯度。

分离反例为低估距离；包含反例为碰撞却返回正距。不得混为全部都高估净空。本轮未定位所有内部根因、未更改实现，也未推断这些合成盒必然对应某个实际机器人碰撞构型。

## 决定与复用结论

无新的用户取舍。继承自碰撞保留 OBB、成熟上游优先且按证据择优、双源连续观测、失败锁存等已接受边界。

- 保留现有 OBB 数据和生成器作为已核验网格包络基线；保留表示不意味着距离内核及排除表已获准入。
- FCL Box–Box 是当前独立对照候选；环境继续优先验证 PCL/实体体素/FCL。版本和 BSD 许可证见上游研究。不因这几个例子通过就替换正式 JAX 热路径。
- 必要适配包括对象/叶身份、坐标和单位、查询状态、最近点、关节 Jacobian、动态时变项、覆盖和年龄。FCL C++ 距离不能直接成为 JAX 可微常数。
- FCL 0.7.0 Box–OcTree signed fallback 的 scalar/result 不一致及重复 contact 点风险已由源码发现，尚未本机实测。接触、穿透不能沿用正间隙的点差归一化。
- 数值误差预算只能先给分项公式，尚不能冻结毫米阈值；少量反例的误差不是全域误差上界。

## 剩余缺口与编码门

本票不能关闭，不能声明下游几何前置已满足。尚缺：

1. 修正或替换距离实现后，独立解析/FCL 对照的旋转盒、接触、穿透、包含、尺度及多步长导数证据；九轴点 Jacobian 和最近特征切换验证。
2. 排除对逐项依据，特别是 Link6 覆盖与全关节域接近构型；未简化网格/实物包络和附件清单。不要自动加入所有近邻对而制造不可满足的约束。
3. Box–OcTree 单/多叶、未知/空树、压缩叶、交换对象顺序、最近点和 signed 返回矩阵；动态更新与固定环境模型并存。
4. 真实双源覆盖、时间、标定误差与环境回放；漏检/误报和端到端容量、尾延迟。

**可以开始**独立回归用例、离线几何修复/上游内核适配对照和碰撞对审计；正式产品代码实施仍待用户实施指令，沿用窗口默认范围。**尚不能**把现有结果接作已验证的连续环境距离/导数，或冻结控制裕度与真机准入。

修复验收要覆盖本报告反例、独立参考、未改变 OBB 表示边界、有效性失败路径及性能证据；不能仅重跑现有自比较测试。回退保留原模型、配置、实现和证据；恢复旧实现不消除本次已知缺口。

## 改动与后续

本票新增上游研究、隔离审计及输出、本交接；无产品运行行为变化、无安装、无硬件运行、无主分支提交/合入。主分支已有修改保持原样。已知缺口仍由本票持有，不创建同义修复票、不撤销下游依赖、不追加地图 Decisions so far。

本票后续应先完成上述几何修复和核验；若继续选取新的未阻塞、未认领主队列研究窗口，当前是[冗余策略对比：9-DOF 对 5D 工具轴任务](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5)，下次必须重新查询 frontier。本窗口不自动启动它。
