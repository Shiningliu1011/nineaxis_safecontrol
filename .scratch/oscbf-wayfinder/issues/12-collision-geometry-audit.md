# 12 — 自碰撞/障碍几何模型核对（OBB / 球槽 / 32 球包络 vs 真实几何）

**What to build:** 自碰撞用 M2 OBB 模型（10 OBB + 14 碰撞对 + exclusions
Link3–Link5）经 dpax 边对+点面内核得到真距离,障碍用 8×10 槽球,整臂还有
32 球环境包络(enable_sdf 时)。这些模型是否与真实连杆几何(URDF/STEP 网格)
一致、排除的 Link3–Link5 是否安全,目前没有核对证据。本票:

- 用 URDF/collision mesh 顶点对 OBB 半长轴/中心做最小包围核对(或给出
  「数据不可得」结论并标 Not yet specified);
- 核对 exclusions 对安全性影响的论证(是否覆盖到接近构型);
- 核对 8 槽球表示对外形保守度(球半径 vs OBB 半长轴);
- 产出: 一致性报告 + 修正项。

**Blocked by:** 部分依赖 URDF/CAD 数据是否存在(标 Not yet specified 的部分
不得臆造)。

**Queue:** wayfinder-core
**Tracker:** #8 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8)

**Status:** ready-for-agent(数据可得部分)

- [ ] 定位并读取 URDF/collision 网格数据源
- [ ] OBB vs 网格包围核对(逐连杆)
- [ ] 8 槽球/32 球保守度对比表
- [ ] exclusions 论证(或明确标注「仅凭经验排除,需实测」为未定项)
