## 7. 碰撞模型

### 7.1 自碰撞 —— OBB 几何 + DCOL 可微距离内核（目标）

> ⚠️ **现状是 17 球 + 14 对**（`collision_envelope.py` 的 17 个球 + `oscbf_collision_config.py` 的 `SELF_COLLISION_PAIRS` 14 对），本文描述的是**目标**（每连杆 OBB + DCOL）。

**要求**（目标）: 不用球模型；每个机械臂关节系下定义**一个 OBB（定向包围盒），对齐该关节坐标系 xyz 轴**，尺寸参考 STL 实测，**紧密贴合连杆几何，不产生大范围包络空气**。

`obb_collision_model.py` 定义每个 OBB 的参数：

```python
OBB_LINK_INDICES:        每个 OBB 附着的连杆索引
OBB_LOCAL_CENTERS_M:     盒心在连杆坐标系的位置
OBB_HALF_EXTENTS_M:      半边长 (hx, hy, hz)，按 STL 包围盒标定
# 姿态: 默认对齐连杆系 xyz 轴; 若连杆系未对齐 STL 主轴, 附加局部旋转 R_obb
```

**距离内核 = DCOL**（`dpax_collision.py`）:

```python
# 每步: FK 得各 OBB 世界位姿 → DCOL 可微距离
from dpax.endpoints import proximity          # 线段/点基元可微距离
from dpax.polytopes import polytope_proximity # OBB/凸多面体可微距离
proximity_jit = jax.jit(proximity)
grad = jax.grad(proximity, argnums=(...))     # 提供 ∇h

h = d_DCOL(OBB_i, OBB_j) - d_safe
∇h = jax.grad(proximity) @ J_point            # DCOL 梯度 → CBF 行
```

**DCOL alpha 校准**: 每个自碰撞约束行的 CBF 增益 `α` 必须**离线校准**（V2 DPAX alpha 校准流程：3/14 行已定、拓扑已认证、11 行需近接触数据）。⚠️ 该"14 行"对应**现状球对拓扑**的 14 对；切换到 OBB 目标拓扑后配对数量会重新确定，需按新拓扑重跑校准。alpha 不是全局常数，是逐约束行参数。

**标定要求**:
1. 从 STL 求每连杆的 OBB：对连杆系坐标下的网格顶点求主轴方向 → 旋转使 xyz 对齐 → 求最小包围盒
2. 若连杆系与 STL 主轴有固定偏差，在 `R_obb` 中记录，随 FK 组合
3. **验证**：OBB 体积 / 连杆包围盒体积比尽量接近 1（包络空气最小）；RViz 渲染 OBB 与 STL 重合检查
4. 确定自碰撞对：仅非相邻连杆，跳过已标定拓扑豁免（机械近邻按 L6/L71 流程扫描确认）
5. 逐对校准 DCOL alpha（近接触数据驱动）

### 7.2 环境碰撞 —— DCOL 障碍物 + FCL/ESDF 原始点云感知

- **障碍物基元距离走 DCOL**（与自碰撞同一可微内核）：机器人 OBB vs 障碍物基元（球/胶囊/盒）
- 输出转为 `obs_pos / obs_radii / obs_enabled / obs_d_safe / obs_vel` 传入 JAX 内核
- **原始点云感知仍用 FCL/ESDF**：`point_cloud_obstacles.py`（FCL 距离）、`safety_snapshot.py`（ESDF 距离场）——DCOL 当前无点云原语，点云经体素化/ESDF 后进入

**边界**: 原始点云感知不追求可微（L67 感知 I/O 边界），但**碰撞距离计算全部走 DCOL**（自碰撞 + 基元障碍物），FCL 仅保留在原始点云感知与离线验证。

---
