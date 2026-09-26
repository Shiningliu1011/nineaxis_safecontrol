# 生成组合 ellipsoid 几何资产与覆盖证明

## 来源

- GitHub 票据：[I20] 生成组合 ellipsoid 几何资产与覆盖证明。
- `docs/planning/oscbf-reuse/spec-collision-safety-module-20260921.md` 的 User Stories 52–57、Implementation Decisions 4–6 及 geometry artifact 测试要求。
- ADR 0011、0012、0032。

## 目标

为机器人本体、J1 滑台及当前末端工具生成可复核的 `Collision geometry artifact`。每个参与碰撞的 link 使用固定容量的组合 ellipsoid 与 `active_mask`，其体积覆盖依据绑定源 mesh 和 `geometry_hash`。

## 需求

- 输入清单明确列出全部生产 collision mesh、link identity、工具 identity、坐标单位及文件来源；缺项立即拒绝。
- 每个 mesh 必须经过 closed、拓扑、方向、正体积与单位检查。生成过程保留原始文件 hash。
- 对 mesh volume 执行四面体化；每个四面体的四个顶点须同时位于至少一个启用的 outer ellipsoid 内。未覆盖的四面体拒绝生成。
- 每个 link 的槽位数量固定；未使用槽位通过 `active_mask` 关闭。记录包络范围、体积及离线生成时间证据；查询时间按用户确认的范围交给 V8 验收。
- 从完整 link 集合生成 self collision 候选。允许接触项须记录两个 link identity、原因、验证依据及适用的 `geometry_hash`；没有登记的 pair 保持候选状态。
- artifact 及其校验过程可重复。源 mesh、工具或生成规则变化使 `geometry_hash` 变化。

## 验收条件

- 对全部生产 collision mesh 生成 artifact；重新生成得到相同的几何数据、证明数据及 `geometry_hash`。
- 经过检查的 closed mesh 全部通过覆盖校验；非 closed、无效拓扑、缺少单位及任一四面体未覆盖的输入均被拒绝。
- 输出包含每个 link 的 mesh hash、固定槽位、`active_mask`、四面体覆盖证明、包络范围、运行时间证据及 self collision 候选。
- 几何或工具 identity 变化使 `geometry_hash` 变化。没有经过核验的允许接触证书时保留全部 self collision 候选，并拒绝非空允许接触登记。

## 当前输入情况

- 用户选择基于现有 STL 制作封闭网格。`models/ninezzhou/urdf/ninezzhou.urdf` 引用的十个 STL 均有未闭合边界；生成器保存原始文件身份，并生成包含这些源顶点的封闭凸包。
- `tool0` 是 `Link9` 局部 +X 方向 0.235 m 处的工作点。`Link9` 的 STL 延伸到该位置，生产清单把当前工具几何登记在 `Link9`，单位逐项声明为米。
- 生成资产使用从原 URDF 派生的独立 `ninezzhou_collision.urdf`；原仿真和 MoveIt 入口继续使用原 URDF。目标设备查询运行时间由执行地图中的 V8 验收票处理。
