# 几何资产生成设计

## 边界

当前生产碰撞代码使用 URDF STL 和旧 OBB 数据，没有可复核的四面体体积覆盖证明。本任务新增离线生成与检查入口、封闭凸包、派生 URDF、输入清单、固定槽位资产和对应测试；`CollisionSafety` 的计算接入由后续执行票负责。

代码位置：`portable_oscbf/scripts/` 保存离线生成入口，`models/ninezzhou/collision_meshes/` 保存封闭网格与来源记录，`portable_oscbf/config/` 保存输入清单和通过检查的资产，`portable_oscbf/tests/` 保存真实 closed mesh 与拒绝输入的验证。运行中的控制线程不导入网格处理依赖。

## 输入与身份

输入清单保存 URDF 路径、link、mesh 路径、明确的 `unit`、工具 identity、槽位预算及允许接触登记。原始 STL 经过凸包生成封闭网格；独立 URDF 仅替换 collision mesh 路径。来源记录绑定原始 URDF、原始 STL、封闭网格与生成规则的 SHA-256。几何生成器复算凸包及来源记录，解析派生 URDF 后核对 collision link 集合、实际 mesh 文件、mesh scale 和 collision origin，拒绝重复、遗漏和未登记的工具。`geometry_hash` 对来源记录、经过规范化的几何、slot、mask、工具 identity 和生成器源码计算 SHA-256。允许接触登记参与后续 `collision_policy_hash`。离线生成时间单独保存，并明确不构成查询 deadline 证据。

## 几何证明

对每个原始 mesh 检查有限坐标、非退化面、每条边的拓扑、闭合性、面方向和正体积。使用 TetGen 的 PLC 模式对检查通过的表面生成四面体。校验输出四面体的正体积、边界与输入表面一致及总体积一致。每个四面体分配给一个 ellipsoid；采用四面体集合的空间分组生成候选 ellipsoid，再扩大半轴直至该组每个四面体的全部顶点都满足包含条件。对最终槽位独立重算全部四面体的覆盖关系，未覆盖立即失败。证明记录四面体数量、每个槽位覆盖数量、最小包含余量及 tetra 输出 hash。

## 槽位与性能

槽位数量是输入清单中的固定预算。生成器在预算内选择使包络体积下降的分组，生成未使用槽位的 `active_mask=False`。本票保存 CPU 离线生成时间与固定槽位候选。用户确认 I20 以离线几何证据验收；JAX/DCOL 查询时间及目标设备预算由 V8 验收。没有通过目标设备 deadline 的证据时，资产不获得生产准入。

## self collision 候选

对所有不同 link 生成无序 pair。当前没有可核对完整关节范围不可接触证明的证书入口，因此拒绝非空允许接触登记，全部 pair 留在候选集合。以后具备证书验证器时，每项排除必须绑定两个 link、原因、完整关节范围证明和适用的 `geometry_hash`。

## 输入资料门

原始十个 STL 均未闭合。封闭凸包包含每个原始顶点及其三角面，并接受源文件 hash、导出网格体积和四面体覆盖检查。`tool0` 是 `Link9` 的工作点坐标，当前工具几何归属为 `Link9`。凸包资产在后续模块完成目标计算设备验收以前没有命令权限。
