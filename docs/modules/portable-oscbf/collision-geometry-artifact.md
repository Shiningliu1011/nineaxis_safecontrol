# Collision geometry artifact

`portable_oscbf/scripts/generate_collision_geometry.py` 从经过检查的 closed collision mesh 生成固定容量组合 ellipsoid 与体积覆盖证明。它是离线工具，不参与控制线程。安装离线依赖时使用 `portable_oscbf/requirements-geometry.txt`。

输入 JSON 清单包含 `schema_version`、`urdf`、`tool_identity`、`tool_link`、`slot_capacity`、`links` 与 `allowed_contacts`，生产清单还包含 `source_provenance`。`links` 每项包含 `link`、`mesh`、`unit`；`unit` 必须明确为 `m` 或 `mm`。路径相对于清单目录解析。清单须覆盖 URDF 中全部带 collision mesh 的 link，且 mesh 路径必须指向 URDF 为该 link 声明的文件。生成器将 URDF 中的 mesh `scale` 与 collision `origin` 应用到 link 坐标；`mm` 输入要求 URDF mesh `scale` 为 `0.001 0.001 0.001`。清单还须指定当前工具几何所属的 link；工具单独占用 link 时，也须提供其 closed mesh。

生产输入位于 `models/ninezzhou/meshes/`。这些 STL 有未闭合边界，因此使用 `generate_closed_collision_meshes.py` 为十个 link 生成封闭凸包和 `ninezzhou_collision.urdf`。生成时核对每个原始顶点均位于导出凸包内；每个原始三角面随之位于凸包内。`provenance.json` 保存原始 URDF、原始 STL、封闭网格及生成规则的 SHA-256。几何生成器重新计算凸包，并核对这些记录。凸包扩大了碰撞体积，现有仿真和 MoveIt 入口继续使用原 URDF；此资产供后续 `CollisionSafety` 接入使用。

`tool0` 是 `Link9` 局部 +X 方向 0.235 m 处的工作点，没有额外实体网格。生产清单使用 `Link9` 作为当前工具几何所属 link，`tool_identity` 绑定该工作点位置。十个原始 STL 与 URDF 的坐标按米记录，清单逐项声明 `m`。

`allowed_contacts` 当前只接受空列表，生成器保留全部 self collision pair。非空列表需要能够核对完整关节范围、pair 和当前 `geometry_hash` 的证明；缺少这类证明时生成器拒绝排除。

生成与检查命令，在仓库根目录执行：

```bash
python3 portable_oscbf/scripts/generate_closed_collision_meshes.py
python3 portable_oscbf/scripts/generate_closed_collision_meshes.py --check
python3 portable_oscbf/scripts/generate_collision_geometry.py --manifest portable_oscbf/config/collision_geometry_sources.json --output portable_oscbf/config/collision_geometry_artifact.json
python3 portable_oscbf/scripts/generate_collision_geometry.py --manifest portable_oscbf/config/collision_geometry_sources.json --output portable_oscbf/config/collision_geometry_artifact.json --verify
```

`--verify` 根据源 mesh 重新检查四面体边界、体积、每个四面体的 ellipsoid 覆盖、证明和几何身份；`--check` 还核对重新生成的数据与已有资产一致。生产清单预留每个 link 四个槽位，本次有效槽位各一个，未使用槽位的 `active_mask` 为 `false`。全部 45 个 link pair 均保留为 self collision 候选。生成时间放在 `runtime_evidence`，并标记为未取得查询 deadline 准入资格；时间数值不会参与 `geometry_hash`。

本次资产仅证明生成的封闭凸包体积受到 ellipsoid 覆盖；它没有真实设备命令权限。生成时间只说明本地离线处理耗时。目标设备上的 JAX/DCOL 查询 deadline 由 [V8 目标计算设备验收](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/119) 处理。
