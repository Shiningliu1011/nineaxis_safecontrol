# I20 代码审查

## 项目规范

- 离线生成器位于 `portable_oscbf/scripts/`，网格与四面体依赖不进入控制线程。
- 封闭网格原始数据在 `trimesh` 处理前接受有限值和面有效性检查；同一份文件字节用于 hash 与加载。
- 清单 mesh 与 URDF collision mesh 路径一致，URDF scale、origin 参与几何计算，生成器源码及依赖版本参与 `geometry_hash`。
- 十个原始 STL 派生封闭凸包，来源记录保存原始与生成文件 hash；几何生成器重新计算凸包并比较派生 URDF 结构。`models/ninezzhou/CMakeLists.txt` 安装新增网格目录。
- `ninezzhou` 构建通过，安装后的派生 URDF 引用的十个封闭网格 URI 全部存在。
- 解析 closed mesh 与生产资产共十项测试通过，覆盖不闭合、无效拓扑、缺少单位、未覆盖四面体、URDF 几何不匹配、非有限原始坐标、身份变化和小体积输入。

## 规格与验收

- 四面体边界、总体积与源 mesh 比较，每个四面体的四个顶点由同一个启用 ellipsoid 覆盖；`--verify` 根据源数据复核。
- 完整 self collision link pair 集合保留。非空允许接触登记在完整关节范围证书验证入口具备以前被拒绝。
- 生产清单使用十个封闭凸包，`Link9` 包含 `tool0` 的 0.235 m 工作点；单位声明为米。资产包含十个 link 的四面体覆盖证明、45 个 self collision 候选和本地生成时间。
- 用户确认 I20 验收范围为离线几何与生成时间证据。目标设备 JAX/DCOL 查询期限由 V8 验收；资产的 `qualified_for_query_deadline` 保持 `false`。
- 规范与需求两项独立审查均已完成；当前验收范围内没有待处理发现。完整测试与安装检查结果见 `implement.md`。
