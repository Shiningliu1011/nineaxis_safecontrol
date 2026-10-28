# 实施与检查

独立实现工作区为 `.scratch/worktrees/map147-mesh-self-filter`，共同基点为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`。本目录记录需求、设计、真实行为测试与 main 集成，主任务为 `.trellis/tasks/10-27-map147-implementation`。

- [x] 原始几何、逐点时间、反馈历史、误差分类和身份失效行为完成 red/green。
- [x] 真实 PointCloud2、JointState、占据 owner 与 CollisionSafety 接缝验证通过。
- [x] 独立 Spec、Standards 及三项第三方解码错误修订复核通过。
- [x] main 按实际文件差异集成，保留四份 #108 依赖。
- [x] main 311 项直接消费者、Ruff、mypy、组合安装与安装副本探针通过。
- [x] main 完整项目测试与最终文件清单。

## 来源工作区的行为证据

全部 `logs/` 路径以主仓库 `.scratch/map147-105` 为根。完整命令与原工作区记录保存在该来源工作区同名 `implement.md`，冻结清单为 `frozen-source-manifest-review-fix.json`。

| 行为 | red 证据 | green 证据 |
|---|---|---|
| 原始三角面首交点 | `logs/red-geometry.log`：公开模块不存在 | `logs/green-geometry2.log`：3 项通过 |
| 逐点时间、历史与误差分类 | `logs/red-timing.log`：公开时间接口不存在 | `logs/green-timing3.log`：13 项通过 |
| ROS 格式适配 | `logs/red-adapter.log`：公开 adapter 不存在 | `logs/green-adapter.log`：4 项通过 |
| 来源身份变化保持失效 | `logs/red-identity.log`：覆盖状态断言失败 | `logs/green-identity.log`：27 项通过 |
| 必需依赖声明与版本 | `logs/red-manifest.log`：依赖清单断言失败 | `logs/green-manifest.log`：2 项通过 |
| 只读几何与完整输入身份 | `logs/red-content.log`：身份断言失败 | `logs/final-frozen.log`：33 项通过 |
| 历史对象缺少时撤销准入 | `logs/red-empty-history.log`：错误类型断言失败 | `logs/final-frozen.log`：33 项通过 |
| 表面前环境返回与后方 unknown | `logs/red-environment-occlusion.log`：覆盖断言失败 | `logs/final-frozen.log`：33 项通过 |
| 多个 collision 的 part 身份 | `logs/red-part-identity.log`：字段不存在 | `logs/final-frozen.log`：33 项通过 |
| 第三方解码错误撤销旧 free | `logs/review-red.log`：3 项覆盖断言失败 | `logs/review-green.log`：138 项通过 |

最后一组使用真实消息额外字段 count=0、datatype=99 和重名，经过 sensor_msgs_py 解码、实际占据与 `prepare_scene`，检查原始异常参数、第三方 traceback、UNKNOWN、occupied 保留和全部无效 mask。新行为共 36 项。

## main 验收

环境为 `.scratch/map147-105/integration/env.sh`：Python 3.10.12、NumPy 1.26.4、JAX 0.6.2、trimesh 5.1.0、rtree 1.4.1、urdf-parser-py 0.0.4。直接测试使用真实 trimesh、URDF、ROS 消息、占据对象和 CollisionSafety，没有创建硬件控制 I/O。

`integration/run_consumers.sh` 运行新模块、adapter、prepare_scene、occupancy、collision 参数/查询和固定依赖，取得 311 passed；日志为 `integration/consumers.log`，JUnit 为 `integration/consumers.xml`。Ruff 六份文件及 mypy 三份实现通过，日志为 `integration/ruff.log` 和 `integration/mypy.log`。

组合安装由 `.scratch/map147-152/integration/build_install.sh` 从 main 构建，包含 #153、#152 和 #105。真实安装副本的路径、逐文件 SHA256 与探针分别保存在本任务 `integration/installed-files.json`、`installed-imports.json`。原 URDF 共 696330 个三角面，几何与标定均保持 `admission_ready=false`；JSON 数据属于 `analytic_interface_replay`，解析盒体的首交点为 2.5 m。

完整项目验收、跳过原因及最终文件身份记录在 [main 集成验收](research/integration.md)。真实 LiDAR 回放、独立 server、设备误差和生产切换由 #122、#104、#117、#123 分别验收。规范、CONTEXT 与 ADR 保持原内容，全部修改未提交。
