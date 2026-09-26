# 执行与验证

- [x] 确定全部生产 collision mesh 的来源、单位、link 与工具 identity。
- [x] 固定离线输入清单格式和 TetGen 依赖，建立输入检查与四面体边界检查。
- [x] 生成固定容量组合 ellipsoid、独立覆盖证明与完整 self collision 候选。
- [x] 保存可重复的 artifact、`geometry_hash`、包络范围和离线生成运行时间证据。
- [x] 使用解析 closed mesh 运行生成与重复性测试；对非 closed、无效拓扑、单位缺失、未覆盖四面体、URDF 几何不匹配及身份变化运行拒绝测试。
- [x] 对全部生产 mesh 运行生成器和覆盖检查，记录生成命令、输出 hash 与运行时间。
- [x] 更新 `docs/modules/portable-oscbf/` 中的生成说明和依赖说明，运行受影响测试。
- [x] 完成 Trellis 检查与代码审查。

## 当前检查

- `trimesh.load_mesh(...).is_watertight`：十个原始 STL 均返回 `False`；`is_volume` 同样均为 `False`。
- `generate_closed_collision_meshes.py` 为十个 link 生成封闭凸包，逐个核对源顶点包含条件。`tool0` 是 `Link9` 的固定工作点坐标。
- `PYTHONPATH=.scratch/i20-tetgen:src TMPDIR=.scratch/i20-temp OPENBLAS_NUM_THREADS=1 python3 -m pytest -q portable_oscbf/tests/test_collision_geometry_artifact.py tests/test_model_installation.py --basetemp .scratch/i20-pytest`：28 passed。
- 最终 XML 格式及资产身份复核：同一环境运行 `portable_oscbf/tests/test_collision_geometry_artifact.py --basetemp .scratch/i20-pytest-final`，10 passed。
- `OPENBLAS_NUM_THREADS=1 python3 portable_oscbf/scripts/generate_closed_collision_meshes.py --check`：通过。
- `generate_collision_geometry.py --manifest portable_oscbf/config/collision_geometry_sources.json --output portable_oscbf/config/collision_geometry_artifact.json --verify`：由生产资产测试执行，重新核对生成数据；使用 `.scratch/i20-tetgen` 提供 TetGen 0.8.4。
- `PYTHONPATH=.scratch/i20-tetgen:src TMPDIR=.scratch/i20-temp OPENBLAS_NUM_THREADS=1 bash run_all_tests.sh`：主包 547 passed、1 skipped；portable_oscbf 210 passed、34 skipped；AEB 插件 4/4 passed，退出码均为 0。
- 加载 ROS 2 Humble 后执行 `colcon build --packages-select ninezzhou --base-paths models/ninezzhou --event-handlers console_cohesion+`：通过。重新加载 `install/setup.bash`，通过 `ament_index_python` 定位已安装的 `ninezzhou`，解析派生 URDF 并确认十个 collision mesh URI 全部存在。
- `python3 -m compileall -q portable_oscbf/scripts/generate_closed_collision_meshes.py portable_oscbf/scripts/generate_collision_geometry.py portable_oscbf/tests/test_collision_geometry_artifact.py setup.py`：通过。
- `git diff --check`：通过。
- 生产清单预留每个 link 四个槽位，本次各启用一个，全部 45 个 self collision link pair 保留。生成命令输出 `geometry_hash=add9743e201beff9ed1154af1ee2c7283d6f34533fde1b7b5bd169c30b7c5fc7`，离线生成时间约 9.3 秒。
- 目标设备 JAX/DCOL deadline 由 V8 验收票处理，本资产标记为未取得查询准入资格。
- 允许接触排除须经过完整关节范围证书核验；当前生成器拒绝非空登记。

## 完成交接

- Phase 3.3：本票的模型来源、生成流程和验收边界已记录在模块文档；没有需要写入 `.trellis/spec/`、`CONTEXT.md` 或 ADR 的新增通用规则。
- Phase 3.4：用户明确授权仅提交并推送本票改动，随后关闭议题、更新地图、归档任务、合并分支并只保留 `main`。
- `docs/agents/issue-tracker.md` 是用户已有改动，不属于本票提交范围。
- 用户确认 I20 按离线几何证据关闭，JAX/DCOL 查询期限由 V8 验收；资产保持 `qualified_for_query_deadline=false`。
- 实现提交：`7ec63e435236154736bb9193fae60704a522e8d4`。议题已关闭，执行地图已更新；[票尾验收证据](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/106#issuecomment-5842575054)。
