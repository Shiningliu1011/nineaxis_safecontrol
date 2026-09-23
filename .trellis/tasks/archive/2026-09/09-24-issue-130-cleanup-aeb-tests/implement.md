# 执行与验证

## 修改

- [x] 保存独立驱动 README 的有效内容，更新保留的模块说明与项目导航，移除重复驱动目录。
- [x] 归档两份旧指南及章节，更新相对链接、历史用途和目录引用；核对模型唯一来源与安装清单。
- [x] 修订测试碰撞头文件用途说明；登记四个插件测试并修正其资源路径。
- [x] 将插件 ctest 纳入全量测试脚本与运行文档。

## 验证

- [x] 运行 `bash build_aeb_moveit.sh`，记录退出码。
- [x] 运行 `ctest --test-dir build/aeb_rrtstar_ompl -N`，确认四个测试均已登记。
- [x] 运行 `bash run_all_tests.sh`，记录三套测试的结果。
- [x] 检查 `install/` 中失效符号链接与仓库中的旧路径引用，核对文档链接和 `git diff --check`。

## 失败处理

构建或测试失败时，在本票范围内修复原因并重跑受影响的命令；记录无法在当前环境验证的条件。

## 验收记录

日期：2026-09-24。基础提交：`218f43983db9b145fb3789e3ec905b8682fc27a7`。工作分支：`codex/issue-130-cleanup-aeb-tests`。

- `bash build_aeb_moveit.sh`：2 个包构建成功，退出码 0。
- `ctest --test-dir build/aeb_rrtstar_ompl -N`：登记 4 个测试。
- `bash run_all_tests.sh`：主包 544 通过、1 跳过；控制内核 178 通过、34 跳过；插件 ctest 4 通过；退出码 0。
- `bash scripts/agent_check.sh`：全部检查通过，纯逻辑测试 69 通过；退出码 0。
- `python3 portable_oscbf/scripts/generate_kinematics_data.py --check`、`git diff --check`、`bash -n run_all_tests.sh`：通过。
- `find install -xtype l -print`：没有输出。旧目录缺失状态、归档文件和现行文档引用已经核对。
- `ros2 run robot_safecontrol_moveit livox_mid360_tool --help`：退出码 0，已核对文档所列命令入口。

## 交接

本次经验已经写入对应模块和测试说明，没有新增需要修改项目规范的稳定规则。`docs/agents/issue-tracker.md` 的已有未提交修改由用户保留，本任务未修改该文件。验收结果已写入[议题评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/130#issuecomment-5792424180)。用户已授权仅提交并推送本票改动，随后关闭议题、更新执行地图并归档本地任务。
