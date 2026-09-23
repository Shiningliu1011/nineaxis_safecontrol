# 技术设计

## 文件职责

- `src/robot_safecontrol_moveit/livox_mid360/` 保留驱动代码；`tests/test_livox_mid360_*.py` 保留其现有测试。将已记录的驱动说明改为当前 ROS 包的命令和导入路径，避免继续指向独立副本。
- `models/ninezzhou/urdf/ninezzhou.urdf` 保留唯一模型手工来源；`portable_oscbf/scripts/generate_kinematics_data.py` 保留生成入口。当前工作区已经没有 `portable_oscbf/urdf/`，验收仍检查其缺失状态和安装产物。
- 将两份指南及其章节目录一起迁至 `docs/archive/`，保持指南与章节之间的相对路径关系。当前文档索引指向归档入口，运行说明指向当前代码和测试。
- `ninezzhou_collision.h` 继续供 `test_aeb_full` 的简化碰撞测试使用。注释表明 MoveIt/FCL 是当前生产插件路径，避免把测试模型当作生产碰撞判定。

## 测试入口

- 在插件的 `CMakeLists.txt` 为四个现有可执行文件登记 ctest，使用 CMake 目标路径调用已构建文件。
- `run_all_tests.sh` 在加载已构建环境后运行插件 ctest，并与主包、内核 pytest 分别记录退出码；任何一项失败均使全量入口失败。
- `test_plugin_init` 从已安装的 `robot_safecontrol_moveit` 包资源目录取得 URDF 和 SRDF，检查资源文件后初始化真实插件。该包由 `build_aeb_moveit.sh` 同时构建。

## 验证边界

构建脚本只编译本仓库两个软件包；测试可执行文件使用 ROS 包资源，不发送真实硬件命令。检查安装目录的失效符号链接，并核对目录、文档与安装清单的引用。
