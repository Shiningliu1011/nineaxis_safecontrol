# 清理旧目录并纳入 AEB 插件测试

## 来源与目标

来源：[[I15] 清理不受 CollisionSafety 影响的旧目录与文档](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/130)，属于[速度级 OSCBF 与 CollisionSafety 统一执行地图（仅激光雷达感知）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133)。

清理已经有维护来源的重复文件，保存仍需查阅的历史说明，并让当前 AEB-RRT* MoveIt 插件测试进入仓库全量测试入口。

## 需求

- 移除仓库根的 `xy-mid-360-s/` 副本。将其原 README 中仍有效的设备接口、来源、验证范围与使用约束写入保留的 `livox_mid360` 模块说明，项目文档和命令使用现有 ROS 包入口。
- 保持 `models/ninezzhou/urdf/ninezzhou.urdf` 作为运动学与关节位置限幅的手工来源；清理 `portable_oscbf/urdf/` 重复模型及其引用。
- 将旧 OSCBF 移植指南和执行计划归入 `docs/archive/`，保留章节链接可用；当前文档入口准确标明其历史用途。
- 更新 `ninezzhou_collision.h` 的用途说明、项目入门文档、仓库根 README、目录说明与安装清单相关引用。
- 将 AEB-RRT* MoveIt 插件的四个现有测试可执行文件登记进 ctest，并由 `run_all_tests.sh` 运行。当前生产规划路径在 CollisionSafety 切换前持续接受这些检查。
- 保留现有仿真、shadow 与 live 硬件安全边界；本票不修改碰撞授权或真实硬件行为。

## 验收条件

- [x] `xy-mid-360-s/` 与 `portable_oscbf/urdf/` 不存在；保留的驱动说明覆盖原 README 的有效内容，运动学来源与位置限幅生成入口仍指向 `models/ninezzhou/urdf/ninezzhou.urdf`。
- [x] 两份旧指南及其章节位于 `docs/archive/`；仓库根 README、`docs/ONBOARDING.md`、文档索引、目录说明、相关运行说明与安装清单引用一致，文档内部链接有效。
- [x] `ninezzhou_collision.h` 的说明准确表明其测试用途与当前生产碰撞路径。
- [x] 四个 AEB 插件测试均出现在 ctest 清单，`bash build_aeb_moveit.sh` 与包含插件 ctest 的 `bash run_all_tests.sh` 返回成功。
- [x] `install/` 中不存在指向已删除文件的符号链接；相关变更通过静态检查，用户已有修改保持原样。
