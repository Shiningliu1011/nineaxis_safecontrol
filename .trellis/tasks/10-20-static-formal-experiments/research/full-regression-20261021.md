# 当前源码完整回归

在本机项目目录 `/home/lsn/robot/robot_safecontrol` 执行 `bash run_all_tests.sh`，
环境入口为 `.scratch/host-recovery/env.sh`，CPU 0–15，ROS domain 230。
systemd user service 为 `static-final-regression-03.service`，最终退出码为0。
运行源码身份基于 `40d43bb3712e3f99c11a59750a59d1ee65ecb525`；
本轮集成的研究脚本、工单验收及文档均未改变控制源码。

| 验证部分 | 实际结果 | 退出码 |
| --- | --- | ---: |
| 主包 `tests/` | 732 passed、1 skipped，1003.82秒 | 0 |
| `portable_oscbf/tests/` 的52个独立文件 | 486 passed、34 skipped | 0 |
| `build/aeb_rrtstar_ompl` CTest | 4/4 passed | 0 |

日志保存为 `output/static-capacity-20261020/final-regression-03.log`，
SHA-256 为 `4dbcf8270db190f7848e5ef1b5b5c10f308a056ee4101b31c25cda4f0513522e`。
测试产物完整保留在 `.scratch/regression-FxK2lbwI/`。
启动入口为 `.scratch/static-implementation/run_final_regression.sh`。

日志包括实际 ROS/MoveIt2 规划、运动重放、完整路径跟随及报告保存检查，
控制内核测试按文件独立运行以释放各文件的 JAX 编译缓存。
原始实验记录与历史基准保留，本轮回归产物使用独立目录。
正式共同周期、容量准入及1080项实验仍分别按对应证据验收。
