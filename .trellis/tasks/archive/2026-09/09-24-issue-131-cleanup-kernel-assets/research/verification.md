# I16 验收记录

日期：2026-09-24。

- `bash build_aeb_moveit.sh`：两个包构建成功。
- 加载 `/opt/ros/humble/setup.bash` 与 `install/setup.bash` 后运行 `python3 -m pytest portable_oscbf/tests -q`：178 passed，34 skipped。
- `bash run_all_tests.sh`：主包 544 passed、1 skipped；内核 178 passed、34 skipped；AEB 插件 4 项通过；退出码 0。
- `bash scripts/agent_check.sh`：全部检查通过，其中纯逻辑测试 69 passed。
- `git diff --check`：通过。
- 源码内核目录及安装后的内核目录均仅保留指定的配置与 `data/ik_input.mat`；安装目录没有悬空符号链接。生产轨迹 `data/nurbs/ik_input.mat` 在源码和安装目录中均存在，相关轨迹测试通过。

本票仅涉及已有文件的清理和安装清单，未产生需要写入项目规范的新规则。
