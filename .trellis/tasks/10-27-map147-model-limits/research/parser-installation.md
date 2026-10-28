# 模型格式解析依赖

`urdf_parser_py` 和 `srdfdom` 直接解析现有 URDF/SRDF。`package.xml` 声明 `urdfdom_py`、`srdfdom`。官方 [srdfdom 2.0.7 package.xml](https://github.com/moveit/srdfdom/blob/2.0.7/package.xml) 声明 `urdfdom_py` 运行依赖；同版本 [CMakeLists.txt](https://github.com/moveit/srdfdom/blob/2.0.7/CMakeLists.txt) 使用 `ament_python_install_package` 安装 Python 部分。本轮已完整读取两个官方源文件。

Ubuntu 22.04 / Python 3.10 的系统安装入口为 `apt-get install ros-humble-urdfdom-py ros-humble-srdfdom`。离线安装预先保存发行包及系统依赖，使用 `apt-get install ./包文件.deb`。Python 搜索路径包含这些包的 `lib/python3.10/site-packages` 与 `local/lib/python3.10/dist-packages`。

本轮使用 `apt-get download` 取得正式发行包，并以 `dpkg-deb -x` 安装到独立目录 `.scratch/map147-153/parser-install`，不修改系统安装。版本为 srdfdom `2.0.7-1jammy.20260908.002942` 和 urdfdom-py `1.2.1-1jammy.20260907.220847`。包 hash 与模块文档一致。

`python -I .scratch/map147-153/parser_isolation.py` 在独立进程中加载这两个副本及当前 worktree 代码，真实读取全部模型、SRDF 排除项并计算 FK。完整 `sys.path`、模块路径、模型/配置/绑定 hash、FK 和已加载模块写入 `parser-isolation.json`。初始搜索路径不包含 `/opt/ros`，最终 `rclpy/rospy/ament` 均未加载。
