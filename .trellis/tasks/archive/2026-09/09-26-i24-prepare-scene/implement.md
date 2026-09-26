# 执行与验证

- [x] 领取票据，核对前置票与规格，建立规划文件。
- [x] 实现场景元数据、checksum、固定容量 tracking 和准入。
- [x] 更新解析输入并增加公开行为测试。
- [x] 运行场景、参数、DCOL 和 point-scale 首轮回归。
- [x] 完成文档与 trellis-check；按共享内核范围进行规范、需求审查。
- [x] 记录实际命令与结果，发布票据证据并更新地图。

验证中间文件放在已忽略的 .scratch/i24；pytest 使用该目录作为临时目录，不能读取或写入 /tmp。测试使用真实 CollisionSafety 与解析输入，检查范围包括完整 portable_oscbf 测试。生产 ROS 与真实设备不启动。

用户已授权仅提交并推送本票改动、更新议题与执行地图并归档本地任务；本地和远程仅保留 main。规范 promotion 仅记录候选，不直接修改规范。

## 实际验证

- 接口与参数回归：122 passed，25.03 秒；命令为 `TMPDIR="$PWD/.scratch/i24/runtime" python3 -m pytest portable_oscbf/tests/test_collision_safety.py portable_oscbf/tests/test_collision_parameters.py -q --basetemp="$PWD/.scratch/i24/tests/initial"`，日志 `.scratch/i24/initial-tests.log`。
- 场景、DCOL 与 point-scale 首轮回归：158 passed，182.61 秒；日志 `.scratch/i24/behavior-tests.log`。命令按同一 TMPDIR 设置运行 `python3 -m pytest portable_oscbf/tests/test_prepare_scene.py portable_oscbf/tests/test_ellipsoid_dcol.py portable_oscbf/tests/test_ellipsoid_point.py -q --basetemp="$PWD/.scratch/i24/tests/behavior"`。
- `python3 -m compileall -q` 对新场景模块、公开模块和相关测试成功；`git diff --check` 成功。
- 完整内核命令：`source install/setup.bash && TMPDIR="$PWD/.scratch/i24/runtime" python3 -m pytest portable_oscbf/tests -q --basetemp="$PWD/.scratch/i24/tests/kernel"`。实际输出为 10 failed、460 passed、34 skipped，705.38 秒；10 项失败全部由系统解释器缺少 tetgen 引起，对应几何验收已在下述隔离环境全部通过。日志 `.scratch/i24/kernel-tests.log`。
- 时间边界与公开操作回归：124 passed，16.46 秒；日志 `.scratch/i24/time-boundaries.log`。
- 最终碰撞与几何回归：296 passed，254.61 秒，无 pytest warning。命令：`source install/setup.bash && TMPDIR="$PWD/.scratch/i24/runtime" .scratch/i24/geometry-env/bin/python -m pytest portable_oscbf/tests/test_prepare_scene.py portable_oscbf/tests/test_collision_safety.py portable_oscbf/tests/test_collision_parameters.py portable_oscbf/tests/test_ellipsoid_dcol.py portable_oscbf/tests/test_ellipsoid_point.py portable_oscbf/tests/test_collision_geometry_artifact.py -q --basetemp="$PWD/.scratch/i24/tests/final-reviewed"`。日志 `.scratch/i24/final-reviewed-tests.log`。
- 隔离环境通过 `python3 -m venv --without-pip --system-site-packages .scratch/i24/geometry-env` 创建；通过主解释器的 `pip --python .scratch/i24/geometry-env/bin/python install -r portable_oscbf/requirements-geometry.txt` 安装几何依赖，TMPDIR 指向 `.scratch/i24/runtime`。
- 已核对源码全部 CollisionScene/CollisionIdentities 构造调用点，以及 setup.py 的 work/*.py 安装规则。新增文档链接目标存在。
- 最终代码摘要：`.scratch/i24/code-sha256.txt`，测试后逐项核验一致。规范与需求的两个独立只读审查均已通过，记录见 `research/review.md`。
- 数据来源摘要：`.scratch/i24/input-sha256.txt`，覆盖解析参数、场景输入和 geometry artifact；运行环境为 Python 3.10.12、JAX 0.6.2、NumPy 1.26.4、Tetgen 0.8.4，JAX device 为 CPU。

## 规范检查

本次接口信息保存在模块文档；没有经用户确认的规范 promotion 内容。生产参数与设备性能需后续测量，本机解析测试不提供生产命令准入。

## 交付

[实现 prepare_scene 与 PreparedScene 准入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/100#issuecomment-5844061262) 已发布验收评论并关闭；[速度级 OSCBF 与 CollisionSafety 统一执行地图（仅激光雷达感知）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133) 保存本票的证据入口。更新前重新读取地图正文。

实现提交 `ba330c8e6c9cac8008304436f562c1fb34cc913a` 已推送至 origin/main，仅包含本票的 11 个代码、测试和文档文件。提交前检查文件范围、diff 格式及已验证代码的 SHA-256；当前代码与 296 项回归通过时的摘要一致。任务记录单独归档并提交。本地和远程都只有 main。工作区原有的 I18 与 T1 任务目录及 T1 规划文档保持原状。
