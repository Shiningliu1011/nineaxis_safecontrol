# main 集成验收

独立 merger 从 `.scratch/worktrees/map147-mesh-self-filter` 集成 #105 到 main，工作目录为 `/home/lsn/robot/robot_safecontrol`。源与 main HEAD 均为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`。冻结清单为 `.scratch/map147-105/frozen-source-manifest-review-fix.json`；19 份来源文件、四份 #108 依赖与 main 原状态在 `integration/before.json` 核验并记录。

源码、测试、解析 JSON、模块文档和 Trellis 逐项采用文件补丁移植。`lidar_occupancy.py` 只增加可见/未知区间、输入失效和身份状态；OccupancySnapshot、policy 和动态 support 保持原定义。`setup.py` 组合保留 #152 连续包安装及 #105 依赖、JSON 安装。四份 #108 依赖保留 main 内容，#101 尚未集成。

## 直接验收

环境为 `.scratch/map147-105/integration/env.sh`，使用 main 源码和现有真实依赖。Python 3.10.12、NumPy 1.26.4、JAX 0.6.2、trimesh 5.1.0、rtree 1.4.1、urdf-parser-py 0.0.4；临时文件、缓存、ROS 与测试产物均在被忽略 `.scratch`。

| 检查 | 实际结果 | 证据，以 `.scratch/map147-105` 为根 |
|---|---|---|
| 新 self-filter、adapter、prepare_scene、occupancy、collision 参数/查询及固定依赖 | 311 passed，60.60 s | `integration/consumers.log`、`consumers.xml` |
| Ruff 六份源码/测试 | 通过 | `integration/ruff.log` |
| mypy 三份实现 | 通过 | `integration/mypy.log` |
| diff 检查 | 通过 | `integration/diff-check.log` |
| Trellis context | 4 条 implement、3 条 check 通过 | `integration/context.log` |
| 组合安装文件字节 | #105 的 7 项与 main 相同，其他模块同时核验 | `integration/installed-files.json` |
| Python `-I` 安装副本 | 原始几何、标定状态、真实 adapter 与解析射线通过 | `integration/installed-imports.json` |

上述 311 项覆盖真实第三方错误 count=0、datatype=99 和字段重名，公开处理边界撤销旧 free 并保留 occupied；原始异常参数及 traceback 经实际 sensor_msgs_py 核验。没有增加 mock 或真实设备动作。

组合 colcon 从 main 构建至 `.scratch/map147-152/integration/install`，包构建 9.25 s、总 9.89 s。完整实际命令和源码安装映射保存于该目录，#105 证据保留同一构建的副本记录。安装探针加载 10 个原始 STL，共 696330 个三角面，URDF SHA256 为 `766ea5870c41f110d7d57b46e6b31153335f45358faf15e641d465b3b31587a3`；几何与标定均 `admission_ready=false`。回放 JSON 是 `analytic_interface_replay`，独立解析盒体首交点为 2.5 m。

## 完整项目检查

当前 main 组合包括 #108、#153、#152 和 #105。已读取完整项目入口及相关副作用，按 `run_all_tests.sh` 相同三层测试入口执行；主包与控制内核逐文件保留完整 nodeid、结果、跳过原因和退出码。主包 854 项中，847 项功能通过、6 项按原条件跳过；内核 741 项中，706 项功能通过、34 项按原条件跳过。另两项计时分别在已记录 CPU 条件下通过原断言；AEB 4 passed，真实 final launch 1 passed、transition runtime 14 passed，后两组已包括在主包统计内。

共享原始证据及逐项覆盖核验为 `.scratch/map147-152/integration/full/coverage.json`，无缺项；环境、全部可复现命令、五种计时条件和原始失败见 [共同完整验收](../../10-27-map147-butterfly-reference/research/integration.md)。controller 在四个物理 P-core 下为 303 步、p95 14.343 ms、1 步超期（0.33%），原 20 ms/1%/300 步门槛通过；modular 在八个 P-core 下单步 6.379 ms，相对原基准 6.452 ms 的原门槛通过。全部 CPU 及 controller 八核心结果仍失败，原基准文件字节保持不变。

首次绘图环境启动失败、既有研究目录链接导致的评价失败以及内核相对计时失败全部保留；相应 nodeid 的实际复验和条件进入覆盖清单。40 项原始跳过的逐项原因同样保留，不计为通过。最终来源/main、四份 #108 依赖及 #153 保留文件 SHA256 为本任务 `integration/final-files.json`。

## 能力范围

原 mesh 拓扑和标定缺少物理准入证据，继续保持不可准入；解析回放与数学测试不构成现场 LiDAR 证明。真实回放、独立 server、设备误差和生产切换分别由 #122、#104、#117、#123 验收。规范、CONTEXT 和 ADR 保持原内容，全部工程修改未提交，未关闭 issue。
