# 场景服务实现验证记录

验证日期：2026-10-28。工作区：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server`。基点：`8d5e938d4fe0185e07f17bc2359881157967baac`。分支：`codex/map147-scene-server`。本记录对应冻结的 22 份未提交文件；独立复核与 main 集成状态由主会话管理。

main 集成、独立复核和主分支验收已完成，见[最终验收](main-validation.md)。原实现工作区已清理，以下绝对路径中的原工作区 `.scratch/` 对应当前 `/home/lsn/robot/robot_safecontrol/.scratch/map147-104/worker-evidence/`；全部文件字节与 SHA256 已按保存路径重新核验。当前完整复现使用主分支验收入口。

## 环境与复现

Python 为 `/home/lsn/robot/robot_safecontrol/.scratch/host-recovery/venv/bin/python`，版本 3.10.12，架构 x86_64。实际依赖为 JAX 0.6.2、NumPy 1.26.4、trimesh 5.1.0、msgpack 1.1.2，ROS 2 Humble。CollisionSafety 使用 x64 CPU。全部 shell 命令使用 `/bin/bash`、`login=false`，并指定该工作区为 `workdir`。

环境脚本：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/scene-validation-env.sh`。该文件先加载已检查的 host-recovery 环境，再将 PYTHONPATH 的前两项设为本工作区 `src` 和 `portable_oscbf`，追加真实依赖目录。TMPDIR、ROS 日志、Matplotlib、Python bytecode 与其他缓存均指向本工作区 `.scratch/`。

## 实际验证

```bash
source .scratch/scene-validation-env.sh
python -m pytest \
  portable_oscbf/tests/test_shared_collision_scene.py \
  portable_oscbf/tests/test_prepare_scene.py \
  portable_oscbf/tests/test_dynamic_support.py \
  portable_oscbf/tests/test_lidar_occupancy.py \
  portable_oscbf/tests/test_lidar_self_filter.py \
  tests/test_collision_scene_server.py tests/test_timed_lidar_transport.py \
  tests/test_lidar_self_filter_adapter.py tests/test_perception_bridge_points.py \
  tests/test_perception_bridge_startup.py tests/test_perception_bridge_demo.py \
  --basetemp="$PWD/.scratch/pytest/review-final-complete" \
  --junitxml="$PWD/.scratch/scene-final-junit.xml" -q
```

结果为 `322 passed, 1 warning in 76.44s`，退出码 0。唯一 warning 来自现有 Matplotlib Axes3D 导入环境，测试未使用视觉能力。共享、来源、真实 ROS 与逐点传输边界的单独集合为 `31 passed, 1 warning in 11.35s`。

| 制品 | 绝对路径 |
|---|---|
| JUnit | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/scene-final-junit.xml` |
| pytest 输出 | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/scene-final-pytest.log` |
| ROS 输入、manifest 与进程日志 | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/pytest/review-final-complete/test_real_bridge_and_server_pr0/` |
| 环境脚本 | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/scene-validation-env.sh` |
| 源码 SHA-256 清单 | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/scene-source.sha256` |
| 机器交接 JSON | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/scene-handoff.json` |
| 边界集合 JUnit | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/review-boundaries.xml` |
| 边界集合输出 | `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/review-boundaries.log` |

## 隔离安装

```bash
source .scratch/scene-validation-env.sh
python setup.py egg_info --egg-base "$PWD/.scratch/package-metadata" \
  build --build-base .scratch/package-build \
  install --prefix "$PWD/.scratch/package-install" \
  --single-version-externally-managed \
  --record "$PWD/.scratch/package-install-record.txt"
python .scratch/installed_scene_probe.py
```

安装退出码 0。实际安装入口位于 `.scratch/package-install/lib/robot_safecontrol_moveit/`，可执行程序使用默认内核路径，通过独立两个 ROS 进程、原始 PointCloud2 字节比较、共享发布、完整预期来源的 JAX reader、单次准备和无效输入撤销验收，输出 `INSTALLED_COLLISION_SCENE_PROCESS_ACCEPTANCE_PASSED`，退出码 0。源码进程与安装进程验收均未传入 `portable_oscbf_root` override。

- 构建安装日志：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/package-install-final.log`。
- 安装文件清单：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/package-install-record.txt`。
- 安装探针：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/installed_scene_probe.py`。
- 安装进程验收输出：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/installed-scene-final.log`。
- 安装验收输入与子进程日志：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/installed-scene-probe/`。

13 份当前源码与安装副本的 SHA-256 相等，包括共享传输模块、全部新增与修改的 Python 生产入口、配置、launch、package.xml 和场景服务文档；结果位于交接 JSON 的 `installation.matching_installed_files`。受影响 Python 入口和新 launch 通过 `python -m compileall -q`；`git diff --check` 通过；`git merge --ff-only main` 返回 `Already up to date`。

## TDD 命令记录

各命令均从本工作区执行，使用上述环境与 `.scratch/pytest/` basetemp。下列失败均为对应实现前的公开接口或验收断言，随后由当前完整 322 项结果覆盖。

| 公开行为 | 实现前命令与退出码 | 实现后的直接结果 |
|---|---|---|
| 独立共享 writer/reader | `python -m pytest portable_oscbf/tests/test_shared_collision_scene.py --basetemp="$PWD/.scratch/pytest/shared-red" -q`，4 | 首项真实进程用例 1 passed |
| 时间模型原来源 | 同文件，`--basetemp="$PWD/.scratch/pytest/shared-source-red"`，1 | `shared-source-green`：5 passed |
| 完整场景管线 | `python -m pytest tests/test_collision_scene_server.py --basetemp="$PWD/.scratch/pytest/pipeline-red" -q`，4 | 管线与既有 adapter：11 passed |
| 来源 manifest | 同 server 文件，`-k provenance --basetemp="$PWD/.scratch/pytest/sources-red"`，1 | 来源、管线与共享接口集合：17 passed |
| 原始逐点数据转发 | `python -m pytest tests/test_timed_lidar_transport.py --basetemp="$PWD/.scratch/pytest/timed-red" -q`，4 | 真实编码/解码与 ROS 进程验收通过 |
| 现有普通文件保留 | shared 文件，`-k unrelated --basetemp="$PWD/.scratch/pytest/shared-path-red"`，1 | `shared-path-green`：11 passed |
| 当前 epoch 内准备 | shared 文件，`-k before_its_epoch --basetemp="$PWD/.scratch/pytest/writer-epoch-red"`，1 | 当前完整真实集合通过 |
| 已加载来源身份 | `python -m pytest tests/test_collision_scene_server.py -k actual_loaded_source --basetemp="$PWD/.scratch/pytest/review-source-red" --junitxml="$PWD/.scratch/review-source-red.xml" -q`，1 | `review-source-green`：5 passed；另含几何、验证与等价 manifest 回归 |
| 准备期间公开状态 | `python -m pytest portable_oscbf/tests/test_shared_collision_scene.py -k removes_the_public_scene --basetemp="$PWD/.scratch/pytest/review-pending-red" --junitxml="$PWD/.scratch/review-pending-red.xml" -q`，1 | `review-pending-green`：1 passed；真实方法入口暂停后观察 Pending，并覆盖期间健康变化 |
| FIFO 构造 | shared 文件，`-k rejects_a_fifo --basetemp="$PWD/.scratch/pytest/review-fifo-red" --junitxml="$PWD/.scratch/review-fifo-red.xml"`，1 | `review-fifo-green`：1 passed，真实独立进程及时收到 ValueError |
| 默认源码入口 | server 文件，`-k real_bridge_and_server --basetemp="$PWD/.scratch/pytest/review-default-red" --junitxml="$PWD/.scratch/review-default-red.xml"`，1 | `review-default-green`：1 passed；隔离安装默认入口也通过 |

上述四项 `red/green` JUnit 均位于工作区 `.scratch/`，完整绝对路径、大小与 SHA-256 保存在交接 JSON 的 `tdd_junit`。来源验证使用同路径真实 CalibrationRecord 内容变化、独立 ClockMapping/run、SelfFilterBounds、ray 几何及验证；等价 manifest 的路径与 JSON 排列保持相同来源身份。省略低层 reader 的来源参数会拒绝生产来源。准备期测试通过 `sys.setprofile` 暂停真实 `CollisionSafety.prepare_scene` 调用入口，继续执行原方法并核验最终结果，未替换方法、数值或返回值。

## 冻结文件身份

以下为 SHA-256；路径相对于实现工作区。

```text
fcb7a70794218523dd4da9b71f27abc96a83cfd3a7c49dc29fec731f6c5d2157  portable_oscbf/work/shared_collision_scene.py
2e45980d356c14b2a81fce737c003afd10f41aba5b5dc06f2a1fded6f2a3cc9d  src/robot_safecontrol_moveit/collision_scene_pipeline.py
8dee195ccdaf7cb5213c5fd404151e36dcaf15ba7263a72536bf37603f8e2736  src/robot_safecontrol_moveit/collision_scene_sources.py
9a08ad7143d2ac0f648772c4cf881a694e96b3ff18b8bdc77304359aef0186da  src/robot_safecontrol_moveit/collision_scene_server.py
893de7428d138c7e63c80a7dc761cff43acb6a54094a64d9c43747de5093010c  src/robot_safecontrol_moveit/timed_lidar_transport.py
963783fa032d6256904cb0a200f37d1ba767a0606ca9e323fc74eb808fc25210  src/robot_safecontrol_moveit/lidar_self_filter_adapter.py
444c46328753bfbe34310b99ed0cbaedcd80dbac0920017f1acbb3682e36e2c6  src/robot_safecontrol_moveit/oscbf_trajectory.py
3395bf03d09c10732f1a42fdb3385cf5336c526e951590e86c5cd229941acf4a  src/robot_safecontrol_moveit/perception_bridge.py
681065cd70c73301e3dbbf7d3b2065168ab34003933351e6418b0fa79a9b13ae  src/robot_safecontrol_moveit/ros_conventions.py
8ee8f0de606a2b497d2ab25826769cc84a8344c5d97d05d0c52c4b5f6d259808  portable_oscbf/tests/test_shared_collision_scene.py
4d6764c6c81d063bb96c026e0082bd4bda2cded151d2772c07633ac426001474  tests/test_collision_scene_server.py
bb20d0c22a4bc2d32fb990a26288ed7d4f69a64cb4994dd95282ec5fdb4fcbc4  tests/test_timed_lidar_transport.py
ffb3f929eb985f97ab7a6cd496443d3294e022583fa63843cbcb118e618e0d77  setup.py
ee05eb01d065b5a26b257910c40e36bc56835a743fa493a3aa8a8b684c0eb765  package.xml
6cd6ef26a7a28a0ad1a6206faf16b598da267fa11abbbaaf4400bfc13d5bbb89  config/collision_scene_server.yaml
a0bb1dee5e0b29e32252f816158f44a985556a0f1156d2ad89aadcdfb9f68190  launch/collision_scene_server.launch.py
35e456f3ffbc0f00b530e9009ef5fc2bbdb296b1a971d5cddd366c299ab1ca32  docs/modules/collision_scene_server.md
a9f90a33aebdaf69eaeb569743a2898b352a171c95d9a947391b179b42663acb  docs/modules/portable_oscbf.md
2d9aefa44cb1b0948028a0b7c51cf6a0e4ee551e0d974b700c580116931f8a04  docs/modules/lidar_self_filter.md
b4d2d9b69a7d6aa9fac44a1eb78b6b0d01137e235de958cb82b68bc1bf8e915f  docs/modules/portable-oscbf/scene-preparation.md
da85d53dd758da719358dbf1d5c32d114b3be275bce5f6944b459b87b5b9c1da  docs/modules/portable-oscbf/dependencies.md
634799e4c8d3aaad0c3b6472ba2a810c235f170165c17b3068a50f673d1410b0  docs/README.md
```

验证制品 SHA-256：源码清单 `379a08a0bc5919094cfcbb987f37d2cc81b03f75d1249e3bd3257d378868f263`；JUnit `ca6336fd5698256ce8c77d219f71f37bf0bda849a4ed375dd677b0a5ee1545c1`；安装进程输出 `08ec3cc88cdf41d9bbb29a20ba95fc942bf1367d776e3e90117217af6e2d8448`；环境脚本 `05ee3dc79ddcf3c381f40fad174bbe0998069d4283e13f956cc91ffae2af8970`；交接 JSON `b9d8f33923d8e8b706ee59b8bfaac7d2245f9dac4f49f589ad05598b976db2db`。

JSON 使用标准库 `json` 生成和读取，包含 `files` 的相对路径、SHA-256 与字节数量，以及 `worktree/base/head/main/branch`、全部测试与安装证据的绝对路径和 hash。生成及安装副本比较脚本位于 `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server/.scratch/build_scene_handoff.py`。工程文件冻结后仅更新本任务记录。

全量入口未运行，其中包含旧有 mock 机制。实际验证全部使用真实文件、mesh、生产公开对象、ROS 进程、共享内存及 JAX。解析资料的适用范围为软件接口；现场参数、物理几何、时钟、真实 LiDAR 回放和硬件能力分别需要对应任务证据。
