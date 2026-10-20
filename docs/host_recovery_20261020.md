# 2026-10-20 主机恢复

## 项目与环境

恢复项目在 `/home/lsn/robot/robot_safecontrol`。
迁移包原位置为 `/home/lsn/robot-safecontrol`，包内 `SHA256SUMS` 全部通过。
来源说明、文件清单、环境及安装资料已校验并保存到
`output/host-recovery-migration-evidence/`；重复分卷已按用户授权删除。
该目录还保存了三份笔记本历史基准原件，108 份保留资料逐文件校验一致；
原迁移目录中的重复内容已清理。
9 个 tar/zstd 分卷合计 26,774,385,704 字节；来源清单包含 86,374 个条目，
普通文件合计 42,737,916,946 字节。恢复后从归档提交
`68a52279423318d3773e8d6f61c21c10f1d53a24` 快进至远程
`6afe1d34a0d1d33b85f91c53dbe103023ce718ec`。

本机为 Ubuntu 22.04.5、Linux 6.8.0-124、Intel Core i7-14700，
28 个逻辑 CPU、33,247,371,264 字节内存。使用系统 Python 3.10.12、
ROS 2 Humble、FCL 0.7.0、Eigen 3.4.0、pybind11 2.9.1。
GPU 为 NVIDIA GeForce RTX 5060 Ti、8,151 MiB，驱动 595.71.05；
本次控制与评价使用 CPU。
默认 shell 的 Python 来自 Miniconda，项目验证使用以下独立环境。

主环境在 `.scratch/host-recovery/venv`，由系统 Python 创建，
启用 system-site-packages 以使用 ROS。固定依赖来源：

- `portable_oscbf/requirements.txt`
- `portable_oscbf/requirements-geometry.txt`
- `portable_oscbf/requirements-collision-test.txt`
- `requirements-static-recording.txt`
- `requirements-static-report.txt`

另外安装 python-can 4.6.1、mujoco 3.16.0、python-fcl 0.7.0.11、
osqp 1.1.3、urdf-parser-py 0.0.4、setuptools 65.5.1、colcon-core 0.21.3、
pytest 8.3.5、pytest-rerunfailures 15.1。ROS launch testing 使用 pytest 插件自动加载。

本机环境入口：

```bash
cd /home/lsn/robot/robot_safecontrol
source .scratch/host-recovery/env.sh
```

该入口加载 `/opt/ros/humble/setup.bash`、虚拟环境和 `install/setup.bash`，
设置 `PYTHONNOUSERSITE=1`、`ROS_LOCALHOST_ONLY=1`、`ROS_DOMAIN_ID=197`、
`JAX_PLATFORMS=cpu`、`MUJOCO_GL=egl`、`MPLBACKEND=Agg`，
以及 `OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1`。
`OPENBLAS_CORETYPE=Haswell` 固定 NumPy/OpenBLAS 的计算实现，使闭合模型来源
记录能够按原始精度重复生成；该设置参与项目的环境身份计算。
中间文件、ROS 日志、Matplotlib 缓存均位于项目 `.scratch/host-recovery/`。
这些环境目录不通过 Git 同步，另一台设备按上述固定依赖重建。
本机 i7-14700 的逻辑 CPU 0–15 属于 Performance-core；完整回归和周期测量
使用 `taskset -c 0-15`。该编号仅适用于本机，其他设备需要核查实际 CPU 布局。

独立绘图环境为 `.scratch/host-recovery/report-venv`，安装上述绘图、记录、
控制及几何依赖，并通过 `pip install --no-deps --no-build-isolation .`
构建安装原生模块。调用前加载主环境；报告入口保留 ROS 的 Python 路径。
主环境和绘图环境的 `pip check` 均通过。

## Skills、字体与本地资料

25 个个人 skill 完整安装在 `/home/lsn/.codex/skills`，
与迁移包内容一致；`/home/lsn/.agents/skills` 保存对应符号链接。
项目 20 个 skill 以及 `.codex/agents`、`.codex/hooks`、配置均保留。
当前机器原先没有全局工作约定文件，已安装包内相应文件。
Trellis CLI 0.6.17 安装在 `~/.local/bin`。项目已设置为受信任，
`codex features list` 显示 hooks 已启用；工作流注入脚本手动执行通过。
Codex 的首次 hook 审核属于客户端交互，脚本执行检查不能证明该审核已完成。

系统通用 skill 检查器通过 19 个项目 skill，其固定字段集合不接受
另一个 skill 已有的 `disable-model-invocation`。来源 skill 内容完整保留，
本次未修改上游元数据。个人 skill 文件与包内版本逐目录比较一致。

Times New Roman 和 SimSun 安装在
`~/.local/share/fonts/robot-safecontrol/`；fontconfig 和 Matplotlib 均找到正确字体。
Matplotlib 使用本次安装后生成的独立缓存。
Times New Roman Regular SHA-256：
`4e98adeff8ccc8ef4e3ece8d4547e288ff85fdc9c7ca711a4599c234874bbe86`；
SimSun SHA-256：
`2e3cc9dbd787c827f73496f01633bba430e15c5db4d4f6ed9938386ea01f0171`。

大然电机本地检索环境按 `大然电机/local_rag/requirements.lock.txt` 重建。
状态检查确认离线模式、2,749 个片段、512 维向量，
`BAAI/bge-small-zh-v1.5` 模型及资料库版本 `20261001`。
实际查询“B06-PA-36-G PID 参数”通过，14 项测试通过，依赖检查通过。

## 构建和验证入口

```bash
python -m colcon build --symlink-install \
  --base-paths "$PWD" "$PWD/src/aeb_rrtstar_ompl" \
  /home/lsn/robot/pymoveit2_ws/src/pymoveit2 \
  --packages-select aeb_rrtstar_ompl robot_safecontrol_moveit pymoveit2
bash scripts/agent_check.sh
taskset -c 0-15 bash run_all_tests.sh
python scripts/validate_static_storage.py \
  --output output/static-storage-validation --resource-root "$PWD"
```

本机三个 package 构建通过，原生模块、ROS CLI 导入和帮助入口通过。
快速检查的 69 项纯逻辑测试通过。
闭合模型与几何生成物的 10 项检查通过，源 STL、URDF、生成器及来源记录均保留。
主包回归为 678 passed、1 skipped，用时 904.57 s；
跳过项是需要单独启用的 Linux vcan 验收。
控制内核逐文件回归为 486 passed、34 skipped，C++ CTest 为 4 passed；
三个部分退出码均为 0，完整日志为 `.scratch/host-recovery/full-regression-final.log`。
该回归中的 300 步控制性能检查 P95 为 17.163 ms、最大 50.726 ms，
超过 20 ms 的 2 步占 0.67%，符合项目现有 1% 上限，QP 失败为 0。
报告保存在 `output/host-recovery-regression/final/`；
来源包中的 tracking_report 三个文件及两份 M10 报告按原始 SHA-256 保留。
完整运行结果和限制见[实验接手记录](static_obstacle_experiments_handoff.md)。
日志在 `.scratch/host-recovery/`，完整真实存储验证在
`output/static-storage-validation-05/validation.json`。

六类图表通过独立绘图 Python 实际生成，位于
`picture/static-storage-validation/`。字体身份、文字覆盖、
文件 SHA-256、PNG 完整性和尺寸检查通过。没有执行视觉检查。

本次来源检查与产物隔离的定向复核：

- 存储、评价、实验管理：25 passed，33.87 s。
- ROS smoke 与完整流程：14 passed；完整流程另以
  `-W error::pytest.PytestUnhandledThreadExceptionWarning` 复核，
  1 passed，35.70 s，无警告。
- baseline、M8、M6、perception 四个控制内核文件：10 passed。
- M8 与 perception 分别从空测试目录运行并生成真实基准：
  5 passed（111.12 s）、3 passed（62.38 s）。
- 8 份历史基准与报告在复核后逐文件匹配原件 SHA-256。

命令与输出保存在 `.scratch/host-recovery/check_affected_outputs.sh`、
`affected-outputs.log`、`check_final_evidence.sh`、`final-evidence.log`。
单独运行的验证入口与结果为 `check_standalone_baselines.sh` 和
`standalone-baselines.log`，同样位于该目录。

## 数据身份

大型数据继续保留于两台机器各自的项目目录，通过资料包单独传输。
本次来源归档提交为 `68a52279423318d3773e8d6f61c21c10f1d53a24`。

| 文件 | SHA-256 |
|---|---|
| `output/host-recovery-migration-evidence/project-files.json` | `c5bc7df6ffff06404d481563162d11ac487215337c189ebe9e68f0edd1f0f268` |
| `output/host-recovery-migration-evidence/PACKAGE-INFO.json` | `45fbbd4bc4366fa429dc55aaace11fcc6f0fff3188dfc2c35ab0d584c338de20` |
| `output/host-recovery-migration-evidence/SHA256SUMS` | `1a8023b9ff86b4867b52566713d8648f03e32a47e663a92c41653044bc85fa0d` |
| `output/host-recovery-data-inventory.json` | `9ae2b6d70670fc46c907d3d801f4f5a1163f084f250126a2bca56b8f3661dd8a` |
| `output/host-recovery-final-retention.json` | `98ff89ffd41535b334ae22a269f6f25e09e47bb9229f5154ff100fd3125b23b4` |
| `output/static-storage-validation-05/validation.json` | `740eafa49b709b5b70d2d744b854f1cbe1af45d1fc4612181011af78a9005209` |

恢复数据最终核查为 `pass`：14,284 个文件、23,143,586,250 字节，
全部匹配来源清单的逐文件 SHA-256，无差异。
检查范围包含原始实验、轨迹、标定 bag、本地检索资料及大型文件；
完整路径、文件数量和每个文件的校验值保存在上述 inventory。

原始实验数据位于 `output/static-experiments/`；
原始标定 bag 位于 `.scratch/targetless-calibration-trial/bags/` 和
`.scratch/targetless-calibration-trial/bags-archive/`。
每个文件的完整来源身份和校验值见上述 `project-files.json`。
新的紧凑副本包含 `storage_migration.json`，逐项保存原始文件 SHA-256、
manifest、转换数据库 SHA-256 和逐流记录数量。

三份历史基准测试产物已与笔记本原件逐项匹配 SHA-256，
保存于本机 `output/` 及 `output/host-recovery-migration-evidence/`。
本机重新运行生成的产物单独保存在 `output/host-recovery-regression/final/`。
历史文件身份如下：

| 相对于项目的路径 | 原始字节 | 原始 SHA-256 |
|---|---:|---|
| `output/baseline_tracking.npz` | 725910 | `0513c83c64fb793680da6bd4cf47da9217266d1e2df5ce3cf89d0a58ec70bef5` |
| `output/oscbf_m6_perf.md` | 853 | `68e9dd13baa339224240238f1fa5c6558622d43c8c178607a85b771593dca3e5` |
| `output/oscbf_m8_nullspace.md` | 516 | `dfa34726cf2b7d9ac8b34f127818590e1189ce20f9c81b8a522374dd49396369` |

笔记本迁移时的项目位置为 `/home/lsn/robot/robot_safecontrol`，
原迁移包目录为该项目下的 `output/robot-safecontrol-transfer/`。
本次从 `/home/lsn/robot-safecontrol/` 接收的三份原件均符合上述身份。
原始 S0/S2/S4/S5 实验、标定及研究资料按完整清单独立核查。

完整回归入口为每次运行创建 `.scratch/regression-XXXXXXXX/`，
主包 pytest 产物保存在其中的 `main/`，控制内核基准与报告保存在 `portable/`。
直接执行控制内核测试时，产物默认为 `.scratch/portable-oscbf-tests/`，
可通过 `OSCBF_TEST_OUTPUT_DIR` 指定本次测试的独立目录；
baseline、M6、M8 和 perception 测试共享该目录。
M6、M8 和 perception 单独运行时检查基准身份，缺少当前基准时实际执行
baseline 生成过程。
ROS 完整流程和性能测试的报告使用 pytest 提供的独立目录。

GitHub 同步使用工作分支 `codex/static-obstacle-experiments`。
本机 `gh` 已登录 `Shiningliu1011`，使用 HTTPS 和系统 keyring，
远程获取与 `git push --dry-run` 均通过。

本次所需系统依赖已经存在，其余安装在用户目录，没有需要执行的管理员命令。
