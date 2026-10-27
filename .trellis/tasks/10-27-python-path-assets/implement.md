# 实施与验证

## 公开测试范围

用户指定原 MATLAB 功能及真实资料为预期行为。测试通过原公开函数、画笔状态操作、MAT/CSV 导出和完整命令行观察结果。期望值使用源保存数据、解析几何及独立数值方法。

## 执行安排

- [x] 完成本次原始资产和 M01–M12 基线核验。
- [x] 逐项读取原程序并转换几何、参考路径、NURBS、运动规划与姿态公开接口。
- [x] 完成画笔交互、MAT I/O、命令行、依赖及使用文档。
- [x] 每个行为先执行失败测试，再完成该行为并执行通过测试；各负责人记录命令及结果。
- [x] 对真实资料执行完整生成链，核对原始数据、单位、算法约束、连续性和导出内容。
- [x] 检查代码、文档链接、安装入口以及用户已有文档校验值。
- [x] 按完成条件写入任务评论、更新地图和本地入口，保存未完成部分的具体缺口。

## 隔离运行

控制基线使用 `.scratch/host-recovery/venv` 的真实项目环境；原始 MAT 核查使用 `.scratch/predictive-coordination-spec/mat-reader-env`。Python 资产使用独立环境 `.scratch/pipe-assets-runtime/venv`：Python 3.10.12、NumPy 2.2.6、SciPy 1.15.3、Matplotlib 3.10.9、pymatreader 1.3.2、h5py 3.16.0、OSQP 1.1.3。实际依赖安装日志为 `.scratch/pipe-assets-runtime/install.log`。

`TMPDIR`、pytest `--basetemp`、图形缓存、日志和生成输出均指向本次 `.scratch` 目录。交互测试使用真实 Matplotlib canvas；优化使用真实 HiGHS 与 OSQP。没有使用 mock 或截图定位，没有连接真实设备。

## 实际结果

工作区为 `/home/lsn/robot/robot_safecontrol`，分支 `main`，验证基线为 `be963a6bdac16ae48a8284e2a093dc3f41be248d`。代码、测试、依赖、文档和任务记录通过 `main` 同步。用户原有两份静态实验文档的 SHA-256 持续保持一致，并作为独立文档提交同步。

### 基线与来源

控制基线实际通过 93 项 pytest、4 项既有插件 ctest，以及真实 MuJoCo、ROS sim/shadow、两个 live 入口拒绝探针。范围和限制见 `docs/planning/nineaxis-full-project/baseline-audit.md`，命令、环境与 208 个关键文件身份位于 `.scratch/full-project-baseline/`。

原 ZIP 为 `/home/lsn/robot/NURBS曲线.zip`，SHA-256 `907fbfc97e961aca04e83b69b8a38839aff2e4f6e32e98c8e89f7a2c1d06e58d`。来源、版本、许可状态、公开接口和真实样本见 `docs/planning/nineaxis-full-project/asset-inventory.md`。原资料交接包为 `.scratch/full-project-assets/full-project-assets.tar.gz`，SHA-256 `958e0eea9168dfe7522935d8c7919651b8231a2ccc08fa3dd8293d028346d11c`，105 个文件已逐项回读核验。

### 公开接口与最终测试

从仓库根目录执行：

```bash
export PYTHONPATH="$PWD/src"
export TMPDIR="$PWD/.scratch/pipe-assets-runtime/tmp"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export MPLBACKEND=Agg
export MPLCONFIGDIR="$PWD/.scratch/pipe-assets-runtime/matplotlib"
.scratch/pipe-assets-runtime/venv/bin/python -m pytest \
  tests/test_pipe_*.py -q \
  --basetemp .scratch/pipe-assets-runtime/pytest-final
```

退出码 0，`60 passed in 136.72s`。直接日志为 `.scratch/pipe-assets-runtime/pytest-final.log`。测试覆盖原 407/480/743 点路径、解析模型接缝、曲率、三次插值、弧长和前三阶导数、真实 LP/QP、完整几何与运动约束、A2/A3/A4 导出与重载、单块时间律、4 mm 单区间轴向与姿态参考、MAT/CSV、画笔公开操作与真实鼠标键盘事件。

TDD 各组的失败与通过记录分别位于 `.scratch/pipe-geometry/`、`.scratch/pipe-feed/validation.md`、`.scratch/pipe-pose/fixes/verification.md` 和 `.scratch/pipe-assets-runtime/`。GUI 事件测试、输入形状及数值 CLI 无图形行为均通过公开接口验证。LP 第二目标失败分支采用源码核对；正常候选真实求解通过，没有人为制造求解器故障。

### 完整生成与数值身份

在上述环境下实际执行：

```bash
.scratch/pipe-assets-runtime/venv/bin/python scripts/pipe_assets.py run \
  --input tests/data/pipe_assets/pipe_path_20261013_180806.mat \
  --output .scratch/pipe-assets-runtime/final-complete
.scratch/pipe-assets-runtime/venv/bin/python \
  .scratch/pipe-assets-runtime/verify_final.py
```

两项退出码均为 0。完整生成实际耗时 9.524 s，从原始 480 点执行新拟合、PCI 扫描、新 A2 联合整数调度、A3 固定周期插补及 A4 连续姿态。输出 4756 点、9.51 s，所有已配置检查通过，共同时间伸长系数为 1。最大轴向误差 `0.0133011772 rad`，最大角速度 `1.4749246271 rad/s`，最大角加速度 `7.7975406917 rad/s²`；源角运动限制状态保持 `not_configured`。

新生成结果与源保存结果的最大分量差为位置 `3.48166e-12 mm`、速度 `4.88499e-11 mm/s`、加速度 `5.32258e-9 mm/s²`、jerk `5.88946e-7 mm/s³`、参数 `u` 为 `7.93810e-15`；时间逐元素一致。实际校验值保存在 `.scratch/pipe-assets-runtime/final-verification.json`，40 个当前 Python 源文件及 11 个输出文件均与最终 manifest 一致。直接生成日志为 `final-complete.log`。

复用原保存时间律的独立重建证据另见 `.scratch/pipe-feed/numeric-evidence.json`，它与新拟合、新规划的完整生成分别记录。几何三组样本与源 TCP 最大差 `1.53e-12 mm`。

### 图形与安装入口

实际执行 `.scratch/pipe-assets-runtime/check_tk_painter.py`，使用 TkAgg 打开主窗口及独立画布，生成 407 点示例并执行真实 canvas 绘制；定时关闭后 `remaining_figures=[]`，退出码 0。日志为 `.scratch/pipe-assets-runtime/tk-painter.log`。字体使用真实 SimSun、Times New Roman，并检查字符覆盖；本轮没有截图检查。

实际执行 `python setup.py build_py --build-lib .scratch/pipe-assets-runtime/package-build`，退出码 0，日志为 `build-final.log`。随后以该构建目录为 `PYTHONPATH` 执行 `python -m robot_safecontrol_moveit.pipe_assets --help`，退出码 0，列出七个实际子命令。安装声明包含独立 `pipe-assets` extras、console entry point、文档和真实资料。

### 静态与独立复审

使用 `.scratch/pipe-geometry/check-tools` 的工具，以独立环境 Python 执行：

```bash
PYTHONPATH="$PWD/.scratch/pipe-geometry/check-tools" \
.scratch/pipe-assets-runtime/venv/bin/python -m ruff check \
  src/robot_safecontrol_moveit/pipe_assets tests/test_pipe_*.py \
  scripts/pipe_assets.py --select I,F,E4,E7,E9,SIM102 --output-format concise
PYTHONPATH="$PWD/.scratch/pipe-geometry/check-tools" \
.scratch/pipe-assets-runtime/venv/bin/python -m mypy \
  --ignore-missing-imports --follow-imports=silent \
  --cache-dir .scratch/pipe-assets-runtime/mypy-cache \
  src/robot_safecontrol_moveit/pipe_assets
```

两项退出码均为 0。Ruff 全部通过；mypy 报告 `Success: no issues found in 40 source files`。日志为 `ruff-final.log` 与 `mypy-final.log`。代码规范与需求由两个独立审查者核对，最终范围、证据与限制见 `research/review.md`。

### 交接与范围

Python 交接包为 `.scratch/pipe-assets-runtime/python-path-assets.tar.gz`，包括独立验证时的程序、测试资料、文档、任务记录和选定最终证据。校验值与逐项回读结果位于同目录 `python-bundle-identity.json`。日常开发通过 Git 获取当前程序、测试和文档；Python 环境按依赖文件在目标电脑安装。原始 CSV/TXT 使用 `.gitattributes` 保留 CRLF 行尾和逐字节身份；同步前再次核对 69 个程序、测试与资料文件，均与通过测试的独立快照一致。

本次提供工件坐标、mm、源工具 +Z 的独立资产生成链。机器人 M01/M02 集成、工艺物理限制、MPC、真实设备与正式实验继续由地图相应任务管理。没有执行 spec promotion；没有更改控制代码或真实设备准入条件。

### 地图与交付核验

[[01] 核验完整项目基线与 M01–M12 资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/148#issuecomment-6437016447)及[[02] 取得完整工件、路径与 MATLAB 程序资产](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/135#issuecomment-6437698426)均已记录真实结果，并以 `completed` 关闭；地图正文和本地入口已更新。

实际执行 `.scratch/full-project-frontier/verify_delivery.py --completed 148 135`，退出码 0。保留全部 59 个子任务顺序与 179 条前置关系数量，只有本轮两项状态变化；其他任务正文、标题、标签与指派不变。地图正文与本次文件一致，45 个关联文档本地链接通过检查，HEAD 和用户两份文档校验值保持一致。直接结果为 `.scratch/full-project-frontier/delivery-verification.json`。

新的可领取入口为[[04] 完整项目技术复用与实施依赖（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/149)及[[05] M02 九轴模型、TCP 与标定接口（需你拍板）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/136)。两项需要用户参与决定，本轮保持开放。
