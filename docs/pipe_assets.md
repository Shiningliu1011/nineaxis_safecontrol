# 管道路径与运动资产的 Python 入口

本工具在 Linux 执行原始 MATLAB 资料中的管道建模、路径画笔、NURBS 插值、进给规划、固定周期插补及连续姿态生成。程序位于 `src/robot_safecontrol_moveit/pipe_assets/`，命令行入口为 `scripts/pipe_assets.py`，安装后的命令为 `pipe_assets`。

来源文件和样本身份见[资产清单](planning/nineaxis-full-project/asset-inventory.md)。原始 ZIP、MAT 和 CSV 保持原内容。输入与输出使用工件坐标、mm、s 和源工具 `+Z`；画布角度使用 deg，局部曲面角度及角运动使用 rad。`u` 与 `arc_mm` 分别保存。

## 环境

当前电脑已经建立独立环境 `.scratch/pipe-assets-runtime/venv`，其中安装了 `requirements-pipe-assets.txt` 的全部依赖。使用时从仓库根目录执行：

```bash
source .scratch/pipe-assets-runtime/venv/bin/activate
export PYTHONPATH="$PWD/src"
export TMPDIR="$PWD/.scratch/pipe-assets-runtime/tmp"
export MPLCONFIGDIR="$PWD/.scratch/pipe-assets-runtime/matplotlib"
mkdir -p "$TMPDIR" "$MPLCONFIGDIR"
```

其他电脑可以用 Python 的 `virtualenv` 创建独立环境，然后执行：

```bash
python -m pip install -r requirements-pipe-assets.txt
```

画笔使用 Matplotlib 和 Tk。中文字体为 SimSun，英文与数字为 Times New Roman；程序检查实际字体身份。命令行数值生成可以独立使用。MAT 文件由 SciPy 与 pymatreader/h5py 读取，优化使用 SciPy HiGHS 与 OSQP。

## 路径画笔

```bash
python scripts/pipe_assets.py painter --model demo \
  --output .scratch/pipe-assets/paths
```

模型可选择 `demo`、`combined`、`complex`，分别对应原始两个管段模型、五个管段组合和空间多管段模型。自定义 Python 模型使用 `--model-factory MODULE:FUNCTION`，函数返回相同字段的模型。

画笔支持重新绘画、续画、拖动或点击添加关键点、右键删除、Esc 取消、Ctrl+Z 撤销、简化笔迹、设置工作距离和采样间隔、图层显示、示例路径、曲率窗口、独立画布、管段定位、平移和缩放。MAT 导出保留可编辑关键点、路径定义、设置和几何身份，CSV 保存完整采样字段。

程序化调用保留源接口名称。例如：

```python
from robot_safecontrol_moveit.pipe_assets.pipe_painter import pipe_painter

app = pipe_painter(options={"Visible": "off", "PathDirectory": ".scratch/pipe-assets/paths"})
app.setPath([[10, 0], [50, 0], [100, 0]])
samples = app.getSamples()
app.export("pipe_path")
app.close()
```

## 完整生成

使用仓库保存的真实原始路径，重新拟合、规划、插补并生成姿态：

```bash
python scripts/pipe_assets.py run \
  --input tests/data/pipe_assets/pipe_path_20261013_180806.mat \
  --output .scratch/pipe-assets-runs/source-path
```

输出目录必须尚未存在。程序依次生成 `fit/`、`plan/`、`motion/`、`pose/` 和 `manifest.json`。manifest 记录输入 SHA-256、程序身份、实际依赖、耗时与输出文件校验值。求解、输入身份或约束验证失败时直接报告错误。

纯 TCP CSV 可以使用 `run --skip-pose` 完成位置运动生成；完整姿态生成需要原始 MAT 中的表面对应信息。源中尚未配置的物理角运动限制保留其配置状态，验证结果保存相应状态。

## 分别执行

无界面生成示例参考，或者从指定模型重新生成已保存的编辑路径：

```bash
python scripts/pipe_assets.py reference --model combined \
  --output .scratch/pipe-assets-runs/combined-reference
python scripts/pipe_assets.py reference --model demo \
  --input tests/data/pipe_assets/pipe_path_20261013_180806.mat \
  --output .scratch/pipe-assets-runs/source-reference
```

拟合、规划与插补分别执行：

```bash
python scripts/pipe_assets.py fit \
  --input tests/data/pipe_assets/pipe_path_20261013_180806.mat \
  --output .scratch/pipe-assets-runs/fit
python scripts/pipe_assets.py plan --fit-dir .scratch/pipe-assets-runs/fit \
  --output .scratch/pipe-assets-runs/plan
python scripts/pipe_assets.py interpolate \
  --input .scratch/pipe-assets-runs/plan/nurbs_blocks.mat \
  --output .scratch/pipe-assets-runs/motion
python scripts/pipe_assets.py pose --fit-dir .scratch/pipe-assets-runs/fit \
  --input tests/data/pipe_assets/pipe_path_20261013_180806.mat \
  --offline .scratch/pipe-assets-runs/plan/nurbs_blocks.mat \
  --output .scratch/pipe-assets-runs/pose
```

详细算法和接口见[进给与插补](pipe_feed_assets.md)、[连续姿态](pipe_pose_assets.md)。机器人参考转换、控制准入及整段系统验收由[完整项目地图](planning/nineaxis-full-project/README.md)中的对应任务管理。

## 验证

真实测试资料位于 `tests/data/pipe_assets/`，`provenance.json` 保存各文件的原 ZIP 成员与 SHA-256；姿态子集另有原始行号与字段校验值。原始 CSV/TXT 的 CRLF 行尾由 `.gitattributes` 保留，Git 不转换其字节内容。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg \
python -m pytest tests/test_pipe_*.py -q \
  --basetemp .scratch/pipe-assets-runtime/pytest
```

本次命令、实际误差和完整运行记录保存在 `.trellis/tasks/10-27-python-path-assets/implement.md`，基线控制能力见[基线核验](planning/nineaxis-full-project/baseline-audit.md)。

2026-10-27 最终执行结果为 60 项测试通过，136.72 s。独立运行完整 `run` 命令得到 4756 个点、9.51 s，重新拟合和规划后的位置与原始保存结果最大分量差为 `3.49e-12 mm`。当前 40 个 Python 源文件与全部 11 个输出文件的 SHA-256 已回读核验。Ruff 检查、mypy、安装目录构建和模块入口检查通过。

画笔通过真实鼠标、键盘、控件事件及 TkAgg 主窗口、独立画布运行检查；窗口关闭后没有遗留 figure。图形验证采用对象、字符覆盖与事件结果，本轮没有进行截图检查。

跨电脑通过 Git 同步 `main` 中的程序、真实测试资料、依赖和文档，在目标电脑按本页安装环境，并将新的运行结果写入新目录。完整原始资料另行复制 `.scratch/full-project-assets/full-project-assets.tar.gz`；本轮独立验证快照和生成结果位于 `.scratch/pipe-assets-runtime/python-path-assets.tar.gz`。各自目录中的 identity JSON 保存 SHA-256 和回读核验结果。交接包不包含 Python 环境。
