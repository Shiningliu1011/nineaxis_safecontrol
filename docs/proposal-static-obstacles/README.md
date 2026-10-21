# 静态障碍物开题资料

[数据与结论](results-and-conclusions.md)提供实验配置、数值、适用范围，以及可修改后用于开题报告、绪论和 PPT 的文字。
[图片索引](figures.md)提供图片预览、图注及推荐用途。
[指标 CSV](data/metrics.csv)和[指标 JSON](data/metrics.json)包含十份开发记录的完整精度摘要。

截至本次暂停，#172–#181 共十项子任务已经关闭，#182–#183 仍开放。
正式采集为 0/1080；这些计数表示工单和采集进度，不能换算成研究完成百分比。
全部资料来自 sim。按用户要求暂停实验，开题结束后由用户通知继续。

## 另一台设备获取

在已有项目目录执行：

```bash
git fetch origin
git switch codex/static-obstacle-experiments
git pull --ff-only origin codex/static-obstacle-experiments
```

存在本地修改时按[双设备流程](../two_machine_workflow.md)保存修改。
本资料的 Markdown、CSV、压缩 CSV、JSON、NPZ 和全部图片均随 Git 同步，合计约 52 MiB。
直接使用 PNG 插入报告或 PPT；汇总图另有可编辑 SVG 和 PDF。
阅读、引用、修改这些文件无需启动 ROS、仿真或恢复大型数据库。

## 数据组成

- `data/s0/`、`data/s2/`、`data/s4/`：三份完整路径样本，各含原始报告、manifest、配置、路径、来源身份、全部评价反馈的 `feedback-full.csv.gz`，以及方便编辑的 `feedback-25hz.csv`。
- `data/s4-A/`、`data/s4-C/`：同配置恢复规则配对，报告保留全部恢复事件及暂停超时结果。
- `data/s5-{ext4,ntfs,windows}/`：三卷 S5 报告，保留功能通过及运行耗时失败的分别判定。
- `data/period-20-{A,B}/`：共同周期研究暂停前的两份完整报告，均未通过。
- `checksums.json`：资料、图片和工具的 SHA-256 及字节数清单。

`source.json` 记录原始绝对路径、数据库 SHA-256、运行源码逐文件 SHA-256 和复制文件 SHA-256。
SQLite 原始数据库保留于采集主机，未加入 Git。本资料足以重画反馈曲线和汇总图；
重新计算全部几何间距、关节余量或重新执行仿真仍需原始数据库和运行环境。
报告中的历史绝对路径是来源记录，不能作为另一台设备的现有路径使用。

## 字段与统计定义

`feedback-full.csv.gz` 使用 gzip，可直接用 Python `gzip`、pandas 或常见解压工具读取。
第一行列名：`simulation_s` 为仿真时间，`path_distance_m` 为实际 TCP 到路径的距离，
`reference_position_error_m` 为实际 TCP 与当前参考位置的误差，
`tool_axis_angle_rad` 为实际工具轴与参考工具轴的真实夹角，
`tcp_workpiece_distance_m` 为 TCP 到工件的距离；两个 `progress_m` 字段分别记录参考进度与实际投影进度，`phase` 为控制阶段。
长度单位 m，角度单位 rad，时间单位 s。

全部反馈以 500 Hz 仿真采样；摘要采用 `feedback_report.json/metrics` 的全部有效时间采样统计。
`feedback-25hz.csv` 每二十条选择一条并保留终点，仅用于修改曲线，不能替代完整样本重算最大值或 P95。
`arc_statistics` 采用沿路径位置的另一种统计方式，引用时应注明，不能混用两种数值。
`metrics.csv` 中长度误差已转换为 mm，角度转换为 degree，周期转换为 ms。
`runtime_verdict` 原样保留来源报告；同时必须检查 `timing_boundary`。
`legacy_incomplete_cycle` 表示历史计时范围，不能用于当前完整周期资格判定。

## 验证和绘图

另一台设备仅需 Python、NumPy、Pillow 即可验证资料完整性：

```bash
python scripts/verify_proposal_evidence.py
```

脚本检查清单 SHA-256、图片编码、十份报告、完整反馈数量与顺序，
并从全部反馈重新计算四项指标的 RMS、P95、最大值和最小值。
采集主机追加 `--originals`，可同时检查十份原始记录的文件和数据库身份。

修改 `data/metrics.csv` 后重新绘制三张汇总图：

```bash
python scripts/plot_proposal_summary.py
```

依赖为 Matplotlib 与 Pillow。需要安装实际字体 `SimSun` 和 `Times New Roman`；
脚本检查字体名称及字符覆盖，图片标题不含日期。
SVG 保留文本，可在安装相同字体的绘图软件中修改；PDF 可用于论文排版。
修改资料后原校验清单将失效，请将修改版与原始证据分别保存。

采集主机的完整导出入口是 `scripts/export_proposal_evidence.py`，每个运行输出目录必须尚未存在。
本次使用独立绘图依赖目录 `.scratch/proposal-venv/`，原始运行源码固定为
`504a274740a1e0c27c597c8d61a889fe39154aed`（S2 和 A/C 的历史身份见结论文档）。
实际导出命令环境为：

```bash
source .scratch/host-recovery/env.sh
export PYTHONPATH="$PWD/.scratch/proposal-venv/lib/python3.10/site-packages:$VIRTUAL_ENV/lib/python3.10/site-packages:$PWD/src:$PWD/portable_oscbf:$PYTHONPATH"
python -S scripts/export_proposal_evidence.py s0 --series
```

`s2`、`s4` 同样使用 `--series`；其余运行标识只导出报告。
三组曲线在导出时重新执行独立评价并核对已有报告，随后才生成图片。
该操作只读取已有实验，不启动采集。跨设备重新进行周期验证时仍需重新测量设备性能。
