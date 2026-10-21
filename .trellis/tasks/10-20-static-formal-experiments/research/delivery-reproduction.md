# #183 分析、图表与保存反馈播放

正式冻结、三卷容量绑定和串行采集步骤由 `formal-serial-continuation.md` 维护。
本记录说明采集后公开入口及输入要求；当前没有执行正式 1080 项分析或正式图片导出。

## 入口与输入

在主仓库恢复环境，并将以下变量设为真实且已关闭的正式目录、其中一项运行目录：

```bash
project=/home/lsn/robot/robot_safecontrol
cd "$project"
source .scratch/host-recovery/env.sh
export PYTHONPATH="$project/src:$project/portable_oscbf${PYTHONPATH:+:$PYTHONPATH}"
formal="$project/output/static-experiments/formal-01"
run="$formal/runs/<实际运行编号>"
python scripts/static_obstacle_experiment.py paired-report --output "$formal"
python scripts/static_obstacle_experiment.py report --output "$run" \
  --report-python "$project/.scratch/host-recovery/report-venv/bin/python" \
  --figures-output "$project/picture/static-obstacle-experiments/<独立图片目录>"
python scripts/static_obstacle_experiment.py playback --output "$run" \
  --playback-output "$project/output/static-experiments/<独立播放目录>" --domain 232
```

占位符必须替换为现有编号及尚不存在的输出目录。`report` 使用
`requirements-static-report.txt` 的版本；播放使用包含 ROS/MuJoCo 的恢复环境。
图像窗口由用户启动；无窗口检查可追加 `--headless`。播放速度由
`--time-scale` 指定时长倍数，默认 1.0。

每份原始记录至少保留 `manifest.json`、`result.json`、`path.npz`、
`plant.sqlite3`、`controller.sqlite3`、`feedback_report.json`，以及当前格式要求的
`controller.recording.json`。报告绑定原件 SHA，数据库 WAL 必须完成 checkpoint。
当前资源目录必须提供内容身份相同的模型、mesh、碰撞模型和轨迹；跨 worktree
通过既有资源重定位校验。播放保留原始反馈及 ROS 测量时间，只有显示流使用独立 topic。

`replay --output "$run" --save-samples` 在运行目录写入重算报告和逐反馈数组。
需要保留原报告时，使用公开 Python 函数
`evaluate_run(run, resource_root=project, save_samples=True, output_directory=new_directory)`，
将参数转换为 `pathlib.Path`，并保存执行脚本和输出身份。
绘图内部的 `evaluation_samples` 可按需重算，比较原报告与重算结果，并自动清理
本次派生中间文件；无需修改原始数据库。

## 交付检查

- 配对报告要求固定清单全部 1080 个编号；A/B、C/D、A/C、B/D 各计划 216 对。
  连续指标使用实际共同观测数量；失败、partial、未完成继续参与状态统计。
  `collection_complete` 表示存在记录，D 功能与全部方法运行时间需分别核对。
- 六类 PNG 为 `representative-configurations.png`、`errors.png`、
  `feed-and-phase.png`、`clearances.png`、`joint-margins.png`、`cycle-duration.png`。
  代表构型使用实际观测进度，并显示实际比例；未完成路径的后段不能据此认定已到达。
- 出图入口检查 Times New Roman/SimSun 实际字体名称及全部文本字符覆盖，
  标题和图内文本不含具体日期。Pillow 核查 PNG 完整性及最少 600×400 像素。
  `figures.json` 保存六份文件 SHA/大小、字体路径/SHA、源报告、派生数组、
  绘图源码和 Python 包版本。本次未执行图片导出或视觉检查。
- 周期图使用完整 sidecar 的完成时间边界；历史记录的较早边界保留标识，
  不能用于选择最终共同周期。正式报告必须引用最终冻结身份。
- 播放结果独立记录原件和播放源码身份、发送数量、模拟时间范围及 MuJoCo 末态。
  显示结果不生成新的实验成功记录，也不覆盖原有失败报告。

## 本次实现验证

验证输入为
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/e-s5/run`，
原采集源码 `40d43bb`。输入 18,397 条反馈，simulation_s 为 0.002–36.794 秒。
测试逐条比较原始 q、velocity、ROS 时间戳及顺序；实际 1.0 倍时间播放满足
首末接收间隔断言。固定提交 `adbfcc9` 的公共 CLI 以 0.1 倍时间播放，
耗时 3.679447 秒，计划 3.6792 秒，
MuJoCo 末态九轴满足绝对误差 `1e-12`，并确认末条原始时间戳已经应用。
SIGINT/SIGTERM 分别在真实 ROS 接收首条反馈后中断，保存 `interrupted`，
退出 130；独立 ROS 观察节点确认显示节点和 publisher 全部清理，生产状态和
控制命令 publisher 数量始终为零。入口仅构造播放节点与既有 viewer，未创建
controller、plant 或 hardware bridge。

TDD 分项共五项通过：加载/资源重定位 1 项、完整 ROS 接收 1 项、公共 CLI 完成
1 项、SIGINT/SIGTERM 2 项。全部使用 CPU16–21/domain232；每组 fixture 核查
原始 manifest、报告及两份数据库 SHA 前后一致。未执行 GUI 窗口或性能测量。
复验命令在本 worktree 执行，输出指定新的已忽略目录：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
export PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PYTHONPATH"
export STATIC_PLAYBACK_RUN='/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/e-s5/run'
taskset -c 16-21 python -m pytest tests/test_static_experiment_playback.py -q \
  --basetemp .scratch/playback-validation/pytest-review
```

原始身份：

| 文件 | SHA-256 |
| --- | --- |
| manifest.json | `0be9f544f6c9f94ce1d6a4998fbe55c2368bce28c647262e45255024ea43db0c` |
| feedback_report.json | `a5db8fcbab2df3bc8bd07e37b6e35a4fe2dab629e50eafa0a9e48c64b76f1d1a` |
| plant.sqlite3 | `2d7a4c1832745781c1071682c47426cf5cb9f2e361b42c46efa9f97a6baae082` |
| controller.sqlite3 | `4353c912034ab4ce8b9c78b25a525578dbbe21877939f79d73f19fdfb898cbe3` |

验证目录为
`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-evidence/.scratch/playback-validation/`。
`green-01.log`、`green-04.log` 保存加载与完整 ROS 接收结果；RED 和全部 TDD
日志保留同目录。`pytest-green-07/` 保存固定 `adbfcc9` 的完整公共 CLI 验证，
`pytest-green-06/` 保存两次中断。三份记录的播放模块 SHA 均为
`d9f02d0534f54fc7bcc0659e464d66d326612cd82f786da26a7dc3ce00f8bd2c`，
各自来源映射包含 125 个文件，涵盖新增的 `static_experiment_playback.py`。

| 固定源码证据 | SHA-256 |
| --- | --- |
| green-07.log（1 passed，18.99 秒） | `65ec58591eae539638f0fc53f94971053ed40bf185b865521ed161dc7198baad` |
| 完整 playback.json | `af3ffae7da6b8f0723f9cda68702cc1c877b8faab0b0baa7e65204ba7b431874` |
| 完整 playback_result.json | `79674b16755fb67c04b3348a54a0b5bfed33342bb5d46f238efb48402cba757a` |
| SIGINT playback_result.json | `45fd385ed4d2288b3945f46ab066055b76f25714bb3b3dc539c76c63916d4e78` |
| SIGTERM playback_result.json | `78566020ba7ff9cc59f39216bb0093674d58ae3425cdc58248078a9e8edd5f64` |
| fixed-source-evidence.json | `8804a95568c5e8e9305bddde15d09c73f35f4b23d234fcfa681e7e90aa89feaa` |

固定版本单项命令为上述 pytest 命令追加
`-k public_playback_cli_finishes --basetemp .scratch/playback-validation/pytest-green-07`。
所有原件保持原内容，开发期间的 `evidence.json` 及 `pytest-green-05/` 独立保留。
复验完成后普通合入 `2ce24d2`，本次更新仅修改研究说明；公开入口的最终修改
必须进入 study02 的新来源身份。
