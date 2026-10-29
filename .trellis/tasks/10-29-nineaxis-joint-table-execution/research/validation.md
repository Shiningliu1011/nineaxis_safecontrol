# 实验室笔记本验证

## 单组调试补充与当前阻塞

本轮变更基点为 1f78ce9。首组调用前的相关组合运行 `tests/test_joint_table_commissioning.py`、`test_joint_table_execution.py`、`test_single_joint_execution.py`、`test_j1_execution.py`、`test_motion_log_replay.py`、`test_joint_table.py` 和 `test_motor_connection.py`，192 passed in 19.11s，退出码 0；日志和 JUnit 为 `.scratch/nineaxis-execution/commissioning-final-tests.log`、`commissioning-final-tests.xml`。其中实际位置来自 MuJoCo 的持续小幅摆动仍触发停车超时。

用户要求直接真机调试后不再追加仿真。完整真实动作和后续阻塞的最终纯数据回放：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src:portable_oscbf:.scratch/motor_tools/deps:.scratch/nineaxis-execution/deps:$PYTHONPATH \
TMPDIR="$PWD/.scratch/nineaxis-execution/work" python3 -m pytest \
  tests/test_joint_table_commissioning.py -q -p no:cacheprovider \
  --basetemp=.scratch/nineaxis-execution/real-data-regression \
  --junitxml=.scratch/nineaxis-execution/real-data-regression.xml
```

15 passed in 1.87s，退出码 0。包括完整真实静止记录、0.75 参数与实际目标转换、无效请求拒绝、单次请求、锁定原始字节保留、同一 J2 超调重现和拒绝反馈报告、已确认重力阻塞无法恢复。该命令不访问设备、不运行 MuJoCo。

实际设备调用为 `python3 -m robot_safecontrol_moveit.joint_table_commissioning`，使用 `.scratch/nineaxis-execution/requested-target.csv`、`first-pose-075-request.json`、本机适配器及输出 `first-pose-075-execution.jsonl`。退出码 1，原因为 J2:motion_outside_requested_interval。九条目标位置命令、九条保持命令及6484条属性读取已逐帧保存；请求已消耗，旧锁定归档。保持观察结果 external_stop_required，用户操作急停后恢复；全零请求未执行。

急停恢复后的 `joint_table_readonly --scope position --duration 1` 退出码 0，仅有属性读取。J5/J6 位置改变，用户确认失能后的重力转动，且没有支撑或制动。当前锁定具有 hardware_blocker，普通执行、单组调试及 acknowledgement 恢复均拒绝。全部原始字节和用户事实分别位于 `tests/data/first_pose_commissioning_20261029.zip`、`joint_motion_recovery_20261029.zip`，校验值见主题证据文档。

首组未完成，当前没有九轴快速到位、物理加速度、重力停止、断电保持或传感器标定验收。用户已经提供首组和回零的具体授权，当前阻塞为实际停止条件。

最终静态与交付检查：新增调试模块、执行/运行模块和两份测试的完整 Ruff 通过，六个既有文件的 E9/F63/F7/F82 检查通过；九个修改运行模块的 mypy（follow-imports=silent、ignore-missing-imports）通过。`python3 setup.py build_py --build-lib .scratch/nineaxis-execution/commissioning-built-lib` 退出码 0，九个修改模块的构建路径及源码 SHA-256 一致，没有加载 rclpy。MarkdownIt 检查 14 份文档、84 个本地链接目标均存在；不包含锚点验收。任务 JSON 和 git diff --check 通过。没有启动新的仿真或设备进程。

基点 a95831bd6a452364e224eb4bcbbc02b1c8d3ec43；日期 2026-10-29，设备 lsn，分支 main，初始工作区干净。Python 3.10，ROS 2 Humble。使用本机 NumPy 1.26.4、已有 MuJoCo 与 CPU JAX；补充项目要求的 pymoo==0.6.2 至仓库内 `.scratch/nineaxis-execution/deps`，保持 NumPy 1.26.4 和 wrapt 1.17.3。所有测试临时文件使用仓库内路径。

## 受影响行为检查

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src:portable_oscbf:.scratch/motor_tools/deps:.scratch/nineaxis-execution/deps:$PYTHONPATH \
TMPDIR="$PWD/.scratch/nineaxis-execution/work" python3 -m pytest \
  tests/test_joint_table_execution.py tests/test_joint_position_evidence.py \
  tests/test_motion_log_replay.py tests/test_joint_table.py \
  tests/test_j1_execution.py tests/test_single_joint_execution.py tests/test_motor_connection.py \
  tests/test_drempower_joint_control.py tests/test_unit_conversion.py tests/test_drempower_can.py \
  tests/test_hardware_contract.py tests/test_hardware_bridge.py tests/test_launch_structure.py \
  tests/test_j1_model_coordinates.py tests/test_j1_model_mujoco.py \
  -q -p no:cacheprovider --basetemp=.scratch/nineaxis-execution/delivery-tests \
  --junitxml=.scratch/nineaxis-execution/delivery.xml
```

结果：314 passed in 26.01s，退出码 0。完整输出为 `.scratch/nineaxis-execution/delivery-pytest.log`，JUnit 为 `delivery.xml`。执行脚本为同目录 `run_final_checks.sh`，仅运行本地测试，不访问串口。

其中完整设备日志回放 12 项，逐条验证 13215 条反馈及对应原始帧，独立重算速度窗口；九轴流程 28 项，覆盖两个标定种类的 MuJoCo 动态反馈、共同停车与测量推进、失败拒绝、运动与测量期限、来源身份、重复文件、文件后台准备后的再次检查、每组报告及非法时间保存。现场原始证据 5 项，核对非运动回读和授权多圈启用的实际原始帧；不重复发送设备命令。相关既有执行、模型坐标、ROS live 与协议保护包含在 314 项中。

MuJoCo 使用重力关闭、接触关闭、明确的 kp=60/kv=10 位置伺服和 implicitfast 积分，反馈读取实际 qpos/qvel。两个测量种类的结果均标记 simulation_feedback。该检查没有提供真实负载、碰撞环境或设备动作证据。

## 静态检查

- 新增五个运行模块与三个测试文件完整 Ruff 检查通过。
- 四个已有修改文件的 Ruff E9/F63/F7/F82 检查通过。
- 全部九个修改的运行模块 mypy 检查通过（follow-imports=silent、ignore-missing-imports，仅检查指定模块）；第三方库内部未作类型验收。静态检查脚本为 `.scratch/nineaxis-execution/run_static_checks.sh`。
- `setup.py build_py --build-lib .scratch/nineaxis-execution/built-lib` 通过，退出码 0；新增主题文档加入安装资源。独立构建目录导入九个修改模块，逐文件确认来源和源码 SHA-256 一致，没有加载 rclpy。
- 仅使用 `PYTHONPATH=src` 的 CLI help 与离线导入检查通过，无需加载硬件 extras。
- `git diff --check` 通过。
- Markdown 检查实际解析 14 份相关文档，81 个本地链接目标均存在；没有将此项描述为页面锚点验证。

## 设备检查与限制

身份范围非运动读取在 J1 序列号高位空回复后立即中止，退出码 1；报告、完整收发和 SHA-256 见 diagnosis.md 及 docs/joint_motion_limits_evidence.md。三条发送均为属性读取，运动命令数为零，未自动重试。

按用户后续要求，以 `joint_table_readonly --scope position` 独立核对厂家位置模式参数，退出码 0。取得九轴 981 条反馈和速度、加速/减速及轨迹模式参数，全部 1071 条发送为属性读取。实际原始报告与逐轴速度窗口检查保存于 `.scratch/nineaxis-execution/readonly-position-settings.json`、`readonly-position-audit.json`。J1 恢复读取在保存圈数属性空回复后中止，已取得版本、多圈开关 0、限位和零位计数。

用户随后提供距原端点 472 mm 的现场测量，按其近似零位定义推算 J1 约 28 mm；完整首组目标检查导出成功。此项测量的零位基准、精度及与当前设备角度的对应尚未验收。

用户明确授权开启 J1 多圈并给出型号 B06-PA-36-G。独立电池/圈数读取退出码 0；参数操作向节点 1 写入开关 31207=1 并发送保存，回读开关 1，圈数读取为 −1594。该操作报告在圈数与启用前不同处中止，保留已发送记录，没有自动重发；随后独立非运动复查退出码 0，确认开关 1、错误 0、零位计数与限位保持原值，取得两秒快速反馈。设备模式属性 34101 无回复，原始报告保留。逐帧归档检查确认所有额外写入仅有多圈开关和保存命令各一次，运动、零位、清错和重启命令均为零。

本次原始数据归档：`joint_motion_logs_20261028.zip`、`joint_position_readonly_20261029.zip`、`j1_multi_circle_20261029.zip`，均位于 `tests/data/`；文件身份和证据范围见主题文档与各归档的 provenance.json。最后一份同时保存用户位置参照。

运动锁定文件原字节保留，SHA-256 为 e7fb5fb03a5ee76671db0973c7d4ca66ecc230b36300740756656f1fb78440c2，ROS live 与九轴执行入口继续关闭。本次没有运动、回零、限位设置、零位写入或清错。九轴真机位置运动、快速到位、物理加速度、独立停车、J1 设备坐标对应、零点初始化与断电保持尚未验收。既有完整项目性能及独立规划原型问题沿用交接证据，本次没有执行全项目性能或完整原型检查。

## 后续已授权的 J1 后退检查

代码基点为 6a92d0a，用户授权仅 J1 后退约 28 mm，到位后由用户设置零点。运动前非运动读取退出码 0，142 条反馈及全部参数/原始帧保存在 `.scratch/j1-motion/preflight_20261029T101049.json`。审查脚本 `review_j1_retreat.py` 退出码 0，结果为 `rejected_before_motion`：目标超出现有设备限位，连续停车仅 0.168280681 s。没有自动重试，运动、参数和零位命令均为零。

三份原始数据保存在 `tests/data/j1_retreat28_preflight_20261029.zip`，SHA-256 为 2fcf8d10a2a60e7767ce4e7971deee4eab297a29789675aa912ad12df0618618。针对本次证据运行：

```bash
PYTHONPATH=src:.scratch/motor_tools/deps TMPDIR="$PWD/.scratch/nineaxis-execution/work" \
python3 -m pytest tests/test_joint_position_evidence.py -q -p no:cacheprovider \
  --basetemp=.scratch/nineaxis-execution/retreat28-tests \
  --junitxml=.scratch/nineaxis-execution/retreat28.xml
```

结果为 6 passed in 0.06s，退出码 0；Ruff、diff 和运动锁定 SHA-256 检查通过。此次仅补充证据、测试和文档，没有修改运行模块，前述 314 项为其原始运行结果。

## 用户自行归零后的当前记录

用户自行完成 J1 新行程 0 位与上位机零点设置，旧后退请求结束。运行 `.scratch/nineaxis-execution/read_j1_after_operator_zero.py`，退出码 0，得到 95 条反馈与 117 条只读请求，当前位置与用户确认的 0 mm 相符。多圈开关 1、圈数 0、零位计数 7733、各项故障 0；设备限位 −36000°–+1000°，项目目标范围保持 0–500 mm。

原始报告为 `j1-operator-zero-20261029T102356.json`，SHA-256 为 acb5f86db0aecfab4b46c97aa9ebff87cfea670f5e1055904b0dadb866090d5d；归档 `tests/data/j1_operator_zero_20261029.zip` 的 SHA-256 为 71f56b6d764c0986a7e7b3b341f7eb9100963de10362a7af9b03065f8c5fab75。数据归档时逐帧确认发送均为节点 1 的属性读取，未修改原始内容；运动锁定 SHA-256 保持。

按用户要求不追加行为测试。此次仅更新证据与状态文档，没有修改运行模块；已运行的 314 项组合与后退证据六项分别保留原命令和结果。当前记录没有提供断电保持、完整行程、物理停车或九轴动作验收。
