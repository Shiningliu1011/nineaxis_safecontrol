# 实验室笔记本验证

## 确认后回零验证

用户已确认 r 加 Enter 才执行九轴全零目标，沿用各轴梯形参数；回零后持续使能及监测，q 正常退出。Enter 继续表格、q 直接退出保持当前位置，运动期间的 r 不预约回零。执行器使用独立回零尝试，表格来源、索引和完成组数保持准确。当前停止锁定不解除，本轮不连接设备。

在 Python 3.10.12、ROS 2 Humble、已有 MuJoCo、CPU JAX 和设备依赖环境中运行：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PWD/.scratch/motor_tools/deps:$PWD/.scratch/nineaxis-execution/deps${PYTHONPATH:+:$PYTHONPATH}" \
TMPDIR="$PWD/.scratch/joint-positioning/work" python3 -m pytest \
  tests/test_joint_positioning.py tests/test_joint_table_execution.py \
  tests/test_joint_table_commissioning.py tests/test_joint_table.py \
  tests/test_single_joint_execution.py tests/test_j1_execution.py \
  tests/test_motor_connection.py tests/test_drempower_can.py \
  tests/test_drempower_joint_control.py tests/test_motion_log_replay.py \
  tests/test_hardware_contract.py tests/test_hardware_bridge.py \
  tests/test_launch_structure.py tests/test_unit_conversion.py \
  -q -p no:cacheprovider --basetemp=.scratch/joint-positioning/return-zero-final-tests \
  --junitxml=.scratch/joint-positioning/return-zero-final-tests.xml
```

结果 `334 passed in 28.73s`，退出码 0。首轮三个直接受影响测试文件为 70 passed in 20.24s。新增行为使用真实管道、CSV 与 MuJoCo qpos/qvel，验证从非零姿态确认回零、持续零位监测、提示前和运动期间 r 无效、回零不继续表格、不增加完成组数、q 退出、真实位置变化及反馈中断拒绝；非手动会话和运动期间拒绝直接调用回零。没有 mock 或设备成功结果替代。

三个行为修改模块的 mypy 通过（follow-imports=silent、ignore-missing-imports），五个文件完整 Ruff 通过。`setup.py build_py --build-lib .scratch/joint-positioning/built-lib install_data --install-dir .scratch/joint-positioning/installed` 退出码 0；六个运行模块、安装参数脚本和主题文档与源码内容一致。独立安装进程的单组及两组 CSV 预览均包含完整回零目标与相同有效运动参数，未取得资格的执行在创建日志目录和传输对象前拒绝。六份文档的 50 个本地链接目标存在，锚点未验收。`git diff --check` 通过。

锁定 SHA-256 保持 `8d983e8bf1905ebdd87a1698dca072cfb8963a971c0a1c0b6d188ad144e82554`，没有真实连接、运动、参数或零位写入。真实反馈初始化、独立停止与回零路径尚未取得验收证据。上一阶段已授权的真实读取和启动拒绝单独记录于 diagnosis.md，不计为本轮回零动作验收。

## 手动定位入口验证

当前增量基点为 `15a03557a973bd3c5074e0da1b7e3622cbda4bce`，分支 `main`，开始时工作区干净。用户确认文件顶部九轴绝对目标、CSV、Enter 逐组推进、梯形运动、到位后持续使能与监测，并说明失能重力转动无法解决。手动入口的本地实现阶段只完成代码与验证；后续新的设备动作请求及启动拒绝单独记录于 diagnosis.md。

环境为 Python 3.10.12、ROS 2 Humble、已有 CPU JAX 和 MuJoCo；设备相关依赖使用仓库已有的 `.scratch/motor_tools/deps` 与 `.scratch/nineaxis-execution/deps`。测试临时目录、构建和依赖检查均位于 `.scratch/joint-positioning/`。

最终行为命令：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PWD/.scratch/motor_tools/deps:$PWD/.scratch/nineaxis-execution/deps${PYTHONPATH:+:$PYTHONPATH}" \
TMPDIR="$PWD/.scratch/joint-positioning/work" python3 -m pytest \
  tests/test_joint_positioning.py tests/test_joint_table_execution.py \
  tests/test_joint_table_commissioning.py tests/test_joint_table.py \
  tests/test_single_joint_execution.py tests/test_j1_execution.py \
  tests/test_motor_connection.py tests/test_drempower_can.py \
  tests/test_drempower_joint_control.py tests/test_motion_log_replay.py \
  tests/test_hardware_contract.py tests/test_hardware_bridge.py \
  tests/test_launch_structure.py tests/test_unit_conversion.py \
  -q -p no:cacheprovider --basetemp=.scratch/joint-positioning/final-tests \
  --junitxml=.scratch/joint-positioning/final-tests.xml
```

结果为 `326 passed in 24.46s`，退出码 0。新入口使用真实管道输入和既有 MuJoCo 动力学，检查 Enter 不提前预约、第一组手动启动、单组自动启动、两组运动、最后一组持续保持、部分输入不阻塞反馈、q 退出、未执行组计数、反馈中断及实际位置变化拒绝。实际设备日志按原始记录回放，未连接串口。定位记录与测量记录分开，测试没有 mock 或伪造设备成功结果。

静态检查：`joint_positioning.py`、执行器、runtime、参数脚本和新测试共五个文件的完整 Ruff 通过；`joint_table.py`、`motor_motion_limits.py`、`drempower_can.py`、`setup.py` 的 E9/F63/F7/F82 检查通过。七个运行文件的 mypy 通过，使用 `--follow-imports silent --ignore-missing-imports`，第三方库内部未作类型验收。`git diff --check` 通过。

构建命令为 `python3 setup.py build_py --build-lib .scratch/joint-positioning/built-lib install_data --install-dir .scratch/joint-positioning/installed`，退出码 0。`verify_delivery.py` 确认六个修改模块、参数脚本及主题文档与源码字节一致；在仅使用构建模块、安装资源、设备依赖和 ROS Python 路径的独立进程中，单组和 CSV 预览分别取得 1、2 组目标。未取得资格的安装入口在创建运行目录与传输对象前拒绝。结果保存在 `delivery-check.json`。

MarkdownIt 检查六份变更文档的 50 个本地链接目标存在，未验收锚点。当前运动锁定 SHA-256 为 `8d983e8bf1905ebdd87a1698dca072cfb8963a971c0a1c0b6d188ad144e82554`，没有清除或改写。ROS live、独立停止、断电保持及九轴真机到位验收继续保持原状态；本轮没有执行设备连接、位置动作、零位或设备参数写入。

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

## 有人值守手动定位与确认回零

本轮运行十五份相关测试，JUnit 为 `.scratch/joint-positioning/pytest-attended-final.xml`：345 passed in 32.40s，退出码 0。范围包含手动定位、有人值守范围、真实 J7 运动回放、真实静止记录、九轴与单轴执行、协议、硬件接口、launch 和单位转换。六个相关文件完整 Ruff 通过，五个运行文件限定 mypy 通过。构建与安装后核对九个源码文件、单组和 CSV 预览、缺少现场记录时在真实 I/O 前拒绝，退出码 0；六份文档的五十个本地链接目标存在。

实际第二次定位发送九条目标，J2–J9 到达目标，J1 在反馈约 35.54 mm 时发生编码器量程故障，任务中止。原始故障归档含七份文件，SHA-256 为 `757d5a88db24b57cbbc7cb782de65f3a571e6672f37e588d90dc663021812eda`；没有自动重试或回零。

用户随后要求回零。实际六秒静止记录回放通过后，执行 `.scratch/joint-positioning/return_healthy_joints_zero.py --execute`，退出码 0，八条位置命令仅对应 J2–J9。经过约 38.1 秒的运动与持续静止检查，八轴反馈确认回到原机械零位，退出后的六秒独立读取确认仍保持使能；J1 故障状态保留，未收到运动或参数命令。

用户自行恢复 J1 机械零位并重置零点后，执行只读脚本 `read_current_zero.py`，退出码 0，九轴均闭环使能，轴与编码器故障均为 0，位置反馈均接近零位。新 J1 零位计数 11145 已同步至现场记录；此次没有新的运动、清错或设备参数写入。50 mm 目标和完整行程尚未验收，父任务保持 in_progress。
