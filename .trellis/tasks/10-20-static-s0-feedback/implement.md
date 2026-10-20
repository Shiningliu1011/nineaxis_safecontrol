# 实施与验证

## 执行顺序

1. 使用 `grill-with-docs` 核查已确认规格，记录需求与设计。
2. 使用 `trellis-before-dev` 读取涉及层的规则，按 `tdd` 验证公开配置与重放接口。
3. 完成固定 S0 配置和共享场景、真实 ROS 运行入口及持续反馈记录。
4. 完成反馈 FK、路径有序投影、真实角度、作业距离和完成评价。
5. 完成 MuJoCo 显示与数据重放，更新文档和安装资源。
6. 实际运行完整 S0 闭环，重算报告并检查身份、数据及异常结果。
7. 使用 `trellis-check` 和 `code-review` 检查需求、规范与回归。

## 测试边界

- 真实配置文件与 MAT 路径加载，禁止 mock。
- 使用独立解析例子检查投影、真实角度与阈值，不由被测函数生成预期值。
- 使用真实 ROS controller、真实 Ruckig plant 与实际反馈完成集成验收。
- 中间文件与运行输出均位于仓库内忽略目录，TMPDIR 与 ROS_LOG_DIR 显式设置。

## 命令记录

- 配置接口：`tests/test_static_experiment.py` 初次因接口缺失而失败，完成配置后 6 项通过。
- 记录接口：公开记录接口初次导入失败，完成持续 SQLite 记录后纳入相关测试。
- 当前 S0 测试：使用 `source install/setup.bash`，设置仓库内 TMPDIR 和 ROS_LOG_DIR，
  执行 `python3 -m pytest tests/test_static_experiment.py -q --basetemp=.scratch/static-experiments/pytest-s0-green`，
  9 项通过，12.63 s。
- 生产配置与跟踪回归：联合运行中相关 46 项通过。S0 投影接口已经由上述 9 项检查覆盖。
- 完整闭环已完成：
  `python3 scripts/static_obstacle_experiment.py run --output output/static-experiments/s0-validation-02 --domain 198`。
  133,913 条反馈，仿真时间 267.824 s，完整有序覆盖 1.816759744 m。
  路径 RMS/P95/最大值为 0.110915/0.198825/1.030302 mm；
  工具轴角度 RMS 为 0.103047°，任务精度评价为 fail。
  终点连续满足条件 11.45 s；碰撞测量为证据不足。
- `replay --output output/static-experiments/s0-validation-02` 重算后，
  JSON 与保存的首次报告逐字节相同；命令返回 1，与精度失败一致。
- 更新后的 S0 9 项测试通过，14.85 s。
- 跨组件完整回归日志为 `output/static-experiments/s0-regression.log`。
  主测试 572 项通过、1 项跳过、1 项失败；实际 ROS 过渡测试中的单点轨迹
  已增加持续回放处理，相关 13 项测试通过；真实集成用例已再次运行通过，
  与实验配置及 headless 场景合计 13 项通过，43.49 s。
  控制内核测试遇到内存耗尽并以 137 退出，后续逐文件使用独立进程执行。
  AEB C++ 的 4 项测试通过。完整回归目前未通过。

## 已读取规范

backend：目录职责、配置、质量、安全、ROS、错误、日志、控制内核依赖。
guides：公共入口、跨组件数据流。

## 审查范围

记录格式与 ROS 适配属于跨组件改动；完成后核查生产回归、反馈来源、时间、
参考采样及缺失证据处理。基准为 `34a2e34`。
