# 生产配置一致性验收报告

日期：2026-09-11

范围：GitHub #37–#42（配置契约、来源合并、消费者锁定、启动快照、兼容边界与生产入口回归）

固定基线：`84797408ac0cbe344bcb9f06fc70da5ae29dd417`

## 结论

配置一致性范围通过。生产 YAML 现在必须先独立完成严格校验，之后才与显式启动覆盖合并；同一份不可变有效配置贯穿 JAX facade、路径跟踪参数与 ROS timer。运行快照成功落盘是 publisher/timer 创建前的硬门禁。

本结论不表示真机准入，也没有启用 SocketCAN、改变真机模式或接通反馈；这些仍属于 #13。

## 验收矩阵

| 验收项 | 状态 | 证据 |
|---|---|---|
| YAML 缺失、不可读、无法解析 | PASS | `test_missing_production_yaml_is_rejected_before_resource_construction`、`test_unreadable_production_yaml_reports_its_source`、`test_malformed_production_yaml_is_rejected` |
| 必填缺失、类型/有限性/范围非法 | PASS | `test_missing_required_yaml_value_cannot_be_repaired_by_override` 与参数化严格校验用例 |
| 未知及遗留生产字段拒绝；其他节点字段不误伤 | PASS | unknown、legacy、other-node 三组配置契约测试 |
| launch 与直接入口使用同一生产 YAML 契约 | PASS | launch 结构测试确认 YAML 保持独立来源；直接入口失败清理测试；下述两次真实启动 |
| 覆盖优先级、来源及完整覆盖链 | PASS | `test_effective_configuration_records_yaml_and_override_chain`；真实节点哨兵值测试 |
| 三种资源空值/单项/组合/root-only 回退 | PASS | 四组合参数化资源测试；root-only 不重定向空 config |
| 无效覆盖/资源在命令链建立前失败 | PASS | 无效覆盖、缺失资源测试及快照失败门禁测试 |
| 九轴身份、话题与实际圆柱拟合来源 | PASS | 严格 joint_names 校验、真实消费者诊断与快照 geometry 断言 |
| `dt`、`dt_path`、发布频率独立到达消费者 | PASS | 实际 JAX 节点以 0.012 / 0.013 / 80 Hz 哨兵启动并检查 facade、path step、timer |
| 运行时单项/原子批量修改拒绝且消费者不变 | PASS | 两个真实 ROS 参数服务锁定测试 |
| 唯一、原子、可追溯启动快照 | PASS | 唯一文件/无临时残留测试；快照与实际消费者、配置哈希、Git/dirty/source、资源及几何比对 |
| 快照失败阻止 publisher 和 timer | PASS | `/proc` 不可持久化门禁用例同时断言 0 command publisher、0 timer |
| 遗留资产与离线入口兼容 | PASS | `nineaxis.yaml` 同含 kinematics/controller 时仍合法；facade 独立默认签名保持；portable 全量回归通过 |
| 任务误差、路径进给、约束/QP 健康回归 | PASS | 真实 `step_once` 与完整闭环测试；portable tracking、path following、CBF/残差、QP health/baseline 套件 |
| 真机发送与反馈准入 | SKIP（范围外） | 未接通；由 #13 负责 |

## 真实生产入口

1. `bash build_aeb_moveit.sh`
   - PASS：2 packages finished。
   - 安装 share 中的 `config/oscbf_controller.yaml` 与源码逐字一致。
2. 安装环境执行 `ros2 run robot_safecontrol_moveit oscbf_controller`
   - PASS：加载安装资源、完成 JAX 预热、原子写入绝对路径快照并报告 `oscbf_controller ready`。
   - 验证窗口结束时用 SIGINT 停止，节点正常清理。
3. 安装环境执行正式 `mujoco_transition_final.launch.py`，关闭 viewer、plant、自动规划和感知，保持 `hardware_mode:=sim`
   - PASS：正式 launch 启动 `oscbf_controller`，完成 JAX 预热、写入另一份唯一快照并进入 ready。
   - 验证窗口结束时由 launch SIGINT 关闭；控制器进程 cleanly finished。该运行未产生真机后端准入声明。

## 自动化结果

执行 `bash run_all_tests.sh`：

- PASS：主包 `336 passed in 113.31s`。
- PASS：portable OSCBF `148 passed, 34 skipped in 447.35s`。
- PASS：主包退出码 0、portable 退出码 0、总入口退出码 0。
- SKIP：34 项为测试套件已有的可选/归档依赖条件（包括 `newaxis` 对照路径）；本验收使用的 ROS、JAX、cbfpy、qpax、MoveIt 依赖均可用。
- MISSING（非阻塞）：检测到 NVIDIA GPU，但环境没有 CUDA-enabled jaxlib，JAX 按设计回退 CPU；本轮生产链与数值回归均在 CPU 上通过。
- FAIL：0。

附加静态检查：`python3 -m compileall -q src tests portable_oscbf/work` 与 `git diff --check` 均通过。

## 快照抽检

真实启动快照记录了：run id、schema/node、最终值、逐项来源、覆盖链、资源原值/解析路径、生产 YAML SHA-256、Git HEAD/dirty/status、45 个控制源文件摘要、实际拟合圆柱中心/轴/半径、障碍 CBF 基准 alpha 10 及 perception 第十槽运行值来源。连续直接入口与 launch 入口生成不同文件，未覆盖既有证据。
