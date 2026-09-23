## Conventions Detected

- **命名**：模块小写下划线命名（`oscbf_controller.py`、`transition_planning_server.py`）；
  类用 PascalCase（`JaxControlLoop`、`SCurveDriverSimulator`）；测试 `test_*.py`。
- **节点风格**：每个节点一个模块，`main()` 入口注册为 console_scripts；
  模块级 docstring 说明职责与话题约定。
- **共享约定**：话题名/QoS/关节名等统一在 `ros_conventions.py` 与 `robot_spec.py`，
  节点间禁止互相 import 私有符号。
- **轨迹加载与拟合**：`oscbf_trajectory.py` 的两个公开加载入口共享全量标定、
  圆柱投影和抽样流程；仅位置入口不要求时间字段。`CylinderFit.axis_point`
  统一拟合轴心坐标解释，查看器自行决定地面延伸等显示范围。
- **控制内核隔离**：`portable_oscbf/work/` 零 ROS 依赖、零裸名同级 import
  （统一 `from work.X`），节点经 `oscbf_trajectory.bootstrap_portable` 引导。
- **错误处理**：launch 文件对必需配置文件做启动时 `FileNotFoundError` 校验；
  内核侧有 QP 健康检查（`qp_solver_health.py`、`safety_snapshot.py`）；
  硬件合同网关可生成拒绝/保持结果并锁存原因；当前 bridge 无 CAN 发送能力，不能据此推断物理停车。
- **测试**：pytest（`testpaths = tests`），主包含 launch 集成测试
  （launch_testing）；`portable_oscbf/tests` 有独立 conftest；
  C++ 包的四个测试可执行文件已登记为 ctest。全量入口：`bash run_all_tests.sh`。
- **Git**：单 main 分支；提交信息中英混合、多为 feat:/fix:/perf: 前缀或中文摘要。
