# 本地验证记录

## 基点与环境

独立 main 工作目录基点为 5952542；代码与原始模型参考为 4c98a64。课题室主机无法连接机械臂，全部验证保持运动锁定及 ROS live 关闭。原工作目录 /home/lsn/robot/robot_safecontrol 的未提交修改保留。

最终环境使用 .scratch/j1-motion-recovery/venv/bin/python，Python 3.10.12、JAX 0.6.2、numpy 1.26.4。该 venv 不加载系统 site-packages，通过任务 venv 的 j1_motion_dependencies.pth 明确复用原工作目录 venv 和任务依赖目录中的已有依赖；pure import 检查清除 PYTHONPATH 后仍能加载这些依赖。Matplotlib 与 mpl_toolkits 同为 3.10.9；SimSun 与 Times New Roman 的字体身份及字符覆盖已经检查。pip check 通过。ROS 环境来自 /opt/ros/humble 与当前 install；ROS 父子进程统一设置 ROS_LOCALHOST_ONLY=1，仅使用本机通信。construct、pyserial、rtree 和检查工具位于任务 .scratch 依赖目录，pymoveit2 使用本机已有源码，robot_static_fcl 使用本机实际构建的扩展。

中间产物均在 .scratch/j1-motion-recovery；最终命令环境保存在 env-isolated.sh，早期环境为 env.sh。原 venv 与系统安装未修改。早期进程在主机重启时中断的输出不计入完整通过结果。

## 已完成检查

- 修改前六组单关节、协议及单位测试：168 passed，baseline.xml。
- 当前执行、协议、单位、姿态表和模型坐标定向组合：213 passed in 12.63s，targeted-final.xml；其中六组执行相关测试共 180 项，姿态表 21 项，模型坐标 12 项。
- JAX、NumPy、MuJoCo、MoveIt、plant、显示几何及生成碰撞制品组合：32 passed in 40.63s，model-consumers.xml。
- 解析椭球及动态支持：141 项通过，另一个多槽解析场景在当前 Link1 局部坐标下通过定向检查。独立数值参考与容差保持原值。
- 旧5952542路径基点两项对照：2 passed in 61.48s，baseline-comparison.xml。当前固定物理路径基准及历史投影检查通过，六秒性能门槛保持原值。
- 物理轨迹、静态控制、S1 历史来源与共同变换组合：35 passed in 115.94s，physical-final.xml。S1 manifest 保持原 SHA-256，四份旧资源保存基点原始字节，当前资源拒绝旧模型身份。
- 早期有效起点与50 Hz性能检查曾有2 passed in 37.41s，current-start-perf.xml。实际301步，p95=10.485 ms，超过20 ms为0步，QP失败为0。最终环境性能检查未通过，完整对照见下文，不能沿用本次单次结果作为当前性能验收。
- 当前有效终点、真实 FCL 自碰撞及跟踪暂停检查：12 passed in 53.14s，consumer-terminal-final.xml。原 S0 终点记录保持原始字节；当前模型使用单独标识的仿真初态，核对原记录、模型和路径 SHA-256、FK、轴向、关节余量、实际 QP 与独立 SLSQP/KKT。两个暂停事件通过 caplog 检查实际 logging 输出，暂停行为断言保持原要求。
- 绘图消费者：25 passed in 8.93s，plot-isolated-final.xml。
- 真实 MoveIt 规划与 ROS 忙碌检查：1 passed in 13.72s，transition-localhost.xml；独立场景进程检查：1 passed in 3.03s，scene-localhost-final.xml。两项均在隔离环境、统一本机 ROS 通信下运行。
- 最新姿态表元数据及完整流程：21 passed in 3.99s，table-delivery.xml，包含节点映射 SHA-256。
- 默认pytest插件下，完整static_controller_warmup与tracking_run两个文件19 passed in 90.86s，logging-default-plugins-final.xml。直接捕获真实JAX与TrackingRun logger，保持全部编译、QP、独立数值和停止行为断言，并在finally移除临时handler。
- Python独立规划原型的距离、J1范围及近似碰撞检查11项通过，aeb-python-model-final.log；完整原型的原生调用缺口见下文。该检查使用独立ompl-venv，numpy 1.26.4、OMPL Python 2.0.1，不改变主包环境。
- generate_kinematics_data.py --check、generate_closed_collision_meshes.py --check、generate_obb_calibration.py --check、generate_collision_geometry.py --verify 均退出 0。十份封闭 mesh 的 SHA-256 保持来源值。
- bash build_aeb_moveit.sh 构建两包通过；C++ ctest 四项全部通过，包含 J1 三组位置检查。
- 最新独立非 symlink colcon安装通过，install-delivery-ready.log。安装副本joint_table入口从package share读取示例、模型和来源资源，camera与laser_tracker分别导出两组目标及各组九条绝对位置载荷，核对节点、零位、传动SHA-256；安装版物理轨迹与基点变换一致，当前终点数据及实际robot_static_fcl扩展均从安装副本加载。检查记录为installed-ready.log，未访问设备。
- 四个执行与姿态表源文件 mypy 通过，mypy-final.log；全部受影响 Python 的 Ruff E9/F63/F7/F82、新文件的 F401/F841 及 git diff --check 通过。
- 最终隔离环境 agent_check.sh 通过，纯逻辑安全组合 69 passed in 1.49s；实际输出为 agent-final-isolated.log。

实际定向命令：

```bash
source .scratch/j1-motion-recovery/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_joint_table.py tests/test_j1_model_coordinates.py \
  tests/test_j1_execution.py tests/test_single_joint_execution.py \
  tests/test_motor_connection.py tests/test_drempower_joint_control.py \
  tests/test_unit_conversion.py tests/test_drempower_can.py \
  --basetemp=.scratch/j1-motion-recovery/targeted-final \
  --junitxml=.scratch/j1-motion-recovery/targeted-final.xml
```

## 整体回归与缺口

早期完整回归记录为 full-final-current.log，实际产物目录为 .scratch/regression-YRwIE7zp。portable_oscbf/tests全部文件执行完毕，退出码0；C++ ctest四项通过，退出码0。该次主包结果为17 failed、1041 passed、6 skipped、15 errors，包含系统Matplotlib混合版本、ROS父子进程通信设置及模型测试消费者问题；整体脚本退出码1。控制内核及C++源码在完整回归期间保持一致。

最终环境完整主包记录为main-delivery-full.log与同名JUnit XML：4 failed、1069 passed、6 skipped in 1550.09s，退出码1。真实ROS/MoveIt规划、回放、完整路径与报告流程通过。三项日志断言的实际事件已写入stderr，caplog.text为空；ROS launch的LaunchLogger默认propagate=False，使根logger捕获无法收到事件。测试直接绑定真实logger后，两个完整文件19项通过。该定向复核不能重标完整主包为全部通过，性能失败继续保留。

最终环境的首次主包检查使用遇到失败停止选项，实际491项通过后在性能检查停止：300步，p95=25.278 ms，129步超过20 ms，QP失败为0；报告 main-isolated-final.xml。完整 smoke 模块为12 passed、1 failed，p95=26.115 ms，221步超过20 ms。独立运行及关闭插件自动加载均复现同类失败，不能将原因归于测试顺序或插件。

同一主机、同一最终环境下，未修改的5952542独立性能检查同样失败：300步，p95=26.029 ms，215步超过20 ms，QP失败为0；报告 perf-baseline-control.xml。该对照使用基点非 symlink 安装的模型、路径及源文件。当前有效配置、模型、路径与先前10.485 ms运行一致，确切性能差异原因尚未确认。保留20 ms、1%与300步要求，未修改生产控制器或性能断言，不能宣称整体性能通过。

最终完整主包的性能结果为301步、p95=27.862 ms、249步超过20 ms（82.72%）、QP失败为0。该结果与基点对照分别保存，不以早期单次通过替代当前失败。

Python OMPL 2.0.1完整原型在TestPlannerBasic.test_clear_then_solve的原生调用中终止：当前版本退出码134，未修改5952542退出码139，报告为aeb-python-final.log与aeb-python-baseline.log。具体内存原因未确认，原型完整规划不计为通过。模型相关11项和三组J1 FK检查已通过；生产C++插件的四项ctest及真实MoveIt流程独立通过。未修改原型内存管理、规划断言或生产参数。

J1 真实保持回放仍要求 external_stop_required。没有 J2/J5/J7 的完整10月28日收发记录、J4 通信中断记录、J1 断电恢复及独立位移、速度和停车测量。多轴真机动作、快速定位能力和物理标定验收均未验证。
