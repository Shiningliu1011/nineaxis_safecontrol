# OFF-01 标定记录：审查修复交付

日期：2026-09-15。关联 [OFF-01 / #45](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/45)、[规格 #62](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/62)。
基线：`08cd8f3295661350b9383b4d08de9bc46e13d1c1`；本轮修复在此基线上交付，最终提交身份以 Git 历史为准。

## 目的与边界

完成三项已复现缺陷的代码修复、针对性回归及本地交付记录。完成标准是畸形内容不能
获得可信身份、冻结 ROS 时间不能停止诊断心跳、指定畸形输入仍以错误码和退出码 3 拒绝。
不修改标定数值、不开放硬件 I/O、不扩大为控制器准入实现；保留工作区其他已有改动。
依赖的原始契约见 [ADR 0009](../../../adr/0009-calibration-record-schema-identity-admission.md)。

## 已解决的决策

三项前置证据均为上一轮审查的可复现反例，用户已授权修复，无待定实现选择。

| 问题 | 证据与前置 | 结果 | 剩余边界 |
| --- | --- | --- | --- |
| 如何避免映射键规范化丢数据？ | residual 的数字键 `1` 与字符串键 `"1"` 被归并，改变前者仍得到相同 id | 递归拒绝非字符串映射键；返回 `CALIB_ID_UNAVAILABLE`，debug 豁免也不能放行；合法固定向量不变 | 标定工具须共用规范化函数；原先含数字映射键的记录需显式迁移，不能自动猜测含义 |
| 心跳应由哪个时钟驱动？ | `use_sim_time=true` 且无 `/clock` 时默认计时器停止 | 仅标定心跳使用 `STEADY_TIME`，保留 ROS 消息时间戳及融合计时器语义 | 消费端仍须用本机单调时钟判断新鲜度；此改动不是 OFF-06 时间域检测 |
| 畸形输入如何遵守拒绝契约？ | 非法日期抛 ValueError；`10**400` 转 float 抛 OverflowError | YAML 构造错误归为 `CALIB_RECORD_YAML_INVALID`；矩阵溢出为 `CALIB_MATRIX_NOT_FINITE`；身份转换溢出为 `CALIB_ID_UNAVAILABLE` | 不捕获所有 Exception，不掩盖程序错误；不承诺任意资源耗尽输入可恢复 |

## 实际验证

环境：Python 3.10.12、NumPy 1.26.4、PyYAML 5.4.1、ROS 2 Humble。
现有 symlink-install 的模块解析到本工作区源码；无新依赖或模型变更。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/off01-fix-ros-logs ROS_LOCALHOST_ONLY=1 python3 -m pytest tests/test_calibration_record.py tests/test_perception_bridge_points.py tests/test_perception_bridge_demo.py -q
ROS_LOG_DIR=/tmp/off01-fix-ros-logs ROS_LOCALHOST_ONLY=1 python3 -m pytest tests/test_perception_bridge_startup.py -q
git diff --check
```

- 加载器、点云、演示：85 passed（1.02 s），其中加载器 75 项，本轮新增 6 项。
- 真节点：17 passed（36.57 s），本轮新增 5 项。使用独立 ROS 域，不启动传感器驱动或硬件。
- 新增节点回归覆盖非法日期、矩阵/残差溢出、映射键碰撞的退出码 3、错误标记、无启动标记和无 traceback；冻结时间用例在收到首帧后继续观察 4.5 秒，断言收到 3–6 份心跳且 ROS 时间戳始终为零。
- 首次未指定日志目录的节点测试因默认 `~/.ros/log` 只读失败，已中断；指定可写日志目录后通过。沙箱存在 DDS UDP 限制，另行在允许 localhost 通信的权限下复核：17 passed（36.72 s）。
- `git diff --check` 通过。
- 未执行全仓测试、真实传感器或真机验收；相关测试通过不代表物理安全准入。

### 验证输入身份（SHA-256）

| 文件 | SHA-256 |
| --- | --- |
| `config/sensor_extrinsics.yaml` | `f891526a64a8fbfdffcbc465956bd70d50c4165bf5901f55816d1b225507da5a` |
| `config/perception_runtime.yaml` | `7807d5d7ac0815d4f0b8f2431bab57b0d6b02bd5d3d909cdbeec204f6b64b89f` |
| `config/perception_dual_sensor_real.yaml` | `34b701752a85378a568f26710eeb8284ac607ccec30c0d1b62a191c07cb7a82a` |
| `src/robot_safecontrol_moveit/calibration_record.py` | `aa3fe0551aa919c7dd7137e0e4515100405fd5546340103cbb8f456d9a655778` |
| `src/robot_safecontrol_moveit/perception_bridge.py` | `a7264c462a6aefc9f88c681eecd440f6bace74b86c80bd108bc92d9c71b951f6` |

## 发布与后续

修复完成时尚未提交或推送；随后用户已授权将本轮交付提交并推送至 `origin/main`。
本轮不评论或关闭工单。后续工单收尾负责人需核对远端最新状态，更新 #45 / #62
的旧测试数与“尚未推送”描述，并附实际提交身份；此前审查时两票仍 OPEN。
ADR/config 引用的两份既有研究记录随本次授权提交一并纳入交付，内容保留研究时点的证据与限制：

- [外参来源研究](../research/extrinsics-source-tf-vs-params-20260914.md)
- [传感器时间对齐研究](../research/sensor-time-alignment-20260914.md)

后续负责人/边界：OFF-16 承接标定工具身份写入；OFF-06 承接时间域判定；OFF-17 / T7
承接运行模式及消费端准入；T5/T6/H6.1 承接真实外参和现场验收。这些不因本轮测试通过而完成。

回退：发布前可逐项撤回本轮四个代码/测试文件及本交付文档、导航文档的差异，保留其他用户改动；
发布后应采用独立 revert 提交，不重写共享历史。回到基线会重新引入上述三项缺陷，不能据此放宽准入。
