# 标定与证据设计

## 当前阶段

本文件记录来源规格已确定的要求，以及现场需要补充的输入。采集方法尚待现场条件确认，任务保持 `planning`。

## 坐标与记录

- `B_T_L` 把实际 LiDAR frame 下的点变换到 `base_link`；平移单位为米。
- `base_link` 固定，采用项目 Y-up 约定；九轴 FK 包含 J1。
- 每个样本保存原始点云、对应采集时间、靶标身份及参考位姿。使用机器人位姿时保留真实关节反馈、关节名称、单位和模型版本。
- 求解集和独立验收集在求解前分别登记；验收样本不参与参数拟合。

## 工具与现场条件

已有标定工具链研究验证过 OpenCV AX=YB 的矩阵方向适配，研究中的物理观测来自相机靶板；LiDAR 到相机的部分另需上游工具和真实特征。
当前票要求 `B_T_L`，因此正式采集前必须确认实际靶标能够提供哪些观测，以及这些观测如何联系到 `base_link`。
不能仅依据旧研究中的工具名称确定现场采集方法。

本次只读检索的 `scripts/`、`src/` 和 `tests/` 文件清单中，未找到可直接执行本票全部求解与独立验收的专用入口。
取得现场靶标、参考位姿和数据条件后，核实可复用工具及所需适配；需要代码修改时先更新本任务范围和验证步骤。

## 唯一真源与部署证据

- 记录目标：`config/sensor_extrinsics.yaml` 的 `lidar` 分节。
- 身份算法：`src/robot_safecontrol_moveit/calibration_record.py` 的 `compute_calibration_id`。
- 加载与字段检查：同一模块的 `load_calibration_record`。
- 运行证据：`perception_bridge` 的启动日志与 `/perception/calibration_status`。
- 独立验收报告保存毫米和角度测量结果；当前加载器中的 `CALIB_ACCEPTANCE_NOT_EVALUATED` 表明诊断字段本身不提供实测精度证明。

## 有效条件与失败处理

支架、传感器安装关系与被验证的记录共同确定本次标定的适用条件。安装关系变化后重新标定并独立验收。
缺少参考位姿、样本不完整、数据方向或单位不明、独立误差超出阈值时，停止生成正式记录并明确需要补充的数据。
正式写入前保存现有文件及其 SHA-256；部署前确认构建脚本和进程归属。需要恢复文件时使用文件编辑工具恢复已保存内容，并重新部署和核对身份。

## 依据

- `docs/adr/0004-calibration-ssot-sensor-extrinsics-authority.md`
- `docs/adr/0007-shared-rear-sensor-stand.md`
- `docs/adr/0009-calibration-record-schema-identity-admission.md`
- `docs/planning/oscbf-reuse/handoffs/25-calibration-toolchain.md`
- `docs/planning/oscbf-reuse/research/calibration-toolchain.md`
