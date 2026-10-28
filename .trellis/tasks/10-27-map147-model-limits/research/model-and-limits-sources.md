# 模型和共同限制来源

## 已确认需求

- 当前主仓库 `/home/lsn/robot/robot_safecontrol/docs/planning/nineaxis-full-project/decision-136-model.md` 定义九轴、TCP、轴向、标定和数值验收。
- 同目录 `decision-138-limits.md` 定义用户确认的逐轴数组、工具工艺、阶段、期限、来源和缺项。
- `decision-141-progress.md` 规定同一运动/参考时间和身份；M07 只检查输入，不维护执行进度。
- 当前 #153 正文保存在主仓库 `.scratch/map147-decisions/issue-153.md`。

## 真实资源

原 URDF 为 `models/ninezzhou/urdf/ninezzhou.urdf`，SHA-256 为 `766ea5870c41f110d7d57b46e6b31153335f45358faf15e641d465b3b31587a3`。SRDF 为 `models/ninezzhou_moveit_config/config/ninezzhou.srdf`，SHA-256 为 `e685e444b0ded09a77833a5d9b8f1b38c1f1180b7cec83e042748fd20605b6cd`。碰撞网格 manifest 和生成链来源由共同模型配置继续核验。

`joint_limit_contract.py` 提供原始控制和起点余量。`oscbf_velocity_config.py` 使用位置 Jacobian 奇异值乘积，既有阈值为 0.005、激活值为 0.05；共同配置保存这两个来源值和指标。五维任务缩放与秩条件仍由 #155/#156 提供。

#136 要求每份标定记录保存适用模型身份。实现使用模型 revision、基准配置和源资产生成 `asset_model_hash`，各份变换和标定容器保存 `applicable_model_hash`，在容器、模型构建、加载和更新入口验证。完整模型 hash 包含该资产身份和标定内容，资产身份计算独立于标定。

## 独立计算

MuJoCo 直接读取相同 URDF。逐轴状态覆盖各关节内部和上下边界附近，使用 MuJoCo 的各 body 位姿及 `mj_jacBody`，与 JAX POE 比较。工具高阶运动使用九点独立 MuJoCo 位姿样本的七阶多项式拟合；已知关节时间多项式包含非零速度、加速度和 jerk，覆盖 TCP 刚体修正、单轴和联合运动。

工具进给导数包含有效路径切向的时间变化。完整工具运动的关节高阶项通过共同模型方向导数取得。明确逐轴 jerk 的生成和停止分别与独立 `ruckig.InputParameter` / `Ruckig.calculate` 比较，完整关节区间检查复用真实分段多项式范围。

## 证据范围

M07 样本检查、完整关节运动/停止检查、碰撞证明和真实设备准入分别具有明确范围。最低正加工进给、工作距离工艺容差、设备停车和正式时序保持取得方式。接口与来源确认已由主会话处理，本任务继续完成本地实现和验证。
