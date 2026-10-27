# 标定准备资料验证记录

日期：2026-10-26。任务状态：planning。

## 实际完成的检查

- 已读取引用会话中的可用文本及本地相关源码，确认设备与数据现状。
- URDF 生成数据检查通过，命令 `python3 portable_oscbf/scripts/generate_kinematics_data.py --check` 退出码为 0。
- 三个合法名义配置的 33 个 link 变换检查通过，URDF 与项目 FK 的最大矩阵元素绝对差为 `8.881784197001252e-16`。
- 现有纯单位换算测试通过：`13 passed`。
- J9 与 tool0 的名义几何及位置 Jacobian 检查通过，名义轴线距离为 `0.235 m`。

完整环境、命令与结果见[项目接口研究](project-interfaces.md)。名义模型的一致性和既有接口测试支持离线设计依据；物理零位与实际精度仍需要真实测量。

## 文档检查

在仓库目录运行 `PYTHONDONTWRITEBYTECODE=1 python3 .scratch/laser-tracker-calibration/verify_preparation.py`，退出码 0：17 个 Markdown 文件、105 个本地链接、任务 JSON 元数据、文档目录入口、七项决定的依赖关系与两项研究完成记录均通过检查。依赖图没有循环。

`git diff --check -- docs/README.md` 退出码 0。新增文件通过 UTF-8 读取与内容检查；原有工作区改动保留，本轮增加独立任务和规划目录，并在文档总目录增加一条入口。

## 后续状态

已完成两项研究。精度与使用范围、设备资料、安装坐标、独立基准、采样和数值验收要求仍由[决定与依赖](../../../../docs/planning/laser-tracker-calibration/decisions.md)管理。

本轮使用本地规划资料，GitHub 正式任务保持其现有状态。研究知识保存在任务目录，后续得到用户确认与实际验证后，由对应项目任务决定长期记录位置。
