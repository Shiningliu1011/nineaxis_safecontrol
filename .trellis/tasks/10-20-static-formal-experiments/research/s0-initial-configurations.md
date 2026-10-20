# S0 补充初始构型验证

2026-10-20 在恢复主机完成 initial 2、initial 3 的完整 D/S0 sim。
生产源码为 `7b643c4`；运行时集成分支的后续提交仅更新文档。
配置来自 `output/static-scene-validation-20261020/development-scenes-06/s0.yaml`
及同目录 `initial_configurations.json`，只替换对应的 `initial_position`。
名义进给、100 ms 周期及其余控制参数保持共同配置。

使用 `.scratch/host-recovery/env.sh`，实际控制器和 plant 运行于 CPU 8–15、
ROS domain 200。同期其他 CPU 运行开发验证；本次耗时作为观测结果，
共同周期仍需在最终固定源码和测量环境中完成全部 20 组检查。

## 结果

| 初始构型 | 完整跟随与终点 | 生命周期 | 完整周期数量 | P99.9 / 最大耗时 | 超期 / 准入拒绝 |
| --- | --- | --- | ---: | ---: | ---: |
| initial 2 | pass | pass | 1,722 | 55.925719 / 71.592613 ms | 0 / 0 |
| initial 3 | pass | pass | 1,721 | 52.122384 / 56.499694 ms | 0 / 0 |

两份记录可用于各自初态的 S0 完整路径证明，以及 S1、S2、S4 的
未经避障调整轨迹检查。其他场景的几何检查结果需要分别保存。

## 文件与身份

主机项目目录：`/home/lsn/robot/robot_safecontrol`。
原始数据根目录：`output/static-scene-validation-20261020/`。
运行入口：`.scratch/static-implementation/run_additional_s0.py`，
计划文件记录入口、基础配置及初态文件的 SHA-256。

`s0-additional-baselines-plan.json`：
`07c56086f95837c7f01c52d2a50dc935df26f0a142d68d3459303b96e5825c76`。

| 文件（相对于数据根目录） | SHA-256 |
| --- | --- |
| `s0-initial-2-01/lifecycle-validation.json` | `5c59aa3a043df881f42d5d0270c02f972c707a643b1cfd44330c8b828526e6e5` |
| `s0-initial-2-01/run/manifest.json` | `09b9fda6dc6d301c808a5141feaa4dd2d780a56007442a090702c63163be287b` |
| `s0-initial-2-01/run/controller.sqlite3` | `726585af536056a3319b4fe48637f95b2a1c38f19dec8a8281e37b11ae2b1731` |
| `s0-initial-2-01/run/plant.sqlite3` | `744c29347216bb304d91a789649d9e1ec5baa8732cfade8638241ec029dc3d41` |
| `s0-initial-2-01/run/feedback_report.json` | `03fe8495782948574ba3a3ff52f90e973c54185756d844a6cb5bee7eb5928afc` |
| `s0-initial-3-01/lifecycle-validation.json` | `623df7c6f27bc8fa79627f5467631807e3e707c30cb8497a62aa2dfd61f79365` |
| `s0-initial-3-01/run/manifest.json` | `55492de956ef27f572159979de58bffdbd91cc981916ca41b95b1b0965c45bb3` |
| `s0-initial-3-01/run/controller.sqlite3` | `29615ba8fb451e599e1b3add9b4d4d5267e6d3f7642642b6aa6041f758af7992` |
| `s0-initial-3-01/run/plant.sqlite3` | `19e6dc1b4dbdff7f9703f838447e6c86bdc65ec4c6e86538e9c683070b98e9ee` |
| `s0-initial-3-01/run/feedback_report.json` | `c5692f42ce0acdcd41ae26abade126a2df8da7d77012afbba8d8cf851191b224` |
