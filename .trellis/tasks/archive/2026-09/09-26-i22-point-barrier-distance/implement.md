# 执行与验证

基准 `105be06`，工作分支 `main`。已有 LiDAR 标定任务文件保持原状。

- [x] 领取实现票，核对执行地图、规格、ADR 与源码；审阅 PRD 和技术设计。
- [x] 实现共享几何变换、point-scale 与独立毫米距离 kernel。
- [x] 接入公开 query、明确 track 与 pair identity、固定 mask 和容量拒绝。
- [x] 解析和独立高精度测试，覆盖九轴梯度、witness、失败状态及命令隔离。
- [x] 同步接口文档，完成受影响回归与质量检查。
- [x] 规范与需求审查，记录结果并处理发现。
- [x] 发布票尾证据、关闭本票并更新地图索引。

## 预设验收

测试全部离线运行；临时文件与日志使用已忽略的 `.scratch/i22/`，设置 `TMPDIR` 与 pytest `--basetemp`。高精度参考采用 mpmath 80 位、独立 KKT Newton 解法与解析退化用例。距离及 witness 绝对误差不超过 `2e-6 mm`；barrier 误差 `2e-9`；九轴梯度绝对误差 `2e-5`。测试 artifact 的距离容差 `1e-7 mm`、最多 100 次迭代；生产参数不变。不同几何尺寸的特殊误差界限须在运行前明确写入测试。

计划运行 point 查询、self DCOL、CollisionSafety、CollisionParameterArtifact 与 geometry artifact 测试，以及 `scripts/agent_check.sh`。若共享变换引起回归，修改该变换并重跑受影响测试，不更改数值阈值以通过测试。

## 交付边界

本次沿用已接受 ADR，没有需要提升到项目规范的新决定。用户已授权仅提交并推送本票改动、更新票据与地图、归档本地任务。提交范围限定为本票代码、测试、接口文档和任务记录。

## 已执行验证

- `TMPDIR=$PWD/.scratch/i22/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps python3 -m pytest portable_oscbf/tests/test_ellipsoid_point.py portable_oscbf/tests/test_ellipsoid_dcol.py portable_oscbf/tests/test_collision_safety.py portable_oscbf/tests/test_collision_parameters.py portable_oscbf/tests/test_collision_geometry_artifact.py -q --basetemp=.scratch/i22/pytest-regression`：194 passed，270.38 s。
- `TMPDIR=$PWD/.scratch/i22/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps bash scripts/agent_check.sh`：全部通过，其中纯逻辑测试 69 passed。
- `python3 -m compileall -q portable_oscbf/work portable_oscbf/tests/test_ellipsoid_point.py` 与 `git diff --check`：通过。
- 最终公共 header 与环境查询回归：`test_ellipsoid_point.py` 和 `test_collision_safety.py`，51 passed，92.58 s；采集时间在成功、失败、超时结果中均保持一致。

完整命令、数据身份与数值证据见 `research/validation.md`。

## 票据收尾

验收证据已发布至 [I22 票尾](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/109#issuecomment-5843597786)，票据已关闭。[统一执行地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133) 在 Decisions so far 中引用该证据。

实现提交 `4788ec81a662d6b6468029790243ebaa51266393` 已推送至 `main`，包含八个代码、测试和接口文档文件。提交前核对七个代码与测试文件的 SHA-256，与最终验收记录一致；`git diff --cached --check` 通过。本地与远程只有 `main`，本票直接在 `main` 完成。本任务归档使用 `--no-commit --skip-branch-validation`，其中后一个选项适用于本次没有独立 PR 分支的任务。

本地归档已完成，目录为 `.trellis/tasks/archive/2026-09/09-26-i22-point-barrier-distance/`，任务状态为 `completed`。本票会话指针已清除，原有 LiDAR 标定任务与文档保持原状。
