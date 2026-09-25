# 实施与验证

- [x] 在 `portable_oscbf/work/collision_safety.py` 定义固定容量配置、输入、身份、状态和 typed result；实现三个公开操作。
- [x] 在 `portable_oscbf/tests/test_collision_safety.py` 检查合法调用、JAX 编译边界、shape、dtype、非有限值、身份、模式、九个状态和无效 mask。
- [x] 在 `docs/modules/portable_oscbf.md` 增加入口，并在对应主题页面记录公开接口、结果字段、精度和当前准入状态。
- [x] 运行受影响测试、静态检查和适用的完整软件测试；记录命令、结果和仍需后续票完成的行为。

## 实际验证

- `python3 -m pytest -q portable_oscbf/tests/test_collision_safety.py`：22 项通过。
- `bash scripts/agent_check.sh`：全部检查通过，其中纯逻辑测试 69 项通过。
- `source /opt/ros/humble/setup.bash` 并保留其 `PYTHONPATH` 后，`portable_oscbf/tests/test_generated_kinematics_data.py`：3 项通过。
- `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH="src:portable_oscbf:${PYTHONPATH:-}" python3 -m pytest -q portable_oscbf/tests'`：200 项通过、34 项跳过。
- 接口测试补充无 ROS 子进程对三个操作的直接调用后，再次运行 `python3 -m pytest -q portable_oscbf/tests/test_collision_safety.py`：22 项通过。
- 查询与区间证明的真实数值计算仍由后续执行票完成；当前结果保持非 `OK` 并关闭全部准入 mask。

## 工作流状态

- Phase 3.3：本票实现的是已有规格与 ADR 确认的接口形状，当前没有需要加入 `.trellis/spec/`、`CONTEXT.md` 或 ADR 的新通用规则。
- Phase 3.4：用户已授权仅提交并推送本票改动，随后关闭对应 GitHub 票据、更新统一执行地图、归档本地任务，并在合并后只保留 `main` 分支。`docs/agents/issue-tracker.md` 的已有改动不属于本票提交范围。
- 实现提交 `1b61e2f` 已快进合并并推送至 `main`；[固定 CollisionSafety 公开接口与 typed result](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/103) 已写入验收记录并关闭；[速度级 OSCBF 与 CollisionSafety 统一执行地图（仅激光雷达感知）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133) 已更新决定索引。
