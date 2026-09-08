# 09 — 配置一致性清理（alpha 增益漂移 / 遗留参数 / 死 launch 参数）

**What to build:** 已确认三处配置漂移,统一为「YAML 唯一来源」:

1. `portable_oscbf/config/nineaxis.yaml` 的 `alpha_joint_limit: 5.0` 与代码硬编码
   8.0 不一致(见 `portable_oscbf/work/oscbf_velocity_config.py` 与
   `jax_kernel_factory.py` 的默认值),控制行为随代码版本漂移;
2. `portable_oscbf/config/controller_params.yaml`(`dt: 0.005 / mode: torque`)
   与生产 `src/robot_safecontrol_moveit/config/oscbf_controller.yaml`
   (`dt: 0.01`) 并存,疑似遗留,需确认谁被消费;
3. `launch/mujoco_transition_final.launch.py` 中 `hardware_bridge` 无条件启动
   且不读任何参数(`hardware_mode` / `drempower.yaml` 为死配置),要按真实
   意图:要么注入参数,要么从 launch 移除真机节点。

**Blocked by:** None — 机械清点为准。

**Queue:** implementation-backlog（已移出 Wayfinder）
**Tracker:** #12 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/12)

**Status:** ready-for-agent

- [ ] 列出所有 alpha/增益参数的实际读取路径(优先 YAML,代码仅作默认)
- [ ] 删除或标记遗留 `controller_params.yaml`(确认无消费方)
- [ ] hardware_bridge: 参数注入 or 默认 disabled,二选一并落实
- [ ] 加一条「配置一致性」测试:YAML 与代码默认值断言一致
