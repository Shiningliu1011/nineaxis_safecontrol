# OFF-02：独立 OBB 判据与首个连续局部证书

日期：2026-09-15。对应 [OFF-02 / #46](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/46)。本轮实现并验证了独立点态重叠接口、22 个遗漏非相邻对的连续关节盒证书，以及一个绑定当前蝴蝶首点的固定起点候选。结果不包含实机准入。

## 结论

- 10 个 OBB 的 45 个组合中，直接相邻的 9 对按用户决定免检；其余 **36 对全部进入独立点态 SAT 判据**。当前在线 CBF 表仍为其中 14 对，另外 **22 对全部进入区域证书**。
- 历史正距离盲点构型被独立判据拒绝，原因码为 `GEOMETRY_OVERLAP`；其中明确包含 `base_link/Link9` 与 `Link6/Link8`。非有限、维度错误、越限和模型身份错误返回 `indeterminate`，不会静默视为分离。
- 找到一个满足当前蝴蝶首点位置和工具 X 轴方向的 OBB 分离 IK 分支，并选定一个固定允许起点候选。两个端点和区域中心均检查 36 对，结果为 `separated`。
- 包含拟议直线关节过渡并在每个关节坐标外扩 0.0005 的轴对齐关节盒，对全部 22 个遗漏对获得连续下界证书。声明要求净空为 1 mm，最小保守下界为 **3.859286859 mm**，瓶颈为 `Link5/Link7`。
- 这张证书只证明数字 OBB 模型中的局部候选盒。完整蝴蝶跟踪关节域、生产 MoveIt 过渡、状态误差、延迟、停车范围和物理几何误差仍为未知，因此不能据此给 OFF-09 报告完整任务 `covered`，也不能开放 live。

## 实现入口

| 文件 | 作用 |
|---|---|
| `portable_oscbf/work/obb_geometry_admission.py` | 36 对点态 `separated / overlap / indeterminate`，稳定原因码、模型/配对身份，22 对相对运动连续界，以及 `covered / outside / indeterminate` 强绑定检查 |
| `portable_oscbf/scripts/certify_obb_region.py` | 从显式 JSON 区域生成确定性证据，记录模型、mesh 和任务输入的 SHA-256 |
| `portable_oscbf/tests/test_obb_geometry_admission.py` | 配对分区、已知反例、非有限输入、相对运动消去、区域证书和身份失配回归 |
| `portable_oscbf/work/fcl_collision_mesh.py` | 离线 mesh checker 恢复 Link6，并只排除运动链直接相邻对；非相邻 Link3/Link5 不再永久豁免 |
| `portable_oscbf/scripts/generate_obb_calibration.py` | 把 14 对表明确标为在线 CBF 子集，生成配置不再宣称 Link6 被 Link5 覆盖或 Link3/Link5 获得几何豁免 |
| `region-input.json` / `certificate.json` | 本次候选、未知项、输入身份、36 对点查与 22 对逐对连续证据 |

模块不拥有锁存或人工恢复。OFF-09 消费时必须同时获得点态 `separated` 和所需动作盒 `covered`，并核对 query、boundary、model、pair-policy、task、attachment 和 certificate ID。OFF-13 仍负责最终过滤命令边界。

## 连续界方法

对遗漏对 `(Link i, Link j)`，先在区域中心用 15 轴 OBB SAT 求一个投影分离下界 `L`。共同上游关节的刚体运动在相对坐标中相消，只累计 J(i+1)…Jj：

- J1 平移使用区间半宽；
- 转动关节使用下游最大力臂和精确弦长上界 `2 r sin(min(delta, pi) / 2)`；
- 从 `L` 中扣除逐轴相对运动和、显式几何误差与数值带；只有结果严格大于声明净空才标记该对 `certified`。

这是一条充分条件。下界不足返回 `not_certified`，不能解释为已经观察到碰撞；中心相交则直接返回 `GEOMETRY_OVERLAP`。区域超出关节限位或输入无效返回 `indeterminate`。

## 固定起点与任务首点候选

当前校准蝴蝶首点和工具轴来自实际工作区文件：

```text
target_position_m = [-0.000045357825003508, 0.3948815348800556, 1.3666583431058257]
target_tool_x_world = [-0.00010018781191595272, 0.0, 0.9999999949812012]

q_start = [0.2, -1.0, 1.2, 0.0, -0.7854, 0.35, 0.0, -0.47, 0.1]
q_task_start = [0.145220506589, -1.125971556525, 1.298605201346,
                0.0, -0.7854, 0.34987160919, 0.015356115966,
                -0.471678659574, 0.10801598214]
```

`q_task_start` 由固定 J4=0、J5=-0.7854 后的确定性 `scipy.optimize.least_squares` 多起点筛选得到。位置误差约 `1.59e-16 m`，工具轴误差约 `3.66e-08 rad`。筛选先发现 12 个自然/随机精确 IK 分支均有 Link3/Link5 OBB 重叠，随后按已接受的失败分流规则调整冗余姿态；没有放宽碰撞规则。

证书区域中心与任务半宽分别为：

```text
center_q = [0.1726102532945, -1.0629857782625, 1.249302600673,
            0.0, -0.7854, 0.349935804595, 0.007678057983,
            -0.470839329787, 0.10400799107]
task_half_width_q = [0.0278897467055, 0.0634857782625, 0.049802600673,
                     0.0005, 0.0005, 0.000564195405, 0.008178057983,
                     0.001339329787, 0.00450799107]
```

这个盒包含两端点之间的整条直线关节插值，并在每一维再外扩 0.0005。它比只检查插值采样点更强，但尚未约束生产 MoveIt 必须走该盒。

## 证据身份与结果

```text
admission_status = separated_and_certified
model_id = obb-model:v1:9e965e050aad0988a92974d62d952a55e74bdd2d05e5ec8e4f96272227ce20c2
pair_policy_id = obb-pairs:v1:b0f9d288f60fc79239ce40f23d20368c1b2975104dc424a7fb2cfedd5d235f3c
certificate_id = obb-region:v1:e7a2c4e62fa0f9e262d8a86e4ad1388231dd62afaaa96a1028ac5b2028caa763
certificate_pair_count = 22
input_sha256 = d61b397cee3e6799760ba1679638c9a905bdd37e71c11720e29ee2f953211f39
certificate_sha256 = 345d9d9384eb504471e04d54d2f78c8c8826aa5691723a07fdb66471b1b46e6c
```

`certificate.json` 另含 10 个 STL、OBB 常量、运动链、证书实现、蝴蝶数据和控制/轨迹源码共 18 个实际文件哈希，以及 22 对逐对中心下界、活动关节、运动界和证书下界。`geometry_error_m`、`state_error_half_width_q` 和 `stop_half_width_q` 在本次模型证书中为零；对应来源字段和 `unknown_items` 明确声明这些零值不代表物理误差或停车能力。

## 实际验证

```bash
python3 -m pytest -q \
  portable_oscbf/tests/test_obb_geometry_admission.py \
  portable_oscbf/tests/test_obb_model.py
# 19 passed in 4.91s
```

覆盖了已知盲点拒绝、36=14+22 配对完备性、相邻免检、Link6 mesh 纳入、非有限输入、模型/证书身份失配、越域、共同上游运动相消、连续证书和生成器确定性。

在同一主机、区域中心、预热 20 次后顺序执行 1000 次 36 对点查：均值 `5.916 ms`，p50 `5.909 ms`，p95 `5.951 ms`，p99 `6.125 ms`，最大 `6.332 ms`。这是隔离 Python 基准，只说明该实现单独运行低于 10 ms；它没有证明与完整 100 Hz 控制回路合并后的时延预算。

证据可复现为：

```bash
python3 portable_oscbf/scripts/certify_obb_region.py \
  --input docs/planning/oscbf-reuse/research/off02-local-certificate-20260915/region-input.json \
  --output docs/planning/oscbf-reuse/research/off02-local-certificate-20260915/certificate.json
```

## 未覆盖项与后续路由

1. **完整蝴蝶任务**：尚无绑定当前控制器实际闭环状态的连续九轴轨迹盒，不能报告完整跟踪 `covered`。下一步应保存实际跟踪 q/dq，先逐段构造区域；界不足时按对细分，而不是只增加采样密度。
2. **生产过渡**：当前只证明拟议直线关节过渡盒；MoveIt 规划结果若离开该盒，域检查必须返回 `outside`。可约束规划器采用该走廊，或对实际规划轨迹重新分段认证。
3. **14 个在线对的连续覆盖**：本证书只负责 22 个遗漏对。运行时必须由 14 对在线约束与点态 36 对判据补齐；过渡若没有相同在线约束，需另加连续验证。
4. **实机包络**：状态误差、命令延迟、反应/停车和物理几何误差没有依据，保持未知。OFF-02 结果不能替代 OFF-09 锁存、OFF-13 命令边界或现场阶段验收。

## 回退边界

本轮没有把新判据接入运行控制器，也没有改变 sim/shadow/live 模式。若接口方案需要回退，可撤销新增 admission 模块、证书脚本、测试和本证据目录，并还原 OBB 生成器与离线 mesh checker 的本轮改动；回退后必须恢复“Link6 和 22 个遗漏非相邻对未获连续依据”的明确状态，不能恢复“Link5 已覆盖 Link6”或“Link3/Link5 永久豁免”的失实结论。
