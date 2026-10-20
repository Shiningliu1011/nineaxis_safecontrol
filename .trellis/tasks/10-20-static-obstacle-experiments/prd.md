# #171 静态障碍物仿真实验

## 需求来源与授权

用户明确要求使用 `implement-spec` 执行 #171，随后授权建立 Trellis 任务。
完整规格见 `docs/specs/static-obstacle-experiments.md`。
GitHub #171–#183 的最新正文、评论与原生依赖已于本次接续时重新读取；
#172–#183 全部开放，尚无验收评论。原生父任务为 #147，各任务引用 #171。
`implement-spec` 负责整个规格的执行、独立 worktree 子任务、集成与最终审查；
Trellis 保存任务状态、规划、验证证据与接手资料。
集成分支为 `codex/static-obstacle-experiments`，本次起点为 `35a1a61`。
已获取 origin 更新并核查本地与远程相同，初始工作区没有未提交改动。
用户明确授权本地修改、安装、构建、sim 验证、子 agent、commit 和 push。

## 共同要求

- 始终使用 sim；九轴、三维位置与二维工具轴方向，roll 自由。
- 位置与工具轴共同使用路径进度；完整蝴蝶为正式路径。
- 路径半径 270.6 mm，工件半径 220.6 mm，TCP 作业距离 50 mm。
- 外部间距 30 mm，自碰撞间距 10 mm；检查全部 36 个非相邻组合。
- A/B/C/D 两因素对照与 E Generalized-SNS 共用场景、限制、构型辅助和评价。
- Ruckig 后检查运动，拒绝候选不能推进参考；暂停期间允许合法构型调整。
- 持续恢复确认至少 0.2 s，持续暂停超过 30 s 终止，终点连续保持至少 0.5 s。
- 根据全部方法完整周期 P99.9 选择 20、40、50、100 ms 中满足 80% 要求的最短周期。
- 正式场景采用三组合法初态、三档进给、三次重复；失败和证据缺失完整保留。

## 子任务与顺序

| Issue | 交付 | 前置 | 本地任务 |
|---|---|---|---|
| #172 | S0、反馈记录与评价 | 无 | `10-20-static-s0-feedback` |
| #173 | 同一参考的五维任务 | #172 | `10-20-static-shared-reference` |
| #174 | S1 与静态外部几何 | #172 | `10-20-static-external-collision` |
| #175 | 全部 mesh 自碰撞 | #174 | `10-20-static-self-collision` |
| #176 | Ruckig 检查与进度结算 | #173、#175 | `10-20-static-execution-admission` |
| #177 | S2–S5 与可行性见证 | #176 | `10-20-static-scene-witnesses` |
| #178 | A 组与基础恢复 | #177 | `10-20-static-method-matrix` |
| #179 | B 组联合求解 | #178 | `10-20-static-method-matrix` |
| #180 | C 组改进恢复 | #178 | `10-20-static-method-matrix` |
| #181 | E 组 Generalized-SNS | #180 | `10-20-static-method-matrix` |
| #182 | 五组组合与周期测量 | #179、#181 | `10-20-static-method-matrix` |
| #183 | 正式实验与复现 | #182 | `10-20-static-formal-experiments` |

## 共同验收

- [ ] 各 issue 的全部验收条件均有对应实现与实际运行证据。
- [ ] D 组在已有可行性证据的 S0–S4 达到精度、间距、完整覆盖、终点及恢复要求。
- [ ] S5 保留安全终止、限制对象、未完成范围及原因。
- [ ] 原始数据可独立重算报告，配置、代码、模型、mesh 与路径身份可追溯。
- [ ] 正式配对统计和图表保留所有失败及缺失证据。
- [ ] 完成相关回归、规范检查与需求检查。
