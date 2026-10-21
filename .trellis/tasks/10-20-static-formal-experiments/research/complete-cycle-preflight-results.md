# 完整周期计时三卷前置结果

运行源码固定为 `504a274740a1e0c27c597c8d61a889fe39154aed`，125项来源一致。
环境为 `.scratch/host-recovery/env.sh`；sim worker CPU0–15/domain200，
monitor CPU22–27，离线文件系统检查CPU16–21。服务
`static-complete-cycle-preflight-01` 已退出0；七项sim流程均符合各自预期。

## 实际结果

- 三卷容量守卫、19,591条真实记录复制、精确值、随机读取、SIGKILL恢复与
  原子JSON验证全部通过，UUID与当前recording源码身份一致。
- 三卷D/S5均按持续暂停规定终止，功能和生命周期通过。
  ext4完整周期P99.9/最大为85.487874/96.130747 ms，超期零次，runtime失败；
  原NTFS为78.416738/81.368136 ms，超期零次，runtime通过；
  Windows为104.603116/122.846921 ms，超期一次，runtime失败。
  两份综合失败报告保留原件。
- 原NTFS D/S0完整通过，1721周期，P99.9/最大为63.264791/65.327040 ms，超期零次。
- 原NTFS D/S4第三初态、0.5倍进给完整通过，3454周期，
  P99.9/最大为68.937104/86.711524 ms，超期零次，独立连续见证通过。
- 420秒timeout记录、完整计时、来源绑定及收尾检查通过；任务未完成状态保留。
- 每writer 8 MiB forced-WAL按budget_exhausted终止，保留功能/validation失败状态，
  两worker退出0，terminal event、实际原因、统计SHA与收尾空间检查成立。

全部周期使用 `controller_after_synchronous_records_and_recovery_v1`，
SQLite sequence与完成表逐项对应，feedback timing及真实admission拒绝计数一致。
正式共同周期仍由完整20项开发测量选择；结论限定于实际配置、来源、设备和输出卷。

## 工程容量

公共容量入口已在上述原件上实际执行并通过，全部10项检查为true。
1080项保留额度200080502784字节，瞬时空间11746148352字节，
派生536870912、来源134217728、三卷保留3221225472字节。
总需求215718965248，可用283659026432，余量67940061184字节。
`formal_bound=false`，正式清单及最终周期固定后仍需现场重新绑定准入。

## 数据与身份

主目录为 `output/static-capacity-20261020/complete-cycle-preflight-01/`。
原NTFS与Windows的原件在各自任务目录的同名子目录，原始来源保持不变。
日志为主目录旁 `complete-cycle-preflight-01.log`。

| 文件（相对主目录） | SHA-256 |
| --- | --- |
| completed.json | `f6f327d9925a12ca400980fe3d5304cc56a56b8e304bc161f6cd80470afd4c5c` |
| source/sources.zip | `0096fbdca9da85de6933ac09997b9c4a3764b667072b780af7496412428d72b5` |
| source/snapshot.json | `545caa8a61080e07877e51b8216b1c93a3b05effffcc30541a6bb1ff9a98e0e2` |
| archive-validation.json | `828fb0bf6418d15d295d3ec62c3d671ec9be64cee6dd2f3235a5959dc90ccbf4` |
| engineering-plan/capacity_plan.json | `4ef0125474a99d2a4bb9f456fbec111fd3791d480afe39a4f4f54e5d1d73b0c5` |
| engineering-plan/engineering_evidence.json | `4a958bc05d04d540180d15dfdacf5a2bc913993fea5aad98439606a5b67f6db3` |
| capacity-execution.json | `9aec0fb777db6b28aad9139764d829ce310a52161cc7c179d8bdee0d8866d012` |

来源归档通过现有snapshot检查及ZIP完整性检查；三个同名成员的内容SHA完全相同。
各run、预算、统计、生命周期与数据库SHA均在completed.json中完整保存。

## 共同周期接续

实际20份预算准备通过，`static-complete-cycle-period-study-02.service` 已按用户要求停止。
预算目录为 `output/static-capacity-20261020/guarded-period-budgets-02/`，
索引SHA为 `03dca59ec1d809ed884226789d443afccd42dbf1d7180b3a8c0a1d76b2a56c68`，
prepared.json SHA为 `03e996d5f79f03b1b3eb36f8f057b8b2fc33191ed759272c38878bc79b2d713f`。
20项保留数据与瞬时额度合计15548391424字节。数据位于原NTFS任务目录
`guarded-period-study-02/`；20 ms的A/B完整原始报告已保存，实际2/20且均失败，
C未启动，正式采集0/1080。开题资料、暂停状态与校验值见 `proposal-pause.md`。
