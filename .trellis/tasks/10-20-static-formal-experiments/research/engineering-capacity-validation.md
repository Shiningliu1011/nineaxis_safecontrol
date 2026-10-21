# 工程容量与输出卷验证

本项覆盖 #183 的完整矩阵容量准入。正式采集继续等待容量、场景和共同周期
全部通过。原始实验数据保留在主仓库 `output/static-experiments/`；
本项新证据位于 `output/static-capacity-20261020/`。

## 已验证的 NTFS 行为

通过 `udisksctl` 正常挂载 `/dev/nvme0n1p4`，文件系统为 `ntfs3`，
UUID 为 `083CBB753CBB5BFE`。挂载参数为
`rw,nosuid,nodev,relatime,uid=1000,gid=1000,windows_names,iocharset=utf8`。
仅使用新目录
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`。

`scripts/validate_static_filesystem.py` 使用生产 `RunRecorder` 逐条提交，
覆盖 420 秒生命周期样本、S5、C/S0 和 E/S0 的 8 个数据库。验证包括
459,371 条完整字段及 IEEE float64 位模式一致、199 次跨 keyframe 随机读取、
8 次 SIGKILL 后 9,493 条已提交记录恢复，以及同目录 JSON rename。

完整报告：

- `filesystem-validation-02/validation.json`：
  `bbf65a6cba5ea22f466c3759e4a2f6e1eb1f2555d271f87e7d622e93ec10e072`。
- 主仓库摘要：
  `output/static-capacity-20261020/ntfs-validation-02-summary.json`。
- 主仓库日志：
  `output/static-capacity-20261020/ntfs-validation-02.log`。

完整数据库共 182,595,584 字节，单个数据库 WAL 最大 2,107,145,192 字节，
离线进程 RSS 最大 165,109,760 字节；验证后该卷可用 133,453,279,232 字节。
离线验证使用 CPU16–27。真实完整控制周期及最长写入需要在最终源码和参数下
另行验证，离线写入耗时只描述文件系统行为。

## 容量依据与守卫

容量分为完整运行保留记录、单次 WAL、串行派生处理、来源归档和每卷保留空间。
全局额外重试数量单独配置；每个原定运行的失败结果属于 1,080 次固定矩阵。
额外重试需要新的运行清单和重新准入。数据库格式 3 保留全部字段与 float64。

worker 在开始新反馈、接收新命令及开始新计算之前执行数量和本地 monotonic
截止检查。已经开始的任务完整收尾，终止事件与统计具有固定额度；
`budget_exhausted` 停止后续正式运行。473 秒表示新任务的截止时刻，
已经开始的任务完成时刻允许更晚。

工程预算使用逐流 keyframe/delta 编码包络、固定记录数量、SQLite 页空间和真实
最长写入证据。压缩包络属于具备明确验证范围的工程条件，不声明任意 float64
内容的无条件压缩上界。运行完成后超过保留额度会保留整个目录并停止后续批次。

WAL 的实测包络乘以 1.25 以及每卷 1 GiB 保留空间属于工程预留。
逐 writer 空间检查与停止收尾机制见
[运行期间记录空间检查](runtime-storage-capacity.md)。
本文件中的规划快照尚未绑定最终守卫源码，正式容量保持未通过。

多卷分配以文件系统 UUID 区分，分别核对可用空间和工作空间，检查每个运行只有
一个目的目录。主正式目录通过 `runs/<id>` 符号链接索引实际输出；分析仍使用
完整正式目录。原始数据已经计入文件系统已用空间，继续保留。

## 完整记录包络与准入结果

19 组已关闭运行共 1,832,469 条记录完成审计，覆盖 A–E、S0/S1/S2/S4/S5
和三档 S3。真实失败 E/S5 的 `sns_failure` 及候选字段计入存储包络；
该运行的功能结果继续标记失败。完整审计包含新增 proposal 来源字段、
不同几何对象数量、QP 与 SNS 的全部实际数组 shape。

审计文件位于主仓库 `output/static-capacity-20261020/`：

| 文件 | SHA-256 |
| --- | --- |
| `record-envelopes-02/envelopes.json` | `cdf842c8f70195f5970e4a0d591065fd862751d5441f040d5880935a1bebe42e` |
| `record-envelopes-03/envelopes.json` | `4a4fb577e9088ae879bd8595872380f405f6e2782dac85b7a4b09f0d0de26571` |
| `record-envelopes-04/envelopes.json` | `499a039469dd22e62f11c1b3d224bccb0d8597e62ea52c903f96b67aac88f7ec` |
| `ext4-filesystem-validation-01/validation.json` | `0f5ed02fa21f18395fc2dec982d47c2ff130b6984e470f85102fe9ab820156aa` |
| `lifecycle-s4-02/lifecycle-validation.json` | `1b22dc7ac9ad903c565622e54465a6c260b0016e5969204561c35d1f89e7614c` |

ext4 的当前 D/S4 完整运行包含 1,756 个唯一候选及实际暂停恢复；
独立生命周期检查通过。NTFS 的 `lifecycle-s5-02` 和
`lifecycle-timeout-02` 均使用 `7b643c4` 的运行源码，后者完整保存
209,690 条反馈、4,192 个唯一候选，两个 worker 正常退出且终止原因为
预期 timeout。其来源归档
`ntfs-lifecycle-source-02-source/sources.zip` 的 SHA-256 为
`25f3f268ba1858a554a33e5277e2f1375776a5c92a641e7b008166db87115148`。

容量代码将每个固定运行编号映射到几何组合数量和记录结构分组。
没有对应最大几何数量的 SNS 样本时，该组使用包含 QP/SNS 字段的公共
最大包络。每组保留 5% 编码裕量；SQLite 开销来自实际页空间，
记录额度按两卷均已验证的 4096 字节分配单位取整。

`engineering-plan-03/capacity_plan.json` 在生成时为 `pass`，
`formal_bound=false`，SHA-256 为
`44fe0e0775d23651ee21b304344419a3443d35f883ef4de18efad01ef3c08a67`。
对应 `engineering_evidence.json` 的 SHA-256 为
`0ac9333b9cbcef9e031bc1d799f3dc84c235c60c1363f9ad8ca391daa4ae7956`。

| 项目 | 字节 |
| --- | ---: |
| 全部 1,080 次记录 | 198626881536 |
| 两卷单次 WAL 工作空间之和 | 7769948160 |
| 主 ext4 串行派生处理 | 536870912 |
| 主 ext4 来源归档 | 134217728 |
| 两卷保留空间之和 | 2147483648 |
| 总需求 | 209215401984 |
| 生成时实际可用空间 | 209441251328 |

额外重试预算为零。样本原件已经计入文件系统已用空间，继续全部保留。
规划分配为 ext4 391 次、NTFS 689 次。准备的空目录分别为
`output/static-capacity-20261020/formal-runs-ext4/` 和 NTFS 专用目录中的
`formal-runs/`，没有启动正式运行。

随后直接调用真实 `require_volume_capacity` 复核现场空间，ext4 已发生其他
验证写入，原分配缺少 1,112,121,344 字节，准入按要求拒绝，已有数据保持。
`volume-preflight.log` 的 SHA-256 为
`e97f493a2113f60bb4312073d863cc263c45bef13793f5ea9573956a1aad40a8`。
正式启动必须按现场空间重新生成分配；生成时的规划结果不能替代现场检查。

`engineering-plan-03/sources.zip` 保存生成时的完整未提交源码，
SHA-256 为
`e78157bef54bdec781639b77f1ae8853a55291c974db2d521126bdd73718a080`。

加入全部审计数据库、manifest 及旧格式原件的 SHA-256 复核后，
`engineering-plan-04` 重新生成工程证据。总需求保持 209,215,401,984 字节，
现场可用 208,859,725,824 字节，缺少 355,676,160 字节。
所有工程证据检查通过，规划因现场容量不足返回 `fail` 和退出码 2，
`formal_bound=false`，没有开始正式运行。
`capacity_plan.json` 的 SHA-256 为
`5f0725eae7c82f467bb31b2a28acc85ff9c52cf4e5d96a648d9de035891c653e`，
`engineering_evidence.json` 的 SHA-256 为
`942d0c60c49827090c615b1ed3c54c7f2b4a0a49be9a71335a1ddf5117d46bdc`。
对应完整源码归档 `engineering-plan-04/sources.zip` 已逐文件匹配规划的
SHA-256，归档 SHA-256 为
`0c7088de78eb1778e9eb10e7c299ae05bae2a56e26d724002910a029792c5277`。

## 实现验证与剩余边界

已普通合并集成分支 `7f3ff79`，包括初始化预热及 SNS 修复。
容量、分卷恢复、正式清单和记录的
31 项测试全部通过，耗时 10.07 秒。命令：

```bash
taskset -c 26-27 python -m pytest \
  tests/test_static_capacity_volumes.py tests/test_static_capacity.py \
  tests/test_static_campaign.py tests/test_static_recording.py -q \
  --basetemp=.scratch/pytest-capacity-final
```

`capacity-final-green.log` 的 SHA-256 为
`672da62dd197b7d406be54b8eaa5a6ad3d5cb961e89bd3e6643e7d7369ec14d0`。
8 个受影响 Python 文件编译和 `git diff --check` 通过。
完整证据索引为 `engineering-delivery-summary-02.json`。

原件身份复核变更后的 17 项容量测试通过，耗时 1.28 秒；
`capacity-identity-green.log` 的 SHA-256 为
`faf83b28b62eb0be8f31d05e4b445c92bea7154ce8b88f7b8e145652786403cf`。

上述规划生成时，运行源码与归档逐项比较，变更仅位于批量准入模块，
生命周期前置依赖及 `RecordingLimits`/`RecordingBudget` 的 AST 身份相同：
`781890b853b90ecade45b52be56c68f006f71e681a1b744ddc27abbf9d7173e3`。
当前初始化预热和 SNS 修复已经改变运行源码，旧运行来源会被准入检查拒绝。
需要补充对应最终来源的验证，
然后结合冻结的正式清单使用 `--formal` 重新生成容量计划。
正式采集继续等待共同周期、方法行为、最终来源与现场容量全部通过。
