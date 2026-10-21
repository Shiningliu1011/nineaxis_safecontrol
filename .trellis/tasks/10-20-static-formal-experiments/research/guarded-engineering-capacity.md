# 当前源码的工程容量核查

2026-10-21，CPU 16–21、`ionice -c 3`，仅执行离线读取、容量生成和
任务制品盘点。运行内容与 `40d43bb` 的 123 份来源文件逐项相同；
本次基于集成 `3e0f27f`，没有修改运行模块、启动 sim 或清理数据。

## 实际容量结果

现有生成器使用十九组记录审计、当前启用容量守卫的五份生命周期报告，
以及两卷事务恢复和随机读取证据。十项工程证据检查全部通过。
固定矩阵保持 1,080 项，额外重试预算为零，逐流编码裕量保持 5%。

| 项目 | 字节 |
| --- | ---: |
| 全部运行保留数据 | 198,629,535,744 |
| 两卷 WAL 工作空间 | 7,778,336,768 |
| 派生处理空间 | 536,870,912 |
| 来源归档空间 | 134,217,728 |
| 两卷保留空间 | 2,147,483,648 |
| 总需求 | 209,226,444,800 |
| ext4 当时可用 | 75,278,811,136 |
| NTFS 当时可用 | 132,867,796,992 |
| 当前缺口 | 1,079,836,672 |

命令退出码为 2，容量状态 `fail`，
`is_validated_engineering_budget=true`，
`is_validated_capacity_upper_bound=false`，`formal_bound=false`。
共同周期和后续开发验证还会占用空间，正式准入需要重新读取现场余量。

## 验证来源

NTFS 任务目录
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/`
中的 `d-s5`、两个 `d-s4-*` 和 `d-timeout` 提供当前完整记录生命周期。
ext4 来源为主仓库
`output/static-capacity-20261020/guarded-ext4-s5-01/`：生命周期与功能通过，
P99.9 为 85.834 ms，总体计时判定仍为失败；该运行只用于文件系统及容量证据。
其生命周期报告 SHA-256 为
`e3729ab37a075b5554eb592f67e1e93f07ad91cfedb30100f422c4a28933d572`。

当前来源归档位于
`output/static-capacity-20261020/guarded-preflight-01/source/sources.zip`，
SHA-256 为
`c381f34a51223f99f575c26eaed9b6045601e8ea200efaadb1576e20ca8560a9`。

两卷文件系统证据引用已经完成的 `runtime-storage-guards-03`：
各自 19,578 条完整精确记录与 SIGKILL 恢复，本次只读补查跨 keyframe
随机记录，并核对来源、重写及恢复数据库 SHA-256。新证据明确记录原报告
身份和新增随机读取位置，全部生产源码与当时验证内容相同。
十九组审计的原始数据库身份由容量生成器集中复核一次。

结果根目录为主仓库
`output/static-capacity-20261020/guarded-capacity-review-01/`。
`execution.json` 保存完整命令、CPU、起止时间和来源身份。

| 文件 | SHA-256 |
| --- | --- |
| `engineering-plan/capacity_plan.json` | `92fab1fadf0842ad5c0be99dd4a97cad0082437ef0832ec6454831fc2cf40287` |
| `engineering-plan/engineering_evidence.json` | `18f01599d7876f269ec683639365b546ed264aa26223187fc7f004f6117ecd34` |
| `ext4-filesystem-evidence.json` | `60364abd119f929ce20e66c6ba9bb9a9a27c0123b8ec331480aa9e0e20d5959e` |
| `ntfs-filesystem-evidence.json` | `d301c9e9034d64d76dc2118fc6b20a0e75831382e2c7bd71849755823334ac49` |

## 任务制品只读盘点

`exact-duplicate-artifacts.json` 保存同卷、不同 inode、完整 SHA-256
相同的任务生成副本：重复分配共 435,240,960 字节。
每组列出参考文件及全部副本的路径、大小、inode、nlink、mtime_ns，
并区分原始运行与测试/验证生成物。主要包含 pytest 数据库和 mesh 副本，
以及两次存储验证中相同的派生数组。该清单 SHA-256 为
`043a33cbc1c18613f10c253f6b7f1825ae4c87745a01de97dc92be707d2a7974`。

另有编码测量数据库 943,423,488 字节，可由已核对原始来源与保存的
`benchmark_source.py` 重新生成；两个验证目录的 `source-replay` 派生数组
共 269,914,112 字节。它们与前述重复组有交集，不能直接相加。
详情 `reproducible-artifacts.json` 的 SHA-256 为
`f5a7b7a76af9b3bf571282bb211c04466f85b2da1861eef171d3418560aa30f6`。

所有文件、原始数据库和 SIGKILL 记录均保持原状。本次没有删除、创建文件
链接或修改权限，也没有将候选空间计入容量准入。完整矩阵仍保持停止。
