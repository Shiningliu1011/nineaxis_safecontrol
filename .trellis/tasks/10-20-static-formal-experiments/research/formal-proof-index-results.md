# 两个 S2 初态与正式证据索引的实际结果

主会话已完成 dev07 的 D/S2 初态 2、3、进给 1 的完整 sim 运行，
并成功执行 [prepare_formal_proof_index.py](prepare_formal_proof_index.py)。
新矩阵包含 36 个不同的完整 ROS 来源，72 个配置中 39 个具有完整物理路径证明。
`formal-scene-input.json` 登记的 33/72 历史快照保持原件身份。

本轮普通合并至 `1063a23`。只读取小型完成记录、summary、索引和矩阵 metadata，
重新计算这些文件及 snapshot.json 的 SHA-256；没有读取数据库、源码归档内容，
没有启动 sim 或重复求解器、FCL、完整来源校验。
完整来源校验由主会话执行的脚本完成，本轮保存其结果及原件指针。

## 两项实际运行

原始数据根目录：
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-s2-initials-01/`。
对应子目录为 `d-s2-initial2-feed1/run/` 和 `d-s2-initial3-feed1/run/`。

| 初态 | 完整报告与功能 | 周期样本 | P99.9 / ms | 最大 / ms | 超期 / 拒绝 |
| --- | --- | ---: | ---: | ---: | --- |
| 2 | pass / pass | 1741 | 69.9093998602 | 70.7837600003 | 0 / 0 |
| 3 | pass / pass | 1740 | 62.7446586919 | 69.4410270003 | 0 / 0 |

两项周期均为 100 ms，`admission_rejections=0`。这些计时属于本次 D/S2，
共同周期仍由完整的 20 组测量确定。

以下 SHA-256 来自已校验的 `completed.json`；本轮没有重复读取数据库计算 hash：

| 原件 | 初态 2 | 初态 3 |
| --- | --- | --- |
| manifest.json | `ed8d12bb3b7940eaf28b9d9f421dde1ac29ffe44fc5c622a9ec59b1cadd0dc50` | `fc2e2f1b039f1dfafb760ec16a0cbeeb8d6c651fe85b75bb01adebac881ff558` |
| result.json | `08697cd7b02adcbd7cdf915ff23ff4727c7d3e8d9e603dce2df33307a57e16a6` | `f87ed00d660076b597b1f3a825516e405ad6f272fb7a5c0e22784981fbf1c2c6` |
| feedback_report.json | `8173e98d1c408de12c29e0f068edbc5f9679feba16f7d48207c1d443d2104f5a` | `28e8a62ae00edfec48ae2f4b88058ed7549d9582e56ac297c2848e9dd9533229` |
| controller.sqlite3 | `84372ae4340cac668589ebc252e99f78568465d7581a717d33578e9e28e29f11` | `a8ef1d68ed8f2fc027ea5fa378ef7035de3cab32f46ff056bce5ed705b5e7358` |
| plant.sqlite3 | `e57c922e360a6994ffc5e47f38268be55030a9bc68d62b52719dba24c9e0e698` | `ce5a1106ec65b526c89d8316d63ec6f9b756ed06472b34bc896dc024cf6b5594` |
| ../lifecycle-validation.json | `ad373faceff8e39e9edfd7d14ca1c15503ed9dc52104997f133b53b498b89ad2` | `7a0132ec43315f121c8465ac6e42e9ef1ee94d98490381adcab8349b3590d1c5` |

## 控制记录和源码来源

控制目录为
`/home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/guarded-s2-initials-01/`。

| 文件 | SHA-256 |
| --- | --- |
| completed.json | `66784ebd757e0c7b90bfb6cee9652abb2783fbbcdda4dcb9499f2e73a3a78ccb` |
| execution.json | `fd3df910d8d186ac43212d417c1416ebb722c3d66a686bb0daf8e0de853eba73` |
| source/snapshot.json | `813002e1314ad365c968e5da519f49a155cd2cc11d0ebc91d10c5d0992bd3d01` |
| source/sources.zip | `e97ed434e33ad3e00a7daab30ea37a6ed56e92e5b380b6dfb8bba8bd27fe9a05` |

snapshot 登记 174 个归档文件；`completed.json` 保存完整 `source_sha256` 映射。
执行记录、manifest、完成记录与源码归档的一致性，以及两份容量统计、生命周期、
配置和 physical identity，均已经过准备脚本的完整检查。

## 新矩阵和正式索引

输出目录为
`/home/lsn/robot/robot_safecontrol/output/static-scene-validation-20261020/formal-feasibility-01/`。

| 文件 | SHA-256 |
| --- | --- |
| summary.json | `3081c40bf675d0aad21797d7c766dadc42c79074763a1ad91434168b9284f9ed` |
| evidence_index.json | `69e724c3cc392dc4980739c586bef85c2b6029b0822de9dcb3e09de2cdc38f16` |
| feasibility_matrix.json | `75494befed0744a911c8e5c55ad6e5f1af94c368ba731658110dadbcee42cf49` |
| verified_sources.json | `69cfa6069d2239502fd528fbb4f32f2665fd83c07602998bca10f2362d1807ca` |

本轮从矩阵 metadata 重新计数，36 个不同来源与以下分布一致：

| 场景 | proven | insufficient_evidence |
| --- | ---: | ---: |
| S0 | 9 | 0 |
| S1 | 3 | 6 |
| S2 | 9 | 0 |
| S3-100 | 3 | 6 |
| S3-70 | 3 | 6 |
| S3-40 | 3 | 6 |
| S4 | 9 | 0 |
| S5 | 0 | 9 |

S2 每个初态的一份完整物理路径来源覆盖该初态三档进给的物理身份。
各方法与进给的正式功能仍逐项测量。S1 和 S3 各档初态 2、3 的 24 项
完整路径证明尚未取得，状态保持明确。S5 继续使用独立阻断与安全终止验收。

索引保留正确的 S4 `s4-paired-independent-witness-01/witness` 目录和
S5 `development-scenes-07/s5_blockage.json` 文件，全部历史完整来源均纳入去重清单。
按既有 `paired_report` 规则，预先证明范围对应 144 次 D 正式功能验收，
另外 72 次 D 运行单列为预先证据不足；正式矩阵仍有 1080 项。

summary 的 `formal_run_admission` 为
`pending_period_capacity_and_final_review`。本次结果提供正式索引，
共同周期、最终参数、容量准入、freeze 和最终审查继续由主会话执行。
