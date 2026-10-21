# #178 验收评论草稿

A组通过公共入口选择`separated/basic`，开发证据覆盖十项条款：

1. RUNNING、REFERENCE_PAUSED、RECOVERING、ENDPOINT_HOLD、FAULT及转换原因
   由共同RecoveryState测试和原始transition记录核查。
2. A/S4暂停参考进度变化为0，实际关节继续调整；候选、执行反馈及参考分别保存。
3. 共同构型辅助包含障碍间距与归一化关节行程余量，作用于五维任务零空间；
   参数保存于manifest，当前速度上限0.02，S3三档与S4实际验证完整保留。
4. 基础恢复在获准正向候选出现时立即执行；实际A/S4九次恢复延迟均为0。
   实际求解与几何的失败处理由公共控制验证覆盖。
5. 共同进给渐增为0.03 m/s²，实际准入与RecoveryState数值验证保存。
6. 同一固定S4有九次暂停、构型调整及恢复，恢复条件和后续误差均进入配对记录。
7. A/S5持续阻塞超过30秒后pause_timeout，327个周期的完整记录保留实际范围及对象；
   安全终止验收通过。
8. A/S0、S1等价映射、S3三档及当前新S2均完成有序全路径和终点验收，
   按位置0.5 mm、工具轴0.5°及至少0.5秒保持的统一阈值评价。
9. 共同入口、Ruckig/几何/反馈准入和状态接口已由实际A–E控制及开发运行验证。
10. `docs/static_obstacle_experiments.md`和方法研究记录保存暂停、恢复、终点及故障说明。

同物理条件的A/C S4来源为`7b643c4`，A最终pause_timeout、完整跟随失败，
实际进度1.201319834 m；该失败与九次已发生恢复同时保留。
配对`output/static-scene-validation-20261020/s4-ac-pair-01/comparison.json`
SHA-256为`00a0c4e8b14fbddbe7a4c063c5aa43df8ec313d7af4cfdcd602e09b891dcfaff`。

新A/S2来源为`40d43bb3712e3f99c11a59750a59d1ee65ecb525`，
NTFS任务根`guarded-preflight-01/a-s2/run`完整通过，实际恢复33次；
生命周期核查SHA为`b51a704e27a13bf1b3955e17b44c20f9d29be97af50c4371d720a9a809266bba`。
九份早期完整开发运行与A/B、A/C配对索引`development-comparison-03.json`
SHA为`aa9b390dcb615cdebee9a530525c65b2c841f0bead2fb32ddb23f76033ab8fc7`。

原件位于主仓库`output/static-method-evidence-20261020/`、
`output/static-scene-validation-20261020/s4-ac-pair-01/`及
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`。
逐来源、独立验证和全部hash见方法`research/method-acceptance.md`、
正式任务`research/s4-ac-development-pair.md`和`research/guarded-preflight.md`。
当前证据用于A方法开发验收；正式重复比较与共同周期另按#182–#183验收。
