# #181 验收证据

E组通过公共入口选择Generalized-SNS与improved恢复，八项条款对应证据如下：

1. 已核查作者论文、README、LICENSE、GSNS_VL.m、FindScaleFactor.m、
   damped_pinv.m和computeVelocityBounds.m全文；固定上游版本
   `0d4df38bc735e137a69796dc3ca3396c9de3a616`、CC BY 4.0，
   作者署名、阻尼SVD参数与Python移植说明保存于依赖及使用文档。
2. 可直接对照的解析实例缩放为0.5和1.1/3，与独立HiGHS最大缩放一致；
   真实输入、输出、广义非零恢复限制及代表DLS数值记录可重放。
3. 保持作者任务整体缩放与逐项饱和语义；联合进给QP为独立方法。
   当前完整运行的最小task_scale为1，按实际结果报告，数值实例覆盖缩放小于1。
4. 同一九轴五维、关节/碰撞限制、构型辅助、Ruckig和改进恢复接口接入，
   实际命令均经过共同准入；原始限制未放宽。
5. E/S0基础跟随、E/S1外部受限、E/S4暂停恢复26次、E/S5持续阻塞终止均有
   完整ROS证据。E/S4完整路径通过，E/S5以pause_timeout通过安全终止验收。
6. 全部反馈、误差、间距、进给、周期与失败进入共同格式3；
   历史E/S4反馈时效故障和E/S5算法故障保留原件及来源，后续通过记录独立保存。
7. QP指标使用not_applicable；实际SNS的完整任务残差、广义限制、饱和及缩放
   逐条重算，代表输入使用独立HiGHS与作者DLS/null-space方程核查。
8. 环境入口、固定依赖、作者许可、运行入口与来源说明随仓库保存。

E/S1来源为`e6fe012ea3eb77ad1adaae5865a4d0d365e522af`，
`output/static-method-evidence-20261020/e-s1-current-01`完整通过；
1710个实际trial最大限制超差`4.376e-17`、完整任务残差`6.266e-16`，
六个代表HiGHS/DLS核查通过。独立证据SHA为
`bcdc62b002d5e8a537db456d6ec8e453ccd316011ff98405ea169bcec88084f5`。

当前E/S4、E/S5来源为`40d43bb3712e3f99c11a59750a59d1ee65ecb525`，
原件位于NTFS任务根`guarded-preflight-01/{e-s4,e-s5}/run`。
分别核查2179/2826个实际trial，186/2745个负上界，940/2787个饱和实例；
最大限制超差为`4.447e-15`/`6.107e-16`，代表HiGHS与DLS全部通过。
两份独立核查SHA分别为
`455ec8b3e82e16334773370ad0adda8a14f5a05fd46d70680f9ae79cb324f4f5`、
`a7d4c1e09ce4e5857364abac94d867f37f6638f5b35ae8a6c5077bde73e84271`。

详细原件、上游文件hash与验证命令见方法`research/method-acceptance.md`、
`research/current-sns-s1-s4.md`、`research/sns-s5-recovery.md`及正式任务
`research/guarded-preflight.md`。当前证据支持外部方法接入与开发验收；
正式比较保持全部方法失败记录，最终结果由#182–#183提供。
