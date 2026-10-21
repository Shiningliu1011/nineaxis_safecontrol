# #180 验收证据

C组通过公共入口选择`separated/improved`，八项条款对应证据如下：

1. 同时检查当前参考位置误差、0–π真实工具轴夹角和下一周期获准正向候选。
   共用状态与真实控制测试、C/S4实际事件保存这些条件。
2. 暂停阈值1 mm/0.2°，恢复阈值0.3 mm/0.04°，恢复阈值严格较小；
   依据与共同构型辅助0.02记录于`parameter-selection.md`及运行manifest。
3. 连续确认至少0.2秒；真实RecoveryState验证0.19秒不恢复、条件中断重新计时。
   C/S4六次实际延迟为0.3、0.2、0.3、0.3、0.2、0.2秒。
4. 共同进给渐增0.03 m/s²；每个周期经过原执行限制和连续几何准入。
5. 同一S4的A/C配对仅recovery.mode不同，源码、路径与其他参数逐项相同；
   原始首次正向候选、恢复时刻、恢复后最大误差和9/6次启停记录完整保留。
6. 真实状态对象覆盖短暂条件、角度超限和准入拒绝；
   C/S0验证有序终点，C/S4验证重复恢复，C/S5持续阻塞后安全终止。
   纯状态测试和完整ROS反馈分别注明证明范围。
7. 恢复规则与solver_mode独立；D的S4九组完整通过，E/S4当前完整通过并恢复26次，
   共同接口与相同规则具有实际方法记录。
8. 使用文档、参数选择、A/C配对研究保存事件定义与延迟统计口径。

A/C S4来源为`7b643c4`，共同initial1/feed1、周期100 ms，
原件`output/static-scene-validation-20261020/s4-ac-pair-01/{A,C}/run`。
两组最终均pause_timeout、完整跟随失败；C实际进度1.201349559 m。
已发生的六次恢复用于本票行为验收，完整失败进入后续统计。
配对汇总SHA为`00a0c4e8b14fbddbe7a4c063c5aa43df8ec313d7af4cfdcd602e09b891dcfaff`；
C反馈报告SHA为`47b15d73706fdf03356efd3a3045803fa352ab421ebea7ef4233ba395ad20e08`。
C/S5为331周期的完整安全终止记录，A/C S5汇总SHA为
`cfa4ee54dd16503b88eda6e7dd742851fa0a33de2585841288b60777ac3aab5f`。

E/S4当前来源为`40d43bb3712e3f99c11a59750a59d1ee65ecb525`，
NTFS任务根`guarded-preflight-01/e-s4/run`，生命周期SHA为
`65e9ef851fb24a536ec4a31f675247f5c14e64a7a630315244d71b3d733ff55f`。
详细证据见正式任务`research/s4-ac-development-pair.md`、
`research/guarded-preflight.md`和方法`research/method-acceptance.md`。
正式重复统计及共同周期仍按#182–#183验收。
