# #177 验收评论草稿

场景证据已按九项条款核对，全部验证限定sim。当前正式物理输入登记为
`.trellis/tasks/10-20-static-scene-witnesses/research/formal-scene-input.json`。
登记文件SHA-256为`0f27070df3e6bd707426db43b85ff61d442e93f89aa55af3fdf12a319e7f3c9e`；
本轮重新核对7份登记来源及72个配置原文件/有效配置hash，全部一致。

1. **S2同时约束**：dev07采用原50%候选7、10。当前完整D/B运行分别有
   42个同时不超过31 mm的候选、32条获准双活动约束，涉及
   Link2/obstacle_1与Link5/obstacle_2；独立FCL距离一致。
   A完整通过并恢复33次，其轨迹没有观察到共同活动约束，按测量保存。
2. **S3行程**：三初态下J1/J4两个方向均按
   `q + scale * (limit - q)`计算100%、70%、40%边界；
   当前72配置快照保存全部实际execution_limits。
3. **S4独立见证**：`s4-paired-independent-witness-01/witness`
   保存保持TCP位置和工具轴的连续构型调整及所有区间安全检查，终态可正向进给。
   D三初态×三进给的九份完整ROS功能均通过，实际恢复次数按初态为
   `[6,3,1]`、`[6,1,1]`、`[6,1,1]`。
4. **S5阻断**：dev07的`s5_blockage.json`保存必经路径几何阻断证明。
   A–E开发记录均验证持续阻塞终止；S5以安全终止和原因为验收用途。
5. **候选与选位**：S1/S2/S4的25%、50%、75%候选、三初态反事实连杆轨迹、
   合法初态和工具通道检查已保存。S2的最终选位证据为
   `development-scenes-07/s2-selection.json`，其余场景沿用同清单明确关联的来源。
6. **逐组合状态**：三初态和三档进给的72项均有状态；
   当前33项已有完整ROS物理路径来源。新S2初态2、3补充运行继续执行，
   通过后可分别增加三档进给的物理证明。其他未证明项保持insufficient_evidence；
   各方法/进给的功能分别验证。S5使用专门阻断验收。
7. **开发与正式输入**：dev07/catalog.json和configuration-feasibility-02的
   当前物理快照已登记为正式场景输入，坐标、初态、行程及内容hash固定。
   #182随后冻结共同周期和完整运行参数；正式采集保持停止。
8. **加载与复查**：共同配置入口、MuJoCo场景对象加载测试和各场景完整ROS记录
   使用同一场景定义；S4见证可由独立验证入口重放，S5有独立几何检查。
   模型对象及初态核查采用真实MuJoCo数据，本轮没有进行人工图像判读。
9. **说明**：使用文档及scene研究资料明确完整跟随、恢复、阻断终止和证据不足的范围。

当前guarded来源为`40d43bb3712e3f99c11a59750a59d1ee65ecb525`，
原件根为`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-preflight-01/`。
`validation-summary.json` SHA-256为
`1753d5d603d8997a637cbeb25ec757720bfb8fbe2e24bb23b104082d1842f836`。
dev07/catalog SHA为`d4f03cb3d47cd76d8d11a44dfe54557e9b112af05d18a3a55d1772d10ac2ced0`；
72配置快照SHA为`18174f4272abe57fc9ca7889503eeaf07a224ef194085cdb17f795cdccec88c6`；
当前可行性矩阵SHA为`bec38ea319f28b87d4a2218c9e8ee5aadb60a7cc393513250ee429bd990a948a`。

详细来源、每份原件校验值及验证步骤见
`.trellis/tasks/10-20-static-scene-witnesses/research/guarded-scene-acceptance.md`
和`scene-acceptance-evidence.md`。正式完整矩阵与统计属于#182–#183。
